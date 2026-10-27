"""One explicitly launched experiment: collector + motor owner, never auto-restart."""
from __future__ import annotations

from dataclasses import asdict
from copy import deepcopy
from datetime import datetime
import hashlib
import json
from pathlib import Path
import threading
import time
import uuid

from .runner import preflight, run_live
from .collector_runtime import run_capture
from .version_context import session_release, versioned_output_root
from .recording_lifecycle import PendingRecording, RecordingError


def failure_message(reason, fresh_ms):
    """Keep the machine reason in logs; explain recovery without guessing cause."""
    if reason == 'vehicle_stale_or_invalid':
        return (f'BeamNG 데이터가 {fresh_ms:g}ms 안에 갱신되지 않아 제어를 중단했습니다. '
                '게임 일시정지·메뉴·창 전환 시 전송이 멈췄을 수 있습니다. '
                '주행 화면을 확인하세요. 자동 재시작하지 않으며 임시 기록을 저장 또는 버린 뒤 다시 연결하세요.')
    if reason in ('vehicle:lateral_acceleration_outlier','vehicle:longitudinal_acceleration_outlier'):
        axis='횡가속도' if 'lateral' in reason else '종가속도'
        return (f'{axis} 입력이 데이터 유효 범위를 벗어나 제어를 중단했습니다. '
                '이 오류만으로 충돌을 확정하지 않습니다. 자동 재시작하지 않습니다. '
                '임시 기록은 저장 또는 버리기를 선택하세요.')
    return str(reason)


def choose_port(requested='auto', ports=None):
    """Enumerate without opening ports. Never guess an unrelated USB/Bluetooth port."""
    if ports is None:
        from serial.tools.list_ports import comports
        ports = list(comports())
    if requested != 'auto':
        return next((p.device for p in ports if p.device == requested), None)
    candidates = [p.device for p in ports if p.vid == 0x0483 and
                  ('STLink' in p.description or 'ST-LINK' in p.description.upper()
                   or p.pid in (0x374B, 0x374E, 0x374F, 0x3752, 0x3753, 0x3754))]
    if len(candidates) > 1:
        raise RuntimeError('STM 보드가 여러 개입니다. 실행 설정의 port를 지정하세요.')
    return candidates[0] if candidates else None


class DesktopSession:
    """Own both workers and stop both when either ends; state is safe to poll by Tk."""
    def __init__(self, profile, settings, *, capture=run_capture, control=run_live,
                 port_probe=choose_port):
        self.profile, self.settings = deepcopy(profile), settings
        self.capture, self.control, self.port_probe = capture, control, port_probe
        self.stop_event = threading.Event()
        self.thread = None
        self.output = None
        self._lock = threading.Lock()
        self._recording_lock=threading.Lock()
        self._recording=None
        self._status = dict(phase='IDLE',message='준비',collector={},motor={})
        self._events = []
        self._started = False
        self._collector_done = False
        self._collector_opened = threading.Event()
        self._error = None
        self._motor_result = None
        self._impact_event = None
        self._labels = dict(profile.get('labels',{}))
        self.release = session_release(self.profile)

    def snapshot(self):
        with self._lock:
            recording=self._recording.snapshot() if self._recording else dict(retention='none',
                recording_id=None,staging_path=None,persisted_path=None,intended_output=None)
            recording.update(deepcopy(self._status),output=str(self.output) if self.output else None,
                             error=self._error,finished=self._started and not self.is_running(),
                             impact_event=deepcopy(self._impact_event))
            return recording

    def _with_impact_history(self, message):
        """Keep an impact stop distinct from the later recording stop/save."""
        if self._impact_event:
            return message+' 충돌 의심 감지 이력이 있습니다.'
        return message

    def save_recording(self):
        with self._recording_lock:
            if self.is_running() or not self._started or self._recording is None:
                raise RecordingError('Stop all recording workers before saving')
            saved=self._recording.save_recording()
            with self._lock:
                self.output=saved
                self._status['message']=self._with_impact_history(self._recording.snapshot()['message'])
            return saved

    def discard_recording(self):
        with self._recording_lock:
            if self.is_running() or not self._started or self._recording is None:
                raise RecordingError('Stop all recording workers before discarding')
            self._recording.discard_recording()
            with self._lock:
                self.output=None
                self._status['message']=self._with_impact_history(self._recording.snapshot()['message'])

    def is_running(self):
        return self.thread is not None and self.thread.is_alive()

    def _event(self, kind, payload=None):
        with self._lock:
            self._events.append(dict(event=kind,host_monotonic_ns=time.perf_counter_ns(),
                                     wall_time_ns=time.time_ns(),payload=payload or {}))

    def request_stop(self, reason='operator_stop'):
        self._event('stop_requested',dict(reason=reason))
        self.stop_event.set()

    def _fail(self, reason):
        with self._lock:
            if self._error is None:
                self._error = str(reason)
            self._status.update(phase='STOPPING',message=failure_message(str(reason),self.settings.fresh_ms))
        self.request_stop(str(reason))

    def _collector_event(self, kind, payload):
        if kind == 'collector_status':
            with self._lock: self._status['collector'] = dict(payload)
        else:
            self._event(kind,payload)
        if kind == 'collector_session_open':
            with self._lock:
                self._status['collector_path'] = payload['path']
                config=payload.get('config',{})
                if self._labels.get('scenario') in (None,'use_collector_scenario_label'):
                    self._labels['scenario']=config.get('scenario_label','unspecified')
                self._labels['vehicle']=config.get('vehicle_label','unspecified')
            self._collector_opened.set()
        elif kind == 'collector_done':
            self._collector_done = True
            with self._lock: self._status['collector']['live'] = False
            if payload.get('error'):
                self._fail('차량 기록 오류: '+str(payload['error']))
            elif not self.stop_event.is_set():
                self._fail('차량 수집기가 종료되어 모터도 중지합니다.')

    def _motor_event(self, kind, payload):
        with self._lock:
            self._status['motor'].update(payload)
            if kind in ('IMPACT_OFF_PENDING','RECORD_ONLY'):
                if self._impact_event is None:
                    self._impact_event=dict(reason=payload.get('reason','impact_suspected'),
                        impact=deepcopy(payload.get('impact',{})),
                        host_monotonic_ns=time.perf_counter_ns(),wall_time_ns=time.time_ns(),
                        off_verified=False)
                if kind == 'RECORD_ONLY' and payload.get('off_verified'):
                    self._impact_event['off_verified']=True
            if kind != 'telemetry':
                self._status['phase'] = kind
                self._status['message'] = {
                    'WAIT_VEHICLE':'BeamNG 연결 대기 — 유효 데이터 2초 후 자동 시작',
                    'WAIT_ARM_STATE':'중립 신호 확인 중',
                    'CONTROL':'자동 제어 + 차량·압력 기록 중',
                    'IMPACT_OFF_PENDING':'충돌 의심 → 모터 OFF 확인 중 · 차량·압력 기록 계속',
                    'RECORD_ONLY':'충돌 의심 → 모터 OFF · 차량·압력 기록 계속 · 자동 재구동 없음',
                    'ended':'모터 정지 및 기록 종료',
                }.get(kind,kind)
        if kind != 'telemetry': self._event('motor_'+kind,payload)

    def start(self):
        if self._started:
            raise RuntimeError('한 실행에서 재ARM하지 않습니다. 프로그램을 다시 실행하세요.')
        preflight(self.settings)  # Physical confirmations are never manufactured.
        self._started = True
        folder = 'A_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8]
        self._recording=PendingRecording(self.profile,self.release['version'],folder,create=True)
        self.output=self._recording.staging_path
        if self._labels.get('replicate') in (None,'unique_session_id'):
            self._labels['replicate']=folder
        try:
            self._write_manifest('starting')
            self.thread = threading.Thread(target=self._run,name='Bolster-A-session',daemon=False)
            self.thread.start()
        except Exception as exc:
            self._error=str(exc)
            self._recording.mark_finished(self._error)
            raise
        return self.output

    def _write_manifest(self, status):
        with self._lock:
            manifest = dict(schema='oasis-integrated-A-session-v1',status=status,
                profile=self.profile,settings=asdict(self.settings),events=list(self._events),
                settings_role='local_preflight_draft; effective parameters come from committed STM Live Expressions',
                firmware_tuning=self._status['motor'].get('firmware_tuning'),
                effective_settings=self._status['motor'].get('effective_settings'),
                collector_path=self._status.get('collector_path'),motor_result=self._motor_result,
                impact_event=deepcopy(self._impact_event),
                error=self._error,clock='PC perf_counter_ns shared by both workers; MCU ms independent',
                policy='A_BeamNG_only',pressure_role='evaluation_only',video_enabled=False,
                code_version=self.release['version'],release=dict(self.release),
                recording_retention_at_capture='pending',
                intended_output=str(self._recording.intended_output),
                path_relocation_note='On explicit save, recording_relocation.json maps original absolute paths without rewriting raw records.',
                source_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest()
                               for p in Path(__file__).parent.glob('*.py')},
                labels=dict(self._labels))
        path=self.output/'experiment.json'
        tmp=path.with_suffix('.tmp')
        tmp.write_text(json.dumps(manifest,indent=2,ensure_ascii=False,allow_nan=False)+'\n',encoding='utf-8')
        tmp.replace(path)

    def _capture_worker(self):
        try:
            self.capture(self.profile['collector_root'],self.output/'beamng',
                         self.stop_event,self._collector_event)
        except Exception as exc:
            self._fail('차량 수집기: '+str(exc))
        finally:
            self._collector_done = True
            if not self.stop_event.is_set(): self._fail('차량 수집기 예상 밖 종료')

    def _run(self):
        collector=threading.Thread(target=self._capture_worker,name='BeamNG-archive',daemon=False)
        try:
            with self._lock:
                self._status.update(phase='WAIT_BOARD',message='STM USB 연결 대기')
            collector.start()
            # Collector must prove archive/socket readiness before opening UART.
            while not self.stop_event.is_set():
                if self._collector_opened.wait(.1): break
            while not self.stop_event.is_set():
                port=self.port_probe(self.profile.get('port','auto'))
                if port:
                    self._event('board_selected',dict(port=port))
                    if self.stop_event.is_set(): break
                    self._motor_result=self.control(port,self.output/'control',self.settings,
                        record_pressure=True,stop_event=self.stop_event,
                        status_callback=self._motor_event,startup_timeout_s=None,
                        session_metadata=dict(integrated_session=str(self.output),
                                              code_version=self.release['version'],release=dict(self.release),
                                              labels=dict(self._labels)))
                    if not self.stop_event.is_set():
                        self._fail('제어 세션 종료 — 자동 재시작하지 않음')
                    break
                self.stop_event.wait(.5)
        except Exception as exc:
            self._fail(str(exc))
        finally:
            self.stop_event.set()
            if collector.ident is not None:
                collector.join(5.)
                if collector.is_alive(): self._fail('차량 원본 파일 저장 종료 지연 — 창을 유지하세요.')
                # Never abandon a live archive writer on ordinary UI close.
                while collector.is_alive(): collector.join(.25)
            with self._lock:
                self._status.update(phase='ERROR' if self._error else 'STOPPED',
                    message=self._with_impact_history(
                        failure_message(self._error,self.settings.fresh_ms) if self._error else
                        '제어·수집 중지 완료 — 임시 기록은 저장 또는 버리기를 선택하세요.'))
            try: self._write_manifest('error' if self._error else 'stopped')
            except Exception as exc:
                with self._lock:
                    self._error='실험 메타데이터 저장 실패: '+str(exc)
                    self._status.update(phase='ERROR',message=self._error)
            try:self._recording.mark_finished(self._error)
            except Exception as exc:
                with self._lock:
                    self._error='임시 기록 종료 표시 실패: '+str(exc)
                    self._status.update(phase='ERROR',message=self._error)
