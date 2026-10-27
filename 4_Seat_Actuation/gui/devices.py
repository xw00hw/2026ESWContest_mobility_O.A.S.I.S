"""Worker threads own serial/camera devices; Tk only consumes snapshots/events."""
from __future__ import annotations

from collections import deque
import queue
import sys
import threading
import time

import cv2
import serial
from serial.tools import list_ports

from protocol import BAUD, SequenceTracker, capabilities, parse_packet, parse_state
from session import stamp


def serial_devices():
    devices = [dict(port=p.device, label=f'{p.device} · {p.description}', serial=p.serial_number,
                    stlink=p.vid == 0x0483) for p in list_ports.comports()
               if not str(p.hwid).startswith('BTHENUM')]
    return sorted(devices, key=lambda p: (not p['stlink'], p['port']))


def camera_devices():
    from cv2_enumerate_cameras import enumerate_cameras
    backend = cv2.CAP_DSHOW if sys.platform == 'win32' else cv2.CAP_V4L2
    return [dict(index=c.index, name=c.name, vid=c.vid, pid=c.pid, backend=backend,
                 label=f'{c.index} · {c.name}') for c in enumerate_cameras(backend)]


class SerialWorker:
    def __init__(self, port, events, vref=3.3, serial_factory=None):
        self.port_name, self.events, self.vref = port, events, vref
        self.serial_factory = serial_factory or serial.Serial
        self.stop_event = threading.Event()
        self.thread = None
        self.connected = self.pressure_active = False
        self.capabilities = {}
        self.last_state_at = self.last_pressure_at = 0.0
        self._pressure_started_at = 0.0
        self.last_state = None
        self.last_ui_at = time.monotonic()
        self.commands = queue.Queue(maxsize=32)
        self.command_lock = threading.Lock()
        self._latest_set = None
        self.hold_requested = threading.Event()
        self.hold_acknowledged = threading.Event()
        self.session_lock = threading.RLock()
        self.session = None
        self.data_lock = threading.Lock()
        self.pressure = {}
        self.history = {(b,c): deque(maxlen=250) for b in range(3) for c in range(4)}
        self.tracker = SequenceTracker()
        self.parse_errors = 0
        self._pending = {}
        self._buffer = bytearray()

    def emit(self, kind, **data):
        self.events.put(dict(type=kind, **data))

    def touch(self):
        self.last_ui_at = time.monotonic()

    def start(self):
        self.thread = threading.Thread(target=self._run, name='STM32 serial', daemon=True)
        self.thread.start()

    def send(self, command):
        if not self.connected:
            raise RuntimeError('STM32 연결을 먼저 완료하세요.')
        if command == 'HOLD':
            self.hold_acknowledged.clear()
            self.hold_requested.set()
            with self.command_lock:
                self._latest_set = None
            # Clear queued ARM/START/SET so nothing can restart after a stop.
            while True:
                try:
                    self.commands.get_nowait()
                except queue.Empty:
                    break
        elif command.startswith('SET,'):
            with self.command_lock:
                self._latest_set = command
        else:
            self.commands.put_nowait(command)

    def attach_session(self, session):
        with self.session_lock:
            self.session = session

    def snapshot(self):
        with self.data_lock:
            return dict(self.pressure), {key:list(value) for key,value in self.history.items()}

    def _write(self, uart, text):
        payload = (text + '\n').encode('ascii')
        if uart.write(payload) != len(payload):
            raise RuntimeError('시리얼 명령 일부만 전송됐습니다.')
        timing = stamp()
        with self.session_lock:
            if self.session:
                self.session.row('commands', dict(direction='TX', text=text), timing)
        if text != 'PING':
            self.emit('tx', text=text, timing=timing)
        if text.startswith('SET,'):
            self._pending[text.split(',')[1]] = time.monotonic()

    def _line(self, line, timing):
        if not line:
            return
        if line.startswith('#HELLO,OASIS_CONTROL,'):
            self.capabilities = capabilities(line)
        elif line.startswith('#STATE,'):
            state = parse_state(line)
            self.last_state_at, self.last_state = time.monotonic(), state
            with self.session_lock:
                if self.session:
                    self.session.row('motion', state, timing)
            self.emit('state', state=state, timing=timing)
        elif line.startswith('#CONTROL,12,'):
            active = ',EN=0' in line
            if active and not self.pressure_active:
                self.tracker = SequenceTracker()
                self.last_pressure_at = 0
                self._pressure_started_at = time.monotonic()
                with self.data_lock:
                    self.pressure.clear()
                    for history in self.history.values():
                        history.clear()
            self.pressure_active = active
            self.emit('pressure_status', active=active)
        elif line.startswith(('#ACK,', '#ERR,')):
            fields = line.split(',')
            if fields[:2] == ['#ACK', 'SET']:
                self._pending.pop(fields[2], None)
            elif fields[0] == '#ERR':
                self._pending.pop(fields[1], None)
            if fields[:2] == ['#ACK','HOLD']:
                self._pending.clear()
                self.hold_acknowledged.set()
            with self.session_lock:
                if self.session:
                    self.session.row('commands', dict(direction='RX', text=line), timing)
            self.emit('reply', text=line, timing=timing)
        elif line.startswith('#CONTROL_ERROR'):
            self.pressure_active = False
            with self.session_lock:
                if self.session:
                    self.session.error(line)
            self.emit('device_error', message=line)
        elif line.startswith('#BOOT'):
            raise RuntimeError('연결 중 STM32 재시작 감지. 다시 연결해야 합니다.')
        elif line[0].isdigit():
            try:
                seq, tick, channel, readings = parse_packet(line)
            except ValueError:
                self.parse_errors += 1
                self.emit('parse_error', count=self.parse_errors)
                with self.session_lock:
                    if self.session:
                        self.session.error('invalid_pressure_packet: ' + line)
                return
            extended = self.tracker.accept(seq, tick)
            if extended is None:
                return
            self.last_pressure_at = time.monotonic()
            for bank, (raw, status) in enumerate(readings):
                voltage = raw * self.vref / 4095 if raw is not None else None
                value = dict(raw=raw, status=status, voltage_v=voltage, received_at=self.last_pressure_at)
                with self.data_lock:
                    self.pressure[bank,channel] = value
                    if (bank,channel) in self.history:
                        self.history[bank,channel].append((timing[0], voltage))
                with self.session_lock:
                    if self.session and self.session.metadata['context'].get('pressure',True):
                        self.session.row('pressure', dict(seq=seq, mcu_ms=tick,
                            mcu_extended_ms=extended[0], channel=channel, bank=bank,
                            sensor_id=f'A{bank}_C{channel}', raw=raw, status=status, voltage_v=voltage), timing)
        else:
            # Keep all bytes in raw_serial.jsonl, including header/statistics lines.
            return

    def _read(self, uart, handshake=False):
        data = uart.read(max(1,min(uart.in_waiting,8192)))
        if not data:
            return
        timing = stamp()
        with self.session_lock:
            if self.session:
                self.session.raw(data, timing)
        self._buffer.extend(data)
        if len(self._buffer) > 65536:
            raise RuntimeError('시리얼 행 길이 제한 초과')
        while b'\n' in self._buffer:
            raw, _, rest = self._buffer.partition(b'\n')
            self._buffer = bytearray(rest)
            line = raw.decode('ascii', errors='replace').strip()
            if handshake and line.startswith(('#BOOT', '#HELLO,FSR_12_SCAN')):
                continue
            self._line(line, timing)

    def _run(self):
        uart = None
        try:
            uart = self.serial_factory(self.port_name, BAUD, timeout=0.02, write_timeout=0.3)
            uart.reset_input_buffer()
            self._write(uart,'CONTROL_HELLO')
            deadline = time.monotonic() + 3
            while not self.capabilities and time.monotonic() < deadline and not self.stop_event.is_set():
                self._read(uart, handshake=True)
            if not self.capabilities:
                raise RuntimeError('C4_R8/DS 펌웨어 v6 응답 없음. bolster_h723 업로드와 포트를 확인하세요.')
            self._write(uart,'HOLD')
            self._write(uart,'DISABLE')
            self._write(uart,'STATE')
            self.connected = True
            self.last_state_at = time.monotonic()
            self.emit('connected', port=self.port_name, capabilities=self.capabilities)
            next_ping = 0
            while not self.stop_event.is_set():
                now = time.monotonic()
                if now-self.last_ui_at > 1.0:
                    raise RuntimeError('GUI 갱신이 멈춰 구동을 정지했습니다.')
                if self.hold_requested.is_set():
                    self.hold_requested.clear()
                    self._write(uart,'HOLD')
                else:
                    try:
                        self._write(uart,self.commands.get_nowait())
                    except queue.Empty:
                        with self.command_lock:
                            command, self._latest_set = self._latest_set, None
                        if command:
                            self._write(uart,command)
                if now >= next_ping:
                    self._write(uart,'PING')
                    next_ping = now+0.25
                self._read(uart)
                if time.monotonic()-self.last_state_at > 2:
                    raise RuntimeError('STM32 상태 응답이 2초 이상 없습니다.')
                if self._pending and time.monotonic()-min(self._pending.values()) > 1:
                    raise RuntimeError('목표 각도 명령의 ACK가 없습니다.')
                if self.pressure_active and time.monotonic()-(self.last_pressure_at or self._pressure_started_at) > 1:
                    raise RuntimeError('압력 데이터 수신이 중단됐습니다.')
        except Exception as error:
            with self.session_lock:
                if self.session:
                    self.session.error('serial: ' + str(error))
            self.emit('serial_error', message=str(error))
        finally:
            self.connected = self.pressure_active = False
            if uart:
                try:
                    if self.capabilities:
                        self._write(uart,'HOLD')
                        self._write(uart,'DISABLE')
                        uart.flush()
                except Exception:
                    pass
                uart.close()
            self.emit('disconnected')

    def close(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=3.5)
        if self.thread and self.thread.is_alive():
            raise RuntimeError('시리얼 종료 지연: STM32 통신 timeout으로 정지합니다.')


class CameraWorker:
    def __init__(self, device, events, width=1280, height=800, fps=30):
        self.device, self.events = device, events
        self.width, self.height, self.fps = width,height,fps
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.frame = None
        self.last_frame_at = 0
        self.sink = None
        self.ready = False
        self.thread = None

    def start(self):
        self.thread = threading.Thread(target=self._run, name='Arducam capture', daemon=True)
        self.thread.start()

    def latest(self):
        with self.lock:
            return None if self.frame is None else self.frame.copy()

    def start_recording(self, sink):
        with self.lock:
            if not self.ready:
                raise RuntimeError('카메라가 준비되지 않았습니다.')
            self.sink = sink
            self.sink.submit(self.frame,self.last_capture_index,self.last_timing)
            self._next_record_ns = time.perf_counter_ns()

    def stop_recording(self):
        with self.lock:
            self.sink = None

    def _run(self):
        cap = None
        try:
            cap = cv2.VideoCapture(self.device['index'], self.device['backend'])
            if not cap.isOpened():
                raise RuntimeError('선택한 카메라를 열지 못했습니다.')
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            cap.set(cv2.CAP_PROP_FPS,self.fps)
            index, failures = 0,0
            while not self.stop_event.is_set():
                ok, frame = cap.read()
                timing = stamp()
                if not ok or frame is None:
                    failures += 1
                    if failures >= 5:
                        raise RuntimeError('카메라 프레임 수신 중단')
                    continue
                failures = 0
                if frame.ndim == 2:
                    frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                with self.lock:
                    self.frame = frame
                    self.last_capture_index, self.last_timing = index,timing
                    self.last_frame_at = time.monotonic()
                    if not self.ready:
                        self.height,self.width = frame.shape[:2]
                        self.ready = True
                        self.events.put(dict(type='camera_ready', size=(self.width,self.height),
                                             requested_fps=self.fps, reported_fps=cap.get(cv2.CAP_PROP_FPS)))
                    if self.sink and timing[0] >= self._next_record_ns:
                        self.sink.submit(frame,index,timing)
                        period = round(1e9/self.fps)
                        self._next_record_ns = (timing[0]+period if
                            timing[0]-self._next_record_ns > 2*period else self._next_record_ns+period)
                index += 1
        except Exception as error:
            with self.lock:
                if self.sink:
                    self.sink.session.error('camera: '+str(error))
            self.events.put(dict(type='camera_error',message=str(error)))
        finally:
            self.ready = False
            if cap:
                cap.release()
            self.events.put(dict(type='camera_closed'))

    def close(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=4)
        if self.thread and self.thread.is_alive():
            raise RuntimeError('카메라 드라이버 종료가 지연되고 있습니다.')
