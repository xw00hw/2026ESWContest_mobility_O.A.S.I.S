"""One monotonic timeline for UART, commands, pressure and camera frames."""
from __future__ import annotations

import base64
import csv
import hashlib
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import queue
import subprocess
import threading
import time
import uuid

import cv2
from kinematics import POSE_FIELDS


def stamp():
    return time.perf_counter_ns(), time.time_ns()


class Session:
    def __init__(self, root, config, context=None):
        self.origin_ns, wall_ns = stamp()
        name = datetime.now().strftime('%Y%m%d_%H%M%S') + '_' + uuid.uuid4().hex[:8]
        self.directory = Path(root) / name
        self.directory.mkdir(parents=True, exist_ok=False)
        (self.directory / 'INCOMPLETE').write_text('recording\n', encoding='utf-8')
        self.lock = threading.RLock()
        self.closed = False
        self.errors = []
        self.counts = dict(pressure_rows=0, motion_states=0, commands=0, video_frames=0)
        self.counts.update(video_source_frames=0, video_repeated_frames=0)
        self._files, self._writers = {}, {}
        try:
            commit = subprocess.check_output(['git', 'rev-parse', 'HEAD'],
                cwd=Path(__file__).resolve().parents[2], stderr=subprocess.DEVNULL,
                creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0)).decode().strip()
        except (OSError, subprocess.SubprocessError):
            commit = None
        self.metadata = dict(schema='oasis-control-session-v2', config=config,
            created_utc=datetime.now(timezone.utc).isoformat(), origin_monotonic_ns=self.origin_ns,
            origin_wall_time_ns=wall_ns, clock='Python perf_counter_ns / Windows QPC',
            context=context or {}, source_commit=commit, status='recording',
            synchronization='Common host receipt/dequeue clock; no hardware trigger or measured USB latency',
            pressure_units='raw 12-bit ADC and estimated V, not calibrated force/pressure',
            motion_units='C4_R8 structure mdeg and commanded PWM; no measured shaft/pose feedback')
        source_root = Path(__file__).resolve().parent
        self.metadata['source_sha256'] = {p.name:hashlib.sha256(p.read_bytes()).hexdigest()
                                         for p in source_root.glob('*.py')}
        self._json('metadata.json', self.metadata)
        columns = {
            'pressure': ['host_monotonic_ns','host_wall_time_ns','session_s','seq','mcu_ms',
                         'mcu_extended_ms','channel','bank','sensor_id','raw','status','voltage_v'],
            'motion': ['host_monotonic_ns','host_wall_time_ns','session_s','mcu_ms','armed','moving',
                       *POSE_FIELDS,*('target_'+name for name in POSE_FIELDS),
                       'D3_us','D4_us','D5_us','D6_us','command_id','fault'],
            'commands': ['host_monotonic_ns','host_wall_time_ns','session_s','direction','text'],
            'frame_timestamps': ['host_monotonic_ns','host_wall_time_ns','session_s',
                                 'video_frame_index','video_time_s','capture_index','repeated'],
        }
        try:
            for key, fields in columns.items():
                file = (self.directory / (key + '.csv')).open('w', encoding='utf-8', newline='')
                self._files[key] = file
                writer = csv.DictWriter(file, fieldnames=fields)
                writer.writeheader()
                self._writers[key] = writer
            for key in ('events', 'raw_serial'):
                self._files[key] = (self.directory / (key + '.jsonl')).open('w', encoding='utf-8')
        except Exception:
            for file in self._files.values():
                file.close()
            raise
        self._last_sync = time.monotonic()
        self.event('recording_started')

    def _json(self, name, value):
        temporary = self.directory / (name + '.tmp')
        temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
        temporary.replace(self.directory / name)

    def timing(self, timing):
        mono, wall = timing
        return dict(host_monotonic_ns=mono, host_wall_time_ns=wall,
                    session_s=(mono-self.origin_ns)/1e9)

    def add_table(self, name, fields):
        with self.lock:
            file = (self.directory/(name+'.csv')).open('x',encoding='utf-8',newline='')
            self._files[name] = file
            self._writers[name] = csv.DictWriter(file,fieldnames=[
                'host_monotonic_ns','host_wall_time_ns','session_s',*fields])
            self._writers[name].writeheader()

    def row(self, name, values, timing=None):
        with self.lock:
            if self.closed:
                return
            self._writers[name].writerow({**self.timing(timing or stamp()), **values})
            counter = dict(pressure='pressure_rows', motion='motion_states', commands='commands',
                           frame_timestamps='video_frames').get(name,name+'_rows')
            self.counts[counter] = self.counts.get(counter,0)+1
            self._sync_if_due()

    def event(self, kind, **values):
        with self.lock:
            if self.closed:
                return
            self._files['events'].write(json.dumps(dict(event=kind, **self.timing(stamp()),
                                                       **values), ensure_ascii=False) + '\n')

    def raw(self, data, timing):
        with self.lock:
            if not self.closed:
                self._files['raw_serial'].write(json.dumps(dict(**self.timing(timing),
                    bytes_base64=base64.b64encode(data).decode('ascii'))) + '\n')

    def error(self, message):
        with self.lock:
            if not self.closed:
                self.errors.append(str(message))
                self.event('error', message=str(message))

    def _sync_if_due(self):
        if time.monotonic() - self._last_sync >= 2:
            for file in self._files.values():
                file.flush()
                os.fsync(file.fileno())
            self._last_sync = time.monotonic()

    def close(self):
        with self.lock:
            if self.closed:
                return
            self.event('recording_stopped', counts=self.counts, errors=self.errors)
            for file in self._files.values():
                file.flush()
                os.fsync(file.fileno())
                file.close()
            self.closed = True
            self.metadata.update(status='error' if self.errors else 'complete', counts=self.counts,
                                 errors=self.errors, stopped_utc=datetime.now(timezone.utc).isoformat())
            self._json('metadata.json', self.metadata)
            if not self.errors:
                (self.directory / 'INCOMPLETE').unlink()


class VideoSink:
    """Bounded writer queue. Overload is an explicit failed session, never silent."""
    def __init__(self, session, size, fps):
        self.session = session
        self.size, self.fps = tuple(size), fps
        self.queue = queue.Queue(maxsize=120)
        self.failed = threading.Event()
        self.ended_ns = None
        self.session.metadata['video'] = dict(codec='MJPG', container='AVI', fps=fps,
            size=list(size), origin_monotonic_ns=session.origin_ns,
            timing='CFR from session start; seed with latest preview and hold previous captures. See frame_timestamps.csv for actual source timestamps.')
        self.writer = cv2.VideoWriter(str(session.directory / 'camera.avi'),
                                      cv2.VideoWriter_fourcc(*'MJPG'), fps, self.size, True)
        if not self.writer.isOpened():
            raise RuntimeError('영상 파일을 열지 못했습니다. 저장 경로와 코덱을 확인하세요.')
        self.thread = threading.Thread(target=self._run, name='Video file writer', daemon=True)
        self.thread.start()

    def submit(self, frame, capture_index, timing):
        if self.failed.is_set():
            return
        try:
            self.queue.put_nowait((frame.copy(), capture_index, timing))
        except queue.Full:
            self.session.error('video_writer_queue_full')
            self.failed.set()

    def _run(self):
        index = 0
        first_ns = self.session.origin_ns
        previous = None
        def write(item, repeated):
            nonlocal index
            frame, capture_index, timing = item
            self.writer.write(frame)
            self.session.row('frame_timestamps', dict(video_frame_index=index,
                video_time_s=index/self.fps, capture_index=capture_index, repeated=int(repeated)), timing)
            self.session.counts['video_repeated_frames' if repeated else 'video_source_frames'] += 1
            index += 1
        try:
            while True:
                item = self.queue.get()
                if item is None:
                    break
                frame, capture_index, timing = item
                if frame.shape[1::-1] != self.size:
                    raise RuntimeError('녹화 중 카메라 해상도가 변경됐습니다.')
                if frame.ndim == 2:
                    frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                item = frame,capture_index,timing
                if previous is None:
                    write(item,False)
                    previous = item
                    continue
                desired_index = max(0,round((timing[0]-first_ns)*self.fps/1e9))
                if desired_index-index > 2*self.fps:
                    raise RuntimeError('카메라 프레임 시간에 2초 이상 공백이 있습니다.')
                while previous is not None and index<desired_index:
                    write(previous,True)
                if desired_index>=index:
                    write(item,False)
                    previous = item
            if first_ns is not None and previous is not None and self.ended_ns is not None:
                end_count = max(1,round((self.ended_ns-first_ns)*self.fps/1e9))
                if end_count-index>2*self.fps:
                    raise RuntimeError('카메라 종료 시각이 마지막 프레임에서 너무 멉니다.')
                while index<end_count:
                    write(previous,True)
        except Exception as error:
            self.session.error('video_write: ' + str(error))
            self.failed.set()
        finally:
            self.writer.release()

    def close(self, end_monotonic_ns=None):
        self.ended_ns = end_monotonic_ns or time.perf_counter_ns()
        if self.thread.is_alive():
            # Wait for space without hanging if a writer has already failed.
            deadline = time.monotonic() + 8
            while self.thread.is_alive():
                try:
                    self.queue.put(None, timeout=0.1)
                    break
                except queue.Full:
                    if time.monotonic() >= deadline:
                        self.session.error('video_stop_timeout')
                        raise RuntimeError('영상 저장 종료가 지연되고 있습니다.')
            self.thread.join(timeout=8)
        if self.thread.is_alive():
            self.session.error('video_writer_still_running')
            raise RuntimeError('영상 저장 스레드가 종료되지 않았습니다.')
        # OpenCV can fail an encode without raising. Check the finished artifact.
        check = cv2.VideoCapture(str(self.session.directory / 'camera.avi'))
        count = int(check.get(cv2.CAP_PROP_FRAME_COUNT))
        check.release()
        if count != self.session.counts['video_frames'] or count == 0:
            self.session.error(f'video_frame_count_mismatch: decoded={count}')
