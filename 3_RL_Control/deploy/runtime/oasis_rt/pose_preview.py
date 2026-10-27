"""Optional loopback-only preview of the existing pose producer's latest frame.

There is no camera/model/serial owner here and no recording endpoint. Encoding
and HTTP writes happen outside CameraWorker. At most one pending image and one
encoded result are retained; a slow encoder/client never builds a frame queue.
The preview does not qualify a pose, a neutral reference, or a motor command.
"""
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import threading
import time
from urllib.parse import urlsplit


SKELETON = ((5, 6), (5, 7), (7, 9), (6, 8), (8, 10), (5, 11), (6, 12),
            (11, 12), (11, 13), (13, 15), (12, 14), (14, 16))


@dataclass(frozen=True)
class PreviewJob:
    sequence: int
    image: object
    capture_ns: int
    landmarks: object
    status: dict


def render_jpeg(job, *, max_width=960, quality=75, min_confidence=.5):
    """Copy the same captured image; overlays are visualization only."""
    import cv2
    import numpy as np
    image = np.asarray(job.image)
    if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
        raise ValueError('preview_requires_uint8_bgr')
    h, w = image.shape[:2]
    if min(h, w) < 2:
        raise ValueError('preview_frame_too_small')
    scale = min(1., max_width / w)
    canvas = cv2.resize(image, (max(2, round(w * scale)), max(2, round(h * scale)))) if scale < 1 else image.copy()
    points = {}
    landmarks = job.landmarks or {}
    xy, scores = landmarks.get('xy', ()), landmarks.get('scores', ())
    if len(xy) == 17 and len(scores) == 17:
        for i, (p, score) in enumerate(zip(xy, scores)):
            if (len(p) == 2 and all(isinstance(x, (int, float)) and math.isfinite(x) for x in (*p, score))
                    and score >= min_confidence and 0 <= p[0] < w and 0 <= p[1] < h):
                points[i] = (round(p[0] * scale), round(p[1] * scale))
        for a, b in SKELETON:
            if a in points and b in points:
                cv2.line(canvas, points[a], points[b], (60, 220, 255), 2, cv2.LINE_AA)
        for i, point in points.items():
            cv2.circle(canvas, point, 4 if i in (5, 6, 11, 12) else 3, (80, 230, 80), -1, cv2.LINE_AA)
    roi = landmarks.get('roi_xyxy')
    if isinstance(roi, (tuple, list)) and len(roi) == 4 and all(isinstance(x, (int, float)) and math.isfinite(x) for x in roi):
        x0, y0, x1, y1 = (round(x * scale) for x in roi)
        cv2.rectangle(canvas, (x0, y0), (x1 - 1, y1 - 1), (190, 150, 70), 1)
    valid = job.status.get('reason') == 'pose_valid'
    label = 'POSE VALID' if valid else 'POSE UNAVAILABLE: ' + str(job.status.get('reason', 'unknown'))
    # OpenCV's built-in font is ASCII. The browser supplies the Korean UI.
    label = label.encode('ascii', 'replace').decode()[:110]
    cv2.rectangle(canvas, (0, 0), (canvas.shape[1], min(34, canvas.shape[0])), (20, 20, 20), -1)
    cv2.putText(canvas, label, (8, 23), cv2.FONT_HERSHEY_SIMPLEX, .48,
                (90, 230, 90) if valid else (80, 200, 255), 1, cv2.LINE_AA)
    ok, encoded = cv2.imencode('.jpg', canvas, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        raise RuntimeError('preview_jpeg_encode_failed')
    return encoded.tobytes()


def placeholder_jpeg():
    import numpy as np
    job = PreviewJob(0, np.zeros((360, 640, 3), dtype=np.uint8), 0, None,
                     {'reason': 'no fresh camera frame'})
    return render_jpeg(job)


class LatestPreview:
    """Nonblocking producer handoff; one encoder thread, replacement not FIFO."""
    def __init__(self, *, max_age_s=.3, encoder=render_jpeg, clock_ns=time.monotonic_ns):
        if not 0 < max_age_s <= 1:
            raise ValueError('invalid_preview_age')
        self.max_age_s, self.encoder, self.clock_ns = max_age_s, encoder, clock_ns
        self._condition = threading.Condition()
        self._pending = self._published = None
        self._sequence = self._encoded = self._superseded = 0
        self._invalidated_sequence = 0
        self._status, self._encoding_error, self._closed = {}, None, False
        self._thread = threading.Thread(target=self._encode, name='pose-preview-encoder', daemon=True)
        self._thread.start()

    def submit(self, frame, landmarks, status):
        with self._condition:
            if self._closed:
                return
            self._sequence += 1
            if self._pending is not None:
                self._superseded += 1
            self._status = dict(status)
            self._pending = PreviewJob(self._sequence, frame.image, frame.capture_ns, landmarks, dict(status))
            self._condition.notify()

    def unavailable(self, status):
        with self._condition:
            if self._closed:
                return
            self._status = dict(status)
            # Normal polling gaps do not mean the camera disappeared. The last
            # image still expires against its original capture time, never now.
            if status.get('reason') in ('camera_ready', 'no_new_frame'):
                return
            self._sequence += 1
            self._invalidated_sequence = self._sequence
            self._pending = self._published = None
            self._condition.notify_all()

    def _encode(self):
        while True:
            with self._condition:
                self._condition.wait_for(lambda: self._closed or self._pending is not None)
                if self._closed:
                    return
                job, self._pending = self._pending, None
            try:
                jpeg = self.encoder(job)
                if not isinstance(jpeg, bytes) or not 0 < len(jpeg) <= 4 * 1024 * 1024:
                    raise ValueError('invalid_or_oversized_preview_jpeg')
            except Exception as exc:
                with self._condition:
                    self._encoding_error = f'{type(exc).__name__}:{exc}'
                continue
            with self._condition:
                # Disconnect invalidates in-flight work. A newer pending image
                # does not starve an encoder slower than the capture cadence;
                # completed images still must meet the original capture TTL.
                age = (self.clock_ns() - job.capture_ns) / 1e9
                if not self._closed and job.sequence > self._invalidated_sequence and 0 <= age < self.max_age_s:
                    self._published = (job.sequence, job.capture_ns, jpeg)
                    self._encoded += 1
                    self._encoding_error = None
                self._condition.notify_all()

    def snapshot(self):
        with self._condition:
            published = self._published
            age_ms = None if published is None else (self.clock_ns() - published[1]) / 1e6
            fresh = not self._closed and age_ms is not None and 0 <= age_ms < self.max_age_s * 1000
            status = dict(self._status)
            status['preview'] = dict(enabled=True, image_fresh=fresh,
                frame_capture_ns=published[1] if published is not None else None,
                frame_age_ms=age_ms, max_age_ms=self.max_age_s * 1000,
                encoded_frames=self._encoded, replaced_pending_frames=self._superseded,
                pending_frames=int(self._pending is not None), encoder_error=self._encoding_error,
                closed=self._closed, recording=False, motor_access=False)
            return (published[0] if fresh else self._sequence, published[2] if fresh else None, status)

    def summary(self):
        return self.snapshot()[2]['preview']

    def close(self):
        with self._condition:
            self._closed = True
            self._pending = self._published = None
            self._condition.notify_all()
        # A native JPEG failure cannot indefinitely block shutdown or the RL process.
        self._thread.join(1.)


PAGE = r'''<!doctype html><html lang="ko"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>시트 자세 카메라</title>
<style>body{margin:0;background:#101820;color:#e9eff5;font:16px system-ui;padding:24px}
main{max-width:1040px;margin:auto}h1{font-size:24px}.view{position:relative;background:#000;min-height:240px}
img{display:block;width:100%;max-height:72vh;object-fit:contain}.cover{position:absolute;inset:0;background:#111e;display:grid;place-items:center}
.ok{color:#84e0ad}.warn{color:#ffcc77}pre{white-space:pre-wrap;color:#abc2d6;font-size:14px}</style>
<main><h1>시트 자세 카메라</h1><p id="state">카메라 상태 확인 중</p>
<div class="view"><img src="/stream.mjpg" alt="실시간 자세 영상"><div class="cover" id="cover">새 영상 대기 중</div></div>
<p>현재 영상만 표시하며 영상은 저장하지 않습니다. 중립 미확인 자세는 RL 입력으로 유효 처리하지 않습니다.</p>
<pre id="details"></pre></main><script>
const state=document.querySelector('#state'), cover=document.querySelector('#cover'), details=document.querySelector('#details');
let validUntil=0;
setInterval(()=>{if(performance.now()>=validUntil){cover.style.display='grid';cover.textContent='새 영상 대기 중';
state.textContent='자세 영상 사용 불가 · 새 상태 확인 중';state.className='warn';}},100);
async function poll(){const controller=new AbortController(), timeout=setTimeout(()=>controller.abort(),1000), started=performance.now();
try{const r=await fetch('/status.json',{cache:'no-store',signal:controller.signal});if(!r.ok)throw Error(r.status);
const s=await r.json(), p=s.preview||{}, valid=s.reason==='pose_valid';
const remaining=p.image_fresh?Math.max(0,(p.max_age_ms||300)-(p.frame_age_ms||0)-(performance.now()-started)):0;
validUntil=performance.now()+remaining;
state.textContent=(valid?'자세 입력 유효':'자세 입력 사용 불가')+' · '+(s.reason||'starting');state.className=valid?'ok':'warn';
cover.style.display=remaining>0?'none':'grid';cover.textContent='새 영상 없음 · '+(s.reason||'starting');
details.textContent='Backend: '+(s.active_backend||'unavailable')+'\nFrame age: '+(p.frame_age_ms==null?'—':p.frame_age_ms.toFixed(0)+' ms')+
'\nInference: '+(s.inference_ms==null?'—':s.inference_ms.toFixed(1)+' ms')+' · observed '+Number(s.inference_fps||0).toFixed(1)+' fps'+
'\nConfidence: '+(s.last_body_confidence??'—')+' · neutral supplied: '+Boolean(s.neutral_provided)+
'\nCamera restarts: '+(s.camera_restarts||0)+(p.encoder_error?'\nPreview: '+p.encoder_error:'');
}catch(e){validUntil=0;cover.style.display='grid';cover.textContent='미리보기 연결 끊김';state.textContent='상태 수신 불가';state.className='warn';}
finally{clearTimeout(timeout);}setTimeout(poll,150);}poll();</script></html>'''


class _Server(ThreadingHTTPServer):
    daemon_threads = True
    block_on_close = False
    request_queue_size = 4

    def __init__(self, address, handler, preview, blank):
        self.preview, self.blank = preview, blank
        self.stopped = threading.Event()
        self.slots = threading.BoundedSemaphore(4)
        super().__init__(address, handler)

    def process_request(self, request, address):
        request.settimeout(1.)
        if not self.slots.acquire(blocking=False):
            try:
                request.sendall(b'HTTP/1.0 503 Service Unavailable\r\nContent-Length: 0\r\n\r\n')
            finally:
                self.shutdown_request(request)
            return
        try:
            super().process_request(request, address)
        except Exception:
            self.slots.release()
            raise

    def process_request_thread(self, request, address):
        try:
            super().process_request_thread(request, address)
        finally:
            self.slots.release()


class _Handler(BaseHTTPRequestHandler):
    def log_message(self, *_args):
        pass

    def _headers(self, code, content_type, length=None):
        self.send_response(code)
        self.send_header('Content-Type', content_type)
        self.send_header('Cache-Control', 'no-store, max-age=0')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('Cross-Origin-Resource-Policy', 'same-origin')
        self.send_header('X-Frame-Options', 'DENY')
        if length is not None:
            self.send_header('Content-Length', str(length))
        self.end_headers()

    def do_GET(self):
        # Local-only address and Host check also reject browser DNS rebinding.
        try:
            host = urlsplit('http://' + self.headers.get('Host', '')).hostname
        except ValueError:
            host = None
        if host not in ('127.0.0.1', 'localhost'):
            self.send_error(403)
            return
        path = urlsplit(self.path).path
        try:
            if path == '/':
                data = PAGE.encode('utf-8')
                self._headers(200, 'text/html; charset=utf-8', len(data))
                self.wfile.write(data)
            elif path == '/status.json':
                data = json.dumps(self.server.preview.snapshot()[2], allow_nan=False).encode('utf-8')
                self._headers(200, 'application/json; charset=utf-8', len(data))
                self.wfile.write(data)
            elif path == '/stream.mjpg':
                self._headers(200, 'multipart/x-mixed-replace; boundary=oasisframe')
                last = None
                while not self.server.stopped.is_set():
                    sequence, jpeg, status = self.server.preview.snapshot()
                    identity = (sequence, jpeg is not None)
                    if identity != last:
                        payload = jpeg if jpeg is not None else self.server.blank
                        capture_ns = status['preview']['frame_capture_ns'] if jpeg is not None else 0
                        headers = (b'--oasisframe\r\nContent-Type: image/jpeg\r\nContent-Length: ' +
                                   str(len(payload)).encode() + b'\r\nX-Frame-Fresh: ' +
                                   (b'1' if jpeg is not None else b'0') + b'\r\nX-Capture-Ns: ' +
                                   str(capture_ns).encode() + b'\r\nX-Frame-Sequence: ' +
                                   str(sequence).encode() + b'\r\n\r\n')
                        self.wfile.write(headers + payload + b'\r\n')
                        self.wfile.flush()
                        last = identity
                    self.server.stopped.wait(.05)
            else:
                self.send_error(404)
        except (BrokenPipeError, ConnectionResetError, TimeoutError, OSError):
            return  # A browser cannot affect capture/inference or the pose UDP sender.


class PreviewServer:
    """Explicitly started localhost service; use an SSH tunnel to view from PC."""
    def __init__(self, port=8080, *, max_age_s=.3, max_width=960, quality=75,
                 min_confidence=.5, clock_ns=time.monotonic_ns, encoder=None, blank=None):
        if type(port) is not int or not 0 <= port <= 65535 or not 160 <= max_width <= 1920 or not 30 <= quality <= 95:
            raise ValueError('invalid_preview_port_or_encoding')
        encode = encoder or (lambda job: render_jpeg(job, max_width=max_width, quality=quality,
                                                    min_confidence=min_confidence))
        blank = placeholder_jpeg() if blank is None else blank
        self.latest = LatestPreview(max_age_s=max_age_s, encoder=encode, clock_ns=clock_ns)
        try:
            self.server = _Server(('127.0.0.1', port), _Handler, self.latest, blank)
        except Exception:
            self.latest.close()
            raise
        self.port = self.server.server_address[1]
        self.url = f'http://127.0.0.1:{self.port}/'
        self._thread = threading.Thread(target=self.server.serve_forever, kwargs={'poll_interval': .1},
                                        name='pose-preview-http', daemon=True)
        self._closed = False
        self._thread.start()

    def submit(self, frame, landmarks, status):
        self.latest.submit(frame, landmarks, status)

    def unavailable(self, status):
        self.latest.unavailable(status)

    def summary(self):
        return dict(self.latest.summary(), url=self.url, bind='127.0.0.1', max_clients=4)

    def close(self):
        if self._closed:
            return
        self._closed = True
        self.server.stopped.set()
        self.server.shutdown()
        self.server.server_close()
        self._thread.join(.5)
        self.latest.close()
