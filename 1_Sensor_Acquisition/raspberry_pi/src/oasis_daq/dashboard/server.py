"""Optional laptop dashboard.

A small threaded HTTP server that *observes* the supervisor and offers
START/STOP.  It is deliberately isolated from the acquisition path:

* preview frames are re-encoded from a newest-wins slot, never from the
  recording pipeline;
* every handler is wrapped so a browser error cannot propagate into a worker;
* closing the browser, losing Wi-Fi or killing the server has no effect on the
  running session -- recording state lives entirely in the supervisor.
"""

from __future__ import annotations

import json
import logging
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

from ..clock import monotonic_ns

log = logging.getLogger(__name__)

__all__ = ["DashboardServer"]

_STATIC_DIR = Path(__file__).resolve().parent / "static"


class _PreviewEncoder:
    """Re-encode the newest frame at a bounded rate, shared by all clients."""

    def __init__(self, preview_slot, width: int, quality: int, fps: float):
        self._slot = preview_slot
        self._width = width
        self._quality = quality
        self._min_interval_ns = int(1e9 / max(fps, 0.5))
        self._lock = threading.Lock()
        self._jpeg: bytes | None = None
        self._stamp = 0
        self._frame_id = 0

    def latest(self) -> tuple[bytes | None, int]:
        with self._lock:
            if (self._jpeg is not None
                    and monotonic_ns() - self._stamp < self._min_interval_ns):
                return self._jpeg, self._frame_id
        item, _, _ = self._slot.get()
        if item is None:
            with self._lock:
                return self._jpeg, self._frame_id
        frame_id, _capture_ns, data = item
        try:
            import cv2
            import numpy as np
            img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_GRAYSCALE)
            if img is None:
                return None, 0
            h, w = img.shape[:2]
            if w > self._width:
                scale = self._width / float(w)
                img = cv2.resize(img, (self._width, max(int(h * scale), 1)),
                                 interpolation=cv2.INTER_AREA)
            ok, buf = cv2.imencode(".jpg", img,
                                   [cv2.IMWRITE_JPEG_QUALITY, self._quality])
            if not ok:
                return None, 0
            jpeg = buf.tobytes()
        except Exception:
            log.exception("preview encode failed")
            return None, 0
        with self._lock:
            self._jpeg, self._stamp, self._frame_id = jpeg, monotonic_ns(), frame_id
            return jpeg, frame_id


class _Handler(BaseHTTPRequestHandler):
    server_version = "OasisDAQ/0.1"
    protocol_version = "HTTP/1.1"
    # Without this a client that connects and sends nothing (or declares a
    # Content-Length and never sends the body) pins a worker thread forever, on
    # an unauthenticated port.  It cannot stop recording, but it is free
    # resource exhaustion.  The MJPEG stream writes continuously so it is not
    # affected by a read timeout.
    timeout = 30

    # -- plumbing --------------------------------------------------------
    def log_message(self, fmt: str, *args) -> None:   # quieten stderr spam
        log.debug("dashboard %s - %s", self.address_string(), fmt % args)

    @property
    def supervisor(self):
        return self.server.supervisor          # type: ignore[attr-defined]

    @property
    def preview(self) -> _PreviewEncoder:
        return self.server.preview             # type: ignore[attr-defined]

    def _send_json(self, payload, status: int = 200) -> None:
        body = json.dumps(payload, default=str).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def _send_bytes(self, body: bytes, content_type: str, status: int = 200) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    # -- routes ----------------------------------------------------------
    def do_GET(self) -> None:                          # noqa: N802
        try:
            path = urlparse(self.path).path
            if path in ("/", "/index.html"):
                return self._serve_index()
            if path == "/api/status":
                return self._send_json(self.supervisor.status())
            if path == "/api/health":
                return self._send_json(self.supervisor.health_row())
            if path == "/api/latest":
                return self._send_json(self._latest())
            if path == "/api/preview.jpg":
                return self._serve_preview()
            if path == "/api/stream.mjpg":
                return self._serve_stream()
            if path == "/api/route.geojson":
                return self._serve_route()
            self._send_json({"error": "not found", "path": path}, status=404)
        except (BrokenPipeError, ConnectionResetError):
            pass                                       # client went away, fine
        except Exception as exc:
            log.exception("dashboard GET %s failed", self.path)
            try:
                self._send_json({"error": str(exc)}, status=500)
            except Exception:
                pass

    def do_POST(self) -> None:                         # noqa: N802
        try:
            path = urlparse(self.path).path
            length = int(self.headers.get("Content-Length") or 0)
            if length > 64 * 1024:      # no legitimate control request is large
                return self._send_json({"error": "request body too large"},
                                       status=413)
            raw = self.rfile.read(length) if length else b""
            try:
                body = json.loads(raw) if raw else {}
            except json.JSONDecodeError:
                body = {}
            if path == "/api/session/start":
                result = self.supervisor.start_session(
                    notes=str(body.get("notes", "")),
                    suffix=str(body.get("suffix", "")))
                return self._send_json(result, status=200 if result.get("ok") else 409)
            if path == "/api/session/stop":
                result = self.supervisor.stop_session(
                    reason=str(body.get("reason", "dashboard")))
                return self._send_json(result, status=200 if result.get("ok") else 409)
            self._send_json({"error": "not found", "path": path}, status=404)
        except (BrokenPipeError, ConnectionResetError):
            pass
        except Exception as exc:
            log.exception("dashboard POST %s failed", self.path)
            try:
                self._send_json({"error": str(exc)}, status=500)
            except Exception:
                pass

    # -- handlers --------------------------------------------------------
    def _serve_index(self) -> None:
        index = _STATIC_DIR / "index.html"
        if not index.exists():
            return self._send_bytes(b"<h1>dashboard assets missing</h1>",
                                    "text/html; charset=utf-8", status=500)
        self._send_bytes(index.read_bytes(), "text/html; charset=utf-8")

    def _latest(self) -> dict:
        sup = self.supervisor
        gps, _, _ = sup.gps_slot.get()
        imu, _, _ = sup.imu_slot.get()
        pose, _, _ = sup.pose_result_slot.get()
        return {"gps": gps, "imu": imu, "pose": pose,
                "state": sup.state, "session_id":
                    (sup.paths.session_id if sup.paths else None)}

    def _serve_preview(self) -> None:
        jpeg, _ = self.preview.latest()
        if not jpeg:
            return self._send_json({"error": "no frame available"}, status=503)
        self._send_bytes(jpeg, "image/jpeg")

    def _serve_stream(self) -> None:
        """multipart/x-mixed-replace MJPEG stream for the live preview."""
        boundary = "oasisframe"
        self.send_response(200)
        self.send_header("Content-Type",
                         f"multipart/x-mixed-replace; boundary={boundary}")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")
        self.end_headers()
        interval = 1.0 / max(self.server.preview_fps, 0.5)   # type: ignore[attr-defined]
        last_id = -1
        try:
            while not self.server.stopping.is_set():         # type: ignore[attr-defined]
                jpeg, frame_id = self.preview.latest()
                if jpeg and frame_id != last_id:
                    last_id = frame_id
                    self.wfile.write(
                        f"--{boundary}\r\nContent-Type: image/jpeg\r\n"
                        f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                    self.wfile.write(jpeg)
                    self.wfile.write(b"\r\n")
                    self.wfile.flush()
                import time
                time.sleep(interval)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass    # the laptop closed the tab; recording is unaffected

    def _serve_route(self) -> None:
        """Live track as GeoJSON so the map can follow the drive."""
        sup = self.supervisor
        track = list(sup.gps.track) if sup.gps is not None else []
        self._send_json({
            "type": "FeatureCollection",
            "features": [{
                "type": "Feature",
                "geometry": {"type": "LineString",
                             "coordinates": [[lon, lat] for lon, lat in track]},
                "properties": {"point_count": len(track)},
            }] if len(track) >= 2 else [],
        })


class DashboardServer:
    """Runs the HTTP server on its own thread."""

    def __init__(self, supervisor, host: str = "0.0.0.0", port: int = 8080,
                 preview_width: int = 640, preview_quality: int = 60,
                 preview_fps: float = 8.0):
        self.supervisor = supervisor
        self.host = host
        self.port = port
        self._httpd: ThreadingHTTPServer | None = None
        self._thread: threading.Thread | None = None
        self._preview = _PreviewEncoder(supervisor.preview_slot, preview_width,
                                        preview_quality, preview_fps)
        self._preview_fps = preview_fps
        self._stopping = threading.Event()

    def start(self) -> None:
        # A browser opens several parallel connections (the long-lived MJPEG
        # stream, status polling, the map) and socketserver's default backlog of
        # 5 is easy to exhaust, which shows up as a refused connection rather
        # than a queued one.
        ThreadingHTTPServer.request_queue_size = 64
        httpd = ThreadingHTTPServer((self.host, self.port), _Handler)
        httpd.daemon_threads = True
        httpd.supervisor = self.supervisor          # type: ignore[attr-defined]
        httpd.preview = self._preview               # type: ignore[attr-defined]
        httpd.preview_fps = self._preview_fps       # type: ignore[attr-defined]
        httpd.stopping = self._stopping             # type: ignore[attr-defined]
        self._httpd = httpd
        self._thread = threading.Thread(target=httpd.serve_forever,
                                        kwargs={"poll_interval": 0.5},
                                        name="oasis-dashboard", daemon=True)
        self._thread.start()
        log.info("dashboard listening on http://%s:%d", self.host, self.port)

    def stop(self) -> None:
        self._stopping.set()
        if self._httpd is not None:
            self._httpd.shutdown()
            self._httpd.server_close()
        if self._thread is not None:
            self._thread.join(timeout=5)
