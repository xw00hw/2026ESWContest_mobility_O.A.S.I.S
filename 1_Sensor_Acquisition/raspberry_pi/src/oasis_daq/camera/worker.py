"""Camera capture and raw recording.

The camera worker is the only owner of the V4L2 device.  It pulls *encoded*
MJPEG buffers -- ``CAP_PROP_CONVERT_RGB=0`` keeps OpenCV from decoding them --
and hands them straight to an ffmpeg ``segment`` muxer with ``-c copy``.  No
decode happens on the recording path, so recorded FPS does not depend on
MediaPipe or on the dashboard.

Frame timestamps are taken the instant ``read()`` returns and are written to
``camera/frame_timestamps.csv``; the container's own timestamps are nominal and
must not be used for analysis.
"""

from __future__ import annotations

import logging
import os
import queue
import subprocess
import threading
from pathlib import Path

import cv2

from ..clock import now
from ..storage import CsvWriter
from ..workers import BaseWorker, DropCountingQueue

log = logging.getLogger(__name__)

__all__ = ["CameraWorker", "probe_camera", "FRAME_CSV_FIELDS"]

FRAME_CSV_FIELDS = [
    "frame_id",
    "capture_monotonic_ns",
    "host_wall_time_ns",
    "segment_id",
    "frame_index_in_segment",
    "recorded",
    "capture_valid",
    "dropped_since_previous",
    "jpeg_bytes",
]

_JPEG_SOI = b"\xff\xd8"
_JPEG_EOI = b"\xff\xd9"


def _trim_jpeg(buf: bytes) -> tuple[bytes, bool]:
    """Trim the V4L2 buffer to the real end of the JPEG.

    UVC buffers are padded to a fixed size; ffmpeg reports "EOI missing,
    emulating" when the padding is passed through.  Returns ``(data, valid)``.
    """
    if not buf.startswith(_JPEG_SOI):
        return buf, False
    end = buf.rfind(_JPEG_EOI)
    if end == -1:
        return buf, False
    return buf[:end + 2], True


def probe_camera(device: str, width: int, height: int, fourcc: str,
                 fps: int, seconds: float = 3.0) -> dict:
    """Open the camera briefly and measure what it actually delivers."""
    import time
    result = {
        "device": device, "requested": {"width": width, "height": height,
                                        "fourcc": fourcc, "fps": fps},
        "opened": False, "actual": {}, "frames": 0, "measured_fps": 0.0,
        "valid_jpeg": 0, "mean_jpeg_bytes": 0, "error": None,
    }
    cap = None
    try:
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not cap.isOpened():
            result["error"] = "device did not open"
            return result
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
        result["opened"] = True
        result["actual"] = {
            "width": int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
            "height": int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
            "fps": cap.get(cv2.CAP_PROP_FPS),
            "convert_rgb": cap.get(cv2.CAP_PROP_CONVERT_RGB),
        }
        total = 0
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            ok, frame = cap.read()
            if not ok:
                continue
            result["frames"] += 1
            data, valid = _trim_jpeg(frame.tobytes())
            if valid:
                result["valid_jpeg"] += 1
                total += len(data)
        elapsed = max(time.monotonic() - t0, 1e-6)
        result["measured_fps"] = round(result["frames"] / elapsed, 2)
        result["mean_jpeg_bytes"] = (total // result["valid_jpeg"]
                                     if result["valid_jpeg"] else 0)
    except Exception as exc:
        result["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        if cap is not None:
            cap.release()
    return result


class CameraWorker(BaseWorker):
    """Capture MJPEG and record it without ever decoding on the write path."""

    def __init__(self, device: str, camera_dir: Path, width: int = 1280,
                 height: int = 800, fourcc: str = "MJPG", capture_fps: int = 100,
                 record_decimation: int = 4, segment_seconds: int = 120,
                 container: str = "mkv", flush_interval_s: float = 5.0,
                 fsync_interval_s: float = 15.0, write_queue_maxsize: int = 240,
                 on_event=None, pose_slot=None, preview_slot=None,
                 record: bool = True):
        super().__init__(name="camera", on_event=on_event)
        self.device = device
        self.camera_dir = camera_dir
        self.width = width
        self.height = height
        self.fourcc = fourcc
        self.capture_fps = max(int(capture_fps), 1)
        self.record_decimation = max(int(record_decimation), 1)
        self.segment_seconds = max(int(segment_seconds), 1)
        self.container = container
        self.flush_interval_s = flush_interval_s
        self.fsync_interval_s = fsync_interval_s
        self.pose_slot = pose_slot
        self.preview_slot = preview_slot
        self.record = record

        # Nominal rate handed to the muxer; real time lives in the CSV.
        self.record_fps = max(self.capture_fps / self.record_decimation, 1.0)
        self._frames_per_segment = max(
            int(round(self.segment_seconds * self.record_fps)), 1)

        self._cap: cv2.VideoCapture | None = None
        self._ffmpeg: subprocess.Popen | None = None
        self._csv: CsvWriter | None = None

        self.frame_id = 0
        self.frames_recorded = 0
        self.frames_invalid = 0
        self.estimated_drops = 0
        self.bytes_recorded = 0
        self.actual_settings: dict = {}
        self._last_capture_ns = 0
        self._expected_dt_ns = int(1e9 / self.capture_fps)
        self._recent: list[int] = []          # capture timestamps for live FPS
        self._synced_segments: set[str] = set()
        self._segment_sync_stop = threading.Event()
        self._segment_sync_thread: threading.Thread | None = None
        self._stderr_thread: threading.Thread | None = None
        self._stderr_tail: list[str] = []
        self._write_queue = DropCountingQueue(max(int(write_queue_maxsize), 8),
                                              name="camera_write")
        self._writer_thread: threading.Thread | None = None
        self._writer_stop = threading.Event()
        self._writer_error: BaseException | None = None
        self.write_queue_drops = 0
        self.segments_synced = 0

    # -- lifecycle -------------------------------------------------------
    def setup(self) -> None:
        self.camera_dir.mkdir(parents=True, exist_ok=True)
        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not cap.isOpened():
            raise RuntimeError(f"camera {self.device} did not open")
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.capture_fps)
        cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)     # hand back encoded JPEG
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 4)
        self._cap = cap
        self.actual_settings = {
            "width": int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
            "height": int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
            "fps": cap.get(cv2.CAP_PROP_FPS),
            "fourcc": self.fourcc,
            "record_fps_nominal": self.record_fps,
            "record_decimation": self.record_decimation,
        }
        self.stats.extra.update(self.actual_settings)
        self.stats.extra["device"] = self.device

        self._csv = CsvWriter(self.camera_dir / "frame_timestamps.csv",
                              FRAME_CSV_FIELDS, self.flush_interval_s,
                              self.fsync_interval_s)
        self._writer_thread = threading.Thread(target=self._writer_loop,
                                               name="oasis-camera-writer",
                                               daemon=True)
        if self.record:
            self._start_ffmpeg()
            self._segment_sync_thread = threading.Thread(
                target=self._segment_sync_loop, name="oasis-segment-sync",
                daemon=True)
            self._segment_sync_thread.start()
        self._writer_thread.start()
        log.info("camera: %s %dx%d @%s fps, recording every %d%s frame",
                 self.device, self.actual_settings["width"],
                 self.actual_settings["height"], self.actual_settings["fps"],
                 self.record_decimation,
                 {1: "st", 2: "nd", 3: "rd"}.get(self.record_decimation, "th"))

    def _start_ffmpeg(self) -> None:
        pattern = str(self.camera_dir / f"driver_%03d.{self.container}")
        seg_format = "matroska" if self.container == "mkv" else "mp4"
        cmd = [
            "ffmpeg", "-hide_banner", "-loglevel", "error",
            "-f", "image2pipe", "-vcodec", "mjpeg",
            "-framerate", f"{self.record_fps:g}", "-i", "-",
            "-c", "copy",
            "-f", "segment",
            "-segment_time", str(self.segment_seconds),
            "-reset_timestamps", "1",
            "-segment_format", seg_format,
            "-segment_list", str(self.camera_dir / "segments.csv"),
            "-segment_list_type", "csv",
            pattern,
        ]
        if seg_format == "mp4":
            # Fragmented MP4 so a truncated file is still readable.
            cmd[-1:-1] = ["-movflags", "+frag_keyframe+empty_moov+default_base_moof"]
        self._ffmpeg = subprocess.Popen(
            cmd, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
        )
        # stderr MUST be drained continuously.  A Linux pipe holds 64 KiB; if
        # ffmpeg fills it (a repeated per-frame mjpeg or muxer error at
        # AV_LOG_ERROR survives -loglevel error) it blocks writing stderr, stops
        # reading stdin, and the capture thread then blocks forever inside
        # proc.stdin.write().  Recording would stop with no symptom beyond a
        # stale-sensor warning.
        self._stderr_thread = threading.Thread(
            target=self._drain_stderr, args=(self._ffmpeg,),
            name="oasis-ffmpeg-stderr", daemon=True)
        self._stderr_thread.start()
        self.emit("recorder_started", command=" ".join(cmd))

    def _drain_stderr(self, proc: subprocess.Popen) -> None:
        """Keep ffmpeg's stderr pipe empty; retain the tail for diagnostics."""
        stream = proc.stderr
        if stream is None:
            return
        try:
            for raw in iter(stream.readline, b""):
                line = raw.decode("utf-8", "replace").rstrip()
                if not line:
                    continue
                self._stderr_tail.append(line)
                if len(self._stderr_tail) > 40:
                    del self._stderr_tail[:-40]
                log.warning("ffmpeg: %s", line)
        except (OSError, ValueError):
            pass
        finally:
            try:
                stream.close()
            except OSError:
                pass

    # -- main loop -------------------------------------------------------
    def work(self) -> None:
        assert self._cap is not None
        consecutive_failures = 0
        while not self.stopping():
            ok, frame = self._cap.read()
            capture_ns, wall_ns = now()
            if not ok or frame is None:
                consecutive_failures += 1
                self.frames_invalid += 1
                self.stats.invalid += 1
                if consecutive_failures >= 50:
                    self.emit("device_lost", device=self.device,
                              consecutive_failures=consecutive_failures)
                    raise RuntimeError(
                        f"camera {self.device} returned {consecutive_failures} "
                        "consecutive failed reads")
                continue
            consecutive_failures = 0
            self._handle_frame(frame, capture_ns, wall_ns)

    def _handle_frame(self, frame, capture_ns: int, wall_ns: int) -> None:
        self.frame_id += 1
        data, valid = _trim_jpeg(frame.tobytes())
        if not valid:
            self.frames_invalid += 1
            self.stats.invalid += 1

        # Infer gaps from the capture interval; V4L2 does not report drops here.
        dropped = 0
        if self._last_capture_ns:
            gap = capture_ns - self._last_capture_ns
            if gap > self._expected_dt_ns * 1.5:
                dropped = int(round(gap / self._expected_dt_ns)) - 1
                if dropped > 0:
                    self.estimated_drops += dropped
        self._last_capture_ns = capture_ns

        self._recent.append(capture_ns)
        if len(self._recent) > 120:
            del self._recent[:-120]

        # Newest frame for the pose process and the dashboard.  Both slots
        # overwrite, so a slow consumer can never stall capture.
        if valid:
            if self.pose_slot is not None:
                self.pose_slot.put((self.frame_id, capture_ns, wall_ns, data))
            if self.preview_slot is not None:
                self.preview_slot.put((self.frame_id, capture_ns, data))

        # Everything that touches the filesystem -- the ffmpeg pipe and the
        # timestamp CSV (which fsyncs on an interval) -- happens on the writer
        # thread.  The capture thread must never block on it: a blocking pipe
        # write cost 5 stalls of 125-289 ms in 14 minutes even after the
        # segment fsync was moved off-thread.
        to_write = data if (valid and self.record
                            and self.frame_id % self.record_decimation == 0) else None
        item = (self.frame_id, capture_ns, wall_ns, to_write, valid, dropped,
                len(data))
        if not self._write_queue.put_nowait_counted(item):
            # The writer is behind.  Losing the record is bad; blocking capture
            # is worse, because it loses the next frames too and corrupts their
            # timestamps.  Count it and carry on.
            self.write_queue_drops += 1
            if self.write_queue_drops in (1, 10, 100, 1000):
                self.emit("write_queue_overflow",
                          dropped=self.write_queue_drops,
                          queue_maxsize=self._write_queue.maxsize)
        self.note_sample()

        if self._writer_error is not None:
            raise self._writer_error

    # -- segment durability ----------------------------------------------
    def _segment_sync_loop(self) -> None:
        """fsync each segment once ffmpeg has finished writing it.

        Runs on its own thread.  ffmpeg closes a segment but leaves it in the
        page cache, which is not durable on this removable exFAT volume -- but
        fsyncing a finished 56 MB segment takes 0.6-0.7 s here, so doing it on
        the capture thread stalled capture once per rollover (measured: 10
        stalls of 583-743 ms in a 10 minute run, ~70 frames lost each).  The
        capture path must never block on the filesystem.

        ``segments.csv`` gains a line per completed segment, which is a cheap
        and reliable way to know which files are final.
        """
        listing = self.camera_dir / "segments.csv"
        while not self._segment_sync_stop.wait(5.0):
            self._sync_finished_segments(listing)
        self._sync_finished_segments(listing)     # final pass on shutdown

    def _sync_finished_segments(self, listing: Path) -> None:
        try:
            lines = listing.read_text().splitlines()
        except OSError:
            return
        for line in lines:
            name = line.split(",", 1)[0].strip()
            if not name or name in self._synced_segments:
                continue
            self._synced_segments.add(name)
            path = self.camera_dir / Path(name).name
            try:
                fd = os.open(path, os.O_RDONLY)
                try:
                    os.fsync(fd)
                    self.segments_synced += 1
                finally:
                    os.close(fd)
            except OSError:
                pass        # segment may be gone or the device may be away

    def _writer_loop(self) -> None:
        """Own the ffmpeg pipe and the timestamp CSV.

        Runs off the capture thread so neither a pipe stall nor a CSV fsync can
        add latency to `VideoCapture.read()`.  Segment bookkeeping lives here
        because it depends on how many frames were actually written.
        """
        while True:
            try:
                item = self._write_queue.get(timeout=0.5)
            except queue.Empty:
                if self._writer_stop.is_set():
                    break
                continue
            if item is None:
                break
            frame_id, capture_ns, wall_ns, data, valid, dropped, nbytes = item
            recorded, segment_id, index_in_segment = 0, -1, -1
            try:
                if data is not None and self._write_frame(data):
                    segment_id = self.frames_recorded // self._frames_per_segment
                    index_in_segment = self.frames_recorded % self._frames_per_segment
                    self.frames_recorded += 1
                    self.bytes_recorded += len(data)
                    recorded = 1
            except Exception as exc:
                # Surface it on the capture thread, which owns the worker's
                # lifecycle, rather than dying quietly here.
                self._writer_error = exc
                log.error("camera writer failed: %s", exc)
                break
            if self._csv is not None:
                self._csv.write({
                    "frame_id": frame_id,
                    "capture_monotonic_ns": capture_ns,
                    "host_wall_time_ns": wall_ns,
                    "segment_id": segment_id,
                    "frame_index_in_segment": index_in_segment,
                    "recorded": recorded,
                    "capture_valid": int(valid),
                    "dropped_since_previous": dropped,
                    "jpeg_bytes": nbytes,
                })

    def _write_frame(self, data: bytes) -> bool:
        proc = self._ffmpeg
        if proc is None or proc.stdin is None:
            return False
        if proc.poll() is not None:
            # The recorder died mid-session: that is a session-fatal error.
            # The stderr tail comes from the drain thread, which owns the pipe.
            self.emit("recorder_died", returncode=proc.poll(),
                      stderr="\n".join(self._stderr_tail[-10:]))
            raise RuntimeError(f"ffmpeg recorder exited with {proc.poll()}")
        try:
            proc.stdin.write(data)
            return True
        except (BrokenPipeError, OSError) as exc:
            self.emit("recorder_write_failed", error=str(exc))
            raise RuntimeError(f"writing to recorder failed: {exc}") from exc

    # -- shutdown --------------------------------------------------------
    def teardown(self) -> None:
        # Drain the writer first: frames still queued must reach ffmpeg before
        # its stdin is closed, or the tail of the session is silently lost.
        if self._writer_thread is not None:
            self._writer_stop.set()
            try:
                self._write_queue.put_nowait(None)
            except Exception:
                pass
            self._writer_thread.join(timeout=30)
            if self._writer_thread.is_alive():
                log.warning("camera: writer thread did not drain in time")
        # Stop syncing after ffmpeg is closed so the final pass sees every
        # segment ffmpeg has listed.
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                log.exception("camera: release failed")
        if self._ffmpeg is not None:
            proc = self._ffmpeg
            try:
                if proc.stdin and not proc.stdin.closed:
                    proc.stdin.close()          # flushes and finalises segments
            except OSError:
                pass
            try:
                proc.wait(timeout=20)
            except subprocess.TimeoutExpired:
                log.warning("camera: ffmpeg did not exit, terminating")
                proc.terminate()
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
            self.emit("recorder_stopped", returncode=proc.returncode,
                      frames=self.frames_recorded)
        if self._stderr_thread is not None:
            self._stderr_thread.join(timeout=5)
        if self._segment_sync_thread is not None:
            self._segment_sync_stop.set()
            self._segment_sync_thread.join(timeout=30)
        if self._csv is not None:
            self._csv.close()
        self.stats.extra.update({
            "frames_captured": self.frame_id,
            "frames_recorded": self.frames_recorded,
            "frames_invalid": self.frames_invalid,
            "estimated_drops": self.estimated_drops,
            "bytes_recorded": self.bytes_recorded,
            "segments_fsynced": self.segments_synced,
            "write_queue_drops": self.write_queue_drops,
            "recorder_stderr_tail": list(self._stderr_tail[-10:]),
            "segments": sorted(p.name for p in self.camera_dir.glob(
                f"driver_*.{self.container}")),
        })

    # -- live view -------------------------------------------------------
    def measured_fps(self) -> float:
        """Capture rate over the recent window, not since start."""
        if len(self._recent) < 2:
            return 0.0
        span = (self._recent[-1] - self._recent[0]) / 1e9
        return (len(self._recent) - 1) / span if span > 0 else 0.0

    def snapshot(self) -> dict:
        return {
            "device": self.device,
            "alive": self.stats.running,
            "connected": bool(self._cap is not None and self._cap.isOpened()),
            "width": self.actual_settings.get("width"),
            "height": self.actual_settings.get("height"),
            "configured_fps": self.actual_settings.get("fps"),
            "measured_fps": round(self.measured_fps(), 2),
            "recording": bool(self.record and self._ffmpeg is not None
                              and self._ffmpeg.poll() is None),
            "record_fps_nominal": self.record_fps,
            "frames_captured": self.frame_id,
            "frames_recorded": self.frames_recorded,
            "frames_invalid": self.frames_invalid,
            "estimated_drops": self.estimated_drops,
            "mb_recorded": round(self.bytes_recorded / (1024 * 1024), 2),
            "age_s": (None if self.stats.age_s() == float("inf")
                      else round(self.stats.age_s(), 2)),
        }
