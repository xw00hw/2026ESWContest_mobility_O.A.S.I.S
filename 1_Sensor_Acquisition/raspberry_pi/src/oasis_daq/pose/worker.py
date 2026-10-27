"""MediaPipe Pose worker.

Inference runs in a **separate process**, not a thread: a MediaPipe frame costs
~70 ms of native work on a Pi 5, and keeping it out of the capture process
guarantees that recorded FPS cannot be dragged down by vision load.  The parent
side owns all file writing so the child stays a pure function of its input.

The child is fed the *newest* frame only.  When it falls behind, frames are
skipped rather than queued -- pose output is allowed to run slower than
recording, which is the documented design.
"""

from __future__ import annotations

import logging
import multiprocessing as mp
import queue
import time
from pathlib import Path

from ..clock import now, monotonic_ns
from ..storage import CsvWriter, JsonlWriter
from ..workers import BaseWorker
from . import features as feat

log = logging.getLogger(__name__)

__all__ = ["PoseWorker", "pose_process_main", "DEFAULT_MODEL_DIR"]

DEFAULT_MODEL_DIR = Path.home() / ".local/share/oasis-daq/models"
_STOP = None            # sentinel pushed to end the child cleanly


def pose_process_main(model_path: str, in_q, out_q, ready_ev,
                      input_width: int, input_height: int,
                      grayscale_to_rgb: bool,
                      min_detection_confidence: float,
                      min_presence_confidence: float,
                      min_tracking_confidence: float) -> None:
    """Child-process entry point.  Imports heavy deps only here."""
    try:
        import cv2
        import numpy as np
        import mediapipe as mp_lib
        from mediapipe.tasks.python import vision
        from mediapipe.tasks.python.core.base_options import BaseOptions

        landmarker = vision.PoseLandmarker.create_from_options(
            vision.PoseLandmarkerOptions(
                base_options=BaseOptions(model_asset_path=model_path),
                running_mode=vision.RunningMode.VIDEO,
                num_poses=1,
                min_pose_detection_confidence=min_detection_confidence,
                min_pose_presence_confidence=min_presence_confidence,
                min_tracking_confidence=min_tracking_confidence,
                output_segmentation_masks=False,
            )
        )
    except Exception as exc:                       # report, do not crash silently
        try:
            out_q.put(("fatal", f"{type(exc).__name__}: {exc}"))
        except Exception:
            pass
        return

    ready_ev.set()
    # MediaPipe VIDEO mode needs strictly increasing millisecond timestamps.
    last_ts_ms = 0
    try:
        while True:
            try:
                item = in_q.get(timeout=1.0)
            except queue.Empty:
                continue
            if item is _STOP or item is None:
                break
            frame_id, capture_ns, wall_ns, jpeg = item
            t_start = time.perf_counter()
            try:
                buf = np.frombuffer(jpeg, dtype=np.uint8)
                gray = cv2.imdecode(buf, cv2.IMREAD_GRAYSCALE
                                    if grayscale_to_rgb else cv2.IMREAD_COLOR)
                if gray is None:
                    out_q.put(("invalid", frame_id, capture_ns, wall_ns, None, 0.0))
                    continue
                if (gray.shape[1], gray.shape[0]) != (input_width, input_height):
                    gray = cv2.resize(gray, (input_width, input_height),
                                      interpolation=cv2.INTER_AREA)
                rgb = (cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)
                       if grayscale_to_rgb else cv2.cvtColor(gray, cv2.COLOR_BGR2RGB))
                image = mp_lib.Image(image_format=mp_lib.ImageFormat.SRGB,
                                     data=np.ascontiguousarray(rgb))
                ts_ms = max(capture_ns // 1_000_000, last_ts_ms + 1)
                last_ts_ms = ts_ms
                result = landmarker.detect_for_video(image, ts_ms)
                latency_ms = (time.perf_counter() - t_start) * 1000.0
                # Landmarks are converted to plain tuples: MediaPipe objects do
                # not survive pickling across the process boundary.
                payload = None
                if result.pose_landmarks:
                    payload = [
                        (lm.x, lm.y, lm.z,
                         float(lm.visibility or 0.0), float(lm.presence or 0.0))
                        for lm in result.pose_landmarks[0]
                    ]
                out_q.put(("ok", frame_id, capture_ns, wall_ns, payload, latency_ms))
            except Exception as exc:
                latency_ms = (time.perf_counter() - t_start) * 1000.0
                try:
                    out_q.put(("error", frame_id, capture_ns, wall_ns,
                               f"{type(exc).__name__}: {exc}", latency_ms))
                except Exception:
                    break
    finally:
        try:
            landmarker.close()
        except Exception:
            pass


class _Landmark:
    """Rehydrated landmark so features.py sees the attribute names it expects."""

    __slots__ = ("x", "y", "z", "visibility", "presence")

    def __init__(self, t):
        self.x, self.y, self.z, self.visibility, self.presence = t


class PoseWorker(BaseWorker):
    """Feed the newest frame to the pose process and persist its output."""

    def __init__(self, pose_dir: Path, pose_slot, model_path: Path,
                 target_fps: float = 10.0, input_width: int = 640,
                 input_height: int = 400, grayscale_to_rgb: bool = True,
                 min_detection_confidence: float = 0.5,
                 min_tracking_confidence: float = 0.5,
                 neutral_cfg: dict | None = None,
                 flush_interval_s: float = 5.0, fsync_interval_s: float = 15.0,
                 on_event=None, result_slot=None):
        super().__init__(name="pose", on_event=on_event)
        self.pose_dir = pose_dir
        self.pose_slot = pose_slot
        self.model_path = Path(model_path)
        self.target_fps = max(float(target_fps), 0.5)
        self.input_width = input_width
        self.input_height = input_height
        self.grayscale_to_rgb = grayscale_to_rgb
        self.min_detection_confidence = min_detection_confidence
        self.min_tracking_confidence = min_tracking_confidence
        self.flush_interval_s = flush_interval_s
        self.fsync_interval_s = fsync_interval_s
        self.result_slot = result_slot

        cfg = neutral_cfg or {}
        self._neutral_enabled = bool(cfg.get("enabled", True))
        self.calibrator = feat.NeutralCalibrator(
            start_after_s=float(cfg.get("start_after_s", 5.0)),
            window_s=float(cfg.get("window_s", 10.0)),
            min_valid_frames=int(cfg.get("min_valid_frames", 30)),
            max_stddev_px=float(cfg.get("max_landmark_stddev_px", 12.0)),
            image_width=input_width, image_height=input_height,
        )
        self.neutral = feat.NeutralPosture(
            reason="disabled" if not self._neutral_enabled else "not attempted")

        self._ctx = mp.get_context("spawn")
        self._in_q = None
        self._out_q = None
        self._proc = None
        self._ready = None

        self._jsonl: JsonlWriter | None = None
        self._csv: CsvWriter | None = None
        self._previous: feat.PoseFeatures | None = None
        self._last_sent_ns = 0
        self._last_seq = 0

        self.frames_submitted = 0
        self.frames_processed = 0
        self.frames_with_pose = 0
        self.frames_invalid = 0
        self.child_errors = 0
        self.latencies: list[float] = []
        self.last_features: feat.PoseFeatures | None = None
        self.last_landmarks: list[dict] | None = None

    # -- lifecycle -------------------------------------------------------
    def setup(self) -> None:
        if not self.model_path.exists():
            raise FileNotFoundError(
                f"pose model not found: {self.model_path} -- run "
                "scripts/fetch_pose_model.sh")
        self.pose_dir.mkdir(parents=True, exist_ok=True)
        self._jsonl = JsonlWriter(self.pose_dir / "landmarks.jsonl",
                                  self.flush_interval_s, self.fsync_interval_s)
        self._csv = CsvWriter(self.pose_dir / "features.csv", feat.CSV_FIELDS,
                              self.flush_interval_s, self.fsync_interval_s)

        self._in_q = self._ctx.Queue(maxsize=2)
        self._out_q = self._ctx.Queue(maxsize=64)
        self._ready = self._ctx.Event()
        self._proc = self._ctx.Process(
            target=pose_process_main,
            args=(str(self.model_path), self._in_q, self._out_q, self._ready,
                  self.input_width, self.input_height, self.grayscale_to_rgb,
                  self.min_detection_confidence, 0.5,
                  self.min_tracking_confidence),
            daemon=True, name="oasis-pose",
        )
        self._proc.start()
        if not self._ready.wait(timeout=60):
            raise RuntimeError("pose process did not become ready within 60 s")
        self.stats.extra["model"] = self.model_path.name
        self.stats.extra["target_fps"] = self.target_fps
        log.info("pose: process ready (pid=%s, model=%s)",
                 self._proc.pid, self.model_path.name)

    def work(self) -> None:
        interval_ns = int(1e9 / self.target_fps)
        while not self.stopping():
            self._drain_results()
            if self._proc is not None and not self._proc.is_alive():
                raise RuntimeError(
                    f"pose process exited with code {self._proc.exitcode}")
            self._maybe_submit(interval_ns)
            time.sleep(0.002)
        # Give the child a moment to hand back what it already started.
        deadline = monotonic_ns() + int(2e9)
        while monotonic_ns() < deadline:
            if not self._drain_results():
                break

    def _maybe_submit(self, interval_ns: int) -> None:
        if monotonic_ns() - self._last_sent_ns < interval_ns:
            return
        item, _, seq = self.pose_slot.get()
        if item is None or seq == self._last_seq:
            return          # nothing new since last submission
        try:
            self._in_q.put_nowait(item)
        except queue.Full:
            # The child is behind.  Skipping is the designed behaviour -- pose
            # is allowed to run slower than capture -- but back off for a full
            # interval rather than retrying every 2 ms: without this the loop
            # re-attempted the same frame ~500 times a second and `dropped`
            # counted retries, not frames (5325 "drops" for 592 processed
            # frames in one 86 s session).
            self._last_sent_ns = monotonic_ns()
            self.stats.dropped += 1
            return
        self._last_seq = seq
        self._last_sent_ns = monotonic_ns()
        self.frames_submitted += 1

    def _drain_results(self) -> bool:
        """Consume everything the child has produced.  True if anything came."""
        got = False
        while True:
            try:
                msg = self._out_q.get_nowait()
            except queue.Empty:
                return got
            got = True
            kind = msg[0]
            if kind == "fatal":
                self.child_errors += 1
                raise RuntimeError(f"pose process failed to start: {msg[1]}")
            if kind == "error":
                self.child_errors += 1
                self.stats.errors += 1
                self.stats.last_error = str(msg[4])
                continue
            _, frame_id, capture_ns, wall_ns, payload, latency_ms = msg
            self._handle_result(frame_id, capture_ns, wall_ns, payload, latency_ms)

    def _handle_result(self, frame_id: int, capture_ns: int, wall_ns: int,
                       payload, latency_ms: float) -> None:
        self.frames_processed += 1
        self.latencies.append(latency_ms)
        if len(self.latencies) > 4000:
            del self.latencies[:-2000]

        landmarks = [_Landmark(t) for t in payload] if payload else None
        if landmarks:
            self.frames_with_pose += 1
        else:
            self.frames_invalid += 1

        mono, wall = now()
        f = feat.extract_features(
            landmarks, frame_id=frame_id, capture_monotonic_ns=capture_ns,
            host_monotonic_ns=mono, host_wall_time_ns=wall,
            neutral=self.neutral if self.neutral.valid else None,
            previous=self._previous, image_width=self.input_width,
            image_height=self.input_height, latency_ms=latency_ms,
        )

        if self._neutral_enabled and not self.calibrator.done:
            previous_valid = self.neutral.valid
            self.neutral = self.calibrator.feed(f, mono)
            if self.neutral.valid and not previous_valid:
                self.emit("neutral_calibrated",
                          frames_used=self.neutral.frames_used,
                          shoulder_width=self.neutral.shoulder_width)
            elif self.calibrator.done and not self.neutral.valid:
                self.emit("neutral_calibration_failed", reason=self.neutral.reason)

        if self._csv is not None:
            self._csv.write(f.to_row())
        if self._jsonl is not None and landmarks:
            self._jsonl.write({
                "frame_id": frame_id,
                "capture_monotonic_ns": capture_ns,
                "host_wall_time_ns": wall_ns,
                "latency_ms": round(latency_ms, 3),
                "landmarks": feat.landmarks_to_records(landmarks),
            })
        self.last_features = f
        self.last_landmarks = feat.landmarks_to_records(landmarks) if landmarks else None
        if self.result_slot is not None:
            self.result_slot.put({"features": f.to_row(),
                                  "landmarks": self.last_landmarks})
        self._previous = f
        self.note_sample()

    def teardown(self) -> None:
        if self._in_q is not None:
            try:
                self._in_q.put_nowait(_STOP)
            except Exception:
                pass
        if self._proc is not None:
            self._proc.join(timeout=8)
            if self._proc.is_alive():
                log.warning("pose: process did not exit, terminating")
                self._proc.terminate()
                self._proc.join(timeout=5)
        for w in (self._csv, self._jsonl):
            if w is not None:
                w.close()
        self.stats.extra.update({
            "frames_submitted": self.frames_submitted,
            "frames_processed": self.frames_processed,
            "frames_with_pose": self.frames_with_pose,
            "valid_ratio": (round(self.frames_with_pose / self.frames_processed, 4)
                            if self.frames_processed else 0.0),
            "latency_p50_ms": self.latency_percentile(50),
            "latency_p95_ms": self.latency_percentile(95),
            "neutral": self.neutral.to_dict(),
        })

    # -- live view -------------------------------------------------------
    def latency_percentile(self, pct: float) -> float | None:
        if not self.latencies:
            return None
        ordered = sorted(self.latencies)
        k = min(int(len(ordered) * pct / 100.0), len(ordered) - 1)
        return round(ordered[k], 2)

    def snapshot(self) -> dict:
        f = self.last_features
        return {
            "alive": self.stats.running,
            "process_alive": bool(self._proc is not None and self._proc.is_alive()),
            "model": self.model_path.name,
            "fps": round(self.stats.rate_hz(), 2),
            "target_fps": self.target_fps,
            "frames_submitted": self.frames_submitted,
            "frames_processed": self.frames_processed,
            "frames_with_pose": self.frames_with_pose,
            "valid_ratio": (round(self.frames_with_pose / self.frames_processed, 4)
                            if self.frames_processed else 0.0),
            "dropped_frames": self.stats.dropped,
            "latency_p50_ms": self.latency_percentile(50),
            "latency_p95_ms": self.latency_percentile(95),
            "pose_confidence": f.pose_confidence if f else None,
            "lean_score": f.lean_score if f else None,
            "lean_velocity": f.lean_velocity if f else None,
            "torso_angle_deg": f.torso_angle_deg if f else None,
            "vision_valid": bool(f.vision_valid) if f else False,
            "neutral_valid": self.neutral.valid,
            "neutral_reason": self.neutral.reason,
            "landmarks": self.last_landmarks,
        }
