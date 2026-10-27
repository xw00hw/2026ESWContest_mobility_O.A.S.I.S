"""Posture features derived from MediaPipe Pose landmarks.

Landmark coordinates are MediaPipe-normalised (x, y in [0, 1] of the image, z
roughly in the same scale as x).  Shifts are reported both normalised and in
pixels so a change of preview resolution does not silently rescale the data.

Nothing here throws on a missing landmark: an absent or low-visibility point
propagates as ``None`` and clears ``vision_valid``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, asdict
from typing import Any, Sequence

__all__ = [
    "LANDMARK_NAMES", "TRACKED_INDICES", "PoseFeatures",
    "NeutralPosture", "NeutralCalibrator", "extract_features",
]

# The subset the DAQ schema promises, by MediaPipe Pose index.
LANDMARK_NAMES: dict[int, str] = {
    0: "nose",
    7: "left_ear",
    8: "right_ear",
    11: "left_shoulder",
    12: "right_shoulder",
    13: "left_elbow",
    14: "right_elbow",
    15: "left_wrist",
    16: "right_wrist",
    23: "left_hip",
    24: "right_hip",
}
TRACKED_INDICES = tuple(sorted(LANDMARK_NAMES))

# Landmarks that must be present for the frame to count as usable.
_CORE_INDICES = (11, 12, 23, 24)
_MIN_VISIBILITY = 0.5


@dataclass
class _Point:
    x: float
    y: float
    z: float
    visibility: float
    presence: float

    @property
    def usable(self) -> bool:
        return self.visibility >= _MIN_VISIBILITY


def _midpoint(a: _Point | None, b: _Point | None) -> tuple[float, float, float] | None:
    if a is None or b is None:
        return None
    return ((a.x + b.x) / 2.0, (a.y + b.y) / 2.0, (a.z + b.z) / 2.0)


@dataclass
class NeutralPosture:
    """The reference posture that shifts are measured against."""

    shoulder_center: tuple[float, float, float] | None = None
    hip_center: tuple[float, float, float] | None = None
    head_center: tuple[float, float, float] | None = None
    shoulder_width: float | None = None
    torso_angle_deg: float | None = None
    frames_used: int = 0
    established_monotonic_ns: int = 0
    valid: bool = False
    reason: str = "not attempted"

    def to_dict(self) -> dict:
        return asdict(self)


@dataclass
class PoseFeatures:
    """One row of ``pose/features.csv``."""

    host_monotonic_ns: int = 0
    host_wall_time_ns: int = 0
    frame_id: int = 0
    capture_monotonic_ns: int = 0

    shoulder_center_x: float | None = None
    shoulder_center_y: float | None = None
    hip_center_x: float | None = None
    hip_center_y: float | None = None
    head_center_x: float | None = None
    head_center_y: float | None = None

    shoulder_width: float | None = None
    torso_angle_deg: float | None = None

    shoulder_shift_x: float | None = None
    shoulder_shift_y: float | None = None
    hip_shift_x: float | None = None
    hip_shift_y: float | None = None
    head_shift_x: float | None = None
    head_shift_y: float | None = None

    shoulder_shift_px: float | None = None
    head_shift_px: float | None = None

    lean_score: float | None = None
    lean_velocity: float | None = None

    pose_confidence: float | None = None
    landmark_valid_mask: str = ""
    landmarks_visible: int = 0
    pose_processing_latency_ms: float | None = None
    neutral_valid: int = 0
    vision_valid: int = 0

    def to_row(self) -> dict[str, Any]:
        return asdict(self)


CSV_FIELDS = list(PoseFeatures().to_row().keys())


def _to_points(landmarks: Sequence[Any] | None) -> dict[int, _Point | None]:
    """Pick the tracked subset out of a MediaPipe landmark list."""
    out: dict[int, _Point | None] = {i: None for i in TRACKED_INDICES}
    if not landmarks:
        return out
    for idx in TRACKED_INDICES:
        if idx >= len(landmarks):
            continue
        lm = landmarks[idx]
        point = _Point(
            x=float(getattr(lm, "x", 0.0)),
            y=float(getattr(lm, "y", 0.0)),
            z=float(getattr(lm, "z", 0.0)),
            visibility=float(getattr(lm, "visibility", 0.0) or 0.0),
            presence=float(getattr(lm, "presence", 0.0) or 0.0),
        )
        out[idx] = point if point.usable else point  # keep it; usability checked later
    return out


def landmarks_to_records(landmarks: Sequence[Any] | None) -> list[dict]:
    """The full per-landmark payload written to ``pose/landmarks.jsonl``."""
    records: list[dict] = []
    if not landmarks:
        return records
    for idx in TRACKED_INDICES:
        if idx >= len(landmarks):
            continue
        lm = landmarks[idx]
        records.append({
            "i": idx,
            "n": LANDMARK_NAMES[idx],
            "x": round(float(getattr(lm, "x", 0.0)), 5),
            "y": round(float(getattr(lm, "y", 0.0)), 5),
            "z": round(float(getattr(lm, "z", 0.0)), 5),
            "v": round(float(getattr(lm, "visibility", 0.0) or 0.0), 4),
            "p": round(float(getattr(lm, "presence", 0.0) or 0.0), 4),
        })
    return records


def _torso_angle_deg(shoulder_c, hip_c) -> float | None:
    """Lean of the shoulder-over-hip axis; 0 deg = upright, + = leaning +x."""
    if shoulder_c is None or hip_c is None:
        return None
    dx = shoulder_c[0] - hip_c[0]
    dy = hip_c[1] - shoulder_c[1]          # image y grows downward
    if abs(dx) < 1e-9 and abs(dy) < 1e-9:
        return None
    return math.degrees(math.atan2(dx, dy))


def extract_features(
    landmarks: Sequence[Any] | None,
    *,
    frame_id: int,
    capture_monotonic_ns: int,
    host_monotonic_ns: int,
    host_wall_time_ns: int,
    neutral: NeutralPosture | None = None,
    previous: "PoseFeatures | None" = None,
    image_width: int = 640,
    image_height: int = 400,
    latency_ms: float | None = None,
) -> PoseFeatures:
    """Turn one landmark set into a feature row.  Never raises."""
    f = PoseFeatures(
        host_monotonic_ns=host_monotonic_ns,
        host_wall_time_ns=host_wall_time_ns,
        frame_id=frame_id,
        capture_monotonic_ns=capture_monotonic_ns,
        pose_processing_latency_ms=(round(latency_ms, 3)
                                    if latency_ms is not None else None),
    )

    points = _to_points(landmarks)
    # Bit per tracked landmark, MSB first, in TRACKED_INDICES order.
    mask_bits = "".join(
        "1" if (points[i] is not None and points[i].usable) else "0"
        for i in TRACKED_INDICES
    )
    f.landmark_valid_mask = mask_bits
    f.landmarks_visible = mask_bits.count("1")

    if not landmarks:
        f.vision_valid = 0
        f.neutral_valid = int(bool(neutral and neutral.valid))
        return f

    visibilities = [points[i].visibility for i in TRACKED_INDICES
                    if points[i] is not None]
    f.pose_confidence = round(sum(visibilities) / len(visibilities), 4) if visibilities else None

    core_ok = all(points[i] is not None and points[i].usable for i in _CORE_INDICES)
    shoulder_c = _midpoint(points[11], points[12])
    hip_c = _midpoint(points[23], points[24])

    # Head centre: nose plus both ears when visible, nose alone otherwise.
    head_pts = [points[i] for i in (0, 7, 8)
                if points[i] is not None and points[i].usable]
    head_c = None
    if head_pts:
        head_c = (sum(p.x for p in head_pts) / len(head_pts),
                  sum(p.y for p in head_pts) / len(head_pts),
                  sum(p.z for p in head_pts) / len(head_pts))

    if shoulder_c:
        f.shoulder_center_x, f.shoulder_center_y = round(shoulder_c[0], 5), round(shoulder_c[1], 5)
    if hip_c:
        f.hip_center_x, f.hip_center_y = round(hip_c[0], 5), round(hip_c[1], 5)
    if head_c:
        f.head_center_x, f.head_center_y = round(head_c[0], 5), round(head_c[1], 5)

    if points[11] is not None and points[12] is not None:
        f.shoulder_width = round(math.dist((points[11].x, points[11].y),
                                           (points[12].x, points[12].y)), 5)
    angle = _torso_angle_deg(shoulder_c, hip_c)
    f.torso_angle_deg = round(angle, 4) if angle is not None else None

    f.vision_valid = int(bool(core_ok and shoulder_c and hip_c))
    f.neutral_valid = int(bool(neutral and neutral.valid))

    if neutral and neutral.valid:
        if shoulder_c and neutral.shoulder_center:
            f.shoulder_shift_x = round(shoulder_c[0] - neutral.shoulder_center[0], 5)
            f.shoulder_shift_y = round(shoulder_c[1] - neutral.shoulder_center[1], 5)
            f.shoulder_shift_px = round(
                math.hypot(f.shoulder_shift_x * image_width,
                           f.shoulder_shift_y * image_height), 3)
        if hip_c and neutral.hip_center:
            f.hip_shift_x = round(hip_c[0] - neutral.hip_center[0], 5)
            f.hip_shift_y = round(hip_c[1] - neutral.hip_center[1], 5)
        if head_c and neutral.head_center:
            f.head_shift_x = round(head_c[0] - neutral.head_center[0], 5)
            f.head_shift_y = round(head_c[1] - neutral.head_center[1], 5)
            f.head_shift_px = round(
                math.hypot(f.head_shift_x * image_width,
                           f.head_shift_y * image_height), 3)

        # Lean score: lateral shoulder displacement normalised by the neutral
        # shoulder width, so it is comparable across drivers and camera zoom.
        if (f.shoulder_shift_x is not None and neutral.shoulder_width
                and neutral.shoulder_width > 1e-6):
            f.lean_score = round(f.shoulder_shift_x / neutral.shoulder_width, 5)

        if (f.lean_score is not None and previous is not None
                and previous.lean_score is not None
                and previous.capture_monotonic_ns is not None):
            # Differentiate against CAPTURE time, not the time the inference
            # result was dequeued: the latter carries 66-94 ms of variable
            # MediaPipe latency, which showed up as ~18% error in the
            # derivative over a ~101 ms sampling interval.
            # Guard on the elapsed time, not on the truthiness of the previous
            # timestamp -- a legitimate stamp of 0 must not disable velocity.
            dt = (capture_monotonic_ns - previous.capture_monotonic_ns) / 1e9
            if dt > 1e-6:
                f.lean_velocity = round((f.lean_score - previous.lean_score) / dt, 5)
    return f


class NeutralCalibrator:
    """Learn the neutral posture from a stable window early in the session.

    Frames are collected between ``start_after_s`` and
    ``start_after_s + window_s``.  The window is accepted only if enough valid
    frames arrived and the landmark scatter stayed below the configured
    threshold -- otherwise the reason is recorded and shifts stay ``None``
    rather than being measured against a bad reference.
    """

    def __init__(self, start_after_s: float = 5.0, window_s: float = 10.0,
                 min_valid_frames: int = 30, max_stddev_px: float = 12.0,
                 image_width: int = 640, image_height: int = 400):
        self.start_after_ns = int(start_after_s * 1e9)
        self.window_ns = int(window_s * 1e9)
        self.min_valid_frames = min_valid_frames
        self.max_stddev_px = max_stddev_px
        self.image_width = image_width
        self.image_height = image_height

        self._t0: int | None = None
        self._samples: list[PoseFeatures] = []
        self.result = NeutralPosture(reason="waiting for calibration window")
        self._closed = False

    @property
    def done(self) -> bool:
        return self._closed

    def feed(self, features: PoseFeatures, monotonic_ns: int) -> NeutralPosture:
        if self._closed:
            return self.result
        if self._t0 is None:
            self._t0 = monotonic_ns
        elapsed = monotonic_ns - self._t0
        if elapsed < self.start_after_ns:
            self.result.reason = "waiting for calibration window"
            return self.result
        if elapsed <= self.start_after_ns + self.window_ns:
            if features.vision_valid:
                self._samples.append(features)
            self.result.reason = f"collecting ({len(self._samples)} frames)"
            return self.result
        return self._finalise(monotonic_ns)

    def _finalise(self, monotonic_ns: int) -> NeutralPosture:
        self._closed = True
        n = len(self._samples)
        if n < self.min_valid_frames:
            self.result = NeutralPosture(
                frames_used=n, valid=False,
                reason=f"only {n} valid frames in window, need {self.min_valid_frames}",
            )
            return self.result

        def _mean(attr: str) -> float | None:
            vals = [getattr(s, attr) for s in self._samples if getattr(s, attr) is not None]
            return sum(vals) / len(vals) if vals else None

        def _stddev_px(attr: str, scale: float) -> float:
            vals = [getattr(s, attr) for s in self._samples if getattr(s, attr) is not None]
            if len(vals) < 2:
                return float("inf")
            mean = sum(vals) / len(vals)
            var = sum((v - mean) ** 2 for v in vals) / (len(vals) - 1)
            return math.sqrt(var) * scale

        scatter = max(
            _stddev_px("shoulder_center_x", self.image_width),
            _stddev_px("shoulder_center_y", self.image_height),
        )
        if scatter > self.max_stddev_px:
            self.result = NeutralPosture(
                frames_used=n, valid=False,
                reason=(f"posture not stable: shoulder scatter {scatter:.1f}px "
                        f"> {self.max_stddev_px}px"),
            )
            return self.result

        sx, sy = _mean("shoulder_center_x"), _mean("shoulder_center_y")
        hx, hy = _mean("hip_center_x"), _mean("hip_center_y")
        hex_, hey = _mean("head_center_x"), _mean("head_center_y")
        width = _mean("shoulder_width")
        angle = _mean("torso_angle_deg")

        if sx is None or hx is None or not width:
            self.result = NeutralPosture(
                frames_used=n, valid=False,
                reason="window lacked shoulder/hip geometry",
            )
            return self.result

        self.result = NeutralPosture(
            shoulder_center=(sx, sy, 0.0),
            hip_center=(hx, hy, 0.0),
            head_center=((hex_, hey, 0.0) if hex_ is not None else None),
            shoulder_width=width,
            torso_angle_deg=angle,
            frames_used=n,
            established_monotonic_ns=monotonic_ns,
            valid=True,
            reason="established from stable window",
        )
        return self.result
