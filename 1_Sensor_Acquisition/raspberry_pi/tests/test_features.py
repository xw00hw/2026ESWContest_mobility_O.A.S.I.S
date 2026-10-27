"""Posture features and neutral-posture calibration."""

import math

import pytest

from oasis_daq.pose import features as feat


class LM:
    """Stand-in for a MediaPipe NormalizedLandmark."""

    def __init__(self, x, y, z=0.0, visibility=1.0, presence=1.0):
        self.x, self.y, self.z = x, y, z
        self.visibility, self.presence = visibility, presence


def make_pose(dx=0.0, dy=0.0, visibility=1.0):
    """33 landmarks with an upright torso, optionally translated."""
    pts = [LM(0.5 + dx, 0.5 + dy, visibility=visibility) for _ in range(33)]
    pts[0] = LM(0.50 + dx, 0.20 + dy, visibility=visibility)   # nose
    pts[7] = LM(0.46 + dx, 0.22 + dy, visibility=visibility)   # left ear
    pts[8] = LM(0.54 + dx, 0.22 + dy, visibility=visibility)   # right ear
    pts[11] = LM(0.40 + dx, 0.40 + dy, visibility=visibility)  # left shoulder
    pts[12] = LM(0.60 + dx, 0.40 + dy, visibility=visibility)  # right shoulder
    pts[23] = LM(0.44 + dx, 0.70 + dy, visibility=visibility)  # left hip
    pts[24] = LM(0.56 + dx, 0.70 + dy, visibility=visibility)  # right hip
    return pts


def extract(pts, **kw):
    kw.setdefault("frame_id", 1)
    kw.setdefault("capture_monotonic_ns", 0)
    kw.setdefault("host_monotonic_ns", 0)
    kw.setdefault("host_wall_time_ns", 0)
    return feat.extract_features(pts, **kw)


def test_centres_are_midpoints_of_the_landmark_pairs():
    f = extract(make_pose())
    assert f.shoulder_center_x == pytest.approx(0.5)
    assert f.shoulder_center_y == pytest.approx(0.4)
    assert f.hip_center_x == pytest.approx(0.5)
    assert f.shoulder_width == pytest.approx(0.2)


def test_upright_torso_has_zero_angle():
    assert extract(make_pose()).torso_angle_deg == pytest.approx(0.0, abs=1e-6)


def test_torso_angle_sign_follows_lean_direction():
    pts = make_pose()
    pts[11] = LM(0.50, 0.40)      # shift both shoulders to +x
    pts[12] = LM(0.70, 0.40)
    assert extract(pts).torso_angle_deg > 0


def test_missing_landmarks_never_raise_and_clear_vision_valid():
    assert extract(None).vision_valid == 0
    assert extract([]).vision_valid == 0


def test_low_visibility_core_landmarks_clear_vision_valid():
    f = extract(make_pose(visibility=0.1))
    assert f.vision_valid == 0
    assert f.landmark_valid_mask.count("1") == 0


def test_landmark_mask_length_matches_tracked_landmarks():
    f = extract(make_pose())
    assert len(f.landmark_valid_mask) == len(feat.TRACKED_INDICES)
    assert f.landmarks_visible == len(feat.TRACKED_INDICES)


def test_shifts_are_none_without_a_neutral_reference():
    f = extract(make_pose(dx=0.1))
    assert f.shoulder_shift_x is None and f.lean_score is None
    assert f.neutral_valid == 0


def _neutral():
    return feat.NeutralPosture(
        shoulder_center=(0.5, 0.4, 0.0), hip_center=(0.5, 0.7, 0.0),
        head_center=(0.5, 0.213, 0.0), shoulder_width=0.2,
        torso_angle_deg=0.0, frames_used=60, valid=True, reason="test")


def test_shift_and_lean_score_measured_against_neutral():
    f = extract(make_pose(dx=0.1), neutral=_neutral())
    assert f.shoulder_shift_x == pytest.approx(0.1)
    # lean_score normalises the shift by the neutral shoulder width
    assert f.lean_score == pytest.approx(0.5)
    assert f.neutral_valid == 1


def test_lean_score_is_scale_invariant_across_shoulder_widths():
    wide = feat.NeutralPosture(shoulder_center=(0.5, 0.4, 0.0),
                               hip_center=(0.5, 0.7, 0.0), shoulder_width=0.4,
                               valid=True, reason="t")
    f = extract(make_pose(dx=0.1), neutral=wide)
    assert f.lean_score == pytest.approx(0.25)


def test_lean_velocity_uses_the_capture_clock_difference():
    neutral = _neutral()
    first = extract(make_pose(dx=0.0), neutral=neutral, capture_monotonic_ns=0)
    second = extract(make_pose(dx=0.1), neutral=neutral,
                     capture_monotonic_ns=1_000_000_000, previous=first)
    # 0.5 lean units gained over exactly one second of CAPTURE time
    assert second.lean_velocity == pytest.approx(0.5, rel=1e-6)


def test_lean_velocity_ignores_inference_latency():
    """Regression: differentiating against the time the inference result was
    dequeued folded 66-94 ms of variable MediaPipe latency into the derivative.
    Two frames captured 1 s apart must give the same velocity no matter how
    long inference took on each."""
    neutral = _neutral()
    first = extract(make_pose(dx=0.0), neutral=neutral,
                    capture_monotonic_ns=0, host_monotonic_ns=70_000_000)
    second = extract(make_pose(dx=0.1), neutral=neutral,
                     capture_monotonic_ns=1_000_000_000,
                     host_monotonic_ns=1_150_000_000,   # 150 ms of latency
                     previous=first)
    assert second.lean_velocity == pytest.approx(0.5, rel=1e-6)


def test_shift_in_pixels_scales_with_image_size():
    f = extract(make_pose(dx=0.1), neutral=_neutral(),
                image_width=640, image_height=400)
    assert f.shoulder_shift_px == pytest.approx(64.0, rel=1e-6)


def test_latency_is_carried_through():
    assert extract(make_pose(), latency_ms=12.345).pose_processing_latency_ms == 12.345


def test_landmark_records_use_the_documented_subset():
    records = feat.landmarks_to_records(make_pose())
    assert {r["n"] for r in records} == set(feat.LANDMARK_NAMES.values())
    assert {"i", "n", "x", "y", "z", "v", "p"} == set(records[0])


# --- neutral calibration ------------------------------------------------
def test_calibrator_waits_for_the_configured_delay():
    cal = feat.NeutralCalibrator(start_after_s=5.0, window_s=10.0,
                                 min_valid_frames=3)
    result = cal.feed(extract(make_pose()), 0)
    assert result.valid is False and "waiting" in result.reason


def test_calibrator_establishes_neutral_from_a_stable_window():
    cal = feat.NeutralCalibrator(start_after_s=0.0, window_s=1.0,
                                 min_valid_frames=5, max_stddev_px=5.0)
    for i in range(20):
        cal.feed(extract(make_pose()), int(i * 0.05e9))
    result = cal.feed(extract(make_pose()), int(2e9))
    assert result.valid is True and result.frames_used == 20
    assert result.shoulder_width == pytest.approx(0.2)


def test_calibrator_rejects_an_unstable_window_with_a_reason():
    cal = feat.NeutralCalibrator(start_after_s=0.0, window_s=1.0,
                                 min_valid_frames=5, max_stddev_px=1.0,
                                 image_width=640, image_height=400)
    for i in range(20):
        cal.feed(extract(make_pose(dx=0.02 * (i % 5))), int(i * 0.05e9))
    result = cal.feed(extract(make_pose()), int(2e9))
    assert result.valid is False and "not stable" in result.reason


def test_calibrator_reports_too_few_frames_rather_than_guessing():
    cal = feat.NeutralCalibrator(start_after_s=0.0, window_s=1.0,
                                 min_valid_frames=50)
    for i in range(5):
        cal.feed(extract(make_pose()), int(i * 0.05e9))
    result = cal.feed(extract(make_pose()), int(2e9))
    assert result.valid is False and "only 5 valid frames" in result.reason


def test_calibrator_ignores_frames_without_a_usable_pose():
    cal = feat.NeutralCalibrator(start_after_s=0.0, window_s=1.0,
                                 min_valid_frames=1, max_stddev_px=5.0)
    for i in range(10):
        cal.feed(extract(None), int(i * 0.05e9))
    result = cal.feed(extract(make_pose()), int(2e9))
    assert result.valid is False and result.frames_used == 0
