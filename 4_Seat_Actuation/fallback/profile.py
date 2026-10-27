"""Recorded-participant empirical pressure envelopes, never force calibration."""
from __future__ import annotations

from dataclasses import dataclass, field
import hashlib
import json
import math
from pathlib import Path

DEFAULT_PROFILE = Path(__file__).with_name("profiles") / "bench_20261025.json"
CHANNELS = tuple(f"A{bank}_C{channel}" for bank in range(3) for channel in range(4))


def finite(value, name, lo, hi):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not lo <= value <= hi:
        raise ValueError(f"{name} must be finite and within [{lo}, {hi}]")
    return float(value)


@dataclass(frozen=True)
class Settings:
    # Defaults deliberately contain no guessed physical channel mapping.
    left_channels: tuple[str, ...] = ()
    right_channels: tuple[str, ...] = ()
    guard_channels: tuple[str, ...] = ("A1_C0", "A1_C1", "A1_C2", "A1_C3")
    pressure_fraction: float = .20  # Live pressure guard only; does not scale yaw.
    max_yaw_deg: float = 20.0       # User-selected structure yaw cap, 2026-10-26.
    max_rate_deg_s: float = 2.0
    full_scale_ay_mps2: float = 2.941995  # 0.30 g; a cue reference, not m*a.
    on_ay_mps2: float = .2941995         # 0.03 g
    off_ay_mps2: float = .196133         # 0.02 g
    filter_tau_s: float = .20
    baseline_seconds: float = 2.0
    fresh_ms: float = 200.0
    pressure_trip_seconds: float = .10
    max_baseline_std_v: float = .10
    max_baseline_drift_v: float = .40
    neutral_tolerance_mdeg: int = 100
    confirmations: dict = field(default_factory=dict)

    def __post_init__(self):
        for name in ("left_channels", "right_channels", "guard_channels"):
            values = getattr(self, name)
            if not isinstance(values, (list, tuple)) or any(x not in CHANNELS for x in values) or len(set(values)) != len(values):
                raise ValueError(f"Invalid {name}")
            object.__setattr__(self, name, tuple(values))
        if set(self.left_channels) & set(self.right_channels):
            raise ValueError("A channel cannot represent both physical sides")
        if not set(self.left_channels + self.right_channels) <= set(self.guard_channels):
            raise ValueError("Every side channel must also be guarded")
        for name, lo, hi in (
            ("pressure_fraction", .01, .20), ("max_yaw_deg", .01, 20),
            ("max_rate_deg_s", .01, 2), ("full_scale_ay_mps2", .5, 9.80665),
            ("on_ay_mps2", .01, 1), ("off_ay_mps2", .001, 1),
            ("filter_tau_s", .05, 2), ("baseline_seconds", 2, 30),
            ("fresh_ms", 50, 200), ("pressure_trip_seconds", .02, .2),
            ("max_baseline_std_v", .01, .1), ("max_baseline_drift_v", .01, .4)):
            finite(getattr(self, name), name, lo, hi)
        if not self.off_ay_mps2 < self.on_ay_mps2 < self.full_scale_ay_mps2:
            raise ValueError("Need off < on < full-scale acceleration")
        if type(self.neutral_tolerance_mdeg) is not int or not 0 <= self.neutral_tolerance_mdeg <= 100:
            raise ValueError("Invalid neutral tolerance")
        if not isinstance(self.confirmations, dict) or any(type(x) is not bool for x in self.confirmations.values()):
            raise ValueError("Confirmations must be explicit booleans")

    @classmethod
    def load(cls, path):
        return cls(**json.loads(Path(path).read_text(encoding="utf-8")))

    def missing_confirmations(self):
        names = ("hardware_matches_bench", "sensor_mapping_confirmed",
                 "positive_yaw_supports_occupant", "neutral_start_clear",
                 "same_recorded_participant", "initial_range_checked")
        return [name for name in names if self.confirmations.get(name) is not True]


def interpolate(xs, ys, x):
    """Interpolation inside observations only, including the neutral endpoint."""
    if not math.isfinite(x) or x < xs[0]-1e-9 or x > xs[-1]+1e-9:
        raise ValueError("No extrapolation outside measured yaw")
    for i in range(1, len(xs)):
        if x <= xs[i]:
            t = (x-xs[i-1])/(xs[i]-xs[i-1])
            return ys[i-1] + t*(ys[i]-ys[i-1])
    return ys[-1]


def first_crossing(xs, ys, limit):
    """Largest contiguous-from-neutral interval below a cumulative envelope."""
    if limit < 0:
        return 0.0
    for i in range(1, len(xs)):
        if ys[i] > limit:
            if ys[i] == ys[i-1]:
                return xs[i-1]
            return xs[i-1]+(xs[i]-xs[i-1])*(limit-ys[i-1])/(ys[i]-ys[i-1])
    return xs[-1]


class BenchProfile:
    def __init__(self, path=DEFAULT_PROFILE):
        self.path = Path(path)
        raw = self.path.read_bytes()
        self.sha256 = hashlib.sha256(raw).hexdigest()
        self.data = json.loads(raw)
        p = self.data
        if p.get("schema") != "oasis-bench-pressure-profile-v1" or p.get("sensors") != list(CHANNELS):
            raise ValueError("Unexpected profile schema/channels")
        if not p.get("quality_all_points_ok") or p.get("force_calibrated") is not False:
            raise ValueError("Profile quality or units changed; review required")
        self.base = p["baseline"]["channels"]
        self.points, self.envelopes = {}, {}
        for side in ("left", "right"):
            points = p["branches"][side]["points"]
            xs = [finite(x["yaw_deg"], "yaw", 0, 27) for x in points]
            if xs[0] != 0 or any(b <= a for a, b in zip(xs, xs[1:])):
                raise ValueError("Need strictly increasing yaw from neutral")
            env = {}
            for channel in CHANNELS:
                baseline = finite(self.base[channel]["p95_v"], "baseline p95", 0, 3.3)
                growth, peak = [], 0.0
                for point in points:
                    c = point["channels"][channel]
                    if not point["motion_check"]["passed"] or not point["quality_ok"] or not c["quality_ok"]:
                        raise ValueError("Unqualified measurement in control profile")
                    value = finite(c["p95_v"], "p95", 0, 3.3)
                    finite(c["mean_v"], "mean", 0, 3.3)
                    finite(c["std_v"], "std", 0, 3.3)
                    peak = max(peak, value-baseline)
                    growth.append(peak)
                if growth[0] > 1e-9:
                    raise ValueError("Branch neutral differs from baseline")
                env[channel] = growth
            self.points[side] = (xs, points)
            self.envelopes[side] = env

    def derive(self, settings: Settings):
        return CalibratedEnvelope(self, settings)


class CalibratedEnvelope:
    """Configured yaw bounds and separate empirical live pressure guards.

    Historical pressure response is not a reproducible inverse plant model.
    Its envelope supplies pressure-increment budgets, never a static yaw cap.
    Position commands use the requested angle cap inside the measured domain.
    """
    def __init__(self, profile, settings):
        self.profile, self.settings = profile, settings
        self.max_yaw, self.side_span, self.side_curves = {}, {}, {}
        self.direction_evidence = {}
        self.channel_caps_v, self.noise_v = {}, {}
        for channel in settings.guard_channels:
            span = max(profile.envelopes[s][channel][-1] for s in ("left", "right"))
            self.channel_caps_v[channel] = settings.pressure_fraction*span
            self.noise_v[channel] = max(.01, 3*profile.base[channel]["std_v"])
        self.reasons = []
        self.advisories = []
        for side in ("left", "right"):
            channels = getattr(settings, side+"_channels")
            xs, points = profile.points[side]
            limit = min(settings.max_yaw_deg, xs[-1])
            if not channels:
                self.reasons.append(side+":physical_channel_mapping_missing")
                curve = [0.0]*len(xs)
            else:
                # Direction evidence must use simultaneous SIGNED means. A mean
                # of channel-wise historical p95 maxima is only a guard: peaks
                # can occur at different angles while total side load decreases.
                base_mean = sum(profile.base[c]["mean_v"] for c in channels)/len(channels)
                signed = [sum(point["channels"][c]["mean_v"] for c in channels)/len(channels)-base_mean
                          for point in points]
                noise = sum(profile.base[c]["std_v"] for c in channels)/len(channels)
                prefix = [i for i in range(1, len(xs)) if xs[i-1] < limit]
                credible = bool(prefix) and signed[prefix[0]] > 3*noise
                credible = credible and all(signed[i] >= signed[i-1]-noise for i in prefix)
                self.direction_evidence[side] = dict(signed_side_mean_delta_v=signed,
                    baseline_mean_std_v=noise, initial_positive_response_resolved=credible)
                # This remains a diagnostic only. The user confirmed +yaw moves
                # BOTH pads toward the body; pressure observations do not identify
                # a reproducible inverse plant model and are never inverted.
                curve, peak = [], 0.0
                for value in signed:
                    peak = max(peak, value)
                    curve.append(peak)
                if not credible:
                    self.advisories.append(side+":pressure_gain_not_identified_use_position_cue_only")
            self.max_yaw[side] = max(0.0, limit)
            self.side_curves[side] = curve
            self.side_span[side] = curve[-1]
            if limit < .01:
                self.reasons.append(side+":no_usable_initial_range")

    def angle_for_demand(self, side, demand):
        demand = finite(demand, "demand", 0, 1)
        if demand == 0 or self.reasons:
            return 0.0
        # Position feedforward within the explicit configured yaw cap.
        # No PI, no pressure inverse, no predicted pressure/force cancellation.
        return demand*self.max_yaw[side]

    def report(self):
        return dict(dataset_id=self.profile.data["dataset_id"], profile_sha256=self.profile.sha256,
                    measured_yaw_range_deg=[0, 27], initial_yaw_caps_deg=self.max_yaw,
                    configured_max_yaw_deg=self.settings.max_yaw_deg,
                    yaw_limit_basis="operator_configured_within_measured_range",
                    pressure_fraction_limits_yaw=False,
                    pressure_fraction=self.settings.pressure_fraction, channel_increment_caps_v=self.channel_caps_v,
                    noise_allowance_v=self.noise_v, configuration_gaps=self.reasons,
                    direction_evidence=self.direction_evidence,
                    advisories=self.advisories, policy="position_cue_with_pressure_increment_guards",
                    missing_confirmations=self.settings.missing_confirmations(),
                    pressure_unit="estimated volts, not N or Pa", hardware_verified=False)
