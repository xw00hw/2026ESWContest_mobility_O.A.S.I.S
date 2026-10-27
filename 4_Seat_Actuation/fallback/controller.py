"""Pure deterministic controller. No threads, serial, network, camera or RL.

Positive vehicle lateral acceleration points left. Relative occupant loading
is toward the right, so the right bolster supports leftward (and vice versa).
This is a scaled stationary-rig pressure cue, not cancellation of vehicle ay.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import statistics

from .profile import CalibratedEnvelope, Settings


@dataclass(frozen=True)
class Decision:
    mode: str
    reason: str
    pose_mdeg: tuple[int, int, int, int] = (0, 0, 0, 0)
    active_side: str = "none"
    filtered_ay_mps2: float = 0.0
    demand: float = 0.0
    permit_lease: bool = False
    emergency_off: bool = False


class LateralDemand:
    """Low-pass and Schmitt trigger; the source's original age is preserved."""
    def __init__(self, settings):
        self.settings = settings
        self.filtered = 0.0
        self.side = "none"
        self.last_ns = None

    def reset(self):
        self.filtered, self.side, self.last_ns = 0.0, "none", None

    def update(self, ay, now_ns):
        if not math.isfinite(ay):
            raise ValueError("Nonfinite acceleration")
        dt = 0 if self.last_ns is None else (now_ns-self.last_ns)/1e9
        if dt < 0:
            raise ValueError("Clock moved backwards")
        self.last_ns = now_ns
        # No catch-up jump after a stalled host loop.
        dt = min(dt, .05)
        self.filtered += (1-math.exp(-dt/self.settings.filter_tau_s))*(ay-self.filtered)
        a = abs(self.filtered)
        desired = "right" if self.filtered > 0 else "left"
        if a <= self.settings.off_ay_mps2:
            self.side = "none"
        elif a >= self.settings.on_ay_mps2:
            self.side = desired
        elif self.side != desired:
            # Hysteresis must not retain the old side across a sign change.
            self.side = "none"
        if self.side == "none":
            return "none", 0.0
        demand = min(1.0, max(0.0, (a-self.settings.off_ay_mps2)/
                          (self.settings.full_scale_ay_mps2-self.settings.off_ay_mps2)))
        return self.side, demand


class FallbackController:
    """Explicit ARM happens outside; this controller owns the continuing lease.

    The first armed state must be neutral. Baseline capture requires two seconds
    of fresh, stable ADC frames at commanded neutral. There is no automatic
    recovery from FAULT; a new controller/session and deliberate ARM are needed.
    """
    def __init__(self, calibration: CalibratedEnvelope):
        self.calibration = calibration
        self.settings: Settings = calibration.settings
        self.demand_filter = LateralDemand(self.settings)
        self.mode, self.fault = "BASELINE", None
        self.baseline = None
        self.samples = []
        self.last_sample_ns = self.last_tick_ns = None
        self.pose = [0.0]*4
        self.active_side = "none"
        self.reversing = False
        self.over_since = self.low_since = None

    def fail(self, reason):
        self.fault, self.mode = reason, "FAULT"
        self.demand_filter.reset()
        return Decision("FAULT", reason, emergency_off=True)

    def _fresh(self, timestamp, now):
        return type(timestamp) is int and 0 <= now-timestamp <= self.settings.fresh_ms*1e6

    def _neutral(self, state):
        tol = self.settings.neutral_tolerance_mdeg
        return not state.moving and all(abs(x) <= tol for x in state.pose_mdeg) and all(x == 0 for x in state.target_mdeg)

    def tick(self, vehicle, pressure, state, now_ns):
        if self.fault:
            return Decision("FAULT", self.fault, emergency_off=True)
        if self.calibration.reasons:
            return self.fail("calibration:"+";".join(self.calibration.reasons))
        if self.last_tick_ns is not None and not 0 <= now_ns-self.last_tick_ns <= 200_000_000:
            return self.fail("host_control_clock_gap")
        dt = 0 if self.last_tick_ns is None else min(.05, (now_ns-self.last_tick_ns)/1e9)
        self.last_tick_ns = now_ns
        if state is None or not self._fresh(state.received_ns, now_ns):
            return self.fail("board_state_stale")
        if not state.armed or state.fault != "NONE":
            return self.fail("board_disarmed_or_fault")
        if len(state.pose_mdeg) != 4 or len(state.target_mdeg) != 4:
            return self.fail("board_pose_shape")
        if state.pose_mdeg[1] or state.pose_mdeg[3] or state.target_mdeg[1] or state.target_mdeg[3]:
            return self.fail("unexpected_pitch")
        for index, side in ((0, "left"), (2, "right")):
            upper = math.ceil(self.calibration.max_yaw[side]*1000)+2
            if not 0 <= state.pose_mdeg[index] <= upper or not 0 <= state.target_mdeg[index] <= upper:
                return self.fail("board_outside_profile_range")
        if pressure is None or not pressure.valid or not self._fresh(pressure.received_ns, now_ns):
            return self.fail("pressure_invalid_or_stale")
        for c in self.settings.guard_channels:
            v = pressure.voltages.get(c)
            if v is None or not math.isfinite(v) or not 0 <= v < 3.25:
                return self.fail("pressure_missing_or_saturated:"+c)
        if vehicle is None or not self._fresh(vehicle.received_monotonic_ns, now_ns):
            return self.fail("vehicle_stale")
        if not vehicle.valid and vehicle.reason != "stopped":
            return self.fail("vehicle:"+vehicle.reason)
        if self.mode == "BASELINE":
            if not self._neutral(state):
                return self.fail("startup_requires_commanded_neutral")
            if pressure.received_ns != self.last_sample_ns:
                self.last_sample_ns = pressure.received_ns
                self.samples.append((pressure.received_ns, dict(pressure.voltages)))
            if not self.samples or self.samples[-1][0]-self.samples[0][0] < self.settings.baseline_seconds*1e9:
                return Decision("BASELINE", "collecting_fresh_neutral_pressure", permit_lease=True)
            # Require actual new sweeps throughout the interval, not a repeatedly
            # refreshed cached ADC value. The parser additionally checks MCU time.
            if len(self.samples) < self.settings.baseline_seconds*20:
                return self.fail("baseline_insufficient_samples")
            base = {}
            for c in self.settings.guard_channels:
                readings = [x[1][c] for x in self.samples]
                mean, std = statistics.fmean(readings), statistics.pstdev(readings)
                if mean < .02:
                    return self.fail("baseline_no_contact:"+c)
                if std > self.settings.max_baseline_std_v:
                    return self.fail("baseline_unstable:"+c)
                if abs(mean-self.calibration.profile.base[c]["mean_v"]) > self.settings.max_baseline_drift_v:
                    return self.fail("baseline_differs_from_recorded_participant:"+c)
                base[c] = mean
            self.baseline = base
            self.samples.clear()
            self.mode = "RUN"
            self.demand_filter.reset()
            return Decision("RUN", "neutral_baseline_acquired", permit_lease=True)

        increments = {c: pressure.voltages[c]-self.baseline[c] for c in self.settings.guard_channels}
        over = any(increments[c] > self.calibration.channel_caps_v[c]+self.calibration.noise_v[c]
                   for c in increments)
        severe = any(increments[c] > self.calibration.channel_caps_v[c]+2*self.calibration.noise_v[c]
                     for c in increments)
        if severe:
            return self.fail("pressure_hard_increment_limit")
        self.over_since = (self.over_since if self.over_since is not None else now_ns) if over else None
        if over and now_ns-self.over_since >= self.settings.pressure_trip_seconds*1e9:
            return self.fail("pressure_increment_limit_persisted")
        lost_contact = sum(pressure.voltages[c] for c in increments) < .5*sum(self.baseline.values())
        self.low_since = (self.low_since if self.low_since is not None else now_ns) if lost_contact else None
        if lost_contact and now_ns-self.low_since >= 200_000_000:
            return self.fail("pressure_contact_changed")

        if vehicle.reason == "stopped":
            self.demand_filter.reset()
            side, demand = "none", 0.0
        else:
            side, demand = self.demand_filter.update(vehicle.ay_mps2, now_ns)
        if over or lost_contact:
            side, demand = "none", 0.0
        if self.active_side != "none" and side != self.active_side:
            self.reversing = True
        if self.reversing:
            # Commanded state completion is not an encoder measurement, but
            # prevents opposite targets while the firmware is still ramping.
            if self._neutral(state) and max(abs(x) for x in self.pose) <= 1e-9:
                self.active_side, self.reversing = "none", False
            else:
                side, demand = "none", 0.0
        if not self.reversing:
            self.active_side = side
        desired = [0.0]*4
        if side != "none":
            desired[0 if side == "left" else 2] = self.calibration.angle_for_demand(side, demand)*1000
        step = self.settings.max_rate_deg_s*1000*dt
        for i in (0, 2):
            delta = desired[i]-self.pose[i]
            self.pose[i] += max(-step, min(step, delta))
        # Round downward so quantization cannot exceed the configured angle cap.
        pose = tuple(int(max(0, x)) for x in self.pose)
        reason = "returning_neutral_before_side_change" if self.reversing else "pressure_limited_feedforward"
        if over or lost_contact:
            reason = "pressure_guard_returning_neutral"
        return Decision("RUN", reason, pose, self.active_side,
                        self.demand_filter.filtered, demand, True)
