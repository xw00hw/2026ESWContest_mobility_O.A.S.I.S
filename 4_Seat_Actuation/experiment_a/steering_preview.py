"""Small, time-bounded steering advance cue for the declared stock vehicle.

This maps steering-wheel magnitude to at most 2 degrees of bolster Yaw. It
does not estimate road-wheel angle, lateral acceleration, force or body lean.
No empirical steering gain from another vehicle/configuration is transferred.
The owner calls step exactly once per validated source sample, using the
same-PC receive monotonic interval for the dwell and real-time episode limit;
normal command rates, CAD limits, reversal passage and leases remain external.
"""
from dataclasses import asdict, dataclass
import math


def _finite(value):
    return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)


def _clamp(value, low, high):
    return max(low, min(high, value))


@dataclass(frozen=True)
class SteeringPreviewState:
    active: bool = False
    reason: str = 'not_started'
    side: str = 'none'
    yaw_deg: float = 0.0
    pitch_deg: float = 0.0
    steering_fraction: float = 0.0
    speed_fraction: float = 0.0
    projected_speed_mps: float = 0.0
    brake_factor: float = 1.0
    dwell_elapsed_s: float = 0.0
    episode_age_s: float = 0.0
    throttle_used: bool = False

    def to_dict(self):
        return asdict(self)


class SteeringPreview:
    def __init__(self, settings):
        self.settings = settings
        self.reset()

    def reset(self):
        self._time_s = 0.0
        self._candidate_side = 'none'
        self._candidate_since_s = None
        self._episode_start_s = None
        self.state = SteeringPreviewState()
        return self.state

    def _result(self, reason, **values):
        self.state = SteeringPreviewState(reason=reason, **values)
        return self.state

    def _ineligible(self, reason):
        # A lost qualifier may restart dwell, but never renews an episode.
        self._candidate_side = 'none'
        self._candidate_since_s = None
        age = 0.0 if self._episode_start_s is None else self._time_s-self._episode_start_s
        return self._result(reason, episode_age_s=age)

    def step(self, vehicle, dt_s, *, raw_ay_mps2, filtered_ay_mps2, primary_active=False):
        s = self.settings
        if not _finite(dt_s) or not 0 <= dt_s <= s.fresh_ms/1000+1e-10:
            raise ValueError('invalid_steering_preview_sample_dt')
        self._time_s += dt_s
        if not s.steering_preview_enabled or not s.steering_preview_max_yaw_deg:
            return self._ineligible('disabled')
        speed = getattr(vehicle, 'speed_mps', None)
        steering = getattr(vehicle, 'steering_deg', None)
        if vehicle.reason == 'stopped':
            self.reset()
            return self._result('stopped_reset')
        if not _finite(steering) or abs(steering) > s.steering_preview_max_abs_deg:
            return self._ineligible('steering_missing_or_out_of_range')
        if abs(steering) <= s.steering_preview_deadband_deg:
            self.reset()
            return self._result('steering_centered_reset')
        if not _finite(speed) or speed < s.steering_preview_min_speed_mps:
            return self._ineligible('speed_below_preview_minimum')
        gear = getattr(vehicle, 'gear_index', None)
        if not _finite(gear) or gear <= 0 or not float(gear).is_integer():
            return self._ineligible('forward_gear_not_verified')
        roll, pitch = getattr(vehicle, 'roll_rad', None), getattr(vehicle, 'pitch_rad', None)
        if any(not _finite(x) or abs(x) >= math.radians(20) for x in (roll, pitch)):
            return self._ineligible('orientation_missing_or_excessive')
        brake = getattr(vehicle, 'brake', None)
        if not _finite(brake) or not 0 <= brake <= 1:
            return self._ineligible('brake_missing_or_out_of_range')
        # Velocity projection uses gravity-free longitudinal acceleration even
        # if the primary controller was configured to use specific force.
        ax = getattr(vehicle, 'ax_mps2', None)
        if not _finite(ax) or abs(ax) > 15:
            return self._ineligible('linear_longitudinal_acceleration_invalid')
        if any(not _finite(x) for x in (raw_ay_mps2, filtered_ay_mps2)):
            raise ValueError('invalid_steering_preview_lateral_acceleration')

        side = 'right' if steering*s.steering_to_ay_sign > 0 else 'left'
        if side != self._candidate_side:
            self._candidate_side = side
            self._candidate_since_s = self._time_s
        dwell = self._time_s-self._candidate_since_s
        if dwell+1e-10 < s.steering_preview_dwell_s:
            return self._result('steering_dwell', side=side, dwell_elapsed_s=dwell)
        if self._episode_start_s is None:
            self._episode_start_s = self._time_s
        age = self._time_s-self._episode_start_s
        common = dict(side=side, dwell_elapsed_s=dwell, episode_age_s=age)
        if age+1e-10 >= s.steering_preview_timeout_s:
            return self._result('episode_expired_recenter_required', **common)
        # Actual ay always takes precedence, including raw input that the
        # response filter has not yet reached. Do not renew this episode.
        if primary_active or max(abs(raw_ay_mps2), abs(filtered_ay_mps2)) >= s.on_ay_mps2:
            return self._result('actual_lateral_acceleration_priority', **common)
        side_sign = 1 if side == 'right' else -1
        if any(value*side_sign <= -s.off_ay_mps2 for value in (raw_ay_mps2, filtered_ay_mps2)):
            return self._result('opposite_lateral_acceleration', **common)
        if s.available_bolsters == 'right_only' and side == 'left':
            return self._result('left_bolster_unavailable', **common)

        projected_speed = _clamp(max(0.0, speed+ax*s.steering_preview_speed_lookahead_s), .8*speed, 1.1*speed)
        speed_fraction = _clamp((projected_speed**2-s.steering_preview_min_speed_mps**2)/
                               (s.steering_preview_full_speed_mps**2-s.steering_preview_min_speed_mps**2), 0.0, 1.0)
        steering_fraction = _clamp((abs(steering)-s.steering_preview_deadband_deg)/
                                  (s.steering_preview_reference_deg-s.steering_preview_deadband_deg), 0.0, 1.0)
        brake_factor = 1.0-.5*brake
        yaw = min(s.steering_preview_max_yaw_deg, s.max_yaw_deg)*steering_fraction*speed_fraction*brake_factor
        return self._result('steering_advance' if yaw > 0 else 'zero_preview_demand',
            active=yaw > 0, yaw_deg=yaw, steering_fraction=steering_fraction,
            speed_fraction=speed_fraction, projected_speed_mps=projected_speed,
            brake_factor=brake_factor, **common)
