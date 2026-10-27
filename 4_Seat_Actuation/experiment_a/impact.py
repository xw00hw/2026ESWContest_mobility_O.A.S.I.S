"""Causal planar shock heuristic, not a measured collision/contact signal.

Use EVERY original qualified vehicle sample. Simulation time measures physical
duration and the integral; host time is checked elsewhere for freshness. The
vector integral uses local vehicle axes over a short continuous high segment;
it is an acceleration-derived proxy, not independent measured delta velocity.
An isolated moderate peak does not latch. A very large peak has a separate
immediate guard. No pressure or motor position enters this detector.
"""
from dataclasses import asdict, dataclass
import math

# Broad input-integrity limit, separate from motion saturation and shock policy.
# The collector also rejects derivative magnitudes beyond 200 m/s².
MAX_ABS_ACCEL_MPS2 = 200.0


@dataclass(frozen=True)
class ImpactState:
    triggered: bool = False
    reason: str = 'no_impact_evidence'
    ax_mps2: float = 0.0
    ay_mps2: float = 0.0
    planar_mps2: float = 0.0
    jerk_mps3: float = 0.0
    high_duration_s: float = 0.0
    vector_delta_v_mps: float = 0.0
    abrupt_onset: bool = False
    sequence_id: int | None = None
    sim_time_s: float | None = None

    def to_dict(self):
        return asdict(self)


class ImpactDetector:
    """Latched evidence from consecutive, unique, qualified source samples."""
    def __init__(self, settings):
        self.settings = settings
        self.state = ImpactState()
        self._last = None
        self._clear_segment()

    def _clear_segment(self):
        self._high_since = None
        self._impulse_x = self._impulse_y = 0.0
        self._abrupt = False

    def observe(self, vehicle):
        if self.state.triggered:
            return self.state
        if vehicle is None or not (vehicle.valid or vehicle.reason == 'stopped'):
            self._last = None
            self._clear_segment()
            self.state = ImpactState(reason='unqualified_sample')
            return self.state
        ax, ay, sim = vehicle.ax_mps2, vehicle.ay_mps2, vehicle.sim_time_s
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or
               not math.isfinite(v) for v in (ax, ay, sim)):
            raise ValueError('impact_input_invalid')
        if max(abs(ax), abs(ay)) > MAX_ABS_ACCEL_MPS2 or sim < 0:
            raise ValueError('impact_input_outside_integrity_bounds')
        key = (vehicle.stream_id, vehicle.vehicle_id, vehicle.sequence_id,
               vehicle.received_monotonic_ns, sim)
        old = self._last
        if old is not None and key == old[0]:
            return self.state  # Holding the same sample never earns duration.
        dt = 0.0
        if old is not None:
            prior = old[0]
            continuous = (key[:2] == prior[:2] and key[2] == prior[2]+1 and
                          key[3] > prior[3] and
                          0 < sim-prior[4] <= self.settings.max_model_sample_dt_s+1e-12)
            if continuous:
                dt = sim-prior[4]
            else:
                self._clear_segment()
        magnitude = math.hypot(ax, ay)
        jerk = math.hypot(ax-old[1], ay-old[2])/dt if dt else 0.0
        high = magnitude >= self.settings.impact_planar_threshold_mps2
        if high:
            if self._high_since is None:
                self._high_since = sim
                self._impulse_x = self._impulse_y = 0.0
                self._abrupt = False
            elif dt:
                # Only intervals with two high endpoints contribute. Do not
                # backdate a new peak across the preceding low sample.
                self._impulse_x += .5*(old[1]+ax)*dt
                self._impulse_y += .5*(old[2]+ay)*dt
            if not self._abrupt and jerk >= self.settings.impact_jerk_threshold_mps3:
                # A long, steady high-g corner before a later abrupt change
                # must not supply pre-onset duration/impulse to that change.
                self._high_since = sim
                self._impulse_x = self._impulse_y = 0.0
                self._abrupt = True
        else:
            self._clear_segment()
        duration = sim-self._high_since if self._high_since is not None else 0.0
        delta_v = math.hypot(self._impulse_x, self._impulse_y)
        extreme = magnitude >= self.settings.impact_extreme_planar_mps2
        sustained = (high and self._abrupt and
                     duration+1e-12 >= self.settings.impact_min_duration_s and
                     delta_v+1e-12 >= self.settings.impact_min_delta_v_mps)
        reason = ('impact_suspected_extreme' if extreme else
                  'impact_suspected_sustained' if sustained else
                  'high_acceleration_monitoring' if high else 'no_impact_evidence')
        self.state = ImpactState(extreme or sustained, reason, ax, ay, magnitude,
                                 jerk, duration, delta_v, self._abrupt, key[2], sim)
        self._last = (key, ax, ay)
        return self.state
