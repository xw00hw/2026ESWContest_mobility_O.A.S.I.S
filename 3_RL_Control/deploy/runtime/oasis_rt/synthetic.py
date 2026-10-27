"""Synthetic BeamNG collector rows for SIL and network commissioning without the game.

Rows follow the BeamNG Collector v0.3 / revision 2 columns that BeamNG_Bolster_FB validates
(fallback/beamng.py REQUIRED_COLUMNS + steering/brake/gear/orientation). They are labelled
source_kind = 'synthetic_test', so the Pi accepts them only with beamng.allow_synthetic: true.
Vehicle frame +x forward, +y LEFT; +steering = left turn = +a_y (the runtime's steer_sign 1).

  drive(t)            : handwheel angle [deg] of the test route (straight / left / right / slalom)
  SyntheticCar        : steady-state bicycle a_y with a first-order yaw lag (nominal car, not BeamNG)
  SyntheticReceiver   : row source on a (simulated) clock, same receive() contract as beamng_rx.RelayReceiver
"""
import math

G = 9.81


def drive(t, period=40.0):
    """Handwheel [deg]: 4 s straight, left 60 deg (1 s ramps, 4 s hold), 4 s straight, right 60 deg, slalom 45 deg."""
    u = t % period
    ramp = lambda a, b: min(max((u - a) / (b - a), 0.0), 1.0)
    if u < 4: return 0.0
    if u < 10: return 60.0 * (ramp(4, 5) - ramp(9, 10))
    if u < 14: return 0.0
    if u < 20: return -60.0 * (ramp(14, 15) - ramp(19, 20))
    if u < 24: return 0.0
    if u < 36: return 45.0 * math.sin(2 * math.pi * 0.3 * (u - 24))
    return 0.0


class SyntheticCar:
    def __init__(self, speed_mps=14.0, wheelbase_m=2.7, steer_ratio=15.0, understeer_K=0.0012, tau_s=0.15):
        self.v, self.L, self.ratio, self.K, self.tau = speed_mps, wheelbase_m, steer_ratio, understeer_K, tau_s
        self.ay = 0.0

    def step(self, sw_deg, dt):
        v = self.v
        a_ss = v * v * math.radians(sw_deg) / self.ratio / (self.L * (1 + self.K * v * v))
        self.ay += (a_ss - self.ay) * (1 - math.exp(-dt / self.tau))
        return self.ay


def row(seq, sim_t, dt, ay, steer_deg, speed=14.0, stream='synthetic-1', vehicle='1', **over):
    r = dict(host_monotonic_ns=0, sim_time_s=sim_t, stream_id=stream, vehicle_id=vehicle, sequence_id=seq,
             source_kind='synthetic_test', linear_acc_y_mps2=ay, linear_acc_x_mps2=0.0, speed_mps=speed,
             derivative_valid=1, derivative_reason='ok', config_match=1, sample_dt_s=dt, missing_packets_before=0,
             steering_angle_deg=steer_deg, throttle_input=0.2, brake_input=0.0, gear_index=3,
             roll_rad=-0.006 * ay, pitch_rad=0.0)
    r.update(over)
    return r


class SyntheticReceiver:
    """Collector rows at `rate_hz` on `clock_ns` (simulated or real), as read_bridge_packet would return them
    (`_bridge_session_id`, `_bridge_config_sha256` attached). Fault injection for tests:
      pause_at=(t0, t1)  : no rows in [t0, t1) (BeamNG menu / lost link)
      bad_at=t           : one row with derivative_valid 0 at t
      shock_at=(t0, t1)  : a_y +30, a_x -10 m/s^2 in [t0, t1) (impact-like planar shock)
      steer_sign=-1      : steering column with the wrong sign (sign-check test)"""
    def __init__(self, clock_ns, rate_hz=50.0, route=drive, car=None, pause_at=None, bad_at=None, steer_sign=1, speed=14.0,
                 shock_at=None):
        self.clock_ns, self.dt, self.route = clock_ns, 1.0 / rate_hz, route
        self.car = car or SyntheticCar(speed_mps=speed); self.speed = speed
        self.t0 = clock_ns(); self.k = 0; self.pause, self.bad, self.sign, self.shock = pause_at, bad_at, steer_sign, shock_at
        self.sent = 0

    def receive(self):
        now = self.clock_ns()
        due = self.t0 + int(self.k * self.dt * 1e9)
        if due > now: return None
        t = self.k * self.dt; self.k += 1
        sw = self.route(t); ay = self.car.step(sw, self.dt)
        if self.pause and self.pause[0] <= t < self.pause[1]: return None
        over = dict(derivative_valid=0, derivative_reason='invalid_test') if self.bad is not None and abs(t - self.bad) < self.dt / 2 else {}
        if self.shock and self.shock[0] <= t < self.shock[1]: over.update(linear_acc_y_mps2=30.0, linear_acc_x_mps2=-10.0)
        r = row(self.k, t, self.dt, ay, self.sign * sw, speed=self.speed, **over)
        r.update(host_monotonic_ns=due, _bridge_session_id='synthetic', _bridge_config_sha256='0' * 64)
        self.sent += 1
        return r

    def close(self): pass
