"""a_y / a_x look-ahead for observation slots 0-19 (10 x 0.1 s ... 1.0 s ahead, [g]).

The policy was trained with a STAND-IN predictor (sim/sensors.AccelPredictor: true future x gain
0.8-1.1, lag 0-0.15 s, horizon-growing error 0.005-0.02 g + 0.01-0.06 g/s). Any predictor used
here must be at least that good, or the feed-forward prior (which reads slot ff_idx = 0.7 s ahead)
acts on noise. Check with `quality()` on a recorded drive before driving with it.

  SequencePredictor  : known future (BeamNG demo / replay of a recorded drive) -- what the sim assumed
  KinematicPredictor : real car baseline from OBD speed + steering wheel angle (bicycle model,
                       steering-rate extrapolation with decay). Replace with the L1-trained model.
  HoldPredictor      : current a_y held over the horizon (no preview; degrades to reactive)
  WheelPreviewPredictor : BeamNG -- raw wheel input read before the car reacts -> genuine 0.1-0.3 s preview
  BeamNGPredictor    : BeamNG final (v7) -- wheel preview + road curvature ahead (BeamNGpy road network, bridge
                       `road_ay_g`) + state -> small MLP trained in sim/v3 (sim/v3/final/train_predictor.py)

real-vehicle drive (sim/v2/l1_refined_seq.csv, 1165 s): fitted steer ratio 13.5, K 0.0016 s^2/m^2, but
the logged steering angle is ~0.38 s LATE relative to the IMU a_y, so the kinematic preview is
worse than Hold at every horizon (RMS 0.033-0.050 g vs 0.023-0.039 g). Car default = hold until
the steering latency is fixed (CAN instead of OBD polling) or the L1 predictor exists; the
BeamNG demo uses SequencePredictor / the simulator's route preview.
"""
import json
import numpy as np

G = 9.81
H = np.arange(1, 11) * 0.1


class WheelPreviewPredictor:
    """BeamNG.drive: the raw steering-wheel input (wheel device / BeamNG electrics.steering_input) is read BEFORE
    the car reacts, so it previews a_y by the vehicle's yaw-response lag, and its RATE shows where the driver is
    still winding to:
        sw(h)  = sw + rate * Ts (1 - exp(-h / Ts))           driver intent: the winding decays with Ts (0 = hold)
        a_ss   = v^2 (sw / ratio) / (L (1 + K v^2))          where the car's a_y is heading (steady state)
        da/dt  = (a_ss(sw(h)) - a) / tau                     the car follows with its yaw lag tau, from a_now
    Same class in the simulator (env_v3, v3.predictor.kind = wheel) with nominal vehicle parameters, so its model
    error vs the episode's car is part of the training."""
    def __init__(self, wheelbase_m=2.85, steer_ratio=13.5, understeer_K=0.006, tau_s=0.15, ay_max_g=1.0,
                 steer_tau_s=0.0, rate_lp_s=0.1, dt_int=0.02):
        self.L, self.ratio, self.K, self.tau, self.amax = wheelbase_m, steer_ratio, understeer_K, tau_s, ay_max_g * G
        self.Ts, self.lp, self.dti = steer_tau_s, rate_lp_s, dt_int
        self.hi = np.arange(1, int(round(H[-1] / dt_int)) + 1) * dt_int
        self.pick = np.round(H / dt_int).astype(int) - 1
        self.prev = None; self.rate = 0.0
    def steady_ay(self, sw, v):
        v = max(v, 0.0)
        return np.clip(v ** 2 * (np.asarray(sw) / self.ratio) / (self.L * (1 + self.K * v ** 2)), -self.amax, self.amax)
    def __call__(self, s):
        sw, v, t = s.get('steer_input_rad', s['steer_rad']), s['speed_mps'], s.get('t')
        if self.Ts > 0 and t is not None:
            if self.prev is not None and t > self.prev[0]:
                r = (sw - self.prev[1]) / (t - self.prev[0]); a = min(1.0, (t - self.prev[0]) / self.lp)
                self.rate += a * (r - self.rate)
            self.prev = (t, sw)
            a_ss = self.steady_ay(sw + self.rate * self.Ts * (1 - np.exp(-self.hi / self.Ts)), v)
        else:
            a_ss = np.full(len(self.hi), self.steady_ay(sw, v))
        a, k, ay = s["ay_mps2"], 1 - np.exp(-self.dti / self.tau), np.empty(len(self.hi))
        for i, x in enumerate(a_ss):
            a += (x - a) * k; ay[i] = a
        return (ay[self.pick] / G).astype(np.float32), np.full(10, s['ax_mps2'] / G, np.float32)


class BeamNGPredictor:
    """Learned 1 s a_y look-ahead for BeamNG (model v7). Features (all available on the bench at 20 Hz):
      wheel prior a_y(h) (WheelPreviewPredictor, nominal car), road a_y(h) = v^2 kappa_road(s + v h) / g from the
      bridge (0 and road_valid 0 when BeamNG gives no road network, e.g. BeamNG.drive), current a_y, a_x, yaw rate,
      speed, wheel angle and rate, a_y 0.1-0.4 s ago.
    Output = wheel prior + MLP residual (so with weights = None it IS the wheel prior). Same class in the simulator
    (env_v3, v3.predictor.kind = beamng) and on the Pi; weights + normaliser + wheel parameters in one npz."""
    NF = 31
    def __init__(self, npz=None, **wheel):
        self.W = None
        if npz is not None:
            z = np.load(npz, allow_pickle=False)
            self.W = [(z[f'W{i}'], z[f'b{i}']) for i in range(3)]; self.mu, self.sd = z['x_mean'], z['x_std']
            wheel = dict(json.loads(str(z['wheel_params'])), **wheel)
        self.wheel = WheelPreviewPredictor(**wheel); self.hist = np.zeros(8)
    def features(self, s):
        prior, ax = self.wheel(s)
        valid = float(s.get('road_valid', 0.0)); road = np.asarray(s.get('road_ay_g') if valid else np.zeros(10), float)
        ay = s['ay_mps2'] / G; sw = s.get('steer_input_rad', s['steer_rad'])
        x = np.r_[prior, road, valid, ay, s['ax_mps2'] / G, s['yaw_rate_radps'], s['speed_mps'] / 20.0, sw / 3.0,
                  self.wheel.rate / 3.0, self.hist[[1, 3, 5, 7]]].astype(np.float32)
        self.hist = np.r_[ay, self.hist[:-1]]                       # 20 Hz calls: [1, 3, 5, 7] = 0.1 ... 0.4 s ago
        return x, prior, ax
    def __call__(self, s):
        x, prior, ax = self.features(s); self.x = x                 # last features (sim/v3/final/train_predictor.py logs them)
        if self.W is None: return prior, ax
        h = (x - self.mu) / self.sd
        for i, (W, b) in enumerate(self.W):
            h = h @ W + b
            if i < 2: h = np.maximum(h, 0.0)
        return (prior + h).astype(np.float32), ax


class HoldPredictor:
    def __call__(self, s):
        return np.full(10, s['ay_mps2'] / G, np.float32), np.full(10, s['ax_mps2'] / G, np.float32)


class MozaPriorityPredictor:
    """Prefer calibrated, fresh direct wheel intent; keep the original predictor warm.

    This substitutes a wheel-response model for the learned preview only while
    VehicleInput supplies verified MOZA input. Measured BeamNG acceleration,
    speed, and measured steering are never replaced. It does not establish that
    the wheel stream leads the game or that this predictor improves accuracy.
    """
    def __init__(self, fallback):
        self.fallback = fallback
        reference = getattr(fallback, 'wheel', fallback)
        if not isinstance(reference, WheelPreviewPredictor):
            raise ValueError('MOZA priority requires a wheel/BeamNG predictor with vehicle parameters')
        self.params = dict(wheelbase_m=reference.L, steer_ratio=reference.ratio,
                           understeer_K=reference.K, tau_s=reference.tau,
                           ay_max_g=reference.amax/G, steer_tau_s=reference.Ts,
                           rate_lp_s=reference.lp, dt_int=reference.dti)
        self.wheel = WheelPreviewPredictor(**self.params)
        self.selected = 'beamng_predictor'
        self.last_query = self.stream = None
        self.counts = dict(moza_wheel=0, beamng_predictor=0)
        self.switches = 0

    def __call__(self, s):
        measured = dict(s, steer_input_rad=s['steer_rad'])
        baseline = self.fallback(measured)
        query, stream = s.get('_moza_query_ns'), s.get('_moza_stream_id')
        angle = s.get('steer_input_rad')
        usable = (s.get('_moza_valid') is True and type(query) is int and query >= 0
                  and isinstance(stream, str) and bool(stream)
                  and isinstance(angle, (int, float)) and not isinstance(angle, bool)
                  and np.isfinite(angle) and abs(angle) <= 8*np.pi)
        selected = 'moza_wheel' if usable else 'beamng_predictor'
        if selected != self.selected:
            self.switches += 1
            self.wheel = WheelPreviewPredictor(**self.params)
        if usable:
            if stream != self.stream or (self.last_query is not None and query < self.last_query):
                self.wheel = WheelPreviewPredictor(**self.params)
            # Repeated host queries are normal when the control rate is higher
            # than the wheel rate. They must not create a synthetic derivative.
            result = self.wheel(dict(s, t=query/1e9))
            self.last_query, self.stream = query, stream
        else:
            result = baseline
            self.last_query = self.stream = None
        self.selected = selected
        self.counts[selected] += 1
        return result

    def summary(self):
        return dict(kind='moza_priority', selected=self.selected, counts=dict(self.counts),
                    switches=self.switches, wheel_params=dict(self.params),
                    latency_advantage_measured=False, accuracy_improvement_verified=False)


class SequencePredictor:
    """Future from a time series (t, ax, ay) aligned to the vehicle clock: replay / simulator preview."""
    def __init__(self, t, ax, ay, t0=0.0):
        self.t, self.ax, self.ay, self.t0 = np.asarray(t), np.asarray(ax), np.asarray(ay), t0
    def __call__(self, s):
        tt = s['t'] - self.t0 + H
        return (np.interp(tt, self.t, self.ay) / G).astype(np.float32), (np.interp(tt, self.t, self.ax) / G).astype(np.float32)


class KinematicPredictor:
    """a_y(t+h) = v(t+h) * r(t+h),  r = v * delta / (L (1 + K v^2)),  delta = sw / ratio.
    Steering wheel angle extrapolated with its rate, decaying with tau (drivers do not keep
    winding forever); speed extrapolated with a_x. Sign: + = left (vehicle frame +y left)."""
    def __init__(self, wheelbase_m=2.85, steer_ratio=13.5, understeer_K=0.0016, steer_tau_s=0.4, rate_lp_s=0.15):
        self.L, self.ratio, self.K, self.tau, self.lp = wheelbase_m, steer_ratio, understeer_K, steer_tau_s, rate_lp_s
        self.prev = None; self.rate = 0.0
    def __call__(self, s):
        t, sw, v, ax = s['t'], s['steer_rad'], max(s['speed_mps'], 0.0), s['ax_mps2']
        if self.prev is not None and t > self.prev[0]:
            r = (sw - self.prev[1]) / (t - self.prev[0]); a = min(1.0, (t - self.prev[0]) / self.lp)
            self.rate += a * (r - self.rate)
        self.prev = (t, sw)
        sw_h = sw + self.rate * self.tau * (1 - np.exp(-H / self.tau))
        v_h = np.maximum(v + ax * H, 0.0)
        ay = v_h ** 2 * (sw_h / self.ratio) / (self.L * (1 + self.K * v_h ** 2))
        return (ay / G).astype(np.float32), np.full(10, ax / G, np.float32)


def quality(pred, rows, dt=0.1):
    """Offline check on a recorded drive (rows: dicts with t, ay_mps2, ax_mps2, speed_mps, steer_rad).
    Returns RMS error [g] per horizon vs the true future; compare with the stand-in's
    0.005-0.02 g + 0.01-0.06 g/s * h band the policy was trained on."""
    t = np.array([r['t'] for r in rows]); ay = np.array([r['ay_mps2'] for r in rows]) / G
    P = np.array([pred(r)[0] for r in rows])
    truth = np.stack([np.interp(t + h, t, ay) for h in H], 1)
    ok = t + H[-1] <= t[-1]
    err = np.sqrt(np.mean((P[ok] - truth[ok]) ** 2, 0))
    band = np.stack([0.005 + 0.01 * H, 0.02 + 0.06 * H], 1)
    return dict(h_s=H.tolist(), rms_g=err.round(4).tolist(), trained_band_g=band.round(3).tolist())
