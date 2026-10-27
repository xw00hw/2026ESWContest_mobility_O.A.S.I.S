"""Sensor / actuator models whose parameters are domain-randomised.

Everything here is causal and stream-rate aware: each sensor samples at its own
rate, adds its own latency, and the env builds observations with the same
ZOH + age/valid rule the real-time pipeline uses (review doc 4.2 / 4-C).
"""
import numpy as np
from collections import deque


class DelayLine:
    """Fixed pure delay on a sampled stream (latency of a sensor pipeline)."""
    def __init__(self, delay_s, dt):
        self.n = max(0, int(round(delay_s / dt))); self.q = deque()
    def __call__(self, x):
        self.q.append(x)
        return self.q.popleft() if len(self.q) > self.n else None


class FSRChannel:
    """Force -> 12-bit ADC counts through a voltage divider, with the FSR defects
    seen in the L1 log: unit-to-unit gain spread, power-law nonlinearity, creep
    (log-time), play-operator hysteresis, white noise, rail clipping, dead channel.
    R_fsr = R1 * (F/1N)^(-beta);  V = Vcc * Rm / (Rm + R_fsr)."""
    def __init__(self, rng, P, preload_N):
        self.rng = rng
        self.gain = np.exp(rng.normal(0, P['fsr_gain_logsd']))
        self.beta = rng.uniform(*P['fsr_beta'])
        self.R1 = P['fsr_R1_ohm'] * np.exp(rng.normal(0, 0.3))
        self.Rm = P['fsr_Rm_ohm']
        self.noise = P['fsr_noise_counts']
        self.creep = rng.uniform(*P['fsr_creep_frac_per_decade'])
        self.hyst = rng.uniform(*P['fsr_hyst_frac'])
        self.dead = rng.random() < P['fsr_p_dead']
        self.preload = preload_N * np.exp(rng.normal(0, 0.3))
        self.t_loaded, self.play = 0.0, preload_N
        self.tau_pad = rng.uniform(*P['fsr_pad_tau_s'])   # foam/pressure-pad mechanical low-pass
        self.Ff = preload_N

    def __call__(self, F, dt):
        if self.dead:
            return float(self.rng.integers(0, 4))
        self.Ff += min(dt / self.tau_pad, 1.0) * (F - self.Ff)
        F = max(self.Ff + self.preload, 0.0) * self.gain
        r = self.hyst * 20.0                       # play width in N
        self.play = min(max(self.play, F - r), F + r)
        Fh = (1 - self.hyst) * F + self.hyst * self.play
        self.t_loaded += dt
        Fh *= 1 + self.creep * np.log10(1 + self.t_loaded / 10.0)
        if Fh < 0.05:
            v = 0.0
        else:
            R = self.R1 * Fh ** (-self.beta)
            v = self.Rm / (self.Rm + R)
        c = 4095 * v + self.rng.normal(0, self.noise)
        return float(np.clip(np.round(c), 0, 4095))


class PoseCamera:
    """MediaPipe-like 2-D landmark stream: decimated rate, capture->output latency,
    projection gain (oblique A-pillar view -> partial lateral sensitivity), jitter,
    dropouts. Outputs normalised image-x of shoulder / head / hip centres."""
    def __init__(self, rng, P, dt):
        self.rng, self.dt = rng, dt
        self.period = 1.0 / rng.uniform(*P['pose_rate_hz'])
        self.delay = DelayLine(rng.uniform(*P['pose_latency_s']), dt)
        self.gain = rng.uniform(*P['pose_proj_gain_per_m']) * rng.choice([-1, 1]) if P['pose_random_sign'] else rng.uniform(*P['pose_proj_gain_per_m'])
        self.jit = rng.uniform(*P['pose_jitter_norm'])
        self.p_drop = rng.uniform(*P['pose_dropout'])
        self.bias = rng.normal(0, 0.02, 3) + np.array([0.16, 0.16, 0.41])
        self.t_next, self.last, self.age = 0.0, None, 1.0

    def __call__(self, t, y_sh, y_head, y_hip):
        out = self.delay((t, y_sh, y_head, y_hip))
        self.age += self.dt
        if out is not None and out[0] >= self.t_next:
            self.t_next = out[0] + self.period
            if self.rng.random() > self.p_drop:
                x = self.bias + self.gain * np.array(out[1:]) + self.rng.normal(0, self.jit, 3)
                self.last, self.age = x, t - out[0]
        return self.last, self.age


class SampledSignal:
    """Generic rate-limited, delayed, noisy, biased, quantised sensor (IMU / OBD)."""
    def __init__(self, rng, rate_hz, latency_s, noise, bias, quant, dt):
        self.period, self.delay = 1.0 / rate_hz, DelayLine(latency_s, dt)
        self.noise, self.bias, self.quant, self.rng, self.dt = noise, bias, quant, rng, dt
        self.t_next, self.last, self.age = 0.0, 0.0, 1.0
    def __call__(self, t, x):
        out = self.delay((t, x)); self.age += self.dt
        if out is not None and out[0] >= self.t_next:
            self.t_next = out[0] + self.period
            v = out[1] + self.bias + self.rng.normal(0, self.noise)
            if self.quant: v = np.round(v / self.quant) * self.quant
            self.last, self.age = v, t - out[0]
        return self.last, self.age


class BolsterActuator:
    """Command -> transport delay -> rate limit -> 1st-order lag -> backlash -> angle.
    All parameters are L2-bench quantities (not measured yet): wide priors."""
    def __init__(self, rng, P, dt):
        self.dt = dt
        self.delay = DelayLine(rng.uniform(*P['act_delay_s']), dt)
        self.tau = rng.uniform(*P['act_tau_s'])
        self.rate = np.deg2rad(rng.uniform(*P['act_rate_dps']))
        self.backlash = np.deg2rad(rng.uniform(*P['act_backlash_deg']))
        self.th_max = np.deg2rad(P['theta_max_deg'])
        self.th, self.th_int, self.out = 0.0, 0.0, 0.0
        self.cmd = 0.0
    def __call__(self, cmd):
        c = self.delay(cmd)
        if c is not None: self.cmd = c
        d = np.clip((self.cmd - self.th_int) * self.dt / self.tau, -self.rate * self.dt, self.rate * self.dt)
        self.th_int = np.clip(self.th_int + d, 0, self.th_max)
        h = self.backlash / 2
        self.out = min(max(self.out, self.th_int - h), self.th_int + h)
        return self.out


class AccelPredictor:
    """Stand-in for the L1-trained 1 s look-ahead a_y predictor: true future
    trajectory + horizon-growing correlated error + phase lag + gain error."""
    def __init__(self, rng, P, horizon_n, dt_h):
        self.rng = rng
        self.sig0, self.sig1 = rng.uniform(*P['pred_sigma0_g']), rng.uniform(*P['pred_sigma_per_s_g'])
        self.gain = rng.uniform(*P['pred_gain'])
        self.lag = rng.uniform(*P['pred_lag_s'])
        self.h = np.arange(1, horizon_n + 1) * dt_h
        self.e = np.zeros(horizon_n)
    def __call__(self, future_fn, t):
        true = np.array([future_fn(t + hh - self.lag) for hh in self.h])
        self.e = 0.9 * self.e + np.sqrt(1 - 0.81) * self.rng.normal(0, 1, len(self.h))
        return self.gain * true + 9.81 * (self.sig0 + self.sig1 * self.h) * self.e
