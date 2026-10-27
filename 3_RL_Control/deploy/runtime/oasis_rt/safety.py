"""Host-side bolster interlock, identical to the one the policy was trained with
(sim/v2/env_v2.BolsterEnvV2._advance, safety.mode = model). The servo rig has no current
sensing, so the squeeze is estimated from the side pads: force = sum over the wing's pad zones of
(counts - rest) / counts_per_N (v3: lower + upper pad), minus the occupant's own inertial load on that
side (a_y > 0 loads the RIGHT wing, vehicle +y left).

  hard : force > hard_N on a closed side -> release that side to 0 deg, hold it there hold_s
  soft : force > soft_N on a closed side -> no further closing (freeze at the reported angle)
  invalid : NaN / negative infinity -> release and hold regardless of reported angle

This is a comfort/safety layer on top of the firmware's own bounds, rate limit and timeout,
not a replacement for them. Output flag feeds observation slot 45 (max(trip, 0.5*hold)).
"""
import json
import numpy as np


class FsrInverse:
    """pad reading [counts] -> force [N] on the calibration contact: inverse of the fitted FSR curve
    counts = cmax F^n / (F^n + F50^n) (deploy/tools/fsr_curve.py --write). The pads saturate near
    1880-1920 counts (~50 N where the gain drops below 4 counts/N): a linear counts_per_N would put the
    60 / 120 N thresholds ~2000 counts above rest, beyond what the sensor can read."""
    def __init__(self, path):
        f = json.load(open(path))['fit']; self.cm, self.f50, self.n = f['cmax_counts'], f['F50_N'], f['n']
    def __call__(self, c):
        if c >= self.cm: return float('inf')                 # at / beyond saturation: treat as over any limit
        if c <= 0: return 0.0
        return self.f50 * (c / (self.cm - c)) ** (1.0 / self.n)


class Interlock:
    def __init__(self, cfg, console_N_per_g):
        self.soft, self.hard = cfg['fsr_force_soft_N'], cfg['fsr_force_hard_N']
        self.arm, self.hold_s, self.model = cfg['arm_deg'], cfg['hold_s'], cfg.get('mode', 'model') == 'model'
        self.npg = console_N_per_g
        self.hold_until = [-1e9, -1e9]; self.hold_th = [0.0, 0.0]
        self.trip = 0.0; self.hold = 0.0; self.force = [0.0, 0.0]

    def __call__(self, now, cmd, reported, side_force_N, ay_g):
        """cmd/reported: [left, right] yaw deg; side_force_N: [L, R] pad force above rest, summed over the
        wing's pad zones (None = no data). Returns the gated yaw command [left, right]."""
        cmd = list(cmd); self.trip = 0.0; self.hold = 0.0
        for j, load in enumerate((-ay_g, ay_g)):
            if side_force_N[j] is None:
                # A missing sample must not cancel a release/hold already in progress.
                if now < self.hold_until[j]: cmd[j] = min(cmd[j], self.hold_th[j])
                continue
            F = side_force_N[j]
            if self.model: F -= self.npg * max(load, 0.0)
            self.force[j] = F
            if np.isnan(F) or np.isneginf(F):
                # E.g. two saturated nonlinear readings give inf - inf = NaN.
                # Unknown force cannot authorize closing, even below arm_deg.
                # Positive infinity remains the explicit saturation/hard-limit path.
                cmd[j] = 0.0
                self.trip = 1.0; self.hold_until[j] = now + self.hold_s; self.hold_th[j] = 0.0
            elif F > self.hard and reported[j] > self.arm:
                self.trip = 1.0; self.hold_until[j] = now + self.hold_s; self.hold_th[j] = 0.0
            elif F > self.soft and reported[j] > self.arm:
                self.hold = 1.0
                self.hold_th[j] = min(self.hold_th[j], reported[j]) if now < self.hold_until[j] else reported[j]
                self.hold_until[j] = now + self.hold_s
            if now < self.hold_until[j]: cmd[j] = min(cmd[j], self.hold_th[j])
        return cmd

    @property
    def flag(self):
        return max(self.trip, 0.5 * self.hold)
