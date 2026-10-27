"""Numpy-only actor for the Raspberry Pi (no torch). Load the .npz written by
export_actor.py; feed the same observation the simulator builds (v0.2: 58-d, v2: 46-d), every 50 ms.

    actor = DeployedActor('actor.npz', filter_tau=0.3)   # 0.3 s output low-pass (v2: -71 % jitter, same CoM effect)
    actor.reset(obs0)               # session start: zero-padded history
    c, d = actor.step(obs)          # each control tick -> send to the MCU (safety layer there)
"""
import numpy as np


class DeployedActor:
    def __init__(self, path, filter_tau=0.0, dt=0.05):
        z = np.load(path)
        self.alpha = 1.0 if filter_tau <= 0 else min(1.0, dt / filter_tau); self.a_f = None
        self.K, self.OD = int(z['K']), int(z['OD'])
        self.res, self.scale = bool(z['residual']), float(z['res_scale'])
        self.ff_gain, self.ff_idx = float(z['ff_gain']), int(z['ff_idx'])
        self.pitch_prior = float(z['pitch_prior']) if 'pitch_prior' in z.files else 0.0
        self.both = float(z['ff_both']) if 'ff_both' in z.files else 0.0; self.db = float(z['ff_deadband']) if 'ff_deadband' in z.files else 0.0
        self.mean = np.tile(z['mean'], self.K + 1); self.std = np.tile(z['std'], self.K + 1)
        self.L = [(z[f'W{i}'], z[f'b{i}']) for i in (0, 2, 4)]
        self.h = np.zeros((self.K + 1, self.OD), np.float32)

    def act(self, x_hist):
        z = np.clip((x_hist - self.mean) / self.std, -5, 5)
        for j, (W, b) in enumerate(self.L):
            z = z @ W + b
            if j < 2: z = np.maximum(z, 0)
        A = len(z) // 2; mu = np.tanh(z[:A])              # A = 2 (yaw) or 4 (yaw + pitch)
        if not self.res:
            return mu
        last = x_hist[-self.OD:]
        ay = last[self.ff_idx]; m = max(abs(ay) - self.db, 0.0)
        prior = np.zeros(A); prior[0] = np.clip(-1.0 + self.both * m, -1, 1); prior[1] = np.clip(-self.ff_gain * np.sign(ay) * m, -1, 1)
        if A == 4 and self.pitch_prior:                    # same as sim/rl/common.ff_baseline
            s = np.clip(-ay / 0.15, -1, 1); pL, pR = self.pitch_prior * max(s, 0.0), self.pitch_prior * max(-s, 0.0)
            prior[2], prior[3] = (pL + pR) / 2, (pL - pR) / 2
        return np.clip(prior + self.scale * mu, -1, 1)

    def _filt(self, a):
        self.a_f = a if self.a_f is None else self.a_f + self.alpha * (a - self.a_f)
        return self.a_f

    def reset(self, o):
        self.h[:] = 0; self.h[-1] = o; self.a_f = None
        return self._filt(self.act(self.h.reshape(-1)))

    def step(self, o):
        self.h = np.roll(self.h, -1, 0); self.h[-1] = o
        return self._filt(self.act(self.h.reshape(-1)))
