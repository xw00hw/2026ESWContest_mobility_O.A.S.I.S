"""Shared RL pieces: observation history, running normaliser, rule-based policies,
and the paired-seed evaluation bank. Everything the deployed actor needs
(history length, normaliser stats) is saved with the checkpoint."""
import os, sys
from collections import deque
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
if os.environ.get('OASIS_ENV') == 'v2':          # calibrated servo-bench environment (sim/v2)
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'v2'))
    from env_v2 import BolsterEnvV2 as BolsterEnv  # noqa: E402
elif os.environ.get('OASIS_ENV') == 'v3':        # v2 + bolster pitch, two pad zones, body-shape coupling (sim/v3)
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'v3'))
    from env_v3 import BolsterEnvV3 as BolsterEnv  # noqa: E402
else:
    from bolster_env import BolsterEnv  # noqa: E402

EVAL_KINDS = ['urban_turn', 'lane_change', 'slalom', 'l1_replay']


class History:
    """Stack of the last K observations (oldest first). Real-time code must use the
    same K and the same zero-padding at session start."""
    def __init__(self, obs_dim, K):
        self.K, self.dim = K, obs_dim
        self.q = deque(maxlen=K + 1)   # K past frames + current
    def reset(self, o):
        self.q.clear()
        for _ in range(self.K): self.q.append(np.zeros_like(o))
        self.q.append(o); return self.get()
    def push(self, o):
        self.q.append(o); return self.get()
    def get(self):
        return np.concatenate(list(self.q)).astype(np.float32)


class RunningNorm:
    def __init__(self, dim, clip=5.0):
        self.n, self.mean, self.M2, self.clip = 1e-4, np.zeros(dim), np.ones(dim), clip
        self.frozen = False
    def update(self, x):
        if self.frozen: return
        self.n += 1; d = x - self.mean; self.mean += d / self.n; self.M2 += d * (x - self.mean)
    @property
    def std(self):
        return np.sqrt(self.M2 / self.n) + 1e-6
    def __call__(self, x):
        return np.clip((x - self.mean) / self.std, -self.clip, self.clip).astype(np.float32)
    def state(self):
        return dict(n=self.n, mean=self.mean, M2=self.M2)
    def load(self, s):
        self.n, self.mean, self.M2 = s['n'], np.asarray(s['mean']), np.asarray(s['M2'])


PITCH_PRIOR_AY_G = 0.15     # a_y prediction at which the pitch prior reaches full amplitude


def ff_baseline(o, gain=6.0, lead_idx=3, action_dim=2, pitch_gain=0.0, both_gain=0.0, deadband_g=0.0):
    """Predictive feed-forward rule on the a_y predicted (lead_idx+1)*0.1 s ahead, beyond a deadband:
      differential d = -gain * sign(a) * max(|a| - deadband, 0)      (support the side the body goes to)
      common      c = -1 + both_gain * max(|a| - deadband, 0)        (squeeze both sides in strong turns)
      pitch (4 actions): pitch_gain x the loaded side, < 0 = upper pad inward
    Used as baseline and as the residual-RL prior. both_gain = deadband = pitch_gain = 0 is the v2 rule.
    Same formula in deploy_actor.py and deploy/runtime/oasis_rt (actor.py, actor_core.c)."""
    ay = o[lead_idx]; m = max(abs(ay) - deadband_g, 0.0)
    a = np.zeros(action_dim); a[0] = np.clip(-1.0 + both_gain * m, -1, 1); a[1] = np.clip(-gain * np.sign(ay) * m, -1, 1)
    if action_dim == 4 and pitch_gain:
        s = np.clip(-ay / PITCH_PRIOR_AY_G, -1, 1)                      # > 0: left side will be loaded (a_y < 0)
        pL, pR = pitch_gain * max(s, 0.0), pitch_gain * max(-s, 0.0)
        a[2], a[3] = (pL + pR) / 2, (pL - pR) / 2                      # pitch_L = 8 clip(cp+dp), pitch_R = 8 clip(cp-dp)
    return a


def reactive_baseline(o, gain=6.0):
    return np.array([-1.0, np.clip(-gain * o[20], -1, 1)])


def run_episode(policy, seed, level, kind, record=False):
    """policy(obs, env) -> action. Returns metrics (+ traces if record)."""
    env = BolsterEnv(level=level, seed=seed, scenario_kind=kind)
    o, info = env.reset()
    com, F, th, a_hist, trips, R, tr_ = [], [], [], [], 0, 0.0, None
    dF = []                                    # v3+: force the bolsters add vs the open-bolster twin
    if hasattr(policy, 'reset'): policy.reset(o)
    while True:
        a = policy(o, env)
        o, r, te, tr, info = env.step(a)
        p = info['privileged']; R += r
        com.append(p['y_com']); F.append(max(p['F_L'], p['F_R'])); th.append(np.rad2deg(p['theta']))
        a_hist.append(np.asarray(a, float)); trips += info.get('trip', o[57] if len(o) > 57 else 0)
        if 'added_squeeze_N' in p: dF.append(p['added_squeeze_N'])
        if record:
            tr_ = tr_ or dict(t=[], ay=[], y_com=[], thL=[], thR=[], FL=[], FR=[])
            tr_['t'].append(env.t); tr_['ay'].append(p['ay']); tr_['y_com'].append(p['y_com'])
            tr_['thL'].append(np.rad2deg(p['theta'][0])); tr_['thR'].append(np.rad2deg(p['theta'][1]))
            tr_['FL'].append(p['F_L']); tr_['FR'].append(p['F_R'])
        if tr: break
    com, a_hist = np.array(com), np.array(a_hist)
    m = dict(seed=seed, level=level, kind=kind, arch=info['params']['archetype'], ret=R,
             rms_cm=100 * np.sqrt(np.mean(com ** 2)), peak_cm=100 * np.max(np.abs(com)),
             trips=trips, theta_max=float(np.max(th)), F_max=float(np.max(F)),
             jerk=float(np.mean(np.abs(np.diff(a_hist[:, 1])))))
    if dF:                                     # 'pressed too hard' metrics (reward.over_F_N, default 20 N)
        F0 = env.cfg['reward'].get('over_F_N', 20.0); dF = np.array(dF)
        m.update(squeeze_p95_N=float(np.percentile(dF, 95)), squeeze_max_N=float(dF.max()),
                 over_time_pct=float(100 * np.mean(dF > F0)), over_press=float(np.mean((np.maximum(dF - F0, 0) / F0) ** 2)))
    return (m, tr_) if record else m
