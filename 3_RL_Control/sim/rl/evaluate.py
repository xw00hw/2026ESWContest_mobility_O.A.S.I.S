"""Paired-seed evaluation bank (review doc 7, L3): every controller sees the same
episodes (seed -> scenario, occupant, sensors). Metric = CoM lateral RMS reduction
vs the passive seat, reported as a distribution, plus comfort/safety counters.

  python evaluate.py --n 24 --runs asym resid sym
"""
import argparse, os, json
import numpy as np, pandas as pd, torch, torch.nn as nn
from common import run_episode, ff_baseline, reactive_baseline, History, RunningNorm, EVAL_KINDS


class ActorPolicy:
    """Deterministic deployed actor: history -> normalise -> MLP -> tanh(mean)."""
    def __init__(self, path):
        ck = torch.load(path, weights_only=False)
        a = ck['args']; self.K, self.OD, self.res = a['K'], ck['OD'], a['residual']
        self.scale = a.get('res_scale', 0.5); self.ff = (a.get('ff_gain', 6.0), a.get('ff_lead_idx', 3))
        self.pitch_prior = a.get('pitch_prior', 0.0); self.both, self.db = a.get('ff_both', 0.0), a.get('ff_deadband', 0.0)
        h = a['hidden']; XA = self.OD * (self.K + 1)
        self.AD = ck['actor']['net.4.bias'].numel() // 2                     # 2 (yaw) or 4 (yaw + pitch)
        self.net = nn.Sequential(nn.Linear(XA, h), nn.ReLU(), nn.Linear(h, h), nn.ReLU(), nn.Linear(h, 2 * self.AD))
        self.net.load_state_dict({k.replace('net.', ''): v for k, v in ck['actor'].items()})
        self.norm = RunningNorm(self.OD); self.norm.load(ck['norm'])
        self.hist = History(self.OD, self.K)
    def reset(self, o):
        self.hist.reset(o)
    def __call__(self, o, env):
        xa = self.hist.get().reshape(self.K + 1, self.OD)
        x = ((xa - self.norm.mean) / self.norm.std).clip(-5, 5).reshape(1, -1).astype(np.float32)
        with torch.no_grad():
            mu = torch.tanh(self.net(torch.as_tensor(x))[0, :self.AD]).numpy()
        a = np.clip(ff_baseline(o, *self.ff, self.AD, self.pitch_prior, self.both, self.db) + self.scale * mu, -1, 1) if self.res else mu
        self._last = a
        return a
    def post(self, o):   # called after env.step to push the new observation
        self.hist.push(o)


def wrap(pol):
    """run_episode calls policy(o) before each step; keep the actor's history in sync."""
    first = {'v': True}
    def f(o, env):
        if not first['v']: pol.hist.push(o)
        first['v'] = False
        return pol(o, env)
    f.reset = lambda o: (pol.reset(o), first.update(v=True))
    return f


class FbV6Policy:
    """BeamNG_Bolster_FB Experiment A V6 controller (4_Seat_Actuation/experiment_a/controller.py, unchanged)
    in the simulator: its vehicle frame is what the Pi receives (env.veh_sample: BeamNG-like acceleration without
    gravity, handwheel angle, speed), its board state the simulated OASIS_FALLBACK v2 pose. Its yaw targets map back to
    [c, d]; its pitch is dropped (yaw-only env, v8). tune='episode': STM yaw_max / rate of the episode (same as the
    other controllers); tune='fb_default': FB's own defaults (15 deg, 2 deg/s)."""
    def __init__(self, tune='episode'):
        import sys as _s
        _s.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'deploy', 'runtime'))
        from oasis_rt import fbstack
        self.fb, self.tune = fbstack.load(), tune
    def reset(self, o): self.ctl = None; self.k = 0
    def __call__(self, o, env):
        fb = self.fb
        if self.ctl is None:
            s = fb.settings.Settings()
            if self.tune == 'episode':
                s.max_yaw_deg = min(15.0 if env.v3p.get('fb_cap') is None else env.v3p['fb_cap'], 20.0)
                s.max_yaw_rate_deg_s = s.max_pitch_rate_deg_s = float(env.v3p.get('fb_rate', 20.0))
            self.ctl = fb.controller.ExperimentAController(s.validate()); self.th = float(env.th_max)
        self.k += 1; now = int((10.0 + env.t) * 1e9); vs = env.veh_sample(); v = float(vs['speed_mps'])
        fr = fb.vehicle.VehicleFrame(received_monotonic_ns=now, sim_time_s=float(env.t), stream_id='sim', vehicle_id='1',
                                     sequence_id=self.k, ay_mps2=float(vs['ay_mps2']), ax_mps2=float(vs['ax_mps2']), speed_mps=v,
                                     steering_deg=float(np.rad2deg(vs['steer_rad'])), throttle=0.2, brake=0.0,
                                     valid=v >= 1.0, reason='ok' if v >= 1.0 else 'stopped', source_kind='beamng',
                                     roll_rad=0.0, pitch_rad=0.0, gear_index=3.0)
        sv = env.servo; pose = tuple(int(round(1000 * x)) for x in (sv['L'].reported[0], 0.0, sv['R'].reported[0], 0.0))
        tgt = tuple(int(round(1000 * x)) for x in (sv['L'].target[0], 0.0, sv['R'].target[0], 0.0))
        st = fb.serial_link.BoardState(received_ns=now, armed=True, moving=pose != tgt, pose_mdeg=pose, target_mdeg=tgt,
                                       pulses_us=(1800, 1200, 1200, 1800), command_id=self.k, fault='NONE')
        d = self.ctl.tick(fr, st, now)
        if d.emergency_off: return np.array([-1.0, 0.0])                  # FB stops; the sim keeps the bolsters open
        yL, yR = d.pose_mdeg[0] / 1000, d.pose_mdeg[2] / 1000
        return np.clip([2 * (yL + yR) / self.th - 1, (yL - yR) / self.th], -1, 1)


def _policy(spec):
    if spec[0] == 'passive': return lambda o, e: np.array([-1.0, 0.0])
    if spec[0] == 'fb_v6': return FbV6Policy(spec[1])
    if spec[0] == 'reactive': return lambda o, e: reactive_baseline(o)
    if spec[0] == 'ff': return lambda o, e: ff_baseline(o, *spec[1:3], spec[3], spec[4], spec[5], spec[6])
    return wrap(ActorPolicy(spec[1]))


def _job(j):
    name, spec, seed, lv, kind = j
    m = run_episode(_policy(spec), seed, lv, kind); m['ctrl'] = name; return m


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, default=24)
    ap.add_argument('--runs', nargs='*', default=[])
    ap.add_argument('--levels', nargs='*', type=float, default=[0, 2])
    ap.add_argument('--out', default='eval_bank.csv')
    ap.add_argument('--overlay', help='DR overlay applied to every episode (e.g. ../overlays/safety_v2.yaml)')
    ap.add_argument('--rule', help='tuned rule json (sim/v3/tune_rule.py): adds rule_tuned (+ rule_tuned_nopitch if it pitches)')
    ap.add_argument('--workers', type=int, default=1)
    ap.add_argument('--fb_v6', action='store_true', help='add BeamNG_Bolster_FB V6 (episode STM tune and FB defaults)')
    ap.add_argument('--only', nargs='*', help='keep only these controllers (+ passive, the reference)')
    args = ap.parse_args()
    if args.overlay:
        os.environ['OASIS_DR_OVERLAY'] = os.path.abspath(args.overlay)
    here = os.path.dirname(os.path.abspath(__file__))
    # controllers as picklable specs (the episodes run in a process pool)
    ctrls = {'passive': ('passive',), 'reactive': ('reactive',), 'feedforward': ('ff', 6.0, 3, 2, 0.0, 0.0, 0.0),
             # rule tuned to the same reward on separate seeds (2000+, ff_sweep): gain 1.5, lead 0.6 s
             'ff_tuned': ('ff', 1.5, 5, 2, 0.0, 0.0, 0.0)}
    if os.environ.get('OASIS_ENV') in ('v2', 'v3'):   # v2: rule tuned on seeds 2000+ (ff_sweep_v2): gain 8, lead 0.7 s
        ctrls['ff_tuned'] = ('ff', 8.0, 6, 2, 0.0, 0.0, 0.0)
    if os.environ.get('OASIS_ENV') == 'v3':   # same rule + loaded-side pitch (upper pad in), the v3 pitch prior
        ctrls['ff_tuned_pitch'] = ('ff', 8.0, 6, 4, -0.5, 0.0, 0.0)
    if args.rule:
        b = json.load(open(args.rule))['best']
        ctrls['rule_tuned'] = ('ff', b['gain'], int(b['lead_idx']), 4, b['pitch'], b['both'], b['deadband_g'])
        if b['pitch']: ctrls['rule_tuned_nopitch'] = ('ff', b['gain'], int(b['lead_idx']), 4, 0.0, b['both'], b['deadband_g'])
    for r in args.runs:
        best = os.path.join(here, 'runs', r, 'actor_best.pt')   # chosen on validation seeds (select_ckpt.py)
        ctrls['sac_' + r] = ('actor', best if os.path.exists(best) else os.path.join(here, 'runs', r, 'actor.pt'))
    if args.fb_v6: ctrls.update(fb_v6=('fb_v6', 'episode'), fb_v6_default=('fb_v6', 'fb_default'))
    if args.only: ctrls = {k: v for k, v in ctrls.items() if k in set(args.only) | {'passive'}}
    jobs = [(name, spec, 1000 + s, lv, EVAL_KINDS[s % 4]) for lv in args.levels for s in range(args.n) for name, spec in ctrls.items()]
    if args.workers > 1:
        from multiprocessing import Pool
        with Pool(args.workers) as pool: rows = pool.map(_job, jobs, chunksize=2)
    else:
        rows = [_job(j) for j in jobs]
    D = pd.DataFrame(rows)
    base = D[D.ctrl == 'passive'].set_index(['level', 'seed']).rms_cm
    D['reduction_pct'] = 100 * (1 - D.rms_cm / D.set_index(['level', 'seed']).index.map(base).values)
    if 'over_time_pct' in D:   # same criterion as sim/v3/tune_rule.py: CoM reduction minus 'pressed too hard'
        D['score'] = D.reduction_pct - 0.5 * D.over_time_pct.fillna(0) - 100.0 * D.over_press.fillna(0)
    D.to_csv(os.path.join(here, args.out), index=False)
    g = D.groupby(['level', 'ctrl'])
    S = g.agg(ret=('ret', 'median'), rms_cm=('rms_cm', 'median'), red_p10=('reduction_pct', lambda x: np.percentile(x, 10)),
              red_med=('reduction_pct', 'median'), red_p90=('reduction_pct', lambda x: np.percentile(x, 90)),
              trips=('trips', 'mean'), F_max=('F_max', 'median'), th_max=('theta_max', 'median'),
              jerk=('jerk', 'median')).round(3)
    print(S.to_string())
    print(D.groupby(['level', 'ctrl', 'arch']).reduction_pct.median().unstack().round(1).to_string())
