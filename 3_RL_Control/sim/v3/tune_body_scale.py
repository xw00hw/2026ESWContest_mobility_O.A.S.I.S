"""v9: tune ONE thing -- the body scale of the seated G controller (G base curve + bounded v8 RL residual).

The controller is the final seated-bench controller, unchanged: oasis_rt.g_residual_control (imported from the Pi
runtime, so sim and Pi run the same code) on measured a_y and the 0.7 s BeamNG prediction, the v8 actor's
effective angle minus its training prior as a +-1.5 deg residual, one signed 0.3 s filter, speed gate. The
actor sees the deployed profile (75 kg / 0.42 m / 0.58 m: profile.from_camera stays false), like on the Pi.
The bench actuator is the reviewed trial setting (STM yaw rate 10 deg/s, yaw cap 20 deg).

The reference user is the sim nominal body (shoulder 0.42 m, sitting torso 0.58 m = the trial's profile). For each
body variant the scale s of the base / preview angle is swept on paired episodes (rule-tuning seeds 3000+). The
criterion is support, the mean CoM RMS reduction vs the open-bolster twin: with it the reference body's optimum is
s = 1, as the reference user reported for the final seated setting. (The evaluation score with its comfort penalty picks
the smallest scale even for the reference body, i.e. it disagrees with that report; it is kept as a diagnostic.)
The optima are fitted as s = 1 + K_W (r_w - 1) + K_T (r_t - 1), r = body / reference; --validate checks a law on
held-out eval-bank episodes (seeds 1000+) including the body_type_eval bodies.

  python sim/v3/tune_body_scale.py --runtime deploy/runtime --out results/v9/body_scale.json --csv <episodes.csv>
  python sim/v3/tune_body_scale.py --runtime deploy/runtime --from_csv <episodes.csv> --out results/v9/body_scale.json
  python sim/v3/tune_body_scale.py --runtime deploy/runtime --validate -1.2 0 0.9 1.1 --out results/v9/body_scale_validation.json
"""
import argparse, json, os, sys, tempfile
from multiprocessing import Pool
import numpy as np

H = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(H, '..', 'rl'), H]
REF = (0.42, 0.58)
WIDTHS = (0.36, 0.39, 0.42, 0.45, 0.48)          # DR range of shoulder width (sim/v2/dr_v2.yaml)
TORSOS = (0.50, 0.54, 0.62, 0.66)                 # DR range of sitting torso, at the reference width
SCALES = (0.8, 0.85, 0.9, 0.95, 1.0, 1.05, 1.1, 1.15, 1.2)
PROFILE = slice(47, 50)                           # mass / shoulder / torso obs slots (0 = deployed 75/0.42/0.58)


def overlay(w, t):
    import yaml
    p = os.path.join(tempfile.mkdtemp(), f'body_{w}_{t}.yaml')
    yaml.safe_dump({'profile': {'shoulder_width_m': {'dist': 'uniform', 'range': [w, w], 'nominal': w, 'level': 0},
                                'sitting_torso_m': {'dist': 'uniform', 'range': [t, t], 'nominal': t, 'level': 0}},
                    'v3': {'fb_a2': {'rate_dps_choices': [10], 'rate_p': [1.0],      # reviewed trial STM tune
                                     'yaw_cap_choices': [20], 'yaw_cap_p': [1.0]}}}, open(p, 'w'))
    return p


def load_runtime(runtime):
    """The Pi runtime's oasis_rt (g_residual_control + body_scale) under its own name, so a workspace copy of
    oasis_rt already on sys.path cannot shadow the code being tuned."""
    import importlib, importlib.util
    name = 'oasis_rt_pi'
    if name not in sys.modules:
        pkg = os.path.join(runtime, 'oasis_rt')
        spec = importlib.util.spec_from_file_location(name, os.path.join(pkg, '__init__.py'), submodule_search_locations=[pkg])
        mod = importlib.util.module_from_spec(spec); sys.modules[name] = mod; spec.loader.exec_module(mod)
    importlib.import_module(name + '.g_residual_control'); importlib.import_module(name + '.body_scale')
    return sys.modules[name]


class GSeated:
    """The Pi's controller._g_action in the simulator (same governor class, same filter, same action preimage)."""
    def __init__(self, ck, scale, runtime, prior=(16.0, 6)):
        GResidualGovernor = load_runtime(runtime).g_residual_control.GResidualGovernor
        from evaluate import ActorPolicy
        from obs_layout_v3 import action_to_targets
        self.P, self.gov, self.t2 = ActorPolicy(ck), GResidualGovernor(None, body_scale=scale), action_to_targets
        self.prior, self.dt, self.alpha = prior, 0.05, 0.05 / 0.3
        self.first, self.y = True, 0.0

    def reset(self, o):
        self.P.reset(self._dep(o)); self.gov.reset(); self.first, self.y = True, 0.0

    @staticmethod
    def _dep(o):
        o = o.copy(); o[PROFILE] = 0.0; return o

    def __call__(self, o, env):
        from common import ff_baseline
        od = self._dep(o)
        if not self.first: self.P.hist.push(od)
        self.first = False
        raw = self.P(od, env); prior = ff_baseline(od, self.prior[0], self.prior[1], 2)
        raw_lr, prior_lr = self.t2(raw, 20.0)[:2], self.t2(prior, 20.0)[:2]
        req, snap = self.gov.step(float(o[20]), float(o[6]), float(o[23]) * 20.0, raw_lr, prior_lr, self.dt)
        target = req[1] - req[0]
        if snap['speed_gate']: self.y = 0.0
        y = self.y + self.alpha * (target - self.y)
        self.y = target if abs(y - target) < 1e-3 else y
        left, right = max(-self.y, 0.0), max(self.y, 0.0)
        return np.array([2 * (left + right) / 20.0 - 1, (left - right) / 20.0], np.float32)


def job(args):
    seed, level, kind, w, t, scale, ck, runtime, base_ov = args
    os.environ['OASIS_DR_OVERLAY'] = base_ov + os.pathsep + overlay(w, t)
    from common import run_episode
    pol = (lambda o, e: np.array([-1.0, 0.0], np.float32)) if scale is None else GSeated(ck, scale, runtime)
    m = run_episode(pol, seed, level, kind)
    return dict(seed=seed, level=level, kind=kind, w=w, t=t, scale=scale, rms=m['rms_cm'],
                over_pct=m.get('over_time_pct', 0.0), over_press=m.get('over_press', 0.0), sq_max=m.get('squeeze_max_N', 0.0))


HELDOUT = {'B-like': (0.374, 0.497), 'A-like': (0.42, 0.58), 'broad': (0.46, 0.62)}   # sim/v3/body_type_eval.py


def validate(a, law):
    """Held-out check of the fitted law: eval-bank episodes (seeds 1000+, levels 0 / 2), the grid bodies plus the
    body_type_eval bodies, paired s = law vs s = 1 on support (CoM reduction), score and squeeze."""
    import pandas as pd
    from scipy.stats import trim_mean
    from common import EVAL_KINDS
    kw, kt, lo, hi = law
    law_s = lambda w, t: float(min(max(1 + kw * (w / REF[0] - 1) + kt * (t / REF[1] - 1), lo), hi))
    ck = os.path.join(H, '..', 'rl', 'runs', a.run, 'actor_best.pt'); base_ov = os.environ.get('OASIS_DR_OVERLAY', '')
    bodies = {f'{w:.2f}/{REF[1]:.2f}': (w, REF[1]) for w in WIDTHS}
    bodies.update({f'{REF[0]:.2f}/{t:.2f}': (REF[0], t) for t in TORSOS}); bodies.update(HELDOUT)
    eps = [(1000 + s, (0.0, 2.0)[s % 2], EVAL_KINDS[s % 4]) for s in range(a.n)]
    jobs = [(sd, lv, k, w, t, sc, ck, a.runtime, base_ov) for (w, t) in set(bodies.values()) for sd, lv, k in eps
            for sc in sorted({None, 1.0, law_s(w, t)}, key=lambda v: -1 if v is None else v)]
    with Pool(a.workers) as pool:
        D = pd.DataFrame(pool.map(job, jobs, chunksize=2))
    P = D[D.scale.isna()].set_index(['w', 't', 'seed']).rms; D = D[D.scale.notna()].copy()
    D['red'] = 100 * (1 - D.rms / D.set_index(['w', 't', 'seed']).index.map(P).values)
    D['score'] = D.red - 0.5 * D.over_pct - 100 * D.over_press
    out = {}
    for name, (w, t) in bodies.items():
        s = law_s(w, t); sub = D[(D.w == w) & (D.t == t)]
        one, lw = sub[sub.scale == 1.0].set_index('seed'), sub[sub.scale == s].set_index('seed')
        r = dict(width_ratio=round(w / REF[0], 3), torso_ratio=round(t / REF[1], 3), scale=round(s, 3),
                 red_s1=round(float(one.red.mean()), 3), red_law=round(float(lw.red.mean()), 3),
                 over_time_s1=round(float(one.over_pct.mean()), 2), over_time_law=round(float(lw.over_pct.mean()), 2),
                 sq_max_s1=round(float(one.sq_max.mean()), 1), sq_max_law=round(float(lw.sq_max.mean()), 1),
                 J_s1=round(float(trim_mean(one.score, 0.1)), 2), J_law=round(float(trim_mean(lw.score, 0.1)), 2))
        if s != 1.0:
            d = (lw.red - one.red).dropna(); rng = np.random.default_rng(0)
            bs = [d.values[rng.integers(0, len(d), len(d))].mean() for _ in range(2000)]
            r.update(red_paired=round(float(d.mean()), 3), red_ci95=[round(float(np.percentile(bs, 2.5)), 3), round(float(np.percentile(bs, 97.5)), 3)],
                     red_better=f'{int((d > 1e-9).sum())}/{len(d)}', score_paired=round(float((lw.score - one.score).mean()), 2))
        out[name] = r
    print(json.dumps(out, indent=1))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--runtime', required=True, help='Pi runtime dir with oasis_rt (g_residual_control with body_scale)')
    ap.add_argument('--run', default='v8_s1'); ap.add_argument('--n', type=int, default=24)
    ap.add_argument('--levels', type=float, nargs='+', default=[1.0, 2.0]); ap.add_argument('--workers', type=int, default=12)
    ap.add_argument('--out', required=True); ap.add_argument('--csv')
    ap.add_argument('--from_csv', help='re-analyse a saved episode table instead of simulating')
    ap.add_argument('--validate', type=float, nargs=4, metavar=('K_W', 'K_T', 'LO', 'HI'),
                    help='held-out check of a law instead of the sweep')
    a = ap.parse_args()
    if a.validate:
        res = validate(a, a.validate)
        json.dump(dict(run=a.run, n=a.n, episodes='eval bank seeds 1000+, levels 0 / 2', law=dict(zip(('K_W', 'K_T', 'LO', 'HI'), a.validate)),
                       bodies=res), open(a.out, 'w'), indent=1)
        return
    import pandas as pd
    from scipy.stats import trim_mean
    from common import EVAL_KINDS
    ck = os.path.join(H, '..', 'rl', 'runs', a.run, 'actor_best.pt'); base_ov = os.environ.get('OASIS_DR_OVERLAY', '')
    bodies = [(w, REF[1]) for w in WIDTHS] + [(REF[0], t) for t in TORSOS]
    eps = [(3000 + s, a.levels[s % len(a.levels)], EVAL_KINDS[s % 4]) for s in range(a.n)]   # rule-tuning validation set
    jobs = [(sd, lv, k, w, t, sc, ck, a.runtime, base_ov) for (w, t) in bodies for sd, lv, k in eps for sc in (None,) + SCALES]
    if a.from_csv:
        D = pd.read_csv(a.from_csv)
    else:
        with Pool(a.workers) as pool:
            D = pd.DataFrame(pool.map(job, jobs, chunksize=2))
        P = D[D.scale.isna()].set_index(['w', 't', 'seed']).rms
        D = D[D.scale.notna()].copy()
        D['red'] = 100 * (1 - D.rms / D.set_index(['w', 't', 'seed']).index.map(P).values)
        D['score'] = D.red - 0.5 * D.over_pct - 100 * D.over_press
        if a.csv: D.to_csv(a.csv, index=False)
    key = lambda k: f'{k[0]:.2f}/{k[1]:.2f}'
    J = D.groupby(['w', 't', 'scale']).score.agg(lambda x: trim_mean(x, 0.1)).unstack('scale')
    red = D.groupby(['w', 't', 'scale']).red.mean().unstack('scale'); over = D.groupby(['w', 't', 'scale']).over_pct.mean().unstack('scale')

    def argmax_s(s, j):
        """(optimum, censored): parabola through the best grid point and its neighbours; a grid edge = censored."""
        i = int(np.argmax(j))
        if 0 < i < len(s) - 1:
            c = np.polyfit(s[i - 1:i + 2], j[i - 1:i + 2], 2)
            if c[0] < 0: return float(np.clip(-c[1] / (2 * c[0]), s[i - 1], s[i + 1])), False
        return float(s[i]), True
    S = np.array(SCALES, float)
    # 1) the evaluation score as is: its comfort penalty makes even the reference body prefer the smallest scale
    #    (censored at 0.8), i.e. it disagrees with the reference user's feel -> diagnostic only
    score_best = {k: argmax_s(S, J.loc[k].values) for k in J.index}
    # 2) support (mean CoM RMS reduction): the reference body's optimum is s = 1, as the reference user reported
    #    -> the tuning criterion; seed-bootstrap 90 % interval of each optimum
    support, rng = {}, np.random.default_rng(0)
    for k in red.index:
        piv = D[(D.w == k[0]) & (D.t == k[1])].pivot_table(index='seed', columns='scale', values='red')
        bs = [argmax_s(S, piv.iloc[rng.integers(0, len(piv), len(piv))].mean().values)[0] for _ in range(400)]
        opt, cens = argmax_s(S, red.loc[k].values)
        support[k] = dict(opt=round(opt, 4), censored=cens, ci90=[round(float(np.percentile(bs, 5)), 3), round(float(np.percentile(bs, 95)), 3)])
    X = np.array([[k[0] / REF[0] - 1, k[1] / REF[1] - 1] for k in support]); y = np.array([v['opt'] - 1 for v in support.values()])
    free = np.array([not v['censored'] for v in support.values()])
    fit_all = np.linalg.lstsq(X, y, rcond=None)[0]; fit_free = np.linalg.lstsq(X[free], y[free], rcond=None)[0]
    print(J.round(2).to_string()); print(red.round(2).to_string())
    print('support optimum', {key(k): v for k, v in support.items()})
    print('fit (censored at the grid edge) K_W %.3f K_T %.3f | uncensored only K_W %.3f K_T %.3f' % (*fit_all, *fit_free))
    out = dict(run=a.run, n=a.n, levels=a.levels, episodes='rule-tuning validation seeds 3000+',
               reference_body=dict(shoulder_width_m=REF[0], sitting_torso_m=REF[1]), scales=list(SCALES),
               criterion='support: mean CoM RMS reduction vs the open-bolster twin (reference optimum = 1, as the user reported)',
               J={key(k): {str(s): round(float(v), 3) for s, v in J.loc[k].items()} for k in J.index},
               red_mean={key(k): {str(s): round(float(v), 3) for s, v in red.loc[k].items()} for k in red.index},
               over_time_pct={key(k): {str(s): round(float(v), 2) for s, v in over.loc[k].items()} for k in over.index},
               score_best_scale={key(k): dict(opt=round(v[0], 4), censored=v[1]) for k, v in score_best.items()},
               support_best_scale={key(k): v for k, v in support.items()},
               fit=dict(censored_at_edge=dict(K_W=round(float(fit_all[0]), 3), K_T=round(float(fit_all[1]), 3)),
                        uncensored=dict(K_W=round(float(fit_free[0]), 3), K_T=round(float(fit_free[1]), 3))))
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True); json.dump(out, open(a.out, 'w'), indent=1)


if __name__ == '__main__':
    main()
