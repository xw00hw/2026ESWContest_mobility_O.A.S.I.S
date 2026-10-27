"""Per body type: does the policy treat a thin occupant (clearance, subject B) differently from a broad one
(contact, subject A), and is it right to? Fixed body build per run, eval seeds 1000+ (paired across controllers).

  body types: B-like 0.374 m / sitting torso 0.497 m, A-like 0.42 / 0.58, broad 0.46 / 0.62
  controllers: passive, tuned rule, given SAC runs, and each SAC run with the body cues MASKED (obs 47-53 at
               the population mean: build estimates + rest pressures) -> value of the body-shape information
  metrics: score (CoM reduction - 0.5 over-press time - 100 over-press), CoM reduction, over-press time,
           mean commanded squeeze on the loaded side (deg)

    OASIS_ENV=v3 OASIS_CALIB=... OASIS_L1_SEQ=... OASIS_DR_OVERLAY=... python sim/v3/body_type_eval.py --runs v4_yaw_s1 v5_yaw_s0 --n 24 --out ...
"""
import argparse, json, os, sys, tempfile
import numpy as np, pandas as pd
from multiprocessing import Pool

H = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(H, '..', 'rl'))
BODIES = {'B-like': (0.374, 0.497), 'A-like': (0.42, 0.58), 'broad': (0.46, 0.62)}
BODY = slice(47, 54)


def body_overlay(w, t):
    import yaml
    p = os.path.join(tempfile.mkdtemp(), f'body_{w}.yaml')
    yaml.safe_dump({'profile': {'shoulder_width_m': {'dist': 'uniform', 'range': [w, w], 'nominal': w, 'level': 0},
                                'sitting_torso_m': {'dist': 'uniform', 'range': [t, t], 'nominal': t, 'level': 0}}}, open(p, 'w'))
    return p


def job(args):
    seed, level, body, ctrl, ck, fill, base_ov, rule = args
    w, t = BODIES[body]
    os.environ['OASIS_DR_OVERLAY'] = base_ov + os.pathsep + body_overlay(w, t)
    from common import run_episode, ff_baseline, EVAL_KINDS
    from evaluate import ActorPolicy
    kind = EVAL_KINDS[seed % 4]; squeeze = []
    if ctrl == 'passive':
        pol = lambda o, e: np.array([-1.0, 0.0])
    elif ctrl == 'ff_tuned':
        pol = lambda o, e: ff_baseline(o, rule[0], rule[1])
    else:
        P = ActorPolicy(ck); first = {'v': True}
        def m(o): o = o.copy(); o[BODY] = fill; return o
        def pol(o, e):
            oo = m(o) if fill is not None else o
            if not first['v']: P.hist.push(oo)
            first['v'] = False
            return P(oo, e)
        pol.reset = lambda o: (P.reset(m(o) if fill is not None else o), first.update(v=True))
    def rec(o, e):
        a = pol(o, e); c = (np.clip(a[0], -1, 1) + 1) / 2 * 10; d = np.clip(a[1], -1, 1) * 10
        squeeze.append(max(c + d, c - d)); return a
    if hasattr(pol, 'reset'): rec.reset = pol.reset
    mm = run_episode(rec, seed, level, kind)
    return dict(seed=seed, level=level, body=body, ctrl=ctrl, rms=mm['rms_cm'], over_pct=mm.get('over_time_pct', 0.0),
                over_press=mm.get('over_press', 0.0), squeeze_cmd_deg=float(np.mean(squeeze)), sq_max_N=mm.get('squeeze_max_N', 0.0))


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--runs', nargs='+', required=True); ap.add_argument('--n', type=int, default=24)
    ap.add_argument('--level', type=float, default=1.0); ap.add_argument('--out', required=True); ap.add_argument('--masked', nargs='*', default=None)
    ap.add_argument('--rule', help='tuned rule json for the ff_tuned controller (default: gain 8, lead 0.7 s)')
    a = ap.parse_args()
    rule = (8.0, 6)
    if a.rule:
        b = json.load(open(a.rule))['best']; rule = (float(b['gain']), int(b['lead_idx']))
    base_ov = os.environ.get('OASIS_DR_OVERLAY', '')
    from common import BolsterEnv
    fill = np.mean([BolsterEnv(level=a.level, seed=9000 + s).reset()[0][BODY] for s in range(60)], 0)   # population mean
    rd = os.path.join(H, '..', 'rl', 'runs')
    ctrls = [('passive', None, None), ('ff_tuned', None, None)] + [(f'sac_{r}', os.path.join(rd, r, 'actor_best.pt'), None) for r in a.runs]
    ctrls += [(f'sac_{r}_masked', os.path.join(rd, r, 'actor_best.pt'), fill) for r in (a.masked if a.masked is not None else a.runs)]
    jobs = [(1000 + s, a.level, b, c, ck, f, base_ov, rule) for b in BODIES for s in range(a.n) for c, ck, f in ctrls]
    with Pool(12) as pool:
        D = pd.DataFrame(pool.map(job, jobs, chunksize=4))
    base = D[D.ctrl == 'passive'].set_index(['body', 'seed']).rms
    D['red'] = 100 * (1 - D.rms / D.set_index(['body', 'seed']).index.map(base).values)
    D['score'] = D.red - 0.5 * D.over_pct - 100 * D.over_press
    D.to_csv(a.out, index=False)
    from scipy.stats import trim_mean
    S = D[D.ctrl != 'passive'].groupby(['body', 'ctrl']).agg(J=('score', lambda x: trim_mean(x, 0.1)), red=('red', 'mean'), over=('over_pct', 'mean'),
                                                             squeeze_deg=('squeeze_cmd_deg', 'mean'), sq_max=('sq_max_N', 'max')).round(2)
    pd.set_option('display.width', 200); print(S.to_string())
    res = {}
    for b in BODIES:
        P = D[D.body == b].pivot_table(index='seed', columns='ctrl', values='score')
        for c in [c for c in P.columns if c.startswith('sac_') and not c.endswith('_masked')]:
            rng = np.random.default_rng(0)
            for ref in ('ff_tuned', c + '_masked'):
                if ref not in P: continue
                d = (P[c] - P[ref]).dropna(); ci = np.percentile([rng.choice(d, len(d)).mean() for _ in range(4000)], [2.5, 97.5])
                res[f'{b}: {c} - {ref}'] = dict(mean=round(float(d.mean()), 2), ci95=[round(float(ci[0]), 2), round(float(ci[1]), 2)], better=f'{int((d > 0).sum())}/{len(d)}')
    print(json.dumps(res, indent=1, ensure_ascii=False))
    json.dump(dict(summary=S.reset_index().to_dict('records'), paired=res), open(os.path.splitext(a.out)[0] + '.json', 'w'), indent=1, ensure_ascii=False)


if __name__ == '__main__':
    main()
