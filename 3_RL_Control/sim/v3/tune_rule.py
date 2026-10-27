"""Optimise the feed-forward bolster rule in the simulator (rule baseline + residual-RL prior).

Rule (sim/rl/common.ff_baseline): on the a_y predicted (lead+1) x 0.1 s ahead, beyond a deadband,
  differential support d = -gain * sign(a) m, common squeeze c = -1 + both * m, loaded-side pitch.
Score per episode (higher is better), the criterion used for rules and policies alike:
  score = CoM RMS reduction vs passive [%]                          (upper-body CoM of the occupant model)
          - 0.5 x time with added squeeze > over_F_N [%]            ('pressed too hard', vs the open-bolster twin)
          - 100 x mean ((added squeeze - over_F_N) / over_F_N)^2     (how much too hard)
J = 10 % trimmed mean of the scores over the tuning episodes (seeds 2000+, levels 1 and 2).
The optimum is re-checked on validation seeds 3000+ (never the eval bank 1000+).

    OASIS_ENV=v3 OASIS_CALIB=... OASIS_L1_SEQ=... OASIS_DR_OVERLAY=... python sim/v3/tune_rule.py --n 24 --out models/v4/rule.json
    ... python sim/v3/tune_rule.py --grid --n 48 --out models/v6/rule.json     # v6: robust grid (DE overfit the tuning seeds in v4)
"""
import argparse, json, os, sys, time
import numpy as np
from multiprocessing import Pool

H = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(H, '..', 'rl'))
os.environ.setdefault('OASIS_ENV', 'v3')
LAMBDA_TIME, LAMBDA_SQ = 0.5, 100.0
NAMES = ('gain', 'lead_idx', 'both', 'deadband_g', 'pitch')


def episode(job):
    from common import run_episode, ff_baseline, EVAL_KINDS
    seed, level, p, ad = job
    kind = EVAL_KINDS[seed % 4]
    if p is None:
        m = run_episode(lambda o, e: np.array([-1.0, 0.0]), seed, level, kind)
    else:
        g, li, b, db, pt = p
        m = run_episode(lambda o, e: ff_baseline(o, g, int(li), ad, pt, b, db), seed, level, kind)
    return m


def score(m, base):
    red = 100 * (1 - m['rms_cm'] / base['rms_cm'])
    return red - LAMBDA_TIME * m.get('over_time_pct', 0.0) - LAMBDA_SQ * m.get('over_press', 0.0), red


class Objective:
    def __init__(self, pool, seeds, levels, ad):
        self.pool, self.jobs, self.ad = pool, [(s, levels[i % len(levels)]) for i, s in enumerate(seeds)], ad
        self.base = pool.map(episode, [(s, l, None, ad) for s, l in self.jobs]); self.n = 0; self.log = []
    def metrics(self, p):
        M = self.pool.map(episode, [(s, l, tuple(p), self.ad) for s, l in self.jobs])
        sc = [score(m, b) for m, b in zip(M, self.base)]
        S = np.array([x[0] for x in sc]); red = np.array([x[1] for x in sc])
        from scipy.stats import trim_mean
        return dict(J=float(trim_mean(S, 0.1)), score_median=float(np.median(S)), red_median=float(np.median(red)), red_mean=float(red.mean()),
                    red_p10=float(np.percentile(red, 10)), over_time_pct_mean=float(np.mean([m.get('over_time_pct', 0) for m in M])),
                    squeeze_p95_median_N=float(np.median([m.get('squeeze_p95_N', 0) for m in M])),
                    squeeze_max_N=float(np.max([m.get('squeeze_max_N', 0) for m in M])), jerk=float(np.median([m['jerk'] for m in M])))
    def __call__(self, p):
        r = self.metrics(p); self.n += 1; self.log.append(dict(zip(NAMES, map(float, p)), **r))
        if self.n % 10 == 0: print(f'  eval {self.n}: best J so far {max(l["J"] for l in self.log):.3f}', flush=True)
        return -r['J']


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, default=24); ap.add_argument('--out', required=True)
    ap.add_argument('--maxiter', type=int, default=6); ap.add_argument('--popsize', type=int, default=8)
    ap.add_argument('--workers', type=int, default=12)
    ap.add_argument('--grid', action='store_true', help='gain x lead x deadband grid, top 5 re-checked on validation seeds')
    a = ap.parse_args()
    if a.grid: return grid(a)
    from scipy.optimize import differential_evolution
    t0 = time.time(); res = {}
    with Pool(a.workers) as pool:
        tune = Objective(pool, list(range(2000, 2000 + a.n)), [1.0, 2.0], 4)
        ref = (8.0, 6, 0.0, 0.0, 0.0)                                   # v2/v3 tuned rule
        res['reference_rule_tuning'] = dict(zip(NAMES, ref), **tune.metrics(ref))
        bounds = [(0.0, 20.0), (3, 9), (0.0, 10.0), (0.0, 0.08), (-1.0, 1.0)]
        de = differential_evolution(tune, bounds, integrality=[False, True, False, False, False], maxiter=a.maxiter,
                                    popsize=a.popsize, seed=0, polish=False, init='sobol', x0=np.array(ref, float), tol=1e-3)
        best = [float(x) for x in de.x]; best[1] = int(round(best[1]))
        nopitch = list(best); nopitch[4] = 0.0
        res['best'] = dict(zip(NAMES, best)); res['best_tuning'] = tune.metrics(best); res['best_nopitch_tuning'] = tune.metrics(nopitch)
        val = Objective(pool, list(range(3000, 3000 + a.n)), [1.0, 2.0], 4)
        res['validation'] = {'reference_rule': val.metrics(ref), 'best': val.metrics(best), 'best_without_pitch': val.metrics(nopitch)}
        res['evals'] = tune.n; res['log'] = tune.log
    res['criterion'] = dict(score='reduction_pct - %.1f * over_time_pct - %.0f * over_press' % (LAMBDA_TIME, LAMBDA_SQ),
                            J='10 %% trimmed mean over %d tuning episodes (seeds 2000+, levels 1 / 2)' % a.n,
                            over_F_N_from='reward.over_F_N of the active config')
    res['wall_s'] = round(time.time() - t0)
    json.dump(res, open(a.out, 'w'), indent=1)
    print(json.dumps({k: v for k, v in res.items() if k != 'log'}, indent=1))


def grid(a):
    """Robust alternative to DE (v4: DE's optimum lost on validation): coarse grid, both = pitch = 0; the top 5 by
    tuning J are re-scored on validation seeds and the best VALIDATION J is chosen."""
    import itertools
    t0 = time.time(); res = {}
    # v8: leads up to 1.0 s (the OASIS_FALLBACK v2 actuator moves ~5 deg/s); v7 grid: gains 4-14, leads 0-6
    G = list(itertools.product([6.0, 8.0, 10.0, 12.0, 14.0, 16.0], [2, 3, 4, 6, 8, 9], [0.0, 0.02]))
    with Pool(a.workers) as pool:
        tune = Objective(pool, list(range(2000, 2000 + a.n)), [1.0, 2.0], 4)
        rows = [dict(gain=g, lead_idx=li, deadband_g=db, **tune.metrics((g, li, 0.0, db, 0.0))) for g, li, db in G]
        top = sorted(rows, key=lambda r: -r['J'])[:5]
        val = Objective(pool, list(range(3000, 3000 + a.n)), [1.0, 2.0], 4)
        for r in top: r['validation'] = val.metrics((r['gain'], r['lead_idx'], 0.0, r['deadband_g'], 0.0))
        ref = (8.0, 6, 0.0, 0.0, 0.0)
        res['reference_rule'] = dict(zip(NAMES, ref), tuning=tune.metrics(ref), validation=val.metrics(ref))
    b = max(top, key=lambda r: r['validation']['J'])
    res['best'] = dict(gain=b['gain'], lead_idx=b['lead_idx'], both=0.0, deadband_g=b['deadband_g'], pitch=0.0)
    res['best_tuning'] = {k: v for k, v in b.items() if k not in ('gain', 'lead_idx', 'deadband_g', 'validation')}
    res['validation'] = {'best': b['validation'], 'reference_rule': res['reference_rule']['validation']}
    res['top5'] = top; res['grid'] = rows
    res['criterion'] = dict(score='reduction_pct - %.1f * over_time_pct - %.0f * over_press' % (LAMBDA_TIME, LAMBDA_SQ),
                            J='10 %% trimmed mean over %d episodes (tuning seeds 2000+, validation 3000+, levels 1 / 2)' % a.n)
    res['wall_s'] = round(time.time() - t0)
    json.dump(res, open(a.out, 'w'), indent=1)
    print(json.dumps({k: v for k, v in res.items() if k not in ('grid',)}, indent=1))


if __name__ == '__main__':
    main()
