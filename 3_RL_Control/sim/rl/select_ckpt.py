"""Pick the best snapshot of a run on VALIDATION seeds (3000+), never on the
evaluation bank (1000+), then copy it to runs/<tag>/actor_best.pt.

  python select_ckpt.py resid_v2f --overlay ../overlays/safety_v2.yaml --n 8 --last 6
  python select_ckpt.py v8_s0 --criterion score --n 48 --levels 1 2 --last 6 --prior 16 6 --workers 12   # v8: the rule's
      validation episodes (sim/v3/tune_rule.py), with the prior rule itself scored as the reference
"""
import argparse, glob, json, os, shutil
import numpy as np, pandas as pd

ap = argparse.ArgumentParser()
ap.add_argument('tag'); ap.add_argument('--overlay'); ap.add_argument('--n', type=int, default=8)
ap.add_argument('--last', type=int, default=6, help='only the last N snapshots')
ap.add_argument('--level', type=float, default=2.0)
ap.add_argument('--levels', nargs='*', type=float, help='alternate these levels over the seeds (rule validation: 1 2); default --level')
ap.add_argument('--criterion', default='ret', choices=['ret', 'score'],
                help="ret: median episode return; score: 10 %% trimmed mean of CoM reduction vs passive minus the "
                     "'pressed too hard' terms (sim/v3/tune_rule.py, same criterion as the rule tuning)")
ap.add_argument('--prior', nargs=2, type=float, metavar=('GAIN', 'LEAD_IDX'), help='also score the prior rule (no correction)')
ap.add_argument('--workers', type=int, default=1)
a = ap.parse_args()
if a.overlay:
    os.environ['OASIS_DR_OVERLAY'] = os.path.abspath(a.overlay)
from common import run_episode, ff_baseline, EVAL_KINDS   # noqa: E402  (env reads the overlay at construction)
from evaluate import ActorPolicy, wrap                    # noqa: E402

here = os.path.dirname(os.path.abspath(__file__))
LEVELS = a.levels or [a.level]
EPIS = [(3000 + s, LEVELS[s % len(LEVELS)], EVAL_KINDS[s % 4]) for s in range(a.n)]


def _job(j):
    cand, seed, level, kind = j
    if cand == 'passive': pol = lambda o, e: np.array([-1.0, 0.0])
    elif cand == 'prior_rule': pol = lambda o, e: ff_baseline(o, a.prior[0], int(a.prior[1]), 2)
    else: pol = wrap(ActorPolicy(os.path.join(here, 'runs', a.tag, 'snap', cand)))
    m = run_episode(pol, seed, level, kind); m['snap'] = cand; return m


if __name__ == '__main__':
    snaps = [os.path.basename(p) for p in sorted(glob.glob(os.path.join(here, 'runs', a.tag, 'snap', 'actor_*.pt')))[-a.last:]]
    cands = snaps + (['prior_rule'] if a.prior else []) + (['passive'] if a.criterion == 'score' else [])
    jobs = [(c, *e) for c in cands for e in EPIS]
    if a.workers > 1:
        from multiprocessing import Pool
        with Pool(a.workers) as pool: rows = pool.map(_job, jobs, chunksize=1)
    else:
        rows = [_job(j) for j in jobs]
    D = pd.DataFrame(rows)
    if a.criterion == 'score':
        from scipy.stats import trim_mean
        base = D[D.snap == 'passive'].set_index('seed').rms_cm; D = D[D.snap != 'passive'].copy()
        D['reduction_pct'] = 100 * (1 - D.rms_cm / D.seed.map(base))
        D['score'] = D.reduction_pct - 0.5 * D.get('over_time_pct', 0.0) - 100.0 * D.get('over_press', 0.0)
        S = D.groupby('snap').agg(score=('score', lambda x: trim_mean(x, 0.1)), red=('reduction_pct', 'median'), ret=('ret', 'median'),
                                  trips=('trips', 'mean')).sort_values('score')
    else:
        S = D.groupby('snap').agg(ret=('ret', 'median'), rms=('rms_cm', 'median'), trips=('trips', 'mean')).sort_values('ret')
    print(S.round(3).to_string())
    best = [s for s in S.index if s != 'prior_rule'][-1]
    shutil.copy(os.path.join(here, 'runs', a.tag, 'snap', best), os.path.join(here, 'runs', a.tag, 'actor_best.pt'))
    D.to_csv(os.path.join(here, 'runs', a.tag, 'select_ckpt.csv'), index=False)
    out = dict(best=best, levels=LEVELS, n=a.n, criterion=a.criterion, table=S.round(4).to_dict('index'))
    if a.prior and a.criterion == 'score':
        P = D.pivot_table(index='seed', columns='snap', values='score'); d = P[best] - P['prior_rule']
        out['vs_prior'] = dict(J_best=float(S.loc[best, 'score']), J_prior=float(S.loc['prior_rule', 'score']),
                               paired_mean=round(float(d.mean()), 3), better=f'{int((d > 0).sum())}/{len(d)}')
        print('vs prior rule:', out['vs_prior'])
    json.dump(out, open(os.path.join(here, 'runs', a.tag, 'select_ckpt.json'), 'w'), indent=1)
    print('best', best)
