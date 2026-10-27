"""Eval-bank summary (paired seeds): J (10 % trimmed mean score), mean CoM reduction, over-press time, max added
squeeze per level and controller, plus paired differences of every SAC run vs a reference controller.

    python sim/v3/summarize_eval.py sim/rl/eval_bank_v6.csv --ref rule_tuned --out sim/v3/eval_v6_summary.json
"""
import argparse, json
import numpy as np, pandas as pd
from scipy.stats import trim_mean


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('csv'); ap.add_argument('--ref', default='rule_tuned'); ap.add_argument('--out', required=True)
    a = ap.parse_args()
    D = pd.read_csv(a.csv); out = {'eval_bank': {}, 'paired': {}}
    for lv, g in D.groupby('level'):
        out['eval_bank'][str(lv)] = {c: dict(J=round(float(trim_mean(x.score, 0.1)), 2), red_mean=round(float(x.reduction_pct.mean()), 2),
                                             red_p10=round(float(np.percentile(x.reduction_pct, 10)), 2),
                                             over_time_pct=round(float(x.over_time_pct.mean()), 2), squeeze_max_N=round(float(x.squeeze_max_N.max()), 1),
                                             trips=round(float(x.trips.mean()), 3))
                                     for c, x in g.groupby('ctrl') if c != 'passive'}
        P = g.pivot_table(index='seed', columns='ctrl', values='score')
        for c in [c for c in P.columns if c.startswith('sac_')] + [c for c in P.columns if c in ('ff_tuned',)]:
            if a.ref not in P or c == a.ref: continue
            d = (P[c] - P[a.ref]).dropna().values; rng = np.random.default_rng(0)
            ci = np.percentile([rng.choice(d, len(d)).mean() for _ in range(4000)], [2.5, 97.5])
            out['paired'][f'L{lv}: {c} - {a.ref}'] = dict(mean=round(float(d.mean()), 2), ci95=[round(float(ci[0]), 2), round(float(ci[1]), 2)],
                                                         better=f'{int((d > 0).sum())}/{len(d)}')
    allp = D.pivot_table(index=['level', 'seed'], columns='ctrl', values='score')
    for c in [c for c in allp.columns if c.startswith('sac_')]:
        if a.ref not in allp: break
        d = (allp[c] - allp[a.ref]).dropna().values; rng = np.random.default_rng(1)
        ci = np.percentile([rng.choice(d, len(d)).mean() for _ in range(4000)], [2.5, 97.5])
        out['paired'][f'all: {c} - {a.ref}'] = dict(mean=round(float(d.mean()), 2), ci95=[round(float(ci[0]), 2), round(float(ci[1]), 2)],
                                                   better=f'{int((d > 0).sum())}/{len(d)}')
    json.dump(out, open(a.out, 'w'), indent=1)
    for lv, t in out['eval_bank'].items():
        print(f'--- level {lv}'); print(pd.DataFrame(t).T.sort_values('J', ascending=False).to_string())
    print(json.dumps(out['paired'], indent=1))


if __name__ == '__main__':
    main()
