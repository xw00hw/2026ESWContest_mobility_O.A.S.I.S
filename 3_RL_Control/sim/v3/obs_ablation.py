"""Which observations does a policy actually use? Closed-loop ablation: one observation group at a time is
replaced by its mean over typical histories (deploy/model_*/calib_raw.npy, newest frame), history included.
Score / CoM reduction / over-press drop vs the intact policy on the eval bank (seeds 1000+, paired).

    OASIS_ENV=v3 ... python sim/v3/obs_ablation.py --run v5_yaw_s0 --model deploy/model_v5 --n 24 --out models/v5/obs_ablation.json
"""
import argparse, json, os, sys
import numpy as np, pandas as pd
from multiprocessing import Pool

H = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(H, '..', 'rl'))
GROUPS = {'none': [], 'a_y/a_x preview': list(range(0, 20)), 'vehicle now (a_y, a_x, yaw rate, speed, steer)': list(range(20, 25)),
          'pressure (side pads, back, seat, R-L)': list(range(25, 37)), 'pose (torso lean, shoulder shift)': [37, 38],
          'reported bolster angles': list(range(39, 43)), 'body build + rest pressure': list(range(47, 54))}


def job(args):
    seed, level, group, ck, fill = args
    from common import run_episode, EVAL_KINDS
    from evaluate import ActorPolicy
    idx = GROUPS.get(group, []); kind = EVAL_KINDS[seed % 4]
    if group == 'passive':
        m = run_episode(lambda o, e: np.array([-1.0, 0.0]), seed, level, kind)
    else:
        P = ActorPolicy(ck); st = {'first': True}
        def msk(o):
            o = o.copy(); o[idx] = fill[idx]; return o
        def pol(o, e):
            oo = msk(o)
            if not st['first']: P.hist.push(oo)
            st['first'] = False
            return P(oo, e)
        pol.reset = lambda o: (P.reset(msk(o)), st.update(first=True))
        m = run_episode(pol, seed, level, kind)
    return dict(seed=seed, level=level, group=group, rms=m['rms_cm'], over=m.get('over_time_pct', 0.0), op=m.get('over_press', 0.0))


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--run', required=True); ap.add_argument('--model', required=True)
    ap.add_argument('--n', type=int, default=24); ap.add_argument('--out', required=True)
    a = ap.parse_args()
    X = np.load(os.path.join(a.model, 'calib_raw.npy')); OD = 57; fill = X[:, -OD:].mean(0)
    ck = os.path.join(H, '..', 'rl', 'runs', a.run, 'actor_best.pt')
    jobs = [(1000 + s, [1.0, 2.0][s % 2], g, ck, fill) for s in range(a.n) for g in ['passive'] + list(GROUPS)]
    with Pool(12) as pool:
        D = pd.DataFrame(pool.map(job, jobs, chunksize=2))
    base = D[D.group == 'passive'].set_index('seed').rms
    D['red'] = 100 * (1 - D.rms / D.seed.map(base)); D['score'] = D.red - 0.5 * D.over - 100 * D.op
    P = D[D.group != 'passive'].pivot_table(index='seed', columns='group', values=['score', 'red', 'over'])
    from scipy.stats import trim_mean
    out = {}
    for g in GROUPS:
        d = P['score'][g] - P['score']['none']; rng = np.random.default_rng(0)
        ci = np.percentile([rng.choice(d, len(d)).mean() for _ in range(4000)], [2.5, 97.5])
        out[g] = dict(J=round(float(trim_mean(P['score'][g], 0.1)), 2), d_score=round(float(d.mean()), 2), ci95=[round(float(ci[0]), 2), round(float(ci[1]), 2)],
                      d_red=round(float((P['red'][g] - P['red']['none']).mean()), 2), d_over=round(float((P['over'][g] - P['over']['none']).mean()), 2))
        print(f"{g:48s} J {out[g]['J']:6.2f} | score change {out[g]['d_score']:+6.2f} [{ci[0]:+.2f}, {ci[1]:+.2f}] | CoM red {out[g]['d_red']:+5.2f} pp | over {out[g]['d_over']:+5.2f} pp")
    json.dump(dict(run=a.run, n=a.n, masked_with='mean of the newest frame of calib_raw histories', groups=out), open(a.out, 'w'), indent=1, ensure_ascii=False)


if __name__ == '__main__':
    main()
