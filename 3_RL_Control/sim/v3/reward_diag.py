"""Why the v8 SAC policies scored below the tuned rule: split the training reward (env_v3.step) into its terms per
episode and compare how the reward and the evaluation score (sim/v3/tune_rule.py: CoM reduction - 0.5 x over-time -
100 x over-press) rank the same controllers on the same episodes (eval bank: seeds 1000+, levels 0 / 2).

  source sim/v3/final/env.sh; python -P sim/v3/reward_diag.py --runs v8_s0 v8_s1 --rule models/v8/rule.json --out models/v8/reward_diag.json
"""
import argparse, json, os, sys
import numpy as np

H = os.path.dirname(os.path.abspath(__file__)); RL = os.path.join(H, '..', 'rl'); sys.path.insert(0, RL)
from common import BolsterEnv, EVAL_KINDS, ff_baseline  # noqa: E402
from evaluate import ActorPolicy, wrap  # noqa: E402

TERMS = ('com', 'du', 'theta', 'pitch', 'force', 'trip', 'over')


def episode(spec, seed, level):
    env = BolsterEnv(level=level, seed=seed, scenario_kind=EVAL_KINDS[(seed - 1000) % 4]); o, _ = env.reset()
    pol = (lambda o, e: np.array([-1.0, 0.0])) if spec[0] == 'passive' else \
          (lambda o, e: ff_baseline(o, spec[1], spec[2], env.action_dim)) if spec[0] == 'rule' else wrap(ActorPolicy(spec[1]))
    if hasattr(pol, 'reset'): pol.reset(o)
    W = env.cfg['reward']; F0 = W.get('over_F_N', 20.0); S = {k: 0.0 for k in TERMS}; y2 = yt2 = 0.0; dFs = []; n = 0
    while True:
        a = np.zeros(4); x = np.clip(np.asarray(pol(o, env), float), -1, 1); a[:len(x)] = x; du = np.sum((a - env.a_prev) ** 2)
        o, r, te, tr, info = env.step(x)
        y = (env.body.com_lateral() - env.com0) / W['com_scale_m']; yt = (env.twin.com_lateral() - env.com0) / W['com_scale_m']
        dF = env.added_squeeze_N(); dFs.append(dF); y2 += y * y; yt2 += yt * yt; n += 1
        S['com'] -= W['w_com'] * (y * y - yt * yt); S['du'] -= W['w_du'] * du
        S['theta'] -= W['w_theta'] * np.sum((env.th / env.th_max) ** 2); S['pitch'] -= W.get('w_pitch', 0.0) * np.sum((env.pt / env.pmax) ** 2)
        S['force'] -= W['w_force'] * (dF / W.get('force_scale_N', 100.0)) ** 2; S['trip'] -= W['w_trip'] * env.trip
        S['over'] -= W.get('w_over', 0.0) * env.over_press() ** 2
        if tr: break
    dFs = np.array(dFs); red = 100 * (1 - np.sqrt(y2 / max(yt2, 1e-12)))
    over_t = 100 * np.mean(dFs > F0); over_p = float(np.mean((np.maximum(dFs - F0, 0) / F0) ** 2))
    return dict(S, ret=sum(S.values()), red=red, over_time=over_t, over_press=over_p, score=red - 0.5 * over_t - 100 * over_p,
                passive_rms_cm=2 * np.sqrt(yt2 / n), seed=seed, level=level, kind=EVAL_KINDS[(seed - 1000) % 4])


def _job(j):
    name, spec, seed, level = j; d = episode(spec, seed, level); d['ctrl'] = name; return d


if __name__ == '__main__':
    ap = argparse.ArgumentParser(); ap.add_argument('--runs', nargs='*', default=[]); ap.add_argument('--rule', required=True)
    ap.add_argument('--n', type=int, default=24); ap.add_argument('--workers', type=int, default=12); ap.add_argument('--out', required=True)
    a = ap.parse_args(); b = json.load(open(a.rule))['best']
    ctrls = {'passive': ('passive',), 'rule_tuned': ('rule', b['gain'], int(b['lead_idx']))}
    for r in a.runs: ctrls['sac_' + r] = ('actor', os.path.join(RL, 'runs', r, 'actor_best.pt'))
    jobs = [(k, v, 1000 + s, lv) for lv in (0.0, 2.0) for s in range(a.n) for k, v in ctrls.items()]
    from multiprocessing import Pool
    with Pool(a.workers) as p: rows = p.map(_job, jobs, chunksize=2)
    import pandas as pd
    D = pd.DataFrame(rows); cols = list(TERMS) + ['ret', 'red', 'over_time', 'over_press', 'score']
    mean = D.groupby('ctrl')[cols].mean().round(3); med = D.groupby('ctrl')[cols].median().round(3)
    # how often the reward and the score agree on which of two controllers is better (same episode)
    P = D.set_index(['level', 'seed', 'ctrl']); agree = {}
    for x, y in [('rule_tuned', 'passive')] + [('sac_' + r, 'rule_tuned') for r in a.runs]:
        dr = P.xs(x, level='ctrl').ret - P.xs(y, level='ctrl').ret; ds = P.xs(x, level='ctrl').score - P.xs(y, level='ctrl').score
        agree[f'{x} vs {y}'] = dict(ret_diff_mean=round(float(dr.mean()), 2), score_diff_mean=round(float(ds.mean()), 2),
                                    sign_agreement=round(float(np.mean(np.sign(dr) == np.sign(ds))), 2))
    res = dict(mean=mean.to_dict('index'), median=med.to_dict('index'), agreement=agree,
               passive_com_rms_cm=dict(median=round(float(D[D.ctrl == 'passive'].passive_rms_cm.median()), 3),
                                       p10=round(float(D[D.ctrl == 'passive'].passive_rms_cm.quantile(0.1)), 3),
                                       p90=round(float(D[D.ctrl == 'passive'].passive_rms_cm.quantile(0.9)), 3)),
               note='reward terms summed per 40 s episode (negative = cost); eval bank seeds 1000+, levels 0 / 2')
    json.dump(res, open(a.out, 'w'), indent=1)
    print('mean per episode'); print(mean.to_string()); print('median'); print(med.to_string())
    print(json.dumps(agree, indent=1)); print('passive CoM RMS [cm]', res['passive_com_rms_cm'])
