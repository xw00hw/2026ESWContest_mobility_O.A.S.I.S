"""One short scenario, three controllers on the SAME occupant / car / sensor draw (same seed): passive (bolsters
open), the tuned rule, the SAC policy. Writes the time series a front-view animation needs (JSON).

  per step (20 Hz): t, a_y (car) and the a_y the policy saw 0.3 s ahead, CoM shift (occupant and its open-bolster
  twin), torso roll, pelvis shift, bolster yaw / pitch L R (reported), pad zone forces, added squeeze, pad counts

    OASIS_ENV=v3 ... python -P sim/v3/scenario_trace.py --run v6_s0 --rule models/v6/rule.json --seed 1001 --kind lane_change \
        --window 14 --out models/v6/scenario_trace.json
"""
import argparse, json, os, sys
import numpy as np

H = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(H, '..', 'rl'))
G = 9.81


def rollout(pol, seed, level, kind, reset=None):
    from common import BolsterEnv
    e = BolsterEnv(level=level, seed=seed, scenario_kind=kind); o, info = e.reset()
    if reset: reset(o)
    R = {k: [] for k in ('t', 'ay', 'ay_seen', 'com', 'com_twin', 'phi', 'y_pelvis', 'yawL', 'yawR', 'pitchL', 'pitchR',
                         'F_L_lo', 'F_L_hi', 'F_R_lo', 'F_R_hi', 'added_N', 'cnt_R_hi', 'cnt_R_lo', 'cnt_L_hi', 'cnt_L_lo', 'flag')}
    while True:
        a = pol(o, e); o, r, te, tr, info = e.step(a); p = info['privileged']
        if hasattr(pol, 'post'): pol.post(o)
        vals = dict(t=e.t, ay=e._ay(e.t) / G, ay_seen=float(o[2]), com=100 * p['y_com'], com_twin=100 * p['y_com_twin'],
                    phi=float(np.rad2deg(p['phi'])), y_pelvis=100 * p['y_pelvis'], yawL=float(e.th[0]), yawR=float(e.th[1]),
                    pitchL=float(e.pt[0]), pitchR=float(e.pt[1]), F_L_lo=p['F_L_lo'], F_L_hi=p['F_L_hi'], F_R_lo=p['F_R_lo'], F_R_hi=p['F_R_hi'],
                    added_N=p.get('added_squeeze_N', 0.0), flag=float(o[56]) if len(o) > 56 else 0.0,
                    **{f'cnt_{s}_{z}': float(e.counts[(s, z)] - e.base[(s, z)]) for s, z in e.counts})
        for k, v in vals.items(): R[k].append(round(float(v), 4))
        if tr: break
    meta = dict(kind=info['params']['kind'], arch=info['params']['archetype'], shoulder_m=round(info['params']['profile']['shoulder_width_m'], 3),
                torso_m=round(info['params']['profile']['sitting_torso_m'], 3), mass_kg=round(info['params']['profile']['mass_kg'], 1),
                clearance_cm=round(100 * getattr(e, 'clearance_m', 0.0), 2), preload_N=round(info['params']['preload_N'], 1),
                h_bolster_m=round(e.hp.h_bolster, 3), zone_dh_m=round(e.v3p['zone_dh'], 3), h_com_m=round(e.hp.h_com, 3))
    return R, meta


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--run', required=True); ap.add_argument('--rule', required=True)
    ap.add_argument('--seed', type=int, default=1001); ap.add_argument('--kind', default='lane_change'); ap.add_argument('--level', type=float, default=1.0)
    ap.add_argument('--window', type=float, default=14.0); ap.add_argument('--out', required=True)
    a = ap.parse_args()
    from common import ff_baseline
    from evaluate import ActorPolicy
    b = json.load(open(a.rule))['best']
    rule = lambda o, e: ff_baseline(o, b['gain'], int(b['lead_idx']), 2, 0.0, b.get('both', 0.0), b.get('deadband_g', 0.0))
    P = ActorPolicy(os.path.join(H, '..', 'rl', 'runs', a.run, 'actor_best.pt'))
    out = {}
    for name, pol, rs in (('passive', lambda o, e: np.array([-1.0, 0.0]), None), ('rule', rule, None), ('rl', P, P.reset)):
        out[name], meta = rollout(pol, a.seed, a.level, a.kind, rs)
    t = np.array(out['passive']['t']); ay = np.abs(np.array(out['passive']['ay'])); n = int(a.window / 0.05)
    k = np.convolve(ay, np.ones(n), 'valid'); i0 = int(np.argmax(k))                # the busiest window
    for name in out: out[name] = {kk: v[i0:i0 + n] for kk, v in out[name].items()}
    rms = lambda x: float(np.sqrt(np.mean(np.square(x))))
    summ = {nm: dict(com_rms_cm=round(rms(out[nm]['com']), 3), com_peak_cm=round(float(np.max(np.abs(out[nm]['com']))), 3),
                     added_max_N=round(float(np.max(out[nm]['added_N'])), 1)) for nm in out}
    for nm in ('rule', 'rl'): summ[nm]['com_reduction_pct'] = round(100 * (1 - summ[nm]['com_rms_cm'] / summ['passive']['com_rms_cm']), 1)
    json.dump(dict(seed=a.seed, kind=a.kind, level=a.level, run=a.run, rule=b, window_s=[round(float(t[i0]), 2), round(float(t[i0] + a.window), 2)],
                   occupant=meta, summary=summ, traces=out), open(a.out, 'w'))
    print(json.dumps(dict(window=[float(t[i0]), float(t[i0] + a.window)], occupant=meta, summary=summ), indent=1))


if __name__ == '__main__':
    main()
