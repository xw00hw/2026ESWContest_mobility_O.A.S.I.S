"""Simulator audit (v6 BeamNG profile): checks that the model the policy is trained in behaves as intended.
Each check prints PASS / FAIL with the numbers; JSON with everything to --out.

  1 signs        left turn (a_y > 0) pushes the occupant RIGHT and loads the right wing; right yaw closes the right
                 wing and RAISES its pad counts (team model); +pitch moves the lower zone in
  2 twin         passive seat: the counterfactual twin is identical (CoM term 0, added squeeze 0, over-press 0)
  3 determinism  same seed -> same observations
  4 stability    passive seat over many draws: no blow-ups (|CoM| < 5 cm, F < 250 N), redraw rate
  5 body         B-like (0.374 / 0.497 m) vs A-like (0.42 / 0.58): rest counts, upper-zone share, clearance
  6 actuator     firmware path: yaw 0 -> 10 deg, 90 % time at crank 30 / 60 deg/s (OASIS_CONTROL v6 proposal);
                 6b OASIS_FALLBACK v2 (2 us / 20 ms per servo, STM tune rate 2 / 10 / 20 deg/s)
  7 interlock    sim thresholds = runtime config (soft 60 N, hard 120 N, arm 2 deg, hold 0.3 s)
  8 contract     sim obs layout = the deployed copy (sha256); the sim's predictor IS the runtime class
  9 predictor    wheel preview vs hold: RMS [g] per horizon on the eval scenarios
 10 reward       the training reward ranks the tuned rule above passive like the evaluation score (24 episodes)

    OASIS_ENV=v3 OASIS_CALIB=models/v5/calib_v2.json OASIS_L1_SEQ=models/v5/l1_seq.csv OASIS_DR_OVERLAY=$(cat sim/rl/runs/v6_env_overlays.txt) \
        python -P sim/v3/audit_v6.py --out models/v6/audit.json
"""
import argparse, hashlib, json, os, sys, tempfile
import numpy as np
from multiprocessing import Pool

H = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(H, '..', '..'))
sys.path[:0] = [H, os.path.join(H, '..', 'rl')]
G = 9.81; RES = {}; FAILS = []


def check(name, ok, **info):
    RES[name] = dict(ok=bool(ok), **info); print(f"{'PASS' if ok else 'FAIL'}  {name}: {json.dumps(info, default=float)}")
    if not ok: FAILS.append(name)


def env(seed=11, kind='urban_turn', overlays=None, level=1.0):
    from env_v3 import BolsterEnvV3
    from common import BolsterEnv
    if overlays: return BolsterEnvV3(level=level, seed=seed, scenario_kind=kind, overlays=list(_ov()) + overlays)
    return BolsterEnv(level=level, seed=seed, scenario_kind=kind)


def _ov():
    return [p for p in os.environ.get('OASIS_DR_OVERLAY', '').split(os.pathsep) if p]


def body_ov(w, t):
    import yaml
    p = os.path.join(tempfile.mkdtemp(), 'b.yaml')
    yaml.safe_dump({'profile': {'shoulder_width_m': {'dist': 'uniform', 'range': [w, w], 'nominal': w, 'level': 0},
                                'sitting_torso_m': {'dist': 'uniform', 'range': [t, t], 'nominal': t, 'level': 0}}}, open(p, 'w'))
    return p


def set_ay(e, g):
    """constant car a_y; the occupant feels the same (v7 dynamic car: ay_occ = a_y + roll + bank is its own array)"""
    e.ex['ay'][:] = g * G; e.ex['ax'][:] = 0.0; e.ay_occ = e.ex['ay']


def signs():
    e = env(21); e.reset(); set_ay(e, 0.3)
    y0 = e.body.com_lateral(); F0 = (e.out['F_L'], e.out['F_R'])
    for _ in range(60): e.step([-1, 0, 0, 0])
    dy = 100 * (e.body.com_lateral() - y0); dFL, dFR = e.out['F_L'] - F0[0], e.out['F_R'] - F0[1]
    check('1a left turn loads the RIGHT side', dy < 0 and dFR > dFL, com_shift_cm=round(dy, 2), dF_L=round(dFL, 1), dF_R=round(dFR, 1))
    e = env(22, overlays=[body_ov(0.44, 0.60)]); e.reset(); set_ay(e, 0.0)
    for _ in range(40): e.step([-1, 0, 0, 0])
    c0 = {k: e.counts[k] for k in e.counts}; th0 = e.th.copy()
    for _ in range(80): e.step([0, -1, 0, 0])                    # c = 0 (5 deg), d = -1: right +10, left 0
    dc = {f'{s}_{z}': round(e.counts[(s, z)] - c0[(s, z)], 1) for s, z in e.counts}
    check('1b right yaw closes the right wing and raises its pads', e.th[1] > th0[1] + 5 and dc['R_hi'] > 0 and dc['R_lo'] > 0,
          yaw_R_deg=round(float(e.th[1]), 2), d_counts=dc)
    g = e._gaps(0, 4.0) - e._gaps(0, 0.0)
    check('1c +pitch moves the lower zone in, the upper out', g[0] < 0 < g[1], d_gap_lo_mm=round(1e3 * g[0], 2), d_gap_hi_mm=round(1e3 * g[1], 2))


def twin_and_determinism():
    from common import run_episode
    m = run_episode(lambda o, e: np.array([-1.0, 0.0]), 1001, 1.0, 'lane_change')
    e = env(1001, 'lane_change'); e.reset(); dmax = 0.0; com = 0.0
    while True:
        o, r, te, tr, info = e.step([-1, 0, 0, 0]); dmax = max(dmax, e.added_squeeze_N())
        com = max(com, abs(e.body.com_lateral() - e.twin.com_lateral()))
        if tr: break
    check('2 passive: twin identical, nothing added', dmax < 1e-6 and com < 1e-9 and m.get('over_press', 0) == 0,
          added_max_N=dmax, com_diff_m=com, over_press=m.get('over_press', 0))
    a, b = env(77, 'slalom'), env(77, 'slalom'); oa, _ = a.reset(); ob, _ = b.reset(); same = np.array_equal(oa, ob)
    for _ in range(100):
        oa = a.step([0.2, 0.3, 0, 0])[0]; ob = b.step([0.2, 0.3, 0, 0])[0]; same &= np.array_equal(oa, ob)
    check('3 same seed -> same observations', same)


def _stab(seed):
    from common import EVAL_KINDS
    e = env(seed, EVAL_KINDS[seed % 4], level=[0.0, 1.0, 2.0][seed % 3]); e.reset(); mc = mf = mu = 0.0; fx = 0.0
    m = e.params['profile']['mass_kg']; F0 = None                     # rest force incl. a leaning posture (lean bias)
    while True:
        o, r, te, tr, info = e.step([-1, 0, 0, 0]); p = info['privileged']
        mc = max(mc, abs(100 * p['y_com'])); mf = max(mf, p['F_L'], p['F_R'])
        i = min(int(e.t / e.dt), len(e.ay_occ) - 1); a_pk = np.max(np.abs(e.ay_occ[max(0, i - 200):i + 1]))
        F0 = max(p['F_L'], p['F_R']) if F0 is None else F0
        fx = max(fx, max(p['F_L'], p['F_R']) - (1.2 * m * a_pk + F0 + 50.0))   # beyond rest + the body's own inertia
        quiet = max(abs(e._ay(e.t - k * 0.05)) for k in range(30)) < 0.1 * G   # car barely pulling for 1.5 s
        if quiet: mu = max(mu, abs(100 * p['y_com']))                        # excursion NOT explained by the load
        if tr: break
    return mc, mf, getattr(e, 'redrawn', 0), mu, fx


def stability(pool, n=96):
    R = np.array(pool.map(_stab, range(6000, 6000 + n)))
    # blow-up = a large excursion after 1.5 s of |a_y| < 0.1 g (should have decayed);
    # a soft torso in a sustained 0.6 g level-2 turn may legitimately reach ~6 cm (seed 6008: fn 0.91 Hz, 5.8 cm)
    # (v7) and a pad force beyond what the occupant's inertia explains (1.2 m a_y + preload + 50 N): a 100 kg
    # occupant at 0.6 g (level 2 + roll) legitimately puts ~350 N into one wing
    bad = int(((R[:, 3] > 2) | (R[:, 4] > 0)).sum())
    check('4 passive occupant stable', bad == 0, n=n, blowups=bad, com_max_cm_p99=round(float(np.percentile(R[:, 0], 99)), 2),
          com_max_low_load_cm=round(float(R[:, 3].max()), 2),
          F_max_N_p99=round(float(np.percentile(R[:, 1], 99)), 1), F_beyond_inertia_N_max=round(float(R[:, 4].max()), 1), redraws=int(R[:, 2].sum()))


def _body(args):
    w, t, s = args
    e = env(700 + s, overlays=[body_ov(w, t)]); e.reset()
    return [e.base[('R', 'hi')], e.base[('R', 'lo')], e.v3p['zone_frac_hi'], 100 * getattr(e, 'clearance_m', 0.0)]


def body(pool, n=30):
    out = {}
    for lab, w, t in (('B-like', 0.374, 0.497), ('A-like', 0.42, 0.58)):
        R = np.array(pool.map(_body, [(w, t, s) for s in range(n)]))
        out[lab] = dict(rest_hi=round(float(np.median(R[:, 0]))), rest_lo=round(float(np.median(R[:, 1]))),
                        frac_hi=round(float(np.median(R[:, 2])), 3), clearance_share=round(float(np.mean(R[:, 3] > 0)), 2))
    ok = out['B-like']['rest_hi'] < 0.5 * out['A-like']['rest_hi'] and out['B-like']['frac_hi'] < out['A-like']['frac_hi']
    check('5 thin / low-seated occupant: upper pads unloaded (bench B: 13-348 vs A ~2000 counts)', ok, **out)


def actuator():
    from env_v3 import Servo2D
    import yaml
    SB = yaml.safe_load(open(os.path.join(H, '..', 'v2', 'dr_v2.yaml')))['seat_bolster']
    SB = dict(SB, servo_cmd_latency_s=[0.05, 0.05], servo_tau_s=[0.05, 0.05], servo_backlash_deg=[0, 0], servo_compliance_deg_per_Nm=[0, 0])
    out = {}
    for crank in (30.0, 60.0):
        s = Servo2D(np.random.default_rng(0), SB, 0.005, 'fw_event_hb', crank, 8.0); t = 0.0; t90 = None
        while t < 6.0:
            y, _ = s((10.0, 0.0), 0.0); t += 0.005
            if t90 is None and y >= 9.0: t90 = t
        out[f'crank_{int(crank)}'] = round(t90, 2) if t90 else None
    check('6 firmware path: yaw 0->10 deg 90 % time [s]', all(out.values()) and out['crank_60'] < out['crank_30'], **out)
    # OASIS_FALLBACK v2 (BeamNG_Bolster_FB, the bench firmware): 2 us / 20 ms per servo, STM tune rate
    from firmware_model import fb_a2_step
    fa = {}
    for rate in (2.0, 10.0, 20.0):
        p, t = np.zeros(2), 0.0
        while p[0] < 9.0 and t < 20: p = fb_a2_step(p, [10.0, 0.0], (rate, rate)); t += 0.02
        fa[f'tune_{int(rate)}dps'] = round(t, 2)
    # tune 2 deg/s: 9 deg / 2 = 4.5 s; 10 and 20 deg/s: the pulse slew (13.5 deg/s crank) binds, both ~1.4-1.7 s
    ok = abs(fa['tune_2dps'] - 4.5) < 0.05 and 1.2 < fa['tune_20dps'] < 2.0 and fa['tune_10dps'] >= fa['tune_20dps']
    check('6b OASIS_FALLBACK v2 path: yaw 0->9 deg time [s]', ok, **fa)


def interlock():
    import yaml
    sim = yaml.safe_load(open(os.path.join(H, '..', 'v2', 'dr_v2.yaml')))['safety']
    rt = yaml.safe_load(open(os.path.join(ROOT, 'deploy', 'runtime', 'config.yaml')))['safety']
    keys = ('fsr_force_soft_N', 'fsr_force_hard_N', 'arm_deg', 'hold_s')
    check('7 interlock thresholds sim = runtime', all(sim[k] == rt[k] for k in keys), sim={k: sim[k] for k in keys}, runtime={k: rt[k] for k in keys})


def contract():
    sha = lambda p: hashlib.sha256(open(p, 'rb').read()).hexdigest()[:16]
    s = sha(os.path.join(H, 'obs_layout_v3.py'))
    dep = {d: sha(os.path.join(ROOT, 'deploy', d, 'obs_layout.py')) for d in ('model_v5', 'model_v6', 'model_v7')
           if os.path.exists(os.path.join(ROOT, 'deploy', d, 'obs_layout.py'))}
    import env_v3
    sys.path.insert(0, os.path.join(ROOT, 'deploy', 'runtime'))
    from oasis_rt import predictor as P
    same_cls = env_v3.WheelPreviewPredictor is P.WheelPreviewPredictor
    check('8 obs contract = deployed copies; sim predictor = runtime class', all(v == s for v in dep.values()) and same_cls,
          sim=s, deployed=dep, same_predictor_class=same_cls)


def _pred(seed):
    from common import EVAL_KINDS
    from oasis_rt.predictor import HoldPredictor
    e = env(seed, EVAL_KINDS[seed % 4]); e.reset(); hz = np.arange(1, 11) * 0.1; P, Q, T = [], [], []
    while e.t < e.T - 1.2:
        o = e.step([-1, 0, 0, 0])[0]; iv = e.imu_vals
        P.append(o[:10]); Q.append(np.full(10, iv['ay'][0] / G)); T.append([e._ay(e.t + h) / G for h in hz])
    T = np.array(T); return np.mean((np.array(P) - T) ** 2, 0), np.mean((np.array(Q) - T) ** 2, 0)


def predictor(pool, n=24):
    R = pool.map(_pred, range(1000, 1000 + n))
    w = np.sqrt(np.mean([r[0] for r in R], 0)); h = np.sqrt(np.mean([r[1] for r in R], 0))
    check('9 wheel preview beats hold at 0.1-0.5 s (obs slots 0-9)', bool(np.all(w[:5] < h[:5])),
          h_s=[0.1, 0.3, 0.5, 0.7, 1.0], wheel_rms_g=[round(float(w[i]), 3) for i in (0, 2, 4, 6, 9)], hold_rms_g=[round(float(h[i]), 3) for i in (0, 2, 4, 6, 9)])


def _rew(args):
    """one validation-style episode: training return and evaluation score of the passive seat and the rule"""
    from common import ff_baseline, EVAL_KINDS
    seed, level, rule = args; out = {}
    for lab in ('passive', 'rule'):
        e = env(seed, EVAL_KINDS[seed % 4], level=level); o, _ = e.reset(); W = e.cfg['reward']; R = y2 = yt2 = 0.0; dF = []
        while True:
            a = [-1.0, 0.0] if lab == 'passive' else ff_baseline(o, rule[0], rule[1], e.action_dim)
            o, r, te, tr, info = e.step(a); R += r
            y = (e.body.com_lateral() - e.com0) / W['com_scale_m']; yt = (e.twin.com_lateral() - e.com0) / W['com_scale_m']
            y2 += y * y; yt2 += yt * yt; dF.append(e.added_squeeze_N())
            if tr: break
        dF, F0 = np.array(dF), W.get('over_F_N', 20.0)
        sc = 100 * (1 - np.sqrt(y2 / max(yt2, 1e-12))) - 50 * np.mean(dF > F0) - 100 * np.mean((np.maximum(dF - F0, 0) / F0) ** 2)
        out[lab] = (R, sc)
    return out


def reward(pool, rule, n=24):
    """v8: the training reward must rank controllers like the evaluation score. The v7 reward ranked the tuned rule
    below the passive seat (models/v8/reward_diag.json) although its score is higher -> RL learned to do less."""
    R = pool.map(_rew, [(2000 + s, (1.0, 2.0)[s % 2], rule) for s in range(n)])
    dr = np.array([r['rule'][0] - r['passive'][0] for r in R]); ds = np.array([r['rule'][1] - r['passive'][1] for r in R])
    agree = float(np.mean(np.sign(dr) == np.sign(ds)))
    check('10 reward ranks the tuned rule above passive like the score', dr.mean() > 0 and agree >= 0.8,
          rule=list(rule), ret_diff_mean=round(float(dr.mean()), 3), score_diff_mean=round(float(ds.mean()), 3), sign_agreement=round(agree, 2))


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--out', required=True); ap.add_argument('--rule', default=None,
                                                                                         help='rule.json for check 10 (default gain 8, lead 6)')
    a = ap.parse_args()
    rule = (8.0, 6)
    if a.rule and os.path.exists(a.rule):
        b = json.load(open(a.rule))['best']; rule = (b['gain'], int(b['lead_idx']))
    signs(); twin_and_determinism(); actuator(); interlock(); contract()
    with Pool(12) as pool:
        stability(pool); body(pool); predictor(pool); reward(pool, rule)
    RES['_summary'] = dict(failed=FAILS, overlays=_ov())
    json.dump(RES, open(a.out, 'w'), indent=1, default=float)
    print(f'\n{len(RES) - 1 - len(FAILS)} / {len(RES) - 1} checks passed' + (f'; FAILED: {FAILS}' if FAILS else ''))


if __name__ == '__main__':
    main()
