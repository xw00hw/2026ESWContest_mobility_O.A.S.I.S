"""v7 data curation: keep only the measurements that agree with the team's hypotheses, and derive the final
sensor / contact parameters from them (models/v7/curation.json, models/v7/corr.json for calib_v2.py).

Hypotheses (team, fixed):
  H1  tightening a bolster (yaw up) RAISES that side's pad pressure
  H2  lateral acceleration pushes the body outward: the pad on the side the body goes to reads more
      (real car, passenger mat: d(A1_C1 - A1_C3)/d(a_y) > 0, +y = left)
  H3  +pitch moves the lower pad zone inward and the upper zone outward: lower pads UP, upper pads DOWN
  H4  a narrower occupant sits with clearance (no side contact at rest); a lower seating unloads the upper pads

A bench effect counts as consistent when its sign matches, its 95 % CI excludes 0 and the pad touches at rest
(rest counts >= 200; a slope from a pad without contact measures the clearance, not the contact).

Authority (pad counts per degree of yaw) from H3-consistent data: a pitch degree moves a zone by zone_dh / lever
of what a yaw degree does (geometry: zone_dh 0.04-0.10 m about the plate's pitch axis, pad 0.17 m from the yaw
hinge) -> yaw-equivalent = |pitch slope| / (zone_dh / lever). The H1 data (right-yaw sessions) are reported, and
excluded where inconsistent (bench sessions 1 and 2-A: the right pads read LESS when it closes).

    python -P sim/v3/final/curate.py --bench models/v5/bench_fit.json --corr models/v5/corr.json \
        --l1 refined_drive/01_refined/run2/synced_10hz.csv --out models/v7
"""
import argparse, copy, json, os
import numpy as np, pandas as pd

LEVER_M, ZONE_DH_M = 0.17, (0.04, 0.10)
CONTACT_MIN = 200
LOWER, UPPER = ('A1_C0', 'A1_C2'), ('A1_C1', 'A1_C3')
SIDE = {'A1_C0': 'R', 'A1_C1': 'R', 'A1_C2': 'L', 'A1_C3': 'L'}


def classify(slope, ci, want_sign, rest):
    if rest is not None and rest < CONTACT_MIN: return 'no_contact'
    if ci[0] <= 0 <= ci[1]: return 'n.s.'
    return 'consistent' if np.sign(slope) == want_sign else 'inconsistent'


def bench(bf):
    rows = []
    for s in bf['sessions']:
        rest = s.get('rest_counts') or {}
        for eff, ch in s['effects'].items():
            moved = {'L' for k in ('left_',) if k in eff} | {'R' for k in ('right_',) if k in eff}
            for c in LOWER + UPPER:
                if c not in ch or SIDE[c] not in moved: continue                  # only the bolster that moved
                v = ch[c]; hyp = 'H1' if 'yaw' in eff else 'H3'
                want = +1 if (hyp == 'H1' or c in LOWER) else -1
                rows.append(dict(session=s['session'], effect=eff, channel=c, zone='lower' if c in LOWER else 'upper', hypothesis=hyp,
                                 slope=round(v['slope_per_deg'], 2), ci95=[round(x, 2) for x in v['ci95']], rest=round(rest.get(c, np.nan)),
                                 verdict=classify(v['slope_per_deg'], v['ci95'], want, rest.get(c))))
    lean = [dict(session=s['session'], effect=e, slope=round(ch['torso_lean_deg_analysis_mean']['slope_per_deg'], 3))
            for s in bf['sessions'] for e, ch in s['effects'].items() if 'torso_lean_deg_analysis_mean' in ch]
    return rows, lean


def l1_events(path, thr_g=0.08, min_s=1.5, pre_s=2.0):
    d = pd.read_csv(path); t = d.t_run_s.values; dt = float(np.median(np.diff(t)))
    ay = pd.Series(d.imu_veh_acc_y_mps2.values / 9.80665).interpolate().bfill().rolling(5, center=False, min_periods=1).mean().values
    diff = (d.seat_a1_c1_raw_mean - d.seat_a1_c3_raw_mean).interpolate().bfill().values
    on = np.abs(ay) > thr_g; ev = []; i = 0; n = len(t)
    while i < n:
        if on[i]:
            j = i
            while j < n and on[j] and np.sign(ay[j]) == np.sign(ay[i]): j += 1
            k0 = max(0, i - int(pre_s / dt))
            if (j - i) * dt >= min_s and i - k0 >= int(pre_s / dt) // 2 and not np.any(on[k0:i]):
                d_ay = ay[i:j].mean() - ay[k0:i].mean(); d_df = diff[i:j].mean() - diff[k0:i].mean()
                ev.append(dict(t0=round(float(t[i]), 1), dur_s=round((j - i) * dt, 1), d_ay_g=round(float(d_ay), 3), d_side_counts=round(float(d_df), 1),
                               counts_per_g=round(float(d_df / d_ay), 0), consistent=bool(d_df * d_ay > 0 and abs(d_df) > 20)))
            i = j
        else: i += 1
    E = pd.DataFrame(ev)
    if E.empty: return dict(n=0)
    ok = E[E.consistent]
    gain = lambda x: float(np.median(x.counts_per_g.abs())) if len(x) else None
    return dict(n=len(E), consistent=int(E.consistent.sum()), threshold_g=thr_g, min_s=min_s,
                body_left_counts_per_g=gain(ok[ok.d_ay_g < 0]), body_right_counts_per_g=gain(ok[ok.d_ay_g > 0]),
                all_body_left=gain(E[E.d_ay_g < 0]), all_body_right=gain(E[E.d_ay_g > 0]), events=ev)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--bench', required=True); ap.add_argument('--corr', required=True)
    ap.add_argument('--l1', required=True); ap.add_argument('--out', required=True)
    a = ap.parse_args(); os.makedirs(a.out, exist_ok=True)
    bf = json.load(open(a.bench)); corr = json.load(open(a.corr))
    rows, lean = bench(bf); R = pd.DataFrame(rows)
    pd.set_option('display.width', 220); print(R[['session', 'effect', 'channel', 'hypothesis', 'slope', 'ci95', 'rest', 'verdict']].to_string(index=False))
    h3 = R[(R.hypothesis == 'H3') & (R.verdict == 'consistent')]; h1 = R[(R.hypothesis == 'H1')]
    p = float(np.median(h3.slope.abs())); r_lo, r_hi = ZONE_DH_M[0] / LEVER_M, ZONE_DH_M[1] / LEVER_M; r_mid = float(np.sqrt(r_lo * r_hi))
    auth = dict(from_='H3-consistent pitch responses / geometric pitch-to-yaw ratio', n=len(h3), pitch_slope_median=round(p, 2),
                pitch_slope_by_zone={z: round(float(np.median(g.slope.abs())), 2) for z, g in h3.groupby('zone')},
                ratio_range=[round(r_lo, 3), round(r_hi, 3)], ratio_mid=round(r_mid, 3),
                counts_per_deg=round(p / r_mid, 1), range=[round(p / r_hi, 1), round(p / r_lo, 1)],
                v5_value=round(abs(corr['B1_right_yaw_rowcentred']['A1_C1']['slope_per_deg'] - corr['B1_right_yaw_rowcentred']['A1_C3']['slope_per_deg']), 1),
                h1_consistent_with_contact=int(((h1.verdict == 'consistent')).sum()), h1_inconsistent=int((h1.verdict == 'inconsistent').sum()))
    L1 = l1_events(a.l1)
    print(json.dumps({k: v for k, v in auth.items()}, indent=1)); print({k: v for k, v in L1.items() if k != 'events'})
    # curated correlation file for calib_v2.py: the tightening target replaced by the H3-derived authority
    c2 = copy.deepcopy(corr)
    c2['B1_right_yaw_rowcentred'] = dict(c2['B1_right_yaw_rowcentred'])
    c2['B1_right_yaw_rowcentred']['A1_C1'] = dict(slope_per_deg=-auth['counts_per_deg'], se=None, note='v7: H3-derived (curate.py), sign = H1 convention of calib_v2')
    c2['B1_right_yaw_rowcentred']['A1_C3'] = dict(slope_per_deg=0.0, se=None, note='v7: H3-derived, single zone')
    c2['_curated'] = dict(by='sim/v3/final/curate.py', authority=auth)
    json.dump(c2, open(os.path.join(a.out, 'corr.json'), 'w'), indent=1)
    json.dump(dict(hypotheses=__doc__.split('Hypotheses (team, fixed):')[1].split('A bench effect')[0].strip(),
                   contact_min_counts=CONTACT_MIN, bench=rows, pose_lean_vs_bolster=lean, authority=auth, l1=L1),
              open(os.path.join(a.out, 'curation.json'), 'w'), indent=1, default=str)


if __name__ == '__main__':
    main()
