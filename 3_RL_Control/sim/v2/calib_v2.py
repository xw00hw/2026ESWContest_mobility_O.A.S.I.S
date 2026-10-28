"""Calibrate the v2 occupant/seat/sensor model from the data we have.

Assumption (team decision): tightening a bolster INCREASES the pressure on
that side's side-channel. The bench right-side data showed the opposite sign, which is
read as the right yaw axis having been recorded reversed; its magnitude is kept.

One physical model (HumanLateral + bolster contact + linear count readout) must
reproduce BOTH
  car (refined run2, passenger): side-diff (R-L) = 7234 counts/g toward the console,
                                 3468 counts/g toward the door, 1st-order 0.085 s + 0.1 s
  bench:                         side-diff = +15.7 counts/deg of bolster tightening
Unknowns: effective lever arm of the pad (normal travel per degree at the contact point),
counts-per-newton G, door support k_door. With a linear contact the ratio of the two gains
does not depend on the contact stiffness k, so k is NOT identifiable from these data; it is
fixed at a nominal value here and randomised in the env, where the arm is re-solved per
sample so every sample keeps the measured ratio.
Then the calibrated model is validated by replaying the whole car drive and
predicting the measured side-diff and the other channels (held-out halves).
Writes sim/v2/calib_v2.json.

With a scale calibration of the side pads (tools/pad_calib.py -> pad_calib.json) G is MEASURED,
and the console gain 7234 counts/g then fixes the console force per g, which identifies k:

    python sim/v2/calib_v2.py [--pad_calib deploy/model/pad_calib.json] [--out sim/v2/calib_v2.json]
"""
import argparse, os, sys, json
import numpy as np, pandas as pd
from scipy.optimize import brentq

_ap = argparse.ArgumentParser()
_ap.add_argument('--pad_calib', default=os.environ.get('OASIS_PAD_CALIB'))
_ap.add_argument('--out')
ARGS = _ap.parse_args()

H = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.join(H, '..', '..')
sys.path.insert(0, os.path.join(H, '..'))
from human_model import HumanLateral, HumanParams  # noqa: E402

DT, G0 = 0.005, 9.80665
CM = json.load(open(os.environ.get('OASIS_CORR') or os.path.join(ROOT, 'analysis', 'corr_model.json')))   # pipeline: bench_fit output
CAR_CONSOLE = CM['C1_turn_dose_response']['right_counts_per_g']     # body pushed left (right turn)
CAR_DOOR = CM['C1_turn_dose_response']['left_counts_per_g']         # body pushed right toward the door
BENCH_TIGHT = abs(CM['B1_right_yaw_rowcentred']['A1_C1']['slope_per_deg'] - CM['B1_right_yaw_rowcentred']['A1_C3']['slope_per_deg'])
ARM_REF, PRELOAD, THETA_MAX, K_NOM = 0.17, 20.0, 20.0, 3000.0
ARM = ARM_REF


def passenger(k, door_k=0.0):
    return HumanParams(mass=72, torso_frac=0.55, h_com=0.32, fn_torso=1.6, zeta_torso=1.5, passive_frac=0.5,
                       reflex_delay=0.08, fn_pelvis=4.0, zeta_pelvis=1.5, mu_cushion=0.5, brace_k=0.0, ff_gain=0.0,
                       h_bolster=0.20, k_contact=k, n_contact=1.0, c_contact=250.0, door_k=door_k, door_side=-1)


def static(hp, ay_g=0.0, thR=0.0, thL=0.0, T=4.0, arm=None):
    arm = ARM if arm is None else arm
    b = HumanLateral(hp, DT); gap0 = -(PRELOAD / hp.k_contact)
    for i in range(int(T / DT)):
        r = min(1.0, i * DT / 0.5)
        o = b.step(ay_g * G0 * r, ay_g * G0 * r, gap0 - arm * np.sin(np.deg2rad(thL * r)), gap0 - arm * np.sin(np.deg2rad(thR * r)))
    return o


def gains(k, door_k=0.0, arm=None):
    hp = passenger(k, door_k); z = static(hp, arm=arm)
    D0 = z['F_R'] - z['F_L']
    a = static(hp, ay_g=-0.2, arm=arm); console = (a['F_R'] - a['F_L'] - D0) / -0.2       # N/g (body pushed left)
    a = static(hp, ay_g=+0.2, arm=arm); door = (a['F_R'] - a['F_L'] - D0) / 0.2           # N/g (body pushed right)
    a = static(hp, thR=THETA_MAX, arm=arm); tight = (a['F_R'] - a['F_L'] - D0) / THETA_MAX  # N/deg
    return console, door, tight


target_ratio = BENCH_TIGHT / CAR_CONSOLE                      # g of side-diff per deg of tightening


def solve_arm(k):
    c_ref, _, t_ref = gains(k, arm=ARM_REF)
    return ARM_REF * target_ratio / (t_ref / c_ref)          # ratio is linear in the arm for small angles


K = K_NOM; ARM = solve_arm(K); PAD = None; K_IDENT = False
if ARGS.pad_calib:                                            # measured counts/N -> console N/g -> k
    pj = json.load(open(ARGS.pad_calib)); g = pj['controller_counts_per_N']
    G_MEAS = 0.5 * (g['A1_C1'] + g['A1_C3'])                  # side-diff uses both pads
    want = CAR_CONSOLE / G_MEAS
    f = lambda lk: gains(np.exp(lk), arm=solve_arm(np.exp(lk)))[0] - want
    try:
        K = float(np.exp(brentq(f, np.log(300.0), np.log(30000.0), xtol=1e-3))); K_IDENT = True
    except ValueError:                                        # outside what the occupant model can produce
        print(f'pad calibration G = {G_MEAS:.1f} counts/N needs {want:.0f} N/g: outside the model range, k kept nominal')
    ARM = solve_arm(K)
    PAD = dict(file=os.path.abspath(ARGS.pad_calib), G_meas_counts_per_N=G_MEAS, A1_C1=g['A1_C1'], A1_C3=g['A1_C3'],
               console_N_per_g_implied=want, k_identified=K_IDENT)
console, door, tight = gains(K, arm=ARM)
G_SIDE = CAR_CONSOLE / console                                # counts per newton of side-diff force
# door support so that loading toward the door gives 3468/7234 of the console-side gain
fd = lambda ld: gains(K, np.exp(ld))[1] / gains(K, np.exp(ld))[0] - CAR_DOOR / CAR_CONSOLE
LD = brentq(fd, np.log(1), np.log(1e5), xtol=1e-3); KD = float(np.exp(LD))
# console force per g vs contact k (nominal passenger, this arm): env_v2 anchors G per episode with it
K_TAB = [[float(k), float(gains(k, arm=ARM)[0])] for k in np.geomspace(800, 12000, 7)]
# door/contact stiffness ratio used in the env: the static 0.2 g fit gives KD/K (~6.6), but the drive
# replay's door/console ratio (0.44 vs car 0.48) needs ~0.8-2 because most of the replay asymmetry comes
# from the drive itself; 2.0 is the team's choice
DOOR_RATIO_DRIVE = 2.0
print(f'effective arm = {100 * ARM:.2f} cm (geometric 17 cm), contact k = {K:.0f} N/m ({"identified from pad calibration" if K_IDENT else "nominal, not identifiable"}), G = {G_SIDE:.2f} counts/N, door k = {KD:.0f} N/m')
print(f'  sim: console {console:.0f} N/g, tightening {tight:.2f} N/deg -> {tight * G_SIDE:.1f} counts/deg (bench {BENCH_TIGHT:.1f})')
print(f'  bolster authority: {THETA_MAX:.0f} deg = {tight * THETA_MAX:.0f} N side force = {tight * THETA_MAX / console:.3f} g equivalent')

# ------------------------------------------------ validation: replay the whole car drive
REF = os.environ.get('OASIS_CAR_RUN') or os.path.join(ROOT, 'refined_drive', '01_refined', 'run2')
d = pd.read_csv(os.path.join(REF, 'synced_10hz.csv'))
t10 = d.t_run_s.to_numpy(); ay10 = np.nan_to_num(d.imu_veh_acc_y_mps2.to_numpy())
t = np.arange(t10[0], t10[-1], DT); ay = np.interp(t, t10, ay10)
hp = passenger(K, KD); b = HumanLateral(hp, DT); gap0 = -(PRELOAD / K)
FR, FL, YP, PHI = (np.zeros(len(t)) for _ in range(4))
for i in range(len(t)):
    o = b.step(ay[i], ay[i], gap0, gap0); FR[i], FL[i], YP[i], PHI[i] = o['F_R'], o['F_L'], o['y_pelvis'], o['phi']
step = int(round(0.1 / DT)); idx = np.searchsorted(t, t10).clip(0, len(t) - 1)
S = pd.DataFrame({'FR': FR[idx], 'FL': FL[idx], 'yp': YP[idx], 'phi': PHI[idx]})
pred_side = G_SIDE * (S.FR - S.FL)


def hp_causal(x, w=300):
    x = pd.Series(np.asarray(x, float)); return (x - x.rolling(w, min_periods=30).median().shift(1)).to_numpy()


meas = hp_causal(d.seat_a1_c1_raw_mean - d.seat_a1_c3_raw_mean); ps = hp_causal(pred_side)
mov = (d.gps_speed_kmh > 5).to_numpy() & ~np.isnan(meas) & ~np.isnan(ps)
r2 = lambda a, b_, m: float(1 - np.var((a - b_)[m]) / np.var(a[m]))
half = np.arange(len(t10)) < len(t10) // 2
val = dict(side_diff_r2_all=r2(meas, ps, mov), side_diff_r2_h1=r2(meas, ps, mov & half), side_diff_r2_h2=r2(meas, ps, mov & ~half),
           note='no parameter was fitted to this trace: k and G come from two scalar gains, door k from the gain ratio')
# other channels -- STRUCTURAL readout (fitted on the car, tested on held-out halves and on the bench):
#   side channels   = base + g * (own-side contact force)        (contact sensors)
#   cushion / back  = base + w . (pelvis shift, torso roll)     (load-distribution sensors)
# A free regression on [F_R, F_L, y, phi] is not identifiable in the car (all four move together
# with a_y) and extrapolates wrongly to the bench, where only the contact force changes.
FEATS = {'a1_c1': ['FR'], 'a1_c3': ['FL'], 'a0_c0': ['yp', 'phi'], 'a0_c1': ['yp', 'phi'], 'a0_c2': ['yp', 'phi'],
         'a0_c3': ['yp', 'phi'], 'a2_c1': ['yp', 'phi'], 'a2_c2': ['yp', 'phi']}
Z = {'FR': hp_causal(S.FR), 'FL': hp_causal(S.FL), 'yp': hp_causal(S.yp * 100), 'phi': hp_causal(np.rad2deg(S.phi))}
readout = {}
for ch, fe in FEATS.items():
    y = hp_causal(d[f'seat_{ch}_raw_mean']); X = np.c_[[Z[f] for f in fe]].T
    m = mov & ~np.isnan(y) & ~np.isnan(X).any(1); res = {}
    for nm, tr, te in (('h1->h2', half, ~half), ('h2->h1', ~half, half)):
        w, *_ = np.linalg.lstsq(X[m & tr], y[m & tr], rcond=None); res[nm] = r2(y, X @ w, m & te)
    w, *_ = np.linalg.lstsq(X[m], y[m], rcond=None)
    readout[ch] = dict(features=fe, weights=[float(v) for v in w], base_counts=float(np.nanmedian(d[f'seat_{ch}_raw_mean'])), r2_test=res)
print('validation (no fit to trace):', {k: round(v, 3) for k, v in val.items() if k != 'note'})
print('structural readout test R2:', {k: [round(x, 2) for x in v['r2_test'].values()] for k, v in readout.items()})
z0 = static(passenger(K)); z1 = static(passenger(K), thR=THETA_MAX)
dz = {'FR': z1['F_R'] - z0['F_R'], 'FL': z1['F_L'] - z0['F_L'], 'yp': 100 * (z1['y_pelvis'] - z0['y_pelvis']), 'phi': np.rad2deg(z1['phi'] - z0['phi'])}
pred_bench = {ch: float(sum(w * dz[f] for f, w in zip(r['features'], r['weights'])) / THETA_MAX) for ch, r in readout.items()}
B1 = CM['B1_right_yaw_rowcentred']
bench_meas = {ch: -B1[ch.upper()]['slope_per_deg'] for ch in readout}      # sign flipped: tightening
print('bench per-deg  predicted vs measured(flipped):', {ch: (round(pred_bench[ch], 1), round(bench_meas[ch], 1)) for ch in readout})

out = dict(assumption='tightening increases same-side pressure; bench right-yaw sign flipped, magnitude kept',
           arm_eff_m=ARM, target_ratio_g_per_deg=target_ratio, contact_k_N_per_m=K, contact_k_identifiable=K_IDENT, pad_calib=PAD, preload_N=PRELOAD, arm_m=ARM, theta_max_deg=THETA_MAX, door_k_N_per_m=KD,
           G_side_counts_per_N=G_SIDE, sim_console_N_per_g=console, sim_tight_N_per_deg=tight,
           authority_g_equiv_at_theta_max=tight * THETA_MAX / console,
           targets=dict(car_console=CAR_CONSOLE, car_door=CAR_DOOR, bench_tight=BENCH_TIGHT),
           validation=val, readout=readout, bench_other_channels=dict(pred=pred_bench, meas_flipped=bench_meas),
           servo=dict(rate_dps=15.0, cmd_to_arrival_s_median=0.51, report_hz=5, range_validated_deg=[0, 20]),
           sensors=dict(noise_counts_car=[1, 7], noise_counts_bench=[22, 58], hysteresis_counts=CM['B3_A1_C1_hysteresis']['asc_minus_desc_counts'],
                        pose_torso_lean_noise_deg=0.3),
           console_N_per_g_vs_k_nominal=K_TAB, door_k_over_contact_k=DOOR_RATIO_DRIVE, door_k_over_contact_k_static=KD / K,
           door_note='static 0.2 g calibration gave %.1f; drive-replay ratio 0.48 needs ~0.8-2 (asymmetry in the replay is mostly from the drive itself); 2.0 used' % (KD / K))
json.dump(out, open(ARGS.out or os.path.join(H, 'calib_v2.json'), 'w'), indent=1, default=float)
# save a trace for the figure
pd.DataFrame({'t': t10, 'meas': meas, 'pred': ps, 'ay_g': ay10 / G0}).to_csv(os.path.join(H, 'calib_v2_trace.csv'), index=False)
