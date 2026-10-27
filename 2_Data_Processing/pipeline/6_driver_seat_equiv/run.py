"""Part F (driver-seat equivalent): run AFTER pipeline/4_time_sync.

  python pipeline\\6_driver_seat_equiv\\run.py

The run was recorded from the PASSENGER side (safety): the IMU sat on the passenger dashboard and the
pressure seat on the passenger seat. This stage estimates what a sensor at the DRIVER seat would have read.

1. IMU: rigid-body transfer  a_T = a_IMU + alpha x r + omega x (omega x r)  to the passenger-seat point (P) and the
   driver-seat point (D). Angular rate / orientation are identical everywhere on a rigid body and are not changed.
2. Seat: per-channel linear response to the passenger-seat acceleration is fitted on detrended data, then the
   driver-minus-passenger acceleration difference is applied:  raw_D = raw_P + b_y*(a_Dy - a_Py) + b_x*(a_Dx - a_Px).
   This is the inertial part only; a driver's active bracing (wheel, pedals) is NOT modelled.

Writes (run2 only; existing refined files are not modified):
  01_refined/run2/imu_driver_equiv.csv, seat_driver_equiv.csv, synced_10hz_driver_equiv.csv, driver_equiv.json
  02_qc/6_driver_seat_equiv/report.md, turns.csv, turns.png
"""
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.optimize import least_squares
from scipy.signal import savgol_filter

ROOT = Path(__file__).resolve().parents[2]
R2 = ROOT / "01_refined" / "run2"
QC = ROOT / "02_qc" / "6_driver_seat_equiv"
sys.path.insert(0, str(ROOT / "pipeline" / "4_time_sync"))
from sync import write_text  # noqa: E402

# --------------------------------------------------------------------------- geometry (calibration knobs)
# Vehicle frame of imu_mounting.json: +x forward, +y left, +z up; origin = IMU.
# User-measured mount (2026-10-24): IMU on the passenger dashboard, centred on the passenger seat, 0.40 m ahead of the
# passenger seat-cushion centre at the same height.
IMU_TO_PSEAT = np.array([-0.40, 0.0, 0.0])
# Driver/passenger seat-centre spacing from the test vehicle's front shoulder room 1473 mm and hip room
# 1438 mm -> half-width 0.737 / 0.719 m. Both front seats share the same x and z.
SEAT_SPACING_M = 0.74
SEAT_SPACING_RANGE_M = (0.72, 0.76)
IMU_TO_DSEAT = IMU_TO_PSEAT + np.array([0.0, SEAT_SPACING_M, 0.0])
G = 9.80665

DT = 0.1              # IMU is 10 Hz
SG_WIN = 9            # 0.9 s Savitzky-Golay window for alpha = d(omega)/dt
DETREND_S = 30.0      # rolling median removed before the seat fit (posture drift, road grade)
LAGS_S = np.round(np.arange(0.0, 1.01, 0.1), 1)
CH = [f"a{a}_c{c}" for a in range(3) for c in range(4)]


def transfer(acc, omega, alpha, r):
    """Specific force at point r (m, vehicle frame) from the specific force measured at the IMU."""
    return acc + np.cross(alpha, r) + np.cross(omega, np.cross(omega, r))


def selfcheck():
    # steady yaw, 0.5 rad/s: a point 0.74 m to the left feels 0.5^2*0.74 m/s^2 less leftward (centripetal) force
    w = np.array([[0, 0, 0.5]])
    d = transfer(np.zeros((1, 3)), w, np.zeros((1, 3)), np.array([0, 0.74, 0]))
    assert np.allclose(d, [[0, -0.185, 0]])
    # yaw acceleration 1 rad/s^2 (speeding up a left turn): a point 0.40 m behind is pushed 0.40 m/s^2 rightward
    d = transfer(np.zeros((1, 3)), np.zeros((1, 3)), np.array([[0, 0, 1.0]]), np.array([-0.4, 0, 0]))
    assert np.allclose(d, [[0, -0.4, 0]])


# --------------------------------------------------------------------------- 1. IMU
def imu_part():
    mount = json.loads((R2 / "imu_mounting.json").read_text(encoding="utf-8"))
    R = np.array(mount["R_sensor_to_vehicle"])
    bias = R @ np.array(mount["gyro_bias_radps"])
    imu = pd.read_csv(R2 / "imu.csv", low_memory=False)
    imu = imu[imu.sample_valid == 1].sort_values("t_run_s").reset_index(drop=True)
    t = imu.t_run_s.to_numpy(float)
    acc = imu[[f"veh_acc_{a}_mps2" for a in "xyz"]].to_numpy(float)
    omega = imu[[f"veh_gyro_{a}_radps" for a in "xyz"]].to_numpy(float) - bias

    grid = np.arange(t[0], t[-1] + DT / 2, DT)
    wg = np.column_stack([np.interp(grid, t, omega[:, k]) for k in range(3)])
    ag = savgol_filter(wg, SG_WIN, 2, deriv=1, delta=DT, axis=0)
    # grid points farther than 0.15 s from a real sample are interpolated across a gap; alpha near them is invalid
    near = np.abs(grid - t[np.clip(np.searchsorted(t, grid), 0, len(t) - 1)])
    near = np.minimum(near, np.abs(grid - t[np.clip(np.searchsorted(t, grid) - 1, 0, len(t) - 1)]))
    bad = np.convolve((near > 0.15).astype(float), np.ones(SG_WIN), "same") > 0
    alpha = np.column_stack([np.interp(t, grid, ag[:, k]) for k in range(3)])
    alpha_bad = np.interp(t, grid, bad.astype(float)) > 0

    aP = transfer(acc, omega, alpha, IMU_TO_PSEAT)
    aD = transfer(acc, omega, alpha, IMU_TO_DSEAT)
    out = pd.DataFrame({"t_utc_ns": imu.t_utc_ns, "t_run_s": t, "in_obd_window": imu.in_obd_window})
    for k, a in enumerate("xyz"):
        out[f"omega_{a}_radps"] = omega[:, k]
    for k, a in enumerate("xyz"):
        out[f"alpha_{a}_radps2"] = alpha[:, k]
    for name, arr in (("imu", acc), ("pseat", aP), ("dseat", aD)):
        for k, a in enumerate("xyz"):
            out[f"{name}_acc_{a}_mps2"] = arr[:, k]
    for k, a in enumerate("xyz"):
        out[f"d_minus_p_acc_{a}_mps2"] = aD[:, k] - aP[:, k]
    out["flag_alpha_gap"] = alpha_bad.astype(int)
    out.to_csv(R2 / "imu_driver_equiv.csv", index=False, float_format="%.6f")
    return out, mount


# --------------------------------------------------------------------------- 2. seat
def rmed(x, n):
    return pd.Series(x).rolling(n, center=True, min_periods=n // 4).median().to_numpy()


def huber_fit(X, y):
    b0 = np.linalg.lstsq(X, y, rcond=None)[0]
    s = 1.4826 * np.median(np.abs(y - X @ b0)) + 1e-9
    return least_squares(lambda b: X @ b - y, b0, loss="huber", f_scale=s).x


def r2(X, y, b):
    return 1 - np.sum((y - X @ b) ** 2) / np.sum((y - y.mean()) ** 2)


def seat_part(imu):
    syn = pd.read_csv(R2 / "synced_10hz.csv", low_memory=False)
    t = syn.t_run_s.to_numpy(float)
    ti = imu.t_run_s.to_numpy(float)
    aPy, aPx = (np.interp(t, ti, imu[f"pseat_acc_{a}_mps2"]) for a in "yx")
    n = int(DETREND_S / DT) + 1
    ok_t = (syn.seat_n > 0).to_numpy() & (syn.imu_valid == 1).to_numpy()
    Y = {c: syn[f"seat_{c}_raw_mean"].to_numpy(float) for c in CH}
    Yd = {c: Y[c] - rmed(Y[c], n) for c in CH}
    xy, xx = aPy - rmed(aPy, n), aPx - rmed(aPx, n)

    def design(lag):
        k = int(round(lag / DT))
        sh = lambda v: np.r_[np.full(k, np.nan), v[:len(v) - k]] if k else v  # noqa: E731  seat(t) ~ acc(t - lag)
        return np.column_stack([np.ones_like(t), sh(xy), sh(xx)])

    # common lag: maximise the summed in-sample R^2 over the 12 channels
    scan = []
    for lag in LAGS_S:
        X = design(lag)
        m = ok_t & np.isfinite(X).all(1)
        scan.append(sum(max(r2(X[m], Yd[c][m], huber_fit(X[m], Yd[c][m])), 0) for c in CH if np.isfinite(Yd[c][m]).all()))
    lag = float(LAGS_S[int(np.argmax(scan))])
    X = design(lag)
    m = ok_t & np.isfinite(X).all(1)
    half = t < np.median(t[m])

    fits = {}
    for c in CH:
        y = Yd[c]
        mm = m & np.isfinite(y)
        b = huber_fit(X[mm], y[mm])
        b1 = huber_fit(X[mm & half], y[mm & half])
        b2 = huber_fit(X[mm & ~half], y[mm & ~half])
        oos = (r2(X[mm & ~half], y[mm & ~half], b1), r2(X[mm & half], y[mm & half], b2))
        keep = min(oos) > 0 and np.nanmedian(Y[c][mm]) > 10  # an unloaded FSR has no linear response
        use = [bool(keep and np.sign(b1[i]) == np.sign(b2[i]) == np.sign(b[i])) for i in (1, 2)]
        fits[c] = dict(b_y_counts_per_mps2=b[1], b_x_counts_per_mps2=b[2], r2_in_sample=r2(X[mm], y[mm], b),
                       r2_out_of_sample_halves=list(oos), b_y_halves=[b1[1], b2[1]], b_x_halves=[b1[2], b2[2]],
                       applied_b_y=b[1] if use[0] else 0.0, applied_b_x=b[2] if use[1] else 0.0,
                       raw_p50=float(np.nanmedian(Y[c][mm])),
                       # specific force +y (left turn) pushes the occupant right: b_y > 0 -> sensor on the right
                       inferred_side=("right" if b[1] > 0 else "left") if use[0] else "undetermined")

    # driver estimate at 100 Hz (seat timestamps), acceleration difference taken at t - lag
    seat = pd.read_csv(R2 / "seat.csv", low_memory=False)
    ts = seat.t_run_s.to_numpy(float) - lag
    dy = np.interp(ts, ti, imu.d_minus_p_acc_y_mps2)
    dx = np.interp(ts, ti, imu.d_minus_p_acc_x_mps2)
    cover = (ts >= ti[0]) & (ts <= ti[-1])
    so = pd.DataFrame({"t_utc_ns": seat.t_utc_ns, "t_run_s": seat.t_run_s, "in_obd_window": seat.in_obd_window,
                       "frame_valid": seat.frame_valid, "d_minus_p_acc_y_mps2": dy, "d_minus_p_acc_x_mps2": dx})
    for c in CH:
        raw = seat[f"{c}_raw"].to_numpy(float)
        est = raw + fits[c]["applied_b_y"] * dy + fits[c]["applied_b_x"] * dx
        so[f"{c}_raw"] = raw
        so[f"{c}_raw_driver_est"] = np.where(cover, np.clip(est, 0, 4095), np.nan)
        so[f"{c}_delta"] = so[f"{c}_raw_driver_est"] - raw
    low = np.column_stack([seat[f"{c}_raw"] <= 10 for c in CH if fits[c]["applied_b_y"] or fits[c]["applied_b_x"]])
    so["flag_low_signal"] = low.any(1).astype(int) if low.size else 0  # an unloaded FSR cannot show a linear response
    so["flag_no_imu"] = (~cover).astype(int)
    so.to_csv(R2 / "seat_driver_equiv.csv", index=False, float_format="%.3f")

    # 10 Hz add-on, joins synced_10hz.csv on t_run_s
    s10 = pd.DataFrame({"t_utc_ns": syn.t_utc_ns, "t_run_s": t})
    for name in ("imu", "pseat", "dseat"):
        for a in "xyz":
            s10[f"{name}_acc_{a}_mps2"] = np.interp(t, ti, imu[f"{name}_acc_{a}_mps2"])
    s10["omega_z_radps"] = np.interp(t, ti, imu.omega_z_radps)
    s10["alpha_z_radps2"] = np.interp(t, ti, imu.alpha_z_radps2)
    dy10 = np.interp(t - lag, ti, imu.d_minus_p_acc_y_mps2)
    dx10 = np.interp(t - lag, ti, imu.d_minus_p_acc_x_mps2)
    for c in CH:
        s10[f"seat_{c}_raw_mean_driver_est"] = np.clip(Y[c] + fits[c]["applied_b_y"] * dy10
                                                       + fits[c]["applied_b_x"] * dx10, 0, 4095)
    s10.to_csv(R2 / "synced_10hz_driver_equiv.csv", index=False, float_format="%.6f")
    return fits, lag, dict(zip([float(x) for x in LAGS_S], scan)), s10, syn


# --------------------------------------------------------------------------- 3. QC
def turns(s10, syn):
    wz = pd.Series(s10.omega_z_radps).rolling(5, center=True, min_periods=1).mean().to_numpy()
    on = (np.abs(wz) > 0.1) & (syn.gps_speed_kmh.fillna(0).to_numpy() > 5)
    edges = np.flatnonzero(np.diff(np.r_[0, on.astype(int), 0]))
    rows = []
    for a, b in zip(edges[::2], edges[1::2]):
        if (b - a) * DT < 2.0:
            continue
        sl = slice(a, b)
        sgn = np.sign(np.median(wz[sl]))
        P, D, I = (s10[f"{n}_acc_y_mps2"].to_numpy()[sl] for n in ("pseat", "dseat", "imu"))
        k = int(np.argmax(np.abs(P)))
        rows.append(dict(t_start_s=round(s10.t_run_s[a], 1), duration_s=round((b - a) * DT, 1),
                         direction="left" if sgn > 0 else "right", peak_yaw_dps=round(np.degrees(wz[sl][np.argmax(np.abs(wz[sl]))]), 1),
                         peak_imu_ay_g=round(np.max(np.abs(I)) / G, 3), peak_pseat_ay_g=round(np.max(np.abs(P)) / G, 3),
                         peak_dseat_ay_g=round(np.max(np.abs(D)) / G, 3),
                         dseat_over_pseat_at_peak=round(abs(D[k]) / max(abs(P[k]), 1e-6), 3),
                         mean_d_minus_p_ay_g=round(np.mean(D - P) / G, 4)))
    return pd.DataFrame(rows)


def esc_check(s10, syn):
    """Fit the IMU lever arm relative to the car's own ESC lateral-g sensor (OBD, ~1.1 Hz, 0.01 g steps).
    a_IMU_y - k*g*obd_lat = c + alpha_z*rx - omega_z^2*ry  -> (rx, ry) of the IMU relative to the ESC sensor."""
    fresh = (syn.obd_valid == 1) & (syn.obd_age_s < 0.15) & (syn.gps_speed_kmh > 5)
    d = pd.DataFrame({"ai": s10.imu_acc_y_mps2, "ae": G * syn.obd_accel_lat_g, "az": s10.alpha_z_radps2,
                      "w2": s10.omega_z_radps ** 2})[fresh.to_numpy()].dropna()
    X = np.column_stack([np.ones(len(d)), d.ae, d.az, -d.w2])
    b = huber_fit(X, d.ai.to_numpy())
    res = d.ai - X @ b
    cov = np.linalg.inv(X.T @ X) * np.var(res)
    se = np.sqrt(np.diag(cov))
    return dict(n=len(d), scale_k=b[1], rx_m=b[2], rx_se_m=se[2], ry_m=b[3], ry_se_m=se[3],
                note="ry > 0 means the IMU is left of the ESC sensor; expected about -0.37 m (passenger side of the "
                     "centreline). OBD lateral g is OCR'd at 0.01 g resolution and ~1.1 Hz, so this is a sign/order "
                     "check only, not a calibration.")


def plot(s10, syn, tr, fits, lag):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    pick = [tr[tr.direction == d].sort_values("peak_pseat_ay_g").iloc[-1] for d in ("right", "left") if (tr.direction == d).any()]
    ch = max(CH, key=lambda c: abs(fits[c]["applied_b_y"]))
    fig, ax = plt.subplots(3, len(pick), figsize=(6 * len(pick), 8), squeeze=False, sharex="col")
    for j, r in enumerate(pick):
        m = (s10.t_run_s >= r.t_start_s - 4) & (s10.t_run_s <= r.t_start_s + r.duration_s + 4)
        tt = s10.t_run_s[m]
        ax[0, j].plot(tt, np.degrees(s10.omega_z_radps[m]), "k")
        ax[0, j].set_ylabel("yaw rate (deg/s)")
        ax[0, j].set_title(f"{r.direction} turn @ t={r.t_start_s:.0f}s")
        for n, lab in (("imu", "IMU (dash)"), ("pseat", "passenger seat"), ("dseat", "driver seat (est)")):
            ax[1, j].plot(tt, s10[f"{n}_acc_y_mps2"][m] / G, label=lab)
        ax[1, j].set_ylabel("lateral (g, +left)")
        ax[1, j].legend(fontsize=8)
        ax[2, j].plot(tt, syn[f"seat_{ch}_raw_mean"][m], label=f"{ch} measured (passenger)")
        ax[2, j].plot(tt, s10[f"seat_{ch}_raw_mean_driver_est"][m], label=f"{ch} driver est")
        ax[2, j].set_ylabel("ADC counts")
        ax[2, j].set_xlabel("t_run_s")
        ax[2, j].legend(fontsize=8)
    fig.suptitle(f"Driver-seat equivalent (seat spacing {SEAT_SPACING_M} m, seat lag {lag} s)")
    fig.tight_layout()
    fig.savefig(QC / "turns.png", dpi=110)
    plt.close(fig)


def clean(o):
    if isinstance(o, dict):
        return {str(k): clean(v) for k, v in o.items()}
    if isinstance(o, (list, tuple, np.ndarray)):
        return [clean(v) for v in o]
    if isinstance(o, (np.floating, float)):
        return round(float(o), 6)
    if isinstance(o, np.integer):
        return int(o)
    if isinstance(o, np.bool_):
        return bool(o)
    return o


def main():
    selfcheck()
    QC.mkdir(parents=True, exist_ok=True)
    imu, mount = imu_part()
    fits, lag, scan, s10, syn = seat_part(imu)
    tr = turns(s10, syn)
    tr.to_csv(QC / "turns.csv", index=False)
    esc = esc_check(s10, syn)
    plot(s10, syn, tr, fits, lag)

    w2 = imu.omega_z_radps ** 2
    meta = dict(
        schema="esw-driver-equiv/1", run="run2", generated_by="pipeline/6_driver_seat_equiv/run.py",
        reason="data was recorded on the passenger side (IMU on passenger dashboard, pressure seat on the "
               "passenger seat) for safety; this estimates driver-seat values.",
        frame="imu_mounting.json vehicle frame: +x forward, +y left, +z up; origin at the IMU",
        geometry=dict(imu_to_passenger_seat_m=IMU_TO_PSEAT, imu_to_driver_seat_m=IMU_TO_DSEAT,
                      seat_spacing_m=SEAT_SPACING_M, seat_spacing_range_m=SEAT_SPACING_RANGE_M,
                      sources=["user measurement 2026-10-24: IMU centred on the passenger seat, 0.40 m ahead of the "
                               "cushion centre, same height",
                               "test vehicle front shoulder room 1473 mm, hip room 1438 mm; "
                               "front track 1618-1623 mm (cararac); overall 4905 x 1860 x 1445 mm, wheelbase 2850 mm",
                               "seat spacing = half of shoulder/hip room; no published seat-centre spacing"]),
        imu=dict(formula="a_T = a_IMU + alpha x r + omega x (omega x r) (specific force; gravity is uniform)",
                 omega="veh_gyro minus R @ gyro_bias_radps from imu_mounting.json",
                 alpha=f"Savitzky-Golay derivative, {SG_WIN} samples ({SG_WIN * DT:.1f} s), order 2, on a 10 Hz grid",
                 unchanged="orientation, angular rate, GPS position/speed: identical on a rigid body",
                 d_minus_p_lateral_mean_mps2=float(np.mean(imu.d_minus_p_acc_y_mps2)),
                 d_minus_p_lateral_expected="-omega_z^2 * seat_spacing: the driver is always farther from the turn "
                                            "centre in right turns (more lateral g) and closer in left turns (less)",
                 max_abs_d_minus_p_mps2=dict(x=float(np.abs(imu.d_minus_p_acc_x_mps2).max()),
                                             y=float(np.abs(imu.d_minus_p_acc_y_mps2).max())),
                 max_omega_z_sq=float(w2.max()),
                 seat_spacing_sensitivity="the lateral correction scales linearly with seat_spacing_m: "
                                          "+/-0.02 m changes it by +/-2.7 %"),
        seat=dict(model="raw(t) - rolling_median_30s = c + b_y*a_Py(t-lag) + b_x*a_Px(t-lag), Huber fit, 10 Hz",
                  lag_s=lag, lag_scan_sum_r2=scan,
                  apply_rule="a coefficient is applied only if both half-split fits predict the other half with "
                             "R^2 > 0, the full and both half fits agree in sign, and the channel is loaded (median raw > 10); "
                             "otherwise 0 (no correction)",
                  channels=fits),
        esc_lever_arm_check=esc,
        limitations=[
            "Passive passenger vs active driver: the driver braces on the wheel and pedals and anticipates turns. "
            "Only the inertial difference between the two seat positions is corrected; behaviour is not.",
            "Seat sensor left/right positions are not recorded (seat_config side=unknown), so a mirrored "
            "inboard/outboard (door-side vs console-side) remap is not possible. Left/right in the vehicle frame "
            "is the same for both seats, so no sign flip is needed for inertial loads.",
            "Linear ADC-count response; FSRs are nonlinear and an unloaded channel (raw <= 10) cannot respond "
            "(flag_low_signal). Different occupant body mass is not corrected.",
            "Seat spacing is an estimate from interior widths, not a measured value.",
            "run1 is not processed: its IMU fragments have no vehicle-frame mounting and its seat sessions do not "
            "overlap the OBD window.",
        ],
    )
    write_text(R2 / "driver_equiv.json", json.dumps(clean(meta), ensure_ascii=False, indent=1))

    lines = ["# 6_driver_seat_equiv — 조수석 측정값 → 운전석 환산", "",
             f"- 좌석 간격 {SEAT_SPACING_M} m (범위 {SEAT_SPACING_RANGE_M}), IMU→조수석 {IMU_TO_PSEAT.tolist()} m, "
             f"IMU→운전석 {IMU_TO_DSEAT.tolist()} m", f"- 압력 응답 지연 {lag} s", "",
             "## 회전 구간", "", "\n".join(["| " + " | ".join(tr.columns) + " |", "|" + "---|" * tr.shape[1]]
                                       + ["| " + " | ".join(map(str, r)) + " |" for r in tr.itertuples(index=False)]), "",
             "## 압력 채널 회귀", "", "| ch | b_y | b_x | R² in | R² oos (h1→h2, h2→h1) | 적용 b_y | 적용 b_x | p50 raw |",
             "|---|---|---|---|---|---|---|---|"]
    for c, f in fits.items():
        lines.append(f"| {c} | {f['b_y_counts_per_mps2']:.1f} | {f['b_x_counts_per_mps2']:.1f} | {f['r2_in_sample']:.3f} | "
                     f"{f['r2_out_of_sample_halves'][0]:.3f}, {f['r2_out_of_sample_halves'][1]:.3f} | "
                     f"{f['applied_b_y']:.1f} | {f['applied_b_x']:.1f} | {f['raw_p50']:.0f} |")
    lines += ["", "## ESC(OBD 횡가속) 대비 IMU 레버암 점검", "", "```", json.dumps(clean(esc), indent=1), "```", "",
              "![turns](turns.png)", ""]
    write_text(QC / "report.md", "\n".join(lines))
    print(json.dumps(clean(dict(lag=lag, n_turns=len(tr), esc=esc,
                                applied={c: (round(f["applied_b_y"], 1), round(f["applied_b_x"], 1)) for c, f in fits.items()})), indent=1))


if __name__ == "__main__":
    main()
