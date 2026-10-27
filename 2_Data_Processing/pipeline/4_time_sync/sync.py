"""Part D: time synchronisation estimates, added time columns, 10 Hz synced tables.

Everything is derived from the refined outputs of parts A/B/C. Columns added by this part
(t_run_s, in_obd_window; t_utc_ns / t_utc_ns_est in OBD files) are stripped before any
estimation, so re-running is idempotent. Measured columns are never re-serialised: the
original CSV text of every pre-existing column is copied byte for byte.
"""
import io
import os
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.optimize import least_squares

DRIVE_DATE = os.environ.get("OASIS_DRIVE_DATE", "2026-01-01")  # 수집일 (UTC, YYYY-MM-DD)

ROOT = Path(__file__).resolve().parents[2]
R1 = ROOT / "01_refined" / "run1"
R2 = ROOT / "01_refined" / "run2"
G0 = 9.80665
KST = pd.Timedelta(hours=9)
D_TAIL = ["t_run_s", "in_obd_window"]          # appended as the last two columns of every table
D_HEAD = {"obd.csv", "obd_text.csv", "obd_changes.csv"}   # OBD tables also get a leading time column
SEAT_CH = [f"a{b}_c{c}_raw" for b in range(3) for c in range(4)]


# ----------------------------------------------------------------------------- CSV helpers
def _split(path):
    raw = path.read_bytes()
    assert b'"' not in raw, f"quoted CSV not supported: {path}"
    nl = b"\r\n" if b"\r\n" in raw else b"\n"
    lines = raw.split(nl)
    assert lines[-1] == b"", f"no trailing newline: {path}"
    return raw, nl, [ln.split(b",") for ln in lines[:-1]]


def _d_cols(path, header):
    """Indices of columns previously added by part D."""
    drop = [i for i, h in enumerate(header) if h.decode() in D_TAIL]
    if path.name in D_HEAD and header[0] in (b"t_utc_ns", b"t_utc_ns_est"):
        drop.append(0)
    return set(drop)


def load(path):
    """Read a refined CSV without the columns part D added."""
    df = pd.read_csv(path, low_memory=False)
    hdr = [c.encode() for c in df.columns]
    return df.drop(columns=[df.columns[i] for i in _d_cols(path, hdr)])


def add_cols(path, t_ref_ns, origin_ns, end_ns, head=None):
    """Append t_run_s / in_obd_window (and an optional leading integer time column `head` = name)."""
    raw, nl, rows = _split(path)
    drop = _d_cols(path, rows[0])
    keep = [i for i in range(len(rows[0])) if i not in drop]
    t_ref_ns = np.asarray(t_ref_ns, dtype=np.int64)
    assert len(t_ref_ns) == len(rows) - 1, path
    rel = t_ref_ns - origin_ns
    out = []
    for k, r in enumerate(rows):
        cells = [r[i] for i in keep]
        if k == 0:
            cells = ([head.encode()] if head else []) + cells + [c.encode() for c in D_TAIL]
        else:
            d = int(rel[k - 1])
            cells = ([b"%d" % t_ref_ns[k - 1]] if head else []) + cells + [
                ("%.6f" % (d / 1e9)).encode(), b"1" if 0 <= d <= end_ns - origin_ns else b"0"]
        out.append(b",".join(cells))
    new = nl.join(out) + nl
    if new != raw:
        tmp = path.with_suffix(".tmp")
        tmp.write_bytes(new)
        os.replace(tmp, path)
    return int((rel >= 0).sum() - (rel > end_ns - origin_ns).sum())


def write_text(path, text):
    data = text.encode("utf-8")
    if not path.exists() or path.read_bytes() != data:
        path.write_bytes(data)


def iso(ns, kst=False):
    t = pd.Timestamp(int(ns), unit="ns") + (KST if kst else pd.Timedelta(0))
    return t.strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + ("+09:00" if kst else "Z")


# ----------------------------------------------------------------------------- estimators
def box(x, n):
    return x if n <= 1 else np.convolve(x, np.ones(n) / n, "same")


def lag_scan(tv, y, xt, xv, center, half=3.0, step=0.02):
    """Offset o maximising |corr(y(tv), x(tv + o))|; parabolic sub-step refinement.
    Equivalent to least squares with free gain and intercept. Returns dict."""
    offs = center + np.arange(-half, half + step / 2, step)
    X = np.interp(tv[None, :] + offs[:, None], xt, xv)
    yc = y - y.mean()
    Xc = X - X.mean(1, keepdims=True)
    den = np.sqrt((Xc ** 2).sum(1) * (yc ** 2).sum())
    r = np.divide(Xc @ yc, den, out=np.zeros(len(offs)), where=den > 0)
    a = np.abs(r)
    i = int(a.argmax())
    interior = 0 < i < len(offs) - 1
    d = 0.0
    if interior:
        cv = a[i - 1] - 2 * a[i] + a[i + 1]
        d = 0.0 if cv == 0 else 0.5 * (a[i - 1] - a[i + 1]) / cv
    o = offs[i] + d * step
    x = np.interp(tv + o, xt, xv)
    gain, icpt = np.polyfit(x, y, 1)
    return dict(offset=float(o), corr=float(r[i]), gain=float(gain), intercept=float(icpt),
                rms=float(np.sqrt(np.mean((y - gain * x - icpt) ** 2))), interior=bool(interior),
                n=int(len(tv)), lags=offs, r=r)


def block_offsets(tv, y, xt, xv, center, t_end, nb=12, min_std=0.0):
    """Independent offset estimates in nb equal-duration blocks of the recording."""
    edges = np.linspace(0, t_end, nb + 1)
    out = []
    for a, b in zip(edges[:-1], edges[1:]):
        m = (tv >= a) & (tv < b)
        if m.sum() < 30 or y[m].std() < min_std:
            continue
        s = lag_scan(tv[m], y[m], xt, xv, center, half=1.5)
        if s["interior"] and abs(s["corr"]) >= 0.8:
            out.append(((a + b) / 2, s["offset"], s["corr"]))
    return np.array(out).reshape(-1, 3)


def drift_fit(blocks):
    """Linear trend of block offsets vs video time -> (ppm, se_ppm)."""
    if len(blocks) < 4:
        return float("nan"), float("nan")
    p, cov = np.polyfit(blocks[:, 0], blocks[:, 1], 1, cov=True)
    return float(p[0] * 1e6), float(np.sqrt(cov[0, 0]) * 1e6)


# ----------------------------------------------------------------------------- run 2
def run2_sync():
    obd = load(R2 / "obd.csv")
    gps = load(R2 / "gps.csv")
    imu = load(R2 / "imu.csv")
    seat = load(R2 / "seat.csv")
    S = {}  # everything that goes to sync.json / report

    # --- reference clock: Pi monotonic mapped linearly onto leap-corrected GPS UTC
    g = gps.dropna(subset=["gps_utc_corrected_ns"])
    m = g.host_monotonic_ns.to_numpy(np.int64)
    u = g.gps_utc_corrected_ns.to_numpy(np.int64)
    m0, u0 = int(m[0]), int(u[0])
    p = np.polyfit((m - m0).astype(float), (u - u0).astype(float), 1)
    res = (u - u0) - np.polyval(p, (m - m0).astype(float))
    gw = gps.t_utc_ns.to_numpy(np.int64)
    gref = u0 + np.rint(np.polyval(p, (gps.host_monotonic_ns.to_numpy(np.int64) - m0).astype(float))).astype(np.int64)
    c_ns = (gw - gref).astype(float)          # Pi wall clock minus reference clock at each GPS row

    def pi_to_ref(t_wall_ns):
        t_wall_ns = np.asarray(t_wall_ns, dtype=np.int64)
        c = np.interp((t_wall_ns - gw[0]).astype(float), (gw - gw[0]).astype(float), c_ns)
        return t_wall_ns - np.rint(c).astype(np.int64)

    S["ref"] = dict(m0=m0, u0=u0, slope=float(p[0]), intercept_ns=float(p[1]),
                    slope_minus1_ppm=float((p[0] - 1) * 1e6), n_epochs=int(len(g)),
                    resid_ms=dict(std=float(res.std() / 1e6), min=float(res.min() / 1e6), max=float(res.max() / 1e6)),
                    wall_minus_ref_ms=dict(min=float(c_ns.min() / 1e6), median=float(np.median(c_ns) / 1e6),
                                           max=float(c_ns.max() / 1e6)),
                    knots=[(iso(gw[i]), round(c_ns[i] / 1e6, 3)) for i in range(0, len(gw), 60)])
    S["ref_plot"] = dict(t=(gw - gw[0]) / 1e9, c_ms=c_ns / 1e6, t_fit=(m - m0) / 1e9, res_ms=res / 1e6)

    # --- OBD video offset. Prior from container: creation_time (= end) minus duration.
    prior_ns = pd.Timestamp(f"{DRIVE_DATE}T07:53:26Z").value - int(round(1165.273267 * 1e9))
    base = prior_ns                                           # seconds below are relative to this
    ev = obd[obd.any_value_changed == 1]                      # display update instants (values are fresh)
    tv = ev.t_video_s.to_numpy(float)
    t_end = float(obd.t_video_s.iloc[-1])
    gt = (gref - base) / 1e9
    it = (pi_to_ref(imu.t_utc_ns) - base) / 1e9
    sigs = {   # name: (obd values, ref time axis, ref values, unit conversion note, block min std)
        "speed_vs_gps": (ev.vehicle_speed_kmh.to_numpy(float), gt, gps.speed_kmh.to_numpy(float), "km/h vs km/h", 3.0),
        "yaw_vs_imu": (ev.yaw_rate_dps.to_numpy(float), it, box(np.degrees(imu.veh_gyro_z_radps.to_numpy(float)), 3),
                       "deg/s vs degrees(veh_gyro_z_radps), 0.3 s boxcar", 1.0),
        "along_vs_imu": (ev.accel_long_g.to_numpy(float), it, box(imu.veh_acc_x_mps2.to_numpy(float) / G0, 5),
                         "G vs veh_acc_x_mps2/9.80665, 0.5 s boxcar", 0.02),
        "alat_vs_imu": (ev.accel_lat_g.to_numpy(float), it, box(imu.veh_acc_y_mps2.to_numpy(float) / G0, 5),
                        "G vs veh_acc_y_mps2/9.80665, 0.5 s boxcar", 0.02),
    }
    per, curves, blocks_all = {}, {}, {}
    for k, (y, xt, xv, note, mstd) in sigs.items():
        s = lag_scan(tv, y, xt, xv, 0.0, half=4.0)
        bl = block_offsets(tv, y, xt, xv, s["offset"], t_end, min_std=mstd)
        dppm, dse = drift_fit(bl)
        curves[k] = (s.pop("lags"), s.pop("r"))
        blocks_all[k] = bl
        per[k] = dict(s, units=note, sign=int(np.sign(s["corr"])), n_blocks=int(len(bl)),
                      block_sd_s=float(bl[:, 1].std(ddof=1)), block_se_s=float(bl[:, 1].std(ddof=1) / np.sqrt(len(bl))),
                      block_median_s=float(np.median(bl[:, 1])), drift_ppm=dppm, drift_se_ppm=dse,
                      T0_utc=iso(base + int(round(s["offset"] * 1e9))))
    # sensitivity of the IMU-based estimates to the smoothing window
    sens = {}
    raw = dict(yaw_vs_imu=np.degrees(imu.veh_gyro_z_radps.to_numpy(float)),
               along_vs_imu=imu.veh_acc_x_mps2.to_numpy(float) / G0, alat_vs_imu=imu.veh_acc_y_mps2.to_numpy(float) / G0)
    for k, xv in raw.items():
        sens[k] = {str(n): round(lag_scan(tv, sigs[k][0], it, box(xv, n), per[k]["offset"], half=1.5)["offset"], 4)
                   for n in (1, 3, 5, 10)}
    imu_keys = ["yaw_vs_imu", "along_vs_imu", "alat_vs_imu"]
    w = np.array([1 / per[k]["block_se_s"] ** 2 for k in imu_keys])
    o = np.array([per[k]["offset"] for k in imu_keys])
    off = float((w * o).sum() / w.sum())                       # adopted: weighted mean of the IMU-based estimates
    stat = float(max(np.sqrt(1 / w.sum()), o.std(ddof=1) / np.sqrt(3)))
    allb = np.vstack([blocks_all[k] for k in imu_keys])
    dppm, dse = drift_fit(allb)
    T0 = base + int(round(off * 1e9))
    end = T0 + int(round(t_end * 1e9))
    S["obd"] = dict(prior_T0_utc=iso(prior_ns), per_signal=per, smoothing_sensitivity=sens,
                    offset_vs_prior_s=off, T0_ns=T0, stat_unc_s=stat,
                    speed_minus_imu_s=float(per["speed_vs_gps"]["offset"] - off),
                    drift_ppm=dppm, drift_se_ppm=dse, t_end_video_s=t_end, n_update_frames=int(len(ev)))
    S["window"] = (T0, end)
    S["curves"], S["blocks"] = curves, blocks_all

    # --- checks: GPS speed derivative vs IMU, seat vs IMU (ref time axis, seconds from T0)
    gt0, it0 = (gref - T0) / 1e9, (pi_to_ref(imu.t_utc_ns) - T0) / 1e9
    dv = np.gradient(gps.speed_mps.to_numpy(float), gt0)
    mv = gps.speed_mps.to_numpy(float) > 2
    s = lag_scan(gt0[mv], dv[mv], it0, box(imu.veh_acc_x_mps2.to_numpy(float), 10), 0.0, half=2.0, step=0.05)
    S["gps_vs_imu"] = dict(gps_lag_s=float(-s["offset"]), corr=s["corr"],
                           note="GPS dv/dt(t) best matches IMU veh_acc_x(t - lag): positive = GPS speed lags the IMU")
    sv = seat[seat.frame_valid == 1]
    stt = (sv.t_utc_ns.to_numpy(np.int64) - T0) / 1e9
    grid = np.arange(max(stt[0], 0) + 5, min(stt[-1], t_end) - 5, 0.1)
    hp = lambda x: x - box(x, 300)                              # remove >30 s trends
    X = np.column_stack([hp(np.interp(grid, stt, box(sv[c].to_numpy(float), 10))) for c in SEAT_CH])
    core = slice(400, -400)
    seatchk = {}
    for nm, col in (("lateral", "veh_acc_y_mps2"), ("longitudinal", "veh_acc_x_mps2")):
        y = hp(np.interp(grid, it0, box(imu[col].to_numpy(float), 5)))
        wgt = np.linalg.lstsq(X[core], y[core], rcond=None)[0]
        z = X @ wgt                                             # seat-predicted acceleration
        # IMU(t) vs seat(t + lag): positive lag = seat responds later than the IMU
        s = lag_scan(grid[core], y[core], grid, z, 0.0, half=1.5, step=0.1)
        bl = []
        for a in range(400, len(grid) - 400 - 1800, 1800):
            b = lag_scan(grid[a:a + 1800], y[a:a + 1800], grid, z, 0.0, half=1.5, step=0.1)
            if b["interior"]:
                bl.append(b["offset"])
        seatchk[nm] = dict(seat_lag_s=s["offset"], corr=s["corr"], blocks_s=[round(v, 3) for v in bl],
                           block_median_s=float(np.median(bl)), block_sd_s=float(np.std(bl, ddof=1)))
        seatchk[nm + "_curve"] = (s["lags"], s["r"])
    S["seat_vs_imu"] = seatchk
    S["pi_to_ref"] = pi_to_ref
    # camera cross-check: wall-based conversion vs direct monotonic conversion
    cam = load(R2 / "camera_frames.csv")
    direct = u0 + np.rint(np.polyval(p, (cam.capture_monotonic_ns.to_numpy(np.int64) - m0).astype(float))).astype(np.int64)
    S["ref"]["camera_wall_vs_mono_path_max_ms"] = float(np.abs(pi_to_ref(cam.t_utc_ns) - direct).max() / 1e6)
    return S


# ----------------------------------------------------------------------------- run 1
def run1_sync():
    obd = load(R1 / "obd.csv")
    gfr = load(R1 / "gps_fragments.csv")
    v = obd.vehicle_speed_kmh.to_numpy(float)
    n = len(v)
    assert (obd.sample_idx.to_numpy() == np.arange(1, n + 1)).all()
    g = gfr[(gfr.valid == 1) & gfr.gps_utc_corrected_ns.notna()]
    base = pd.Timestamp(f"{DRIVE_DATE}T06:30:00Z").value          # 15:30 KST
    gt = (g.gps_utc_corrected_ns.to_numpy(np.int64) - base) / 1e9
    gv = g.speed_kmh.to_numpy(float)
    sid = g.session_id.str[-6:].to_numpy()
    idx = np.arange(n, dtype=float)
    S = {}

    # --- global search: is (t_start, interval) unique?  cost = rms(OBD - GPS) over covered epochs,
    #     requiring >= 30 moving (>5 km/h) GPS epochs inside the OBD span.
    t0s = np.arange(0.0, 1200.0, 0.25)                           # 15:30 .. 15:50 KST
    dts = np.arange(0.100, 1.0001, 0.002)
    cost = np.full((len(dts), len(t0s)), np.nan)
    for i, dt in enumerate(dts):
        x = (gt[None, :] - t0s[:, None]) / dt
        pred = np.interp(x, idx, v, left=np.nan, right=np.nan)
        ok = ~np.isnan(pred)
        nmov = (ok & (gv[None, :] > 5)).sum(1)
        err = np.where(ok, (pred - gv[None, :]) ** 2, 0.0).sum(1) / np.maximum(ok.sum(1), 1)
        cost[i] = np.where(nmov >= 30, np.sqrt(err), np.nan)
    i, j = np.unravel_index(np.nanargmin(cost), cost.shape)
    far = cost.copy()
    far[max(i - 15, 0):i + 16, :] = np.nan                       # exclude |d interval| <= 0.03 s
    S["global"] = dict(best_rms_kmh=float(cost[i, j]), best_interval_s=float(dts[i]), best_t0_kst=iso(base + int(t0s[j] * 1e9), True),
                       second_best_rms_kmh=float(np.nanmin(far)), second_best_interval_s=float(dts[np.unravel_index(np.nanargmin(far), far.shape)[0]]),
                       search="interval 0.100..1.000 s step 0.002; start 15:30:00..15:50:00 KST step 0.25 s; >=30 moving GPS epochs covered")
    S["cost_profile"] = (dts, np.nanmin(np.where(np.isnan(cost), np.inf, cost), axis=1))

    # --- refinement: least squares on (t_start, interval, gain, intercept)
    def resid(p, mask):
        return np.interp((gt[mask] - p[0]) / p[1], idx, v) - (p[2] * gv[mask] + p[3])

    def fit(mask):
        r = least_squares(resid, [t0s[j], dts[i], 1.0, 0.0], args=(mask,), x_scale=[1, 1e-3, 1e-2, 0.1])
        cov = np.linalg.inv(r.jac.T @ r.jac) * (r.fun ** 2).sum() / (len(r.fun) - 4)
        return r.x, np.sqrt(np.diag(cov)), float(np.sqrt(np.mean(r.fun ** 2)))

    inw = (gt >= t0s[j] - 2) & (gt <= t0s[j] + dts[i] * (n - 1) + 2)
    px, se, rms = fit(inw)
    inw = (gt >= px[0]) & (gt <= px[0] + px[1] * (n - 1))        # epochs strictly inside the fitted span
    px, se, rms = fit(inw)
    end_s = px[0] + px[1] * (n - 1)
    used = pd.Series(sid[inw]).value_counts().sort_index()
    loo = {}
    for s in used.index[used >= 5]:
        q, _, _ = fit(inw & (sid != s))
        loo[s] = dict(t0_s=float(q[0] - px[0]), interval_s=float(q[1] - px[1]), end_s=float(q[0] + q[1] * (n - 1) - end_s))
    q, _, _ = fit(inw & np.isin(sid, used.index[used >= 5]))
    loo["all_single_epochs"] = dict(t0_s=float(q[0] - px[0]), interval_s=float(q[1] - px[1]), end_s=float(q[0] + q[1] * (n - 1) - end_s))
    local = {}                                                   # per-fragment residual time shift (uniform-interval check)
    for s in used.index:
        mk = inw & (sid == s)
        sh = np.arange(-3, 3.001, 0.02)
        c = [np.mean(resid([px[0] + d, px[1], px[2], px[3]], mk) ** 2) for d in sh]
        k = int(np.argmin(c))
        local[s] = dict(n=int(mk.sum()), shift_s=float(sh[k]), rms_kmh=float(np.sqrt(c[k])),
                        rms_at_zero_kmh=float(np.sqrt(np.mean(resid(px, mk) ** 2))),
                        speed_span_kmh=float(np.ptp(gv[mk])))
    T0 = base + int(round(px[0] * 1e9))
    step_ns = px[1] * 1e9
    t_est = T0 + np.rint(idx * step_ns).astype(np.int64)
    # uncertainties: max of formal SE and leave-one-out spread
    u_t0 = max(se[0], max(abs(d["t0_s"]) for d in loo.values()))
    u_dt = max(se[1], max(abs(d["interval_s"]) for d in loo.values()))
    u_end = max(abs(d["end_s"]) for d in loo.values())
    S["fit"] = dict(t_start_ns=int(T0), interval_s=float(px[1]), interval_ns_exact=repr(float(step_ns)), gain=float(px[2]), intercept_kmh=float(px[3]), rms_kmh=rms,
                    n_epochs=int(inw.sum()), epochs_by_session={k: int(c) for k, c in used.items()},
                    formal_se=dict(t_start_s=float(se[0]), interval_s=float(se[1])), leave_one_out=loo, local_shift=local,
                    unc_t_start_s=float(u_t0), unc_interval_s=float(u_dt), unc_end_s=float(u_end),
                    header_time_minus_fit_s=float((pd.Timestamp(f"{DRIVE_DATE}T06:41:00Z").value - T0) / 1e9))
    S["window"] = (int(t_est[0]), int(t_est[-1]))
    S["t_est"] = t_est
    S["plot"] = dict(gt=(g.gps_utc_corrected_ns.to_numpy(np.int64) - T0) / 1e9, gv=gv, sid=sid, inw=inw, v=v,
                     t=(t_est - T0) / 1e9, gain=px[2], icpt=px[3])

    # --- Pi fragment clock: per-session median (wall - corrected GPS UTC); sessions without a fix are interpolated
    d = (g.t_utc_ns.to_numpy(np.int64) - g.gps_utc_corrected_ns.to_numpy(np.int64))
    tab = pd.DataFrame(dict(s=g.session_id.to_numpy(), t=g.t_utc_ns.to_numpy(np.int64), d=d)).groupby("s").agg(t=("t", "median"), d=("d", "median")).sort_values("t")
    S["frag_clock"] = tab

    S["frag_clock_ns"] = {}                                      # session -> (constant offset ns, source)

    def frag_to_ref(df):
        t = df.t_utc_ns.to_numpy(np.int64)
        for s, tm in df.groupby("session_id").t_utc_ns.median().items():
            if s in tab.index:
                S["frag_clock_ns"][s] = (int(np.rint(tab.d[s])), "gps")
            elif s not in S["frag_clock_ns"]:                    # no fix in this session: interpolate between neighbours
                c = np.interp(tm - tab.t.iloc[0], (tab.t.to_numpy() - tab.t.iloc[0]).astype(float), tab.d.to_numpy(float))
                S["frag_clock_ns"][s] = (int(np.rint(c)), "interpolated")
        return t - df.session_id.map({k: v[0] for k, v in S["frag_clock_ns"].items()}).to_numpy(np.int64)
    S["frag_to_ref"] = frag_to_ref
    return S


# ----------------------------------------------------------------------------- 10 Hz tables
def fmt(a, dec):
    """Fixed-point text without trailing zeros; NaN -> empty."""
    return ["" if np.isnan(x) else (("%.*f" % (dec, x)).rstrip("0").rstrip(".") if dec else "%d" % round(x)) for x in np.asarray(a, float)]


def zoh_index(t_src, grid):
    """Index of the last source sample at or before each grid time (-1 if none)."""
    return np.searchsorted(t_src, grid, side="right") - 1


def interp_valid(grid, t, y, max_gap):
    """Linear interpolation; valid only between two samples no more than max_gap apart."""
    j = np.clip(np.searchsorted(t, grid, side="right"), 1, len(t) - 1)
    ok = (grid >= t[j - 1]) & (grid <= t[j]) & (t[j] - t[j - 1] <= max_gap)
    return np.where(ok, np.interp(grid, t, y), np.nan), ok


def bin_count(t, grid, half=0.05):
    return np.searchsorted(t, grid + half, side="left") - np.searchsorted(t, grid - half, side="left")


def seat_block(cols, grid, seat_frames):
    """seat_frames: list of (t_run_s array, DataFrame of valid frames). 100 ms mean per raw channel + count."""
    t = np.concatenate([a for a, _ in seat_frames]) if seat_frames else np.array([])
    lo = np.searchsorted(t, grid - 0.05, side="left")
    hi = np.searchsorted(t, grid + 0.05, side="left")
    cnt = hi - lo
    for c in SEAT_CH:
        x = np.concatenate([d[c].to_numpy(float) for _, d in seat_frames]) if seat_frames else np.array([])
        cs = np.concatenate([[0.0], np.nancumsum(x)])
        nn = np.concatenate([[0], np.cumsum(~np.isnan(x))])
        k = nn[hi] - nn[lo]
        cols["seat_" + c + "_mean"] = fmt(np.where(k > 0, (cs[hi] - cs[lo]) / np.maximum(k, 1), np.nan), 3)
    cols["seat_n"] = cnt.astype(str)


def write_table(path, cols):
    names = list(cols)
    n = len(cols[names[0]])
    buf = io.StringIO()
    buf.write(",".join(names) + "\n")
    for row in zip(*[cols[c] for c in names]):
        buf.write(",".join(row) + "\n")
    write_text(path, buf.getvalue())
    return n


POSE_COLS = ["shoulder_center_x", "shoulder_center_y", "hip_center_x", "hip_center_y", "head_center_x", "head_center_y",
             "shoulder_width", "torso_angle_deg", "shoulder_shift_x", "shoulder_shift_y", "hip_shift_x", "hip_shift_y",
             "head_shift_x", "head_shift_y", "lean_score", "pose_confidence"]
OBD2 = ["engine_rpm", "vehicle_speed_kmh", "accel_long_g", "steering_angle_deg", "accel_lat_g", "yaw_rate_dps"]
OBD2_DEC = [0, 0, 2, 1, 2, 1]
OBD1 = ["throttle_rel_pct", "throttle_abs_pct", "reference_torque_nm", "engine_rpm", "vehicle_speed_kmh"]
OBD1_DEC = [3, 3, 0, 1, 0]


def synced_run2(T0, end):
    """All inputs are read back from the refined files (t_run_s already added)."""
    rd = lambda f: pd.read_csv(R2 / f, low_memory=False)
    obd, gps, imu, pose, seat = rd("obd.csv"), rd("gps.csv"), rd("imu.csv"), rd("pose.csv"), rd("seat.csv")
    n = int(np.floor((end - T0) / 1e8)) + 1
    grid = np.arange(n) / 10.0
    cols = {"t_utc_ns": [str(T0 + k * 100_000_000) for k in range(n)], "t_run_s": fmt(grid, 1)}
    # OBD: zero-order hold of the frame shown at the grid time + age since the last display update
    to = obd.t_run_s.to_numpy(float)
    j = zoh_index(to, grid + 1e-9)
    upd = to[obd.any_value_changed.to_numpy() == 1]
    ju = zoh_index(upd, grid + 1e-9)
    cols["obd_frame_idx"] = obd.frame_idx.to_numpy()[j].astype(str)
    for c, d in zip(OBD2, OBD2_DEC):
        cols["obd_" + c] = fmt(obd[c].to_numpy(float)[j], d)
    cols["obd_age_s"] = fmt(np.where(ju >= 0, grid - upd[np.maximum(ju, 0)], np.nan), 3)
    cols["obd_valid"] = obd[[c + "_ok" for c in OBD2]].to_numpy().all(1)[j].astype(int).astype(str)
    # GPS: linear interpolation between valid epochs <= 1.5 s apart
    gg = gps[gps.valid == 1]
    tg = gg.t_run_s.to_numpy(float)
    for c, d in (("lat_deg", 9), ("lon_deg", 9), ("east_m", 3), ("north_m", 3), ("speed_kmh", 3)):
        val, ok = interp_valid(grid, tg, gg[c].to_numpy(float), 1.5)
        cols["gps_" + c] = fmt(val, d)
    cols["gps_valid"] = ok.astype(int).astype(str)
    # IMU: linear interpolation between samples <= 0.25 s apart; imu_n = samples inside the +-50 ms bin
    ti = imu.t_run_s.to_numpy(float)
    for c in ("veh_acc_x_mps2", "veh_acc_y_mps2", "veh_acc_z_mps2", "veh_gyro_x_radps", "veh_gyro_y_radps", "veh_gyro_z_radps"):
        val, ok = interp_valid(grid, ti, imu[c].to_numpy(float), 0.25)
        cols["imu_" + c] = fmt(val, 6)
    cols["imu_n"] = bin_count(ti, grid).astype(str)
    cols["imu_valid"] = ok.astype(int).astype(str)
    # pose: nearest pose frame within +-75 ms (no interpolation)
    tp = pose.t_run_s.to_numpy(float)
    k = np.clip(np.searchsorted(tp, grid), 1, len(tp) - 1)
    k = np.where(np.abs(tp[k - 1] - grid) <= np.abs(tp[k] - grid), k - 1, k)
    near = np.abs(tp[k] - grid) <= 0.075
    cols["pose_frame_id"] = [str(v) if o else "" for v, o in zip(pose.frame_id.to_numpy()[k], near)]
    cols["pose_dt_ms"] = fmt(np.where(near, (tp[k] - grid) * 1e3, np.nan), 3)
    for c in POSE_COLS:
        cols[c if c.startswith("pose_") else "pose_" + c] = fmt(np.where(near, pose[c].to_numpy(float)[k], np.nan), 5)
    cols["pose_valid"] = (near & (pose.vision_valid.to_numpy()[k] == 1)).astype(int).astype(str)
    cols["pose_shift_valid"] = (near & (pose.shift_valid.to_numpy()[k] == 1)).astype(int).astype(str)
    # seat: mean of valid frames with t in [t-50 ms, t+50 ms)
    sv = seat[seat.frame_valid == 1]
    seat_block(cols, grid, [(sv.t_run_s.to_numpy(float), sv)])
    if (R2 / "pose_corrected.csv").exists():
        pc = rd("pose_corrected.csv")
        tc = pc.t_run_s.to_numpy(float)
        assert len(tc) and np.all(np.diff(tc) > 0), "corrected pose times must increase"
        k = np.searchsorted(tc, grid)
        left, right = np.maximum(k - 1, 0), np.minimum(k, len(tc) - 1)
        k = np.where(np.abs(tc[left] - grid) <= np.abs(tc[right] - grid), left, right)
        near = np.abs(tc[k] - grid) <= 0.025
        cols["posec_frame_id"] = [str(v) if o else "" for v, o in zip(pc.frame_id.to_numpy()[k], near)]
        cols["posec_dt_ms"] = fmt(np.where(near, (tc[k] - grid) * 1e3, np.nan), 3)
        for c in POSE_COLS:
            name = "posec_" + ("confidence" if c == "pose_confidence" else c)
            cols[name] = fmt(np.where(near, pc[c].to_numpy(float)[k], np.nan), 5)
        cols["posec_valid"] = (near & (pc.vision_valid.to_numpy()[k] == 1)).astype(int).astype(str)
        cols["posec_shift_valid"] = (near & (pc.shift_valid.to_numpy()[k] == 1)).astype(int).astype(str)
        lm = rd("pose_landmarks_corrected.csv")
        assert lm.frame_id.is_unique, "duplicate corrected landmark frame"
        reliable = lm.set_index("frame_id").n_reliable.reindex(pc.frame_id)
        assert reliable.notna().all(), "missing corrected landmark frame"
        cols["posec_n_reliable"] = fmt(np.where(near, reliable.to_numpy(float)[k], np.nan), 0)
    return write_table(R2 / "synced_10hz.csv", cols)


def synced_run1(T0, end):
    obd = pd.read_csv(R1 / "obd.csv")
    gps = pd.read_csv(R1 / "gps_fragments.csv", low_memory=False)
    n = int(np.floor((end - T0) / 1e8)) + 1
    grid = np.arange(n) / 10.0
    cols = {"t_utc_ns_est": [str(T0 + k * 100_000_000) for k in range(n)], "t_run_s": fmt(grid, 1)}
    to = obd.t_run_s.to_numpy(float)
    j = zoh_index(to, grid + 1e-9)
    cols["obd_sample_idx"] = obd.sample_idx.to_numpy()[j].astype(str)
    for c, d in zip(OBD1, OBD1_DEC):
        cols["obd_" + c] = fmt(obd[c].to_numpy(float)[j], d)
    cols["obd_age_s"] = fmt(grid - to[j], 3)
    cols["obd_valid"] = obd.valid.to_numpy()[j].astype(str)
    gg = gps[(gps.valid == 1)].sort_values("t_run_s")
    tg = gg.t_run_s.to_numpy(float)
    for c, d in (("lat_deg", 9), ("lon_deg", 9), ("east_m", 3), ("north_m", 3), ("speed_kmh", 3)):
        val, ok = interp_valid(grid, tg, gg[c].to_numpy(float), 1.5)
        cols["gps_" + c] = fmt(val, d)
    cols["gps_valid"] = ok.astype(int).astype(str)
    frames = []
    for f in sorted(R1.glob("seat_session_*.csv")):
        s = pd.read_csv(f)
        s = s[s.frame_valid == 1]
        frames.append((s.t_run_s.to_numpy(float), s))
    seat_block(cols, grid, frames)
    return write_table(R1 / "synced_10hz.csv", cols)
