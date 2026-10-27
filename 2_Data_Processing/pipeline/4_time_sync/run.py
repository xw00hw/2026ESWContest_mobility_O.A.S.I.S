"""Part D (sync / integration): run AFTER pipeline/1_vehicle_obd_ocr, 2_seat_pressure, 3_pi_gps_imu_camera.

  python pipeline\\4_time_sync\\run.py

1. estimates the time models (run2: OBD video -> reference clock; run1: OBD sample index -> time)
2. adds t_run_s / in_obd_window to every refined table (and t_utc_ns / t_utc_ns_est to OBD tables)
3. writes 01_refined/run{1,2}/sync.json and synced_10hz.csv
4. reports the OBD windows and clock uncertainty
Idempotent: columns added by a previous run are stripped before anything is estimated.
"""
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
import sync   # noqa: E402
from sync import R1, R2, ROOT, iso, load, add_cols, write_text  # noqa: E402


def clean(o):
    """numpy -> plain python for json."""
    if isinstance(o, dict):
        return {str(k): clean(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [clean(v) for v in o]
    if isinstance(o, (np.floating, float)):
        return round(float(o), 9)
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, np.bool_):
        return bool(o)
    return o


def window_json(a, b):
    return dict(start_ns=int(a), end_ns=int(b), start_utc=iso(a), end_utc=iso(b), start_kst=iso(a, True), end_kst=iso(b, True),
                duration_s=round((b - a) / 1e9, 6))


def main():
    S2 = sync.run2_sync()
    S1 = sync.run1_sync()
    counts = {}

    def add(path, t_ref, win, head=None):
        counts[path.relative_to(ROOT).as_posix()] = (len(t_ref), add_cols(path, t_ref, win[0], win[1], head))

    # ------------------------------------------------------------------ run 2 columns
    T0, end = S2["window"]
    to_ref = S2["pi_to_ref"]
    for f in ("gps.csv", "imu.csv", "pose.csv", "pose_landmarks.csv", "camera_frames.csv"):
        add(R2 / f, to_ref(load(R2 / f).t_utc_ns), (T0, end))
    for f in ("pose_landmarks_reinferred.csv", "pose_landmarks_corrected.csv", "pose_corrected.csv",
              "pose_before_after.csv", "pose_frame_diagnosis.csv"):
        if (R2 / f).exists():
            add(R2 / f, to_ref(load(R2 / f).t_utc_ns), (T0, end))
    add(R2 / "seat.csv", load(R2 / "seat.csv").t_utc_ns.to_numpy(np.int64), (T0, end))
    obd = load(R2 / "obd.csv")
    t_obd = T0 + np.rint(obd.t_video_s.to_numpy(float) * 1e9).astype(np.int64)
    add(R2 / "obd.csv", t_obd, (T0, end), "t_utc_ns")
    txt = load(R2 / "obd_text.csv")
    assert (txt.frame_idx.to_numpy() == obd.frame_idx.to_numpy()).all()
    add(R2 / "obd_text.csv", t_obd, (T0, end), "t_utc_ns")
    ch = load(R2 / "obd_changes.csv")
    add(R2 / "obd_changes.csv", T0 + np.rint(ch.t_video_s.to_numpy(float) * 1e9).astype(np.int64), (T0, end), "t_utc_ns")

    # ------------------------------------------------------------------ run 1 columns
    W1 = S1["window"]
    add(R1 / "obd.csv", S1["t_est"], W1, "t_utc_ns_est")
    for f in ("gps_fragments.csv", "imu_fragments.csv"):
        add(R1 / f, S1["frag_to_ref"](load(R1 / f)), W1)
    seat_sessions = []
    for p in sorted(R1.glob("seat_session_*.csv")):
        t = load(p).t_utc_ns.to_numpy(np.int64)
        add(p, t, W1)
        nin = counts[p.relative_to(ROOT).as_posix()][1]
        if t[-1] < W1[0]:
            rel = f"구간 시작 {(W1[0] - t[-1]) / 1e9:.2f} s 전에 종료 (밖)"
        elif t[0] > W1[1]:
            rel = f"구간 끝 {(t[0] - W1[1]) / 1e9:.2f} s 후에 시작 (밖)"
        else:
            rel = f"겹침 {nin}프레임"
        seat_sessions.append(dict(file=p.name, first_utc=iso(t[0]), last_utc=iso(t[-1]), first_kst=iso(t[0], True), last_kst=iso(t[-1], True),
                                  rows=len(t), rows_in_obd_window=nin, relation=rel,
                                  margin_before_start_s=round((W1[0] - t[-1]) / 1e9, 3), margin_after_end_s=round((t[0] - W1[1]) / 1e9, 3)))

    # ------------------------------------------------------------------ sync.json (run 2)
    r, o = S2["ref"], S2["obd"]
    syst = 0.15
    j2 = dict(
        schema="esw-sync/1", run="run2", generated_by="pipeline/4_time_sync/run.py",
        reference_clock=dict(
            name="pi_monotonic_on_corrected_gps_utc",
            definition="t_ref_ns = u0 + slope*(host_monotonic_ns - m0) + intercept_ns, least-squares fit of gps_utc_corrected_ns on the Pi monotonic clock at RMC arrival (run2 gps.csv)",
            m0=r["m0"], u0=r["u0"], slope=repr(r["slope"]), intercept_ns=r["intercept_ns"], slope_minus1_ppm=r["slope_minus1_ppm"], n_epochs=r["n_epochs"],
            fit_residual_ms=r["resid_ms"], pi_wall_minus_ref_ms=r["wall_minus_ref_ms"],
            camera_wall_vs_mono_path_max_ms=r["camera_wall_vs_mono_path_max_ms"],
            pi_wall_minus_ref_knots_ms=r["knots"],
            bias_note="GPS rows land on their epoch time. Other Pi timestamps read (true UTC - NMEA output latency); that latency is unknown (plausibly 0..0.1 s) and is NOT corrected."),
        obd_window=window_json(T0, end),
        t_run_s="t_run_s = (t_ref_ns - obd_window.start_ns)/1e9 ; in_obd_window = 1 if start_ns <= t_ref_ns <= end_ns",
        devices=dict(
            obd_video=dict(
                model="t_ref_ns = T0_ns + round(t_video_s*1e9)   (drift fixed to 0)",
                T0_ns=o["T0_ns"], T0_utc=iso(o["T0_ns"]), T0_kst=iso(o["T0_ns"], True),
                prior_T0_utc=o["prior_T0_utc"], offset_vs_prior_s=o["offset_vs_prior_s"],
                method="Display-update frames only (any_value_changed=1). For each signal pair the offset maximising correlation (free gain+intercept) is scanned at 0.02 s and refined by a parabola. Adopted T0 = inverse-variance weighted mean of the three IMU-based estimates; variances from independent blocks (recording split in 12; blocks with too little signal are dropped, see n_blocks per signal).",
                n_update_frames=o["n_update_frames"], per_signal=o["per_signal"], smoothing_sensitivity=o["smoothing_sensitivity"],
                speed_based_minus_adopted_s=o["speed_minus_imu_s"],
                drift_ppm_estimated=o["drift_ppm"], drift_ppm_se=o["drift_se_ppm"], drift_ppm_applied=0.0,
                uncertainty_s=dict(statistical=o["stat_unc_s"], systematic=syst,
                                   note="IMU 타임스탬프는 도착 시각(측정보다 ≥45.8 ms 늦음), 기준 시계의 NMEA 지연(0~0.1 s) 미지, 신호별 표시 지연 차이 가능성, 속도↔GPS 추정과의 차이(GPS 속도 필터 지연으로 설명되나 분리 불가)를 합친 추정 한계"),
                sign_mapping="yaw_rate_dps(+) = veh_gyro_z(+) 좌회전; accel_long_g(+) = veh_acc_x(+) 가속; accel_lat_g(+) = veh_acc_y(+) 좌측",
                meaning="갱신 프레임의 표시값이 IMU 에서 같은 값으로 관측된 기준 시계상 시각. 표시 지연은 T0 에 흡수됨(화면 녹화 시각이 아님). 갱신 사이의 프레임은 0차 홀드된 과거 값.")),
        checks=dict(gps_speed_vs_imu=S2["gps_vs_imu"],
                    seat_vs_imu={k: v for k, v in S2["seat_vs_imu"].items() if not k.endswith("_curve")}),
    )
    pi = dict(model="t_ref_ns = t_utc_ns - c(t_utc_ns);  c = (gps.t_utc_ns - t_ref_ns(gps.host_monotonic_ns)) at every GPS row, linearly interpolated in t_utc_ns (held constant outside)",
              uncertainty_s=0.005)
    j2["devices"]["gps"] = dict(pi, note="t_utc_ns = RMC arrival; on the reference clock this equals the GPS epoch time within the fit residual")
    j2["devices"]["imu"] = dict(pi, known_latency_not_applied_s=0.0458, note="t_utc_ns = reconstructed arrival of the last byte; measurement is >= 45.8 ms earlier")
    j2["devices"]["camera"] = dict(pi, note="capture time")
    j2["devices"]["pose"] = dict(pi, note="capture time of the analysed frame")
    j2["devices"]["seat"] = dict(model="t_ref_ns = t_utc_ns  (offset 0, no correction applied)", uncertainty_s=0.1,
                                 known_latency_not_applied_s="0.001-0.003",
                                 note="시트 t_utc_ns 는 NTP 기반 UTC(±25~30 ms). 기준 시계와의 관계는 NMEA 지연(미지)만큼 불확실. 물리 교차점검(checks.seat_vs_imu)은 신체 반응 지연과 구분되지 않아 보정 근거로 쓰지 않음.")
    j2 = clean(j2)
    write_text(R2 / "sync.json", json.dumps(j2, ensure_ascii=False, indent=1) + "\n")

    # ------------------------------------------------------------------ sync.json (run 1)
    f = S1["fit"]
    j1 = clean(dict(
        schema="esw-sync/1", run="run1", generated_by="pipeline/4_time_sync/run.py",
        time_quality="estimated — OBD 에 타임스탬프가 없어 GPS 조각 속도 정합으로 추정. 해는 유일하고 독립 근거(엑셀 머리말 15:41:00, 단발 epoch 3곳)와 일치하나, "
                     "GPS 가 없는 구간의 샘플 간격 균일성은 검증 불가. 시간 컬럼은 t_utc_ns_est 로 표기.",
        reference_clock=dict(name="corrected_gps_utc (Pi fragments) / NTP UTC (seat)",
                             note="Pi 조각: t_ref = t_utc_ns - 세션별 median(t_utc_ns - gps_utc_corrected_ns); fix 없는 세션은 인접 세션 값 보간. 시트: t_utc_ns 그대로."),
        obd_window=window_json(*W1),
        obd_time_model=dict(model="t_utc_ns_est = t_start_ns + round((sample_idx-1)*float(interval_ns_exact))  (interval_s is the same value rounded for reading)",
                            t_start_ns=f["t_start_ns"], interval_ns_exact=f["interval_ns_exact"],
                            t_start_utc=iso(f["t_start_ns"]), t_start_kst=iso(f["t_start_ns"], True), interval_s=f["interval_s"],
                            gain=f["gain"], intercept_kmh=f["intercept_kmh"], rms_kmh=f["rms_kmh"], n_epochs=f["n_epochs"],
                            epochs_by_session=f["epochs_by_session"], formal_se=f["formal_se"], leave_one_out=f["leave_one_out"],
                            local_shift=f["local_shift"],
                            uncertainty=dict(t_start_s=f["unc_t_start_s"], interval_s=f["unc_interval_s"], end_s=f["unc_end_s"],
                                             note="max(형식 표준오차, leave-one-fragment-out 변화). GPS 연속 구간(15:45:02~15:46:32 KST) 밖에서는 균일 간격 가정에 의존"),
                            header_time_minus_fit_s=f["header_time_minus_fit_s"]),
        global_search=S1["global"],
        fragment_clock=dict(model="t_ref_ns = t_utc_ns - offset_ns (one constant per Pi session). source gps = median(t_utc_ns - gps_utc_corrected_ns) over the valid epochs of that session; "
                                  "interpolated = session has no dated fix, value interpolated in time between neighbouring sessions",
                            sessions={k: dict(offset_ns=v[0], source=v[1]) for k, v in sorted(S1["frag_clock_ns"].items())}),
        seat_sessions=seat_sessions,
        seat_overlap_rows=sum(s["rows_in_obd_window"] for s in seat_sessions),
    ))
    write_text(R1 / "sync.json", json.dumps(j1, ensure_ascii=False, indent=1) + "\n")

    # ------------------------------------------------------------------ synced tables, docs
    n2 = sync.synced_run2(T0, end)
    n1 = sync.synced_run1(*W1)
    counts["01_refined/run2/synced_10hz.csv"] = (n2, n2)
    counts["01_refined/run1/synced_10hz.csv"] = (n1, n1)
    print(f"run2 OBD window {j2['obd_window']['start_kst']} .. {j2['obd_window']['end_kst']}  T0 stat +-{j2['devices']['obd_video']['uncertainty_s']['statistical']:.3f} s")
    print(f"run1 OBD window {j1['obd_window']['start_kst']} .. {j1['obd_window']['end_kst']}  interval {f['interval_s']:.5f} s  seat overlap rows {j1['seat_overlap_rows']}")
    print("Part D done.")


if __name__ == "__main__":
    main()
