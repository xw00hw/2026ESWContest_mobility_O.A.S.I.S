"""Refine run2 camera timestamps and pose using the collector's exact formulas."""
from pathlib import Path
from types import SimpleNamespace
from datetime import timezone, timedelta
import csv
import hashlib
from decimal import Decimal
import importlib.util
import json
import math
import re
import subprocess
import sys

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.dates as mdates

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[2]
RAW = ROOT / "00_raw/pi_run2/session_run2"
OUT = ROOT / "01_refined/run2"
QC = ROOT / "02_qc/3_pi_gps_imu_camera"
KST = timezone(timedelta(hours=9))
CENTERS = [f"{p}_center_{a}" for p in ("shoulder", "hip", "head") for a in ("x", "y")]
GEOMETRY = CENTERS + ["shoulder_width", "torso_angle_deg"]
SCALES = np.array([640, 400] * 3)
WIDTH_PERCENT_WEIGHT_PX2 = 1.0


def read_csv(path):
    header = pd.read_csv(path, nrows=0).columns
    types = {c: "Int64" for c in header if c.endswith("_ns")}
    if "landmark_valid_mask" in header:
        types["landmark_valid_mask"] = str
    return pd.read_csv(path, dtype=types, float_precision="round_trip")


def write_csv(df, path):
    output = df.copy()
    for c in output:
        if c.endswith("_ns"):
            output[c] = pd.array(output[c], dtype="Int64")
        elif path.name in ("pose.csv", "pose_landmarks.csv") and pd.api.types.is_float_dtype(output[c]):
            digits = (4 if c.endswith(("_vis", "_pres")) or c == "pose_confidence" or "torso_angle" in c
                      else 3 if c.endswith(("_px", "_ms"))
                      else 5 if c.endswith(("_x", "_y", "_z")) or c in ("shoulder_width", "lean_score", "lean_velocity")
                      else None)
            def fixed(v):
                if pd.isna(v):
                    return ""
                text = f"{v:.{digits}f}" if digits is not None else format(Decimal(str(v)), "f")
                return text.rstrip("0").rstrip(".") if "." in text else text
            output[c] = output[c].map(fixed)
    output.to_csv(path, index=False, encoding="utf-8", lineterminator="\n")
    assert not re.search(r"(?i)\d(?:\.\d*)?e[+-]\d+", path.read_text(encoding="utf-8")), path
    with path.open(encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        ns = [c for c in reader.fieldnames if c.endswith("_ns")]
        for row in reader:
            assert all(not row[c] or re.fullmatch(r"-?\d+", row[c]) for c in ns), row


def clean(value):
    if isinstance(value, dict):
        return {str(k): clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    if isinstance(value, np.generic):
        return clean(value.item())
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if value is pd.NA:
        return None
    return value


def write_json(value, path):
    path.write_text(json.dumps(clean(value), ensure_ascii=False, indent=2,
                               allow_nan=False) + "\n", encoding="utf-8", newline="\n")


def kst(ns):
    return pd.Timestamp(int(ns), unit="ns", tz="UTC").tz_convert(KST).isoformat()


def dates(ns):
    return pd.to_datetime(ns.astype("int64"), unit="ns", utc=True).dt.tz_convert(KST)


def summary(values):
    a = np.asarray(values, dtype=float)
    a = a[np.isfinite(a)]
    return {"n": len(a), "min": float(a.min()) if len(a) else None,
            "max": float(a.max()) if len(a) else None,
            "mean": float(a.mean()) if len(a) else None,
            **{f"p{q}": float(np.percentile(a, q)) if len(a) else None
               for q in (5, 50, 95, 99)}}


def load_collector():
    path = ROOT / "_work/3_pi_gps_imu_camera/src_ref/src_oasis_daq_pose_features.py"
    spec = importlib.util.spec_from_file_location("wp2_collector_pose", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def camera(raw, events, stats):
    c = raw.loc[raw.recorded == 1].copy().reset_index(drop=True)
    assert raw.frame_id.is_unique
    assert np.all(np.diff(c.frame_id.to_numpy()) > 0)
    assert np.all(np.diff(c.host_wall_time_ns.to_numpy(dtype=np.int64)) > 0)
    assert np.all(np.diff(c.capture_monotonic_ns.to_numpy(dtype=np.int64)) > 0)
    bases = {}
    for sid, part in c.groupby("segment_id", sort=True):
        idx = part.frame_index_in_segment.to_numpy()
        base = int(idx[0])
        assert base in (0, 1) and np.array_equal(idx, np.arange(len(idx)) + base)
        bases[int(sid)] = base
    c["frame_in_segment"] = c.frame_index_in_segment - c.segment_id.map(bases)
    c["segment_file"] = c.segment_id.map(
        lambda n: f"00_raw/pi_run2/session_run2/camera/driver_{n:03d}.mkv")
    c["t_utc_ns"] = c.host_wall_time_ns
    c["recorded_idx"] = np.arange(len(c))
    c["dt_prev_s"] = c.capture_monotonic_ns.diff().astype(float) / 1e9
    c["flag_gap_before"] = (c.dt_prev_s > 0.06).astype(int)
    c["skipped_capture_frames"] = pd.array(c.frame_id.diff() - 1, dtype="Int64")
    c["video_pts_s"] = np.nan
    segment_rows = []
    manifest = {}
    with (RAW / "camera/segments.csv").open(newline="", encoding="utf-8") as stream:
        for name, start, end in csv.reader(stream):
            manifest[name] = {"listed_start_s": float(start), "listed_end_s": float(end)}
    for sid, part in c.groupby("segment_id", sort=True):
        path = ROOT / part.segment_file.iloc[0]
        common = ["ffprobe", "-v", "error", "-select_streams", "v:0"]
        stream = json.loads(subprocess.check_output(
            common + ["-count_packets", "-show_entries",
                      "stream=nb_read_packets,codec_name,width,height,r_frame_rate,avg_frame_rate",
                      "-of", "json", str(path)], text=True))["streams"][0]
        pts_text = subprocess.check_output(
            common + ["-show_entries", "packet=pts_time", "-of", "csv=p=0", str(path)],
            text=True)
        pts = [float(line.strip().split(",")[0]) if line.strip().split(",")[0] != "N/A"
               else None for line in pts_text.splitlines() if line.strip()]
        duration = json.loads(subprocess.check_output(
            common + ["-show_entries", "format=duration", "-of", "json", str(path)],
            text=True))["format"].get("duration")
        matched = min(len(part), len(pts))
        c.loc[part.index[:matched], "video_pts_s"] = pts[:matched]
        finite = np.array([v for v in pts if v is not None])
        error = (float(np.max(np.abs(finite - np.arange(len(pts)) / 25)))
                 if len(finite) == len(pts) and len(pts) else None)
        elapsed = (int(part.capture_monotonic_ns.iloc[-1]) -
                   int(part.capture_monotonic_ns.iloc[0])) / 1e9
        segment_rows.append({
            "segment_id": int(sid), "segment_file": part.segment_file.iloc[0],
            "csv_frames": len(part), "mkv_packets": int(stream["nb_read_packets"]),
            "diff": int(stream["nb_read_packets"]) - len(part), "pts_count": len(pts),
            "frame_index_source_base": bases[int(sid)], "stream": stream,
            "duration_s": float(duration) if duration else None,
            "pts_n_over_25_max_abs_error_s": error,
            "pts_is_n_over_25": error is not None and error < 1e-6,
            "actual_capture_elapsed_s": elapsed,
            "actual_stored_fps": len(part) / elapsed,
            "actual_interval_fps": (len(part) - 1) / elapsed,
            "video_timeline_slow_percent": ((float(finite[-1] - finite[0]) / elapsed - 1) * 100)
            if len(finite) else None,
            "video_first_to_last_pts_s": float(finite[-1] - finite[0]) if len(finite) else None,
            "actual_minus_video_elapsed_s": elapsed - float(finite[-1] - finite[0])
            if len(finite) else None, **manifest.get(path.name, {})})
    gaps = []
    for i in c.index[c.flag_gap_before == 1]:
        prev, row = c.iloc[i - 1], c.iloc[i]
        gaps.append({"t_utc_ns": int(row.t_utc_ns),
                     "start_t_utc_ns": int(prev.t_utc_ns),
                     "end_t_utc_ns": int(row.t_utc_ns),
                     "start_kst": kst(prev.t_utc_ns), "end_kst": kst(row.t_utc_ns),
                     "duration_s": float(row.dt_prev_s),
                     "previous_frame_id": int(prev.frame_id), "next_frame_id": int(row.frame_id),
                     "previous_segment_id": int(prev.segment_id),
                     "next_segment_id": int(row.segment_id)})
    gapdf = pd.DataFrame(gaps, columns=["t_utc_ns", "start_t_utc_ns", "end_t_utc_ns",
        "start_kst", "end_kst", "duration_s", "previous_frame_id", "next_frame_id",
        "previous_segment_id", "next_segment_id"])
    write_csv(gapdf, QC / "camera_gaps.csv")
    largest = max(gaps, key=lambda g: g["duration_s"]) if gaps else None
    related = []
    if largest:
        for ev in events:
            if ev.get("source") == "camera" and (
                largest["start_t_utc_ns"] <= ev.get("host_wall_time_ns", 0)
                <= largest["end_t_utc_ns"]):
                related.append({**ev, "time_kst": kst(ev["host_wall_time_ns"])})
    missing_ranges = []
    for i in np.flatnonzero(np.diff(raw.frame_id.to_numpy()) > 1) + 1:
        before, after = raw.iloc[i - 1], raw.iloc[i]
        containing = [g for g in gaps if g["previous_frame_id"] <= before.frame_id
                      and g["next_frame_id"] >= after.frame_id]
        missing_ranges.append({"first_missing_frame_id": int(before.frame_id) + 1,
            "last_missing_frame_id": int(after.frame_id) - 1,
            "missing_count": int(after.frame_id - before.frame_id - 1),
            "start_kst": kst(before.host_wall_time_ns), "end_kst": kst(after.host_wall_time_ns),
            "containing_recorded_gap_s": containing[0]["duration_s"] if containing else None})
    missing_count = sum(r["missing_count"] for r in missing_ranges)
    write_drops = stats["session_summary_workers"]["camera"]["write_queue_drops"]
    assert missing_count == write_drops
    largest_two = sorted(gaps, key=lambda g: g["duration_s"], reverse=True)[:2]
    missing_in_largest_two = sum(r["missing_count"] for r in missing_ranges
        if any(g["previous_frame_id"] < r["first_missing_frame_id"] and
               g["next_frame_id"] > r["last_missing_frame_id"] for g in largest_two))
    assert missing_in_largest_two == missing_count
    stats["camera_missing_capture_rows"] = {"missing_frame_ids": missing_count,
        "write_queue_drops": write_drops, "missing_in_two_largest_recorded_gaps": missing_in_largest_two,
        "ranges": missing_ranges, "two_largest_recorded_gaps": largest_two,
        "note": "These frame IDs have no rows in frame_timestamps.csv; all are inside the two largest recorded gaps."}
    intervals = c.dt_prev_s.dropna()
    nongap = intervals[intervals <= 0.06]
    stats["camera"] = {
        "capture_timestamp_rows": len(raw), "recorded_rows": len(c),
        "recorded_fraction": len(c) / len(raw),
        "segment_frame_counts": c.groupby("segment_id").size().to_dict(),
        "recorded_frame_id_mod4_counts": (c.frame_id % 4).value_counts().to_dict(),
        "capture_dropped_since_previous_sum_all_rows": int(raw.dropped_since_previous.sum()),
        "capture_dropped_since_previous_sum_recorded_rows": int(c.dropped_since_previous.sum()),
        "interval_s": summary(intervals), "gap_count": len(gaps), "gaps": gaps,
        "largest_gap": largest, "largest_gap_related_events": related,
        "largest_gap_event_interpretation":
            "Queue overflow events fall inside the capture-time gap; causal attribution is inferred.",
        "effective_fps": len(intervals) / intervals.sum(),
        "effective_fps_excluding_gaps": len(nongap) / nongap.sum(),
        "effective_fps_definition": "number of adjacent intervals / sum of capture intervals",
        "segment_fps_definition": "actual_stored_fps = csv_frames / first-to-last capture elapsed; actual_interval_fps = (csv_frames-1) / elapsed",
        "segment_check": segment_rows,
        "segment_packet_mismatch_count": sum(r["diff"] != 0 for r in segment_rows),
        "pts_assignment_note": "Packet order paired with recorded frame order; mismatches retain all CSV rows and blank unavailable PTS.",
        "timeline_note": "Video PTS is nominal 25 fps; capture gaps are folded. Use t_utc_ns for actual time.",
        "raw_capture_frame_id_missing_count": int(raw.frame_id.max() - raw.frame_id.min() + 1 - len(raw))}
    cols = ["t_utc_ns", "capture_monotonic_ns", "frame_id", "segment_id",
            "segment_file", "frame_in_segment", "recorded_idx", "video_pts_s",
            "dt_prev_s", "flag_gap_before", "dropped_since_previous",
            "skipped_capture_frames", "jpeg_bytes"]
    result = c[cols].copy()
    write_csv(result, OUT / "camera_frames.csv")
    assert len(result) == int((raw.recorded == 1).sum())
    return result


def make_landmarks(raw, source, collector, stats):
    payloads = [json.loads(line) for line in (RAW / "pose/landmarks.jsonl").read_text(
        encoding="utf-8").splitlines() if line.strip()]
    ids = [r["frame_id"] for r in payloads]
    assert len(ids) == len(set(ids)), "Nonidentical duplicate JSONL IDs need explicit handling."
    assert source.frame_id.is_unique
    by_json = {r["frame_id"]: r for r in payloads}
    by_feature = source.set_index("frame_id").to_dict("index")
    all_ids = sorted(set(ids) | set(by_feature))
    assert set(all_ids) <= set(raw.frame_id)
    capture = raw.set_index("frame_id")
    rows, inputs, first_features = [], [], []
    for fid in all_ids:
        cap = capture.loc[fid]
        payload = by_json.get(fid, {})
        original = by_feature.get(fid, {})
        records = payload.get("landmarks", [])
        result_wall = original.get("host_wall_time_ns")
        result_wall = None if result_wall is None or pd.isna(result_wall) else int(result_wall)
        latency = payload.get("latency_ms", original.get("pose_processing_latency_ms"))
        row = {"t_utc_ns": int(cap.host_wall_time_ns), "frame_id": fid,
               "capture_monotonic_ns": int(cap.capture_monotonic_ns),
               "result_host_wall_time_ns": result_wall, "latency_ms": latency,
               "detected": int(bool(records)), "n_landmarks": len(records)}
        objects = [None] * 33 if records else None
        present = {r["i"]: r for r in records}
        assert len(present) == len(records)
        for idx in collector.TRACKED_INDICES:
            rec = present.get(idx)
            for suffix, key in (("x", "x"), ("y", "y"), ("z", "z"), ("vis", "v"), ("pres", "p")):
                row[f"{collector.LANDMARK_NAMES[idx]}_{suffix}"] = rec[key] if rec else None
            if rec:
                objects[idx] = SimpleNamespace(x=rec["x"], y=rec["y"], z=rec["z"],
                                              visibility=rec["v"], presence=rec["p"])
        # The collector adapter only accesses tracked entries; absent points remain absent.
        if objects is not None:
            assert all(objects[i] is not None for i in collector.TRACKED_INDICES)
        inputs.append(objects)
        f = collector.extract_features(objects, frame_id=fid,
            capture_monotonic_ns=int(cap.capture_monotonic_ns),
            host_monotonic_ns=int(original.get("host_monotonic_ns", 0)),
            host_wall_time_ns=result_wall if result_wall is not None else 0, latency_ms=latency)
        first_features.append(f.to_row())
        rows.append(row)
    landmarks = pd.DataFrame({c: pd.array([r[c] for r in rows], dtype="Int64")
                             if c.endswith("_ns") else [r[c] for r in rows] for c in rows[0]})
    for c in landmarks:
        if c.endswith("_ns"):
            landmarks[c] = pd.array([r[c] for r in rows], dtype="Int64")
    assert np.all(np.diff(landmarks.frame_id.to_numpy()) > 0)
    assert np.all(np.diff(landmarks.t_utc_ns.to_numpy(dtype=np.int64)) > 0)
    delta = [int(p["host_wall_time_ns"]) - int(capture.loc[p["frame_id"]].host_wall_time_ns)
             for p in payloads]
    result_delta = [(int(r["result_host_wall_time_ns"]) - int(r["t_utc_ns"])) / 1e6
                    for r in rows if r["result_host_wall_time_ns"] is not None]
    stats["pose_input"] = {
        "features_rows": len(source), "jsonl_lines": len(payloads),
        "union_rows": len(all_ids), "same_row_count": len(source) == len(payloads),
        "same_frame_id_set": set(ids) == set(by_feature),
        "features_only_frame_ids": sorted(set(by_feature) - set(ids)),
        "jsonl_only_frame_ids": sorted(set(ids) - set(by_feature)),
        "result_receive_timestamp_missing_count": sum(r["result_host_wall_time_ns"] is None for r in rows),
        "jsonl_capture_wall_delta_ns": summary(delta),
        "jsonl_host_wall_time_interpretation":
            "capture timestamp" if all(d == 0 for d in delta) else "see timestamp differences",
        "result_receive_minus_capture_ms": summary(result_delta),
        "inference_latency_ms": summary(landmarks.latency_ms),
        "detected_count": int(landmarks.detected.sum()),
        "landmark_count_distribution": landmarks.n_landmarks.value_counts().to_dict(),
        "source_neutral_valid_distribution": source.neutral_valid.value_counts().to_dict(),
        "source_shift_nonempty_counts": {c: int(source[c].notna().sum()) for c in source
                                       if "shift" in c or c.startswith("lean_")},
        "missing_jsonl_note": "Collector writes JSONL only when landmarks exist; feature-only frames are preserved as nondetections."}
    write_csv(landmarks, OUT / "pose_landmarks.csv")
    check = read_csv(OUT / "pose_landmarks.csv")
    for c in landmarks:
        if any(c.endswith("_" + s) for s in ("x", "y", "z", "vis", "pres")):
            assert np.array_equal(check[c].to_numpy(), landmarks[c].to_numpy(), equal_nan=True), c
    return landmarks, inputs, pd.DataFrame(first_features)


def compare_features(base, source, inputs, collector, stats):
    columns = GEOMETRY + ["pose_confidence", "landmark_valid_mask",
                          "landmarks_visible", "vision_valid"]
    a = base.set_index("frame_id")
    b = source.set_index("frame_id")
    ids = a.index.intersection(b.index)
    report = {}
    unexplained = []
    for col in columns:
        examples = []
        if col == "landmark_valid_mask":
            unequal = a.loc[ids, col].fillna("") != b.loc[ids, col].fillna("")
            bad_ids = ids[unequal]
            maxdiff = None
        else:
            aa, bb = a.loc[ids, col], b.loc[ids, col]
            diff = (aa - bb).abs()
            missing = aa.isna() != bb.isna()
            bad_ids = ids[(diff > 1e-5 + 1e-12) | missing]
            maxdiff = float(diff.max()) if diff.notna().any() else None
        for fid in bad_ids:
            row = a.loc[fid]
            lm = inputs[a.index.get_loc(fid)]
            boundary = bool(lm and any(abs(lm[i].visibility - 0.5) <= 0.00005001
                                       for i in collector.TRACKED_INDICES))
            bound = 1e-5 + 1e-12
            reason = "coordinate quantization (5 decimal places) and feature rounding"
            if col == "pose_confidence":
                bound = 0.00015 + 1e-12
                reason = "visibility quantization (4 decimal places), then mean rounded to 4 decimals"
            elif col == "shoulder_width":
                bound = math.sqrt(2) * 1e-5 + 1e-5 + 1e-12
                reason = "two-point distance of quantized coordinates, then 5-decimal rounding"
            elif col == "torso_angle_deg":
                dx = row.shoulder_center_x - row.hip_center_x
                dy = row.hip_center_y - row.shoulder_center_y
                radius = math.hypot(dx, dy)
                # Centre output rounding adds a second 5e-6 uncertainty per coordinate.
                bound = math.degrees(math.asin(min(1.0, math.sqrt(2) * 2e-5 /
                              max(radius - math.sqrt(2) * 1e-5, 1e-12)))) + 0.0001
                reason = "atan2 magnifies 5-decimal coordinate quantization; bounded angular perturbation"
            av, bv = a.loc[fid, col], b.loc[fid, col]
            explained = boundary or (
                col != "landmark_valid_mask" and pd.notna(av) and pd.notna(bv)
                and abs(float(av) - float(bv)) <= bound)
            if boundary:
                reason = "visibility rounded across 0.5 threshold; original full precision is unavailable"
            if not explained:
                unexplained.append({"frame_id": int(fid), "column": col,
                                    "recomputed": av, "source": bv})
            if len(examples) < 5:
                examples.append({"frame_id": int(fid), "recomputed": av, "source": bv,
                                 "quantization_bound": bound, "reason": reason,
                                 "explained": explained})
        report[col] = {"max_abs_diff": maxdiff, "over_1e_minus_5_or_missing_count": len(bad_ids),
                       "examples": examples}
    stats["feature_comparison"] = {"columns": report,
        "unexplained_count": len(unexplained), "unexplained_examples": unexplained[:20],
        "note": "Tolerance 1e-5 is reported literally; angle/confidence excesses require quantization explanations."}


def estimate_neutral(base, source, collector, stats, gps):
    valid = base.loc[base.vision_valid == 1]
    driving = valid.loc[valid.speed_kmh_nearest >= 20]
    assert len(driving), "No vision-valid driving frames (nearest GPS >=20 km/h within 1.5 s)."
    typical = driving[GEOMETRY].median()
    assert typical.notna().all() and typical.shoulder_width > 0, "Incomplete driving typical posture."
    t = base.capture_monotonic_ns.to_numpy(dtype=np.int64)
    capture_wall = stats["_capture_wall"]
    def wall_at(mono):
        pos = min(np.searchsorted(capture_wall[0], mono), len(capture_wall[0]) - 1)
        return int(capture_wall[1][pos]) + int(mono) - int(capture_wall[0][pos])
    windows = []
    for start in range(int(t[0]), int(t[-1]) - 10_000_000_000 + 1, 1_000_000_000):
        end = start + 10_000_000_000
        start_wall, end_wall = wall_at(start), wall_at(end)
        part = valid.loc[(valid.capture_monotonic_ns >= start) &
                         (valid.capture_monotonic_ns < end)]
        speeds = gps.loc[(gps.t_utc_ns >= start_wall) & (gps.t_utc_ns < end_wall)
                         & (gps.valid == 1), "speed_kmh"].dropna()
        speed_min, speed_max, speed_mean = speeds.min(), speeds.max(), speeds.mean()
        steady = len(speeds) >= 8 and speed_min >= 20 and speed_max - speed_min <= 5
        mean = part[GEOMETRY].mean()
        scatter = max(part.shoulder_center_x.std(ddof=1) * 640,
                      part.shoulder_center_y.std(ddof=1) * 400) if len(part) > 1 else math.inf
        distance2 = float(np.sum(((mean[CENTERS] - typical[CENTERS]) * SCALES) ** 2))
        width_penalty = WIDTH_PERCENT_WEIGHT_PX2 * ((mean.shoulder_width - typical.shoulder_width)
                                                 / typical.shoulder_width * 100) ** 2
        torso_penalty = (mean.torso_angle_deg - typical.torso_angle_deg) ** 2
        stable = bool(len(part) >= 90 and scatter <= 12 and mean[GEOMETRY].notna().all())
        windows.append({"start_capture_monotonic_ns": start, "end_capture_monotonic_ns": end,
                        "start_t_utc_ns": start_wall, "end_t_utc_ns": end_wall,
                        "valid_frames": len(part), "scatter_px": scatter,
                        "distance_squared_px2": distance2, "width_penalty_px2": width_penalty,
                        "torso_penalty": torso_penalty, "score": distance2 + width_penalty + torso_penalty,
                        "valid_gps_epochs": len(speeds), "speed_min_kmh": speed_min,
                        "speed_max_kmh": speed_max, "speed_mean_kmh": speed_mean,
                        "candidate_any_speed": stable, "candidate": bool(stable and steady),
                        **mean.to_dict()})
    w = pd.DataFrame(windows)
    any_speed = w.loc[w.candidate_any_speed].copy()
    steady_candidates = w.loc[w.candidate].copy()
    candidates = steady_candidates if len(steady_candidates) else any_speed
    selected = None
    if len(candidates):
        candidates = candidates.sort_values(["score", "start_capture_monotonic_ns"], kind="stable")
        selected = candidates.iloc[0]
        values = selected[GEOMETRY]
        used = int(selected.valid_frames)
        established = int(selected.end_capture_monotonic_ns)
        method = "stable_window_steady_speed" if len(steady_candidates) else "stable_window_any_speed"
    else:
        values = typical
        used = len(driving)
        established = int(t[-1])
        method = "session_median_fallback"
    def center(p):
        return (float(values[p + "_center_x"]), float(values[p + "_center_y"]), 0.0)
    neutral = collector.NeutralPosture(
        shoulder_center=center("shoulder"), hip_center=center("hip"),
        head_center=center("head") if pd.notna(values.head_center_x) else None,
        shoulder_width=float(values.shoulder_width), torso_angle_deg=float(values.torso_angle_deg),
        frames_used=used, established_monotonic_ns=established, valid=True, reason=method)
    def window_info(row):
        start, end = int(row.start_capture_monotonic_ns), int(row.end_capture_monotonic_ns)
        frames = base.loc[(base.capture_monotonic_ns >= start) & (base.capture_monotonic_ns < end)]
        return {"start_capture_monotonic_ns": start, "end_capture_monotonic_ns": end,
            "start_t_utc_ns": int(row.start_t_utc_ns), "end_t_utc_ns": int(row.end_t_utc_ns),
            "first_frame_t_utc_ns": int(frames.t_utc_ns.iloc[0]),
            "last_frame_t_utc_ns": int(frames.t_utc_ns.iloc[-1]),
            "start_kst": kst(row.start_t_utc_ns), "end_kst": kst(row.end_t_utc_ns),
            "valid_frames": int(row.valid_frames), "scatter_px": float(row.scatter_px),
            "distance_to_typical_px": math.sqrt(row.distance_squared_px2),
            "distance_squared_px2": float(row.distance_squared_px2), "score": float(row.score),
            "width_penalty_px2": float(row.width_penalty_px2), "torso_penalty": float(row.torso_penalty),
            "valid_gps_epochs": int(row.valid_gps_epochs), "speed_min_kmh": float(row.speed_min_kmh),
            "speed_max_kmh": float(row.speed_max_kmh), "speed_mean_kmh": float(row.speed_mean_kmh),
            "shoulder_width": float(row.shoulder_width), "neutral": row[GEOMETRY].to_dict()}
    selected_info = window_info(selected) if selected is not None else None
    if method == "stable_window_steady_speed":
        assert selected.valid_gps_epochs >= 8 and selected.speed_min_kmh >= 20
        assert selected.speed_max_kmh - selected.speed_min_kmh <= 5
    top = [window_info(row) for _, row in candidates.head(5).iterrows()]
    runner = None
    if len(candidates) >= 2:
        second = candidates.iloc[1]
        dx = (second.shoulder_center_x - values.shoulder_center_x) * 640
        dy = (second.shoulder_center_y - values.shoulder_center_y) * 400
        runner = {"selected_width_over_runner_up_width": float(values.shoulder_width / second.shoulder_width),
            "lean_scale_note": "Multiplicative scale for an unchanged shoulder shift; changing neutral also changes the offset.",
            "shoulder_center_delta_px": {"x": dx, "y": dy, "distance": math.hypot(dx, dy)}}
    def dispersion(frame):
        result = {}
        for c in GEOMETRY:
            scale = 640 if c.endswith("_x") else 400 if c.endswith("_y") else 1
            result[c] = {"sample_std_native": frame[c].std(ddof=1),
                "min_native": frame[c].min(), "max_native": frame[c].max(),
                "sample_std_px": frame[c].std(ddof=1) * scale if c in CENTERS else None,
                "min_px": frame[c].min() * scale if c in CENTERS else None,
                "max_px": frame[c].max() * scale if c in CENTERS else None}
        return result
    dispersions = dispersion(candidates)
    stopped = valid.loc[valid.speed_kmh_nearest < 1]
    moving = valid.loc[valid.speed_kmh_nearest >= 5]
    posture_difference = {"stopped_frames": len(stopped), "moving_frames": len(moving),
        "direction": "moving median minus stopped median",
        "center_difference_px": {c: (moving[c].median() - stopped[c].median()) * scale
                                  for c, scale in zip(CENTERS, SCALES)},
        "torso_difference_deg": moving.torso_angle_deg.median() - stopped.torso_angle_deg.median()}
    minute_rows = []
    elapsed_minute = (valid.capture_monotonic_ns.astype("int64") - int(t[0])) // 60_000_000_000
    for minute, frames in valid.groupby(elapsed_minute, sort=True):
        delta = {c: (frames[c].median() - values[c]) * scale for c, scale in zip(CENTERS, SCALES)}
        minute_rows.append({"minute_from_pose_start": int(minute),
            "start_kst": kst(wall_at(int(t[0]) + int(minute) * 60_000_000_000)),
            "valid_frames": len(frames), "median_minus_neutral_px": delta,
            "all_center_components_within_6px": all(abs(v) <= 6 for v in delta.values())})
    settling = next((r["minute_from_pose_start"] for i, r in enumerate(minute_rows)
                     if all(x["all_center_components_within_6px"] for x in minute_rows[i:])), None)
    minute_info = {"bin_definition": "60-second capture intervals anchored at first pose; all vision-valid frames, final partial bin included",
        "tolerance_px": 6, "within_definition": "all six x/y centre components; sustained through final observed bin",
        "sustained_within_6px_from_minute": settling, "rows": minute_rows}
    differences = {c: float(values[c] - typical[c]) * scale
                   for c, scale in zip(CENTERS, SCALES)}
    original_capture = base.loc[(base.capture_monotonic_ns >= t[0] + 5_000_000_000) &
        (base.capture_monotonic_ns <= t[0] + 15_000_000_000) & (base.vision_valid == 1)]
    capture_scatter = max(original_capture.shoulder_center_x.std(ddof=1) * 640,
                          original_capture.shoulder_center_y.std(ddof=1) * 400)
    calibrator = collector.NeutralCalibrator()
    for row in source.to_dict("records"):
        features = collector.PoseFeatures(**{k: None if pd.isna(v) else v for k, v in row.items()})
        calibrator.feed(features, int(row["host_monotonic_ns"]))
    original_receive = source.loc[(source.host_monotonic_ns >= int(source.host_monotonic_ns.iloc[0]) + 5_000_000_000)
        & (source.host_monotonic_ns <= int(source.host_monotonic_ns.iloc[0]) + 15_000_000_000)
        & (source.vision_valid == 1)]
    receive_scatter = max(original_receive.shoulder_center_x.std(ddof=1) * 640,
                          original_receive.shoulder_center_y.std(ddof=1) * 400)
    info = {"method": method, "rules": {
        "window_s": 10, "step_s": 1, "window_anchor": "first union pose capture time",
        "interval": "[start, end); full 10-second windows only",
        "min_vision_valid_frames": 90, "max_shoulder_sample_std_px": 12,
        "scatter_definition": "max(std(x, ddof=1)*640, std(y, ddof=1)*400)",
        "typical": "component-wise median of vision-valid frames with nearest GPS speed >=20 km/h within 1.5 s",
        "selection": "minimum sum of squared shoulder/hip/head centre pixel differences + width_percent_weight_px2*((width_win-width_typ)/width_typ*100)^2 + torso angle difference in degrees squared; earliest tie",
        "width_percent_weight_px2": WIDTH_PERCENT_WEIGHT_PX2,
        "steady_speed": "valid GPS epochs in the UTC [start,end) window >=8, minimum speed >=20 km/h, maximum-minus-minimum <=5 km/h",
        "min_valid_gps_epochs": 8, "min_speed_kmh": 20, "max_speed_range_kmh": 5,
        "gps_match": "nearest epoch by t_utc_ns; earlier on a tie; blank if missing speed or distance >1.5 s",
        "neutral": "mean of vision-valid rounded features, same as collector _finalise",
        "fallback": "no steady candidates: stable_window_any_speed with flag_neutral_fallback=1; no stable candidates: driving component-wise medians as session_median_fallback with flag=1", "input_width": 640, "input_height": 400},
        "neutral": neutral.to_dict(), "total_windows": len(w), "candidate_windows": len(candidates),
        "selected_window": selected_info, "session_typical": typical.to_dict(),
        "candidate_windows_before_steady_speed": len(any_speed),
        "candidate_windows_after_steady_speed": len(steady_candidates),
        "driving_typical_frames": len(driving), "top_5_candidates": top,
        "runner_up_sensitivity": runner,
        "candidate_mean_dispersion": dispersions,
        "candidate_mean_dispersion_any_speed": dispersion(any_speed),
        "stopped_vs_moving": posture_difference, "minute_drift": minute_info,
        "selected_minus_session_median_px": differences,
        "original_calibration": {
            "capture_5_to_15_s_valid_frames": len(original_capture),
            "capture_5_to_15_s_scatter_px": capture_scatter,
            "receive_5_to_15_s_valid_frames": len(original_receive),
            "receive_5_to_15_s_scatter_px": receive_scatter,
            "collector_replay": calibrator.result.to_dict(),
            "matches_63_2_at_one_decimal": round(receive_scatter, 1) == 63.2,
            "clock_note": "Collector feeds calibration with result receive monotonic time, not capture time."},
        "note": "Camera views the driver from the right/passenger side. Image x may approximate vehicle fore/aft, not vehicle lateral displacement. Formulas remain identical to the collector; offline neutral is a typical stable posture, not independently verified anatomical neutral."}
    info["rule"] = info["rules"]
    width_min, width_max = candidates.shoulder_width.min(), candidates.shoulder_width.max()
    early_drift = max((abs(v) for r in minute_rows if r["minute_from_pose_start"] < 6
                       for v in r["median_minus_neutral_px"].values()), default=math.nan)
    info["note"] += (f" A single neutral cannot separate posture settling from motion: first six minutes have centre-component median shifts up to {early_drift:.2f} px. "
        f"Sustained all-component +/-6 px starts at elapsed minute {settling} (None means not observed). "
        f"Shoulder width is sensitive to torso rotation in a side view; candidate widths span {width_min:.5f} to {width_max:.5f}, "
        f"implying up to {width_max / width_min:.3f}x lean-score scale uncertainty for an unchanged shift.")
    write_json(info, OUT / "pose_neutral.json")
    stats["neutral"] = info
    return neutral, w, selected_info, method


def pose_table(landmarks, inputs, base, camera_df, neutral, method, collector, stats):
    records = []
    previous = None
    for row, objects in zip(base.to_dict("records"), inputs):
        f = collector.extract_features(objects, frame_id=int(row["frame_id"]),
            capture_monotonic_ns=int(row["capture_monotonic_ns"]),
            host_monotonic_ns=int(row["host_monotonic_ns"]),
            host_wall_time_ns=int(row["host_wall_time_ns"]),
            neutral=neutral, previous=previous, image_width=640, image_height=400,
            latency_ms=row["pose_processing_latency_ms"])
        records.append(f.to_row())
        previous = f
    p = pd.DataFrame(records)
    p["t_utc_ns"] = landmarks.t_utc_ns
    p["result_host_wall_time_ns"] = landmarks.result_host_wall_time_ns
    p["recorded"] = p.frame_id.isin(camera_df.frame_id).astype(int)
    recorded_t = camera_df.capture_monotonic_ns.to_numpy(dtype=np.int64)
    pose_t = landmarks.capture_monotonic_ns.to_numpy(dtype=np.int64)
    right = np.searchsorted(recorded_t, pose_t)
    right = np.clip(right, 0, len(recorded_t) - 1)
    left = np.maximum(right - 1, 0)
    near = np.where(np.abs(recorded_t[left] - pose_t) <= np.abs(recorded_t[right] - pose_t), left, right)
    nearest = camera_df.iloc[near].reset_index(drop=True)
    p["nearest_recorded_frame_id"] = nearest.frame_id
    p["nearest_segment_file"] = nearest.segment_file
    p["nearest_frame_in_segment"] = nearest.frame_in_segment
    p["nearest_recorded_dt_ms"] = (recorded_t[near] - pose_t) / 1e6
    p["torso_angle_delta_deg"] = (p.torso_angle_deg - neutral.torso_angle_deg).round(4)
    p["dt_prev_s"] = landmarks.capture_monotonic_ns.diff().astype(float) / 1e9
    p["flag_gap_before"] = (p.dt_prev_s > 0.15).astype(int)
    p["flag_low_visibility"] = [
        int(any(getattr(objects[i], "visibility", 0) < 0.5 for i in collector._CORE_INDICES))
        if objects else 1 for objects in inputs]
    p["flag_neutral_fallback"] = int(method != "stable_window_steady_speed")
    p["shift_valid"] = ((p.neutral_valid == 1) & p.shoulder_shift_x.notna()).astype(int)
    p["speed_kmh_nearest"] = base.speed_kmh_nearest
    front = ["t_utc_ns", "frame_id", "capture_monotonic_ns", "result_host_wall_time_ns",
             "recorded", "nearest_recorded_frame_id", "nearest_segment_file",
             "nearest_frame_in_segment", "nearest_recorded_dt_ms"]
    feature_cols = GEOMETRY + ["shoulder_shift_x", "shoulder_shift_y", "hip_shift_x",
        "hip_shift_y", "head_shift_x", "head_shift_y", "shoulder_shift_px", "head_shift_px",
        "lean_score", "lean_velocity", "pose_confidence", "landmark_valid_mask", "landmarks_visible"]
    p = p[front + feature_cols + ["torso_angle_delta_deg", "pose_processing_latency_ms",
        "neutral_valid", "vision_valid", "shift_valid", "dt_prev_s", "flag_gap_before",
        "flag_low_visibility", "flag_neutral_fallback", "speed_kmh_nearest"]]
    for c in p:
        if c.endswith("_ns"):
            p[c] = pd.array(landmarks[c], dtype="Int64")
    write_csv(p, OUT / "pose.csv")
    assert len(p) == len(landmarks)
    assert np.all(np.diff(p.t_utc_ns.to_numpy(dtype=np.int64)) > 0)
    selected = stats["neutral"]["selected_window"]
    means = None
    if selected:
        within = p.loc[(p.capture_monotonic_ns >= selected["start_capture_monotonic_ns"]) &
                       (p.capture_monotonic_ns < selected["end_capture_monotonic_ns"]) &
                       (p.vision_valid == 1)]
        means = within[["shoulder_shift_x", "shoulder_shift_y", "lean_score"]].mean().to_dict()
        assert all(abs(v) < 1e-4 for v in means.values()), means
    stats["pose"] = {
        "rows": len(p), "vision_valid_count": int(p.vision_valid.sum()),
        "vision_valid_ratio": float(p.vision_valid.mean()),
        "recorded_frame_count": int(p.recorded.sum()), "recorded_frame_ratio": float(p.recorded.mean()),
        "interval_s": summary(p.dt_prev_s), "gap_count": int(p.flag_gap_before.sum()),
        "low_visibility_count": int(p.flag_low_visibility.sum()),
        "nearest_recorded_dt_ms": summary(p.nearest_recorded_dt_ms),
        "selected_window_shift_means": means,
        "shift_valid_count": int(p.shift_valid.sum()),
        "neutral_valid_on_nondetected_frames": int(p.loc[landmarks.detected == 0, "neutral_valid"].sum()),
        "neutral_valid_note": "Collector meaning: a neutral reference exists, even on nondetected frames (neutral_valid=1). shift_valid additionally requires shoulder_shift_x to exist.",
        "speed_kmh_nearest_missing_count": int(p.speed_kmh_nearest.isna().sum())}
    return p


def plots(c, p, windows, selected, neutral):
    def finish(fig, name, time_axes=()):
        for ax in time_axes:
            ax.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M:%S", tz=KST))
            ax.set_xlabel("Capture time (KST)")
        fig.tight_layout()
        fig.savefig(QC / name, dpi=130)
        plt.close(fig)
    ct, pt = dates(c.t_utc_ns), dates(p.t_utc_ns)
    fig, ax = plt.subplots(figsize=(13, 4))
    ax.plot(ct, c.dt_prev_s, linewidth=0.5)
    gaps = c.flag_gap_before == 1
    ax.scatter(ct[gaps], c.loc[gaps, "dt_prev_s"], color="red", s=15, label="Gap > 0.06 s")
    for stamp in ct[c.segment_id.diff().fillna(0) != 0]:
        ax.axvline(stamp, color="gray", alpha=0.25, linewidth=0.7)
    ax.set_yscale("log"); ax.set_ylabel("Recorded interval (s)")
    ax.set_title("Camera capture intervals and segment boundaries"); ax.legend()
    finish(fig, "camera_frame_intervals.png", [ax])
    fig, axes = plt.subplots(4, 1, figsize=(13, 10), sharex=True)
    for ax, col in zip(axes, ["lean_score", "shoulder_shift_px", "head_shift_px", "torso_angle_deg"]):
        ax.plot(pt, p[col], linewidth=0.6); ax.set_ylabel(col)
        if selected:
            ax.axvspan(pd.Timestamp(selected["start_t_utc_ns"], unit="ns", tz="UTC"),
                       pd.Timestamp(selected["end_t_utc_ns"], unit="ns", tz="UTC"),
                       color="green", alpha=0.2)
    axes[0].set_title("Pose features and offline neutral window")
    finish(fig, "pose_features_time.png", [axes[-1]])
    fig, ax = plt.subplots(figsize=(13, 4))
    wt = dates(windows.start_t_utc_ns)
    ax.plot(wt, windows.scatter_px, linewidth=0.6, label="All full windows")
    candidate = windows.candidate
    ax.scatter(wt[candidate], windows.loc[candidate, "scatter_px"], s=8,
               color="green", label="Candidates (>=90 valid, <=12 px, steady speed)")
    if selected:
        idx = windows.index[windows.start_capture_monotonic_ns ==
                            selected["start_capture_monotonic_ns"]][0]
        ax.scatter([wt[idx]], [windows.loc[idx, "scatter_px"]], s=100, marker="*",
                   color="red", label="Selected")
    ax.axhline(12, color="orange", linestyle="--", label="12 px")
    speed_axis = ax.twinx()
    speed_axis.plot(wt, windows.speed_mean_kmh, color="purple", alpha=0.4, linewidth=0.7,
                    label="Window mean speed (valid GPS)")
    speed_axis.fill_between(wt, windows.speed_min_kmh, windows.speed_max_kmh, color="purple", alpha=0.08)
    speed_axis.set_ylabel("Window speed min / mean / max (km/h)")
    ax.set_ylabel("Shoulder scatter (px)"); ax.set_title("10-second neutral windows; steady: >=8 GPS, min>=20 km/h, range<=5 km/h")
    lines, labels = ax.get_legend_handles_labels()
    speed_lines, speed_labels = speed_axis.get_legend_handles_labels()
    ax.legend(lines + speed_lines, labels + speed_labels, fontsize=8)
    finish(fig, "pose_neutral_windows.png", [ax])
    fig, ax = plt.subplots(figsize=(9, 6))
    clipped = 0
    for name, color in zip(("shoulder", "hip", "head"), ("blue", "green", "orange")):
        y = p[name + "_center_y"] * 400
        clipped += int(((y < 0) | (y > 400)).sum())
        ax.scatter(p[name + "_center_x"] * 640, y,
                   s=2, alpha=0.12, color=color, label=name)
        center = getattr(neutral, name + "_center")
        if center:
            ax.scatter([center[0] * 640], [center[1] * 400], s=300, marker="X", color=color, edgecolors="black", linewidths=1.5, zorder=100)
    ax.set_xlim(0, 640); ax.set_ylim(400, 0); ax.set_aspect("equal")
    ax.set_xlabel("Image x (px; 640-wide input)"); ax.set_ylabel("Image y (px; downward)")
    ax.set_title(f"Pose centres and offline neutral (X); y-clipped points: {clipped}"); ax.legend()
    finish(fig, "pose_landmark_scatter.png")
    fig, axes = plt.subplots(3, 1, figsize=(13, 8), sharex=True)
    for ax, col in zip(axes, ("vision_valid", "landmarks_visible", "pose_processing_latency_ms")):
        ax.plot(pt, p[col], linewidth=0.5); ax.set_ylabel(col)
    axes[0].set_title("Pose validity, visible landmarks and inference latency")
    finish(fig, "pose_detection.png", [axes[-1]])


def main():
    gps_path = OUT / "gps.csv"
    if not gps_path.is_file():
        raise FileNotFoundError(f"Neutral selection requires {gps_path}; run gps_imu first.")
    gps = read_csv(gps_path).sort_values("t_utc_ns", kind="stable").reset_index(drop=True)
    assert len(gps) and gps.t_utc_ns.notna().all() and gps.t_utc_ns.is_unique
    OUT.mkdir(parents=True, exist_ok=True)
    QC.mkdir(parents=True, exist_ok=True)
    raw = read_csv(RAW / "camera/frame_timestamps.csv")
    source = read_csv(RAW / "pose/features.csv")
    events = [json.loads(l) for l in (RAW / "events.jsonl").read_text(encoding="utf-8").splitlines() if l.strip()]
    session = json.loads((RAW / "session_summary.json").read_text(encoding="utf-8"))
    collector = load_collector()
    stats = {"session_id": session["session_id"], "session_summary_workers": session["workers"],
             "collector_config": {"input_width": 640, "input_height": 400,
                "neutral_window_s": 10, "collector_min_valid_frames": 30,
                "collector_max_stddev_px": 12, "min_visibility": collector._MIN_VISIBILITY}}
    offsets = {}
    for name, df, mono in (
        ("camera_capture", raw, "capture_monotonic_ns"),
        ("pose_results", source, "host_monotonic_ns"),
        ("gps", read_csv(RAW / "gps/gps.csv"), "host_monotonic_ns"),
        ("imu", read_csv(RAW / "imu/imu.csv"), "host_monotonic_ns")):
        vals = [int(w) - int(m) for w, m in zip(df.host_wall_time_ns, df[mono])]
        center = sorted(vals)[len(vals) // 2]
        offsets[name] = {"median_offset_ns": center,
                         "residual_to_median_ns": summary([v - center for v in vals])}
    stats["clock_offsets"] = offsets
    stats["camera_capture_wall_clock_note"] = "Compare integer wall-minus-monotonic offsets across sensors; exact pose JSONL linkage separately confirms capture wall timestamps."
    stats["_capture_wall"] = (raw.capture_monotonic_ns.to_numpy(dtype=np.int64),
                               raw.host_wall_time_ns.to_numpy(dtype=np.int64))
    print("Probing all video segments...", flush=True)
    c = camera(raw, events, stats)
    landmarks, inputs, base = make_landmarks(raw, source, collector, stats)
    compare_features(base, source, inputs, collector, stats)
    base["t_utc_ns"] = landmarks.t_utc_ns
    gps_t = gps.t_utc_ns.to_numpy(dtype=np.int64)
    pose_t = base.t_utc_ns.to_numpy(dtype=np.int64)
    right = np.clip(np.searchsorted(gps_t, pose_t), 0, len(gps_t) - 1)
    left = np.maximum(right - 1, 0)
    near = np.where(np.abs(gps_t[left] - pose_t) <= np.abs(gps_t[right] - pose_t), left, right)
    nearest = gps.iloc[near].reset_index(drop=True)
    base["speed_kmh_nearest"] = nearest.speed_kmh.where(
        np.abs(gps_t[near] - pose_t) <= 1_500_000_000)
    neutral, windows, selected, method = estimate_neutral(base, source, collector, stats, gps)
    p = pose_table(landmarks, inputs, base, c, neutral, method, collector, stats)
    plots(c, p, windows, selected, neutral)
    del stats["_capture_wall"]
    stats["acceptance"] = {"camera_rows_equal_recorded": True,
        "pose_rows_equal_union": len(p) == len(set(source.frame_id) | set(landmarks.frame_id)),
        "pose_frame_ids_all_in_capture": set(p.frame_id) <= set(raw.frame_id),
        "strict_time_and_frame_order": True, "ns_csv_integer_strings": True,
        "landmarks_roundtrip_exact": True,
        "neutral_shift_means_checked": selected is not None,
        "selected_speed_conditions_checked": method == "stable_window_steady_speed",
        "csv_no_exponent_notation": True,
        "feature_comparison_unexplained": stats["feature_comparison"]["unexplained_count"]}
    write_json(stats, QC / "stats_pose_camera.json")
    assert stats["acceptance"]["pose_rows_equal_union"]
    assert stats["acceptance"]["pose_frame_ids_all_in_capture"]
    assert stats["feature_comparison"]["unexplained_count"] == 0, "See stats: unexplained feature deviations."
    print(json.dumps(clean({"camera_rows": len(c), "pose_rows": len(p),
        "jsonl_lines": stats["pose_input"]["jsonl_lines"],
        "neutral_candidates": stats["neutral"]["candidate_windows"],
        "selected": selected, "acceptance": stats["acceptance"]}), ensure_ascii=False, indent=2))


def output_hashes():
    paths = [OUT / name for name in ("camera_frames.csv", "pose_landmarks.csv", "pose.csv", "pose_neutral.json")]
    paths += [QC / name for name in ("stats_pose_camera.json", "camera_gaps.csv", "camera_frame_intervals.png",
        "pose_features_time.png", "pose_neutral_windows.png", "pose_landmark_scatter.png", "pose_detection.png")]
    return {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}


if __name__ == "__main__":
    main()
    first = output_hashes()
    main()
    second = output_hashes()
    assert first == second, {p: (first[p], second[p]) for p in first if first[p] != second[p]}
    print(json.dumps({"two_runs_byte_identical": True, "sha256": second}, indent=2))
