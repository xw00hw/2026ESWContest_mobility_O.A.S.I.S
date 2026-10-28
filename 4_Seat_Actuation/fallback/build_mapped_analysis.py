"""Offline signed-pressure analysis of the user's confirmed A1 mapping.

Reads only the pressure/motor profile and the previously derived pressure-point
CSV. No device, network, camera, image or pose-estimation inputs are supported.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
import statistics


SID_FIRST = "S06"
SID_EARLIER_LEFT = "S07"
SID_LAST = "S08"
SIDE_CHANNELS = {"left": ("A1_C2", "A1_C3"), "right": ("A1_C0", "A1_C1")}
PHYSICAL_MAPPING = {"A1_C0": "DR", "A1_C1": "UR", "A1_C2": "DL", "A1_C3": "UL"}


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def mean_sums(channels):
    return {side: sum(channels[c]["mean_v"] for c in ids) for side, ids in SIDE_CHANNELS.items()}


def describe(values):
    return {"count": len(values), "minimum": min(values), "median": statistics.median(values),
            "maximum": max(values)}


def build(profile_path, point_table_path):
    profile_path, point_table_path = Path(profile_path), Path(point_table_path)
    p = json.loads(profile_path.read_text(encoding="utf-8"))
    if p["schema"] != "oasis-bench-pressure-profile-v1" or p["session_id"] != SID_LAST:
        raise ValueError("This review is specific to the identified bench profile")
    base = p["baseline"]["channels"]
    base_sums = mean_sums(base)
    branches = {}
    for actuator_side in ("left", "right"):
        points = p["branches"][actuator_side]["points"]
        first_ns = points[0]["interval"]["start_host_monotonic_ns"]
        rows = []
        for point in points:
            sums = mean_sums(point["channels"])
            rows.append({
                "yaw_deg": point["yaw_deg"], "point_index": point["point_index"],
                "command_deg": point["command_deg"], "interval": point["interval"],
                "seconds_since_branch_neutral": (point["interval"]["start_host_monotonic_ns"]-first_ns)/1e9,
                "quality_ok": point["quality_ok"], "commanded_motion_check_passed": point["motion_check"]["passed"],
                "side_sum_of_means_v": sums,
                "side_sum_delta_from_session_neutral_v": {s: sums[s]-base_sums[s] for s in SIDE_CHANNELS},
                "a1_channel_mean_delta_v": {c: point["channels"][c]["mean_v"]-base[c]["mean_v"]
                                             for c in PHYSICAL_MAPPING},
                "a1_channel_statistics": {c: {key: point["channels"][c][key]
                    for key in ("samples", "mean_v", "std_v", "p95_v")} for c in PHYSICAL_MAPPING},
            })
        summary = {}
        for sensor_side in SIDE_CHANNELS:
            values = [row["side_sum_delta_from_session_neutral_v"][sensor_side] for row in rows[1:]]
            summary[sensor_side] = dict(describe(values), positive_points=sum(x > 0 for x in values),
                                       negative_points=sum(x < 0 for x in values), zero_points=sum(x == 0 for x in values))
        branches[actuator_side] = {"points": rows, "nonzero_yaw_signed_response_summary": summary}

    earlier = p["qualitative_earlier_left_only"]["points"]
    earlier_base = earlier[0]["earlier_mean_v"]
    earlier_rows = []
    for point in earlier:
        later = branches["left"]["points"][point["left_yaw_deg"]]
        earlier_sums = {s: sum(point["earlier_mean_v"][c]-earlier_base[c] for c in cs)
                        for s, cs in SIDE_CHANNELS.items()}
        earlier_rows.append({"left_yaw_deg": point["left_yaw_deg"], "other_angles_deg": 0,
            "earlier_side_sum_delta_from_own_neutral_v": earlier_sums,
            "later_side_sum_delta_from_own_neutral_v": later["side_sum_delta_from_session_neutral_v"]})

    # This local derived table contains only pressure statistics and commanded
    # angles. Do not substitute an experiment-wide merged/video/pose table.
    with point_table_path.open(encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        required = {"session_id", "point_index", "left_yaw_deg", "left_pitch_deg", "right_yaw_deg", "right_pitch_deg"}
        expected = required | {f"A{b}_C{c}_{suffix}" for b in range(3) for c in range(4)
                               for suffix in ("mean_v", "std_v", "samples")}
        if set(reader.fieldnames or []) != expected:
            raise ValueError("Only the pressure-only command-point CSV schema is accepted")
        table = {}
        for row in reader:
            angles = tuple(float(row[k]) for k in ("left_yaw_deg", "left_pitch_deg", "right_yaw_deg", "right_pitch_deg"))
            key = (row["session_id"], angles)
            if key in table:
                raise ValueError("Repeated full command requires a separate trial model")
            table[key] = row
    diagonal = []
    for yaw in range(21):
        angles = (yaw, 0, yaw, 0)
        first, last = table[(SID_FIRST, angles)], table[(SID_LAST, angles)]
        item = {"left_yaw_deg": yaw, "right_yaw_deg": yaw, "both_pitch_deg": 0,
                "first_point_index": int(first["point_index"]), "last_point_index": int(last["point_index"])}
        for label, row, sid in (("first", first, SID_FIRST), ("last", last, SID_LAST)):
            neutral = table[(sid, (0, 0, 0, 0))]
            item[label+"_a1_mean_v"] = {c: float(row[c+"_mean_v"]) for c in PHYSICAL_MAPPING}
            item[label+"_side_sum_delta_from_own_neutral_v"] = {
                s: sum(float(row[c+"_mean_v"])-float(neutral[c+"_mean_v"]) for c in cs)
                for s, cs in SIDE_CHANNELS.items()}
        item["last_minus_first_a1_mean_v"] = {c: float(last[c+"_mean_v"])-float(first[c+"_mean_v"])
                                               for c in PHYSICAL_MAPPING}
        diagonal.append(item)
    return {
        "schema": "oasis-mapped-pressure-evidence-v1", "dataset_id": p["dataset_id"],
        "mapping_provenance": {"source": "Team-confirmed sensor placement",
            "a1_physical_mapping": PHYSICAL_MAPPING, "side_channels": SIDE_CHANNELS,
            "mount_horns_links_sensors_unchanged_reported": True,
            "no_discomfort_or_binding_during_recording_reported": True,
            "same_participant_confirmed": None, "positive_yaw_physical_direction_confirmed": True,
            "positive_yaw_direction": "User subsequently confirmed that yaw 0 to +27 degrees closes both sides toward the body.",
            "physical_response_independently_verified": False,
            "note": "User reports do not transform recorded electrical values into calibrated contact forces."},
        "sources": {"profile": {"path": profile_path.name, "sha256": digest(profile_path)},
            "pressure_only_point_table": {"path": str(point_table_path.resolve()), "sha256": digest(point_table_path),
                "kind": "locally derived pressure/motor-only table"}, "builder_sha256": digest(__file__)},
        "units": "sum of per-channel mean estimated ADC volts; not N, Pa, total load, or summed-signal p95",
        "synchrony_note": "Means share a dwell interval. MUX channels are sampled sequentially, not simultaneously; sum variance/quantiles are not derived here.",
        "later_session": {"session_id": SID_LAST, "neutral_a1_statistics": {c: base[c] for c in PHYSICAL_MAPPING},
                          "neutral_side_sum_of_means_v": base_sums, "independent_branches": branches},
        "earlier_left_only_comparison": {"earlier_session_id": SID_EARLIER_LEFT, "later_session_id": SID_LAST,
            "same_full_commands": True, "points": earlier_rows,
            "interpretation": "Same-direction signed responses differ, including sign reversal between sessions; causal pressure gain is not identified."},
        "first_last_equal_yaw_comparison": {"first_session_id": SID_FIRST, "last_session_id": SID_LAST,
            "same_full_commands": True, "points": diagonal,
            "a1_absolute_same_command_mean_difference_v": {c: describe([abs(x["last_minus_first_a1_mean_v"][c]) for x in diagonal]) for c in PHYSICAL_MAPPING},
            "interpretation": "Both sides moved together in these matched commands. This comparison is drift evidence, not independent-actuator gain."},
        "conclusions": [
            "In the later session, every nonzero independent-branch yaw point reduces the left mapped mean-voltage sum relative to that session's neutral.",
            "The right mapped response is nonmonotonic; its first positive-yaw point reduces the right mean-voltage sum.",
            "Earlier left-only measurements reverse the sign at multiple identical full commands; physical causal polarity cannot be inferred from the later session alone.",
            "Historical per-channel running p95 maxima are guard envelopes, not a simultaneous group response or evidence of positive gain.",
            "Current data do not identify an angle for a desired support-pressure increase. Do not flip motor/channel signs or extrapolate negative yaw to force a pressure response.",
            "User-confirmed closing direction can support a separate position-feedforward cue: normalized vehicle demand times a bounded initial angle, with fresh-pressure guards. It does not establish pressure tracking or sway reduction.",
            "A new local repeated neutral-to-small-angle pressure/motor-only identification is needed before claiming a pressure gain or adding pressure-target tracking. Actual motion requires a separate deliberate commissioning step.",
        ],
        "limitations": ["No repeated randomized neutral returns in the source experiment", "Time, preload, posture/contact drift, and hysteresis are confounded",
            "Commanded telemetry is not shaft or plate feedback", "No calibrated force/pressure or verified human-contact ceiling",
            "No video/image/frame/pose-derived input used", "Historical absence of discomfort does not identify a general safe range"],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path, default=Path(__file__).with_name("profiles")/"bench_profile.json")
    parser.add_argument("--point-table", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = build(args.profile, args.point_table)
    args.output.write_text(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False)+"\n", encoding="utf-8")


if __name__ == "__main__":
    main()
