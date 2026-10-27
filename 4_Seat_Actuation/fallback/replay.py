"""Vehicle-only replay: demand and candidate targets, never simulated proof of pressure."""
from __future__ import annotations

from collections import Counter
from dataclasses import asdict
import csv
import hashlib
import json
from pathlib import Path
import subprocess

from .beamng import VehicleValidator, read_vehicle_csv
from .controller import LateralDemand
from .profile import BenchProfile, Settings


def replay_vehicle(vehicle_csv, output_dir, settings=None, profile=None, plot=False):
    settings, profile = settings or Settings(), profile or BenchProfile()
    calibration = profile.derive(settings)
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    result_csv = output/"vehicle_demand_replay.csv"
    if result_csv.exists() or (output/"report.json").exists():
        raise FileExistsError("Choose a new output directory; existing results are immutable")
    validator, demand = VehicleValidator(), LateralDemand(settings)
    reasons, total, first, ready_since = Counter(), 0, None, None
    maximum = {"left": 0., "right": 0.}
    rows_for_plot = []
    fields = ["host_monotonic_ns", "elapsed_s", "sim_time_s", "stream_id", "sequence_id",
              "source_valid", "source_reason", "ay_mps2", "filtered_ay_mps2", "active_side",
              "demand_fraction", "stable_input", "candidate_left_yaw_deg", "candidate_right_yaw_deg",
              "pressure_feedback_available", "hardware_output_enabled"]
    with result_csv.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fields)
        writer.writeheader()
        for raw in read_vehicle_csv(vehicle_csv):
            # Offline time is the recorded original host clock. Live receiver
            # uses perf_counter_ns instead: reading an old file cannot ARM.
            timestamp = int(raw["host_monotonic_ns"])
            frame = validator.parse(raw, timestamp)
            if first is None:
                first = timestamp
            total += 1
            reasons[frame.reason] += 1
            healthy = frame.valid or frame.reason == "stopped"
            if not healthy:
                ready_since = None
                demand.reset()
                side, fraction = "none", 0.
            else:
                if ready_since is None:
                    ready_since = timestamp
                if frame.reason == "stopped":
                    demand.reset()
                    side, fraction = "none", 0.
                else:
                    side, fraction = demand.update(frame.ay_mps2, timestamp)
            stable = healthy and timestamp-ready_since >= settings.baseline_seconds*1e9
            angles = {"left": 0., "right": 0.}
            if stable and side != "none" and not calibration.reasons:
                angles[side] = calibration.angle_for_demand(side, fraction)
            for key in angles:
                maximum[key] = max(maximum[key], angles[key])
            elapsed = (timestamp-first)/1e9
            row = dict(host_monotonic_ns=timestamp, elapsed_s=elapsed, sim_time_s=frame.sim_time_s,
                       stream_id=frame.stream_id, sequence_id=frame.sequence_id, source_valid=int(frame.valid),
                       source_reason=frame.reason, ay_mps2=frame.ay_mps2,
                       filtered_ay_mps2=demand.filtered, active_side=side, demand_fraction=fraction,
                       stable_input=int(stable), candidate_left_yaw_deg=angles["left"],
                       candidate_right_yaw_deg=angles["right"], pressure_feedback_available=0,
                       hardware_output_enabled=0)
            writer.writerow(row)
            if plot:
                rows_for_plot.append((elapsed, frame.ay_mps2, demand.filtered, fraction, int(healthy)))
    try:
        revision = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=Path(__file__).parent,
                                           text=True, stderr=subprocess.DEVNULL).strip()
    except (OSError, subprocess.CalledProcessError):
        revision = "unavailable"
    source_hash = hashlib.sha256(Path(vehicle_csv).read_bytes()).hexdigest()
    code_hashes = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in Path(__file__).parent.glob("*.py")}
    report = dict(mode="offline_vehicle_demand_replay", rows=total, reasons=dict(reasons),
                  source_path=str(Path(vehicle_csv).resolve()), source_sha256=source_hash,
                  base_git_commit=revision, code_sha256=code_hashes, settings=asdict(settings),
                  calibration=calibration.report(),
                  candidate_peak_yaw_deg=maximum, hardware_commands_sent=0,
                  pressure_feedback_available=False,
                  interpretation="Only vehicle data quality/demand are evaluated. Candidate targets are not motor trajectories or pressure predictions. No measured sway reduction or force cancellation.",
                  live_fault_recovery="Live faults latch OFF. Replay resumes separate valid windows only for offline analysis.")
    (output/"report.json").write_text(json.dumps(report, indent=2, ensure_ascii=False)+"\n", encoding="utf-8")
    if plot and rows_for_plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        t, ay, filtered, fraction, healthy = zip(*rows_for_plot)
        fig, axes = plt.subplots(3, 1, figsize=(12, 7), sharex=True, constrained_layout=True)
        axes[0].plot(t, ay, lw=.4, label="Recorded ay (m/s²)")
        axes[0].plot(t, filtered, lw=.7, label="Filtered valid ay")
        axes[0].legend()
        axes[1].plot(t, fraction, lw=.7)
        axes[1].set_ylabel("Scaled cue demand [0, 1]")
        axes[2].plot(t, healthy, lw=.7)
        axes[2].set_ylabel("Input usable")
        axes[2].set_xlabel("Original host elapsed time (s)")
        fig.suptitle("OFFLINE vehicle replay — no pressure feedback or hardware commands")
        fig.savefig(output/"vehicle_demand_replay.png", dpi=160)
        plt.close(fig)
    return report
