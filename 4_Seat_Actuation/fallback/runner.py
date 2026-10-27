"""Explicit hardware launcher. One loop owns both control decisions and leases.

Nothing here runs on import. No auto-flash, port scanning, reconnect or re-ARM.
"""
from __future__ import annotations

from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import time

from .beamng import BridgeReceiver, VehicleValidator
from .controller import FallbackController


def preflight(settings, profile):
    calibration = profile.derive(settings)
    missing = settings.missing_confirmations()
    if calibration.reasons or missing:
        raise ValueError("Hardware session blocked before opening any device: "+
                         "; ".join(calibration.reasons+["confirm:"+x for x in missing]))
    return calibration


def run_live(port, output_dir, settings, profile):
    calibration = preflight(settings, profile)
    # Dependency import follows preflight, so an incomplete configuration does
    # not even require pyserial to be installed.
    from .serial_link import BoardLink, open_serial

    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    log_path = output/"control.jsonl"
    if log_path.exists() or (output/"manifest.json").exists():
        raise FileExistsError("Choose a new session output directory")
    manifest = dict(schema="oasis-fallback-run-v1", status="starting", port=port,
                    settings=asdict(settings), calibration=calibration.report(),
                    started_wall_time_ns=time.time_ns(),
                    code_sha256={p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                 for p in Path(__file__).parent.glob("*.py")},
                    data_policy="No camera/video/image/pose/RL inputs", physical_effect_verified=False)
    manifest_path = output/"manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2)+"\n", encoding="utf-8")
    receiver = link = None
    arm_attempted = False
    result = "startup_error"
    with log_path.open("x", encoding="utf-8") as log:
        def record(kind, **payload):
            log.write(json.dumps(dict(event=kind, host_monotonic_ns=time.perf_counter_ns(), **payload),
                                 allow_nan=False, separators=(",", ":"))+"\n")
        try:
            receiver = BridgeReceiver()
            link = BoardLink(open_serial(port))
            now = time.perf_counter_ns()
            link.start(now)
            started, waiting_since = now, now
            last_ping = last_state = last_set = last_pressure_ping = last_flush = last_tick = 0
            good_since = startup_good_since = None
            validator, vehicle = VehicleValidator(), None
            controller = FallbackController(calibration)
            last_sent_pose = (0,0,0,0)
            phase = "WAIT_INPUTS"
            print("Waiting for fallback firmware and fresh BeamNG/pressure data. Ctrl+C sends OFF.", flush=True)
            while True:
                now = time.perf_counter_ns()
                link.poll(now)
                if link.fault:
                    raise RuntimeError("board_link:"+link.fault)
                # Drain at most 32 datagrams, validating EVERY one. A later good
                # sample in the same poll cannot hide an earlier bad sample.
                for _ in range(32):
                    row = receiver.receive()
                    if row is None:
                        break
                    vehicle = validator.parse(row, now)
                    healthy = vehicle.valid or vehicle.reason == "stopped"
                    if not healthy:
                        good_since = None
                        startup_good_since = None
                        if arm_attempted:
                            raise RuntimeError("vehicle:"+vehicle.reason)
                    elif good_since is None:
                        good_since = now
                    record("vehicle", frame=asdict(vehicle),
                           collector_session_id=row.get("_bridge_session_id"),
                           collector_config_sha256=row.get("_bridge_config_sha256"))
                vehicle_fresh = (vehicle is not None and
                                 0 <= now-vehicle.received_monotonic_ns <= settings.fresh_ms*1e6)
                if not vehicle_fresh:
                    good_since = None
                    startup_good_since = None
                    if arm_attempted:
                        raise RuntimeError("vehicle_stale")
                if not arm_attempted:
                    if now-started > 30_000_000_000:
                        raise TimeoutError("No qualified startup inputs within 30 seconds")
                    pressure = link.pressure
                    pressure_good = pressure is not None and pressure.valid and 0 <= now-pressure.received_ns <= settings.fresh_ms*1e6
                    qualified = link.ready and vehicle_fresh and pressure_good and good_since is not None
                    if not qualified:
                        startup_good_since = None
                    elif startup_good_since is None:
                        startup_good_since = now
                    if (qualified and now-startup_good_since >= settings.baseline_seconds*1e9
                            and link.pending is None):
                        if link.arm(now):
                            arm_attempted = True
                            waiting_since = now
                            phase = "WAIT_ARM_STATE"
                            record("arm_sent", command=link.pending.command if link.pending else None)
                elif phase == "WAIT_ARM_STATE":
                    if link.state and link.state.armed and link.state.received_ns >= waiting_since:
                        phase = "CONTROL"
                        record("arm_confirmed", state=asdict(link.state))
                    elif now-waiting_since > 250_000_000:
                        raise TimeoutError("Fresh armed state not received")
                if phase == "CONTROL" and now-last_tick >= 20_000_000:
                    last_tick = now
                    decision = controller.tick(vehicle, link.pressure, link.state, now)
                    record("control", decision=asdict(decision), pressure=asdict(link.pressure),
                           board=asdict(link.state), baseline_v=controller.baseline,
                           vehicle_key=dict(stream_id=vehicle.stream_id, vehicle_id=vehicle.vehicle_id,
                                            sequence_id=vehicle.sequence_id,
                                            host_monotonic_ns=vehicle.received_monotonic_ns))
                    if decision.emergency_off or not decision.permit_lease:
                        raise RuntimeError("policy:"+decision.reason)
                    # Independent authority refresh comes only from this healthy
                    # decision. A background pressure logger cannot keep it alive.
                    if link.pending is None:
                        if now-last_ping >= 100_000_000:
                            if link.ping(now):
                                last_ping = now
                        elif now-last_state >= 100_000_000:
                            if link.request_state(now):
                                last_state = now
                        elif decision.pose_mdeg != last_sent_pose and now-last_set >= 40_000_000:
                            if link.set_pose(decision.pose_mdeg, now):
                                last_sent_pose, last_set = decision.pose_mdeg, now
                # Before ARM, and while awaiting the first state, no motor
                # heartbeat is sent. ARM's initial lease covers this brief wait.
                if link.ready and link.pending is None:
                    if now-last_state >= 100_000_000:
                        if link.request_state(now):
                            last_state = now
                    elif now-last_pressure_ping >= 1_000_000_000:
                        if link.pressure_ping(now):
                            last_pressure_ping = now
                if now-last_flush >= 1_000_000_000:
                    log.flush()
                    last_flush = now
                time.sleep(.002)
        except KeyboardInterrupt:
            result = "operator_stop"
            record("stop", reason=result)
        except Exception as exc:
            result = str(exc)
            record("fault", reason=result)
            raise
        finally:
            # Best effort OFF; watchdog/lease also cover a crashed host. PWM OFF
            # is not a promise that a loaded mechanism releases or stays still.
            off_verified = False
            if link is not None:
                try:
                    off_started_ns = time.perf_counter_ns()
                    link.off()
                    deadline = time.perf_counter_ns()+300_000_000
                    while time.perf_counter_ns() < deadline:
                        link.poll(time.perf_counter_ns())
                        if (link.state and link.state.received_ns >= off_started_ns
                                and not link.state.armed and not any(link.state.pulses_us)):
                            off_verified = True
                            break
                        time.sleep(.002)
                except Exception as exc:
                    record("off_error", reason=str(exc))
                finally:
                    link.close()
            if receiver is not None:
                receiver.close()
            manifest.update(status="ended", result=result, ended_wall_time_ns=time.time_ns(),
                            arm_attempted=arm_attempted, pwm_off_telemetry_verified=off_verified)
            manifest_path.write_text(json.dumps(manifest, indent=2)+"\n", encoding="utf-8")
            print("Session ended; no automatic reconnect or re-ARM. Result: "+result, flush=True)
