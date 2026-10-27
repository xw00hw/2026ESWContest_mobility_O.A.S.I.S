"""Explicit Experiment A launcher with BeamNG-only controller inputs.

The same loop owns model ticks and motor leases. Optional pressure acquisition
is an observation log only; no pressure values reach the controller. Importing,
offline validation and replay never open UART/UDP. Faults never auto re-ARM.
"""
from __future__ import annotations

from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import time

from .vehicle import BridgeReceiver, VehicleValidator
from .cad_limits import CAD_PROFILE, CAD_PROFILE_SHA256
from .tuning import apply_firmware_tuning
from .impact import ImpactDetector, MAX_ABS_ACCEL_MPS2


def _write_manifest(path, manifest):
    temporary=path.with_suffix('.tmp')
    temporary.write_text(json.dumps(manifest,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    temporary.replace(path)


def preflight(settings):
    """Reject unresolved experiment/hardware settings before opening a device."""
    settings.validate()
    missing = settings.missing_confirmations()
    if missing:
        raise ValueError("Hardware session blocked before opening any device: "+
                         "; ".join("confirm:"+name for name in missing))


def _vehicle_issue(vehicle, settings):
    if not (vehicle.valid or vehicle.reason == "stopped"):
        return vehicle.reason
    if getattr(settings, "acceleration_source", "linear_acceleration") == "specific_force":
        if vehicle.specific_force_x_mps2 is None or vehicle.specific_force_y_mps2 is None:
            return "missing_selected_specific_force"
        if (abs(vehicle.specific_force_x_mps2) > MAX_ABS_ACCEL_MPS2 or
                abs(vehicle.specific_force_y_mps2) > MAX_ABS_ACCEL_MPS2):
            return "selected_specific_force_outlier"
    return None


def run_live(port, output_dir, settings, *, record_pressure=False, stop_event=None,
             status_callback=None, startup_timeout_s=30.0, session_metadata=None):
    preflight(settings)
    from .controller import ExperimentAController
    from .transport import BoardLink, open_serial
    from .session_logging import SessionLogger

    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    log_path, manifest_path = output/"control.jsonl", output/"manifest.json"
    if log_path.exists() or manifest_path.exists():
        raise FileExistsError("Choose a new session output directory")
    manifest = dict(schema="oasis-experiment-a-run-v2", status="starting", port=port,
                    cad_profile=CAD_PROFILE, cad_profile_sha256=CAD_PROFILE_SHA256,
                    body_compensation_used=False, human_angle_predicted=False,
                    settings=asdict(settings), started_wall_time_ns=time.time_ns(),
                    settings_role='local_preflight_draft; six command parameters are replaced by committed STM tuning before ARM',
                    effective_settings=None, firmware_tuning=None,
                    code_sha256={p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                 for p in Path(__file__).parent.glob("*.py")},
                    shared_dependency_sha256={name: hashlib.sha256(
                        (Path(__file__).parent.parent/"fallback"/name).read_bytes()).hexdigest()
                        for name in ("beamng.py", "serial_link.py")},
                    runtime_control_input="BeamNG collector vehicle data only",
                    board_telemetry_role="Command/transport health; no shaft-position sensor",
                    pressure_acquisition_enabled=record_pressure,
                    pressure_role="observation_only_never_policy_input",
                    auto_arm="once_after_fresh_vehicle_and_verified_OFF",
                    startup_timeout_s=startup_timeout_s,
                    impact=None, impact_response="motor_off_recording_continues",
                    session_metadata=session_metadata or {},
                    clock_anchor=dict(host_monotonic_ns=time.perf_counter_ns(),wall_time_ns=time.time_ns()),
                    human_posture_measured=False, physical_effect_verified=False)
    _write_manifest(manifest_path,manifest)
    receiver = link = None
    arm_attempted = False
    result = "startup_error"
    logger = SessionLogger(output, pressure_enabled=record_pressure)
    try:
        def record(kind, **payload):
            logger.record(kind, host_monotonic_ns=time.perf_counter_ns(), **payload)
        def notify(kind, **payload):
            if status_callback:
                status_callback(kind, payload)
        try:
            receiver = BridgeReceiver()
            link = BoardLink(open_serial(port), record_pressure=True) if record_pressure else BoardLink(open_serial(port))
            now = time.perf_counter_ns()
            link.start(now)
            started = waiting_since = now
            last_ping = last_state = last_set = last_tick = last_status = 0
            startup_good_since = None
            validator = VehicleValidator(max_age_ms=settings.fresh_ms,
                                         max_host_gap_ms=settings.fresh_ms,
                                         max_sim_dt_s=getattr(settings, "max_model_sample_dt_s", .1))
            vehicle = None
            controller = None
            impact_detector = ImpactDetector(settings)
            impact_event = None
            impact_off_started_ns = None
            frozen_tuning = None
            last_sent_pose = (0, 0, 0, 0)
            phase = "WAIT_VEHICLE"

            def latch_impact(state, reason, frame, detected_ns):
                nonlocal impact_event, startup_good_since, phase
                if impact_event is not None:
                    return
                impact_event = dict(state=state,
                    detected_host_monotonic_ns=detected_ns,
                    vehicle_key=dict(stream_id=frame.stream_id,
                        vehicle_id=frame.vehicle_id, sequence_id=frame.sequence_id,
                        host_monotonic_ns=frame.received_monotonic_ns),
                    motor_off_verified=False)
                manifest['impact'] = impact_event
                manifest['motor_off_reason'] = reason
                record("impact", **impact_event)
                startup_good_since = None
                phase = "IMPACT_OFF_PENDING"
                notify(phase, reason=reason, impact=impact_event, off_verified=False)

            notify(phase,port=port)
            print("Experiment A: waiting for fresh BeamNG and verified motor OFF. "
                  "Pressure is observation-only. Stop sends OFF.", flush=True)
            while True:
                now = time.perf_counter_ns()
                logger.check()
                if stop_event is not None and stop_event.is_set():
                    result = "stop_requested"
                    record("stop",reason=result)
                    break
                link.poll(now)
                if record_pressure:
                    logger.pressure(link.drain_pressure_records())
                if link.fault:
                    raise RuntimeError("board_link:"+link.fault)
                if link.ready:
                    if frozen_tuning is None:
                        settings,frozen_tuning=apply_firmware_tuning(settings,link.tuning)
                        controller=ExperimentAController(settings)
                        manifest['effective_settings']=asdict(settings)
                        manifest['firmware_tuning']=dict(frozen_tuning)
                        manifest['firmware_identity']=dict(link.hello)
                        _write_manifest(manifest_path,manifest)
                        record('firmware_tuning_loaded',firmware_tuning=frozen_tuning,
                               effective_settings=asdict(settings))
                        notify('telemetry',port=port,firmware_tuning=dict(frozen_tuning),
                               effective_settings=asdict(settings))
                    elif link.tuning != frozen_tuning:
                        raise RuntimeError('firmware_tuning_changed_during_session; stop and restart required')
                # Check EVERY row: a valid later row in the same batch cannot
                # hide a reset, derivative error, packet gap or clock fault.
                for _ in range(32):
                    try:
                        row = receiver.receive()
                    except (ValueError, TypeError) as exc:
                        if impact_event is None:
                            raise
                        record("vehicle_rejected", reason=str(exc), recording_only=True)
                        continue
                    if row is None:
                        break
                    # The collector can timestamp a new row while UART work
                    # runs. Validate against receipt-time now, never loop-start.
                    now = time.perf_counter_ns()
                    try:
                        vehicle = validator.parse(row, now)
                    except (ValueError, TypeError) as exc:
                        if impact_event is None:
                            raise
                        record("vehicle_rejected", reason=str(exc), recording_only=True)
                        continue
                    # Preserve the rejected source sample too: stopping must
                    # not erase the acceleration that explains the fault.
                    record("vehicle", frame=asdict(vehicle),
                           collector_session_id=row.get("_bridge_session_id"),
                           collector_config_sha256=row.get("_bridge_config_sha256"))
                    # Unlike the 20 ms position loop, this sees EVERY source
                    # row, including a vehicle that has already come to rest.
                    # A later normal row cannot erase a latched impact event.
                    impact_state = impact_detector.observe(vehicle)
                    if impact_state.triggered and impact_event is None:
                        latch_impact(impact_state.to_dict(), impact_state.reason, vehicle, now)
                    issue = _vehicle_issue(vehicle, settings)
                    if issue:
                        startup_good_since = None
                        if arm_attempted and impact_event is None:
                            raise RuntimeError("vehicle:"+issue)
                now = time.perf_counter_ns()
                if stop_event is not None and stop_event.is_set():
                    result = "stop_requested"
                    record("stop",reason=result)
                    break
                vehicle_good = (vehicle is not None and
                    _vehicle_issue(vehicle, settings) is None and
                    0 <= now-vehicle.received_monotonic_ns <= settings.fresh_ms*1e6)
                if not vehicle_good:
                    startup_good_since = None
                    if arm_attempted and impact_event is None:
                        raise RuntimeError("vehicle_stale_or_invalid")
                if impact_event is not None:
                    # The initial handshake already holds motor outputs OFF.
                    # If the event preceded readiness, finish that handshake
                    # without allowing ARM, then latch recording-only mode.
                    if impact_off_started_ns is None and link.ready:
                        link.motor_off_keep_recording(now)
                        impact_off_started_ns = now
                        impact_event['motor_off_requested_host_monotonic_ns'] = now
                    if impact_off_started_ns is not None and phase == "IMPACT_OFF_PENDING":
                        if (link.motor_off_verified and link.state is not None and
                                link.state.received_ns > impact_off_started_ns and
                                not link.state.armed and not any(link.state.pulses_us)):
                            phase = "RECORD_ONLY"
                            impact_event.update(motor_off_verified=True,
                                motor_off_verified_host_monotonic_ns=link.state.received_ns)
                            record("impact_motor_off_verified", impact=impact_event,
                                   board=asdict(link.state))
                            notify(phase, reason=manifest['motor_off_reason'], impact=impact_event, off_verified=True,
                                   board=asdict(link.state))
                        elif now-impact_off_started_ns >= 300_000_000:
                            raise RuntimeError("impact_motor_off_not_verified")
                elif not arm_attempted:
                    if startup_timeout_s is not None and now-started > startup_timeout_s*1e9:
                        raise TimeoutError("No qualified startup inputs within configured timeout")
                    board_good = (link.ready and controller is not None and link.state is not None and
                        not link.state.armed and not any(link.state.pulses_us) and
                        0 <= now-link.state.received_ns <= settings.fresh_ms*1e6)
                    qualified = board_good and vehicle_good
                    if not qualified:
                        startup_good_since = None
                    elif startup_good_since is None:
                        startup_good_since = now
                    if (qualified and now-startup_good_since >= settings.startup_seconds*1e9
                            and link.pending is None and link.arm(now)):
                        arm_attempted = True
                        waiting_since = now
                        phase = "WAIT_ARM_STATE"
                        record("arm_sent", command=link.pending.command if link.pending else None)
                        notify(phase,port=port)
                elif phase == "WAIT_ARM_STATE":
                    if (link.armed_session is not None and link.state is not None
                            and link.state.armed and link.state.received_ns > waiting_since):
                        if (link.state.pose_mdeg != (0, 0, 0, 0) or
                                link.state.target_mdeg != (0, 0, 0, 0) or link.state.moving):
                            raise RuntimeError("nonneutral_initial_command_state")
                        phase = "CONTROL"
                        record("arm_confirmed", state=asdict(link.state))
                        notify(phase,port=port)
                    elif now-waiting_since > 250_000_000:
                        raise TimeoutError("Fresh neutral armed state not received")
                if phase == "CONTROL" and now-last_tick >= 20_000_000:
                    last_tick = now
                    decision = controller.tick(vehicle, link.state, now)
                    record("control", decision=asdict(decision), board=asdict(link.state),
                           vehicle_key=dict(stream_id=vehicle.stream_id, vehicle_id=vehicle.vehicle_id,
                                            sequence_id=vehicle.sequence_id,
                                            host_monotonic_ns=vehicle.received_monotonic_ns))
                    if decision.emergency_off or not decision.permit_lease:
                        if decision.reason.startswith('impact_suspected_'):
                            latch_impact(decision.diag.get('impact', dict(
                                triggered=True, reason=decision.reason)), decision.reason, vehicle, now)
                            link.motor_off_keep_recording(now)
                            impact_off_started_ns = now
                            impact_event['motor_off_requested_host_monotonic_ns'] = now
                        else:
                            raise RuntimeError("policy:"+decision.reason)
                    if impact_event is None and link.pending is None:
                        if now-last_ping >= 100_000_000:
                            if link.ping(now):
                                last_ping = now
                        elif now-last_state >= 100_000_000:
                            if link.request_state(now):
                                last_state = now
                        elif decision.pose_mdeg != last_sent_pose and now-last_set >= 40_000_000:
                            if link.set_pose(decision.pose_mdeg, now):
                                last_sent_pose, last_set = decision.pose_mdeg, now
                # Startup and ARM confirmation request telemetry only. No
                # motor heartbeat can renew an unqualified policy's authority.
                if link.ready and link.pending is None and now-last_state >= 100_000_000:
                    if link.request_state(now):
                        last_state = now
                if record_pressure and link.ready and link.pending is None:
                    link.pressure_ping(now)
                if now-last_status >= 500_000_000:
                    notify("telemetry",phase=phase,port=port,vehicle_good=vehicle_good,
                           board=asdict(link.state) if link.state else None,
                           impact=impact_event,
                           off_verified=bool(impact_event and impact_event['motor_off_verified']),
                           firmware_tuning=dict(frozen_tuning) if frozen_tuning is not None else None,
                           pressure=link.pressure_status(now) if record_pressure else None)
                    last_status=now
                time.sleep(.002)
        except KeyboardInterrupt:
            result = "operator_stop"
            record("stop", reason=result)
        except Exception as exc:
            result = str(exc)
            try: record("fault", reason=result)
            except Exception: pass  # A failed logger must never prevent motor OFF.
            raise
        finally:
            # PWM OFF is a signal state, not a promise that a loaded mechanism
            # releases pressure or remains stationary when unpowered.
            off_verified = False
            if link is not None:
                try:
                    off_started_ns = time.perf_counter_ns()
                    link.off()
                    deadline = off_started_ns+300_000_000
                    while time.perf_counter_ns() < deadline:
                        link.poll(time.perf_counter_ns())
                        if (link.state and link.state.received_ns > off_started_ns
                                and not link.state.armed and not any(link.state.pulses_us)):
                            off_verified = True
                            if (not record_pressure or
                                    (link.pressure_status(time.perf_counter_ns())['scan_disabled_verified']
                                     and link.pending is None)):
                                break
                        time.sleep(.002)
                except Exception as exc:
                    try: record("off_error", reason=str(exc))
                    except Exception: pass
                finally:
                    link.close()
            if receiver is not None:
                receiver.close()
            manifest.update(status="ended", result=result, ended_wall_time_ns=time.time_ns(),
                            arm_attempted=arm_attempted, pwm_off_telemetry_verified=off_verified,
                            final_board_state=asdict(link.state) if link and link.state else None)
            if record_pressure and link is not None:
                manifest['pressure_summary']=link.pressure_status(time.perf_counter_ns())
                try: logger.pressure(link.drain_pressure_records())
                except Exception: pass
            print("Session ended; no automatic reconnect or re-ARM. Result: "+result, flush=True)
    finally:
        try:
            manifest['logging']=logger.close()
        except Exception as exc:
            manifest['logging_error']=str(exc)
            manifest['status']='incomplete'
            raise
        finally:
            _write_manifest(manifest_path,manifest)
            if status_callback:
                status_callback("ended",dict(result=result,manifest=str(manifest_path),
                    off_verified=manifest.get('pwm_off_telemetry_verified',False),
                    board=manifest.get('final_board_state'),
                    pressure=manifest.get('pressure_summary'),
                    logging_error=manifest.get('logging_error')))
    return manifest
