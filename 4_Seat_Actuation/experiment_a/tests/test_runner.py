"""Launcher integration using real parsers/link, a fixture policy and fake I/O.

The policy fixture tests orchestration and pressure exclusion independently of
the separately tested human model. No actual UART, socket, game or board opens.
"""
from __future__ import annotations

import contextlib
import io
import json
from pathlib import Path
import socket
import sys
import tempfile
import threading
import types
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from experiment_a import runner, transport
from experiment_a.vehicle import BridgeReceiver
from experiment_a.tests.runtime_fixtures import (
    FakeClock, FakeSocket, MotorUART, RuntimeController, RuntimeSettings,
)


class RunnerTests(unittest.TestCase):
    def run_fixture(self, *, settings=None, stop_after=3.5, source_invalid=None,
                    source_stop=None, noise=False, suppress_final_off=False,
                    nonneutral_arm=False, ay=2., receiver_type=FakeSocket, real_policy=False,
                    stop_event=None, uart_type=MotorUART):
        clock = FakeClock(stop_after)
        uart = uart_type(clock, noise=noise, suppress_final_off=suppress_final_off,
                         nonneutral_arm=nonneutral_arm)
        udp = receiver_type(clock, source_invalid=source_invalid, source_stop=source_stop, ay=ay)
        fake_controller_module = types.ModuleType("experiment_a.controller")
        fake_controller_module.ExperimentAController = RuntimeController
        policy_context = (contextlib.nullcontext() if real_policy else
                          patch.dict(sys.modules, {"experiment_a.controller": fake_controller_module}))
        error = None
        with tempfile.TemporaryDirectory() as temp:
            with patch.object(runner, "time", clock), \
                 patch.object(runner, "BridgeReceiver", lambda: BridgeReceiver(socket_factory=lambda *a: udp)), \
                 patch.object(transport, "open_serial", return_value=uart) as opening, \
                 patch.object(socket, "socket", side_effect=AssertionError("Real socket forbidden")), \
                 policy_context, \
                 contextlib.redirect_stdout(io.StringIO()):
                try: runner.run_live("OFFLINE_FAKE_ONLY", temp, settings or RuntimeSettings(),
                                     stop_event=stop_event)
                except Exception as exc: error = exc
            manifest_path = Path(temp)/"manifest.json"
            log_path = Path(temp)/"control.jsonl"
            manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else None
            events = [json.loads(line) for line in log_path.read_text().splitlines()] if log_path.exists() else []
        return dict(error=error, clock=clock, uart=uart, socket=udp,
                    opens=opening.call_count, manifest=manifest, events=events)

    def assert_stopped(self, result, *, verified=True):
        self.assertTrue(result["uart"].closed)
        self.assertTrue(result["socket"].closed)
        self.assertEqual(result["manifest"]["pwm_off_telemetry_verified"], verified)
        commands = [c for _, c in result["uart"].commands]
        end_off = [i for i, c in enumerate(commands) if c == "FB_OFF"][1]
        self.assertFalse(any(c.startswith(("FB_ARM,", "FB_SET,", "FB_PING,")) for c in commands[end_off+1:]))

    def test_beamng_only_startup_both_axes_and_clean_operator_stop(self):
        result = self.run_fixture()
        self.assertIsNone(result["error"])
        commands = result["uart"].commands
        arms = [(ns, c) for ns, c in commands if c.startswith("FB_ARM,")]
        self.assertEqual(len(arms), 1)
        self.assertGreaterEqual(arms[0][0]-result["clock"].started_ns, 2_000_000_000)
        poses = [tuple(map(int, c.split(",")[3:])) for _, c in commands if c.startswith("FB_SET,")]
        self.assertIn((0, 0, 1000, 500), poses)
        self.assertFalse(any(c == "HELLO" or c == "PING" or c.startswith("START") for _, c in commands))
        self.assertFalse(result["manifest"]["pressure_acquisition_enabled"])
        control = [event for event in result["events"] if event["event"] == "control"]
        self.assertTrue(control)
        self.assertFalse(any("pressure" in event or "baseline" in event for event in control))
        self.assert_stopped(result)

    def test_row_timestamp_generated_during_receive_is_not_false_future_fault(self):
        class ConcurrentCollectorSocket(FakeSocket):
            def packet(self, **kwargs):
                # Simulate the collector thread timestamping after this owner
                # sampled loop-start time, while UART/UDP work was in progress.
                self.clock.now_ns += 1_000_000
                return super().packet(**kwargs)
        result = self.run_fixture(receiver_type=ConcurrentCollectorSocket)
        self.assertIsNone(result["error"])
        commands = [command for _, command in result["uart"].commands]
        self.assertEqual(sum(command.startswith("FB_ARM,") for command in commands), 1)
        self.assertTrue(any(command.startswith("FB_SET,") for command in commands))
        frames = [event["frame"] for event in result["events"] if event["event"] == "vehicle"]
        self.assertTrue(frames)
        self.assertFalse(any(frame["reason"] == "future_timestamp" for frame in frames))
        self.assert_stopped(result)

    def test_stop_during_receive_preempts_same_iteration_arm_or_lease(self):
        baseline = self.run_fixture()
        self.assertIsNone(baseline["error"])
        commands = baseline["uart"].commands
        arm_ns = next(ns for ns, command in commands if command.startswith("FB_ARM,"))
        active_lease_ns = next(ns for ns, command in commands
                              if command.startswith("FB_PING,") and ns > arm_ns+500_000_000)
        for phase, stop_ns in (("before_arm", arm_ns), ("active_control", active_lease_ns)):
            with self.subTest(phase=phase):
                stop = threading.Event()
                class StoppingSocket(FakeSocket):
                    stopped_at_ns = None
                    def recvfrom(self, count):
                        if self.stopped_at_ns is None and self.clock.now_ns >= stop_ns:
                            self.stopped_at_ns = self.clock.now_ns
                            stop.set()
                        return super().recvfrom(count)
                result = self.run_fixture(receiver_type=StoppingSocket, stop_event=stop)
                self.assertIsNone(result["error"])
                self.assertEqual(result["socket"].stopped_at_ns, stop_ns)
                self.assertEqual(result["manifest"]["result"], "stop_requested")
                moving = [(ns, command) for ns, command in result["uart"].commands
                          if command.startswith(("FB_ARM,", "FB_SET,", "FB_PING,"))]
                self.assertFalse(any(ns >= stop_ns for ns, _ in moving), moving)
                self.assertEqual(sum(command.startswith("FB_ARM,") for _, command in moving),
                                 0 if phase == "before_arm" else 1)
                self.assert_stopped(result)

    def test_pressure_bytes_absent_or_broken_leave_targets_identical(self):
        clean, noise = self.run_fixture(), self.run_fixture(noise=True)
        self.assertIsNone(noise["error"])
        self.assertEqual(clean["uart"].commands, noise["uart"].commands)
        self.assert_stopped(noise)

    def test_invalid_then_valid_same_batch_cannot_hide_fault(self):
        result = self.run_fixture(source_invalid=2.8)
        self.assertIn("invalid_derivative", str(result["error"]))
        self.assert_stopped(result)

    def test_sustained_impact_is_preserved_and_recording_continues_without_rearm(self):
        class CollisionSocket(FakeSocket):
            def packet(self, **kwargs):
                payload, peer = super().packet(**kwargs)
                packet = json.loads(payload)
                if self.clock.elapsed_ns >= 2_800_000_000:
                    packet['row']['linear_acc_y_mps2'] = 30.2687
                return json.dumps(packet).encode('ascii'), peer
        result = self.run_fixture(receiver_type=CollisionSocket)
        self.assertIsNone(result['error'])
        impacts = [e for e in result['events'] if e['event'] == 'impact']
        self.assertEqual(len(impacts), 1)
        self.assertEqual(impacts[0]['state']['reason'], 'impact_suspected_sustained')
        self.assertTrue(result['manifest']['impact']['motor_off_verified'])
        frames = [e for e in result['events'] if e['event'] == 'vehicle'
                  and e['host_monotonic_ns'] > impacts[0]['host_monotonic_ns']]
        self.assertGreater(len(frames), 5)
        self.assertEqual(frames[-1]['frame']['ay_mps2'], 30.2687)
        self.assertTrue(frames[-1]['frame']['valid'])
        self.assertEqual(sum(c.startswith('FB_ARM,') for _, c in result['uart'].commands), 1)
        self.assert_stopped(result)

    def test_large_single_cornering_sample_does_not_stop_motor(self):
        class CorneringSpike(FakeSocket):
            def packet(self, **kwargs):
                payload, peer = super().packet(**kwargs)
                packet = json.loads(payload)
                if 2_800_000_000 <= self.clock.elapsed_ns < 2_820_000_000:
                    packet['row'].update(linear_acc_y_mps2=12.871359949150792,
                                         linear_acc_x_mps2=7.251468709664833)
                return json.dumps(packet).encode('ascii'), peer
        result = self.run_fixture(receiver_type=CorneringSpike)
        self.assertIsNone(result['error'])
        self.assertIsNone(result['manifest']['impact'])
        self.assertTrue(any(ns > result['clock'].started_ns+2_820_000_000 and
                            c.startswith('FB_PING,') for ns, c in result['uart'].commands))

    def test_impact_before_first_arm_keeps_recording_and_never_arms(self):
        result = self.run_fixture(ay=65.)
        self.assertIsNone(result['error'])
        self.assertFalse(result['manifest']['arm_attempted'])
        self.assertEqual(result['manifest']['impact']['state']['reason'], 'impact_suspected_extreme')
        self.assertTrue(result['manifest']['impact']['motor_off_verified'])
        self.assertFalse(any(c.startswith('FB_ARM,') for _, c in result['uart'].commands))
        self.assertGreater(sum(e['event'] == 'vehicle' for e in result['events']), 100)

    def test_impact_after_arm_still_requires_verified_motor_off(self):
        class ExtremeSocket(FakeSocket):
            def packet(self, **kwargs):
                payload, peer = super().packet(**kwargs)
                packet = json.loads(payload)
                if self.clock.elapsed_ns >= 2_800_000_000:
                    packet['row']['linear_acc_x_mps2'] = -65.
                return json.dumps(packet).encode('ascii'), peer
        result = self.run_fixture(receiver_type=ExtremeSocket, suppress_final_off=True)
        self.assertIn('ack_timeout:FB_OFF', str(result['error']))
        self.assertFalse(result['manifest']['impact']['motor_off_verified'])
        self.assert_stopped(result, verified=False)

    def test_impact_then_normal_same_batch_cannot_restore_motor_authority(self):
        class ImpactBatchSocket(FakeSocket):
            def recvfrom(self, count):
                if (self.clock.elapsed_ns >= 2_800_000_000 and not self.injected and
                        self.clock.now_ns >= self.next_ns):
                    self.injected = True
                    self.next_ns = self.clock.now_ns+20_000_000
                    payload, peer = self.packet(offset_ns=-1)
                    packet = json.loads(payload)
                    packet['row']['linear_acc_x_mps2'] = -65.
                    self.queued.append(self.packet())
                    return json.dumps(packet).encode('ascii'), peer
                return super().recvfrom(count)
        result = self.run_fixture(receiver_type=ImpactBatchSocket)
        self.assertIsNone(result['error'])
        self.assertEqual(result['manifest']['impact']['state']['reason'], 'impact_suspected_extreme')
        self.assertTrue(result['manifest']['impact']['motor_off_verified'])
        self.assertEqual(sum(c.startswith('FB_ARM,') for _, c in result['uart'].commands), 1)
        self.assert_stopped(result)

    def test_malformed_vehicle_packets_after_impact_are_logged_without_ending_recording(self):
        class MalformedAfterImpactSocket(FakeSocket):
            def packet(self, **kwargs):
                payload, peer = super().packet(**kwargs)
                if 3_000_000_000 <= self.clock.elapsed_ns < 3_100_000_000:
                    return b'{', peer
                packet = json.loads(payload)
                if 2_800_000_000 <= self.clock.elapsed_ns < 2_820_000_000:
                    packet['row']['linear_acc_x_mps2'] = -65.
                return json.dumps(packet).encode('ascii'), peer
        result = self.run_fixture(receiver_type=MalformedAfterImpactSocket)
        self.assertIsNone(result['error'])
        self.assertTrue(result['manifest']['impact']['motor_off_verified'])
        self.assertTrue(any(e['event'] == 'vehicle_rejected' for e in result['events']))
        self.assertTrue(any(e['event'] == 'vehicle' and e['frame']['valid'] and
                            e['host_monotonic_ns'] > result['clock'].started_ns+3_100_000_000
                            for e in result['events']))
        self.assert_stopped(result)

    def test_stale_vehicle_stops_no_lease_without_data(self):
        result = self.run_fixture(source_stop=2.8)
        self.assertIn("vehicle_stale", str(result["error"]))
        self.assert_stopped(result)

    def test_startup_quality_error_restarts_full_qualification(self):
        result = self.run_fixture(source_invalid=1.5, stop_after=4.5)
        self.assertIsNone(result["error"])
        arms = [ns for ns, c in result["uart"].commands if c.startswith("FB_ARM,")]
        self.assertEqual(len(arms), 1)
        self.assertGreaterEqual(arms[0]-result["clock"].started_ns, 3_500_000_000)
        self.assert_stopped(result)

    def test_nonneutral_arm_state_never_gets_motion_or_lease(self):
        result = self.run_fixture(nonneutral_arm=True)
        self.assertIn("nonneutral_initial", str(result["error"]))
        self.assertFalse(any(c.startswith(("FB_SET,", "FB_PING,")) for _, c in result["uart"].commands))
        self.assert_stopped(result)

    def test_unconfirmed_settings_open_no_devices(self):
        result = self.run_fixture(settings=RuntimeSettings(confirmations={"fixture_only": False}))
        self.assertIn("before opening any device", str(result["error"]))
        self.assertEqual(result["opens"], 0)
        self.assertIsNone(result["socket"].bound)
        self.assertIsNone(result["manifest"])

    def test_silent_shutdown_cannot_reuse_old_off_state(self):
        result = self.run_fixture(suppress_final_off=True)
        self.assert_stopped(result, verified=False)

    def test_negative_lateral_input_uses_other_side(self):
        result = self.run_fixture(ay=-2.)
        self.assertIsNone(result["error"])
        commands = [c for _, c in result["uart"].commands]
        self.assertTrue(any(c.endswith(",1000,-500,0,0") for c in commands if c.startswith("FB_SET,")))
        self.assert_stopped(result)

    def test_stream_reset_reordering_and_future_clock_each_stop_runtime(self):
        for changes, expected in (({"stream_id": "changed"}, "stream_changed"),
                                  ({"sequence_id": 1}, "duplicate_or_reordered"),
                                  ({"sim_time_s": .001}, "nonincreasing_sim_time"),
                                  ({"host_monotonic_ns": 90_000_000_000}, "future_timestamp")):
            class ChangedSocket(FakeSocket):
                def packet(self, **kwargs):
                    payload, peer = super().packet(**kwargs)
                    if self.clock.elapsed_ns >= 2_800_000_000:
                        packet = json.loads(payload)
                        packet["row"].update(changes)
                        packet["forwarded_monotonic_ns"] = max(
                            packet["forwarded_monotonic_ns"], packet["row"]["host_monotonic_ns"])
                        payload = json.dumps(packet).encode("ascii")
                    return payload, peer
            with self.subTest(reason=expected):
                result = self.run_fixture(receiver_type=ChangedSocket)
                self.assertIn(expected, str(result["error"]))
                self.assert_stopped(result)

    def test_missing_selected_specific_force_never_arms(self):
        result = self.run_fixture(settings=RuntimeSettings(acceleration_source="specific_force"))
        self.assertIsNone(result["error"])
        self.assertFalse(result["manifest"]["arm_attempted"])
        self.assertFalse(any(c.startswith("FB_ARM,") for _, c in result["uart"].commands))
        self.assert_stopped(result)

    def test_specific_force_disappearing_after_arm_stops_without_substitution(self):
        class SelectedSourceSocket(FakeSocket):
            def packet(self, **kwargs):
                payload, peer = super().packet(**kwargs)
                packet = json.loads(payload)
                if self.clock.elapsed_ns < 2_800_000_000:
                    packet["row"].update(specific_force_x_mps2=0., specific_force_y_mps2=2.)
                return json.dumps(packet).encode("ascii"), peer
        result = self.run_fixture(settings=RuntimeSettings(acceleration_source="specific_force"),
                                  receiver_type=SelectedSourceSocket)
        self.assertIn("missing_selected_specific_force", str(result["error"]))
        self.assert_stopped(result)

    def test_selected_specific_force_outlier_never_arms(self):
        class OutlierSocket(FakeSocket):
            def packet(self, **kwargs):
                payload, peer = super().packet(**kwargs)
                packet = json.loads(payload)
                packet["row"].update(specific_force_x_mps2=0., specific_force_y_mps2=201.)
                return json.dumps(packet).encode("ascii"), peer
        result = self.run_fixture(settings=RuntimeSettings(acceleration_source="specific_force"),
                                  receiver_type=OutlierSocket)
        self.assertIsNone(result["error"])
        self.assertFalse(result["manifest"]["arm_attempted"])
        self.assertFalse(any(c.startswith("FB_ARM,") for _, c in result["uart"].commands))
        self.assert_stopped(result)

    def real_settings(self):
        from experiment_a.settings import Settings
        settings = Settings()
        # Invented fixture approvals only; never written to a hardware config.
        settings.confirmations = {name: True for name in settings.confirmations}
        settings.metadata["parameter_selection"] = "offline_test_fixture_only"
        return settings

    def test_real_model_right_then_left_passes_commanded_neutral(self):
        class ReversingSocket(FakeSocket):
            def packet(self, **kwargs):
                self.ay = 3. if self.clock.elapsed_ns < 16_000_000_000 else -3.
                return super().packet(**kwargs)
        result = self.run_fixture(settings=self.real_settings(), real_policy=True,
                                  receiver_type=ReversingSocket, stop_after=40.)
        self.assertIsNone(result["error"])
        poses = [(ns, tuple(map(int, command.split(",")[3:])))
                 for ns, command in result["uart"].commands if command.startswith("FB_SET,")]
        self.assertEqual(max(pose[2] for _, pose in poses), 15000)
        self.assertEqual(max(pose[0] for _, pose in poses), 15000)
        self.assertTrue(any(pose[3] > 0 for _, pose in poses if pose[2] > 0))
        self.assertTrue(any(pose[1] > 0 for _, pose in poses if pose[0] > 0))
        self.assertTrue(all(not(pose[0] > 0 and pose[2] > 0) for _, pose in poses))
        first_left = next(i for i, (_, pose) in enumerate(poses) if pose[0] > 0)
        self.assertTrue(any(pose == (0, 0, 0, 0) for _, pose in poses[:first_left]))
        control = [event for event in result["events"] if event["event"] == "control"]
        self.assertTrue(any(event["decision"]["reason"] == "returning_neutral_before_side_change"
                            for event in control))
        self.assertTrue(all(event["decision"]["diag"]["pressure_used"] is False for event in control))
        self.assertTrue(all(event["decision"]["diag"]["observed_body_pose_used"] is False for event in control))
        self.assert_stopped(result)

    def test_real_model_pressure_exclusion_identical_commands(self):
        clean = self.run_fixture(settings=self.real_settings(), real_policy=True, stop_after=5.)
        noise = self.run_fixture(settings=self.real_settings(), real_policy=True, stop_after=5., noise=True)
        self.assertIsNone(clean["error"])
        self.assertIsNone(noise["error"])
        self.assertEqual(clean["uart"].commands, noise["uart"].commands)
        self.assert_stopped(clean)
        self.assert_stopped(noise)

    def test_committed_stm_settings_drive_controller_and_are_recorded(self):
        class TunedUART(MotorUART):
            def write(self,data):
                if data==b'TUNE_STATE\n':
                    self.commands.append((self.clock.now_ns,'TUNE_STATE'))
                    self.emit('#TUNE,OASIS_A,v1,revision=9,yaw_max_mdeg=12000,pitch_max_mdeg=3000,'
                        'yaw_rate_mdeg_s=1500,pitch_rate_mdeg_s=1000,full_scale_ay_mmps2=4000,pitch_scale_permille=250')
                    return len(data)
                return super().write(data)
        result=self.run_fixture(settings=self.real_settings(),real_policy=True,uart_type=TunedUART,
                                ay=6.,stop_after=22.)
        self.assertIsNone(result['error'])
        effective=result['manifest']['effective_settings']
        self.assertEqual((effective['max_yaw_deg'],effective['max_pitch_deg'],effective['max_yaw_rate_deg_s']),
                         (12.,3.,1.5))
        self.assertEqual(result['manifest']['firmware_tuning']['revision'],9)
        self.assertEqual(result['manifest']['firmware_identity']['servo_range_deg'],'270')
        self.assertTrue(any(c=='FB_ARM,1,9' for _,c in result['uart'].commands))
        poses=[tuple(map(int,c.split(',')[3:])) for _,c in result['uart'].commands if c.startswith('FB_SET,')]
        self.assertEqual(max(pose[2] for pose in poses),12000)
        self.assertLessEqual(max(abs(pose[3]) for pose in poses),3000)
        self.assertTrue(any(e['event']=='firmware_tuning_loaded' for e in result['events']))
        self.assert_stopped(result)

    def test_real_default_settings_block_before_uart_or_udp(self):
        from experiment_a.settings import Settings
        result = self.run_fixture(settings=Settings(), real_policy=True)
        self.assertIn("before opening any device", str(result["error"]))
        self.assertEqual(result["opens"], 0)
        self.assertIsNone(result["socket"].bound)


if __name__ == "__main__": unittest.main()
