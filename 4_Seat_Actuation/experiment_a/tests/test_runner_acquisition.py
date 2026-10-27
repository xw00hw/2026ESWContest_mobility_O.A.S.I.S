"""Automatic-session orchestration and pressure collection, all I/O injected.

No UART/socket/game is opened. The board fixture is command telemetry, not a
physical actuator simulation. Actual model settings are approved only inside
this temporary test process and are never saved to a hardware configuration.
"""
from __future__ import annotations

import contextlib
import csv
import io
import json
from pathlib import Path
import socket
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from experiment_a import runner, transport
from experiment_a.settings import Settings
from experiment_a.vehicle import BridgeReceiver
from experiment_a.tests.runtime_fixtures import FakeClock, FakeSocket
from experiment_a.tests.test_pressure_logging import PressureUART


class YieldingClock(FakeClock):
    def sleep(self, seconds):
        super().sleep(seconds)
        # Let the real bounded file-writer thread keep pace with accelerated
        # fake time. This does not sleep on any actual device/network.
        time.sleep(0)


class TimedStop:
    def __init__(self, clock, stop_after):
        self.clock, self.stop_ns = clock, round(stop_after*1e9)

    def is_set(self):
        return self.clock.elapsed_ns >= self.stop_ns


class ScanningUART(PressureUART):
    def __init__(self, clock, *, high_pressure=False, pressure_gap=False):
        super().__init__(clock)
        self.scan = False
        self.sequence = 0
        self.next_sample_ns = clock.now_ns
        self.high_pressure = high_pressure
        self.pressure_gap = pressure_gap
        self.emitted_pressure = []

    def write(self, data):
        for command in data.decode("ascii").splitlines():
            if command == "START_12,15":
                self.scan = True
                self.next_sample_ns = self.clock.now_ns
            elif command == "DISABLE":
                self.scan = False
        return super().write(data)

    @property
    def in_waiting(self):
        super().in_waiting  # Generate the fixture's ordinary board STATE.
        while self.scan and self.next_sample_ns <= self.clock.now_ns:
            channel = self.sequence % 4
            raw = 3500+(self.sequence%300) if self.high_pressure else 100+(self.sequence%300)
            line = f"{self.sequence},{self.next_sample_ns//1_000_000},{channel},{raw},0,{raw+1},0,{raw+2},0"
            if self.pressure_gap and self.sequence == 500:
                line = "500,broken,pressure,row"
            # One omitted wire row gives a source gap, independent of values.
            if not (self.pressure_gap and self.sequence == 700):
                self.emit(line)
                self.emitted_pressure.append(line)
            self.sequence += 1
            self.next_sample_ns += 2_500_000
        return len(self.buffer)


class AcquisitionRunnerTests(unittest.TestCase):
    def run_fixture(self, *, high_pressure=False, pressure_gap=False,
                    beamng_delay=0., source_stop=None, stop_after=4., startup_timeout_s=None,
                    receiver_type=FakeSocket, uart_type=ScanningUART):
        clock = YieldingClock(stop_after+10.)
        uart = uart_type(clock, high_pressure=high_pressure, pressure_gap=pressure_gap)
        udp = receiver_type(clock, source_stop=source_stop, ay=2.)
        udp.next_ns = clock.now_ns+round(beamng_delay*1e9)
        settings = Settings()
        settings.confirmations = {key: True for key in settings.confirmations}
        settings.metadata["parameter_selection"] = "offline_test_fixture_only"
        notifications, error = [], None
        with tempfile.TemporaryDirectory() as temp:
            with patch.object(runner, "time", clock), \
                 patch.object(runner, "BridgeReceiver", lambda: BridgeReceiver(socket_factory=lambda *args: udp)), \
                 patch.object(transport, "open_serial", return_value=uart), \
                 patch.object(socket, "socket", side_effect=AssertionError("Real socket forbidden")), \
                 contextlib.redirect_stdout(io.StringIO()):
                try:
                    runner.run_live("OFFLINE_ONLY", temp, settings,
                        record_pressure=True, stop_event=TimedStop(clock, stop_after),
                        startup_timeout_s=startup_timeout_s,
                        status_callback=lambda kind, payload: notifications.append((kind, payload)))
                except Exception as exc:
                    error = exc
            output = Path(temp)
            manifest = json.loads((output/"manifest.json").read_text())
            events = [json.loads(line) for line in (output/"control.jsonl").read_text().splitlines()]
            pressure = [json.loads(line) for line in (output/"pressure.jsonl").read_text().splitlines()]
            with (output/"pressure_raw.csv").open(newline="", encoding="utf-8") as handle:
                raw = list(csv.DictReader(handle))
            with (output/"pressure_sweeps.csv").open(newline="", encoding="utf-8") as handle:
                sweeps = list(csv.DictReader(handle))
            incomplete = (output/"INCOMPLETE").exists()
        return dict(clock=clock, uart=uart, udp=udp, error=error, manifest=manifest,
                    events=events, pressure=pressure, raw=raw, sweeps=sweeps,
                    notifications=notifications, incomplete=incomplete)

    def test_auto_arm_and_both_logs_start_then_stop_off_and_disable(self):
        result = self.run_fixture()
        self.assertIsNone(result["error"])
        self.assertEqual(result["manifest"]["result"], "stop_requested")
        self.assertTrue(result["manifest"]["pwm_off_telemetry_verified"])
        self.assertFalse(result["manifest"]["pressure_summary"]["scan_active"])
        self.assertFalse(result["uart"].scan)
        self.assertTrue(result["uart"].closed and result["udp"].closed)
        self.assertFalse(result["incomplete"])
        commands = result["uart"].commands
        arms = [(ns, command) for ns, command in commands if command.startswith("FB_ARM,")]
        self.assertEqual(len(arms), 1)
        self.assertGreaterEqual(arms[0][0]-result["clock"].started_ns, 2_000_000_000)
        self.assertTrue(any(command.startswith("FB_SET,") for _, command in commands))
        samples = [row for row in result["pressure"] if row["type"] == "sample"]
        self.assertLess(samples[0]["host_monotonic_ns"], arms[0][0])
        self.assertEqual(len(samples), len(result["raw"]))
        self.assertEqual(len(samples), result["manifest"]["pressure_summary"]["samples"])
        self.assertEqual([row["raw_line"] for row in samples], result["uart"].emitted_pressure)
        self.assertTrue(result["sweeps"])
        for bank in range(3):
            for channel in range(4):
                self.assertIn(f"A{bank}_C{channel}_raw", result["sweeps"][0])
        self.assertTrue(any(kind == "CONTROL" for kind, _ in result["notifications"]))
        self.assertEqual(result["notifications"][-1][0], "ended")
        ended=result['notifications'][-1][1]
        self.assertTrue(ended['off_verified'])
        self.assertEqual(json.loads(json.dumps(ended['board'])),result['manifest']['final_board_state'])
        self.assertFalse(ended['board']['armed'])
        self.assertEqual(ended['board']['pulses_us'],(0,0,0,0))
        self.assertEqual(ended['pressure'],result['manifest']['pressure_summary'])
        self.assertEqual(ended['pressure']['samples'],len(samples))
        self.assertTrue(ended['pressure']['scan_disabled_verified'])

    def test_pressure_values_never_change_real_model_motor_commands(self):
        low, high = self.run_fixture(), self.run_fixture(high_pressure=True)
        self.assertIsNone(low["error"])
        self.assertIsNone(high["error"])
        self.assertEqual(low["uart"].commands, high["uart"].commands)
        self.assertNotEqual(low["raw"][0]["raw0"], high["raw"][0]["raw0"])
        controls = [row for row in high["events"] if row["event"] == "control"]
        self.assertTrue(controls)
        self.assertTrue(all(row["decision"]["diag"]["pressure_used"] is False for row in controls))

    def test_malformed_and_missing_pressure_are_persisted_without_policy_change(self):
        clean, gaps = self.run_fixture(), self.run_fixture(pressure_gap=True)
        self.assertIsNone(gaps["error"])
        self.assertEqual(clean["uart"].commands, gaps["uart"].commands)
        self.assertTrue(any(row.get("reason") == "malformed_sample" for row in gaps["pressure"]))
        self.assertEqual(gaps["manifest"]["pressure_summary"]["malformed"], 1)
        self.assertGreaterEqual(gaps["manifest"]["pressure_summary"]["missing_samples"], 1)
        self.assertEqual(gaps["manifest"]["pressure_summary"]["queue_dropped"], 0)

    def test_auto_wait_beyond_thirty_seconds_keeps_pressure_alive_and_arms_once(self):
        result = self.run_fixture(beamng_delay=31., stop_after=34.)
        self.assertIsNone(result["error"])
        commands = result["uart"].commands
        arms = [(ns, c) for ns, c in commands if c.startswith("FB_ARM,")]
        self.assertEqual(len(arms), 1)
        self.assertGreaterEqual(arms[0][0]-result["clock"].started_ns, 33_000_000_000)
        pings_before_arm = [ns for ns, c in commands if c == "PING" and ns < arms[0][0]]
        self.assertGreaterEqual(len(pings_before_arm), 30)
        self.assertGreater(len(result["raw"]), 12000)
        self.assertEqual(result["manifest"]["pressure_summary"]["queue_dropped"], 0)

    def test_vehicle_loss_after_auto_arm_stops_and_never_rearms(self):
        result = self.run_fixture(source_stop=2.8, stop_after=5.)
        self.assertIn("vehicle_stale", str(result["error"]))
        commands = [c for _, c in result["uart"].commands]
        self.assertEqual(sum(c.startswith("FB_ARM,") for c in commands), 1)
        off_index = [i for i, c in enumerate(commands) if c == "FB_OFF"][1]
        self.assertFalse(any(c.startswith(("FB_ARM,", "FB_SET,", "FB_PING,")) for c in commands[off_index+1:]))
        self.assertTrue(result["manifest"]["pwm_off_telemetry_verified"])
        self.assertFalse(result["uart"].scan)
        ended=result['notifications'][-1][1]
        self.assertFalse(ended['board']['armed'])
        self.assertEqual(ended['board']['pulses_us'],(0,0,0,0))
        self.assertFalse(ended['pressure']['scan_active'])

    def test_impact_only_stops_motor_and_keeps_pressure_and_vehicle_until_operator_stop(self):
        class ImpactThenInvalidSocket(FakeSocket):
            def packet(self, **kwargs):
                payload, peer = super().packet(**kwargs)
                packet = json.loads(payload)
                if 2_800_000_000 <= self.clock.elapsed_ns < 3_000_000_000:
                    packet['row'].update(linear_acc_x_mps2=-40., linear_acc_y_mps2=10.)
                elif self.clock.elapsed_ns >= 3_000_000_000:
                    packet['row'].update(derivative_valid=0, derivative_reason='post_impact_gap')
                return json.dumps(packet).encode('ascii'), peer
        result = self.run_fixture(receiver_type=ImpactThenInvalidSocket,
                                  source_stop=3.4, stop_after=5.)
        self.assertIsNone(result['error'])
        self.assertEqual(result['manifest']['result'], 'stop_requested')
        impact = result['manifest']['impact']
        self.assertEqual(impact['state']['reason'], 'impact_suspected_sustained')
        self.assertTrue(impact['motor_off_verified'])
        self.assertTrue(any(kind == 'RECORD_ONLY' for kind, _ in result['notifications']))
        self.assertEqual(sum(row['event'] == 'impact' for row in result['events']), 1)
        stopped_ns = impact['motor_off_requested_host_monotonic_ns']
        commands = result['uart'].commands
        self.assertEqual(sum(c.startswith('FB_ARM,') for _, c in commands), 1)
        self.assertFalse(any(ns >= stopped_ns and c.startswith(('FB_ARM,', 'FB_SET,', 'FB_PING,'))
                             for ns, c in commands))
        post_impact_vehicle = [e for e in result['events'] if e['event'] == 'vehicle'
                               and e['host_monotonic_ns'] > stopped_ns]
        self.assertTrue(any(e['frame']['reason'] == 'invalid_derivative' for e in post_impact_vehicle))
        self.assertGreater(int(result['raw'][-1]['host_monotonic_ns']),
                           result['clock'].started_ns+4_900_000_000)
        self.assertGreater(int(result['sweeps'][-1]['completed_host_monotonic_ns']),
                           result['clock'].started_ns+4_900_000_000)
        self.assertTrue(any(ns > stopped_ns+1_000_000_000 and c == 'PING' for ns, c in commands))
        disables = [ns for ns, c in commands if c == 'DISABLE' and ns > stopped_ns]
        self.assertTrue(disables)
        self.assertGreaterEqual(min(disables), result['clock'].started_ns+5_000_000_000)
        self.assertTrue(result['manifest']['pwm_off_telemetry_verified'])
        self.assertTrue(result['manifest']['pressure_summary']['scan_disabled_verified'])
        self.assertFalse(result['incomplete'])

    def test_uart_failure_after_impact_remains_fatal(self):
        class ExtremeSocket(FakeSocket):
            def packet(self, **kwargs):
                payload, peer = super().packet(**kwargs)
                packet = json.loads(payload)
                if self.clock.elapsed_ns >= 2_800_000_000:
                    packet['row']['linear_acc_y_mps2'] = 65.
                return json.dumps(packet).encode('ascii'), peer
        class RestartedUART(ScanningUART):
            injected = False
            @property
            def in_waiting(self):
                count = super().in_waiting
                if self.clock.elapsed_ns >= 3_300_000_000 and not self.injected:
                    self.injected = True
                    self.emit('#BOOT,reset')
                    count = len(self.buffer)
                return count
        result = self.run_fixture(receiver_type=ExtremeSocket, uart_type=RestartedUART, stop_after=5.)
        self.assertIn('MCU reboot', str(result['error']))
        self.assertTrue(result['manifest']['impact']['motor_off_verified'])
        self.assertLess(result['clock'].elapsed_ns, 5_000_000_000)
        self.assertEqual(sum(c.startswith('FB_ARM,') for _, c in result['uart'].commands), 1)


if __name__ == "__main__":
    unittest.main()
