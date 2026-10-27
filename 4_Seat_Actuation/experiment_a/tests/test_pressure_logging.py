"""Observation-only pressure recording and shared-UART protocol tests."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from experiment_a.pressure_logging import PressureClockReset, PressureRecorder
from experiment_a.transport import BoardLink, FSR_HELLO
from experiment_a.tests.runtime_fixtures import FakeClock, MotorUART


def sample(seq, tick, channel, raw=1000):
    return f"{seq},{tick},{channel},{raw},0,{raw+1},0,{raw+2},0"


class PressureUART(MotorUART):
    def write(self, data):
        if data in (b"HELLO\n", b"START_12,15\n", b"PING\n"):
            command = data.decode("ascii").strip()
            self.commands.append((self.clock.now_ns, command))
            if not self.silent:
                self.emit({"HELLO": FSR_HELLO, "START_12,15": "#CONTROL,12,MASK=15,EN=0",
                           "PING": "#PONG,12"}[command])
            return len(data)
        return super().write(data)


class PressureRecordsTests(unittest.TestCase):
    def test_twelve_raw_values_sweep_has_original_timestamps(self):
        recorder = PressureRecorder()
        for channel in range(4):
            recorder.sample(sample(channel, 100+channel*3, channel, 1000+10*channel), 1_000_000_000+channel*3_000_000)
        rows = recorder.drain()
        self.assertEqual([r["type"] for r in rows], ["sample"]*4+["sweep"])
        sweep = rows[-1]
        self.assertTrue(sweep["valid"])
        self.assertEqual(len(sweep["values"]), 12)
        self.assertEqual(sweep["host_monotonic_ns"], 1_000_000_000)
        self.assertEqual(sweep["completed_host_monotonic_ns"], 1_009_000_000)
        self.assertEqual(sweep["values"]["A1_C3"]["raw"], 1031)
        self.assertEqual(sweep["values"]["A1_C3"]["t_ms"], 109)
        self.assertAlmostEqual(sweep["values"]["A1_C3"]["voltage_v"], 1031*3.3/4095)

    def test_sequence_loss_not_filled_with_retained_channels(self):
        recorder = PressureRecorder()
        recorder.sample(sample(0, 100, 0), 1_000_000_000)
        recorder.sample(sample(2, 106, 2), 1_006_000_000)
        recorder.sample(sample(3, 109, 3), 1_009_000_000)
        self.assertFalse(any(r["type"] == "sweep" for r in recorder.drain()))
        self.assertEqual(recorder.status(1_009_000_000)["missing_samples"], 1)
        for channel in range(4):
            recorder.sample(sample(4+channel, 112+channel*3, channel), 1_012_000_000+channel*3_000_000)
        self.assertTrue(recorder.drain()[-1]["valid"])

    def test_adc_invalid_raw_and_malformed_sample_preserved(self):
        recorder = PressureRecorder()
        text = "0,100,0,,2,1001,0,1002,0"
        recorder.sample(text, 1_000_000_000)
        row = recorder.drain()[0]
        self.assertEqual(row["raw_line"], text)
        self.assertFalse(row["valid"])
        self.assertIsNone(row["raw0"])
        self.assertEqual(row["status0"], 2)
        self.assertIsNone(row["voltage0_v"])
        recorder.sample("garbage,pressure,row", 1_001_000_000)
        rows = recorder.drain()
        self.assertEqual(rows[-1]["raw_line"], "garbage,pressure,row")
        self.assertEqual(rows[-1]["reason"], "malformed_sample")

    def test_clock_wrap_supported_but_backwards_tick_is_board_reset(self):
        recorder = PressureRecorder()
        recorder.sample(sample(0xffffffff, 0xfffffffe, 3), 1_000_000_000)
        recorder.sample(sample(0, 1, 0), 1_003_000_000)
        self.assertEqual(recorder.counters["missing_samples"], 0)
        self.assertEqual(recorder.last_reason, "ok")
        with self.assertRaises(PressureClockReset):
            recorder.sample(sample(1, 0, 1), 1_006_000_000)
        self.assertEqual(recorder.drain()[-1]["reason"], "mcu_clock_regression")

    def test_host_delay_marks_quality_without_raising(self):
        recorder = PressureRecorder()
        recorder.sample(sample(0, 100, 0), 1_000_000_000)
        recorder.sample(sample(1, 103, 1), 1_150_000_000)
        record = next(row for row in recorder.drain() if row.get("seq") == 1 and row["type"] == "sample")
        self.assertFalse(record["valid"])
        self.assertIn("transport_delay", record["reason"])

    def test_queue_is_bounded_and_dropped_records_explicit(self):
        recorder = PressureRecorder(4)
        for index in range(10):
            recorder.event(index, "fixture")
        rows = recorder.drain()
        self.assertEqual(len(rows), 5)
        self.assertEqual(rows[0]["reason"], "record_queue_overflow")
        self.assertEqual(rows[0]["dropped_records"], 6)
        self.assertEqual(rows[0]["first_dropped_host_monotonic_ns"], 0)
        self.assertEqual(rows[0]["last_dropped_host_monotonic_ns"], 5)
        self.assertEqual(recorder.drain(), [])
        self.assertEqual(recorder.counters["queue_dropped"], 6)


class PressureTransportTests(unittest.TestCase):
    def setup_link(self):
        clock = FakeClock(100.)
        uart = PressureUART(clock)
        link = BoardLink(uart, record_pressure=True)
        link.start(clock.now_ns)
        for _ in range(12):
            clock.now_ns += 2_000_000
            link.poll(clock.now_ns)
        self.assertIsNone(link.fault)
        self.assertTrue(link.ready)
        return clock, uart, link

    def test_opt_in_scanner_handshake_keeps_motor_off(self):
        clock, uart, link = self.setup_link()
        self.assertEqual([c for _, c in uart.commands],
            ["CONTROL_HELLO", "FB_OFF", "CAD_INFO", "TUNE_STATE", "DISABLE", "HELLO", "START_12,15", "STATE"])
        self.assertFalse(link.state.armed)
        self.assertTrue(link.pressure_status(clock.now_ns)["scan_active"])
        self.assertFalse(link.pressure_status(clock.now_ns)["scan_disabled_verified"])
        # No valid pressure sample is required to ARM Experiment A.
        self.assertTrue(link.arm(clock.now_ns))
        clock.now_ns += 2_000_000
        link.poll(clock.now_ns)
        self.assertEqual(link.armed_session, 1)

    def test_pressure_ping_schedule_is_independent_of_motor_lease(self):
        clock, uart, link = self.setup_link()
        self.assertFalse(link.pressure_ping(clock.now_ns))
        for _ in range(11):
            clock.now_ns += 100_000_000
            link.poll(clock.now_ns)
        self.assertTrue(link.pressure_ping(clock.now_ns))
        self.assertEqual(link.pending.command, "PING")
        count = len(uart.commands)
        self.assertFalse(link.request_state(clock.now_ns))
        self.assertEqual(count, len(uart.commands))
        clock.now_ns += 2_000_000
        link.poll(clock.now_ns)
        self.assertIsNone(link.pending)
        self.assertFalse(link.pressure_ping(clock.now_ns))
        self.assertIsNone(link.armed_session)
        self.assertFalse(any(c.startswith("FB_PING") for _, c in uart.commands))

    def test_motor_only_off_keeps_scanner_watchdog_and_final_stop_disables_it(self):
        clock, uart, link = self.setup_link()
        link.arm(clock.now_ns)
        clock.now_ns += 2_000_000
        link.poll(clock.now_ns)
        before = len(uart.commands)
        link.motor_off_keep_recording(clock.now_ns)
        for _ in range(4):
            clock.now_ns += 2_000_000
            link.poll(clock.now_ns)
        self.assertTrue(link.motor_off_verified)
        self.assertTrue(link.pressure_status(clock.now_ns)['scan_active'])
        self.assertNotIn('DISABLE', [c for _, c in uart.commands[before:]])
        for _ in range(11):
            clock.now_ns += 100_000_000
            link.poll(clock.now_ns)
        self.assertTrue(link.pressure_ping(clock.now_ns))
        clock.now_ns += 2_000_000
        link.poll(clock.now_ns)
        self.assertEqual(link.last_ack['command'], 'PING')
        link.off()
        for _ in range(5):
            clock.now_ns += 2_000_000
            link.poll(clock.now_ns)
        self.assertFalse(link.pressure_status(clock.now_ns)['scan_active'])
        self.assertTrue(link.pressure_status(clock.now_ns)['scan_disabled_verified'])

    def test_malformed_samples_and_gap_are_logs_not_motor_inputs(self):
        clock, uart, link = self.setup_link()
        for line in (sample(0, 100, 0), "2,nan,broken,pressure", sample(4, 112, 0),
                     "#STATS,adc_errors=2,skipped_cycles=1,overruns=2"):
            uart.emit(line)
        link.poll(clock.now_ns)
        self.assertIsNone(link.fault)
        self.assertEqual(link.pressure_status(clock.now_ns)["missing_samples"], 3)
        records = link.drain_pressure_records()
        self.assertTrue(any(r.get("reason") == "malformed_sample" for r in records))
        self.assertTrue(link.arm(clock.now_ns))

    def test_mismatched_motor_ack_cannot_be_swallowed_by_pressure(self):
        clock, uart, link = self.setup_link()
        link.arm(clock.now_ns)
        uart.buffer.clear()
        uart.emit(sample(0, 100, 0))
        uart.emit("#ACK,FB_ARM,999")
        link.poll(clock.now_ns)
        self.assertIn("mismatched", link.fault)
        self.assertIsNone(link.armed_session)

    def test_uart_stats_and_reset_still_latch_motor_off(self):
        for lines in (("#STATS,uart_errors=1",), (sample(0, 100, 0), sample(1, 99, 1)),
                      ("#CONTROL_ERROR,ADC",), ("#BOOT,FSR_12_SCAN,v10",)):
            with self.subTest(lines=lines):
                clock, uart, link = self.setup_link()
                for line in lines:
                    uart.emit(line)
                link.poll(clock.now_ns)
                self.assertIsNotNone(link.fault)
                self.assertFalse(link.pressure_status(clock.now_ns)["scan_disabled_verified"])
                self.assertTrue(any(r.get("reason") == "transport_fault" for r in link.drain_pressure_records()))

    def test_off_preempts_pressure_ping_and_verifies_disabled(self):
        clock, uart, link = self.setup_link()
        for _ in range(11):
            clock.now_ns += 100_000_000
            link.poll(clock.now_ns)
        link.pressure_ping(clock.now_ns)
        link.off()
        for _ in range(4):
            clock.now_ns += 2_000_000
            link.poll(clock.now_ns)
        self.assertIsNone(link.fault)
        self.assertFalse(link.state.armed)
        self.assertFalse(link.pressure_status(clock.now_ns)["scan_active"])
        self.assertTrue(link.pressure_status(clock.now_ns)["scan_disabled_verified"])


if __name__ == "__main__":
    unittest.main()
