"""Pi journal integration: temporary files + mock board only, no network/devices."""
import copy
import csv
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE.parent), str(HERE)]
from oasis_rt import app, controller as C, fbstack

FB = fbstack.load()
from oasis_rt import pressure_log

QUIET = lambda *args: None


def config(directory):
    cfg = copy.deepcopy(C.load_config(str(HERE.parent/'config.yaml')))
    cfg['control']['mode'] = 'fb_v6'
    cfg['log_dir'] = directory
    return cfg


def csv_rows(path):
    with path.open(newline='', encoding='utf-8') as handle:
        return list(csv.DictReader(handle))


class WrappedFile:
    def __init__(self, handle, fail=None, gate=None, entered=None):
        self.handle, self.fail, self.gate, self.entered = handle, fail, gate, entered

    def __getattr__(self, name):
        return getattr(self.handle, name)

    def write(self, value):
        if self.fail is not None and self.fail.is_set():
            raise OSError('invented disk full')
        if self.gate is not None:
            self.entered.set()
            if not self.gate.wait(5):
                raise OSError('test writer gate expired')
        return self.handle.write(value)


class AppPressureLoggingTests(unittest.TestCase):
    def test_accelerated_sil_waits_for_slow_writer_without_losing_records(self):
        class SlowFile(WrappedFile):
            def write(self, value):
                time.sleep(.001)  # Real writer time; the mock board clock runs much faster.
                return super().write(value)
        def opener(path, **kwargs):
            handle = path.open(**kwargs)
            return SlowFile(handle) if path.name == 'pressure.jsonl' else handle
        logger_type = pressure_log.PressureLogger
        with tempfile.TemporaryDirectory() as directory:
            # A smaller test queue makes starvation reproducible on fast PCs too.
            with patch.object(pressure_log, 'PressureLogger', side_effect=lambda output: logger_type(output, queue_limit=256, file_opener=opener)):
                man, session, dev = app.run(config(directory), 'sil', .5, log=QUIET)
            self.assertEqual(man['result']['result'], 'completed')
            self.assertTrue(man['result']['off_verified'])
            summary = man['pressure_logging']
            self.assertTrue(summary['complete'])
            self.assertEqual(summary['accepted_records'], summary['written_records'])
            self.assertEqual(summary['queue_pending'], 0)
            self.assertEqual(len(csv_rows(Path(session.out)/'pressure_raw.csv')), dev.p_seq)
            self.assertGreater(dev.p_seq, 1000)
            self.assertEqual(dev.lease_faults, 0)

    def test_all_samples_12_channels_and_shutdown_are_retained(self):
        with tempfile.TemporaryDirectory() as directory:
            man, session, dev = app.run(config(directory), 'sil', .1, log=QUIET)
            output = Path(session.out)
            raw = csv_rows(output/'pressure_raw.csv')
            sweeps = csv_rows(output/'pressure_sweeps.csv')
            records = [json.loads(line) for line in (output/'pressure.jsonl').read_text().splitlines()]
            self.assertEqual(man['result']['result'], 'completed')
            self.assertTrue(man['result']['off_verified'])
            self.assertEqual(len(raw), dev.p_seq)
            self.assertEqual(len(raw), len(sweeps)*4)
            self.assertEqual([int(row['seq']) for row in raw], list(range(1, dev.p_seq+1)))
            self.assertEqual({row['channel'] for row in raw}, {'0', '1', '2', '3'})
            first = sweeps[0]
            for bank in range(3):
                for channel in range(4):
                    sensor = f'A{bank}_C{channel}'
                    self.assertEqual(first[sensor+'_raw'], '1000')
                    self.assertEqual(first[sensor+'_status'], '0')
                    self.assertEqual(first[sensor+'_seq'], raw[channel]['seq'])
                    self.assertEqual(first[sensor+'_host_monotonic_ns'], raw[channel]['host_monotonic_ns'])
                    self.assertEqual(first[sensor+'_t_ms'], raw[channel]['t_ms'])
            self.assertIn('scan_disabled', [record.get('reason') for record in records])
            self.assertTrue(man['pressure_logging']['complete'])
            self.assertEqual(man['pressure_logging']['counts']['samples'], len(raw))
            self.assertEqual(man['pressure_logging']['accepted_records'], man['pressure_logging']['written_records'])
            self.assertFalse((output/'INCOMPLETE').exists())

    def test_sil_writer_barrier_timeout_stops_without_blocking_off_verification(self):
        gate, entered = threading.Event(), threading.Event()
        gate.set()
        def opener(path, **kwargs):
            handle = path.open(**kwargs)
            return WrappedFile(handle, gate=gate, entered=entered) if path.name == 'pressure.jsonl' else handle
        logger_type = pressure_log.PressureLogger
        with tempfile.TemporaryDirectory() as directory:
            with patch.object(pressure_log, 'PressureLogger', side_effect=lambda output: logger_type(output, close_timeout_s=.5, file_opener=opener)):
                _, session, dev = app.assemble(config(directory), 'sil', QUIET)
            def stall_during_control(sess, _duration):
                gate.clear()
                sess.wait(.5)
            session.policy.run = stall_during_control
            close = session.link.close
            def release_then_close(wait):
                gate.set()
                return close(wait)
            session.link.close = release_then_close
            try:
                result = session.run(.5, startup_timeout_s=5)
            finally:
                gate.set()
            self.assertIn('log_flush_timeout', result['logging_error'])
            self.assertTrue(result['off_verified'])
            self.assertFalse(dev.armed)
            self.assertEqual(sum(cmd.startswith('FB_ARM') for _, cmd in dev.commands), 1)
            self.assertTrue((Path(session.out)/'INCOMPLETE').exists())

    def test_invalid_adc_samples_remain_raw_and_do_not_change_fb_control(self):
        with tempfile.TemporaryDirectory() as directory:
            _, session, dev = app.assemble(config(directory), 'sil', QUIET)
            emit = dev.emit
            def invalid_adc(line):
                fields = line.split(',')
                if fields[0].isdigit() and fields[2] == '2':
                    fields[5:7] = ['', '2']
                    line = ','.join(fields)
                emit(line)
            dev.emit = invalid_adc
            result = session.run(.1, startup_timeout_s=5)
            raw = csv_rows(Path(session.out)/'pressure_raw.csv')
            invalid = [row for row in raw if row['channel'] == '2']
            self.assertEqual(result['result'], 'completed')
            self.assertTrue(invalid)
            self.assertTrue(all(row['raw1'] == '' and row['status1'] == '2' and row['valid'] == 'False' for row in invalid))
            self.assertTrue(all(row['voltage1_v'] == '' and ',,2,' in row['raw_line'] for row in invalid))

    def test_same_timestamp_runs_cannot_overwrite_previous_logs(self):
        with tempfile.TemporaryDirectory() as directory, patch.object(app.time, 'strftime', return_value='fixed'):
            _, first, _ = app.run(config(directory), 'sil', .02, log=QUIET)
            original = {path.name: path.read_bytes() for path in Path(first.out).iterdir()}
            _, second, _ = app.run(config(directory), 'sil', .02, log=QUIET)
            self.assertNotEqual(first.out, second.out)
            self.assertEqual(original, {path.name: path.read_bytes() for path in Path(first.out).iterdir()})

    def test_log_setup_failure_opens_no_hardware_or_udp(self):
        with tempfile.TemporaryDirectory() as directory:
            with patch.object(pressure_log, 'PressureLogger', side_effect=OSError('invented read-only disk')):
                with patch.object(app, 'RelayReceiver') as receiver, patch.object(FB.serial_link, 'open_serial') as serial:
                    with self.assertRaisesRegex(OSError, 'read-only'):
                        app.assemble(config(directory), 'hw', QUIET)
                    receiver.assert_not_called()
                    serial.assert_not_called()

    def test_disk_failure_after_arm_stops_and_keeps_incomplete_marker(self):
        fail = threading.Event()
        def opener(path, **kwargs):
            handle = path.open(**kwargs)
            return WrappedFile(handle, fail=fail) if path.name == 'pressure_raw.csv' else handle
        logger_type = pressure_log.PressureLogger
        with tempfile.TemporaryDirectory() as directory:
            with patch.object(pressure_log, 'PressureLogger', side_effect=lambda output: logger_type(output, file_opener=opener)):
                _, session, dev = app.assemble(config(directory), 'sil', QUIET)
            journal = session.link.pressure_log
            def fail_during_control(sess, _duration):
                fail.set()
                sess.wait(.02)
                self.assertTrue(journal._done.wait(1), 'writer did not encounter injected disk failure')
                before = len(dev.commands)
                sess.pump()
                sess.link.authorize()
                sess.link.service()
                self.assertFalse(sess.link.arm())
                self.assertEqual(len(dev.commands), before, 'failed logger allowed motor authority')
            session.policy.run = fail_during_control
            result = session.run(.1, startup_timeout_s=5)
            self.assertIn('pressure_logging:', result['result'])
            self.assertIn('disk full', result['logging_error'])
            self.assertTrue(result['off_verified'])
            self.assertFalse(dev.armed)
            self.assertEqual(sum(cmd.startswith('FB_ARM') for _, cmd in dev.commands), 1)
            self.assertTrue((Path(session.out)/'INCOMPLETE').exists())
            self.assertFalse(journal.summary()['complete'])

    def test_writer_queue_overflow_is_bounded_and_prevents_arm(self):
        gate, entered = threading.Event(), threading.Event()
        def opener(path, **kwargs):
            handle = path.open(**kwargs)
            return WrappedFile(handle, gate=gate, entered=entered) if path.name == 'pressure_raw.csv' else handle
        logger_type = pressure_log.PressureLogger
        with tempfile.TemporaryDirectory() as directory:
            with patch.object(pressure_log, 'PressureLogger', side_effect=lambda output: logger_type(output, queue_limit=2, file_opener=opener)):
                _, session, dev = app.assemble(config(directory), 'sil', QUIET)
            self.assertTrue(entered.wait(1))
            close = session.link.close
            def release_then_close(wait):
                gate.set()
                return close(wait)
            session.link.close = release_then_close
            try:
                result = session.run(.1, startup_timeout_s=5)
            finally:
                gate.set()
            self.assertIn('log_queue_overflow', result['result'])
            self.assertFalse(any(cmd.startswith('FB_ARM') for _, cmd in dev.commands))
            summary = session.link.pressure_log.summary()
            self.assertEqual(summary['accepted_records'], 2)
            self.assertEqual(summary['written_records'], 2)
            self.assertFalse(summary['writer_alive'])
            self.assertTrue((Path(session.out)/'INCOMPLETE').exists())

    def test_acquisition_queue_loss_is_logged_and_prevents_arm(self):
        with tempfile.TemporaryDirectory() as directory:
            _, session, dev = app.assemble(config(directory), 'sil', QUIET)
            session.link.b._pressure.queue_limit = 4
            result = session.run(.1, startup_timeout_s=5)
            self.assertIn('pressure_record_queue_overflow', result['result'])
            self.assertFalse(any(cmd.startswith('FB_ARM') for _, cmd in dev.commands))
            output = Path(session.out)
            self.assertIn('record_queue_overflow', (output/'pressure.jsonl').read_text())
            self.assertTrue((output/'INCOMPLETE').exists())
            self.assertFalse(session.link.pressure_log.summary()['complete'])


if __name__ == '__main__':
    unittest.main()
