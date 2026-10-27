"""Real temporary files and invented records; no sockets, UART or motors."""
from __future__ import annotations

import csv
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest

from experiment_a.pressure_logging import PressureRecorder
from experiment_a.session_logging import SessionLogger, SessionLogError


class WrappedFile:
    def __init__(self, handle, *, fail=False, gate=None, entered=None):
        self.handle, self.fail, self.gate, self.entered = handle, fail, gate, entered

    def __getattr__(self, name):
        return getattr(self.handle, name)

    def write(self, value):
        if self.fail:
            raise OSError("invented disk full")
        if self.gate is not None:
            self.entered.set()
            self.gate.wait(2)
        return self.handle.write(value)


class SessionLoggingTests(unittest.TestCase):
    def wait_failed(self, logger):
        deadline = time.monotonic()+1
        while time.monotonic() < deadline:
            try:
                logger.check()
            except SessionLogError as exc:
                return str(exc)
            time.sleep(.002)
        self.fail("logger did not expose injected failure")

    def test_400_rows_and_100_sweeps_preserve_raw_status_clocks_and_csv(self):
        recorder = PressureRecorder()
        for seq in range(400):
            channel = seq % 4
            tick = 1000 + (seq*5)//2
            recorder.sample(f"{seq},{tick},{channel},100,0,200,0,300,0", 5_000_000_000+seq*2_500_000)
        records = recorder.drain()
        with tempfile.TemporaryDirectory() as directory:
            logger = SessionLogger(directory, pressure_enabled=True)
            payload = dict(sequence_id=7, host_monotonic_ns=123)
            logger.record("vehicle", host_monotonic_ns=555, frame=payload)
            payload["sequence_id"] = 999  # Queued data must not mutate later.
            logger.pressure(records)
            summary = logger.close()
            self.assertTrue(summary["complete"])
            self.assertEqual(summary["counts"], dict(control=1, pressure=500, samples=400, sweeps=100))
            self.assertEqual(summary["accepted_records"], summary["written_records"])
            self.assertFalse((Path(directory)/"INCOMPLETE").exists())
            self.assertEqual(json.loads((Path(directory)/"control.jsonl").read_text())["frame"]["sequence_id"], 7)
            original = [json.loads(line) for line in (Path(directory)/"pressure.jsonl").read_text().splitlines()]
            self.assertEqual(original, records)
            with (Path(directory)/"pressure_raw.csv").open(newline="") as handle:
                raw = list(csv.DictReader(handle))
            with (Path(directory)/"pressure_sweeps.csv").open(newline="") as handle:
                sweeps = list(csv.DictReader(handle))
            self.assertEqual(len(raw), 400)
            self.assertEqual(len(sweeps), 100)
            self.assertEqual(raw[0]["raw_line"], "0,1000,0,100,0,200,0,300,0")
            self.assertEqual(raw[0]["status1"], "0")
            self.assertEqual(sweeps[0]["A1_C3_raw"], "200")
            self.assertEqual(sweeps[0]["A1_C3_t_ms"], "1007")
            self.assertEqual(sweeps[0]["A1_C0_host_monotonic_ns"], "5000000000")
            self.assertEqual(sweeps[0]["A1_C3_host_monotonic_ns"], "5007500000")
            self.assertEqual(logger.close(), summary)

    def test_disabled_pressure_creates_only_control_and_invalid_voltage_not_zero(self):
        with tempfile.TemporaryDirectory() as directory:
            logger = SessionLogger(directory)
            logger.record("start", host_monotonic_ns=0)
            logger.pressure([])
            logger.close()
            self.assertEqual({p.name for p in Path(directory).iterdir()}, {"control.jsonl"})
        recorder = PressureRecorder()
        recorder.sample("1,1,0,100,0,,2,300,0", 1000)
        with tempfile.TemporaryDirectory() as directory:
            logger = SessionLogger(directory, pressure_enabled=True)
            logger.pressure(recorder.drain())
            logger.close()
            with (Path(directory)/"pressure_raw.csv").open(newline="") as handle:
                raw = list(csv.DictReader(handle))
            self.assertEqual(raw[0]["raw1"], "")
            self.assertEqual(raw[0]["voltage1_v"], "")
            self.assertEqual(raw[0]["status1"], "2")
            self.assertEqual(raw[0]["valid"], "False")

    def test_existing_files_never_overwritten(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"pressure_raw.csv"
            path.write_text("previous session", encoding="utf-8")
            with self.assertRaises(FileExistsError):
                SessionLogger(directory, pressure_enabled=True)
            self.assertEqual(path.read_text(), "previous session")
            self.assertFalse((Path(directory)/"control.jsonl").exists())

    def test_disk_failure_exposed_and_marker_retained(self):
        def opener(path, **kwargs):
            return WrappedFile(path.open(**kwargs), fail=path.name == "control.jsonl")
        with tempfile.TemporaryDirectory() as directory:
            logger = SessionLogger(directory, file_opener=opener)
            logger.record("before_failure", host_monotonic_ns=1)
            self.assertIn("disk full", self.wait_failed(logger))
            with self.assertRaises(SessionLogError):
                logger.close()
            self.assertTrue((Path(directory)/"INCOMPLETE").exists())
            self.assertEqual(logger.summary()["written_records"], 0)

    def test_queue_overflow_is_bounded_latched_and_never_silent(self):
        gate, entered = threading.Event(), threading.Event()
        def opener(path, **kwargs):
            return WrappedFile(path.open(**kwargs), gate=gate, entered=entered)
        with tempfile.TemporaryDirectory() as directory:
            logger = SessionLogger(directory, queue_limit=2, file_opener=opener)
            try:
                logger.record("writing", host_monotonic_ns=1)
                self.assertTrue(entered.wait(1))
                logger.record("queued1", host_monotonic_ns=2)
                logger.record("queued2", host_monotonic_ns=3)
                with self.assertRaisesRegex(SessionLogError, "queue_overflow"):
                    logger.record("rejected", host_monotonic_ns=4)
                self.assertEqual(logger.summary()["accepted_records"], 3)
                self.assertEqual(logger.summary()["queue_pending"], 2)
            finally:
                gate.set()
                with self.assertRaisesRegex(SessionLogError, "queue_overflow"):
                    logger.close()
            self.assertEqual(logger.summary()["written_records"], 3)
            self.assertTrue((Path(directory)/"INCOMPLETE").exists())

    def test_close_does_not_wait_indefinitely_for_stuck_disk(self):
        gate, entered = threading.Event(), threading.Event()
        def opener(path, **kwargs):
            return WrappedFile(path.open(**kwargs), gate=gate, entered=entered)
        with tempfile.TemporaryDirectory() as directory:
            logger = SessionLogger(directory, close_timeout_s=.05, file_opener=opener)
            try:
                logger.record("stuck", host_monotonic_ns=1)
                self.assertTrue(entered.wait(1))
                started = time.monotonic()
                with self.assertRaisesRegex(SessionLogError, "close_timeout"):
                    logger.close()
                self.assertLess(time.monotonic()-started, .5)
                self.assertTrue((Path(directory)/"INCOMPLETE").exists())
            finally:
                gate.set()
                logger._thread.join(1)
            self.assertFalse(logger.summary()["complete"])

    def test_flush_fsync_runs_only_writer_and_fsync_error_is_observable(self):
        threads = []
        def fsync(_fd):
            threads.append(threading.current_thread().name)
        with tempfile.TemporaryDirectory() as directory:
            logger = SessionLogger(directory, fsync=fsync)
            logger.record("sample", host_monotonic_ns=1)
            logger.flush(wait=True)
            logger.close()
            self.assertTrue(threads)
            self.assertTrue(all(name == "ExperimentA-log-writer" for name in threads))
        def bad_sync(_fd):
            raise OSError("invented fsync error")
        with tempfile.TemporaryDirectory() as directory:
            logger = SessionLogger(directory, fsync=bad_sync, flush_interval_s=.01)
            logger.record("sample", host_monotonic_ns=1)
            self.assertIn("fsync error", self.wait_failed(logger))
            with self.assertRaises(SessionLogError):
                logger.close()
            self.assertTrue((Path(directory)/"INCOMPLETE").exists())

    def test_nonfinite_event_cannot_enter_journal_and_closes_incomplete(self):
        with tempfile.TemporaryDirectory() as directory:
            logger = SessionLogger(directory)
            with self.assertRaisesRegex(SessionLogError, "serialization"):
                logger.record("invalid", host_monotonic_ns=1, value=float("nan"))
            with self.assertRaises(SessionLogError):
                logger.close()
            self.assertTrue((Path(directory)/"INCOMPLETE").exists())


if __name__ == "__main__":
    unittest.main()
