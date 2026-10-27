"""Bounded asynchronous experiment journal; never an input to motor control.

The owner must call check() every control-loop iteration and stop on failure.
Only the writer thread writes, flushes or fsyncs sample data. Host receipt times
are retained verbatim; MCU ticks and BeamNG simulation time are distinct clocks.
Pressure voltages are nominal ADC volts, not calibrated pressure/force units.
"""
from __future__ import annotations

import csv
import json
import os
from pathlib import Path
import queue
import threading
import time


class SessionLogError(RuntimeError):
    """Collection is incomplete; the caller must end motor authority."""


RAW_COLUMNS = (
    "host_monotonic_ns", "seq", "t_ms", "channel", "valid", "reason",
    "missing_samples_before", "raw0", "status0", "voltage0_v", "raw1",
    "status1", "voltage1_v", "raw2", "status2", "voltage2_v", "raw_line",
)
SENSORS = tuple(f"A{bank}_C{channel}" for bank in range(3) for channel in range(4))
SENSOR_COLUMNS = ("raw", "status", "voltage_v", "host_monotonic_ns", "t_ms", "seq")
SWEEP_COLUMNS = (
    "host_monotonic_ns", "completed_host_monotonic_ns", "first_seq", "last_seq",
    "first_t_ms", "last_t_ms", "valid", "reason",
) + tuple(f"{sensor}_{field}" for sensor in SENSORS for field in SENSOR_COLUMNS)


class SessionLogger:
    """Exclusive files, immutable queued records, bounded joins and explicit loss.

    record()/pressure()/check() perform no disk I/O. A full queue or disk error
    latches a SessionLogError; it cannot be cleared by later successful writes.
    close() drains accepted records for at most close_timeout_s (default 2 s).
    INCOMPLETE remains after any error, overflow, close timeout or process crash.
    The daemon writer permits process shutdown if an OS filesystem call hangs;
    this never makes a failed session complete. File opener/fsync injection is
    intended for entirely offline tests, not alternative production writers.
    """

    def __init__(self, outputdir, pressure_enabled=False, *, queue_limit=8192,
                 flush_interval_s=1.0, close_timeout_s=2.0,
                 file_opener=None, fsync=None):
        if type(pressure_enabled) is not bool:
            raise ValueError("pressure_enabled must be boolean")
        if type(queue_limit) is not int or not 1 <= queue_limit <= 8192:
            raise ValueError("queue_limit must be between 1 and 8192 records")
        if not 0 < flush_interval_s <= 1 or not 0 < close_timeout_s <= 2:
            raise ValueError("flush/close intervals must be positive and at most 1/2 s")
        self.output = Path(outputdir)
        self.pressure_enabled = pressure_enabled
        self.flush_interval_s = float(flush_interval_s)
        self.close_timeout_s = float(close_timeout_s)
        self._opener = file_opener or (lambda path, **kwargs: path.open(**kwargs))
        self._fsync = fsync or os.fsync
        self._queue = queue.Queue(maxsize=queue_limit)
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._done = threading.Event()
        self._error = None
        self._accepting = True
        self._closed_successfully = False
        self._accepted = self._written = 0
        self._counts = {"control": 0, "pressure": 0, "samples": 0, "sweeps": 0}
        self._handles = {}
        self._marker = self.output/"INCOMPLETE"
        names = ["control.jsonl", "INCOMPLETE"]
        if pressure_enabled:
            names += ["pressure.jsonl", "pressure_raw.csv", "pressure_sweeps.csv"]
        self.output.mkdir(parents=True, exist_ok=True)
        for name in names:
            if (self.output/name).exists():
                raise FileExistsError("Session log already exists: "+str(self.output/name))
        try:
            # Setup is synchronous and must precede UART opening/ARM.
            with self._marker.open("x", encoding="utf-8") as marker:
                marker.write("Collection incomplete until all queued logs are durably closed.\n")
                marker.flush()
                os.fsync(marker.fileno())
            for name in names:
                if name != "INCOMPLETE":
                    self._handles[name] = self._opener(
                        self.output/name, mode="x", encoding="utf-8", newline="")
        except Exception:
            for handle in self._handles.values():
                handle.close()
            raise
        self._thread = threading.Thread(target=self._run, name="ExperimentA-log-writer", daemon=True)
        self._thread.start()

    def _latch(self, reason):
        with self._lock:
            if self._error is None:
                self._error = str(reason)

    def check(self):
        """Raise promptly on error; call each loop before renewing motor lease."""
        with self._lock:
            if self._error is not None:
                raise SessionLogError(self._error)
            if self._done.is_set() and self._accepting:
                self._error = "log_writer_stopped_unexpectedly"
                raise SessionLogError(self._error)

    def _enqueue(self, route, value):
        # JSON serialization snapshots mutable caller data before queueing.
        try:
            encoded = json.dumps(value, allow_nan=False, ensure_ascii=False,
                                 separators=(",", ":"))+"\n"
        except Exception as exc:
            self._latch(f"log_serialization:{type(exc).__name__}:{exc}")
            raise SessionLogError(self._error) from exc
        with self._lock:
            self.check()
            if not self._accepting:
                raise SessionLogError("log_closed")
            try:
                self._queue.put_nowait((route, encoded))
            except queue.Full as exc:
                self._latch("log_queue_overflow: accepted records retained; session incomplete")
                raise SessionLogError(self._error) from exc
            self._accepted += 1

    def record(self, kind, *, host_monotonic_ns, **payload):
        if not isinstance(kind, str) or not kind or "event" in payload:
            raise ValueError("Expected one nonempty event kind")
        if type(host_monotonic_ns) is not int or host_monotonic_ns < 0:
            raise ValueError("host_monotonic_ns must be original nonnegative integer ns")
        self._enqueue("control", dict(event=kind, host_monotonic_ns=host_monotonic_ns, **payload))

    def pressure(self, records):
        if not isinstance(records, (list, tuple)):
            raise ValueError("pressure records must be a list or tuple")
        if records and not self.pressure_enabled:
            raise SessionLogError("Pressure recording is disabled")
        for record in records:
            if not isinstance(record, dict) or record.get("type") not in ("sample", "sweep", "event", "stats"):
                raise ValueError("Unknown pressure record type")
            timestamp = record.get("host_monotonic_ns")
            if type(timestamp) is not int or timestamp < 0:
                raise ValueError("Pressure records require original integer host receipt ns")
            self._enqueue("pressure", record)

    def flush(self, *, wait=False):
        """Request writer flush/fsync; default is nonblocking, never use wait in control."""
        completed = threading.Event()
        with self._lock:
            self.check()
            if not self._accepting:
                raise SessionLogError("log_closed")
            try:
                self._queue.put_nowait(("flush", completed))
            except queue.Full as exc:
                self._latch("log_queue_overflow: flush could not be queued")
                raise SessionLogError(self._error) from exc
        if wait:
            if not completed.wait(self.close_timeout_s):
                self.check()
                self._latch("log_flush_timeout")
                raise SessionLogError(self._error)
            self.check()
        return completed

    def _sync(self):
        for handle in self._handles.values():
            handle.flush()
            self._fsync(handle.fileno())

    def _run(self):
        try:
            raw_writer = sweep_writer = None
            if self.pressure_enabled:
                raw_writer = csv.DictWriter(self._handles["pressure_raw.csv"], RAW_COLUMNS, extrasaction="ignore")
                sweep_writer = csv.DictWriter(self._handles["pressure_sweeps.csv"], SWEEP_COLUMNS, extrasaction="ignore")
                raw_writer.writeheader()
                sweep_writer.writeheader()
            next_sync = time.monotonic()+self.flush_interval_s
            while not (self._stop.is_set() and self._queue.empty()):
                try:
                    route, value = self._queue.get(timeout=min(.05, self.flush_interval_s))
                except queue.Empty:
                    route = None
                if route is not None:
                    try:
                        if route == "flush":
                            self._sync()
                            value.set()
                            next_sync = time.monotonic()+self.flush_interval_s
                        else:
                            self._handles[route+".jsonl"].write(value)
                            if route == "pressure":
                                record = json.loads(value)
                                if record["type"] == "sample":
                                    raw_writer.writerow(record)
                                    self._counts["samples"] += 1
                                elif record["type"] == "sweep":
                                    row = dict(record)
                                    for sensor, readings in record.get("values", {}).items():
                                        row.update({sensor+"_"+key: item for key, item in readings.items()})
                                    sweep_writer.writerow(row)
                                    self._counts["sweeps"] += 1
                            self._counts[route] += 1
                            self._written += 1
                    finally:
                        self._queue.task_done()
                if time.monotonic() >= next_sync:
                    self._sync()
                    next_sync = time.monotonic()+self.flush_interval_s
            self._sync()
        except Exception as exc:
            self._latch(f"log_writer:{type(exc).__name__}:{exc}")
        finally:
            for handle in self._handles.values():
                try:
                    handle.close()
                except Exception as exc:
                    self._latch(f"log_close:{type(exc).__name__}:{exc}")
            self._done.set()

    def summary(self):
        return dict(accepted_records=self._accepted, written_records=self._written,
                    counts=dict(self._counts), queue_pending=self._queue.qsize(),
                    writer_alive=self._thread.is_alive(), error=self._error,
                    complete=self._closed_successfully)

    def close(self):
        """Drain once, fsync, close; success alone removes INCOMPLETE."""
        with self._lock:
            if self._closed_successfully:
                return self.summary()
            self._accepting = False
            self._stop.set()
        self._thread.join(self.close_timeout_s)
        if self._thread.is_alive():
            self._latch("log_close_timeout: writer still running; session incomplete")
        if self._accepted != self._written and self._error is None:
            self._latch("log_incomplete: accepted/written count mismatch")
        self.check()
        try:
            self._marker.unlink()
        except OSError as exc:
            self._latch(f"log_completion_marker:{type(exc).__name__}:{exc}")
            raise SessionLogError(self._error) from exc
        self._closed_successfully = True
        return self.summary()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
