"""Shared worker scaffolding: isolated threads and bounded queues.

Every sensor runs in its own thread with its own ``try/except``.  A worker that
raises records the fault, marks itself unhealthy and stops -- it never takes
another sensor's logging down with it.
"""

from __future__ import annotations

import logging
import queue
import threading
import traceback
from dataclasses import dataclass, field
from typing import Any

from .clock import monotonic_ns

__all__ = ["WorkerStats", "BaseWorker", "LatestSlot", "DropCountingQueue"]

log = logging.getLogger(__name__)


@dataclass
class WorkerStats:
    """Live counters every worker exposes to the health monitor."""

    name: str
    started: bool = False
    running: bool = False
    samples: int = 0
    errors: int = 0
    dropped: int = 0
    invalid: int = 0
    last_sample_monotonic_ns: int = 0
    started_monotonic_ns: int = 0
    last_error: str = ""
    extra: dict[str, Any] = field(default_factory=dict)

    def age_s(self) -> float:
        """Seconds since the last sample, or ``inf`` if there never was one."""
        if not self.last_sample_monotonic_ns:
            return float("inf")
        return (monotonic_ns() - self.last_sample_monotonic_ns) / 1e9

    def rate_hz(self) -> float:
        if not self.started_monotonic_ns or not self.samples:
            return 0.0
        elapsed = (monotonic_ns() - self.started_monotonic_ns) / 1e9
        return self.samples / elapsed if elapsed > 0 else 0.0

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "running": self.running,
            "samples": self.samples,
            "errors": self.errors,
            "dropped": self.dropped,
            "invalid": self.invalid,
            "rate_hz": round(self.rate_hz(), 3),
            "age_s": (None if self.age_s() == float("inf") else round(self.age_s(), 3)),
            "last_error": self.last_error,
            **{k: v for k, v in self.extra.items()},
        }


class DropCountingQueue(queue.Queue):
    """Bounded queue that counts, rather than blocks on, overflow.

    Raw sensor rows use a large maxsize so overflow is practically impossible;
    when it does happen the drop is counted and surfaced as an event instead of
    stalling the reader thread and losing the serial stream.
    """

    def __init__(self, maxsize: int, name: str = ""):
        super().__init__(maxsize=maxsize)
        self.name = name
        self.dropped = 0

    def put_nowait_counted(self, item: Any) -> bool:
        """Return True if queued, False if the queue was full (item dropped)."""
        try:
            self.put_nowait(item)
            return True
        except queue.Full:
            self.dropped += 1
            return False

    def fill_ratio(self) -> float:
        return (self.qsize() / self.maxsize) if self.maxsize else 0.0


class LatestSlot:
    """A one-item mailbox that always holds the newest value.

    Used for preview frames and pose input, where a stale item is worthless:
    a slow consumer must never apply back-pressure to the camera.
    """

    __slots__ = ("_lock", "_value", "_stamp", "_seq", "_overwrites")

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._value: Any = None
        self._stamp: int = 0
        self._seq: int = 0
        self._overwrites: int = 0

    def put(self, value: Any) -> None:
        with self._lock:
            if self._value is not None:
                self._overwrites += 1
            self._value = value
            self._stamp = monotonic_ns()
            self._seq += 1

    def get(self) -> tuple[Any, int, int]:
        """Return ``(value, monotonic_ns, sequence)`` without consuming."""
        with self._lock:
            return self._value, self._stamp, self._seq

    def take(self) -> tuple[Any, int, int]:
        """Return the value and clear the slot."""
        with self._lock:
            value, stamp, seq = self._value, self._stamp, self._seq
            self._value = None
            return value, stamp, seq

    @property
    def overwrites(self) -> int:
        return self._overwrites


class BaseWorker(threading.Thread):
    """A daemon thread with lifecycle, stats and fault isolation."""

    def __init__(self, name: str, on_event=None):
        super().__init__(name=name, daemon=True)
        self.stats = WorkerStats(name=name)
        # NOTE: must not be called `_stop` -- threading.Thread has a private
        # `_stop()` method that join() calls, and shadowing it breaks join().
        self._stop_event = threading.Event()
        self._on_event = on_event

    # -- lifecycle -------------------------------------------------------
    def setup(self) -> None:
        """Acquire resources.  Raising here marks the worker failed."""

    def work(self) -> None:
        """Main loop.  Must poll :meth:`stopping` and return when set."""
        raise NotImplementedError

    def teardown(self) -> None:
        """Always called, even after a failure in setup or work."""

    # -- helpers ---------------------------------------------------------
    def stopping(self) -> bool:
        return self._stop_event.is_set()

    def stop(self) -> None:
        self._stop_event.set()

    def emit(self, kind: str, **payload: Any) -> None:
        """Record a structured event (queue overflow, device loss, ...)."""
        if self._on_event is not None:
            try:
                self._on_event(kind, self.stats.name, payload)
            except Exception:  # an event sink must never break a worker
                log.exception("event sink failed for %s", self.stats.name)

    def note_sample(self, n: int = 1) -> None:
        self.stats.samples += n
        self.stats.last_sample_monotonic_ns = monotonic_ns()

    def note_error(self, exc: BaseException, context: str = "") -> None:
        self.stats.errors += 1
        self.stats.last_error = f"{context}: {type(exc).__name__}: {exc}".strip(": ")
        log.warning("%s error (%s): %s", self.stats.name, context, exc)
        self.emit("worker_error", context=context, error=str(exc),
                  error_type=type(exc).__name__)

    # -- thread body -----------------------------------------------------
    def run(self) -> None:
        self.stats.started = True
        self.stats.started_monotonic_ns = monotonic_ns()
        try:
            self.setup()
            self.stats.running = True
            self.emit("worker_started")
            self.work()
        except Exception as exc:
            self.stats.errors += 1
            self.stats.last_error = f"{type(exc).__name__}: {exc}"
            log.error("%s failed: %s\n%s", self.stats.name, exc, traceback.format_exc())
            self.emit("worker_failed", error=str(exc),
                      error_type=type(exc).__name__,
                      traceback=traceback.format_exc(limit=8))
        finally:
            self.stats.running = False
            try:
                self.teardown()
            except Exception:
                log.exception("%s teardown failed", self.stats.name)
            self.emit("worker_stopped", samples=self.stats.samples,
                      errors=self.stats.errors, dropped=self.stats.dropped)
