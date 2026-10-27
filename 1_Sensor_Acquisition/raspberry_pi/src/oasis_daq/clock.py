"""Common time base for every OASIS DAQ source.

All records carry a host monotonic timestamp so that streams recorded at
different rates can be aligned in post-processing without resampling during
acquisition.  ``CLOCK_MONOTONIC_RAW`` is preferred because it is not slewed by
NTP; ``time.monotonic_ns`` is the documented fallback.
"""

from __future__ import annotations

import time
from typing import Callable

__all__ = [
    "monotonic_ns",
    "wall_time_ns",
    "clock_source",
    "now",
    "Stopwatch",
]


def _select_monotonic() -> tuple[Callable[[], int], str]:
    """Pick the best available monotonic source, once, at import time."""
    raw = getattr(time, "CLOCK_MONOTONIC_RAW", None)
    if raw is not None:
        try:
            time.clock_gettime_ns(raw)
        except (OSError, AttributeError, ValueError):
            pass
        else:
            return (lambda: time.clock_gettime_ns(raw)), "CLOCK_MONOTONIC_RAW"
    return time.monotonic_ns, "CLOCK_MONOTONIC"


_monotonic_ns, CLOCK_SOURCE = _select_monotonic()


def monotonic_ns() -> int:
    """Nanoseconds from an unspecified but monotonic epoch."""
    return _monotonic_ns()


def wall_time_ns() -> int:
    """Nanoseconds since the UNIX epoch (subject to NTP steps)."""
    return time.time_ns()


def clock_source() -> str:
    """Name of the monotonic clock actually in use, recorded in metadata."""
    return CLOCK_SOURCE


def now() -> tuple[int, int]:
    """Return ``(host_monotonic_ns, host_wall_time_ns)`` sampled back-to-back."""
    return _monotonic_ns(), time.time_ns()


class Stopwatch:
    """Small helper for measuring worker latency in nanoseconds."""

    __slots__ = ("_start",)

    def __init__(self) -> None:
        self._start = _monotonic_ns()

    def reset(self) -> None:
        self._start = _monotonic_ns()

    def elapsed_ns(self) -> int:
        return _monotonic_ns() - self._start

    def elapsed_ms(self) -> float:
        return (_monotonic_ns() - self._start) / 1e6
