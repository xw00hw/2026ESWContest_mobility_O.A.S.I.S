"""System and worker health sampling.

Reads the Pi's own counters straight from ``/proc`` and ``/sys`` rather than
pulling in psutil, so the DAQ has one fewer dependency to install on the target.
"""

from __future__ import annotations

import os
import time
from dataclasses import dataclass
from pathlib import Path

__all__ = [
    "SessionState", "cpu_temperature_c", "cpu_percent", "memory_info",
    "load_average", "throttled_flags", "HEALTH_CSV_FIELDS", "SystemSampler",
]

_THERMAL = Path("/sys/class/thermal/thermal_zone0/temp")


class SessionState:
    INIT = "INIT"
    READY = "READY"
    RECORDING = "RECORDING"
    SAVING = "SAVING"
    ERROR = "ERROR"

    ALL = (INIT, READY, RECORDING, SAVING, ERROR)


HEALTH_CSV_FIELDS = [
    "host_monotonic_ns", "host_wall_time_ns", "state",
    "cpu_percent", "cpu_temp_c", "load_1m",
    "mem_total_mb", "mem_available_mb", "mem_used_percent",
    "disk_free_gb", "disk_write_latency_ms",
    "gps_alive", "gps_rate_hz", "gps_age_s", "gps_fix_valid", "gps_satellites",
    "imu_alive", "imu_rate_hz", "imu_age_s", "imu_checksum_errors",
    "camera_alive", "camera_fps", "camera_frames_recorded", "camera_drops",
    "pose_alive", "pose_fps", "pose_latency_p50_ms", "pose_valid_ratio",
    "queue_camera_write", "camera_write_drops", "queue_pose_dropped",
    "events_warning", "events_error",
]


def cpu_temperature_c() -> float | None:
    """SoC temperature in degrees Celsius, or None if unreadable."""
    try:
        return int(_THERMAL.read_text().strip()) / 1000.0
    except (OSError, ValueError):
        return None


def load_average() -> tuple[float, float, float]:
    try:
        return os.getloadavg()
    except OSError:
        return (0.0, 0.0, 0.0)


def memory_info() -> dict:
    """Total/available memory in MB plus used percentage, from /proc/meminfo."""
    info: dict[str, int] = {}
    try:
        for line in Path("/proc/meminfo").read_text().splitlines():
            key, _, rest = line.partition(":")
            parts = rest.split()
            if parts:
                info[key] = int(parts[0])          # kB
    except (OSError, ValueError):
        return {"total_mb": 0.0, "available_mb": 0.0, "used_percent": 0.0}
    total = info.get("MemTotal", 0) / 1024.0
    available = info.get("MemAvailable", 0) / 1024.0
    used_pct = ((total - available) / total * 100.0) if total else 0.0
    return {"total_mb": round(total, 1), "available_mb": round(available, 1),
            "used_percent": round(used_pct, 1)}


def throttled_flags() -> str | None:
    """``vcgencmd get_throttled`` value, when the tool is available."""
    import shutil
    import subprocess
    exe = shutil.which("vcgencmd")
    if not exe:
        return None
    try:
        out = subprocess.run([exe, "get_throttled"], capture_output=True,
                             text=True, timeout=3, check=False).stdout.strip()
        return out.split("=", 1)[1] if "=" in out else out
    except (OSError, subprocess.SubprocessError):
        return None


class SystemSampler:
    """Stateful CPU sampler: percentage needs two /proc/stat readings."""

    def __init__(self) -> None:
        self._prev = self._read_stat()

    @staticmethod
    def _read_stat() -> tuple[int, int] | None:
        try:
            first = Path("/proc/stat").read_text().splitlines()[0]
        except (OSError, IndexError):
            return None
        parts = first.split()
        if parts[0] != "cpu" or len(parts) < 5:
            return None
        values = [int(v) for v in parts[1:8]]
        idle = values[3] + (values[4] if len(values) > 4 else 0)
        return sum(values), idle

    def cpu_percent(self) -> float:
        """CPU utilisation since the previous call."""
        current = self._read_stat()
        if current is None or self._prev is None:
            self._prev = current
            return 0.0
        total_delta = current[0] - self._prev[0]
        idle_delta = current[1] - self._prev[1]
        self._prev = current
        if total_delta <= 0:
            return 0.0
        return round(max(0.0, (1.0 - idle_delta / total_delta)) * 100.0, 1)


def cpu_percent(interval: float = 0.2) -> float:
    """One-shot CPU utilisation over *interval* seconds."""
    sampler = SystemSampler()
    time.sleep(interval)
    return sampler.cpu_percent()


def measure_write_latency_ms(directory: Path, payload: bytes = b"\0" * 65536) -> float | None:
    """Time a 64 KiB write+fsync in *directory*; None when it fails.

    Deliberately writes to a temporary file that is removed immediately; it
    never touches any recorded data.
    """
    probe = directory / ".oasis_write_latency"
    try:
        start = time.perf_counter()
        with open(probe, "wb") as fh:
            fh.write(payload)
            fh.flush()
            os.fsync(fh.fileno())
        elapsed = (time.perf_counter() - start) * 1000.0
        return round(elapsed, 3)
    except OSError:
        return None
    finally:
        try:
            probe.unlink()
        except OSError:
            pass
