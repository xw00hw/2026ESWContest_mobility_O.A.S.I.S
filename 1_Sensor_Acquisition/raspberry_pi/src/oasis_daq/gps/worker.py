"""GPS acquisition worker.

Reads NMEA lines from the receiver, archives the raw stream byte-for-byte and
emits parsed rows.  The raw log is written first so a parser change can always
be replayed against the original data.
"""

from __future__ import annotations

import logging
import os
from pathlib import Path

import serial

from ..clock import monotonic_ns, now
from ..storage import CsvWriter
from ..workers import BaseWorker
from . import nmea

log = logging.getLogger(__name__)

__all__ = ["GpsWorker", "probe_gps"]

# Poll interval for the serial read; bounds timestamp granularity.
_READ_POLL_S = 0.01


def probe_gps(device: str, baud: int = 9600, seconds: float = 5.0) -> dict:
    """Open *device* briefly and report whether it looks like an NMEA source.

    Used by ``oasis-daqctl check`` before a session starts, so a wrong port is
    caught at bring-up instead of producing an empty log.
    """
    import time
    result = {
        "device": device, "baud": baud, "opened": False, "bytes": 0,
        "lines": 0, "nmea_sentences": 0, "checksum_ok": 0, "checksum_bad": 0,
        "sentence_types": {}, "fix_valid": False, "satellites_in_view": None,
        "error": None,
    }
    try:
        with serial.Serial(device, baud, timeout=0.5) as ser:
            result["opened"] = True
            ser.reset_input_buffer()
            asm = nmea.GpsAssembler()
            buf = b""
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                chunk = ser.read(512)
                if not chunk:
                    continue
                result["bytes"] += len(chunk)
                buf += chunk
                while b"\n" in buf:
                    line, _, buf = buf.partition(b"\n")
                    text = line.decode("ascii", errors="replace").strip()
                    if not text:
                        continue
                    result["lines"] += 1
                    if not text.startswith(("$", "!")):
                        continue
                    result["nmea_sentences"] += 1
                    fix = asm.feed(text, 0, 0)
                    if fix and fix.fix_valid:
                        result["fix_valid"] = True
            result["checksum_ok"] = asm.sentences_seen - asm.checksum_failures
            result["checksum_bad"] = asm.checksum_failures
            result["sentence_types"] = dict(asm.sentence_counts)
            result["satellites_in_view"] = asm.satellites_in_view
    except (serial.SerialException, OSError) as exc:
        result["error"] = f"{type(exc).__name__}: {exc}"
    return result


class GpsWorker(BaseWorker):
    """Own the GPS port for the lifetime of a session."""

    def __init__(self, device: str, baud: int, gps_dir: Path,
                 flush_interval_s: float = 5.0, fsync_interval_s: float = 15.0,
                 read_timeout_s: float = 1.0,
                 on_event=None,
                 latest_slot=None):
        super().__init__(name="gps", on_event=on_event)
        self.device = device
        self.baud = baud
        self.gps_dir = gps_dir
        self.flush_interval_s = flush_interval_s
        self.fsync_interval_s = fsync_interval_s
        self.read_timeout_s = read_timeout_s
        self.latest_slot = latest_slot
        self.assembler = nmea.GpsAssembler()

        self._serial: serial.Serial | None = None
        self._raw_log = None
        self._csv: CsvWriter | None = None
        self._buf = b""
        self._last_raw_flush = 0
        self._last_raw_fsync = 0
        self._raw_flush_interval_ns = int(flush_interval_s * 1e9)
        self._raw_fsync_interval_ns = int(fsync_interval_s * 1e9)
        self.track: list[tuple[float, float]] = []   # (lon, lat) for route.geojson
        self._last_track_point: tuple[float, float] | None = None

    # -- lifecycle -------------------------------------------------------
    def setup(self) -> None:
        self.gps_dir.mkdir(parents=True, exist_ok=True)
        # Short timeout for the same reason as the IMU: stamp sentences close
        # to arrival rather than once per filled buffer.
        self._serial = serial.Serial(self.device, self.baud,
                                     timeout=_READ_POLL_S)
        self._serial.reset_input_buffer()
        # Raw first: whatever happens to the parser, the bytes survive.
        self._raw_log = open(self.gps_dir / "raw_nmea.log", "w",
                             buffering=1 << 16, newline="")
        self._last_raw_flush = monotonic_ns()
        self._csv = CsvWriter(self.gps_dir / "gps.csv", nmea.CSV_FIELDS,
                              self.flush_interval_s, self.fsync_interval_s)
        self.stats.extra["device"] = self.device
        self.stats.extra["baud"] = self.baud
        log.info("gps: opened %s @ %d baud", self.device, self.baud)

    def work(self) -> None:
        assert self._serial is not None
        while not self.stopping():
            try:
                pending = self._serial.in_waiting
                chunk = self._serial.read(min(pending, 1024) if pending else 1)
            except (serial.SerialException, OSError) as exc:
                self.note_error(exc, "read")
                self.emit("device_lost", device=self.device)
                return
            if not chunk:
                continue
            self._buf += chunk
            while b"\n" in self._buf:
                line, _, self._buf = self._buf.partition(b"\n")
                self._handle_line(line)
            # A receiver spewing bytes with no newline must not grow the buffer
            # without bound.
            if len(self._buf) > 4096:
                self.stats.invalid += 1
                self._buf = self._buf[-512:]

    def _handle_line(self, line: bytes) -> None:
        text = line.decode("ascii", errors="replace").strip()
        if not text:
            return
        mono, wall = now()
        if self._raw_log is not None:
            # Raw archive carries the host timestamp so the log can be aligned
            # even if the sentence itself has no time field.
            self._raw_log.write(f"{mono} {wall} {text}\n")
            # Flush on an interval so a power cut costs seconds, not the session.
            if mono - self._last_raw_flush >= self._raw_flush_interval_ns:
                self._raw_log.flush()
                self._last_raw_flush = mono
                # flush() only reaches the page cache; fsync reaches the device.
                if mono - self._last_raw_fsync >= self._raw_fsync_interval_ns:
                    try:
                        os.fsync(self._raw_log.fileno())
                    except OSError:
                        pass
                    self._last_raw_fsync = mono
        if not text.startswith(("$", "!")):
            self.stats.invalid += 1
            return
        try:
            fix = self.assembler.feed(text, mono, wall)
        except Exception as exc:                     # parser must never kill the reader
            self.note_error(exc, "parse")
            return
        if fix is None:
            return
        row = fix.to_row()
        if self._csv is not None:
            self._csv.write(row)
        self.note_sample()
        if self.latest_slot is not None:
            self.latest_slot.put(row)
        if fix.fix_valid and fix.latitude_deg is not None and fix.longitude_deg is not None:
            self._append_track(fix.longitude_deg, fix.latitude_deg)

    def _append_track(self, lon: float, lat: float) -> None:
        """Keep a decimated position track for route.geojson."""
        point = (round(lon, 7), round(lat, 7))
        if point != self._last_track_point:
            self.track.append(point)
            self._last_track_point = point

    def teardown(self) -> None:
        for closer in (self._csv,):
            if closer is not None:
                closer.close()
        if self._raw_log is not None and not self._raw_log.closed:
            try:
                self._raw_log.flush()
                os.fsync(self._raw_log.fileno())
            except OSError:
                pass
            finally:
                self._raw_log.close()
        if self._serial is not None and self._serial.is_open:
            self._serial.close()
        self.stats.extra.update({
            "sentence_types": dict(self.assembler.sentence_counts),
            "checksum_failures": self.assembler.checksum_failures,
            "track_points": len(self.track),
        })

    # -- live view -------------------------------------------------------
    def snapshot(self) -> dict:
        fix = self.assembler.last_fix
        return {
            "device": self.device,
            "alive": self.stats.running,
            "age_s": (None if self.stats.age_s() == float("inf")
                      else round(self.stats.age_s(), 2)),
            "rate_hz": round(self.stats.rate_hz(), 2),
            "fix_valid": bool(fix.fix_valid) if fix else False,
            "fix_quality": fix.fix_quality if fix else None,
            "satellite_count": fix.satellite_count if fix else None,
            "satellites_in_view": self.assembler.satellites_in_view,
            "hdop": fix.hdop if fix else None,
            "latitude_deg": fix.latitude_deg if fix else None,
            "longitude_deg": fix.longitude_deg if fix else None,
            "altitude_m": fix.altitude_m if fix else None,
            "ground_speed_mps": fix.ground_speed_mps if fix else None,
            "course_deg": fix.course_deg if fix else None,
            "gps_utc": fix.gps_utc if fix else None,
            "checksum_failures": self.assembler.checksum_failures,
            "sentences": self.assembler.sentences_seen,
            "track_points": len(self.track),
        }
