"""IMU acquisition worker (WIT standard protocol over a USB-UART bridge).

The raw serial byte stream is archived in numbered chunks before any parsing
happens, so the recording survives a protocol misunderstanding.
"""

from __future__ import annotations

import logging
import time
from pathlib import Path

import serial

from ..clock import now
from ..storage import CsvWriter, RawChunkWriter
from ..workers import BaseWorker
from . import wit

log = logging.getLogger(__name__)

__all__ = ["ImuWorker", "probe_imu", "probe_baud_rates"]

# Poll interval for the serial read; bounds timestamp granularity.
_READ_POLL_S = 0.01


def probe_imu(device: str, baud: int, seconds: float = 3.0) -> dict:
    """Judge a candidate baud by frame quality, not by "bytes arrived".

    A wrong baud still produces bytes; what it does not produce is a high ratio
    of checksum-valid WIT frames.  ``frame_ratio`` is the deciding number.
    """
    result = {
        "device": device, "baud": baud, "opened": False, "bytes": 0,
        "frames_ok": 0, "frames_bad_checksum": 0, "bytes_discarded": 0,
        "packet_types": {}, "frame_ratio": 0.0, "rate_hz": 0.0,
        "looks_like_wit": False, "error": None,
    }
    try:
        with serial.Serial(device, baud, timeout=0.3) as ser:
            result["opened"] = True
            time.sleep(0.2)
            ser.reset_input_buffer()
            parser = wit.FrameParser()
            counts: dict[str, int] = {}
            t0 = time.monotonic()
            while time.monotonic() - t0 < seconds:
                chunk = ser.read(256)
                if not chunk:
                    continue
                result["bytes"] += len(chunk)
                for frame in parser.feed(chunk):
                    counts[frame.name] = counts.get(frame.name, 0) + 1
            elapsed = max(time.monotonic() - t0, 1e-6)
            result["frames_ok"] = parser.frames_ok
            result["frames_bad_checksum"] = parser.frames_bad_checksum
            result["bytes_discarded"] = parser.bytes_discarded
            result["packet_types"] = counts
            total = parser.frames_ok + parser.frames_bad_checksum
            # Fraction of received bytes that landed inside a valid frame.
            result["frame_ratio"] = (
                (parser.frames_ok * wit.FRAME_LEN) / result["bytes"]
                if result["bytes"] else 0.0)
            result["rate_hz"] = parser.frames_ok / elapsed
            result["looks_like_wit"] = bool(
                parser.frames_ok >= 10
                and result["frame_ratio"] > 0.8
                and (parser.frames_bad_checksum / total if total else 1.0) < 0.05
            )
    except (serial.SerialException, OSError) as exc:
        result["error"] = f"{type(exc).__name__}: {exc}"
    return result


def probe_baud_rates(device: str, candidates: list[int],
                     seconds: float = 3.0) -> dict:
    """Try each candidate baud and return the best WIT match.

    Read-only: no configuration frame is ever written to the device.
    """
    attempts = [probe_imu(device, baud, seconds) for baud in candidates]
    good = [a for a in attempts if a["looks_like_wit"]]
    best = max(good, key=lambda a: (a["frame_ratio"], a["frames_ok"])) if good else None
    return {
        "device": device,
        "attempts": attempts,
        "selected_baud": best["baud"] if best else None,
        "confident": bool(best),
    }


class ImuWorker(BaseWorker):
    """Own the IMU port for the lifetime of a session."""

    def __init__(self, device: str, baud: int, imu_dir: Path,
                 acc_range_g: float = 16.0, gyro_range_dps: float = 2000.0,
                 angle_range_deg: float = 180.0, rotation_matrix=None,
                 calibration: dict | None = None,
                 save_raw_stream: bool = True, raw_chunk_bytes: int = 1 << 20,
                 flush_interval_s: float = 5.0, fsync_interval_s: float = 15.0,
                 read_timeout_s: float = 1.0,
                 on_event=None, latest_slot=None):
        super().__init__(name="imu", on_event=on_event)
        self.device = device
        self.baud = baud
        self.imu_dir = imu_dir
        self.save_raw_stream = save_raw_stream
        self.raw_chunk_bytes = raw_chunk_bytes
        self.flush_interval_s = flush_interval_s
        self.fsync_interval_s = fsync_interval_s
        self.read_timeout_s = read_timeout_s
        self.latest_slot = latest_slot

        self.parser = wit.FrameParser()
        self.assembler = wit.SampleAssembler(
            acc_range_g=acc_range_g, gyro_range_dps=gyro_range_dps,
            angle_range_deg=angle_range_deg, rotation_matrix=rotation_matrix,
            calibration=calibration,
        )
        self._serial: serial.Serial | None = None
        self._raw: RawChunkWriter | None = None
        self._csv: CsvWriter | None = None
        self._last_sample: wit.WitSample | None = None

    def setup(self) -> None:
        self.imu_dir.mkdir(parents=True, exist_ok=True)
        # A short timeout is deliberate.  `read(256)` with a 1 s timeout blocks
        # until the buffer fills, so at 9600 baud a whole second of frames would
        # share one host timestamp (measured: 669 duplicate stamps in 809
        # samples).  Reading only what has arrived keeps each frame's timestamp
        # within one frame time (~11 ms at 9600 baud) of its arrival.
        self._serial = serial.Serial(self.device, self.baud,
                                     timeout=_READ_POLL_S)
        time.sleep(0.1)
        self._serial.reset_input_buffer()
        if self.save_raw_stream:
            self._raw = RawChunkWriter(self.imu_dir, "raw",
                                       chunk_bytes=self.raw_chunk_bytes,
                                       flush_interval_s=self.flush_interval_s)
        self._csv = CsvWriter(self.imu_dir / "imu.csv", wit.CSV_FIELDS,
                              self.flush_interval_s, self.fsync_interval_s)
        self.stats.extra["device"] = self.device
        self.stats.extra["baud"] = self.baud
        log.info("imu: opened %s @ %d baud", self.device, self.baud)

    def work(self) -> None:
        assert self._serial is not None
        while not self.stopping():
            try:
                pending = self._serial.in_waiting
                # read(1) blocks for at most _READ_POLL_S when the line is idle.
                chunk = self._serial.read(min(pending, 512) if pending else 1)
            except (serial.SerialException, OSError) as exc:
                self.note_error(exc, "read")
                self.emit("device_lost", device=self.device)
                return
            if not chunk:
                continue
            # Stamp before archiving: RawChunkWriter.write() can fsync, and an
            # exFAT stall would otherwise be added to every sample's timestamp.
            mono, wall = now()
            if self._raw is not None:
                self._raw.write(chunk)          # archive before interpreting
            # The try sits INSIDE the loop on purpose.  Outside it, one bad
            # frame aborted the whole chunk -- and if the fault were systematic
            # (a malformed mounting matrix, say) every chunk would abort at its
            # first sample forever, leaving imu.csv empty while the worker still
            # reported itself running.
            for frame in self.parser.feed(chunk):
                try:
                    sample = self.assembler.feed(frame, mono, wall)
                except Exception as exc:
                    self.note_error(exc, "parse")
                    continue
                if sample is not None:
                    self._emit_sample(sample)

    def _emit_sample(self, sample: wit.WitSample) -> None:
        self._last_sample = sample
        row = sample.to_row()
        if self._csv is not None:
            self._csv.write(row)
        self.note_sample()
        if not sample.sample_valid:
            self.stats.invalid += 1
        if self.latest_slot is not None:
            self.latest_slot.put(row)

    def teardown(self) -> None:
        mono, wall = now()
        try:
            tail = self.assembler.flush(mono, wall)
            if tail is not None:
                self._emit_sample(tail)
        except Exception:
            log.exception("imu: flush failed")
        if self._csv is not None:
            self._csv.close()
        if self._raw is not None:
            self._raw.close()
        if self._serial is not None and self._serial.is_open:
            self._serial.close()
        self.stats.extra.update({
            "frames_ok": self.parser.frames_ok,
            "frames_bad_checksum": self.parser.frames_bad_checksum,
            "bytes_discarded": self.parser.bytes_discarded,
            "packet_types": dict(self.assembler.type_counts),
            "raw_bytes": self._raw.total_bytes if self._raw else 0,
            "raw_chunks": list(self._raw.chunks) if self._raw else [],
        })

    def snapshot(self) -> dict:
        s = self._last_sample
        total = self.parser.frames_ok + self.parser.frames_bad_checksum
        return {
            "device": self.device,
            "alive": self.stats.running,
            "age_s": (None if self.stats.age_s() == float("inf")
                      else round(self.stats.age_s(), 2)),
            "rate_hz": round(self.stats.rate_hz(), 2),
            "frames_ok": self.parser.frames_ok,
            "checksum_errors": self.parser.frames_bad_checksum,
            "checksum_error_ratio": round(
                self.parser.frames_bad_checksum / total, 5) if total else 0.0,
            "acc_mps2": ([s.veh_acc_x_mps2, s.veh_acc_y_mps2, s.veh_acc_z_mps2]
                         if s else None),
            "gyro_radps": ([s.veh_gyro_x_radps, s.veh_gyro_y_radps, s.veh_gyro_z_radps]
                           if s else None),
            "orientation_rad": ([s.roll_rad, s.pitch_rad, s.yaw_rad] if s else None),
            "g_force": (
                round(((s.veh_acc_x_mps2 ** 2 + s.veh_acc_y_mps2 ** 2
                        + s.veh_acc_z_mps2 ** 2) ** 0.5) / wit.G_TO_MPS2, 4)
                if s and s.veh_acc_x_mps2 is not None else None),
            "temperature_c": s.temperature_c if s else None,
            "packet_types": dict(self.assembler.type_counts),
        }
