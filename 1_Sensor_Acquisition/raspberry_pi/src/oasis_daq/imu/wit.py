"""WITMOTION WIT-standard binary protocol.

Frame layout (11 bytes, little-endian payload)::

    0x55  <type>  d0 d1 d2 d3 d4 d5 d6 d7  <checksum>
    checksum = sum(frame[0:10]) & 0xFF

Confirmed on the attached unit (CH340 bridge, 9600 baud): packet types 0x51
(acceleration), 0x52 (angular velocity), 0x53 (Euler angles) and 0x54
(magnetic field) each arriving at 10 Hz with zero checksum failures over 418
frames.  Scaling constants follow the WIT standard protocol reference in
``sensor-drivers/WitStandardProtocol_JY901`` (WITMOTION official repository).

Only bytes are interpreted here; no serial I/O and no configuration writes.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field, asdict
from typing import Any, Iterator

__all__ = [
    "HEADER", "FRAME_LEN", "PacketType", "PACKET_NAMES",
    "WitFrame", "WitSample", "FrameParser", "SampleAssembler",
    "validate_rotation", "InvalidRotation", "IDENTITY",
    "G_TO_MPS2", "DEG_TO_RAD", "CSV_FIELDS",
]

HEADER = 0x55
FRAME_LEN = 11

G_TO_MPS2 = 9.80665
DEG_TO_RAD = 0.017453292519943295


class PacketType:
    TIME = 0x50
    ACC = 0x51
    GYRO = 0x52
    ANGLE = 0x53
    MAG = 0x54
    PORT = 0x55
    PRESSURE = 0x56
    LONLAT = 0x57
    GPS_SPEED = 0x58
    QUATERNION = 0x59
    GPS_ACCURACY = 0x5A


PACKET_NAMES = {
    0x50: "time", 0x51: "acc", 0x52: "gyro", 0x53: "angle", 0x54: "mag",
    0x55: "port", 0x56: "pressure", 0x57: "lonlat", 0x58: "gps_speed",
    0x59: "quaternion", 0x5A: "gps_accuracy",
}

# The four packet types that make up one logical IMU sample on this unit.
_SAMPLE_TYPES = (PacketType.ACC, PacketType.GYRO, PacketType.ANGLE, PacketType.MAG)


def checksum(frame: bytes) -> int:
    """WIT checksum: low byte of the sum of the first 10 bytes."""
    return sum(frame[:10]) & 0xFF


def _s16(data: bytes, offset: int) -> int:
    return struct.unpack_from("<h", data, offset)[0]


@dataclass
class WitFrame:
    packet_type: int
    payload: bytes
    checksum_valid: bool
    raw: bytes

    @property
    def name(self) -> str:
        return PACKET_NAMES.get(self.packet_type, f"unknown_0x{self.packet_type:02X}")

    def words(self) -> tuple[int, int, int, int]:
        """The four signed 16-bit words of the payload."""
        return (_s16(self.payload, 0), _s16(self.payload, 2),
                _s16(self.payload, 4), _s16(self.payload, 6))


class FrameParser:
    """Incremental byte-stream -> frame parser.

    Resynchronises by discarding bytes until a plausible header is found, so a
    mid-stream connect or a dropped byte costs at most one frame.
    """

    def __init__(self) -> None:
        self._buf = bytearray()
        self.frames_ok = 0
        self.frames_bad_checksum = 0
        self.bytes_discarded = 0
        self.bytes_consumed = 0

    def feed(self, data: bytes) -> Iterator[WitFrame]:
        """Append *data* and yield every complete frame it completes."""
        if data:
            self._buf.extend(data)
            self.bytes_consumed += len(data)
        while len(self._buf) >= FRAME_LEN:
            if self._buf[0] != HEADER:
                del self._buf[0]
                self.bytes_discarded += 1
                continue
            raw = bytes(self._buf[:FRAME_LEN])
            ptype = raw[1]
            # A 0x55 that is not followed by a known type is payload noise, not
            # a header: drop just that byte and keep looking.
            if ptype not in PACKET_NAMES:
                del self._buf[0]
                self.bytes_discarded += 1
                continue
            del self._buf[:FRAME_LEN]
            ok = checksum(raw) == raw[10]
            if ok:
                self.frames_ok += 1
            else:
                self.frames_bad_checksum += 1
            yield WitFrame(packet_type=ptype, payload=raw[2:10],
                           checksum_valid=ok, raw=raw)

    @property
    def pending_bytes(self) -> int:
        return len(self._buf)


@dataclass
class WitSample:
    """One assembled IMU sample, raw counts and engineering units side by side."""

    host_monotonic_ns: int = 0
    host_wall_time_ns: int = 0
    sensor_sequence: int = 0

    # raw 16-bit counts, exactly as received
    raw_acc_x: int | None = None
    raw_acc_y: int | None = None
    raw_acc_z: int | None = None
    raw_gyro_x: int | None = None
    raw_gyro_y: int | None = None
    raw_gyro_z: int | None = None
    raw_mag_x: int | None = None
    raw_mag_y: int | None = None
    raw_mag_z: int | None = None
    raw_roll: int | None = None
    raw_pitch: int | None = None
    raw_yaw: int | None = None
    raw_q0: int | None = None
    raw_q1: int | None = None
    raw_q2: int | None = None
    raw_q3: int | None = None
    temperature_c: float | None = None

    # sensor frame, engineering units
    acc_x_mps2: float | None = None
    acc_y_mps2: float | None = None
    acc_z_mps2: float | None = None
    gyro_x_radps: float | None = None
    gyro_y_radps: float | None = None
    gyro_z_radps: float | None = None
    mag_x: float | None = None
    mag_y: float | None = None
    mag_z: float | None = None
    roll_rad: float | None = None
    pitch_rad: float | None = None
    yaw_rad: float | None = None
    quaternion_w: float | None = None
    quaternion_x: float | None = None
    quaternion_y: float | None = None
    quaternion_z: float | None = None

    # vehicle frame (+x forward, +y left, +z up) after the mounting transform
    veh_acc_x_mps2: float | None = None
    veh_acc_y_mps2: float | None = None
    veh_acc_z_mps2: float | None = None
    veh_gyro_x_radps: float | None = None
    veh_gyro_y_radps: float | None = None
    veh_gyro_z_radps: float | None = None

    packet_type: str = ""
    packets_in_sample: int = 0
    checksum_valid: bool = True
    sample_valid: bool = False

    def to_row(self) -> dict[str, Any]:
        row = asdict(self)
        row["checksum_valid"] = int(bool(self.checksum_valid))
        row["sample_valid"] = int(bool(self.sample_valid))
        return row


CSV_FIELDS = list(WitSample().to_row().keys())


IDENTITY = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]


class InvalidRotation(ValueError):
    """The configured mounting transform is not a usable rotation."""


def validate_rotation(matrix) -> list[list[float]]:
    """Check a mounting matrix before it silently corrupts every sample.

    A malformed matrix raises IndexError deep inside the acquisition loop; a
    sign typo giving det = -1 mirrors the vehicle frame and is invisible in the
    data.  Both are caught here, at construction, where the operator can act.
    """
    if matrix is None:
        return [row[:] for row in IDENTITY]
    try:
        rows = [[float(v) for v in row] for row in matrix]
    except (TypeError, ValueError) as exc:
        raise InvalidRotation(f"rotation_matrix is not numeric: {exc}") from exc
    if len(rows) != 3 or any(len(r) != 3 for r in rows):
        raise InvalidRotation(
            f"rotation_matrix must be 3x3, got {len(rows)}x"
            f"{[len(r) for r in rows]}")

    # Orthonormal: R * R^T == I
    for i in range(3):
        for j in range(3):
            dot = sum(rows[i][k] * rows[j][k] for k in range(3))
            if abs(dot - (1.0 if i == j else 0.0)) > 1e-6:
                raise InvalidRotation(
                    "rotation_matrix is not orthonormal "
                    f"(row {i}.row {j} = {dot:.6f})")

    det = (rows[0][0] * (rows[1][1] * rows[2][2] - rows[1][2] * rows[2][1])
           - rows[0][1] * (rows[1][0] * rows[2][2] - rows[1][2] * rows[2][0])
           + rows[0][2] * (rows[1][0] * rows[2][1] - rows[1][1] * rows[2][0]))
    if abs(det - 1.0) > 1e-6:
        raise InvalidRotation(
            f"rotation_matrix has determinant {det:.6f}, not +1 -- a value of "
            "-1 mirrors the vehicle frame, which is almost certainly a sign typo")
    return rows


def _apply_rotation(matrix, x, y, z):
    """v_vehicle = R @ v_sensor, tolerating missing components."""
    if x is None or y is None or z is None:
        return None, None, None
    r = matrix
    return (
        r[0][0] * x + r[0][1] * y + r[0][2] * z,
        r[1][0] * x + r[1][1] * y + r[1][2] * z,
        r[2][0] * x + r[2][1] * y + r[2][2] * z,
    )


class SampleAssembler:
    """Group the per-quantity packets into whole samples.

    The WIT protocol sends each quantity as its own frame.  A sample is emitted
    when a packet type repeats (the next round has started) or when every
    expected type has been seen, so no data is ever held back waiting for a
    packet the unit does not send.
    """

    def __init__(
        self,
        acc_range_g: float = 16.0,
        gyro_range_dps: float = 2000.0,
        angle_range_deg: float = 180.0,
        rotation_matrix=None,
        calibration: dict | None = None,
    ):
        self.acc_scale = acc_range_g / 32768.0
        self.gyro_scale = gyro_range_dps / 32768.0
        self.angle_scale = angle_range_deg / 32768.0
        self.rotation = validate_rotation(rotation_matrix)
        cal = calibration or {}
        self.gyro_bias = cal.get("gyro_bias_radps", [0.0, 0.0, 0.0])
        self.acc_bias = cal.get("acc_bias_mps2", [0.0, 0.0, 0.0])

        self._current = WitSample()
        self._seen: set[int] = set()
        self._sequence = 0
        self.samples_emitted = 0
        self.packets_applied = 0
        self.packets_invalid = 0
        self.type_counts: dict[str, int] = {}

    def _finalise(self, monotonic_ns_: int, wall_ns: int) -> WitSample | None:
        if not self._seen:
            return None
        s = self._current
        self._sequence += 1
        s.sensor_sequence = self._sequence
        s.host_monotonic_ns = monotonic_ns_
        s.host_wall_time_ns = wall_ns
        s.packets_in_sample = len(self._seen)
        s.packet_type = "+".join(sorted(PACKET_NAMES.get(t, hex(t)) for t in self._seen))
        # A sample is usable when at least acceleration and gyro are present.
        s.sample_valid = bool(
            s.acc_x_mps2 is not None and s.gyro_x_radps is not None and s.checksum_valid
        )
        # Bias correction, then rotation into the vehicle frame.
        if s.acc_x_mps2 is not None:
            ax = s.acc_x_mps2 - self.acc_bias[0]
            ay = s.acc_y_mps2 - self.acc_bias[1]
            az = s.acc_z_mps2 - self.acc_bias[2]
            s.veh_acc_x_mps2, s.veh_acc_y_mps2, s.veh_acc_z_mps2 = _apply_rotation(
                self.rotation, ax, ay, az)
        if s.gyro_x_radps is not None:
            gx = s.gyro_x_radps - self.gyro_bias[0]
            gy = s.gyro_y_radps - self.gyro_bias[1]
            gz = s.gyro_z_radps - self.gyro_bias[2]
            s.veh_gyro_x_radps, s.veh_gyro_y_radps, s.veh_gyro_z_radps = _apply_rotation(
                self.rotation, gx, gy, gz)

        self._current = WitSample()
        self._seen = set()
        self.samples_emitted += 1
        return s

    def feed(self, frame: WitFrame, monotonic_ns_: int, wall_ns: int) -> WitSample | None:
        """Apply one frame; return a completed sample when one closes."""
        self.type_counts[frame.name] = self.type_counts.get(frame.name, 0) + 1

        if not frame.checksum_valid:
            self.packets_invalid += 1
            self._current.checksum_valid = False
            return None

        emitted: WitSample | None = None
        # Repeat of an already-seen type means the previous round is complete.
        if frame.packet_type in self._seen:
            emitted = self._finalise(monotonic_ns_, wall_ns)

        self._apply(frame)
        self._seen.add(frame.packet_type)
        self.packets_applied += 1

        # All expected types collected -> close the sample immediately.
        if emitted is None and self._seen >= set(_SAMPLE_TYPES):
            emitted = self._finalise(monotonic_ns_, wall_ns)
        return emitted

    def flush(self, monotonic_ns_: int, wall_ns: int) -> WitSample | None:
        """Emit whatever is half-assembled (used at STOP)."""
        return self._finalise(monotonic_ns_, wall_ns)

    def _apply(self, frame: WitFrame) -> None:
        s = self._current
        w0, w1, w2, w3 = frame.words()
        t = frame.packet_type

        if t == PacketType.ACC:
            s.raw_acc_x, s.raw_acc_y, s.raw_acc_z = w0, w1, w2
            s.acc_x_mps2 = w0 * self.acc_scale * G_TO_MPS2
            s.acc_y_mps2 = w1 * self.acc_scale * G_TO_MPS2
            s.acc_z_mps2 = w2 * self.acc_scale * G_TO_MPS2
            s.temperature_c = w3 / 100.0
        elif t == PacketType.GYRO:
            s.raw_gyro_x, s.raw_gyro_y, s.raw_gyro_z = w0, w1, w2
            s.gyro_x_radps = w0 * self.gyro_scale * DEG_TO_RAD
            s.gyro_y_radps = w1 * self.gyro_scale * DEG_TO_RAD
            s.gyro_z_radps = w2 * self.gyro_scale * DEG_TO_RAD
        elif t == PacketType.ANGLE:
            s.raw_roll, s.raw_pitch, s.raw_yaw = w0, w1, w2
            s.roll_rad = w0 * self.angle_scale * DEG_TO_RAD
            s.pitch_rad = w1 * self.angle_scale * DEG_TO_RAD
            s.yaw_rad = w2 * self.angle_scale * DEG_TO_RAD
        elif t == PacketType.MAG:
            s.raw_mag_x, s.raw_mag_y, s.raw_mag_z = w0, w1, w2
            s.mag_x, s.mag_y, s.mag_z = float(w0), float(w1), float(w2)
            if s.temperature_c is None:
                s.temperature_c = w3 / 100.0
        elif t == PacketType.QUATERNION:
            s.raw_q0, s.raw_q1, s.raw_q2, s.raw_q3 = w0, w1, w2, w3
            s.quaternion_w = w0 / 32768.0
            s.quaternion_x = w1 / 32768.0
            s.quaternion_y = w2 / 32768.0
            s.quaternion_z = w3 / 32768.0
