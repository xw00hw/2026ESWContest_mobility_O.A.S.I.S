"""BeamNG collector v0.3 vehicle rows: strict parsing, clocks and quality gates.

Vehicle axes are +x forward, +y left, +z up. Acceleration is the collector's
gravity-excluded velocity derivative, not physical occupant/pressure feedback.
Importing this module never opens a socket, file, GUI or device.
"""
from __future__ import annotations

import csv
from dataclasses import dataclass, replace
import json
import math
from pathlib import Path
import re
import socket
from typing import Iterator, Mapping

BRIDGE_SCHEMA = "oasis.beamng.fallback.v1"
BRIDGE_HOST, BRIDGE_PORT = "127.0.0.1", 64257
MAX_DATAGRAM_BYTES = 16384
CLOCK_ID = "host_perf_counter_ns_same_machine"
REQUIRED_COLUMNS = frozenset(("host_monotonic_ns", "sim_time_s", "stream_id", "vehicle_id",
    "sequence_id", "source_kind", "linear_acc_y_mps2", "linear_acc_x_mps2", "speed_mps",
    "derivative_valid", "derivative_reason", "config_match", "sample_dt_s"))


@dataclass(frozen=True, slots=True)
class VehicleFrame:
    received_monotonic_ns: int
    sim_time_s: float
    stream_id: str
    vehicle_id: str
    sequence_id: int
    ay_mps2: float | None
    ax_mps2: float | None
    speed_mps: float | None
    steering_deg: float | None
    throttle: float | None
    brake: float | None
    valid: bool
    reason: str
    source_kind: str


def _integer(value, field):
    if type(value) is int:
        result = value
    elif isinstance(value, str) and re.fullmatch(r"[0-9]{1,20}", value):
        result = int(value)
    else:
        raise ValueError("Invalid integer: " + field)
    if not 0 <= result <= 0x7fffffffffffffff:
        raise ValueError("Integer outside supported range: " + field)
    return result


def _number(value):
    if value is None or value == "":
        return None
    if isinstance(value, bool) or not isinstance(value, (str, int, float)):
        raise ValueError("Expected a finite numeric value")
    try:
        result = float(value)
    except (ValueError, OverflowError) as exc:
        raise ValueError("Expected a finite numeric value") from exc
    if not math.isfinite(result):
        raise ValueError("Expected a finite numeric value")
    return result


def _identifier(value, field, *, numeric=False):
    if numeric and type(value) is int and value >= 0:
        value = str(value)
    if not isinstance(value, str) or not value or len(value) > 1024 or any(ord(x) < 32 for x in value):
        raise ValueError("Invalid identifier: " + field)
    return value


class VehicleValidator:
    """One active stream with boundary recovery; never restamps received data.

    Malformed essential clocks/identities raise ValueError. Usable structural
    rows with a failed quality/age/motion gate return an invalid VehicleFrame.
    The first row of a stream is invalid; the next good row can recover. Policy
    stability dwell and actuator rate limits remain separate responsibilities.
    """
    def __init__(self, *, allow_synthetic=False, max_age_ms=250, max_host_gap_ms=250,
                 max_sim_dt_s=.25, max_abs_ay=12, max_abs_ax=15, min_speed_mps=1):
        if type(allow_synthetic) is not bool:
            raise ValueError("allow_synthetic must be an explicit bool")
        values = (max_age_ms, max_host_gap_ms, max_sim_dt_s, max_abs_ay, max_abs_ax, min_speed_mps)
        if any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) or x <= 0 for x in values):
            raise ValueError("Validator limits must be finite positive numbers")
        self.allow_synthetic = allow_synthetic
        self.max_age_ns = round(max_age_ms*1_000_000)
        self.max_host_gap_ns = round(max_host_gap_ms*1_000_000)
        self.max_sim_dt_s, self.max_abs_ay, self.max_abs_ax = max_sim_dt_s, max_abs_ay, max_abs_ax
        self.min_speed_mps = min_speed_mps
        self.reset()

    def reset(self):
        self._last = None
        self._retired = set()

    def parse(self, row: Mapping, now_ns: int) -> VehicleFrame:
        if not isinstance(row, Mapping):
            raise ValueError("Vehicle row must be a mapping")
        now_ns = _integer(now_ns, "now_ns")
        received = _integer(row.get("host_monotonic_ns"), "host_monotonic_ns")
        sequence = _integer(row.get("sequence_id"), "sequence_id")
        sim_time = _number(row.get("sim_time_s"))
        if sim_time is None or sim_time < 0:
            raise ValueError("Invalid simulation time")
        stream = _identifier(row.get("stream_id"), "stream_id")
        vehicle = _identifier(row.get("vehicle_id"), "vehicle_id", numeric=True)
        source = _identifier(row.get("source_kind"), "source_kind")
        numbers, malformed = {}, []
        fields = dict(ay_mps2="linear_acc_y_mps2", ax_mps2="linear_acc_x_mps2", speed_mps="speed_mps",
                      steering_deg="steering_angle_deg", throttle="throttle_input", brake="brake_input")
        for dest, name in fields.items():
            try:
                numbers[dest] = _number(row.get(name))
            except ValueError:
                numbers[dest] = None
                malformed.append(name)
        frame = VehicleFrame(received, sim_time, stream, vehicle, sequence, **numbers,
                             valid=False, reason="unvalidated", source_kind=source)
        invalid = lambda reason: replace(frame, reason=reason)
        allowed = ("beamng", "synthetic_test") if self.allow_synthetic else ("beamng",)
        if source not in allowed:
            return invalid("unsupported_source")
        if received > now_ns:
            return invalid("future_timestamp")
        if now_ns-received > self.max_age_ns:
            return invalid("stale")

        session = row.get("_bridge_session_id", "")
        config_hash = row.get("_bridge_config_sha256", "")
        if not isinstance(session, str) or not isinstance(config_hash, str):
            raise ValueError("Invalid bridge context")
        identity = (session, stream, vehicle)
        boundary = None
        previous = self._last
        if previous is None:
            boundary = "first_sample"
        elif identity != previous[0]:
            if identity in self._retired:
                return invalid("retired_stream")
            self._retired.add(previous[0])
            if len(self._retired) > 4096:
                raise ValueError("Too many stream transitions; explicitly reset validator")
            boundary = "stream_changed"
        else:
            if sequence <= previous[1]:
                return invalid("duplicate_or_reordered")
            if received <= previous[2]:
                return invalid("nonincreasing_host_time")
            if received-previous[2] > self.max_host_gap_ns:
                boundary = "host_gap"
            elif sim_time <= previous[3]:
                boundary = "nonincreasing_sim_time"
            elif sim_time-previous[3] > self.max_sim_dt_s+1e-12:
                boundary = "long_sim_gap"
            elif sequence != previous[1]+1:
                boundary = "sequence_gap"
            elif config_hash != previous[4]:
                boundary = "config_changed"
        # A well-formed fresh row anchors recovery, even if its derivative or
        # acceleration is unusable. Reordered/future/stale rows never do.
        self._last = (identity, sequence, received, sim_time, config_hash)
        if boundary not in (None, "first_sample", "stream_changed"):
            return invalid(boundary)
        if malformed:
            return invalid("invalid_number:"+malformed[0])
        try:
            config_match = _integer(row.get("config_match"), "config_match")
            derivative_valid = _integer(row.get("derivative_valid"), "derivative_valid")
        except ValueError:
            return invalid("invalid_quality_flags")
        if config_match != 1:
            return invalid("config_mismatch")
        if derivative_valid != 1 or row.get("derivative_reason") != "ok":
            return invalid("invalid_derivative")
        try:
            sample_dt = _number(row.get("sample_dt_s"))
            missing = _integer(row.get("missing_packets_before", 0), "missing_packets_before")
        except ValueError:
            return invalid("invalid_sample_interval")
        if sample_dt is None or not 0 < sample_dt <= self.max_sim_dt_s+1e-12:
            return invalid("invalid_sample_interval")
        if missing:
            return invalid("source_packet_gap")
        for name in ("ay_mps2", "ax_mps2", "speed_mps"):
            if numbers[name] is None:
                return invalid("missing:"+name)
        if abs(frame.ay_mps2) > self.max_abs_ay:
            return invalid("lateral_acceleration_outlier")
        if abs(frame.ax_mps2) > self.max_abs_ax:
            return invalid("longitudinal_acceleration_outlier")
        if boundary:
            return invalid(boundary)
        if frame.speed_mps < 0:
            return invalid("negative_speed")
        if frame.speed_mps < self.min_speed_mps:
            return invalid("stopped")
        return replace(frame, valid=True, reason="ok")


def read_vehicle_csv(path) -> Iterator[dict]:
    """Yield the original string cells, including exact integer-ns timestamps."""
    with Path(path).open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        fields = reader.fieldnames or []
        if len(fields) != len(set(fields)) or not REQUIRED_COLUMNS <= set(fields):
            raise ValueError("CSV does not have the collector vehicle quality/time columns")
        for row in reader:
            if None in row or any(value is None for value in row.values()):
                raise ValueError("Incomplete or malformed vehicle CSV row")
            yield row


def _json_object(pairs):
    value = {}
    for key, item in pairs:
        if key in value:
            raise ValueError("Duplicate JSON key: " + key)
        value[key] = item
    return value


def read_bridge_packet(payload: bytes) -> dict:
    """Validate the bridge envelope, preserving the collector receive timestamp."""
    if not isinstance(payload, bytes) or not 0 < len(payload) <= MAX_DATAGRAM_BYTES:
        raise ValueError("Invalid/oversized bridge datagram")
    def reject_constant(value):
        raise ValueError("Nonfinite JSON constant: " + value)
    def finite_float(value):
        parsed = float(value)
        if not math.isfinite(parsed):
            raise ValueError("Nonfinite JSON number")
        return parsed
    try:
        packet = json.loads(payload.decode("utf-8"), parse_constant=reject_constant,
                            parse_float=finite_float, object_pairs_hook=_json_object)
    except (UnicodeError, json.JSONDecodeError, RecursionError) as exc:
        raise ValueError("Malformed bridge JSON") from exc
    if not isinstance(packet, dict) or packet.get("schema") != BRIDGE_SCHEMA:
        raise ValueError("Unsupported bridge schema")
    if packet.get("collector_version") != "0.3" or type(packet.get("csv_revision")) is not int or packet["csv_revision"] != 2:
        raise ValueError("Unsupported collector version/revision")
    if packet.get("clock") != CLOCK_ID:
        raise ValueError("Unsupported bridge clock")
    forwarded = _integer(packet.get("forwarded_monotonic_ns"), "forwarded_monotonic_ns")
    provenance, row = packet.get("provenance"), packet.get("row")
    # Invalid derivative rows intentionally omit acceleration keys. They must
    # still reach the validator so an active policy can withdraw its demand.
    row_keys = REQUIRED_COLUMNS-{"linear_acc_y_mps2", "linear_acc_x_mps2", "speed_mps"}
    if not isinstance(provenance, dict) or not isinstance(row, dict) or not row_keys <= row.keys():
        raise ValueError("Missing bridge provenance/vehicle row")
    session = _identifier(provenance.get("session_id"), "session_id")
    digest = provenance.get("config_sha256")
    hashes = provenance.get("source_sha256")
    if not isinstance(digest, str) or not re.fullmatch(r"[a-f0-9]{64}", digest):
        raise ValueError("Missing configuration provenance hash")
    if not isinstance(hashes, dict) or not hashes or not all(
        isinstance(k, str) and isinstance(v, str) and re.fullmatch(r"[a-f0-9]{64}", v) for k, v in hashes.items()):
        raise ValueError("Missing collector source hashes")
    if any(name.startswith("_bridge_") for name in row):
        raise ValueError("Reserved bridge fields in source row")
    received = _integer(row["host_monotonic_ns"], "host_monotonic_ns")
    if forwarded < received:
        raise ValueError("Bridge forwarding timestamp precedes source receive timestamp")
    result = dict(row)
    result.update(_bridge_session_id=session, _bridge_config_sha256=digest)
    return result


class BridgeReceiver:
    """Explicit, exclusive, nonblocking loopback receiver; never binds 64256."""
    def __init__(self, *, socket_factory=socket.socket):
        self.socket = socket_factory(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
                self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 64*1024)
            self.socket.bind((BRIDGE_HOST, BRIDGE_PORT))
            self.socket.setblocking(False)
        except Exception:
            self.socket.close()
            raise

    def receive(self):
        try:
            payload, peer = self.socket.recvfrom(MAX_DATAGRAM_BYTES+1)
        except (BlockingIOError, socket.timeout):
            return None
        except OSError as exc:
            if getattr(exc, "winerror", None) == 10040:
                raise ValueError("Oversized bridge datagram") from exc
            raise
        if peer[0] != BRIDGE_HOST:
            raise ValueError("Bridge peer is not loopback")
        return read_bridge_packet(payload)

    def close(self):
        self.socket.close()

    def __enter__(self):
        return self

    def __exit__(self, *unused):
        self.close()
