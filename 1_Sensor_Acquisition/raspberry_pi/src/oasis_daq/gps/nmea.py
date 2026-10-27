"""NMEA 0183 parsing with strict checksum validation.

Written in-tree rather than pulled from a third-party package so that every
field the DAQ schema promises has an explicit, testable origin, and so that a
malformed sentence degrades to ``checksum_valid=False`` instead of an
exception in the acquisition loop.

Verified against a real u-blox 7 (UBX-G70xx, PROTVER 14.00) emitting
GGA/RMC/VTG/GSA/GSV/GLL/TXT at 1 Hz.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field, asdict
from typing import Any

__all__ = [
    "NmeaSentence",
    "GpsFix",
    "verify_checksum",
    "split_sentence",
    "parse_sentence",
    "GpsAssembler",
    "KNOTS_TO_MPS",
    "KMH_TO_MPS",
]

KNOTS_TO_MPS = 0.514444
KMH_TO_MPS = 1.0 / 3.6

# $ or ! followed by talker+type, comma payload, optional *HH checksum
_SENTENCE_RE = re.compile(r"^[!$]([A-Za-z0-9]{2,})((?:,[^*]*)*)(?:\*([0-9A-Fa-f]{2}))?\s*$")

# Fix quality values from GGA field 6 that mean "the position is usable".
_USABLE_GGA_QUALITY = {1, 2, 3, 4, 5, 6, 7, 8, 9}


def verify_checksum(raw: str) -> tuple[bool, str | None, str | None]:
    """Return ``(ok, computed, declared)`` for one raw sentence.

    The checksum is the XOR of every character between ``$`` and ``*``.  A
    sentence with no ``*HH`` suffix is reported as ``ok=False`` with
    ``declared=None`` so the caller can still keep the raw line.
    """
    line = raw.strip()
    if not line or line[0] not in "$!":
        return False, None, None
    star = line.rfind("*")
    if star == -1 or star + 3 > len(line):
        return False, None, None
    body = line[1:star]
    declared = line[star + 1:star + 3].upper()
    computed = 0
    for ch in body:
        computed ^= ord(ch)
    computed_hex = f"{computed:02X}"
    return computed_hex == declared, computed_hex, declared


def split_sentence(raw: str) -> tuple[str | None, list[str]]:
    """Return ``(sentence_type, fields)`` -- e.g. ``("GPGGA", [...])``."""
    m = _SENTENCE_RE.match(raw.strip())
    if not m:
        return None, []
    stype = m.group(1).upper()
    payload = m.group(2) or ""
    fields = payload.split(",")[1:] if payload else []
    return stype, fields


def _f(value: str | None) -> float | None:
    """Parse a float NMEA field; empty or malformed becomes ``None``."""
    if value is None or value == "":
        return None
    try:
        return float(value)
    except ValueError:
        return None


def _i(value: str | None) -> int | None:
    if value is None or value == "":
        return None
    try:
        return int(float(value))
    except ValueError:
        return None


def _latlon(value: str | None, hemisphere: str | None) -> float | None:
    """Convert ddmm.mmmm / dddmm.mmmm + N/S/E/W into signed decimal degrees."""
    if not value or not hemisphere:
        return None
    try:
        dot = value.index(".")
    except ValueError:
        # Some receivers omit the decimal point when there are no fractions.
        dot = len(value)
    deg_digits = dot - 2
    if deg_digits < 1:
        return None
    try:
        degrees = float(value[:deg_digits])
        minutes = float(value[deg_digits:])
    except ValueError:
        return None
    if minutes >= 60.0:
        return None
    decimal = degrees + minutes / 60.0
    if hemisphere.upper() in ("S", "W"):
        decimal = -decimal
    if hemisphere.upper() in ("N", "S") and not -90.0 <= decimal <= 90.0:
        return None
    if hemisphere.upper() in ("E", "W") and not -180.0 <= decimal <= 180.0:
        return None
    return decimal


def _utc_time(value: str | None) -> str | None:
    """``hhmmss.ss`` -> ``hh:mm:ss.ss``; ``None`` when the field is empty."""
    if not value or len(value) < 6:
        return None
    try:
        hh, mm = int(value[0:2]), int(value[2:4])
        ss = float(value[4:])
    except ValueError:
        return None
    if not (0 <= hh < 24 and 0 <= mm < 60 and 0 <= ss < 61):
        return None
    return f"{hh:02d}:{mm:02d}:{ss:06.3f}"


def _utc_datetime(date_field: str | None, time_field: str | None) -> str | None:
    """RMC ``ddmmyy`` + ``hhmmss.ss`` -> ISO-8601 UTC string."""
    t = _utc_time(time_field)
    if not date_field or len(date_field) != 6 or t is None:
        return None
    try:
        dd, mm, yy = int(date_field[0:2]), int(date_field[2:4]), int(date_field[4:6])
    except ValueError:
        return None
    if not (1 <= dd <= 31 and 1 <= mm <= 12):
        return None
    year = 2000 + yy if yy < 80 else 1900 + yy
    return f"{year:04d}-{mm:02d}-{dd:02d}T{t}Z"


@dataclass
class NmeaSentence:
    """One parsed sentence; ``valid`` only means the checksum verified."""

    raw: str
    sentence_type: str | None
    fields: list[str]
    checksum_valid: bool
    checksum_computed: str | None = None
    checksum_declared: str | None = None
    values: dict[str, Any] = field(default_factory=dict)

    @property
    def formatter(self) -> str | None:
        """Sentence formatter without the talker id, e.g. ``GGA``."""
        if not self.sentence_type:
            return None
        return self.sentence_type[2:] if len(self.sentence_type) > 3 else self.sentence_type


def parse_sentence(raw: str) -> NmeaSentence:
    """Parse one raw NMEA line.  Never raises."""
    ok, computed, declared = verify_checksum(raw)
    stype, fields = split_sentence(raw)
    sentence = NmeaSentence(
        raw=raw.strip(), sentence_type=stype, fields=fields,
        checksum_valid=ok, checksum_computed=computed, checksum_declared=declared,
    )
    if not ok or not stype:
        return sentence

    formatter = sentence.formatter
    try:
        if formatter == "GGA":
            sentence.values = _parse_gga(fields)
        elif formatter == "RMC":
            sentence.values = _parse_rmc(fields)
        elif formatter == "VTG":
            sentence.values = _parse_vtg(fields)
        elif formatter == "GSA":
            sentence.values = _parse_gsa(fields)
        elif formatter == "GSV":
            sentence.values = _parse_gsv(fields)
        elif formatter == "GLL":
            sentence.values = _parse_gll(fields)
        elif formatter == "TXT":
            sentence.values = {"text": ",".join(fields[3:]) if len(fields) > 3 else ""}
    except (IndexError, ValueError):
        # A structurally odd but checksum-valid sentence must not kill the
        # reader; keep the raw line and expose no derived values.
        sentence.values = {}
    return sentence


def _parse_gga(f: list[str]) -> dict[str, Any]:
    quality = _i(f[5]) if len(f) > 5 else None
    return {
        "utc_time": _utc_time(f[0] if len(f) > 0 else None),
        "latitude_deg": _latlon(f[1] if len(f) > 1 else None, f[2] if len(f) > 2 else None),
        "longitude_deg": _latlon(f[3] if len(f) > 3 else None, f[4] if len(f) > 4 else None),
        "fix_quality": quality,
        "satellite_count": _i(f[6]) if len(f) > 6 else None,
        "hdop": _f(f[7]) if len(f) > 7 else None,
        "altitude_m": _f(f[8]) if len(f) > 8 else None,
        "geoid_separation_m": _f(f[10]) if len(f) > 10 else None,
        "fix_valid": bool(quality in _USABLE_GGA_QUALITY),
    }


def _parse_rmc(f: list[str]) -> dict[str, Any]:
    status = (f[1].upper() if len(f) > 1 and f[1] else None)
    speed_knots = _f(f[6]) if len(f) > 6 else None
    return {
        "utc_time": _utc_time(f[0] if len(f) > 0 else None),
        "status": status,
        "fix_valid": status == "A",
        "latitude_deg": _latlon(f[2] if len(f) > 2 else None, f[3] if len(f) > 3 else None),
        "longitude_deg": _latlon(f[4] if len(f) > 4 else None, f[5] if len(f) > 5 else None),
        "ground_speed_mps": (speed_knots * KNOTS_TO_MPS) if speed_knots is not None else None,
        # RMC field order: 0 time, 1 status, 2/3 lat, 4/5 lon, 6 speed(kn),
        # 7 course(deg true), 8 date(ddmmyy), 9/10 magnetic variation.
        "course_deg": _f(f[7]) if len(f) > 7 else None,
        "date": (f[8] if len(f) > 8 and f[8] else None),
        "gps_utc": _utc_datetime(f[8] if len(f) > 8 else None, f[0] if len(f) > 0 else None),
    }


def _parse_vtg(f: list[str]) -> dict[str, Any]:
    knots = _f(f[4]) if len(f) > 4 else None
    kmh = _f(f[6]) if len(f) > 6 else None
    speed = (knots * KNOTS_TO_MPS) if knots is not None else (
        kmh * KMH_TO_MPS if kmh is not None else None)
    return {
        "course_true_deg": _f(f[0]) if len(f) > 0 else None,
        "course_magnetic_deg": _f(f[2]) if len(f) > 2 else None,
        "ground_speed_mps": speed,
    }


def _parse_gsa(f: list[str]) -> dict[str, Any]:
    mode = _i(f[1]) if len(f) > 1 else None
    used = [x for x in f[2:14] if x] if len(f) > 2 else []
    return {
        "fix_mode": mode,                 # 1=no fix, 2=2D, 3=3D
        "satellites_used": len(used),
        "pdop": _f(f[14]) if len(f) > 14 else None,
        "hdop": _f(f[15]) if len(f) > 15 else None,
        "vdop": _f(f[16]) if len(f) > 16 else None,
    }


def _parse_gsv(f: list[str]) -> dict[str, Any]:
    return {
        "total_messages": _i(f[0]) if len(f) > 0 else None,
        "message_number": _i(f[1]) if len(f) > 1 else None,
        "satellites_in_view": _i(f[2]) if len(f) > 2 else None,
    }


def _parse_gll(f: list[str]) -> dict[str, Any]:
    status = (f[5].upper() if len(f) > 5 and f[5] else None)
    return {
        "latitude_deg": _latlon(f[0] if len(f) > 0 else None, f[1] if len(f) > 1 else None),
        "longitude_deg": _latlon(f[2] if len(f) > 2 else None, f[3] if len(f) > 3 else None),
        "utc_time": _utc_time(f[4] if len(f) > 4 else None),
        "fix_valid": status == "A",
    }


@dataclass
class GpsFix:
    """The DAQ-schema row emitted once per position-bearing sentence."""

    host_monotonic_ns: int = 0
    host_wall_time_ns: int = 0
    gps_utc: str | None = None
    latitude_deg: float | None = None
    longitude_deg: float | None = None
    altitude_m: float | None = None
    ground_speed_mps: float | None = None
    course_deg: float | None = None
    fix_quality: int | None = None
    fix_valid: bool = False
    satellite_count: int | None = None
    hdop: float | None = None
    vdop: float | None = None
    pdop: float | None = None
    source_sentence: str = ""
    sequence_id: int = 0
    checksum_valid: bool = True
    # Fields on this row that did NOT come from the sentence that produced it.
    # This receiver emits RMC first in each epoch, so an RMC row's altitude and
    # fix_quality come from the PREVIOUS second's GGA.  Without this the row
    # looks internally consistent when it is not: the first RMC after a fix is
    # acquired reports fix_valid=1 next to last second's fix_quality=0.
    carried_fields: str = ""

    def to_row(self) -> dict[str, Any]:
        row = asdict(self)
        row["fix_valid"] = int(bool(self.fix_valid))
        row["checksum_valid"] = int(bool(self.checksum_valid))
        return row


CSV_FIELDS = list(GpsFix().to_row().keys())


class GpsAssembler:
    """Fuse the sentence types into one row per position update.

    GGA carries altitude and fix quality, RMC carries speed/course/date, GSA
    carries the DOP triple.  A row is emitted when a position-bearing sentence
    (GGA or RMC) arrives, carrying the most recent complementary values.
    """

    def __init__(self) -> None:
        self._sequence = 0
        self._pdop: float | None = None
        self._vdop: float | None = None
        self._hdop: float | None = None
        self._satellites_in_view: int | None = None
        self._last: GpsFix | None = None
        self.sentence_counts: dict[str, int] = {}
        self.checksum_failures = 0
        self.sentences_seen = 0

    @property
    def last_fix(self) -> GpsFix | None:
        return self._last

    @property
    def satellites_in_view(self) -> int | None:
        return self._satellites_in_view

    def feed(self, raw: str, monotonic_ns_: int, wall_ns: int) -> GpsFix | None:
        """Consume one raw line; return a row when one is complete."""
        sentence = parse_sentence(raw)
        self.sentences_seen += 1
        if sentence.sentence_type:
            self.sentence_counts[sentence.sentence_type] = (
                self.sentence_counts.get(sentence.sentence_type, 0) + 1)
        if not sentence.checksum_valid:
            self.checksum_failures += 1
            return None

        formatter = sentence.formatter
        v = sentence.values

        if formatter == "GSA":
            self._pdop = v.get("pdop", self._pdop)
            self._hdop = v.get("hdop", self._hdop)
            self._vdop = v.get("vdop", self._vdop)
            return None
        if formatter == "GSV":
            if v.get("satellites_in_view") is not None:
                self._satellites_in_view = v["satellites_in_view"]
            return None
        if formatter == "VTG":
            if self._last is not None:
                if v.get("ground_speed_mps") is not None:
                    self._last.ground_speed_mps = v["ground_speed_mps"]
                if v.get("course_true_deg") is not None:
                    self._last.course_deg = v["course_true_deg"]
            return None
        if formatter not in ("GGA", "RMC"):
            return None

        self._sequence += 1
        prev = self._last
        fix = GpsFix(
            host_monotonic_ns=monotonic_ns_,
            host_wall_time_ns=wall_ns,
            sequence_id=self._sequence,
            source_sentence=sentence.sentence_type or "",
            checksum_valid=True,
            pdop=self._pdop,
            vdop=self._vdop,
            hdop=v.get("hdop", self._hdop),
        )

        if formatter == "GGA":
            # GGA carries its own HDOP; remember it so the next RMC row (which
            # has no DOP fields of its own) reports the same value.
            if v.get("hdop") is not None:
                self._hdop = v["hdop"]
                fix.hdop = v["hdop"]
            fix.latitude_deg = v.get("latitude_deg")
            fix.longitude_deg = v.get("longitude_deg")
            fix.altitude_m = v.get("altitude_m")
            fix.fix_quality = v.get("fix_quality")
            fix.satellite_count = v.get("satellite_count")
            fix.fix_valid = bool(v.get("fix_valid"))
            fix.gps_utc = prev.gps_utc if prev else None
            # Speed/course only ever come from RMC/VTG.
            carried: list[str] = []
            if prev is not None:
                if prev.ground_speed_mps is not None:
                    fix.ground_speed_mps = prev.ground_speed_mps
                    carried.append("ground_speed_mps")
                if prev.course_deg is not None:
                    fix.course_deg = prev.course_deg
                    carried.append("course_deg")
                if prev.gps_utc is not None:
                    carried.append("gps_utc")
            fix.carried_fields = "|".join(carried)
        else:  # RMC
            fix.latitude_deg = v.get("latitude_deg")
            fix.longitude_deg = v.get("longitude_deg")
            fix.ground_speed_mps = v.get("ground_speed_mps")
            fix.course_deg = v.get("course_deg")
            fix.fix_valid = bool(v.get("fix_valid"))
            fix.gps_utc = v.get("gps_utc")
            carried = []
            if prev is not None:
                if prev.altitude_m is not None:
                    fix.altitude_m = prev.altitude_m
                    carried.append("altitude_m")
                if prev.fix_quality is not None:
                    fix.fix_quality = prev.fix_quality
                    carried.append("fix_quality")
                if prev.satellite_count is not None:
                    fix.satellite_count = prev.satellite_count
                    carried.append("satellite_count")
            fix.carried_fields = "|".join(carried)

        self._last = fix
        return fix
