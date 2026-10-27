#!/usr/bin/env python3
"""Put separately recorded Pi / pressure-seat / OBD sessions on one UTC timeline.

Each device stamps its rows with its own ``host_monotonic_ns`` and
``host_wall_time_ns``.  For every device this tool computes

    t_utc = host_monotonic_ns + median(host_wall_time_ns - host_monotonic_ns) - clock_offset

and writes one CSV on a fixed-rate grid over the time all streams overlap.
Each grid row carries the most recent sample of every stream (as-of join, no
interpolation) plus ``<stream>_age_ms`` so stale values stay visible.  The
session directories are only read, never modified.

``clock_offset`` (host clock minus true UTC) defaults to ``auto``:
  seat, obd  the NTP offset the recorder measured at session start/end
  pi         0 if the Pi clock agrees with GPS time within 2 s, otherwise the
             GPS-derived offset (a Pi with no network has no valid wall clock)
Pass a number of seconds to override, e.g. ``--pi-offset-s 0.18``.

    py align_sessions.py --pi <pi session> --seat <seat session> --obd <obd session> --out aligned.csv
    py align_sessions.py --selftest

Standard library only.
"""
from __future__ import annotations

import argparse
import csv
import json
import statistics
import sys
import tempfile
from bisect import bisect_right
from datetime import datetime, timedelta, timezone
from pathlib import Path

# device -> (csv relative to the session directory, output column prefix, time column)
STREAMS = {
    "pi": [("imu/imu.csv", "imu", "host_monotonic_ns"),
           ("gps/gps.csv", "gps", "host_monotonic_ns"),
           ("pose/features.csv", "pose", "capture_monotonic_ns"),
           ("camera/frame_timestamps.csv", "cam", "capture_monotonic_ns")],
    "seat": [("seat/pressure.csv", "seat", "host_monotonic_ns")],
    "obd": [("obd/obd.csv", "obd", "host_monotonic_ns")],
}
HOST_COLUMNS = ("host_monotonic_ns", "host_wall_time_ns")
GPS_TRUST_NS = 2 * 10**9
UNIX_EPOCH = datetime(1970, 1, 1, tzinfo=timezone.utc)


def full_rows(path: Path):
    """Yield (header, row) for complete rows only; a crashed session can end mid-line."""
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.reader(handle)
        header = next(reader, None)
        for row in reader:
            if len(row) == len(header):
                yield header, row


def iso(ns: int) -> str:
    return datetime.fromtimestamp(ns / 1e9, timezone.utc).isoformat(timespec="milliseconds")


def gps_offset_ns(pi_dir: Path, epoch: int) -> int | None:
    """Pi clock minus GPS UTC, from valid RMC rows.  Includes the receiver's
    NMEA output delay (unmeasured, typically 0.1-0.4 s), so it reads high."""
    path, diffs = pi_dir / "gps" / "gps.csv", []
    if not path.exists():
        return None
    for header, row in full_rows(path):
        r = dict(zip(header, row))
        if (r["fix_valid"] == "1" and r["gps_utc"] and r["source_sentence"].endswith("RMC")
                and "gps_utc" not in r.get("carried_fields", "")):
            utc = datetime.fromisoformat(r["gps_utc"].replace("Z", "+00:00"))
            utc_ns = (utc - UNIX_EPOCH) // timedelta(microseconds=1) * 1000   # exact, no float
            diffs.append(int(r["host_monotonic_ns"]) + epoch - utc_ns)
    return statistics.median_low(diffs) if diffs else None


def ntp_offset_ns(session_dir: Path) -> int | None:
    found = []
    for name, key in (("metadata.json", "clock"), ("session_summary.json", None)):
        try:
            data = json.loads((session_dir / name).read_text(encoding="utf-8"))
        except (OSError, ValueError):
            continue
        for entry in ((data.get(key) or {}).get("ntp_start"), data.get("ntp_end")):
            if entry:
                found.append(entry["offset_ns"])
    return round(statistics.mean(found)) if found else None


def align(dirs: dict[str, Path], out: Path, hz: float = 20.0, max_age_s: float = 1.5,
          offsets: dict[str, str] | None = None, say=print) -> int:
    """Write the aligned CSV; returns the number of rows."""
    streams = []                      # (prefix, path, header, time index, utc times)
    for device, session_dir in dirs.items():
        loaded, diffs = [], []
        for relative, prefix, time_col in STREAMS[device]:
            path = Path(session_dir) / relative
            if not path.exists():
                say(f"  {device}/{relative}: missing, skipped")
                continue
            header, times = None, []
            for header, row in full_rows(path):
                times.append(int(row[header.index(time_col)]))
                if not loaded and all(c in header for c in HOST_COLUMNS):
                    diffs.append(int(row[header.index(HOST_COLUMNS[1])])
                                 - int(row[header.index(HOST_COLUMNS[0])]))
            if times:
                loaded.append((prefix, path, header, header.index(time_col), times))
            else:
                say(f"  {device}/{relative}: no rows, skipped")
        if not diffs:
            raise SystemExit(f"{device}: no stream with both host time columns in {session_dir}")
        # ponytail: one constant per session, so drift between the host's
        # monotonic and wall clocks (tens of ppm) is averaged, not removed.
        # Fit a line here if sessions get long enough for that to matter.
        epoch = statistics.median_low(diffs)

        choice = (offsets or {}).get(device, "auto")
        if choice != "auto":
            offset, why = round(float(choice) * 1e9), "given on the command line"
        elif device == "pi":
            gps = gps_offset_ns(Path(session_dir), epoch)
            if gps is None:
                offset, why = 0, "WARNING: no valid GPS fix to check the Pi clock against"
            elif abs(gps) > GPS_TRUST_NS:
                offset, why = gps, "Pi clock was not synchronised; using GPS time"
            else:
                offset, why = 0, f"Pi clock agrees with GPS (GPS-derived {gps / 1e9:+.3f} s)"
        else:
            ntp = ntp_offset_ns(Path(session_dir))
            offset, why = (ntp, "NTP offset measured by the recorder") if ntp is not None else (
                0, "WARNING: no NTP measurement in this session; host clock trusted as-is")
        say(f"{device}: clock offset {offset / 1e9:+.3f} s  ({why})")
        for prefix, path, header, index, times in loaded:
            utc = [t + epoch - offset for t in times]
            say(f"  {prefix:5s} {len(utc):8d} rows  {iso(utc[0])} .. {iso(utc[-1])}")
            streams.append((prefix, path, header, index, utc))

    start = max(s[4][0] for s in streams)
    end = min(s[4][-1] for s in streams)
    if end <= start:
        raise SystemExit("the streams do not overlap in time; check the clock offsets above")
    step, max_age = round(1e9 / hz), round(max_age_s * 1e9)

    readers = [[full_rows(path), -1, None] for _, path, *_ in streams]   # iterator, index, row
    kept = [[i for i, name in enumerate(header) if name not in HOST_COLUMNS]
            for _, _, header, _, _ in streams]
    with Path(out).open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["t_utc_ns", "t_utc"] + [
            name for (prefix, _, header, _, _), keep in zip(streams, kept)
            for name in [f"{prefix}_age_ms"] + [f"{prefix}_{header[i]}" for i in keep]])
        count = 0
        for t in range(start, end + 1, step):
            line = [t, iso(t)]
            for (_, _, _, _, utc), reader, keep in zip(streams, readers, kept):
                target = bisect_right(utc, t) - 1          # latest sample at or before t
                while reader[1] < target:
                    reader[2] = next(reader[0])[1]
                    reader[1] += 1
                age = t - utc[target]
                line += ([round(age / 1e6, 1)] + [reader[2][i] for i in keep]
                         if age <= max_age else [""] * (len(keep) + 1))
            writer.writerow(line)
            count += 1
    say(f"overlap {iso(start)} .. {iso(end)}  ({(end - start) / 1e9:.1f} s) "
        f"-> {count} rows at {hz:g} Hz in {out}")
    return count


def selftest() -> None:
    """Two devices with wrong clocks record the same ramp (value = true time in
    ms); after alignment both must read the grid time again."""
    true0 = 1_790_000_000 * 10**9
    with tempfile.TemporaryDirectory() as tmp:
        pi, seat, out = Path(tmp, "pi"), Path(tmp, "seat"), Path(tmp, "aligned.csv")
        for sub in (pi / "imu", pi / "gps", seat / "seat"):
            sub.mkdir(parents=True)
        pi_wrong, seat_wrong = -100 * 10**9, 250 * 10**6       # host clock minus true UTC
        with (pi / "imu" / "imu.csv").open("w", newline="") as f:      # 10 Hz, mono epoch 5 s
            w = csv.writer(f)
            w.writerow(["host_monotonic_ns", "host_wall_time_ns", "ramp_ms"])
            for k in range(300):
                t = true0 + k * 10**8
                w.writerow([5 * 10**9 + k * 10**8, t + pi_wrong, k * 100])
        with (pi / "gps" / "gps.csv").open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["host_monotonic_ns", "host_wall_time_ns", "gps_utc", "fix_valid",
                        "source_sentence", "carried_fields"])
            for k in range(30):
                t = true0 + k * 10**9
                w.writerow([5 * 10**9 + k * 10**9, t + pi_wrong,
                            datetime.fromtimestamp(t / 1e9, timezone.utc)
                            .isoformat(timespec="milliseconds").replace("+00:00", "Z"),
                            1, "GPRMC", ""])
        with (seat / "seat" / "pressure.csv").open("w", newline="") as f:  # 100 Hz, starts 3 s late
            w = csv.writer(f)
            w.writerow(["host_monotonic_ns", "host_wall_time_ns", "ramp_ms"])
            for k in range(2000):
                t = true0 + 3 * 10**9 + k * 10**7
                w.writerow([9 * 10**12 + k * 10**7, t + seat_wrong, 3000 + k * 10])
            f.write("123,45")                                    # torn last line
        (seat / "metadata.json").write_text(json.dumps(
            {"clock": {"ntp_start": {"offset_ns": seat_wrong}}}))
        rows = align({"pi": pi, "seat": seat}, out, hz=20, say=lambda _: None)
        table = list(csv.DictReader(out.open()))
        assert rows == len(table) and 390 <= rows <= 401, rows   # ~20 s of overlap
        for r in table:
            true_ms = (int(r["t_utc_ns"]) - true0) / 1e6
            assert 0 <= true_ms - float(r["imu_ramp_ms"]) < 100, r
            assert 0 <= true_ms - float(r["seat_ramp_ms"]) < 10, r
            assert abs(float(r["seat_age_ms"]) - (true_ms - float(r["seat_ramp_ms"]))) < 0.2
    print("selftest ok")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    for device in STREAMS:
        ap.add_argument(f"--{device}", type=Path, help=f"{device} session directory")
        ap.add_argument(f"--{device}-offset-s", default="auto",
                        help="host clock minus true UTC in seconds (default: auto)")
    ap.add_argument("--out", type=Path, default=Path("aligned.csv"))
    ap.add_argument("--hz", type=float, default=20.0, help="output grid rate")
    ap.add_argument("--max-age-s", type=float, default=1.5,
                    help="blank a stream's columns when its latest sample is older than this")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()
    if args.selftest:
        selftest()
        return 0
    dirs = {d: getattr(args, d) for d in STREAMS if getattr(args, d)}
    if not dirs:
        ap.error("give at least one of --pi, --seat, --obd")
    align(dirs, args.out, args.hz, args.max_age_s,
          {d: getattr(args, f"{d}_offset_s") for d in dirs})
    return 0


if __name__ == "__main__":
    sys.exit(main())
