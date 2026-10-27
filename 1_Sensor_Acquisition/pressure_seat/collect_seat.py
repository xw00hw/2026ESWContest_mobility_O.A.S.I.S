#!/usr/bin/env python3
"""Headless driving-session recorder for the 12-FSR pressure seat.

NUCLEO-H723ZG running FSR_12_SCAN v9 -> ST-LINK virtual COM port -> this script
-> one session directory with the same layout and time columns as the Raspberry
Pi DAQ (``host_monotonic_ns`` / ``host_wall_time_ns``), so the streams can be
aligned afterwards with ``../align_sessions.py``.

No GUI, no pressure calibration, no filtering: raw ADC counts and the voltage
estimate ``raw * vref / 4095`` are stored as received.  The live viewer stays in
``03. Pressure_seat/``; this folder does not import from it.

    py collect_seat.py                       # auto-detect the ST-LINK port, Ctrl+C to stop
    py collect_seat.py --port COM7 --suffix driverA_run1
    py collect_seat.py --list-ports
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import platform
import socket
import struct
import subprocess
import sys
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path

import serial
from serial.tools import list_ports

HERE = Path(__file__).resolve().parent
SCHEMA = "oasis-seat/0.1"
BAUD = 460800
FIRMWARE_ID = "#HELLO,FSR_12_SCAN,v9,bits=12,hz=100,banks=3,max_channels=16"
STOP_ACK = "#CONTROL,12,MASK=0,EN=1"
PING_S = 0.5      # the board disables the MUX after 3 s without a heartbeat
STALE_S = 3.0     # no packet for this long -> reconnect


def stamps() -> tuple[int, int]:
    """``(host_monotonic_ns, host_wall_time_ns)`` sampled back-to-back.

    perf_counter, not time.monotonic: on Windows before Python 3.13 the latter
    ticks in ~15.6 ms steps, coarser than the 10 ms frame period.
    """
    return time.perf_counter_ns(), time.time_ns()


def ntp_offset(server: str, tries: int = 3, timeout: float = 2.0) -> dict | None:
    """Host wall clock minus NTP time (positive = host is ahead); None if unreachable.

    Same function as in ``02. OBD2 Analysis&Code/src/obd2_logger.py``.
    """
    best = None
    for _ in range(tries):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                sock.settimeout(timeout)
                sent = time.time_ns()
                sock.sendto(b"\x1b" + 47 * b"\0", (server, 123))
                reply = sock.recv(48)
                received = time.time_ns()
            seconds, fraction = struct.unpack("!II", reply[40:48])  # transmit timestamp
        except (OSError, struct.error):
            break
        server_ns = (seconds - 2208988800) * 10**9 + (fraction * 10**9 >> 32)
        rtt = received - sent
        if best is None or rtt < best["rtt_ns"]:
            # ponytail: assumes a symmetric path, so the error is up to rtt/2.
            best = {"offset_ns": (sent + received) // 2 - server_ns, "rtt_ns": rtt,
                    "server": server, "host_wall_time_ns": received}
    return best


def parse_packet(line: str):
    """``seq,t_ms,channel,raw0,status0,raw1,status1,raw2,status2`` -> typed fields.

    A failed conversion has an empty raw field and a non-zero status; it is
    never replaced by 0.
    """
    fields = line.split(",")
    if len(fields) != 9:
        raise ValueError("expected nine CSV fields")
    seq, tick, channel = map(int, fields[:3])
    if not (0 <= seq <= 0xFFFFFFFF and 0 <= tick <= 0xFFFFFFFF and 0 <= channel <= 15):
        raise ValueError("packet header outside range")
    readings = []
    for bank in range(3):
        raw_text, status_text = fields[3 + 2 * bank:5 + 2 * bank]
        raw, status = (int(raw_text) if raw_text else None), int(status_text)
        if not 0 <= status <= 5 or (status == 0) != (raw is not None):
            raise ValueError("raw/status mismatch")
        if raw is not None and not 0 <= raw <= 4095:
            raise ValueError("raw outside 12-bit range")
        readings.append((raw, status))
    return seq, tick, channel, readings


def load_config(path: Path) -> dict:
    config = json.loads(Path(path).read_text(encoding="utf-8"))
    places, ids = set(), set()
    for row in config["mapping"]:
        place = (row["bank"], row["channel"])
        if not (0 <= row["bank"] <= 2 and 0 <= row["channel"] <= 15):
            raise ValueError(f"{row['id']}: bank must be 0-2 and channel 0-15")
        if place in places or row["id"] in ids:
            raise ValueError(f"{row['id']}: duplicate sensor id or bank/channel")
        places.add(place)
        ids.add(row["id"])
    channels = {row["channel"] for row in config["mapping"] if row["enabled"]}
    if not 1 <= len(channels) <= 4:
        raise ValueError("enable sensors on 1 to 4 MUX channels (100 Hz firmware limit)")
    if not config["vref"] > 0:
        raise ValueError("vref must be positive")
    return config


def find_port() -> str:
    ports = [p for p in list_ports.comports()
             if p.vid == 0x0483 and "STLINK" in str(p.description).upper().replace("-", "")]
    if len(ports) != 1:
        raise SystemExit(f"found {len(ports)} ST-LINK ports; pass --port (see --list-ports)")
    return ports[0].device


class Link:
    """Line-oriented serial I/O; every received line is stamped and logged raw."""

    def __init__(self, port, rawlog):
        self.port, self.rawlog = port, rawlog
        self.buffer, self.pending = bytearray(), deque()

    def send(self, command: str) -> None:
        self.port.write((command + "\n").encode("ascii"))
        self.port.flush()

    def poll(self) -> None:
        chunk = self.port.read(max(1, min(self.port.in_waiting, 4096)))
        if not chunk:
            return
        # ponytail: every line of one USB chunk shares the chunk's receive time
        # (a few ms of jitter).  Fit mcu_t_ms against it if sub-ms timing matters.
        mono, wall = stamps()
        self.buffer += chunk
        if len(self.buffer) > 65536:
            raise RuntimeError("no line ending in 64 KiB of serial data")
        *lines, rest = self.buffer.split(b"\n")
        self.buffer = bytearray(rest)
        for line in lines:
            text = line.decode("ascii", errors="replace").strip()
            if text:
                self.rawlog.write(f"{mono} {wall} {text}\n")
                self.pending.append((mono, wall, text))

    def expect(self, wanted: str, timeout: float = 2.5) -> None:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.poll()
            while self.pending:
                text = self.pending.popleft()[2]
                if text == wanted:
                    return
                if text.startswith("#HELLO,"):
                    raise RuntimeError(f"not the 12-sensor v9 firmware: {text}")
        raise RuntimeError(f"no reply from the board (expected {wanted})")


class Session:
    """One session directory: files, frame assembly and counters."""

    def __init__(self, data_root: Path, config: dict, suffix: str = ""):
        self.config, self.vref = config, float(config["vref"])
        # dict order = config order = CSV column order
        self.sensors = {(r["bank"], r["channel"]): r["id"]
                        for r in config["mapping"] if r["enabled"]}
        self.channels = sorted({channel for _, channel in self.sensors})
        self.mask = sum(1 << c for c in self.channels)
        self.start_ack = f"#CONTROL,12,MASK={self.mask},EN=0"

        when = datetime.now()
        self.session_id = f"session_{when:%Y%m%d_%H%M%S}" + (f"_{suffix}" if suffix else "")
        self.root = Path(data_root) / "sessions" / f"{when:%Y-%m-%d}" / self.session_id
        (self.root / "seat").mkdir(parents=True)      # never reuses a directory
        (self.root / "INCOMPLETE").write_text("session open or interrupted\n")
        self.rawlog = (self.root / "seat" / "raw_uart.log").open("w", encoding="ascii",
                                                                 errors="replace")
        self.csvfile = (self.root / "seat" / "pressure.csv").open("w", encoding="utf-8",
                                                                  newline="")
        self.eventfile = (self.root / "events.jsonl").open("w", encoding="utf-8")
        ids = list(self.sensors.values())
        self.writer = csv.writer(self.csvfile)
        self.writer.writerow(["host_monotonic_ns", "host_wall_time_ns", "sequence_id",
                              "packet_seq", "mcu_t_ms", "packets_in_frame", "frame_valid"]
                             + [f"{i}_raw" for i in ids] + [f"{i}_v" for i in ids])
        self.stats = dict(frames=0, invalid_frames=0, packets=0, missing_packets=0,
                          duplicates=0, parse_errors=0, adc_errors=0, reconnects=0)
        self.frame, self.head, self.last_seq, self.latest = {}, None, None, []
        self.started = stamps()
        self._flushed = self._synced = time.monotonic()

    def event(self, level: str, kind: str, **extra) -> None:
        mono, wall = stamps()
        self.eventfile.write(json.dumps(dict(host_monotonic_ns=mono, host_wall_time_ns=wall,
                                             level=level, kind=kind, source="seat", **extra),
                                        ensure_ascii=False) + "\n")

    def reset_link(self) -> None:
        """A (re)started scan begins a new packet sequence and a new frame."""
        self.flush_frame()
        self.last_seq = None

    def packet(self, mono: int, wall: int, text: str) -> bool:
        try:
            seq, tick, channel, readings = parse_packet(text)
            if channel not in self.channels:
                raise ValueError("channel was not requested")
        except ValueError:
            self.stats["parse_errors"] += 1
            return False
        if self.last_seq is not None:
            delta = (seq - self.last_seq) & 0xFFFFFFFF
            if delta == 0:
                self.stats["duplicates"] += 1
                return False
            if delta >= 0x80000000:
                raise RuntimeError("packet sequence went backwards (board reset?)")
            self.stats["missing_packets"] += delta - 1
        self.last_seq = seq
        self.stats["packets"] += 1
        # The firmware scans the selected channels in ascending order, so a
        # channel that does not increase starts the next frame.
        if self.frame and channel <= max(self.frame):
            self.flush_frame()
        if not self.frame:
            self.head = (mono, wall, seq, tick)
        self.frame[channel] = readings
        if len(self.frame) == len(self.channels):
            self.flush_frame()
        return True

    def flush_frame(self) -> None:
        if not self.frame:
            return
        mono, wall, seq, tick = self.head
        valid, raws = len(self.frame) == len(self.channels), []
        for bank, channel in self.sensors:
            raw, status = self.frame[channel][bank] if channel in self.frame else (None, None)
            if status not in (0, None):
                self.stats["adc_errors"] += 1
            valid = valid and status == 0
            raws.append("" if raw is None else raw)
        volts = ["" if raw == "" else round(raw * self.vref / 4095, 5) for raw in raws]
        self.writer.writerow([mono, wall, self.stats["frames"], seq, tick, len(self.frame),
                              int(valid)] + raws + volts)
        self.stats["frames"] += 1
        self.stats["invalid_frames"] += not valid
        self.latest, self.frame = raws, {}

    def housekeeping(self, status) -> None:
        now = time.monotonic()
        if now - self._flushed < 1:
            return
        self._flushed = now
        files = (self.csvfile, self.rawlog, self.eventfile)
        for handle in files:
            handle.flush()
        if now - self._synced >= 5:      # bound what a power cut can lose
            self._synced = now
            for handle in files:
                os.fsync(handle.fileno())
        s = self.stats
        status(f"frames={s['frames']} missing={s['missing_packets']} "
               f"adc_err={s['adc_errors']} reconnects={s['reconnects']} "
               f"raw={' '.join(str(v) for v in self.latest)}")

    def close(self, summary: dict) -> None:
        self.flush_frame()
        for handle in (self.csvfile, self.rawlog, self.eventfile):
            handle.close()
        write_json(self.root / "session_summary.json", summary)
        (self.root / "INCOMPLETE").unlink()


def write_json(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")


def git_commit() -> str | None:
    try:
        return subprocess.run(["git", "-C", str(HERE), "rev-parse", "HEAD"], text=True,
                              capture_output=True, timeout=5).stdout.strip() or None
    except (OSError, subprocess.SubprocessError):
        return None


def stream(link: Link, session: Session, stop, status) -> None:
    """Handshake, then record until ``stop()``; raises on anything that needs a reconnect."""
    link.send("HELLO")
    link.expect(FIRMWARE_ID)
    link.send("DISABLE")
    link.expect(STOP_ACK)
    link.pending.clear()
    link.buffer.clear()
    link.port.reset_input_buffer()
    link.send(f"START_12,{session.mask}")
    link.expect(session.start_ack)
    session.reset_link()
    session.event("info", "recording_started")
    last_ping = last_data = time.monotonic()
    while not stop():
        now = time.monotonic()
        if now - last_ping >= PING_S:
            link.send("PING")
            last_ping = now
        link.poll()
        while link.pending:
            mono, wall, text = link.pending.popleft()
            if text.startswith(("#BOOT", "#CONTROL")):
                raise RuntimeError(f"board stopped or restarted: {text}")
            if not text.startswith("#") and session.packet(mono, wall, text):
                last_data = now
        if now - last_data > STALE_S:
            raise RuntimeError(f"no sensor data for {STALE_S:g} s")
        session.housekeeping(status)


def record(port: str, config: dict, data_root: Path, *, suffix: str = "",
           stop=lambda: False, serial_factory=serial.Serial, ntp_server: str = "pool.ntp.org",
           retry_s: float = 1.0, status=print) -> Path:
    """Record one session; returns its directory.  Ctrl+C or ``stop()`` ends it cleanly.

    A lost port, a board reset or stale data never ends the session: the fault
    is logged to ``events.jsonl`` and the link is re-opened into the same files.
    """
    session = Session(data_root, config, suffix)
    mono, wall = session.started
    write_json(session.root / "metadata.json", {
        "schema_version": SCHEMA, "session_id": session.session_id,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "host": dict(platform.uname()._asdict()), "python": sys.version,
        "pyserial": serial.__version__, "git_commit": git_commit(),
        "clock": {"source": "time.perf_counter_ns", "monotonic_ns_at_start": mono,
                  "wall_time_ns_at_start": wall, "ntp_start": ntp_offset(ntp_server)},
        "port": port, "baud": BAUD, "firmware_expected": FIRMWARE_ID,
        "sampling": "one row per scan of the enabled MUX channels (100 Hz nominal); the "
                    "packets of a frame are converted sequentially, about 2.5 ms apart; "
                    "the row is stamped with the host receive time of its first packet",
        "units": "*_raw: 12-bit ADC counts; *_v: raw * vref / 4095 volts. vref is nominal, "
                 "not measured. Not calibrated to N, kg or kPa.",
        "config": config})
    status(f"recording to {session.root}  (Ctrl+C to stop)")
    link, error = None, None
    try:
        while not stop():
            try:
                link = Link(serial_factory(port, BAUD, timeout=0.02, write_timeout=1),
                            session.rawlog)
                link.port.reset_input_buffer()
                stream(link, session, stop, status)
            except (RuntimeError, serial.SerialException, OSError) as exc:
                session.stats["reconnects"] += 1
                session.event("error", "link_fault", message=str(exc))
                status(f"link fault, retrying: {exc}")
                if link:
                    try:
                        link.port.close()
                    except Exception:
                        pass
                    link = None
                time.sleep(retry_s)
    except KeyboardInterrupt:
        pass
    except Exception as exc:        # still close the files and write the summary
        error = repr(exc)
    disable_verified = False
    if link:
        try:
            link.send("DISABLE")
            link.expect(STOP_ACK, timeout=2)
            disable_verified = True
        except Exception as exc:
            session.event("warning", "disable_not_verified", message=str(exc))
        link.port.close()
    session.flush_frame()           # count the last partial frame in the summary
    end_mono, end_wall = stamps()
    duration = (end_mono - mono) / 1e9
    session.close({
        "schema_version": SCHEMA, "session_id": session.session_id,
        "started_utc": datetime.fromtimestamp(wall / 1e9, timezone.utc).isoformat(),
        "ended_utc": datetime.fromtimestamp(end_wall / 1e9, timezone.utc).isoformat(),
        "duration_s": round(duration, 3),
        "measured_hz": round(session.stats["frames"] / duration, 2) if duration else None,
        "error": error, "disable_verified": disable_verified,
        "ntp_end": ntp_offset(ntp_server), **session.stats})
    status(f"saved {session.stats['frames']} frames to {session.root}")
    return session.root


def main() -> int:
    ap = argparse.ArgumentParser(description="Record the 12-FSR pressure seat while driving.")
    ap.add_argument("--port", help="ST-LINK virtual COM port (default: auto-detect)")
    ap.add_argument("--list-ports", action="store_true")
    ap.add_argument("--config", type=Path, default=HERE / "seat_config.json")
    ap.add_argument("--data-root", type=Path, default=HERE / "data")
    ap.add_argument("--suffix", default="", help="appended to the session name, e.g. driverA_run1")
    ap.add_argument("--seconds", type=float, help="stop after this long (default: until Ctrl+C)")
    ap.add_argument("--ntp-server", default="pool.ntp.org")
    args = ap.parse_args()
    if args.list_ports:
        for p in list_ports.comports():
            print(p.device, "-", p.description)
        return 0
    stop = (lambda: False)
    if args.seconds:
        end = time.monotonic() + args.seconds
        stop = lambda: time.monotonic() >= end
    record(args.port or find_port(), load_config(args.config), args.data_root,
           suffix=args.suffix, stop=stop, ntp_server=args.ntp_server)
    return 0


if __name__ == "__main__":
    sys.exit(main())
