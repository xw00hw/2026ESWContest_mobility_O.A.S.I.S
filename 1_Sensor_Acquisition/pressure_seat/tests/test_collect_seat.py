"""Offline check of the seat recorder against a fake board. Never opens a real port."""
from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import collect_seat as cs

CONFIG = cs.load_config(Path(cs.HERE) / "seat_config.json")


class FakeBoard:
    """Speaks the v9 protocol; each read() during a scan returns one 4-packet frame,
    split mid-line so the line buffering is exercised."""

    def __init__(self, *args, reboot_at=None, **kwargs):
        self.out, self.seq, self.reads, self.active = bytearray(), 0, 0, False
        self.reboot_at, self.commands = reboot_at, []

    in_waiting = property(lambda self: len(self.out))

    def reset_input_buffer(self):
        self.out.clear()

    def flush(self):
        pass

    def close(self):
        pass

    def write(self, payload):
        command = payload.decode().strip()
        self.commands.append(command)
        if command == "HELLO":
            self.out += (cs.FIRMWARE_ID + "\r\n").encode()
        elif command == "DISABLE":
            self.active = False
            self.out += (cs.STOP_ACK + "\r\n").encode()
        elif command.startswith("START_12,"):
            self.active, self.seq = True, 0
            self.out += f"#CONTROL,12,MASK={command.split(',')[1]},EN=0\r\n".encode()
        return len(payload)

    def read(self, amount):
        self.reads += 1
        if self.active and not self.out:
            if self.reboot_at and self.reads >= self.reboot_at:
                self.active, self.reboot_at = False, None
                self.out += b"#BOOT,FSR_12_SCAN,v9\r\n"
            else:
                for channel in range(4):
                    # bank 1 / C2 reports an ADC failure in the very first frame
                    bad = self.seq == 2
                    self.out += (f"{self.seq},{self.seq * 3},{channel},{100 + channel},0,"
                                 f"{'' if bad else 200 + channel},{2 if bad else 0},"
                                 f"{300 + channel},0\r\n").encode()
                    self.seq += 1
        cut = max(1, min(amount, len(self.out) - 7)) if self.out else 0
        chunk, self.out = bytes(self.out[:cut]), self.out[cut:]
        return chunk


def run(tmp_path, **board_kwargs):
    boards = []

    def factory(*args, **kwargs):
        boards.append(FakeBoard(*args, **board_kwargs))
        return boards[-1]

    root = cs.record("FAKE", CONFIG, tmp_path, suffix="t", serial_factory=factory,
                     stop=lambda: sum(b.reads for b in boards) >= 400,
                     ntp_server="invalid.invalid", retry_s=0, status=lambda _: None)
    rows = list(csv.DictReader((root / "seat" / "pressure.csv").open(encoding="utf-8")))
    summary = json.loads((root / "session_summary.json").read_text(encoding="utf-8"))
    return root, rows, summary, boards


def test_frames_are_wide_stamped_and_complete(tmp_path):
    root, rows, summary, boards = run(tmp_path)
    assert len(rows) > 20 and summary["frames"] == len(rows)
    assert not (root / "INCOMPLETE").exists()
    assert boards[0].commands[:3] == ["HELLO", "DISABLE", "START_12,15"]
    assert boards[0].commands[-1] == "DISABLE" and summary["disable_verified"]

    first, second = rows[0], rows[1]
    assert first["A0_C1_raw"] == "101" and first["A2_C3_raw"] == "303"
    assert float(first["A0_C1_v"]) == pytest.approx(101 * 3.3 / 4095, abs=1e-5)
    # the failed conversion is blank, never 0, and invalidates only its frame
    assert first["A1_C2_raw"] == "" and first["A1_C2_v"] == "" and first["frame_valid"] == "0"
    assert second["frame_valid"] == "1" and second["packets_in_frame"] == "4"
    assert summary["adc_errors"] == 1 and summary["missing_packets"] == 0

    mono = [int(r["host_monotonic_ns"]) for r in rows]
    assert mono == sorted(mono)
    assert [int(r["sequence_id"]) for r in rows] == list(range(len(rows)))
    assert abs(int(rows[0]["host_wall_time_ns"]) - cs.time.time_ns()) < 60e9   # UNIX epoch ns
    raw_lines = (root / "seat" / "raw_uart.log").read_text().splitlines()
    assert sum(not l.split(" ", 2)[2].startswith("#") for l in raw_lines) >= 4 * len(rows)


def test_board_reset_reconnects_into_the_same_session(tmp_path):
    root, rows, summary, boards = run(tmp_path, reboot_at=60)
    assert summary["reconnects"] >= 1 and len(boards) >= 2
    events = [json.loads(l) for l in (root / "events.jsonl").read_text().splitlines()]
    assert any(e["kind"] == "link_fault" and "#BOOT" in e["message"] for e in events)
    assert sum(e["kind"] == "recording_started" for e in events) >= 2
    fault = next(e["host_monotonic_ns"] for e in events if e["kind"] == "link_fault")
    assert any(r["frame_valid"] == "1" and int(r["host_monotonic_ns"]) > fault for r in rows)


def test_bad_lines_and_gaps_are_counted_not_guessed(tmp_path):
    session = cs.Session(tmp_path, CONFIG)
    assert not session.packet(1, 1, "garbage")
    assert session.packet(1, 1, "0,0,0,1,0,2,0,3,0")
    assert session.packet(2, 2, "3,7,3,1,0,2,0,3,0")        # seq 1 and 2 never arrived
    assert session.packet(3, 3, "4,10,0,1,0,2,0,3,0")       # channel wrapped -> new frame
    session.close({})
    assert session.stats["parse_errors"] == 1 and session.stats["missing_packets"] == 2
    rows = list(csv.DictReader((session.root / "seat" / "pressure.csv").open(encoding="utf-8")))
    assert [r["packets_in_frame"] for r in rows] == ["2", "1"]
    assert rows[0]["frame_valid"] == "0" and rows[0]["A0_C1_raw"] == ""
    with pytest.raises(ValueError):
        cs.parse_packet("1,2,3,0,0,5000,0,0,0")
