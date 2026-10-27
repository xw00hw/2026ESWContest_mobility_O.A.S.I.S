"""Storage safety, buffered writers and session artefacts."""

import json
import os

import pytest

from oasis_daq import session as sess
from oasis_daq.storage import (CsvWriter, JsonlWriter, RawChunkWriter,
                               SessionPaths, resolve_storage_root)


def test_session_directory_is_never_reused(tmp_path):
    first = SessionPaths.allocate(tmp_path)
    # Allocating again within the same second must not collide with the first.
    second = SessionPaths.allocate(tmp_path)
    assert first.root != second.root
    assert first.root.exists() and second.root.exists()


def test_allocate_creates_every_subdirectory(tmp_path):
    p = SessionPaths.allocate(tmp_path)
    for d in (p.camera, p.pose, p.imu, p.gps, p.health):
        assert d.is_dir()


def test_storage_falls_back_when_no_root_is_mounted(tmp_path):
    cfg = {"roots": [str(tmp_path / "definitely" / "not" / "mounted")],
           "fallback_root": str(tmp_path / "fallback"),
           "require_mountpoint": False}
    root = resolve_storage_root(cfg)
    assert root.is_fallback is True and root.path.exists()


def test_storage_prefers_a_mounted_root_over_the_fallback(tmp_path):
    (tmp_path / "mnt").mkdir()
    cfg = {"roots": [str(tmp_path / "mnt" / "OASIS_DAQ")],
           "fallback_root": str(tmp_path / "fallback"),
           "require_mountpoint": False}      # tmp dirs are not mountpoints
    root = resolve_storage_root(cfg)
    assert root.is_fallback is False
    assert root.path == tmp_path / "mnt" / "OASIS_DAQ"


def test_csv_writer_writes_header_and_rows(tmp_path):
    w = CsvWriter(tmp_path / "a.csv", ["a", "b"])
    w.write({"a": 1, "b": 2})
    w.write({"a": 3, "b": 4, "ignored": 9})
    w.close()
    lines = (tmp_path / "a.csv").read_text().splitlines()
    assert lines[0] == "a,b" and lines[1] == "1,2" and lines[2] == "3,4"


def test_writers_tolerate_being_closed_twice(tmp_path):
    w = JsonlWriter(tmp_path / "e.jsonl")
    w.write({"x": 1})
    w.close()
    w.close()                       # must not raise
    w.write({"x": 2})               # writing after close is a no-op, not a crash
    assert len((tmp_path / "e.jsonl").read_text().splitlines()) == 1


def test_raw_chunk_writer_rolls_over_and_preserves_all_bytes(tmp_path):
    w = RawChunkWriter(tmp_path, "raw", chunk_bytes=4096)
    payload = bytes(range(256)) * 64          # 16 KiB
    # Written the way the IMU worker writes: many small reads, not one blob.
    for i in range(0, len(payload), 512):
        w.write(payload[i:i + 512])
    w.close()
    recovered = b"".join((tmp_path / name).read_bytes() for name in sorted(w.chunks))
    assert recovered == payload
    assert w.total_bytes == len(payload) and len(w.chunks) > 1


def test_raw_chunk_writer_flushes_before_the_chunk_is_full(tmp_path):
    """Regression: a power cut must not discard everything since the last chunk."""
    w = RawChunkWriter(tmp_path, "raw", chunk_bytes=1 << 20, flush_interval_s=0.0)
    w.write(b"important bytes")
    on_disk = (tmp_path / w.chunks[0]).stat().st_size
    w.close()
    assert on_disk == len(b"important bytes")


# --- timestamp quality --------------------------------------------------
def test_timestamp_quality_on_a_perfect_stream():
    stamps = [i * 10_000_000 for i in range(101)]      # exactly 100 Hz
    q = sess.timestamp_quality(stamps, expected_hz=100)
    assert q["monotonic"] and q["duplicates"] == 0 and q["gaps"] == 0
    assert q["measured_hz"] == pytest.approx(100.0)
    assert abs(q["rate_error_percent"]) < 0.01


def test_timestamp_quality_detects_backward_steps():
    q = sess.timestamp_quality([0, 10, 20, 15, 30])
    assert q["monotonic"] is False and q["backward_steps"] == 1


def test_timestamp_quality_counts_duplicates():
    q = sess.timestamp_quality([0, 10, 10, 10, 20])
    assert q["duplicates"] == 2


def test_timestamp_quality_flags_a_gap():
    stamps = [i * 10_000_000 for i in range(50)]
    stamps += [stamps[-1] + 900_000_000]
    q = sess.timestamp_quality(stamps)
    assert q["gaps"] == 1 and q["max_gap_s"] == pytest.approx(0.9)


def test_timestamp_quality_handles_short_streams():
    assert sess.timestamp_quality([])["count"] == 0
    assert sess.timestamp_quality([1])["count"] == 1


# --- route artefacts ----------------------------------------------------
def test_route_geojson_uses_lon_lat_order(tmp_path):
    track = [(11.5, 48.1), (11.6, 48.2)]
    fc = sess.write_route_geojson(tmp_path / "r.geojson", track)
    coords = fc["features"][0]["geometry"]["coordinates"]
    assert coords[0] == [11.5, 48.1]           # RFC 7946: longitude first
    assert json.loads((tmp_path / "r.geojson").read_text())["type"] == "FeatureCollection"


def test_route_geojson_with_no_fix_is_still_valid(tmp_path):
    fc = sess.write_route_geojson(tmp_path / "r.geojson", [])
    assert fc["type"] == "FeatureCollection" and fc["features"] == []


def test_route_summary_distance_is_physically_sensible():
    # One degree of latitude is about 111 km.
    s = sess.route_summary([(0.0, 0.0), (0.0, 1.0)])
    assert 110_000 < s["distance_m"] < 112_000


def test_route_map_html_is_written_and_self_describing(tmp_path):
    sess.write_route_map(tmp_path / "m.html", [(11.5, 48.1), (11.6, 48.2)],
                         "session_test", {"distance_km": 1.0})
    html = (tmp_path / "m.html").read_text()
    assert "session_test" in html and "drawFallback" in html


def test_manifest_lists_files_with_hashes(tmp_path):
    (tmp_path / "sub").mkdir()
    (tmp_path / "sub" / "f.txt").write_text("hello")
    m = sess.build_manifest(tmp_path)
    entry = next(f for f in m["files"] if f["path"].endswith("f.txt"))
    assert entry["bytes"] == 5 and len(entry["sha256"]) == 64


def test_write_json_is_atomic_and_leaves_no_temp_file(tmp_path):
    target = tmp_path / "x.json"
    sess.write_json(target, {"a": 1})
    assert json.loads(target.read_text()) == {"a": 1}
    assert not list(tmp_path.glob("*.tmp"))


def test_jsonl_writer_never_emits_infinity_or_nan(tmp_path):
    """Regression: json.dumps(inf) emits bare `Infinity`, which is not valid
    JSON and makes strict parsers reject the whole events file."""
    w = JsonlWriter(tmp_path / "e.jsonl")
    w.write({"age_s": float("inf"), "x": float("nan"),
             "nested": {"v": float("-inf")}, "list": [1.0, float("inf")]})
    w.close()
    text = (tmp_path / "e.jsonl").read_text()
    assert "Infinity" not in text and "NaN" not in text
    # strict=True is the default; this raises if the line is not valid JSON
    parsed = json.loads(text)
    assert parsed["age_s"] is None and parsed["x"] is None
    assert parsed["nested"]["v"] is None and parsed["list"] == [1.0, None]


def test_jsonl_writer_keeps_finite_floats_intact(tmp_path):
    w = JsonlWriter(tmp_path / "e.jsonl")
    w.write({"a": 1.5, "b": -0.25, "c": 0.0})
    w.close()
    assert json.loads((tmp_path / "e.jsonl").read_text()) == {"a": 1.5, "b": -0.25, "c": 0.0}


# --- git provenance -----------------------------------------------------
def test_porcelain_parsing_keeps_the_full_path():
    """Regression: strip() on the whole porcelain output ate the leading status
    space of the first line, so `01. Data_acquisition/x.py` was recorded as
    `cquisition/x.py` in metadata.json."""
    status = (" M 01. Data_acquisition/src/oasis_daq/camera/worker.py\n"
              "?? 01. Data_acquisition/tests/test_devices.py\n"
              "M  01. Data_acquisition/docs/TEST_RESULTS.md")
    assert sess.parse_porcelain(status) == [
        "01. Data_acquisition/src/oasis_daq/camera/worker.py",
        "01. Data_acquisition/tests/test_devices.py",
        "01. Data_acquisition/docs/TEST_RESULTS.md",
    ]


def test_porcelain_parsing_handles_renames_and_empty_input():
    assert sess.parse_porcelain("R  old/path.py -> new/path.py") == ["new/path.py"]
    assert sess.parse_porcelain("") == []
    assert sess.parse_porcelain(None) == []


def test_git_state_records_a_usable_commit_and_paths(tmp_path):
    """Against this repository, which is a real git checkout."""
    from pathlib import Path
    state = sess._git_state(Path(__file__).resolve().parents[3])
    if not state.get("available"):
        pytest.skip("not a git checkout")
    assert len(state["commit"]) == 40 and "\n" not in state["commit"]
    assert state["branch"] and "\n" not in state["branch"]
    for p in state["dirty_files"]:
        # every recorded path must actually exist relative to the repo root
        assert (Path(__file__).resolve().parents[3] / p).exists(), p


def test_route_summary_has_the_same_keys_with_and_without_a_track():
    """A consumer must not have to special-case 'no GPS fix' by testing for a
    missing key instead of a zero value."""
    empty = sess.route_summary([])
    populated = sess.route_summary([(11.5, 48.1), (11.6, 48.2)])
    assert set(empty) == set(populated)
    assert empty["distance_km"] == 0.0 and empty["point_count"] == 0
    assert empty["bbox"] is None and empty["start"] is None


def test_a_leftover_directory_is_not_mistaken_for_a_mounted_ssd(tmp_path):
    """Regression: is_dir() passes for a directory left behind by an unmount,
    which would silently record to the internal disk while metadata.json
    claimed the SSD."""
    stale = tmp_path / "media" / "T7"          # looks like the mountpoint...
    stale.mkdir(parents=True)                  # ...but nothing is mounted there
    cfg = {"roots": [str(stale / "OASIS_DAQ")],
           "fallback_root": str(tmp_path / "fallback")}
    root = resolve_storage_root(cfg)           # require_mountpoint defaults True
    assert root.is_fallback is True
    assert not (stale / "OASIS_DAQ").exists(), "must not create a tree off-mount"


def test_a_gap_means_a_missed_sample_not_three_times_the_interval():
    """Regression: the old 3x threshold worked out to 2.8 s on a 1 Hz GPS, so a
    genuine three-fix outage was not counted as a gap at all."""
    perfect = [i * 1_000_000_000 for i in range(30)]          # 1 Hz
    outage = perfect[:15] + [t + 3_000_000_000 for t in perfect[15:]]
    q = sess.timestamp_quality(outage)
    assert q["gaps"] == 1
    assert q["estimated_missed_samples"] == 3
    assert q["max_gap_s"] == pytest.approx(4.0)


def test_a_single_missed_sample_is_a_gap_but_not_a_large_gap():
    stamps = [i * 10_000_000 for i in range(50)]              # 100 Hz
    stamps = stamps[:25] + [t + 10_000_000 for t in stamps[25:]]
    q = sess.timestamp_quality(stamps)
    assert q["gaps"] == 1 and q["large_gaps"] == 0
    assert q["estimated_missed_samples"] == 1


def test_jitter_within_tolerance_is_not_a_gap():
    stamps, t = [], 0
    for i in range(60):
        t += 10_000_000 + (2_000_000 if i % 2 else -2_000_000)   # +/-20% jitter
        stamps.append(t)
    assert sess.timestamp_quality(stamps)["gaps"] == 0


def test_a_bimodal_series_reports_no_gap_statistics():
    """Regression: the GPS row series alternates ~0.3 ms and ~1 s, so the gap
    heuristic reported 103 gaps and 5745 missed samples on a healthy
    105-second recording."""
    stamps, t = [], 0
    for _ in range(100):                     # GGA then RMC, then a 1 s wait
        stamps.append(t); t += 300_000       # 0.3 ms apart
        stamps.append(t); t += 999_700_000   # then ~1 s
    q = sess.timestamp_quality(stamps)
    assert q["bimodal"] is True
    assert q["gaps"] is None and q["estimated_missed_samples"] is None
    assert "bimodal" in q["note"]
    assert q["max_gap_s"] is not None        # still reported, it is factual


def test_a_regular_series_is_not_flagged_bimodal():
    q = sess.timestamp_quality([i * 10_000_000 for i in range(200)])
    assert q["bimodal"] is False
    assert q["gaps"] == 0
