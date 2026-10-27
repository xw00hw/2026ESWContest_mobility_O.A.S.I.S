"""Session-start policy: disk thresholds, fallback rules, storage re-resolution.

These exercise the supervisor's decisions without touching real hardware -- no
worker is started because every device is disabled in the config.
"""

import pytest

import pytest

from oasis_daq import devices as dev
from oasis_daq import health as hz
from oasis_daq.supervisor import Supervisor


@pytest.fixture
def no_udev_links(monkeypatch):
    """Isolate from this machine's installed udev symlinks."""
    monkeypatch.setattr(dev, "UDEV_GPS", "/nonexistent/oasis_gps")
    monkeypatch.setattr(dev, "UDEV_IMU", "/nonexistent/oasis_imu")
    monkeypatch.setattr(dev, "UDEV_CAMERA", "/nonexistent/oasis_camera")


def make_config(tmp_path, **storage):
    cfg = {
        "storage": {
            "roots": [str(tmp_path / "ssd" / "OASIS_DAQ")],
            "fallback_root": str(tmp_path / "fallback"),
            "min_free_gb_start": 20.0,
            "min_free_gb_stop": 5.0,
            "allow_fallback_recording": True,
            "require_mountpoint": False,     # tmp dirs are not mountpoints
            "flush_interval_s": 5.0,
            "fsync_interval_s": 15.0,
            **storage,
        },
        # Every device off: this test is about policy, not hardware.
        "gps": {"enabled": False},
        "imu": {"enabled": False},
        "camera": {"enabled": False},
        "pose": {"enabled": False},
        "dashboard": {"enabled": False},
        "session": {"autostart_recording": False},
        "health": {"sample_interval_s": 60.0},
        "queues": {},
    }
    return cfg


def test_prepare_reaches_ready_with_no_devices(tmp_path):
    (tmp_path / "ssd").mkdir()
    sup = Supervisor(make_config(tmp_path))
    sup.prepare()
    assert sup.state == hz.SessionState.READY
    assert sup.storage is not None and not sup.storage.is_fallback


def test_prepare_uses_the_fallback_when_no_root_is_mounted(tmp_path):
    sup = Supervisor(make_config(tmp_path))          # no ssd/ directory
    sup.prepare()
    assert sup.state == hz.SessionState.READY
    assert sup.storage.is_fallback is True


def test_start_is_refused_below_the_free_space_threshold(tmp_path):
    (tmp_path / "ssd").mkdir()
    # tmp_path lives on a normal filesystem with far less than a petabyte free.
    sup = Supervisor(make_config(tmp_path, min_free_gb_start=1_000_000.0))
    sup.prepare()
    result = sup.start_session()
    assert result["ok"] is False and "GB free" in result["error"]
    assert sup.state != hz.SessionState.RECORDING
    assert any(e["kind"] == "start_refused" for e in sup.recent_events)


def test_start_is_refused_on_the_fallback_when_that_is_disallowed(tmp_path):
    sup = Supervisor(make_config(tmp_path, allow_fallback_recording=False))
    sup.prepare()
    assert sup.storage.is_fallback
    result = sup.start_session()
    assert result["ok"] is False and "fallback" in result["error"]


def test_storage_root_is_re_resolved_at_session_start(tmp_path):
    """Regression: the daemon boots before udisks2 mounts the SSD, so a root
    chosen during prepare() can be stale by the time START is pressed."""
    sup = Supervisor(make_config(tmp_path))
    sup.prepare()
    assert sup.storage.is_fallback is True          # SSD not mounted yet

    (tmp_path / "ssd").mkdir()                      # SSD appears
    sup._refresh_storage_root()

    assert sup.storage.is_fallback is False
    assert str(sup.storage.path).endswith("ssd/OASIS_DAQ")
    assert any(e["kind"] == "storage_root_changed" for e in sup.recent_events)


def test_a_session_creates_its_directory_and_marks_it_incomplete(tmp_path):
    (tmp_path / "ssd").mkdir()
    sup = Supervisor(make_config(tmp_path))
    sup.prepare()
    result = sup.start_session(notes="unit test")
    assert result["ok"] is True
    assert sup.state == hz.SessionState.RECORDING
    root = sup.paths.root
    assert (root / "INCOMPLETE").exists()
    assert (root / "metadata.json").exists()

    stopped = sup.stop_session(reason="unit test")
    assert stopped["ok"] is True
    assert sup.state == hz.SessionState.READY
    # The marker is removed only on a clean close.
    assert not (root / "INCOMPLETE").exists()
    assert (root / "session_summary.json").exists()
    assert (root / "manifest.json").exists()
    assert (root / "route.geojson").exists()


def test_a_second_start_is_refused_while_recording(tmp_path):
    (tmp_path / "ssd").mkdir()
    sup = Supervisor(make_config(tmp_path))
    sup.prepare()
    sup.start_session()
    try:
        second = sup.start_session()
        assert second["ok"] is False and "already recording" in second["error"]
    finally:
        sup.stop_session()


def test_stop_without_a_session_is_reported_not_raised(tmp_path):
    (tmp_path / "ssd").mkdir()
    sup = Supervisor(make_config(tmp_path))
    sup.prepare()
    result = sup.stop_session()
    assert result["ok"] is False and "not recording" in result["error"]


def test_sessions_never_share_a_directory(tmp_path):
    (tmp_path / "ssd").mkdir()
    sup = Supervisor(make_config(tmp_path))
    sup.prepare()
    roots = []
    for _ in range(2):
        sup.start_session()
        roots.append(sup.paths.root)
        sup.stop_session()
    assert roots[0] != roots[1]


def test_metadata_records_the_clock_source_and_vehicle_frame(tmp_path):
    import json
    (tmp_path / "ssd").mkdir()
    sup = Supervisor(make_config(tmp_path))
    sup.prepare()
    sup.start_session()
    meta = json.loads(sup.paths.metadata.read_text())
    sup.stop_session()
    assert meta["clock"]["source"] in ("CLOCK_MONOTONIC_RAW", "CLOCK_MONOTONIC")
    assert meta["frames"]["vehicle_frame"]["definition"] == "+x forward, +y left, +z up"
    assert meta["camera_installation"]["position"] == "driver_side_a_pillar_upper"
    assert meta["schema_version"] == "oasis-daq/0.1"


# --- degraded operation: a missing sensor must not block the others ------
def test_prepare_reaches_ready_when_a_device_cannot_be_resolved(tmp_path, no_udev_links):
    """A missing GPS is a recorded warning, not a refusal to run: the other
    sensors must still be able to record."""
    (tmp_path / "ssd").mkdir()
    cfg = make_config(tmp_path)
    cfg["gps"] = {"enabled": True,
                  "match": {"vid_pid": "dead:beef"},   # nothing will match
                  "baud": 9600}
    sup = Supervisor(cfg)
    report = sup.prepare()

    assert sup.state == hz.SessionState.READY
    assert "error" in sup.resolved["gps"]
    assert any("gps" in e for e in report["errors"])


def test_a_session_still_starts_with_an_unresolvable_sensor(tmp_path, no_udev_links):
    (tmp_path / "ssd").mkdir()
    cfg = make_config(tmp_path)
    cfg["gps"] = {"enabled": True, "match": {"vid_pid": "dead:beef"}}
    sup = Supervisor(cfg)
    sup.prepare()
    result = sup.start_session(notes="degraded")
    try:
        assert result["ok"] is True
        assert sup.state == hz.SessionState.RECORDING
        # The unresolvable worker simply does not exist; nothing else is blocked.
        assert sup.gps is None
    finally:
        sup.stop_session()


def test_startup_errors_are_visible_in_status(tmp_path, no_udev_links):
    (tmp_path / "ssd").mkdir()
    cfg = make_config(tmp_path)
    cfg["imu"] = {"enabled": True, "match": {"vid_pid": "dead:beef"}}
    sup = Supervisor(cfg)
    sup.prepare()
    status = sup.status()
    assert status["startup_errors"], "an operator must be able to see this"
    assert status["state"] == hz.SessionState.READY
    assert "warning" in status["state_reason"]


def test_status_is_json_serialisable_with_no_session(tmp_path):
    """The dashboard serialises this on every poll; a non-finite float or an
    unserialisable object here would break the whole page."""
    import json
    (tmp_path / "ssd").mkdir()
    sup = Supervisor(make_config(tmp_path))
    sup.prepare()
    text = json.dumps(sup.status(), allow_nan=False, default=str)
    assert "Infinity" not in text and "NaN" not in text


def test_health_row_is_json_serialisable_and_finite(tmp_path):
    import json
    (tmp_path / "ssd").mkdir()
    sup = Supervisor(make_config(tmp_path))
    sup.prepare()
    sup.start_session()
    try:
        row = sup.health_row()
        text = json.dumps(row, allow_nan=False, default=str)
        assert "Infinity" not in text and "NaN" not in text
        assert row["state"] == hz.SessionState.RECORDING
        assert set(row) == set(hz.HEALTH_CSV_FIELDS), "health.csv columns must match"
    finally:
        sup.stop_session()


def test_health_row_reports_no_phantom_queue_drops(tmp_path):
    """Regression: GPS and IMU pushed every row into a queue nothing drained.
    After 8192 samples it rejected everything, and a 32-minute session reported
    11400 'dropped' IMU samples while imu.csv held every single one."""
    (tmp_path / "ssd").mkdir()
    sup = Supervisor(make_config(tmp_path))
    sup.prepare()
    sup.start_session()
    try:
        row = sup.health_row()
        assert "queue_gps" not in row and "queue_imu" not in row
        assert row["camera_write_drops"] == 0
    finally:
        sup.stop_session()


def test_summary_file_actually_records_the_clean_close_flags(tmp_path):
    """Regression: clean_close was set on the dict AFTER write_json had already
    serialised it, so it never reached session_summary.json - the logic was
    right but the evidence was missing from disk."""
    import json
    (tmp_path / "ssd").mkdir()
    sup = Supervisor(make_config(tmp_path))
    sup.prepare()
    sup.start_session()
    root = sup.paths.root
    sup.stop_session(reason="unit test")

    summary = json.loads((root / "session_summary.json").read_text())
    assert summary["clean_close"] is True
    assert summary["workers_not_joined"] == []
    assert not (root / "INCOMPLETE").exists()


def test_the_manifest_does_not_list_the_marker_it_is_about_to_delete(tmp_path):
    import json
    (tmp_path / "ssd").mkdir()
    sup = Supervisor(make_config(tmp_path))
    sup.prepare()
    sup.start_session()
    root = sup.paths.root
    sup.stop_session()
    manifest = json.loads((root / "manifest.json").read_text())
    listed = {f["path"] for f in manifest["files"]}
    assert "INCOMPLETE" not in listed
    for path in listed:
        assert (root / path).exists(), f"manifest lists a missing file: {path}"


def test_pose_drop_counter_counts_frames_not_retries(monkeypatch):
    """Regression: when the pose child fell behind, the submit loop retried the
    same frame every 2 ms and incremented `dropped` each time -- 5325 'drops'
    against 592 processed frames in one 86 s session, which reads as massive
    data loss when pose is simply allowed to run slower than capture."""
    import queue as _queue

    from oasis_daq.pose.worker import PoseWorker
    from oasis_daq.workers import LatestSlot

    slot = LatestSlot()
    w = PoseWorker.__new__(PoseWorker)          # no process, no model needed
    w.pose_slot = slot
    w.target_fps = 10.0
    w.frames_submitted = 0
    w._last_sent_ns = 0
    w._last_seq = 0

    class _FullQueue:
        def put_nowait(self, item):
            raise _queue.Full

    w._in_q = _FullQueue()

    class _Stats:
        dropped = 0
    w.stats = _Stats()

    slot.put(("frame", 1, 2, b"jpeg"))
    interval_ns = int(1e9 / w.target_fps)

    # One ready frame, many loop iterations while the child stays busy.
    for _ in range(200):
        w._maybe_submit(interval_ns)

    # Backing off for a full interval means at most a couple of counts, not 200.
    assert w.stats.dropped <= 2, f"counted retries, not frames: {w.stats.dropped}"
    assert w.frames_submitted == 0
