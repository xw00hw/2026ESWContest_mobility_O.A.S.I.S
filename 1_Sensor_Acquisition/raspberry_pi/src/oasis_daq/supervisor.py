"""The OASIS DAQ supervisor: state machine, workers, session lifecycle.

Recording is entirely local.  Nothing in this module depends on the dashboard,
on a laptop being connected, or on the network being up -- the dashboard is a
read-only observer plus a START/STOP control surface.
"""

from __future__ import annotations

import logging
import os
import threading
import time
from pathlib import Path
from typing import Any

import yaml

from . import devices as dev
from . import health as hz
from . import session as sess
from .clock import monotonic_ns, now, wall_time_ns
from .storage import (SessionPaths, StorageRoot, CsvWriter, JsonlWriter,
                      resolve_storage_root, free_gb)
from .workers import LatestSlot

log = logging.getLogger(__name__)

__all__ = ["Supervisor", "load_config", "REPO_ROOT", "DEFAULT_CONFIG"]

REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_CONFIG = Path(__file__).resolve().parents[2] / "configs" / "daq.yaml"


def load_config(path: str | os.PathLike[str] | None = None) -> dict:
    """Load YAML config, falling back to the packaged default."""
    cfg_path = Path(path) if path else DEFAULT_CONFIG
    with open(cfg_path, "r", encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh) or {}
    cfg["_config_path"] = str(cfg_path.resolve())
    return cfg


class SessionError(RuntimeError):
    """A condition that must prevent or terminate a recording session."""


class Supervisor:
    """Owns device resolution, the state machine and every worker."""

    def __init__(self, config: dict):
        self.config = config
        self.state = hz.SessionState.INIT
        self.state_reason = "starting up"
        self._lock = threading.RLock()

        self.storage: StorageRoot | None = None
        self.paths: SessionPaths | None = None
        self.resolved: dict[str, Any] = {}
        self.device_snapshot: dict = {}

        # workers
        self.gps = None
        self.imu = None
        self.camera = None
        self.pose = None

        # inter-worker slots (always newest-wins)
        self.pose_slot = LatestSlot()
        self.preview_slot = LatestSlot()
        self.pose_result_slot = LatestSlot()
        self.gps_slot = LatestSlot()
        self.imu_slot = LatestSlot()

        self._events: JsonlWriter | None = None
        self._health_csv: CsvWriter | None = None
        self._health_thread: threading.Thread | None = None
        self._health_stop = threading.Event()
        self._sampler = hz.SystemSampler()

        self.session_started_ns = 0
        self.session_started_wall_ns = 0
        self.event_counts = {"info": 0, "warning": 0, "error": 0}
        self.recent_events: list[dict] = []
        self.last_summary: dict | None = None
        self.last_session_root: str | None = None
        self.startup_errors: list[str] = []
        self.protocol_checks: dict = {}
        self._disk_write_latency_ms: float | None = None
        self._last_disk_probe_ns = 0

    # ------------------------------------------------------------------
    # state
    # ------------------------------------------------------------------
    def set_state(self, state: str, reason: str = "") -> None:
        with self._lock:
            if state == self.state and reason == self.state_reason:
                return
            previous = self.state
            self.state = state
            self.state_reason = reason
        log.info("state: %s -> %s (%s)", previous, state, reason)
        self.event("state_changed", "supervisor",
                   {"from": previous, "to": state, "reason": reason},
                   level="error" if state == hz.SessionState.ERROR else "info")

    @property
    def recording(self) -> bool:
        return self.state == hz.SessionState.RECORDING

    # ------------------------------------------------------------------
    # events
    # ------------------------------------------------------------------
    def event(self, kind: str, source: str, payload: dict | None = None,
              level: str = "info") -> None:
        """Record a structured event.  Never raises."""
        mono, wall = now()
        record = {
            "host_monotonic_ns": mono,
            "host_wall_time_ns": wall,
            "level": level,
            "kind": kind,
            "source": source,
            **(payload or {}),
        }
        self.event_counts[level] = self.event_counts.get(level, 0) + 1
        self.recent_events.append(record)
        if len(self.recent_events) > 200:
            del self.recent_events[:-200]
        writer = self._events
        if writer is not None:
            try:
                writer.write(record)
            except Exception:
                log.exception("failed to write event")
        if level == "error":
            log.error("event %s from %s: %s", kind, source, payload)

    def _worker_event(self, kind: str, source: str, payload: dict) -> None:
        level = "error" if kind in (
            "worker_failed", "device_lost", "recorder_died",
            "recorder_write_failed") else "info"
        if kind in ("worker_error", "neutral_calibration_failed"):
            level = "warning"
        self.event(kind, source, payload, level=level)
        if level == "error" and self.recording:
            # A dead recorder or lost device ends the session cleanly.
            if kind in ("recorder_died", "recorder_write_failed") or \
               (kind == "worker_failed" and source == "camera"):
                threading.Thread(
                    target=self.stop_session,
                    kwargs={"reason": f"{source}: {kind}", "error": True},
                    daemon=True, name="oasis-fatal-stop").start()

    # ------------------------------------------------------------------
    # bring-up
    # ------------------------------------------------------------------
    def prepare(self) -> dict:
        """Resolve storage and devices.  Moves INIT -> READY (or ERROR)."""
        self.startup_errors = []
        report: dict[str, Any] = {}

        try:
            self.storage = resolve_storage_root(self.config.get("storage", {}))
            report["storage"] = self.storage.to_dict()
            if self.storage.is_fallback:
                self.event("storage_fallback", "supervisor",
                           {"path": str(self.storage.path),
                            "reason": self.storage.reason}, level="warning")
        except Exception as exc:
            self.startup_errors.append(f"storage: {exc}")
            report["storage"] = {"error": str(exc)}

        self.device_snapshot = dev.snapshot()
        report["devices"] = self.device_snapshot
        self.resolved = {}

        for name, udev_link, resolver in (
            ("gps", dev.UDEV_GPS, dev.resolve_serial),
            ("imu", dev.UDEV_IMU, dev.resolve_serial),
        ):
            cfg = self.config.get(name, {})
            if not cfg.get("enabled", True):
                self.resolved[name] = {"enabled": False}
                continue
            try:
                path, info = resolver(dev.DeviceMatcher.from_config(cfg.get("match")),
                                      udev_link)
                self.resolved[name] = {
                    "enabled": True, "path": path,
                    "info": info.to_dict() if info else None,
                    "baud": cfg.get("baud"),
                }
            except Exception as exc:
                self.resolved[name] = {"enabled": True, "error": str(exc)}
                self.startup_errors.append(f"{name}: {exc}")

        cam_cfg = self.config.get("camera", {})
        if cam_cfg.get("enabled", True):
            try:
                path, info = dev.resolve_camera(
                    dev.DeviceMatcher.from_config(cam_cfg.get("match")))
                self.resolved["camera"] = {
                    "enabled": True, "path": path,
                    "info": info.to_dict() if info else None,
                }
            except Exception as exc:
                self.resolved["camera"] = {"enabled": True, "error": str(exc)}
                self.startup_errors.append(f"camera: {exc}")
        else:
            self.resolved["camera"] = {"enabled": False}

        self._validate_protocols(report)

        report["resolved"] = self.resolved
        report["errors"] = list(self.startup_errors)

        if self.storage is None:
            self.set_state(hz.SessionState.ERROR, "no writable storage root")
        else:
            self.set_state(hz.SessionState.READY,
                           "ready" if not self.startup_errors
                           else f"ready with {len(self.startup_errors)} device warning(s)")
        return report

    def _validate_protocols(self, report: dict) -> None:
        """Confirm each resolved port actually speaks the expected protocol.

        Resolution proves a device has the right USB identity, not that it is
        the right device: `1a86:7523` is a generic CH340 id shared by countless
        boards.  Without this, a misidentified port produces a session whose
        CSV contains only a header, and the only signal is a stale-sensor
        warning partway in.  Probing is read-only -- no configuration is
        written to either device.
        """
        checks: dict[str, Any] = {}

        imu_cfg = self.config.get("imu", {})
        imu = self.resolved.get("imu", {})
        if imu.get("path") and imu_cfg.get("validate_protocol", True):
            from .imu.worker import probe_imu
            seconds = float(imu_cfg.get("validate_seconds", 2.0))
            result = probe_imu(imu["path"], int(imu_cfg.get("baud", 9600)), seconds)
            checks["imu"] = {k: result[k] for k in
                             ("frames_ok", "frames_bad_checksum", "frame_ratio",
                              "rate_hz", "looks_like_wit", "error")}
            if not result["looks_like_wit"]:
                msg = (f"imu at {imu['path']} did not produce valid WIT frames "
                       f"at {imu_cfg.get('baud')} baud "
                       f"({result['frames_ok']} frames, "
                       f"ratio {result['frame_ratio']:.3f})")
                self.startup_errors.append(msg)
                self.resolved["imu"]["protocol_error"] = msg
                self.event("protocol_validation_failed", "imu",
                           checks["imu"], level="error")

        gps_cfg = self.config.get("gps", {})
        gps = self.resolved.get("gps", {})
        if gps.get("path") and gps_cfg.get("validate_protocol", True):
            from .gps.worker import probe_gps
            seconds = float(gps_cfg.get("validate_seconds", 2.5))
            result = probe_gps(gps["path"], int(gps_cfg.get("baud", 9600)), seconds)
            checks["gps"] = {k: result[k] for k in
                             ("nmea_sentences", "checksum_bad", "fix_valid",
                              "sentence_types", "error")}
            if result["nmea_sentences"] == 0:
                msg = (f"gps at {gps['path']} produced no NMEA sentences in "
                       f"{seconds:.1f} s")
                self.startup_errors.append(msg)
                self.resolved["gps"]["protocol_error"] = msg
                self.event("protocol_validation_failed", "gps",
                           checks["gps"], level="error")

        report["protocol_checks"] = checks
        self.protocol_checks = checks

    # ------------------------------------------------------------------
    # session control
    # ------------------------------------------------------------------
    def start_session(self, notes: str = "", suffix: str = "") -> dict:
        """Begin recording.  Returns a result dict; never raises to callers."""
        with self._lock:
            if self.state == hz.SessionState.RECORDING:
                return {"ok": False, "error": "already recording",
                        "session_id": self.paths.session_id if self.paths else None}
            if self.state == hz.SessionState.SAVING:
                return {"ok": False, "error": "previous session is still saving"}
            if self.storage is None:
                return {"ok": False, "error": "no storage root; run prepare() first"}

            storage_cfg = self.config.get("storage", {})
            # Re-resolve before every session.  The daemon starts at boot, but
            # udisks2 mounts the external SSD only after a desktop login, so the
            # root chosen during prepare() can be the fallback while the real
            # SSD is available by the time the operator presses START.
            self._refresh_storage_root()
            available = free_gb(self.storage.path)
            required = float(storage_cfg.get("min_free_gb_start", 20.0))
            if available < required:
                msg = (f"refusing to start: {available:.1f} GB free, "
                       f"{required:.1f} GB required")
                self.event("start_refused", "supervisor",
                           {"free_gb": round(available, 2),
                            "required_gb": required}, level="error")
                return {"ok": False, "error": msg}
            if (self.storage.is_fallback
                    and not storage_cfg.get("allow_fallback_recording", True)):
                return {"ok": False,
                        "error": "external SSD unavailable and fallback recording "
                                 "is disabled in config"}

            try:
                self._open_session(notes=notes, suffix=suffix)
            except Exception as exc:
                log.exception("failed to start session")
                self.event("session_start_failed", "supervisor",
                           {"error": str(exc)}, level="error")
                self.set_state(hz.SessionState.ERROR, f"start failed: {exc}")
                return {"ok": False, "error": str(exc)}

            return {"ok": True, "session_id": self.paths.session_id,
                    "session_root": str(self.paths.root),
                    "storage_is_fallback": self.storage.is_fallback}

    def _refresh_storage_root(self) -> None:
        """Re-pick the storage root; log it when the choice changes."""
        try:
            fresh = resolve_storage_root(self.config.get("storage", {}))
        except Exception as exc:
            self.event("storage_recheck_failed", "supervisor",
                       {"error": str(exc)}, level="warning")
            return
        previous = self.storage
        if previous is None or str(fresh.path) != str(previous.path):
            self.storage = fresh
            self.event("storage_root_changed", "supervisor", {
                "from": (str(previous.path) if previous else None),
                "to": str(fresh.path),
                "is_fallback": fresh.is_fallback,
            }, level="warning" if fresh.is_fallback else "info")
        else:
            self.storage = fresh

    def _open_session(self, notes: str, suffix: str) -> None:
        assert self.storage is not None
        paths = SessionPaths.allocate(self.storage.path, suffix=suffix)
        self.paths = paths
        self.session_started_ns = monotonic_ns()
        self.session_started_wall_ns = wall_time_ns()
        self.event_counts = {"info": 0, "warning": 0, "error": 0}
        self.recent_events = []

        # Written first, removed on a clean stop: its presence marks a session
        # that was interrupted (crash, power loss) so it is never mistaken for
        # a complete one, and never overwritten by a later run.
        paths.incomplete_marker.write_text(
            f"session {paths.session_id} opened at {time.time()}\n"
            "This file is removed when the session closes cleanly.\n",
            encoding="utf-8")

        storage_cfg = self.config.get("storage", {})
        flush_s = float(storage_cfg.get("flush_interval_s", 5.0))
        fsync_s = float(storage_cfg.get("fsync_interval_s", 15.0))
        self._events = JsonlWriter(paths.events, flush_s, fsync_s)
        self._health_csv = CsvWriter(paths.health / "health.csv",
                                     hz.HEALTH_CSV_FIELDS, flush_s, fsync_s)

        self.event("session_started", "supervisor", {
            "session_id": paths.session_id,
            "session_root": str(paths.root),
            "storage_is_fallback": self.storage.is_fallback,
            "notes": notes,
        })

        metadata = sess.collect_metadata(
            session_id=paths.session_id, config=self.config,
            storage_root=self.storage, session_root=paths.root,
            devices=self.device_snapshot, resolved=self.resolved,
            repo_root=REPO_ROOT)
        metadata["session"] = {"notes": notes,
                               "operator": self.config.get("session", {}).get("operator", ""),
                               "vehicle": self.config.get("session", {}).get("vehicle", "")}
        sess.write_json(paths.metadata, metadata)

        self._start_workers(flush_s, fsync_s)
        self._health_stop.clear()
        self._health_thread = threading.Thread(target=self._health_loop,
                                               name="oasis-health", daemon=True)
        self._health_thread.start()
        self.set_state(hz.SessionState.RECORDING, f"session {paths.session_id}")

    def _start_workers(self, flush_s: float, fsync_s: float) -> None:
        assert self.paths is not None
        q = self.config.get("queues", {})
        started: list[str] = []

        gps_cfg = self.config.get("gps", {})
        if gps_cfg.get("enabled", True) and self.resolved.get("gps", {}).get("path"):
            from .gps.worker import GpsWorker
            self.gps = GpsWorker(
                device=self.resolved["gps"]["path"], baud=int(gps_cfg.get("baud", 9600)),
                gps_dir=self.paths.gps, flush_interval_s=flush_s,
                fsync_interval_s=fsync_s,
                read_timeout_s=float(gps_cfg.get("read_timeout_s", 1.0)),
                on_event=self._worker_event, latest_slot=self.gps_slot)
            self.gps.start(); started.append("gps")
        else:
            self.gps = None

        imu_cfg = self.config.get("imu", {})
        if imu_cfg.get("enabled", True) and self.resolved.get("imu", {}).get("path"):
            from .imu.worker import ImuWorker
            mounting = imu_cfg.get("mounting", {})
            self.imu = ImuWorker(
                device=self.resolved["imu"]["path"], baud=int(imu_cfg.get("baud", 9600)),
                imu_dir=self.paths.imu,
                acc_range_g=float(imu_cfg.get("acc_range_g", 16.0)),
                gyro_range_dps=float(imu_cfg.get("gyro_range_dps", 2000.0)),
                angle_range_deg=float(imu_cfg.get("angle_range_deg", 180.0)),
                rotation_matrix=mounting.get("rotation_matrix"),
                calibration=self._load_calibration(imu_cfg),
                save_raw_stream=bool(imu_cfg.get("save_raw_stream", True)),
                raw_chunk_bytes=int(imu_cfg.get("raw_chunk_bytes", 1 << 20)),
                flush_interval_s=flush_s, fsync_interval_s=fsync_s,
                read_timeout_s=float(imu_cfg.get("read_timeout_s", 1.0)),
                on_event=self._worker_event, latest_slot=self.imu_slot)
            self.imu.start(); started.append("imu")
        else:
            self.imu = None

        cam_cfg = self.config.get("camera", {})
        if cam_cfg.get("enabled", True) and self.resolved.get("camera", {}).get("path"):
            from .camera.worker import CameraWorker
            self.camera = CameraWorker(
                device=self.resolved["camera"]["path"], camera_dir=self.paths.camera,
                width=int(cam_cfg.get("width", 1280)),
                height=int(cam_cfg.get("height", 800)),
                fourcc=str(cam_cfg.get("fourcc", "MJPG")),
                capture_fps=int(cam_cfg.get("capture_fps", 100)),
                record_decimation=int(cam_cfg.get("record_decimation", 4)),
                segment_seconds=int(cam_cfg.get("segment_seconds", 120)),
                container=str(cam_cfg.get("container", "mkv")),
                flush_interval_s=flush_s, fsync_interval_s=fsync_s,
                write_queue_maxsize=int(q.get("camera_write_maxsize", 240)),
                on_event=self._worker_event,
                pose_slot=self.pose_slot, preview_slot=self.preview_slot,
                record=True)
            self.camera.start(); started.append("camera")
        else:
            self.camera = None

        pose_cfg = self.config.get("pose", {})
        if pose_cfg.get("enabled", True) and self.camera is not None:
            from .pose.worker import PoseWorker, DEFAULT_MODEL_DIR
            model = pose_cfg.get("model_path") or (
                DEFAULT_MODEL_DIR / "pose_landmarker_lite.task")
            try:
                self.pose = PoseWorker(
                    pose_dir=self.paths.pose, pose_slot=self.pose_slot,
                    model_path=model,
                    target_fps=float(pose_cfg.get("target_fps", 10)),
                    input_width=int(pose_cfg.get("input_width", 640)),
                    input_height=int(pose_cfg.get("input_height", 400)),
                    grayscale_to_rgb=bool(pose_cfg.get("grayscale_to_rgb", True)),
                    min_detection_confidence=float(
                        pose_cfg.get("min_detection_confidence", 0.5)),
                    min_tracking_confidence=float(
                        pose_cfg.get("min_tracking_confidence", 0.5)),
                    neutral_cfg=pose_cfg.get("neutral_calibration"),
                    flush_interval_s=flush_s, fsync_interval_s=fsync_s,
                    on_event=self._worker_event,
                    result_slot=self.pose_result_slot)
                self.pose.start(); started.append("pose")
            except Exception as exc:
                # Vision is not allowed to prevent raw acquisition.
                self.pose = None
                self.event("pose_disabled", "supervisor", {"error": str(exc)},
                           level="warning")
        else:
            self.pose = None

        self.event("workers_started", "supervisor", {"workers": started})

    def _load_calibration(self, imu_cfg: dict) -> dict:
        """Load an existing calibration file; never create or overwrite one."""
        rel = imu_cfg.get("calibration_file")
        if not rel:
            return {}
        path = Path(rel)
        if not path.is_absolute():
            path = Path(self.config.get("_config_path",
                                        str(DEFAULT_CONFIG))).parent.parent / rel
        if not path.exists():
            return {}
        try:
            with open(path, "r", encoding="utf-8") as fh:
                return yaml.safe_load(fh) or {}
        except Exception as exc:
            self.event("calibration_load_failed", "supervisor",
                       {"path": str(path), "error": str(exc)}, level="warning")
            return {}

    def stop_session(self, reason: str = "operator", error: bool = False) -> dict:
        """Stop workers, flush everything, write the artefacts."""
        with self._lock:
            if self.state not in (hz.SessionState.RECORDING, hz.SessionState.ERROR):
                return {"ok": False, "error": f"not recording (state={self.state})"}
            if self.paths is None:
                return {"ok": False, "error": "no open session"}
            paths = self.paths
            self.set_state(hz.SessionState.SAVING, reason)

        self.event("session_stopping", "supervisor",
                   {"reason": reason, "error": error},
                   level="error" if error else "info")

        self._health_stop.set()
        if self._health_thread is not None:
            self._health_thread.join(timeout=5)

        workers = [w for w in (self.camera, self.pose, self.gps, self.imu)
                   if w is not None]
        for w in workers:
            w.stop()
        stuck: list[str] = []
        for w in workers:
            w.join(timeout=30)
            if w.is_alive():
                # teardown() never ran: this worker's writers were not closed or
                # fsynced, and for the camera the final segment was never
                # finalised.  The session is NOT clean.
                stuck.append(w.stats.name)
                log.warning("%s did not stop within timeout", w.stats.name)
                self.event("worker_stop_timeout", w.stats.name, {}, level="error")

        ended_ns, ended_wall = now()
        worker_stats = {w.stats.name: w.stats.to_dict() for w in workers}

        # Route artefacts from whatever GPS track we actually captured.
        track = list(self.gps.track) if self.gps is not None else []
        route_stats = sess.route_summary(track)
        try:
            sess.write_route_geojson(paths.route_geojson, track, {
                "session_id": paths.session_id,
                "distance_km": route_stats.get("distance_km"),
            })
            sess.write_json(paths.route_summary, route_stats)
            sess.write_route_map(paths.route_map, track, paths.session_id, route_stats)
        except Exception as exc:
            self.event("route_artifacts_failed", "supervisor",
                       {"error": str(exc)}, level="warning")

        timestamps = self._timestamp_quality(paths)
        storage_status = (self.storage.to_dict() if self.storage else {})

        summary = sess.build_summary(
            session_id=paths.session_id,
            state=hz.SessionState.ERROR if error else "COMPLETE",
            started_ns=self.session_started_ns, ended_ns=ended_ns,
            started_wall_ns=self.session_started_wall_ns, ended_wall_ns=ended_wall,
            workers=worker_stats, storage=storage_status,
            events=dict(self.event_counts), timestamps=timestamps,
            notes=reason)
        summary["route"] = route_stats
        summary["stop_reason"] = reason

        self.event("session_stopped", "supervisor",
                   {"duration_s": summary["duration_s"],
                    "session_id": paths.session_id})

        if self._health_csv is not None:
            self._health_csv.close()
        self._health_csv = None
        # NOTE: self._events stays open here on purpose -- set_state() below
        # emits the final state change, and closing the writer first would drop
        # it from events.jsonl.  It is closed at the very end of this method.

        # These have to be decided BEFORE the summary is written, or they never
        # reach the file -- which is the whole point of recording them.
        clean = (not error) and (not stuck)
        summary["clean_close"] = clean
        summary["workers_not_joined"] = stuck

        artifacts_ok = True
        try:
            sess.write_json(paths.summary, summary)
            sess.write_json(paths.manifest, sess.build_manifest(paths.root))
        except Exception as exc:
            log.exception("failed writing summary/manifest")
            artifacts_ok = False
            summary["artifact_error"] = str(exc)
            self.event("artifact_write_failed", "supervisor",
                       {"error": str(exc)}, level="error")

        # The marker may only be removed when the session genuinely closed
        # cleanly.  A stuck worker means unflushed writers and, for the camera,
        # an unfinalised segment; a failed artefact write means no summary and
        # no manifest to read.  Either way the session must stay marked
        # INCOMPLETE rather than become indistinguishable from a good one.
        # (artifacts_ok is not written into the summary: if it is False there is
        # no summary file to read it from -- the surviving marker is the signal.)
        clean = clean and artifacts_ok
        try:
            if clean:
                paths.incomplete_marker.unlink(missing_ok=True)
        except OSError:
            pass

        self.last_summary = summary
        self.last_session_root = str(paths.root)
        self.paths = None
        self.gps = self.imu = self.camera = self.pose = None

        if error or stuck or not artifacts_ok:
            self.set_state(hz.SessionState.ERROR,
                           reason if error else
                           (f"workers did not stop: {', '.join(stuck)}" if stuck
                            else "failed to write session artefacts"))
        else:
            self.set_state(hz.SessionState.READY, "session saved")

        # Now that the final state change has been recorded, close the log.
        if self._events is not None:
            self._events.close()
            self._events = None

        return {"ok": True, "clean": clean, "session_id": summary["session_id"],
                "session_root": str(paths.root), "summary": summary}

    def _timestamp_quality(self, paths: SessionPaths) -> dict:
        """Re-read the written CSVs and audit their host timestamps."""
        out: dict[str, Any] = {}
        sources = [
            # The GPS assembler emits one row per GGA *and* per RMC, ~0.3 ms
            # apart, so auditing every row reports a rock-steady 1 Hz receiver
            # as 2 Hz with ~900 ms of jitter.  Audit the GGA series, which is
            # one row per epoch, and keep the all-rows view alongside it.
            ("gps", paths.gps / "gps.csv", "host_monotonic_ns", None,
             ("source_sentence", "GPGGA")),
            ("gps_all_rows", paths.gps / "gps.csv", "host_monotonic_ns", None, None),
            ("imu", paths.imu / "imu.csv", "host_monotonic_ns",
             self.config.get("imu", {}).get("expected_hz"), None),
            ("camera", paths.camera / "frame_timestamps.csv",
             "capture_monotonic_ns",
             self.config.get("camera", {}).get("capture_fps"), None),
            ("pose", paths.pose / "features.csv", "capture_monotonic_ns",
             None, None),
        ]
        for name, path, column, expected, row_filter in sources:
            if not path.exists():
                continue
            try:
                import csv as _csv
                values: list[int] = []
                with open(path, newline="", encoding="utf-8") as fh:
                    for row in _csv.DictReader(fh):
                        if row_filter and row.get(row_filter[0]) != row_filter[1]:
                            continue
                        raw = row.get(column)
                        if raw:
                            values.append(int(raw))
                out[name] = sess.timestamp_quality(values, expected)
            except Exception as exc:
                out[name] = {"error": str(exc)}
        return out

    # ------------------------------------------------------------------
    # health loop
    # ------------------------------------------------------------------
    def _health_loop(self) -> None:
        cfg = self.config.get("health", {})
        interval = float(cfg.get("sample_interval_s", 1.0))
        temp_warn = float(cfg.get("cpu_temp_warn_c", 75.0))
        temp_error = float(cfg.get("cpu_temp_error_c", 85.0))
        storage_cfg = self.config.get("storage", {})
        min_free_stop = float(storage_cfg.get("min_free_gb_stop", 5.0))
        warned_temp = False
        warned_disk = False

        while not self._health_stop.is_set():
            try:
                row = self.health_row()
                if self._health_csv is not None:
                    self._health_csv.write(row)

                temp = row.get("cpu_temp_c")
                if temp is not None:
                    if temp >= temp_error:
                        self.event("cpu_temperature_critical", "health",
                                   {"temp_c": temp}, level="error")
                    elif temp >= temp_warn and not warned_temp:
                        warned_temp = True
                        self.event("cpu_temperature_high", "health",
                                   {"temp_c": temp}, level="warning")
                    elif temp < temp_warn - 3:
                        warned_temp = False

                free = row.get("disk_free_gb")
                if free is not None and free <= min_free_stop and self.recording:
                    self.event("disk_space_exhausted", "health",
                               {"free_gb": free, "threshold_gb": min_free_stop},
                               level="error")
                    threading.Thread(
                        target=self.stop_session,
                        kwargs={"reason": f"disk below {min_free_stop} GB",
                                "error": True},
                        daemon=True, name="oasis-disk-stop").start()
                    return
                if (free is not None and free <= min_free_stop * 3
                        and not warned_disk):
                    warned_disk = True
                    self.event("disk_space_low", "health", {"free_gb": free},
                               level="warning")

                for worker, key in ((self.gps, "gps"), (self.imu, "imu"),
                                    (self.camera, "camera")):
                    if worker is None or not worker.stats.running:
                        continue
                    limit = float(self.config.get(key, {}).get("stale_after_s", 5.0))
                    stats = worker.stats
                    if stats.samples:
                        age = stats.age_s()
                    else:
                        # A worker that has not produced its first sample yet is
                        # not stale -- it is starting up.  Measure from when the
                        # worker started, otherwise age_s() is inf and every
                        # sensor is reported stale on the very first health tick.
                        age = ((monotonic_ns() - stats.started_monotonic_ns) / 1e9
                               if stats.started_monotonic_ns else 0.0)
                    if age > limit:
                        self.event("sensor_stale", key,
                                   {"age_s": round(age, 2), "limit_s": limit,
                                    "samples": stats.samples}, level="warning")
            except Exception:
                log.exception("health loop iteration failed")
            self._health_stop.wait(interval)

    def health_row(self) -> dict:
        """One health.csv row / one /api/health payload."""
        mono, wall = now()
        mem = hz.memory_info()
        gps_s = self.gps.snapshot() if self.gps else {}
        imu_s = self.imu.snapshot() if self.imu else {}
        cam_s = self.camera.snapshot() if self.camera else {}
        pose_s = self.pose.snapshot() if self.pose else {}

        disk_free = free_gb(self.storage.path) if self.storage else None
        # Probing write latency costs an fsync, so do it at most every 30 s.
        if (self.storage is not None
                and mono - self._last_disk_probe_ns > 30_000_000_000):
            self._last_disk_probe_ns = mono
            self._disk_write_latency_ms = hz.measure_write_latency_ms(
                self.storage.path)

        return {
            "host_monotonic_ns": mono,
            "host_wall_time_ns": wall,
            "state": self.state,
            "cpu_percent": self._sampler.cpu_percent(),
            "cpu_temp_c": hz.cpu_temperature_c(),
            "load_1m": round(hz.load_average()[0], 2),
            "mem_total_mb": mem["total_mb"],
            "mem_available_mb": mem["available_mb"],
            "mem_used_percent": mem["used_percent"],
            "disk_free_gb": round(disk_free, 2) if disk_free is not None else None,
            "disk_write_latency_ms": self._disk_write_latency_ms,
            "gps_alive": int(bool(gps_s.get("alive"))),
            "gps_rate_hz": gps_s.get("rate_hz"),
            "gps_age_s": gps_s.get("age_s"),
            "gps_fix_valid": int(bool(gps_s.get("fix_valid"))),
            "gps_satellites": gps_s.get("satellite_count"),
            "imu_alive": int(bool(imu_s.get("alive"))),
            "imu_rate_hz": imu_s.get("rate_hz"),
            "imu_age_s": imu_s.get("age_s"),
            "imu_checksum_errors": imu_s.get("checksum_errors"),
            "camera_alive": int(bool(cam_s.get("alive"))),
            "camera_fps": cam_s.get("measured_fps"),
            "camera_frames_recorded": cam_s.get("frames_recorded"),
            "camera_drops": cam_s.get("estimated_drops"),
            "pose_alive": int(bool(pose_s.get("alive"))),
            "pose_fps": pose_s.get("fps"),
            "pose_latency_p50_ms": pose_s.get("latency_p50_ms"),
            "pose_valid_ratio": pose_s.get("valid_ratio"),
            # The only bounded queue on a data path is the camera writer's;
            # GPS and IMU write their CSV synchronously in their own thread.
            "queue_camera_write": (self.camera._write_queue.qsize()
                                   if self.camera is not None else 0),
            "camera_write_drops": (self.camera.write_queue_drops
                                   if self.camera is not None else 0),
            "queue_pose_dropped": pose_s.get("dropped_frames", 0),
            "events_warning": self.event_counts.get("warning", 0),
            "events_error": self.event_counts.get("error", 0),
        }

    # ------------------------------------------------------------------
    # status
    # ------------------------------------------------------------------
    def status(self) -> dict:
        elapsed = ((monotonic_ns() - self.session_started_ns) / 1e9
                   if self.recording and self.session_started_ns else 0.0)
        storage_status = self.storage.status().to_dict() if self.storage else {}
        if self.storage:
            storage_status["is_fallback"] = self.storage.is_fallback
        recording_hours = None
        if self.storage and self.camera is not None and self.camera.frames_recorded > 100:
            mb_per_s = (self.camera.bytes_recorded / (1024 * 1024)) / max(elapsed, 1e-6)
            if mb_per_s > 0:
                recording_hours = round(
                    (storage_status.get("free_gb", 0) * 1024) / mb_per_s / 3600.0, 2)
        return {
            "state": self.state,
            "state_reason": self.state_reason,
            "session_id": self.paths.session_id if self.paths else None,
            "session_root": (str(self.paths.root) if self.paths
                             else self.last_session_root),
            "elapsed_s": round(elapsed, 2),
            "elapsed_hms": _hms(elapsed),
            "recording": self.recording,
            "storage": storage_status,
            "estimated_recording_hours_left": recording_hours,
            "gps": self.gps.snapshot() if self.gps else {"alive": False},
            "imu": self.imu.snapshot() if self.imu else {"alive": False},
            "camera": self.camera.snapshot() if self.camera else {"alive": False},
            "pose": self.pose.snapshot() if self.pose else {"alive": False},
            "events": dict(self.event_counts),
            "recent_events": self.recent_events[-15:],
            "resolved_devices": self.resolved,
            "startup_errors": self.startup_errors,
            "protocol_checks": self.protocol_checks,
            "clock_source": __import__("oasis_daq.clock", fromlist=["x"]).clock_source(),
        }

    def shutdown(self, reason: str = "shutdown", timeout_s: float = 150.0) -> None:
        """Graceful SIGTERM path: close the session, then release everything.

        A stop may already be running on another thread -- the fatal-error and
        disk-exhausted paths both call ``stop_session`` from a daemon thread,
        and so does a dashboard STOP.  Returning while that is in progress lets
        the interpreter exit and kill the daemon thread mid-save, losing the
        summary and manifest and leaving writers unflushed, with exit code 0 so
        systemd does not even restart.  So wait it out.
        """
        if self.state == hz.SessionState.SAVING:
            log.info("shutdown: a stop is already in progress, waiting for it")
            deadline = monotonic_ns() + int(timeout_s * 1e9)
            while (self.state == hz.SessionState.SAVING
                   and monotonic_ns() < deadline):
                time.sleep(0.1)
            if self.state == hz.SessionState.SAVING:
                log.error("shutdown: stop did not finish within %.0f s", timeout_s)
        if self.state == hz.SessionState.RECORDING:
            self.stop_session(reason=reason)
        self._health_stop.set()


def _hms(seconds: float) -> str:
    seconds = int(seconds)
    return f"{seconds // 3600:02d}:{(seconds % 3600) // 60:02d}:{seconds % 60:02d}"
