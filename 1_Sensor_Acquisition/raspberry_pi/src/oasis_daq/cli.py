"""``oasis-daqctl`` -- bring-up checks, daemon control and session tooling.

``start``/``stop``/``status`` talk to the running daemon over its local HTTP
API rather than opening the devices themselves: the daemon is the single owner
of the camera and the serial ports, and two owners would corrupt a session.
"""

from __future__ import annotations

import argparse
import json
import logging
import signal
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

from . import devices as dev
from . import health as hz
from . import session as sess
from .storage import free_gb, resolve_storage_root
from .supervisor import Supervisor, load_config

log = logging.getLogger("oasis_daq.cli")

__all__ = ["main"]

_OK = "\033[32mOK\033[0m"
_WARN = "\033[33mWARN\033[0m"
_FAIL = "\033[31mFAIL\033[0m"


def _plain(text: str) -> str:
    return (text.replace("\033[32m", "").replace("\033[33m", "")
                .replace("\033[31m", "").replace("\033[0m", ""))


def _mark(ok: bool, warn: bool = False) -> str:
    tag = _WARN if warn else (_OK if ok else _FAIL)
    return tag if sys.stdout.isatty() else _plain(tag)


# ----------------------------------------------------------------------
# daemon API client
# ----------------------------------------------------------------------
def _api(cfg: dict, path: str, method: str = "GET", payload: dict | None = None,
         timeout: float = 15.0) -> dict:
    dash = cfg.get("dashboard", {})
    host = dash.get("host", "127.0.0.1")
    if host in ("0.0.0.0", "::"):
        host = "127.0.0.1"
    url = f"http://{host}:{dash.get('port', 8080)}{path}"
    data = json.dumps(payload or {}).encode() if method == "POST" else None
    req = urllib.request.Request(url, data=data, method=method,
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return json.loads(resp.read().decode())
    except urllib.error.HTTPError as exc:
        try:
            return json.loads(exc.read().decode())
        except Exception:
            return {"ok": False, "error": f"HTTP {exc.code}"}
    except (urllib.error.URLError, OSError) as exc:
        return {"ok": False, "error": f"daemon not reachable at {url}: {exc}"}


# ----------------------------------------------------------------------
# check
# ----------------------------------------------------------------------
def cmd_check(args) -> int:
    cfg = load_config(args.config)
    problems = 0
    warnings = 0
    report: dict = {"checks": []}

    def record(name: str, ok: bool, detail: str, warn: bool = False,
               data: dict | None = None):
        nonlocal problems, warnings
        if not ok:
            if warn:
                warnings += 1
            else:
                problems += 1
        print(f"  [{_mark(ok, warn and not ok)}] {name}: {detail}")
        report["checks"].append({"name": name, "ok": ok, "warning": warn,
                                 "detail": detail, **(data or {})})

    print("== storage ==")
    try:
        root = resolve_storage_root(cfg.get("storage", {}))
        free = free_gb(root.path)
        needed = float(cfg.get("storage", {}).get("min_free_gb_start", 20.0))
        record("storage root", True,
               f"{root.path} ({free:.1f} GB free)"
               + (" [FALLBACK]" if root.is_fallback else ""),
               data={"path": str(root.path), "free_gb": round(free, 2),
                     "is_fallback": root.is_fallback})
        if root.is_fallback:
            record("external SSD", False,
                   "external SSD not available; using local fallback", warn=True)
        record("free space", free >= needed,
               f"{free:.1f} GB free, {needed:.1f} GB required",
               warn=(free < needed))
    except Exception as exc:
        record("storage root", False, str(exc))
        root = None

    print("== devices ==")
    snapshot = dev.snapshot()
    report["devices"] = snapshot

    for name, link in (("gps", dev.UDEV_GPS), ("imu", dev.UDEV_IMU)):
        sub = cfg.get(name, {})
        if not sub.get("enabled", True):
            record(name, True, "disabled in config")
            continue
        try:
            path, info = dev.resolve_serial(
                dev.DeviceMatcher.from_config(sub.get("match")), link)
            record(name, True,
                   f"{path} ({info.vid_pid if info else 'unknown vid:pid'})",
                   data={"path": path})
        except Exception as exc:
            record(name, False, str(exc))

    cam_cfg = cfg.get("camera", {})
    camera_path = None
    if cam_cfg.get("enabled", True):
        try:
            camera_path, cinfo = dev.resolve_camera(
                dev.DeviceMatcher.from_config(cam_cfg.get("match")))
            record("camera", True, f"{camera_path} ({cinfo.card if cinfo else '?'})",
                   data={"path": camera_path})
        except Exception as exc:
            record("camera", False, str(exc))
    else:
        record("camera", True, "disabled in config")

    print("== pose model ==")
    from .pose.worker import DEFAULT_MODEL_DIR
    model = Path(cfg.get("pose", {}).get("model_path")
                 or DEFAULT_MODEL_DIR / "pose_landmarker_lite.task")
    record("pose model", model.exists(),
           str(model) if model.exists()
           else f"missing: {model} (run scripts/fetch_pose_model.sh)",
           warn=not model.exists())

    print("== system ==")
    mem = hz.memory_info()
    temp = hz.cpu_temperature_c()
    record("memory", mem["available_mb"] > 512,
           f"{mem['available_mb']:.0f} MB available of {mem['total_mb']:.0f} MB")
    record("cpu temperature", temp is None or temp < 80,
           f"{temp:.1f} C" if temp is not None else "unavailable", warn=True)
    throttled = hz.throttled_flags()
    if throttled is not None:
        record("throttling", throttled == "0x0", f"vcgencmd get_throttled={throttled}",
               warn=True)

    if args.probe:
        print("== live probes ==")
        if camera_path:
            from .camera.worker import probe_camera
            pc = probe_camera(camera_path, int(cam_cfg.get("width", 1280)),
                              int(cam_cfg.get("height", 800)),
                              str(cam_cfg.get("fourcc", "MJPG")),
                              int(cam_cfg.get("capture_fps", 100)),
                              seconds=args.probe_seconds)
            record("camera stream", pc["frames"] > 0,
                   f"{pc['measured_fps']} fps, {pc['valid_jpeg']}/{pc['frames']} "
                   f"valid JPEG, mean {pc['mean_jpeg_bytes']} B"
                   + (f" -- {pc['error']}" if pc["error"] else ""),
                   data={"probe": pc})
        gps_res = next((c for c in report["checks"]
                        if c["name"] == "gps" and c["ok"]), None)
        if gps_res:
            from .gps.worker import probe_gps
            pg = probe_gps(gps_res["path"], int(cfg["gps"].get("baud", 9600)),
                           seconds=args.probe_seconds)
            record("gps stream", pg["nmea_sentences"] > 0,
                   f"{pg['nmea_sentences']} sentences, "
                   f"{pg['checksum_bad']} checksum errors, "
                   f"fix={'yes' if pg['fix_valid'] else 'no'}, "
                   f"types={sorted(pg['sentence_types'])}",
                   data={"probe": pg})
            if not pg["fix_valid"]:
                record("gps fix", False,
                       "no position fix (normal indoors; needs sky view)", warn=True)
        imu_res = next((c for c in report["checks"]
                        if c["name"] == "imu" and c["ok"]), None)
        if imu_res:
            from .imu.worker import probe_imu, probe_baud_rates
            if args.probe_imu_baud:
                pb = probe_baud_rates(imu_res["path"],
                                      cfg["imu"].get("baud_candidates",
                                                     [9600, 115200]),
                                      seconds=args.probe_seconds)
                record("imu baud scan", pb["confident"],
                       f"selected {pb['selected_baud']} baud",
                       data={"probe": pb})
            else:
                pi = probe_imu(imu_res["path"], int(cfg["imu"].get("baud", 9600)),
                               seconds=args.probe_seconds)
                record("imu stream", pi["looks_like_wit"],
                       f"{pi['frames_ok']} frames @ {pi['rate_hz']:.1f} Hz, "
                       f"ratio {pi['frame_ratio']:.3f}, "
                       f"{pi['frames_bad_checksum']} checksum errors, "
                       f"types={sorted(pi['packet_types'])}",
                       data={"probe": pi})

    print(f"\n{problems} problem(s), {warnings} warning(s)")
    if args.json:
        report["problems"] = problems
        report["warnings"] = warnings
        Path(args.json).write_text(json.dumps(report, indent=2, default=str))
        print(f"report written to {args.json}")
    return 1 if problems else 0


# ----------------------------------------------------------------------
# status / start / stop
# ----------------------------------------------------------------------
def cmd_status(args) -> int:
    cfg = load_config(args.config)
    status = _api(cfg, "/api/status")
    if args.json:
        print(json.dumps(status, indent=2, default=str))
        return 0 if status.get("state") else 1
    if status.get("error"):
        print(f"[{_mark(False)}] {status['error']}")
        return 1
    print(f"state          : {status['state']} ({status.get('state_reason','')})")
    print(f"session        : {status.get('session_id') or '-'}")
    print(f"elapsed        : {status.get('elapsed_hms','-')}")
    st = status.get("storage", {})
    print(f"storage        : {st.get('path','-')} ({st.get('free_gb','?')} GB free)")
    for key in ("gps", "imu", "camera", "pose"):
        sub = status.get(key) or {}
        print(f"{key:<15}: alive={sub.get('alive')} "
              + " ".join(f"{k}={sub[k]}" for k in
                         ("rate_hz", "measured_fps", "fps", "fix_valid",
                          "frames_recorded", "latency_p95_ms")
                         if k in sub))
    ev = status.get("events", {})
    print(f"events         : {ev.get('warning',0)} warning, {ev.get('error',0)} error")
    return 0


def cmd_start(args) -> int:
    cfg = load_config(args.config)
    result = _api(cfg, "/api/session/start", "POST",
                  {"notes": args.notes or "", "suffix": args.suffix or ""})
    print(json.dumps(result, indent=2, default=str))
    return 0 if result.get("ok") else 1


def cmd_stop(args) -> int:
    cfg = load_config(args.config)
    result = _api(cfg, "/api/session/stop", "POST",
                  {"reason": args.reason or "oasis-daqctl"}, timeout=120)
    if result.get("ok") and not args.json:
        s = result.get("summary", {})
        print(f"session {s.get('session_id')} stopped after {s.get('duration_hms')}")
        print(f"  root  : {result.get('session_root')}")
        for name, stats in (s.get("workers") or {}).items():
            print(f"  {name:<7}: {stats.get('samples')} samples, "
                  f"{stats.get('errors')} errors, {stats.get('dropped')} dropped")
    else:
        print(json.dumps(result, indent=2, default=str))
    return 0 if result.get("ok") else 1


# ----------------------------------------------------------------------
# summarize
# ----------------------------------------------------------------------
def cmd_summarize(args) -> int:
    root = Path(args.session).expanduser().resolve()
    if not root.is_dir():
        print(f"[{_mark(False)}] not a directory: {root}")
        return 1
    summary_path = root / "session_summary.json"
    if summary_path.exists() and not args.rebuild:
        payload = json.loads(summary_path.read_text())
    else:
        print("rebuilding summary from files on disk...")
        payload = {"session_id": root.name, "rebuilt": True}

    manifest = sess.build_manifest(root)
    if args.rebuild:
        sess.write_json(root / "manifest.json", manifest)

    print(f"session   : {payload.get('session_id', root.name)}")
    print(f"state     : {payload.get('final_state','?')}")
    print(f"duration  : {payload.get('duration_hms','?')}")
    print(f"files     : {manifest['file_count']} ({manifest['total_mb']} MB)")
    if (root / "INCOMPLETE").exists():
        print(f"[{_mark(False, True)}] INCOMPLETE marker present: "
              "this session did not close cleanly")
    for name, q in (payload.get("timestamp_quality") or {}).items():
        if "error" in q:
            print(f"  {name:<8}: error {q['error']}")
            continue
        print(f"  {name:<8}: {q.get('count')} rows @ {q.get('measured_hz')} Hz, "
              f"monotonic={q.get('monotonic')} dup={q.get('duplicates')} "
              f"gaps={q.get('gaps')} jitter_p95={q.get('jitter_ms_p95')}ms")
    route = payload.get("route") or {}
    if route:
        print(f"  route   : {route.get('point_count')} points, "
              f"{route.get('distance_km')} km")
    if args.json:
        print(json.dumps({"summary": payload, "manifest": manifest},
                         indent=2, default=str))
    return 0


# ----------------------------------------------------------------------
# daemon
# ----------------------------------------------------------------------
def cmd_daemon(args) -> int:
    cfg = load_config(args.config)
    logging.basicConfig(
        level=getattr(logging, str(cfg.get("logging", {}).get("level", "INFO")).upper(),
                      logging.INFO),
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s")

    supervisor = Supervisor(cfg)
    report = supervisor.prepare()
    for err in report.get("errors", []):
        log.warning("bring-up: %s", err)

    dashboard = None
    dash_cfg = cfg.get("dashboard", {})
    if dash_cfg.get("enabled", True):
        from .dashboard.server import DashboardServer
        dashboard = DashboardServer(
            supervisor, host=dash_cfg.get("host", "0.0.0.0"),
            port=int(dash_cfg.get("port", 8080)),
            preview_width=int(dash_cfg.get("preview_width", 640)),
            preview_quality=int(dash_cfg.get("preview_jpeg_quality", 60)),
            preview_fps=float(dash_cfg.get("preview_fps", 8)))
        dashboard.start()

    stopping = {"flag": False}

    def _signal(signum, _frame):
        log.info("received signal %s, shutting down", signum)
        stopping["flag"] = True

    signal.signal(signal.SIGTERM, _signal)
    signal.signal(signal.SIGINT, _signal)

    if cfg.get("session", {}).get("autostart_recording", False):
        log.info("autostart_recording is enabled; starting a session")
        supervisor.start_session(notes="autostart")
    elif args.record:
        supervisor.start_session(notes=args.notes or "daemon --record")

    if args.duration:
        deadline = time.monotonic() + args.duration
    else:
        deadline = None

    log.info("daemon ready (state=%s)", supervisor.state)
    try:
        while not stopping["flag"]:
            time.sleep(0.25)
            if deadline is not None and time.monotonic() >= deadline:
                log.info("configured duration elapsed")
                break
    finally:
        supervisor.shutdown(reason="daemon shutdown")
        if dashboard is not None:
            dashboard.stop()
    log.info("daemon exited cleanly")
    return 0


# ----------------------------------------------------------------------
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="oasis-daqctl",
        description="OASIS DAQ control: hardware checks, daemon control, sessions.")
    p.add_argument("--config", help="path to daq.yaml")
    sub = p.add_subparsers(dest="command", required=True)

    c = sub.add_parser("check", help="verify storage, devices and system state")
    c.add_argument("--probe", action="store_true",
                   help="also open each device and measure real data")
    c.add_argument("--probe-seconds", type=float, default=3.0)
    c.add_argument("--probe-imu-baud", action="store_true",
                   help="scan the configured IMU baud candidates")
    c.add_argument("--json", help="write a machine-readable report here")
    c.set_defaults(func=cmd_check)

    s = sub.add_parser("status", help="query the running daemon")
    s.add_argument("--json", action="store_true")
    s.set_defaults(func=cmd_status)

    st = sub.add_parser("start", help="start a recording session")
    st.add_argument("--notes", default="")
    st.add_argument("--suffix", default="")
    st.set_defaults(func=cmd_start)

    sp = sub.add_parser("stop", help="stop the recording session")
    sp.add_argument("--reason", default="oasis-daqctl")
    sp.add_argument("--json", action="store_true")
    sp.set_defaults(func=cmd_stop)

    sm = sub.add_parser("summarize", help="summarise a recorded session directory")
    sm.add_argument("session")
    sm.add_argument("--rebuild", action="store_true",
                    help="regenerate manifest.json from the files present")
    sm.add_argument("--json", action="store_true")
    sm.set_defaults(func=cmd_summarize)

    d = sub.add_parser("daemon", help="run the supervisor (used by systemd)")
    d.add_argument("--record", action="store_true",
                   help="start recording immediately after bring-up")
    d.add_argument("--notes", default="")
    d.add_argument("--duration", type=float,
                   help="exit after N seconds (used by the endurance test)")
    d.set_defaults(func=cmd_daemon)
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
