"""Session metadata, manifest, summary and route artefacts."""

from __future__ import annotations

import hashlib
import json
import os
import platform
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from . import clock

__all__ = [
    "collect_metadata", "write_json", "build_manifest", "build_summary",
    "write_route_geojson", "write_route_map", "timestamp_quality",
    "VEHICLE_FRAME",
]

VEHICLE_FRAME = {
    "definition": "+x forward, +y left, +z up",
    "x": "vehicle forward",
    "y": "vehicle left",
    "z": "vehicle up",
    "handedness": "right-handed",
}


def write_json(path: Path, payload: Any) -> None:
    """Write JSON atomically-ish and fsync, so a power cut cannot truncate it."""
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    with open(tmp, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, indent=2, ensure_ascii=False, default=str)
        fh.flush()
        os.fsync(fh.fileno())
    os.replace(tmp, path)


def _git_state(repo_root: Path) -> dict:
    """Best-effort git provenance; absent git is not an error."""
    def _run(*args: str) -> str | None:
        try:
            out = subprocess.run(["git", "-C", str(repo_root), *args],
                                 capture_output=True, text=True, timeout=5,
                                 check=False)
            # rstrip only: `git status --porcelain` encodes the staged/unstaged
            # state in the first two columns, and a leading space is meaningful
            # (" M path").  strip() would eat it and shift every path by one
            # character.
            return out.stdout.rstrip("\n") if out.returncode == 0 else None
        except (OSError, subprocess.SubprocessError):
            return None

    commit = _run("rev-parse", "HEAD")
    if commit is None:
        return {"available": False}
    commit = commit.strip()
    status = _run("status", "--porcelain")
    return {
        "available": True,
        "commit": commit,
        "branch": (_run("rev-parse", "--abbrev-ref", "HEAD") or "").strip() or None,
        "dirty": bool(status and status.strip()),
        "dirty_files": parse_porcelain(status)[:50],
    }


def parse_porcelain(status: str | None) -> list[str]:
    """Extract paths from ``git status --porcelain`` output.

    Each line is ``XY <path>`` where X and Y are single status characters (a
    space is a valid, meaningful status), so the path starts at column 3.
    Rename entries are ``XY <old> -> <new>``; the new path is what matters.
    """
    paths: list[str] = []
    for line in (status or "").splitlines():
        if len(line) < 4:
            continue
        path = line[3:].strip()
        if " -> " in path:
            path = path.split(" -> ", 1)[1]
        paths.append(path.strip('"'))
    return paths


def _package_versions() -> dict:
    versions: dict[str, str | None] = {}
    for name in ("cv2", "mediapipe", "numpy", "serial", "yaml"):
        try:
            module = __import__(name)
            versions[name] = getattr(module, "__version__", "unknown")
        except Exception:
            versions[name] = None
    return versions


def _mount_info(path: Path) -> dict:
    """Filesystem, device and UUID backing *path* -- recorded, never changed."""
    info: dict[str, Any] = {"path": str(path)}
    try:
        out = subprocess.run(
            ["findmnt", "-n", "-o", "SOURCE,TARGET,FSTYPE,OPTIONS,UUID",
             "--target", str(path)],
            capture_output=True, text=True, timeout=5, check=False).stdout.strip()
        if out:
            parts = out.split(None, 4)
            keys = ["source", "mountpoint", "fstype", "options", "uuid"]
            info.update(dict(zip(keys, parts)))
    except (OSError, subprocess.SubprocessError):
        pass
    try:
        import shutil
        usage = shutil.disk_usage(str(path))
        info["total_gb"] = round(usage.total / (1024 ** 3), 2)
        info["free_gb"] = round(usage.free / (1024 ** 3), 2)
    except OSError:
        pass
    return info


def collect_metadata(*, session_id: str, config: dict, storage_root,
                     session_root: Path, devices: dict,
                     resolved: dict, repo_root: Path) -> dict:
    """Everything needed to interpret this session months from now."""
    uname = platform.uname()
    return {
        "schema_version": "oasis-daq/0.1",
        "session_id": session_id,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "created_local": datetime.now().astimezone().isoformat(),
        "host": {
            "hostname": uname.node,
            "system": uname.system,
            "release": uname.release,
            "version": uname.version,
            "machine": uname.machine,
            "processor": uname.processor,
            "model": _pi_model(),
        },
        "os_release": _os_release(),
        "python": {
            "version": sys.version,
            "version_info": list(sys.version_info[:3]),
            "executable": sys.executable,
        },
        "packages": _package_versions(),
        "git": _git_state(repo_root),
        "clock": {
            "source": clock.clock_source(),
            "monotonic_ns_at_start": clock.monotonic_ns(),
            "wall_time_ns_at_start": clock.wall_time_ns(),
            "timezone": datetime.now().astimezone().tzname(),
        },
        "storage": {
            "root": str(storage_root.path),
            "is_fallback": storage_root.is_fallback,
            "reason": storage_root.reason,
            "session_root": str(session_root),
            "mount": _mount_info(storage_root.path),
        },
        "devices": devices,
        "resolved_devices": resolved,
        "camera_installation": {
            "position": config.get("camera", {}).get(
                "mount_position", "driver_side_a_pillar_upper"),
            "count": 1,
        },
        "frames": {
            "vehicle_frame": VEHICLE_FRAME,
            "imu_sensor_frame": "as reported by the WIT unit, before the "
                                "mounting rotation is applied",
            "imu_mounting": config.get("imu", {}).get("mounting", {}),
        },
        "config": config,
    }


def _pi_model() -> str | None:
    try:
        return Path("/proc/device-tree/model").read_text().strip("\x00").strip()
    except OSError:
        return None


def _os_release() -> dict:
    out: dict[str, str] = {}
    try:
        for line in Path("/etc/os-release").read_text().splitlines():
            if "=" in line:
                key, _, value = line.partition("=")
                out[key] = value.strip().strip('"')
    except OSError:
        pass
    return out


def _file_entry(path: Path, session_root: Path, hash_limit_mb: float = 64.0) -> dict:
    """One manifest row.  Large files are sized but not hashed."""
    try:
        size = path.stat().st_size
    except OSError:
        return {"path": str(path.relative_to(session_root)), "missing": True}
    entry: dict[str, Any] = {
        "path": str(path.relative_to(session_root)),
        "bytes": size,
    }
    if size <= hash_limit_mb * 1024 * 1024:
        digest = hashlib.sha256()
        try:
            with open(path, "rb") as fh:
                for block in iter(lambda: fh.read(1 << 20), b""):
                    digest.update(block)
            entry["sha256"] = digest.hexdigest()
        except OSError:
            entry["sha256"] = None
    else:
        entry["sha256"] = None
        entry["sha256_skipped_reason"] = f"larger than {hash_limit_mb:g} MB"
    return entry


def build_manifest(session_root: Path) -> dict:
    """Inventory every file the session produced."""
    files = []
    total = 0
    for path in sorted(session_root.rglob("*")):
        if not path.is_file() or path.name.endswith(".tmp"):
            continue
        # INCOMPLETE is removed right after the manifest is written on a clean
        # close, so listing it would ship a manifest entry that can never be
        # verified.
        if path.name in ("manifest.json", "INCOMPLETE"):
            continue
        entry = _file_entry(path, session_root)
        files.append(entry)
        total += entry.get("bytes", 0)
    return {
        "schema_version": "oasis-daq/0.1",
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "session_root": str(session_root),
        "file_count": len(files),
        "total_bytes": total,
        "total_mb": round(total / (1024 * 1024), 2),
        "files": files,
    }


def timestamp_quality(timestamps: list[int], expected_hz: float | None = None) -> dict:
    """Monotonicity, duplicates, gaps, rate and jitter for one stream."""
    n = len(timestamps)
    result: dict[str, Any] = {
        "count": n, "monotonic": True, "backward_steps": 0, "duplicates": 0,
        "gaps": 0, "large_gaps": 0, "gap_threshold_s": None,
        "estimated_missed_samples": 0, "bimodal": False,
        "max_gap_s": None, "measured_hz": None,
        "jitter_ms_p50": None, "jitter_ms_p95": None, "duration_s": None,
    }
    if n < 2:
        return result

    deltas: list[int] = []
    for prev, cur in zip(timestamps, timestamps[1:]):
        d = cur - prev
        if d < 0:
            result["monotonic"] = False
            result["backward_steps"] += 1
            continue
        if d == 0:
            result["duplicates"] += 1
            continue
        deltas.append(d)

    duration = (timestamps[-1] - timestamps[0]) / 1e9
    result["duration_s"] = round(duration, 3)
    if duration > 0:
        result["measured_hz"] = round((n - 1) / duration, 4)
    if not deltas:
        return result

    ordered = sorted(deltas)
    median = ordered[len(ordered) // 2]

    # A strongly bimodal interval distribution has no meaningful "typical"
    # interval, so the gap heuristic produces nonsense on it.  The GPS row
    # series is bimodal by construction -- GGA and RMC arrive ~0.3 ms apart,
    # then ~1 s of silence -- which yielded "gaps=103, missed=5745" on a
    # perfectly healthy 105-second recording.  Say so instead of reporting it.
    p10 = ordered[int(len(ordered) * 0.10)]
    p90 = ordered[int(len(ordered) * 0.90)]
    if p10 > 0 and p90 / p10 > 10:
        result["bimodal"] = True
        result["note"] = ("interval distribution is bimodal; gap statistics do "
                          "not apply to this series")
        result["max_gap_s"] = round(max(deltas) / 1e9, 4)
        jitter = sorted(abs(d - median) / 1e6 for d in deltas)
        result["jitter_ms_p50"] = round(jitter[len(jitter) // 2], 3)
        result["jitter_ms_p95"] = round(
            jitter[min(int(len(jitter) * 0.95), len(jitter) - 1)], 3)
        result["gaps"] = None
        result["large_gaps"] = None
        result["estimated_missed_samples"] = None
        return result

    # A gap is an interval long enough to imply at least one missed sample.
    # The previous 3x threshold was far too loose: on a 1 Hz GPS it worked out
    # to 2.8 s, so a real three-fix outage was not counted at all.
    result["gaps"] = sum(1 for d in deltas if d > median * 1.75)
    result["large_gaps"] = sum(1 for d in deltas if d > median * 3)
    result["gap_threshold_s"] = round(median * 1.75 / 1e9, 6)
    result["max_gap_s"] = round(max(deltas) / 1e9, 4)
    result["estimated_missed_samples"] = sum(
        int(round(d / median)) - 1 for d in deltas if d > median * 1.75)

    jitter = sorted(abs(d - median) / 1e6 for d in deltas)
    result["jitter_ms_p50"] = round(jitter[len(jitter) // 2], 3)
    result["jitter_ms_p95"] = round(jitter[min(int(len(jitter) * 0.95),
                                               len(jitter) - 1)], 3)
    if expected_hz:
        result["expected_hz"] = expected_hz
        if result["measured_hz"]:
            result["rate_error_percent"] = round(
                (result["measured_hz"] - expected_hz) / expected_hz * 100.0, 2)
    return result


def build_summary(*, session_id: str, state: str, started_ns: int, ended_ns: int,
                  started_wall_ns: int, ended_wall_ns: int,
                  workers: dict, storage: dict, events: dict,
                  timestamps: dict, notes: str = "") -> dict:
    duration_s = (ended_ns - started_ns) / 1e9 if ended_ns > started_ns else 0.0
    return {
        "schema_version": "oasis-daq/0.1",
        "session_id": session_id,
        "final_state": state,
        "started_utc": datetime.fromtimestamp(started_wall_ns / 1e9,
                                              timezone.utc).isoformat(),
        "ended_utc": datetime.fromtimestamp(ended_wall_ns / 1e9,
                                            timezone.utc).isoformat(),
        "duration_s": round(duration_s, 3),
        "duration_hms": _hms(duration_s),
        "clock_source": clock.clock_source(),
        "workers": workers,
        "storage": storage,
        "events": events,
        "timestamp_quality": timestamps,
        "notes": notes,
    }


def _hms(seconds: float) -> str:
    seconds = int(seconds)
    return f"{seconds // 3600:02d}:{(seconds % 3600) // 60:02d}:{seconds % 60:02d}"


def write_route_geojson(path: Path, track: list[tuple[float, float]],
                        properties: dict | None = None) -> dict:
    """GeoJSON LineString of the driven route (lon, lat order per RFC 7946)."""
    props = dict(properties or {})
    props.setdefault("point_count", len(track))
    feature_collection = {
        "type": "FeatureCollection",
        "features": [{
            "type": "Feature",
            "geometry": {
                "type": "LineString",
                "coordinates": [[lon, lat] for lon, lat in track],
            },
            "properties": props,
        }] if len(track) >= 2 else ([{
            "type": "Feature",
            "geometry": {"type": "Point", "coordinates": list(track[0])},
            "properties": props,
        }] if track else []),
    }
    write_json(path, feature_collection)
    return feature_collection


def route_summary(track: list[tuple[float, float]]) -> dict:
    """Distance and bounding box of the track, computed with the haversine."""
    import math
    if not track:
        # Same keys as the populated case: a consumer of route_summary.json
        # should never have to special-case "no fix" by checking for a missing
        # key rather than a zero value.
        return {"point_count": 0, "distance_m": 0.0, "distance_km": 0.0,
                "bbox": None, "start": None, "end": None}
    distance = 0.0
    for (lon1, lat1), (lon2, lat2) in zip(track, track[1:]):
        r = 6371008.8
        p1, p2 = math.radians(lat1), math.radians(lat2)
        dp = p2 - p1
        dl = math.radians(lon2 - lon1)
        a = (math.sin(dp / 2) ** 2
             + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2)
        distance += 2 * r * math.asin(min(1.0, math.sqrt(a)))
    lons = [p[0] for p in track]
    lats = [p[1] for p in track]
    return {
        "point_count": len(track),
        "distance_m": round(distance, 2),
        "distance_km": round(distance / 1000.0, 4),
        "bbox": [min(lons), min(lats), max(lons), max(lats)],
        "start": list(track[0]),
        "end": list(track[-1]),
    }


def write_route_map(path: Path, track: list[tuple[float, float]],
                    session_id: str, summary: dict) -> None:
    """Self-contained route map.

    Leaflet is loaded from a CDN when the *viewing* machine has internet; when
    it does not, an inline SVG fallback draws the track on a plain grid so the
    page is never blank.  No tiles are proxied through the Pi.
    """
    coords = json.dumps([[lat, lon] for lon, lat in track])
    center = ([track[len(track) // 2][1], track[len(track) // 2][0]]
              if track else [0, 0])
    html = _ROUTE_MAP_HTML.format(
        session_id=session_id,
        coords=coords,
        center=json.dumps(center),
        summary=json.dumps(summary, indent=2),
        point_count=len(track),
        distance_km=summary.get("distance_km", 0.0),
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(html, encoding="utf-8")


_ROUTE_MAP_HTML = """<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>OASIS DAQ route -- {session_id}</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
<style>
  body {{ margin:0; font:14px/1.5 system-ui,sans-serif; background:#111; color:#eee; }}
  header {{ padding:10px 16px; background:#1b1b1b; border-bottom:1px solid #333; }}
  h1 {{ font-size:16px; margin:0 0 4px; }}
  #map {{ height:70vh; background:#222; }}
  #fallback {{ display:none; height:70vh; }}
  pre {{ padding:12px 16px; margin:0; font-size:12px; color:#9ad; overflow:auto; }}
  .muted {{ color:#888; }}
</style></head><body>
<header>
  <h1>OASIS DAQ route &mdash; {session_id}</h1>
  <div class="muted">{point_count} GPS points &middot; {distance_km} km</div>
</header>
<div id="map"></div>
<svg id="fallback" viewBox="0 0 1000 600" preserveAspectRatio="xMidYMid meet"></svg>
<pre>{summary}</pre>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script>
const TRACK = {coords};
const CENTER = {center};

function drawFallback() {{
  // No tiles (or no internet): plot the raw track on a plain grid instead of
  // leaving the page blank.
  document.getElementById('map').style.display = 'none';
  const svg = document.getElementById('fallback');
  svg.style.display = 'block';
  if (!TRACK.length) {{
    svg.innerHTML = '<text x="500" y="300" fill="#888" text-anchor="middle">no GPS fix recorded</text>';
    return;
  }}
  const lats = TRACK.map(p => p[0]), lons = TRACK.map(p => p[1]);
  const minLat = Math.min(...lats), maxLat = Math.max(...lats);
  const minLon = Math.min(...lons), maxLon = Math.max(...lons);
  const sx = (maxLon - minLon) || 1e-6, sy = (maxLat - minLat) || 1e-6;
  const pts = TRACK.map(p => {{
    const x = 60 + ((p[1] - minLon) / sx) * 880;
    const y = 540 - ((p[0] - minLat) / sy) * 480;
    return x.toFixed(1) + ',' + y.toFixed(1);
  }}).join(' ');
  let grid = '';
  for (let i = 0; i <= 10; i++) {{
    grid += `<line x1="${{60 + i * 88}}" y1="60" x2="${{60 + i * 88}}" y2="540" stroke="#2a2a2a"/>`;
    grid += `<line x1="60" y1="${{60 + i * 48}}" x2="940" y2="${{60 + i * 48}}" stroke="#2a2a2a"/>`;
  }}
  svg.innerHTML = grid +
    `<polyline points="${{pts}}" fill="none" stroke="#4da3ff" stroke-width="2.5"/>` +
    `<text x="500" y="580" fill="#888" text-anchor="middle" font-size="13">offline fallback -- track only, no basemap</text>`;
}}

try {{
  if (typeof L === 'undefined') throw new Error('leaflet unavailable');
  const map = L.map('map').setView(CENTER, 15);
  const tiles = L.tileLayer('https://{{s}}.tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png', {{
    maxZoom: 19, attribution: '&copy; OpenStreetMap contributors'
  }});
  let tileFailures = 0;
  tiles.on('tileerror', () => {{ if (++tileFailures > 4) drawFallback(); }});
  tiles.addTo(map);
  if (TRACK.length >= 2) {{
    const line = L.polyline(TRACK, {{color:'#4da3ff', weight:4}}).addTo(map);
    map.fitBounds(line.getBounds(), {{padding:[30,30]}});
    L.circleMarker(TRACK[0], {{radius:6,color:'#3c3',fill:true}}).addTo(map).bindPopup('start');
    L.circleMarker(TRACK[TRACK.length-1], {{radius:6,color:'#c33',fill:true}}).addTo(map).bindPopup('end');
  }} else if (TRACK.length === 1) {{
    L.circleMarker(TRACK[0], {{radius:6,color:'#3c3',fill:true}}).addTo(map);
  }} else {{
    drawFallback();
  }}
}} catch (e) {{
  drawFallback();
}}
</script>
</body></html>
"""
