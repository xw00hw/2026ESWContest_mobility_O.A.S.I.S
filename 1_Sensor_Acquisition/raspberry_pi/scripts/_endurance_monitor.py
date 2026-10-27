#!/usr/bin/env python3
"""One status line for endurance_test.sh.

A separate file rather than inline `python -c` in the shell script: nesting
double quotes inside an f-string is a SyntaxError before Python 3.12, and the
target runs 3.11, so the inline form failed at runtime.
"""

from __future__ import annotations

import json
import sys
import time
import urllib.request


def _get(path: str, port: int) -> dict:
    url = f"http://127.0.0.1:{port}{path}"
    with urllib.request.urlopen(url, timeout=10) as resp:
        return json.loads(resp.read().decode())


def main() -> int:
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8080
    try:
        status = _get("/api/status", port)
    except Exception as exc:                       # keep the loop alive
        print(f"{time.strftime('%H:%M:%S')} status unavailable: {exc}")
        return 0
    try:
        health = _get("/api/health", port)
    except Exception:
        health = {}

    cam = status.get("camera") or {}
    pose = status.get("pose") or {}
    imu = status.get("imu") or {}
    gps = status.get("gps") or {}

    def num(value, width, prec=2):
        try:
            return format(float(value), f"{width}.{prec}f")
        except (TypeError, ValueError):
            return "-".rjust(int(width.lstrip('<>')) if width[1:].isdigit() else 6)

    print(
        f"{time.strftime('%H:%M:%S')} "
        f"{str(status.get('elapsed_hms', '-')):<9}"
        f"{num(cam.get('measured_fps'), '<9')}"
        f"{str(cam.get('frames_recorded', '-')):<10}"
        f"{num(pose.get('fps'), '<9')}"
        f"{num(imu.get('rate_hz'), '<8')}"
        f"{str(gps.get('fix_valid')):<8}"
        f"{num(health.get('cpu_percent'), '<7', 1)}"
        f"{num(health.get('cpu_temp_c'), '<7', 1)}"
        f"{str(status.get('events', {}).get('error', 0)):<6}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
