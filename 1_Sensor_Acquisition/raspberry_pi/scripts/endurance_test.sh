#!/usr/bin/env bash
# Run a timed endurance session and audit the result.
#
# Uses the running daemon (systemd) so the test exercises exactly the production
# path.  Read-only with respect to existing data: it starts a new session,
# stops it, and reads back what was written.
#
#   ./endurance_test.sh [minutes]      default 30
set -euo pipefail

MINUTES="${1:-30}"
PY="${OASIS_PYTHON:-/home/esw/sensor-env/bin/python}"
CTL="${OASIS_CTL:-/home/esw/sensor-env/bin/oasis-daqctl}"
INTERVAL=60
PORT="${OASIS_PORT:-8080}"

log() { printf '[%s] %s\n' "$(date +%H:%M:%S)" "$*"; }

command -v vcgencmd >/dev/null && log "throttled before: $(vcgencmd get_throttled)"

log "pre-flight check"
"$CTL" check || { echo "check failed; not starting"; exit 1; }

log "starting a ${MINUTES} min session"
START_JSON=$("$CTL" start --notes "endurance ${MINUTES}min" --suffix endurance)
echo "$START_JSON"
SESSION_ROOT=$("$PY" -c "import json,sys;print(json.loads(sys.argv[1])['session_root'])" "$START_JSON")

END=$(( $(date +%s) + MINUTES * 60 ))
printf '%-9s%-9s%-9s%-10s%-9s%-8s%-8s%-7s%-7s%-6s\n' \
       time elapsed cam_fps recorded pose_fps imu_hz gps_fix cpu_% temp_C errors
while [ "$(date +%s)" -lt "$END" ]; do
  "$PY" "$(dirname "${BASH_SOURCE[0]}")/_endurance_monitor.py" "$PORT" || true
  # An under-voltage event is the most likely cause of data loss on this rig.
  if command -v vcgencmd >/dev/null; then
    T=$(vcgencmd get_throttled | cut -d= -f2)
    [ "$((T))" -ne 0 ] && log "POWER WARNING: throttled=$T"
  fi
  # The external SSD dropping off the bus is the other one.
  findmnt -n "$(dirname "$(dirname "$SESSION_ROOT")")" >/dev/null 2>&1 || \
    log "STORAGE WARNING: session filesystem is no longer mounted"
  sleep "$INTERVAL"
done

log "stopping"
"$CTL" stop --reason "endurance ${MINUTES}min complete"

command -v vcgencmd >/dev/null && log "throttled after: $(vcgencmd get_throttled)"

log "audit"
"$CTL" summarize "$SESSION_ROOT"
echo
log "session root: $SESSION_ROOT"
