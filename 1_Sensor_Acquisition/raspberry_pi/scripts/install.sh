#!/usr/bin/env bash
# Install the udev rules and the systemd unit.  Idempotent: safe to re-run.
#
# Never formats, mounts or modifies the external SSD, and never edits
# /etc/fstab.  The DAQ uses whatever mountpoint the SSD already has.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
ACQ="$REPO_ROOT/1_Sensor_Acquisition/raspberry_pi"
UNIT_SRC="$ACQ/systemd/oasis-daq.service"
UNIT_DST="/etc/systemd/system/oasis-daq.service"
RULES_SRC="$ACQ/udev/99-oasis-daq.rules"
RULES_DST="/etc/udev/rules.d/99-oasis-daq.rules"
PYTHON="${OASIS_PYTHON:-/home/esw/sensor-env/bin/python}"

log() { printf '==> %s\n' "$*"; }

[ "$(id -u)" -eq 0 ] || { echo "run with sudo: sudo $0" >&2; exit 1; }

log "repository: $REPO_ROOT"

# --- udev ------------------------------------------------------------------
if [ -f "$RULES_DST" ] && cmp -s "$RULES_SRC" "$RULES_DST"; then
  log "udev rules already current"
else
  if [ -f "$RULES_DST" ]; then
    cp -a "$RULES_DST" "$RULES_DST.bak.$(date +%Y%m%d%H%M%S)"
    log "backed up existing udev rules"
  fi
  install -m 0644 "$RULES_SRC" "$RULES_DST"
  log "installed $RULES_DST"
fi
udevadm control --reload-rules
udevadm trigger --subsystem-match=tty --subsystem-match=video4linux
sleep 1
log "udev symlinks now present:"
ls -l /dev/oasis_* 2>/dev/null | sed 's/^/    /' || \
  echo "    (none yet -- replug the devices or check the VID/PID in the rules)"

# --- systemd ---------------------------------------------------------------
if [ -f "$UNIT_DST" ] && cmp -s "$UNIT_SRC" "$UNIT_DST"; then
  log "systemd unit already current"
else
  if [ -f "$UNIT_DST" ]; then
    cp -a "$UNIT_DST" "$UNIT_DST.bak.$(date +%Y%m%d%H%M%S)"
    log "backed up existing unit"
  fi
  install -m 0644 "$UNIT_SRC" "$UNIT_DST"
  log "installed $UNIT_DST"
fi
systemctl daemon-reload

# --- sanity ----------------------------------------------------------------
if [ ! -x "$PYTHON" ]; then
  echo "WARNING: $PYTHON not found; set OASIS_PYTHON or fix the unit" >&2
fi
if ! "$PYTHON" -c 'import oasis_daq' 2>/dev/null; then
  echo "WARNING: oasis_daq is not importable by $PYTHON" >&2
  echo "         run: $PYTHON -m pip install -e \"$ACQ\"" >&2
fi

systemctl enable oasis-daq.service
log "enabled oasis-daq.service (starts on boot)"
log "start now with:  sudo systemctl start oasis-daq"
log "follow logs   :  journalctl -u oasis-daq -f"
