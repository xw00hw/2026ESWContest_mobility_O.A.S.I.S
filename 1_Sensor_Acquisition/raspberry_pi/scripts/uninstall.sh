#!/usr/bin/env bash
# Remove the systemd unit and udev rules.  Recorded sessions are never touched.
set -euo pipefail
[ "$(id -u)" -eq 0 ] || { echo "run with sudo: sudo $0" >&2; exit 1; }

log() { printf '==> %s\n' "$*"; }

if systemctl list-unit-files | grep -q '^oasis-daq\.service'; then
  systemctl stop oasis-daq.service 2>/dev/null || true
  systemctl disable oasis-daq.service 2>/dev/null || true
  log "stopped and disabled oasis-daq.service"
fi
rm -f /etc/systemd/system/oasis-daq.service
systemctl daemon-reload
log "removed the systemd unit"

rm -f /etc/udev/rules.d/99-oasis-daq.rules
udevadm control --reload-rules
udevadm trigger --subsystem-match=tty --subsystem-match=video4linux
log "removed the udev rules"

log "recorded sessions on the SSD were NOT touched."
