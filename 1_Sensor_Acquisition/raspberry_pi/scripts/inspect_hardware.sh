#!/usr/bin/env bash
# Dump everything needed to identify the DAQ hardware on this machine.
# Read-only: opens nothing for writing and changes no device configuration.
set -uo pipefail

section() { printf '\n########## %s ##########\n' "$1"; }

section "host"
uname -a; echo; cat /etc/os-release | head -3
[ -r /proc/device-tree/model ] && { echo -n "model: "; tr -d '\0' </proc/device-tree/model; echo; }
echo "python: $(python3 --version 2>&1)"; echo "cores: $(nproc)"
[ -r /sys/class/thermal/thermal_zone0/temp ] && \
  awk '{printf "soc temp: %.1f C\n", $1/1000}' /sys/class/thermal/thermal_zone0/temp
command -v vcgencmd >/dev/null && echo "throttled: $(vcgencmd get_throttled)"

section "memory / storage"
free -h
echo; df -h | grep -vE 'tmpfs|loop'
echo; lsblk -o NAME,TYPE,SIZE,FSTYPE,LABEL,UUID,MOUNTPOINTS,MODEL
echo; findmnt -o SOURCE,TARGET,FSTYPE,OPTIONS | grep -vE 'cgroup|proc|sysfs|devtmpfs|securityfs|pstore|bpf|tracefs|debugfs|configfs|fusectl|nsfs|ramfs'

section "usb"
lsusb
echo; lsusb -t 2>/dev/null

section "serial devices"
ls -l /dev/serial/by-id/ 2>/dev/null || echo "(no /dev/serial/by-id)"
echo; ls -l /dev/serial/by-path/ 2>/dev/null || echo "(no /dev/serial/by-path)"
echo; ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "(no tty devices)"
for dev in /dev/ttyUSB* /dev/ttyACM*; do
  [ -e "$dev" ] || continue
  echo "--- udevadm $dev ---"
  udevadm info --query=property --name="$dev" 2>/dev/null | \
    grep -E 'ID_VENDOR|ID_MODEL|ID_SERIAL|ID_PATH|ID_USB_DRIVER|DEVLINKS'
done

section "video devices"
v4l2-ctl --list-devices 2>/dev/null || echo "(v4l-utils not installed)"
for dev in /dev/video*; do
  [ -e "$dev" ] || continue
  if v4l2-ctl --device="$dev" --list-formats-ext 2>/dev/null | grep -q 'Size:'; then
    echo "--- $dev formats ---"
    v4l2-ctl --device="$dev" --list-formats-ext 2>/dev/null
    udevadm info --query=property --name="$dev" 2>/dev/null | \
      grep -E 'ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL_SHORT'
  fi
done

section "oasis stable symlinks"
ls -l /dev/oasis_* 2>/dev/null || echo "(udev rules not installed; run scripts/install.sh)"

section "accelerators"
command -v hailortcli >/dev/null && hailortcli fw-control identify 2>&1 || \
  echo "(hailortcli not present -- Hailo is not required for v0.1)"

section "oasis device view"
PY=${OASIS_PYTHON:-/home/esw/sensor-env/bin/python}
"$PY" -m oasis_daq.devices 2>/dev/null || echo "(oasis_daq not importable with $PY)"
