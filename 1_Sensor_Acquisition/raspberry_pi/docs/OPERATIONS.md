# OASIS DAQ v0.1 — operations

## Install (once)

```bash
# 1. Python environment (the DAQ runs in /home/esw/sensor-env, Python 3.11)
/home/esw/sensor-env/bin/python -m pip install -e "/home/esw/ESW/1_Sensor_Acquisition/raspberry_pi"

# 2. Pose model (~6 MB, deliberately not in git)
bash "1_Sensor_Acquisition/raspberry_pi/scripts/fetch_pose_model.sh" lite

# 3. udev rules + systemd unit (idempotent, backs up anything it replaces)
sudo bash "1_Sensor_Acquisition/raspberry_pi/scripts/install.sh"

# 4. Verify
oasis-daqctl check --probe --probe-imu-baud
```

`install.sh` never touches `/etc/fstab` and never mounts, formats or partitions
the SSD.

## The directory name contains a space

`1_Sensor_Acquisition` sorts first in the GitHub file listing, which is why it is
named that way. The space is a real trap in three places, all of which are
already handled in the tracked files — do not "simplify" them back:

| Place | Rule | What happens otherwise |
|---|---|---|
| `systemd` `ExecStart=` | the path **must be quoted** | systemd splits on whitespace and passes `/home/esw/ESW/01.` and `Data_acquisition/configs/daq.yaml` as two arguments |
| `systemd` `WorkingDirectory=` | the path must **not** be quoted | it takes the rest of the line verbatim, so quotes become part of the path: `path is not absolute` |
| `systemd` `Documentation=` | percent-encode the space as `%%20` | it is a space-separated URL list, and a single `%` starts a systemd specifier (`Invalid slot`) |

In the shell, always quote it:

```bash
pip install -e "1_Sensor_Acquisition/raspberry_pi"
pytest "1_Sensor_Acquisition/raspberry_pi/tests/" -q
```

Verify a unit-file edit with `systemd-analyze verify /etc/systemd/system/oasis-daq.service`
— it prints nothing when the file is clean.

## Daily use

```bash
sudo systemctl start oasis-daq      # (already enabled at boot)
oasis-daqctl status                 # confirm READY and the storage root
oasis-daqctl start --notes "city driving, driver A"
#   ... drive ...
oasis-daqctl stop
oasis-daqctl summarize /media/esw/T7/OASIS_DAQ/sessions/<date>/<session>
```

**Check the storage root before every session.** If `oasis-daqctl status` shows
`/home/esw/oasis-daq-fallback`, the external SSD was not mounted when the daemon
resolved storage. The supervisor re-resolves on `start`, so simply mounting the
SSD (open the file manager, or `udisksctl mount -b /dev/sda1`) and pressing
START again puts the session on the SSD. See `BLOCKERS.md` §5.

## From the laptop

Browse to `http://<pi-ip>:8080` — on this machine `http://192.168.35.32:8080`,
or `http://esw-desktop.local:8080` via mDNS.

The dashboard shows session state and elapsed time, a live camera preview with
the MediaPipe skeleton overlaid, pose confidence and lean metrics, GPS
fix/satellites/HDOP/position/speed/course, IMU acceleration/gyro/orientation and
g-force, live sensor rates and camera FPS/drops, SSD free space and estimated
recording time left, recent events, a live route map, and START/STOP.

**The laptop is optional.** Closing the browser, losing Wi-Fi or blocking the
port does not affect a running session — this is tested (`TEST_RESULTS.md`).
Original data always goes to the SSD, never over the network.

The map needs internet **on the laptop** for OpenStreetMap tiles; the Pi does not
proxy them. With no tiles the map falls back to a plain grid with the track
drawn on it rather than going blank.

### API

| method | path | purpose |
|---|---|---|
| GET | `/api/status` | full state, all workers, storage |
| GET | `/api/health` | one health sample (CPU, temp, memory, disk latency) |
| GET | `/api/latest` | newest GPS row, IMU row and pose result |
| GET | `/api/preview.jpg` | single downscaled preview frame |
| GET | `/api/stream.mjpg` | multipart MJPEG preview stream |
| GET | `/api/route.geojson` | live track so far |
| POST | `/api/session/start` | `{"notes": "...", "suffix": "..."}` |
| POST | `/api/session/stop` | `{"reason": "..."}` |

There is **no authentication**. Only expose port 8080 on a trusted network.

## CLI

```bash
oasis-daqctl check [--probe] [--probe-seconds N] [--probe-imu-baud] [--json FILE]
oasis-daqctl status [--json]
oasis-daqctl start [--notes TEXT] [--suffix TEXT]
oasis-daqctl stop [--reason TEXT]
oasis-daqctl summarize <session-dir> [--rebuild]
oasis-daqctl daemon [--record] [--duration SECONDS]      # systemd uses this
```

`start`/`stop`/`status` talk to the running daemon over its HTTP API — the
daemon is the single owner of the devices, and two owners would corrupt a
session. `daemon --duration` is for bench and endurance runs.

## Configuration

`1_Sensor_Acquisition/raspberry_pi/configs/daq.yaml`. Values worth knowing:

| key | default | note |
|---|---|---|
| `storage.roots` | `/media/esw/T7/OASIS_DAQ`, `/mnt/T/OASIS_DAQ` | first mounted+writable wins |
| `storage.min_free_gb_start` | 20 | refuses to start below this |
| `storage.min_free_gb_stop` | 5 | closes the session cleanly and goes ERROR |
| `storage.fsync_interval_s` | 15 | bounds data loss on power cut |
| `session.autostart_recording` | **false** | see the warning below |
| `camera.capture_fps` | 100 | the device offers only 100/120 at 1280x800 |
| `camera.record_decimation` | 4 | 25 fps recorded, ≈3.9 GB/h. `1` keeps everything (≈16 GB/h) |
| `camera.segment_seconds` | 60 | worst-case video loss window |
| `pose.target_fps` | 10 | measured ceiling is ~13 fps |
| `dashboard.port` | 8080 | |
| `imu.mounting.rotation_matrix` | identity | **must be measured once the IMU is mounted in the vehicle** |

**Do not set `autostart_recording: true` yet.** The SSD is mounted by udisks2
after login, so a boot-time session would land on the microSD fallback.

Restart after editing: `sudo systemctl restart oasis-daq`.

## Reading the results

```bash
oasis-daqctl summarize <session-dir>
```

Check, in order:

1. **`INCOMPLETE` present?** The session was interrupted; expect the last
   ~15 s of every stream and the in-progress video segment to be missing.
2. **`timestamp_quality`** — `monotonic` must be true and `duplicates` should be
   0. Non-zero duplicates mean several samples share a host timestamp and the
   stream cannot be finely aligned.
3. **Worker counters** — `errors`, `dropped`, `frames_bad_checksum`,
   `estimated_drops`.
4. **`neutral.reason`** under `workers.pose` — if calibration failed, every
   `*_shift_*` and `lean_*` column is null.

**GPS shows `gaps` and ~1000 ms jitter p95 by design** — GGA and RMC arrive
together once per second. That is the receiver's burst behaviour, not a fault.

## Troubleshooting

| symptom | cause | action |
|---|---|---|
| `check` says a device is missing | not plugged in, or udev rules not installed | `ls -l /dev/oasis_*`; re-run `install.sh`; `bash scripts/inspect_hardware.sh` |
| storage root is the fallback | SSD not mounted when the daemon started | mount it, then `oasis-daqctl start` (re-resolves) |
| `start` refused, "GB free" | below `min_free_gb_start` | free space on the SSD, or lower the threshold deliberately |
| pose worker absent, `pose_disabled` event | model missing | `bash scripts/fetch_pose_model.sh lite` |
| camera FPS reads low for a few seconds | transient CPU spike | recovers on its own; sustained low FPS means real contention |
| session went to ERROR | recorder died, disk exhausted, or camera lost | `events.jsonl`, then `journalctl -u oasis-daq` |
| IMU baud unknown after a swap | different firmware | `oasis-daqctl check --probe --probe-imu-baud` (read-only) |

## Power — read this

The Pi is on a **5 V / 3 A (15 W) PD supply**. Raspberry Pi 5 expects 5 V / 5 A
for full capability; on a 15 W supply the firmware caps **total USB current at
600 mA**. The Samsung T7 alone can exceed that during sustained writes, and four
USB devices are attached (GPS, IMU, camera, SSD).

An unexplained SSD disconnect and reboot occurred during the first endurance
run, which is consistent with exceeding that budget.

* Watch `vcgencmd get_throttled` — `0x0` is healthy. Bit 0 = under-voltage now,
  bit 16 = under-voltage has occurred since boot.
* Do not add USB devices during a session.
* Prefer a 5 V / 5 A supply, or power the SSD from a self-powered hub, before
  long drives.
* The SSD currently sits on a **USB 2.0** port (480 Mbps). That is ample for the
  4.4 MB/s the DAQ produces; moving it to a USB 3.0 port would raise both
  throughput and current draw.

## Recovering an interrupted session

Nothing is ever overwritten. A new session always gets a new directory, and a
crashed daemon does not resume into an old one.

```bash
find /media/esw/T7/OASIS_DAQ/sessions -name INCOMPLETE   # list interrupted sessions
oasis-daqctl summarize <session-dir> --rebuild           # rebuild manifest.json
ffprobe -count_frames <session>/camera/driver_000.mkv    # verify each segment
```

Completed Matroska segments survive an abrupt kill; the in-progress one is
usually still playable but truncated (821 of ~1500 frames recovered in testing).

## Uninstall

```bash
sudo bash "1_Sensor_Acquisition/raspberry_pi/scripts/uninstall.sh"    # removes the unit and udev rules
```

Recorded sessions on the SSD are never touched.
