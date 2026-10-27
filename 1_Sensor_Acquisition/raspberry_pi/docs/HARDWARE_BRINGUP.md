# OASIS DAQ v0.1 — hardware bring-up

Everything here was measured on the actual bench hardware on 2026-08-27/28.
No value is copied from a datasheet.

## Host

| | |
|---|---|
| Board | Raspberry Pi 5 Model B Rev 1.1 |
| OS | Ubuntu 24.04.4 LTS (noble), `aarch64` |
| Kernel | 6.8.0-1047-raspi |
| Python | 3.12.3 system; **3.11.16 in `/home/esw/sensor-env`** (the venv the DAQ runs in) |
| Cores / RAM | 4 / 7933 MB |
| Idle SoC temp | 41–47 °C, `vcgencmd get_throttled=0x0` (never throttled during testing) |

## Attached devices, as identified

```
$ lsusb
1546:01a7 U-Blox AG [u-blox 7]                    -> GPS
1a86:7523 QinHeng Electronics CH340 serial conv.  -> IMU bridge
0c45:6366 Microdia Webcam Vitade AF               -> Arducam OV9281 camera
04e8:4001 Samsung Electronics PSSD T7             -> external SSD
```

| Role | Node | Stable identity | udev symlink |
|---|---|---|---|
| GPS | `/dev/ttyACM0` | `usb-u-blox_AG_-_www.u-blox.com_u-blox_7_-_GPS_GNSS_Receiver-if00` | `/dev/oasis_gps` |
| IMU | `/dev/ttyUSB0` | `usb-1a86_USB_Serial-if00-port0` | `/dev/oasis_imu` |
| Camera | `/dev/video0` | V4L2 card `Arducam OV9281 USB Camera` | `/dev/oasis_camera` |

**Neither serial adapter reports a USB serial number**, so the udev rules match
on VID/PID. `1a86:7523` is a *generic* CH340 id — a second CH340 device would
collide. `udev/99-oasis-daq.rules` carries a commented port-pinned variant for
that case.

`/dev/video1` exists but advertises no capture format (it is the UVC metadata
node); the camera udev rule matches `ATTR{index}=="0"` and device enumeration
filters it out, so only the real capture node is ever opened.

## GPS — u-blox 7

Identified from its own startup banner:

```
$GPTXT,01,01,02,u-blox ag - www.u-blox.com*50
$GPTXT,01,01,02,HW  UBX-G70xx   00070000 FF7FFFFFo*69
$GPTXT,01,01,02,ROM CORE 1.00 (59842) Jun 27 2012 17:43:52*59
$GPTXT,01,01,02,PROTVER 14.00*1E
$GPTXT,01,01,02,ANTSTATUS=OK*3B
```

* **Native USB CDC-ACM.** Host-side baud is irrelevant; `gps.baud: 9600` is kept
  only so a USB-UART variant of the same receiver would also work.
* Sentences emitted: `GGA, RMC, VTG, GSA, GSV, GLL, TXT`.
* **1 Hz** position rate → the DAQ writes **2 rows/s** (one per GGA and one per
  RMC, each carrying the latest complementary fields).
* **0 checksum failures** across every test — 792 sentences in the durability
  run alone.
* **No fix indoors.** `GPGGA` quality `0`, `GPRMC` status `V`, 3–4 satellites
  tracked at SNR 22–23. See `BLOCKERS.md` §2.

Receiver configuration is **not** modified in v0.1.

## IMU — WITMOTION WIT standard protocol behind a CH340

Protocol was confirmed by frame quality, not by "bytes appeared". `oasis-daqctl
check --probe --probe-imu-baud` scans the candidate list and the result is
unambiguous:

| baud | bytes | valid frames | bad checksum | discarded | frame ratio | verdict |
|---|---|---|---|---|---|---|
| **9600** | 1220 | **110** | 0 | 0 | **0.992** | **WIT confirmed** |
| 115200 | 3072 | 0 | 0 | 3062 | 0.000 | rejected |
| 57600 | 3072 | 0 | 0 | 3062 | 0.000 | rejected |
| 38400 | 2304 | 0 | 0 | 2294 | 0.000 | rejected |
| 19200 | 1670 | 0 | 0 | 1660 | 0.000 | rejected |

Every wrong baud still produces bytes; none produces a valid frame. That is
exactly why the probe scores `frames_ok × 11 / bytes_received` rather than
"did anything arrive".

**Frame format** (11 bytes, little-endian payload, checksum = low byte of the
sum of the first 10):

```
0x55 <type> d0 d1 d2 d3 d4 d5 d6 d7 <checksum>
types seen: 0x51 acc, 0x52 gyro, 0x53 angle, 0x54 mag
```

Worked example from a real frame `55 51 59 03 8e fb 49 fa 23 0a fb`:

```
sum(first 10) & 0xFF = 0xFB  == declared 0xFB          checksum OK
ax = 0x0359 =  857 -> 857/32768*16 g   =  +0.4175 g
ay = 0xfb8e = -1138 -> -0.5557 g
az = 0xfa49 = -1463 -> -0.7153 g
|a| = 0.9971 g                                          physically correct at rest
temp = 0x0a23 = 2595 -> 25.95 °C
```

* **9600 baud, 10 Hz** per packet type (40.7 frames/s total).
* **0 checksum failures** in every run (3234 frames in one 83 s session).
* Reference: `sensor-drivers/WitStandardProtocol_JY901` — the WITMOTION official
  protocol repository, already present on this machine and used only as
  documentation. No third-party pip package is used for the protocol.
* 10 Hz is a real ceiling at this baud — see `BLOCKERS.md` §4.

Raw serial bytes are archived to `imu/raw_*.bin` before any parsing, so a
protocol misunderstanding can always be replayed against the original stream.

## Camera — Arducam OV9281 USB (global shutter, monochrome)

```
$ v4l2-ctl --device=/dev/video0 --list-formats-ext
[0]: 'MJPG'  1280x800 @120/100 fps   1280x720 @120   800x600 @120/100
             640x480 @120/100        320x240 @120/100
[1]: 'YUYV'  1280x800 @10 fps        1280x720 @10 fps
```

* **YUYV is unusable** — 10 fps at working resolution. MJPG is the only viable
  format.
* The device advertises **only 100 and 120 fps** at 1280x800. Requesting 30 fps
  yields 100 (`v4l2-ctl --set-parm=30` → `Frame rate set to 100.000 fps`), so
  there is no low-rate mode to select; the DAQ captures at 100 and **decimates
  on write**.
* Measured sustained capture: **100.2 fps** via ffmpeg passthrough (1002 frames
  in 10 s), **~101 fps** in the running DAQ, 92 fps through the plain OpenCV
  raw path.
* Mean JPEG size ~37 kB; raw capture is ~4.4 MB/s (≈16 GB/h).
* `record_decimation: 4` → **25 fps recorded, ≈3.9 GB/h**, which is the shipped
  default. Set it to 1 to keep every captured frame.

Chosen profile: **1280x800 MJPG, capture 100 fps, record every 4th frame, 60 s
Matroska segments.**

Matroska was chosen over MP4 and verified by test, not by reputation: after
`SIGKILL` of the daemon, all three segments — including the **in-progress** one —
remained playable, and `ffprobe` counted 821 frames out of the partial segment.

## External SSD — Samsung PSSD T7

```
/dev/sda1  exfat  LABEL="T7"  UUID="8E1B-119F"  931.5 G
mounted by udisks2 at /media/esw/T7
  rw,nosuid,nodev,relatime,uid=1000,gid=1000,fmask=0022,dmask=0022,errors=remount-ro
```

The brief's "T drive" is this **exFAT volume labelled `T7`**, not a Windows
`T:`. It was **already mounted**, so the DAQ uses that mountpoint as-is and
creates only `OASIS_DAQ/` inside it. Nothing is formatted, repartitioned or
deleted; the pre-existing `수업/`, `프로젝트/`, `$RECYCLE.BIN/` and
`System Volume Information/` are untouched.

* Free space at bring-up: **363 GB** (569 GB already in use).
* fsync cost measured on this volume: **27 ms** for 64 KiB (vs 6.9 ms on the
  microSD root). At a 15 s fsync interval this is well under 1 % of a core.
* **It is mounted after desktop login, not at boot** — see `BLOCKERS.md` §5.
* An abrupt disconnect during testing produced
  `exFAT-fs (sda1): Volume was not properly unmounted` and
  `sd 0:0:0:0: [sda] Synchronize Cache(10) failed`. The volume and the existing
  data were intact afterwards; only the in-flight session was lost.

## Accelerator

`hailortcli` is **not installed** and no Hailo device appears in `lspci`.
Nothing Hailo-related was installed, removed or reconfigured. MediaPipe runs on
CPU via the TensorFlow Lite XNNPACK delegate and meets the v0.1 target, so Hailo
is not a dependency.

## Software installed during bring-up

The machine had neither `git` nor `gh`, and no SSH server.

| Package | Version | Why |
|---|---|---|
| `openssh-server` | 1:9.6p1-3ubuntu13.18 | remote access from the laptop |
| `git` | 2.43.0 | required by the task; was absent |
| `gh` | 2.45.0 | required by the task; was absent |
| `ffmpeg` | 7:6.1.1 | segment muxer for the recording path |
| `v4l-utils` | present | camera capability probing |
| `mediapipe` | **1.0.1** | pose landmarker (aarch64 wheel exists for py3.11) |
| `opencv-contrib-python` | 5.0.0.93 | pulled in by mediapipe |
| `PyYAML` | 6.0.3 | config |

**MediaPipe 1.x removed the legacy `mp.solutions` API.** Only the Tasks API
exists, which needs a `.task` bundle — fetched by `scripts/fetch_pose_model.sh`
into `~/.local/share/oasis-daq/models/` and deliberately kept out of git.

Measured on this Pi 5 with `pose_landmarker_lite`:

| input | sustained | inference p50 | inference p95 |
|---|---|---|---|
| 640x400 | 13.3 fps | 66 ms | 94 ms |
| 256x256 | 12.4 fps | 69 ms | 101 ms |

Input resolution barely matters — MediaPipe rescales internally — so 640x400 is
kept for landmark precision. `pose.target_fps` ships at **10** to leave headroom;
under a full live session the pose worker holds 9.3–9.7 fps with p95 ≈ 70 ms.

## Bring-up commands

```bash
bash "1_Sensor_Acquisition/raspberry_pi/scripts/inspect_hardware.sh"            # full read-only dump
bash "1_Sensor_Acquisition/raspberry_pi/scripts/fetch_pose_model.sh" lite       # ~6 MB model
sudo bash "1_Sensor_Acquisition/raspberry_pi/scripts/install.sh"                # udev + systemd
oasis-daqctl check --probe --probe-imu-baud             # verify everything
```

Expected result of the last command on this bench: `0 problem(s), 1 warning(s)`
— the warning being the indoor GPS fix.
