# OASIS DAQ v0.1 — architecture

## What it has to do

Two modes, one codebase:

```
standalone                          with a laptop attached
──────────                          ──────────────────────
GPS + IMU + A-pillar camera         Raspberry Pi (unchanged, still recording)
        │                                    │
   Raspberry Pi 5                            └── Ethernet / Wi-Fi
        │                                             │
 common monotonic timestamp                    laptop web browser
        │                                      (START/STOP, preview,
 external SSD: OASIS_DAQ/                       skeleton, GPS, IMU,
                                                rates, SSD, map)
```

The laptop is an **observer with two buttons**. Recording state lives entirely in
the Pi. Closing the browser, losing Wi-Fi, or killing the HTTP server does not
touch a running session — verified by blocking port 8080 mid-session and by
abruptly dropping an MJPEG stream (see `TEST_RESULTS.md`).

## Process and thread layout

```
oasis-daq.service  (systemd, user esw)
└── oasis-daqctl daemon                     ← one process owns every device
    ├── Supervisor                          state machine, session lifecycle
    ├── thread  gps_worker                  serial → raw_nmea.log + gps.csv
    ├── thread  imu_worker                  serial → raw_*.bin + imu.csv
    ├── thread  camera_worker               V4L2 → ffmpeg pipe + timestamps
    │   └── subprocess ffmpeg               MJPEG -c copy → segmented Matroska
    ├── thread  pose_worker (parent side)   feeds frames, writes results
    │   └── PROCESS oasis-pose              MediaPipe inference only
    ├── thread  health monitor              health.csv, thresholds, events
    └── thread  dashboard HTTP server       read-only observer + START/STOP
```

### Why MediaPipe is a separate process, not a thread

Pose inference costs ~66–83 ms of native work per frame on a Pi 5. Measured
during a live session: the pose child sits at **~60 % of one core** while the
main process — which holds the camera — stays at **~11 %** and keeps a steady
**101 fps** capture rate. Putting inference in the acquisition process would put
recorded FPS at the mercy of vision load, which the brief explicitly forbids.

The child receives only the newest JPEG and returns plain tuples; all file
writing stays in the parent, so the child is a pure function of its input and
can die without taking any raw logging with it.

### Why the other workers are threads

GPS (2 rows/s), IMU (10 Hz) and the camera recorder are I/O-bound and release the
GIL inside `serial.read()`, `VideoCapture.read()` and `pipe.write()`. Each runs
in its own `BaseWorker` thread with its own `try/except`: a worker that raises
records the fault, marks itself not-running and stops — it never stops another
sensor's logging.

## The recording path never decodes

```
V4L2 (MJPEG) ──read()──> encoded JPEG bytes ──┬── trim padding to EOI ──> ffmpeg -c copy
   CAP_PROP_CONVERT_RGB = 0                   ├── LatestSlot → pose process
                                              └── LatestSlot → dashboard preview
```

`CAP_PROP_CONVERT_RGB=0` makes OpenCV hand back the camera's own JPEG buffer
instead of decoding it. Recording is therefore a byte copy: no decode, no
re-encode, no colour conversion. The two consumers get the same bytes through
**newest-wins slots**, so a slow consumer can never apply back-pressure to
capture.

UVC pads each buffer to a fixed size; the padding is trimmed back to the final
`FFD9` marker before it is written (without this, ffmpeg logs `EOI missing,
emulating` on every frame).

## Back-pressure policy

| Path | Policy | Rationale |
|---|---|---|
| GPS / IMU rows | large bounded queue, drop counted | raw data must not be lost; overflow is an event, never a stall |
| camera → recorder | direct pipe write | a blocked recorder is a session-fatal error, not something to paper over |
| camera → pose | `LatestSlot`, newest wins | stale frames are worthless; skipping is correct |
| camera → preview | `LatestSlot`, newest wins | same, and it must never touch recording |
| pose → dashboard | `LatestSlot` | display only |

## Time

Every record carries `host_monotonic_ns` from `CLOCK_MONOTONIC_RAW` (the actual
source is recorded in `metadata.json`; `time.monotonic_ns` is the fallback) plus
`host_wall_time_ns` for human correlation. Nothing is resampled or interpolated
during acquisition — each source is written at its own rate and aligned in
post-processing.

**Timestamps are taken as close to arrival as the transport allows.** An early
version read `serial.read(256)` with a 1 s timeout, which blocked until the
buffer filled and gave a whole second of IMU frames one identical stamp — 669
duplicates in 809 samples. The workers now read only what has arrived
(`in_waiting`, 10 ms poll), which removed every duplicate and cut IMU jitter p95
from 84.8 ms to 3.2 ms.

`session_summary.json` audits monotonicity, duplicates, gaps, rate and jitter for
every stream, so a regression of this kind shows up in the summary rather than
silently corrupting an analysis.

## Durability

`flush()` only reaches the kernel page cache. When the USB SSD was pulled during
the first endurance run, **every CSV had 0 data rows and the video file was 0
bytes** — ~10 minutes gone. So:

* CSV/JSONL writers `fsync` on an interval (`storage.fsync_interval_s`, 15 s);
* the IMU raw-byte archive fsyncs on the same cadence and at each chunk close;
* completed video segments are fsynced as soon as ffmpeg lists them in
  `segments.csv`;
* segments are 60 s, so the in-progress segment bounds worst-case video loss.

An `INCOMPLETE` file is written when a session opens and removed only on a clean
close, so an interrupted session can never be mistaken for a complete one. A new
session always gets a fresh directory — a crashed daemon never resumes into an
old one.

## State machine

```
INIT ──prepare()──> READY ──start_session()──> RECORDING ──stop_session()──> SAVING ──> READY
  │                   │                            │                                     
  └── no writable ────┴──── device/disk/recorder fatal ──────────────────> ERROR
      storage
```

`SAVING` stops workers, writes route artefacts, audits timestamps, builds the
summary and manifest, then removes `INCOMPLETE`. Fatal conditions (recorder
died, disk below the stop threshold, camera lost) close the session cleanly
*and* land in `ERROR` — the data is always finalised first.

## Device identity

Kernel names are never treated as identity. Devices are resolved by, in order:
an explicit config override, the udev symlink (`/dev/oasis_gps`,
`/dev/oasis_imu`, `/dev/oasis_camera`), then USB VID/PID plus
`/dev/serial/by-id`. An ambiguous match is an error, not a guess. The resolved
identity is written into every session's `metadata.json`.

## What v0.1 deliberately does not do

* No interpolation or resampling during acquisition.
* No GPS receiver reconfiguration (records the confirmed factory settings).
* No IMU configuration writes (probing is read-only).
* No Hailo dependency — MediaPipe runs on CPU via XNNPACK.
* No map matching, routing or navigation; the map shows the recorded track only.
* No `/etc/fstab` changes and nothing that formats, partitions or deletes.
