# OASIS DAQ v0.1

Standalone multi-sensor data acquisition for driver monitoring on a Raspberry Pi 5.

Records **GPS + IMU + a single A-pillar camera + MediaPipe pose** onto an
external SSD, on one common monotonic timebase, with an optional laptop
dashboard that can be connected and disconnected at will without disturbing the
recording.

```
GPS + IMU + A-pillar camera  ──►  Raspberry Pi 5  ──►  external SSD: OASIS_DAQ/
                                        │
                                        └── optional: http://<pi>:8080
                                            (START/STOP, preview, skeleton,
                                             GPS track, IMU, rates, SSD)
```

## Quick start

```bash
/home/esw/sensor-env/bin/python -m pip install -e .
bash scripts/fetch_pose_model.sh lite
sudo bash scripts/install.sh
oasis-daqctl check --probe --probe-imu-baud

sudo systemctl start oasis-daq
oasis-daqctl start --notes "first session"
oasis-daqctl stop
```

## Hardware this was built and measured against

| | measured, not assumed |
|---|---|
| Host | Raspberry Pi 5, Ubuntu 24.04.4, kernel 6.8.0-1047-raspi, aarch64 |
| GPS | u-blox 7 (UBX-G70xx, PROTVER 14.00), USB CDC-ACM, NMEA @1 Hz, 0 checksum errors |
| IMU | WITMOTION WIT protocol via CH340, **9600 baud, 10 Hz**, 0 checksum errors |
| Camera | Arducam OV9281 USB, MJPG **1280x800 @ 100 fps sustained** (YUYV caps at 10 fps) |
| Pose | MediaPipe 1.0.1 `pose_landmarker_lite`, ~13 fps ceiling, p50 66 ms / p95 94 ms |
| Storage | Samsung PSSD T7, exFAT, label `T7`, mounted at `/media/esw/T7` |

Full detail: [`1_Sensor_Acquisition/raspberry_pi/docs/HARDWARE_BRINGUP.md`](docs/HARDWARE_BRINGUP.md).

## Design in one paragraph

One process owns every device. The camera worker pulls **encoded** MJPEG buffers
(`CAP_PROP_CONVERT_RGB=0`) and pipes them to `ffmpeg -c copy`, so the recording
path never decodes a frame. MediaPipe runs in a **separate process** fed only the
newest frame, so vision load cannot drag down recorded FPS — measured live, the
pose child uses ~60 % of a core while the main process holds 101 fps at ~11 %.
GPS, IMU and camera each run in an isolated thread; one failing never stops
another's logging. Every record carries `CLOCK_MONOTONIC_RAW`, and nothing is
resampled during acquisition.

## Layout

```
1_Sensor_Acquisition/raspberry_pi/
├── configs/daq.yaml            every value verified on real hardware
├── src/oasis_daq/
│   ├── clock.py                CLOCK_MONOTONIC_RAW with a documented fallback
│   ├── devices.py              VID/PID + by-id resolution; never trusts ttyUSB0
│   ├── storage.py              session dirs, buffered writers, periodic fsync
│   ├── session.py              metadata, manifest, summary, route artefacts
│   ├── health.py               /proc and /sys sampling, no psutil
│   ├── workers.py              isolated worker threads, bounded queues, LatestSlot
│   ├── supervisor.py           state machine and session lifecycle
│   ├── cli.py                  oasis-daqctl
│   ├── gps/                    NMEA parser (own, checksum-strict) + worker
│   ├── imu/                    WIT protocol + baud prober + worker
│   ├── camera/                 V4L2 capture, ffmpeg segment recorder
│   ├── pose/                   features, neutral calibration, pose subprocess
│   └── dashboard/              HTTP API + single-page UI
├── scripts/                    inspect_hardware, fetch_pose_model, install, uninstall
├── systemd/                    oasis-daq.service
├── udev/                       99-oasis-daq.rules -> /dev/oasis_{gps,imu,camera}
└── tests/                      143 unit tests, no hardware required
```

## Tests

```bash
/home/esw/sensor-env/bin/python -m pytest tests/ -q      # 143 passed
```

The unit tests need no hardware. They cover NMEA checksums and coordinate/unit
conversion against canonical vectors **and** against sentences captured from the
attached receiver; WIT framing, checksums, scaling and resynchronisation against
bytes captured from the attached IMU; posture features and neutral calibration;
storage safety, writer durability and the timestamp audit.

Real-hardware results — including a genuine unplanned power event that exposed a
durability defect — are in [`1_Sensor_Acquisition/raspberry_pi/docs/TEST_RESULTS.md`](docs/TEST_RESULTS.md).

## Documentation

| | |
|---|---|
| [ARCHITECTURE](docs/ARCHITECTURE.md) | processes, threads, back-pressure, time, durability |
| [HARDWARE_BRINGUP](docs/HARDWARE_BRINGUP.md) | every measurement, with the commands that produced it |
| [DATA_SCHEMA](docs/DATA_SCHEMA.md) | every file and column |
| [OPERATIONS](docs/OPERATIONS.md) | daily use, config, troubleshooting, recovery |
| [SAFETY](docs/SAFETY.md) | vehicle, electrical, data-loss, privacy |
| [TEST_RESULTS](docs/TEST_RESULTS.md) | what was actually run, and what was not |
| [BLOCKERS](docs/BLOCKERS.md) | what could not be done, and the exact commands to finish it |

## Threading model

```
capture thread        read() -> trim -> newest-wins slots -> bounded queue   (no filesystem)
oasis-camera-writer   ffmpeg pipe + frame_timestamps.csv
oasis-segment-sync    fsync each finished segment
oasis-ffmpeg-stderr   drain ffmpeg's stderr (an undrained pipe deadlocks capture)
oasis-pose (process)  MediaPipe inference only
gps / imu threads     their own serial port and writers
oasis-health          health.csv and thresholds
oasis-dashboard       HTTP, read-only observer + START/STOP
```

The capture thread performs no filesystem work at all. Everything that can
block on exFAT lives on a thread that is allowed to.

## Known limits in v0.1

* **IMU is 10 Hz** at its factory 9600 baud — too slow for sharp transients.
* **GPS has never had a fix** on this bench (indoors).
* **MediaPipe has never seen a person** on this bench; the grayscale path is
  proven against a still image instead.
* **IMU mounting transform is identity** — the unit is not in a vehicle yet.
* The dashboard has **no authentication**.
* Hailo is not used; MediaPipe runs on CPU.

## Data policy

Recorded sessions never enter git. The repository holds source, config, tests,
unit files and documentation only.
