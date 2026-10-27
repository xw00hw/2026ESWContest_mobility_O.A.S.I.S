# OASIS DAQ v0.1 — test results

Everything below was actually executed on the target hardware on 2026-08-27/28.
Nothing is projected. Where a test could not be run, it says so and points at
`BLOCKERS.md`.

Host: Raspberry Pi 5 / Ubuntu 24.04.4 / kernel 6.8.0-1047-raspi / aarch64,
Python 3.11.16 in `/home/esw/sensor-env`.

---

## 1. Unit tests — 143 passed, no hardware required

```
$ /home/esw/sensor-env/bin/python -m pytest 1_Sensor_Acquisition/raspberry_pi/tests/ -q
143 passed
```

| file | covers |
|---|---|
| `test_nmea.py` | checksums (valid, corrupted payload, wrong suffix, missing suffix), lat/lon conversion incl. S/W hemispheres and out-of-range minutes, knots→m/s, RMC field ordering, GGA→RMC HDOP propagation, GSA DOP, VTG speed override, real no-fix sentences, garbage never raising |
| `test_wit.py` | framing against **bytes captured from the attached IMU**, checksum arithmetic, corrupted checksum and corrupted payload, resync after garbage, byte-at-a-time delivery, `0x55` inside a payload, scaling constants, 1 g at rest, raw counts preserved, mounting rotation, gyro bias, sample validity |
| `test_features.py` | centres, torso angle sign, missing/low-visibility landmarks, shift and lean, scale invariance, lean velocity from the host clock, neutral calibration accept/reject/too-few-frames |
| `test_storage_session.py` | session directories never reused, fallback selection, CSV/JSONL writers, double close, raw chunk rollover and pre-rollover flush, timestamp audit (perfect stream, backward steps, duplicates, gaps), GeoJSON lon/lat order, haversine distance, manifest hashing, atomic JSON |
| `test_devices.py` | by-id preferred over kernel name, wrong VID/PID rejected, **empty matcher refuses to guess**, **ambiguous match is an error**, serial number disambiguation, camera card matching, metadata node not selected |
| `test_supervisor_policy.py` | READY/fallback bring-up, start refused below the free-space threshold, fallback refusal, **storage root re-resolved at session start**, INCOMPLETE lifecycle, double-start refused, stop without a session, sessions never share a directory, metadata contents |

Two defects were found by these tests and fixed:
`lean_velocity` was disabled by a truthiness check on a timestamp that can
legitimately be `0`; and `RawChunkWriter`'s rollover semantics were documented
and its pre-rollover flush was added.

---

## 2. Device identification

```
$ oasis-daqctl check --probe --probe-seconds 4 --probe-imu-baud
== storage ==
  [OK] storage root: /media/esw/T7/OASIS_DAQ (363.1 GB free)
  [OK] free space: 363.1 GB free, 20.0 GB required
== devices ==
  [OK] gps: /dev/serial/by-id/usb-u-blox_AG_..._GPS_GNSS_Receiver-if00 (1546:01a7)
  [OK] imu: /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 (1a86:7523)
  [OK] camera: /dev/video0 (Arducam OV9281 USB Camera: Ardu)
== pose model ==
  [OK] pose model: ~/.local/share/oasis-daq/models/pose_landmarker_lite.task
== system ==
  [OK] memory: 6090 MB available of 7933 MB
  [OK] cpu temperature: 41.9 C
  [OK] throttling: vcgencmd get_throttled=0x0
== live probes ==
  [OK] camera stream: 90.3 fps, 362/362 valid JPEG, mean 36894 B
  [OK] gps stream: 42 sentences, 0 checksum errors, fix=no,
       types=['GPGGA','GPGLL','GPGSA','GPGSV','GPRMC','GPTXT','GPVTG']
  [WARN] gps fix: no position fix (normal indoors; needs sky view)
  [OK] imu baud scan: selected 9600 baud

0 problem(s), 1 warning(s)
```

Device enumeration correctly excluded `/dev/video1` (the UVC metadata node) and
all `pispbe`/`rpivid` platform nodes.

### udev stable naming

```
$ sudo bash 1_Sensor_Acquisition/raspberry_pi/scripts/install.sh && ls -l /dev/oasis_*
/dev/oasis_camera -> video0
/dev/oasis_gps    -> ttyACM0
/dev/oasis_imu    -> ttyUSB0
```

**Proven across a real replug.** When the camera dropped off the bus and
re-enumerated as a new USB device (§8), udev recreated `/dev/oasis_camera`
pointing at the new node automatically — the exact failure that hardcoding
`/dev/video0` would have caused.

---

## 3. Sensor validation

### GPS — u-blox 7

Startup banner identified the receiver: `HW UBX-G70xx`, `PROTVER 14.00`,
`ANTSTATUS=OK`. Over every run combined, **0 checksum failures**. Sentence mix
`GGA/RMC/VTG/GSA/GSV/GLL/TXT` at 1 Hz → 2 rows/s.

**No position fix** — indoors. `GPGGA` quality `0`, `GPRMC` status `V`, 3–4
satellites tracked at SNR 22–23. See `BLOCKERS.md` §2.

Parser verified against canonical vectors: `4807.038,N` → `48.117300`,
`01131.000,E` → `11.516667`, `022.4` knots → `11.5235 m/s`, VTG `005.5` knots →
`2.8294 m/s`, RMC `230394` + `123519` → `1994-03-23T12:35:19.000Z`. A
single-character corruption of a valid sentence is rejected.

### IMU — WIT protocol, baud auto-detection

| baud | bytes | valid frames | bad checksum | discarded | frame ratio | verdict |
|---|---|---|---|---|---|---|
| **9600** | 1220 | **110** | 0 | 0 | **0.992** | **selected** |
| 115200 | 3072 | 0 | 0 | 3062 | 0.000 | rejected |
| 57600 | 3072 | 0 | 0 | 3062 | 0.000 | rejected |
| 38400 | 2304 | 0 | 0 | 2294 | 0.000 | rejected |
| 19200 | 1670 | 0 | 0 | 1660 | 0.000 | rejected |

Every wrong baud produced bytes and zero valid frames — which is exactly why the
probe scores frame validity rather than "bytes arrived".

Decoded from a real frame: `|a| = 0.9971 g` at rest, gyro ≈ 0, temperature
25.95 °C. Packet mix `acc/gyro/angle/mag`, **10 Hz each**, **0 checksum
failures** across every run (6504 samples in one 10.8-minute session).

### Camera — Arducam OV9281

`YUYV` caps at 10 fps and is unusable. `MJPG 1280x800` measured at **100.2 fps**
sustained (1002 frames in 10 s via ffmpeg passthrough) and ~101 fps inside the
running DAQ. Requesting 30 fps yields 100 — the device offers only 100/120.

Recording pipeline verified independently before integration: 1169 captured →
292 written at decimation 4 → 3 segments containing exactly 125+125+42 = 292
frames, all playable.

### MediaPipe — grayscale path proven

The camera is monochrome, so the pose input path is grayscale→3-channel. Both
paths were run on MediaPipe's own sample photo:

| | landmarks | left shoulder | shoulder width | torso angle |
|---|---|---|---|---|
| colour (reference) | 33 | (0.5408, 0.4820) v=1.000 | 0.0860 | +1.48° |
| **grayscale→RGB (OV9281 path)** | **33** | (0.5378, 0.4811) v=1.000 | 0.0799 | +1.19° |

Normalised coordinates differ by <0.01 and visibility stays >0.96 — the
monochrome path is viable.

Standalone throughput on this Pi 5 (`pose_landmarker_lite`):

| input | sustained | p50 | p95 |
|---|---|---|---|
| 640x400 | 13.3 fps | 66 ms | 94 ms |
| 256x256 | 12.4 fps | 69 ms | 101 ms |

**No live landmarks** — nobody sits in front of the bench camera, so
`frames_with_pose = 0` in every session. See `BLOCKERS.md` §3.

---

## 4. Integrated sessions

Three sensors plus pose, recording to the SSD via the systemd daemon.

| | 83 s run | 10.8 min run (real device loss) |
|---|---|---|
| camera | 8027 frames @ 98.9 Hz, 2006 recorded, 0 invalid | 64612 frames @ 99.9 Hz, 0 invalid |
| imu | 809 samples @ 9.96 Hz, 0 checksum errors | 6504 samples @ 10.05 Hz, 0 errors |
| gps | 164 rows @ 2.02 Hz, 0 checksum errors | 1292 rows @ 2.00 Hz, 0 errors |
| pose | 768 frames @ 9.46 Hz, p95 110 ms | 6343 frames @ 9.82 Hz |
| errors | 0 | 1 (the camera unplug) |

### Timestamp audit

All streams `monotonic = true`, no backward steps, in every session.

```
gps     n= 1292   1.998 Hz  dup=0  gaps=0
imu     n= 6504  10.047 Hz  dup=4  gaps=1
camera  n=64612  99.919 Hz  dup=0  gaps=11
pose    n= 6343   9.817 Hz  dup=0  gaps=10
```

**GPS reports gaps and ~1000 ms jitter p95 in shorter runs by design** — GGA and
RMC arrive ~5 ms apart once per second. That is the receiver's burst behaviour.

### Defect found and fixed: batched serial timestamps

The first integrated run reported **669 duplicate timestamps in 809 IMU
samples**. Cause: `serial.read(256)` with a 1 s timeout blocks until the buffer
fills, so a whole second of frames received one identical stamp. The workers now
read only what has arrived (`in_waiting`, 10 ms poll).

| | before | after |
|---|---|---|
| IMU duplicate stamps | 669 / 809 | **0** |
| IMU jitter p95 | 84.77 ms | **3.22 ms** |

This was caught by the timestamp audit that ships in every `session_summary.json`.

---

## 5. Durability

### The failure that started it

During the first endurance attempt the USB SSD disconnected and the machine
rebooted (physical event — see §8). The interrupted session was correctly marked
`INCOMPLETE`, and **all of its data was gone**: every CSV had 0 data rows and
`driver_000.mkv` was 0 bytes, despite `frames_recorded=505` being observed live
22 s in. `flush()` only reaches the kernel page cache; `dmesg` showed
`Synchronize Cache(10) failed` and the volume remounting read-only.

Fixes: periodic `fsync` on every writer (`storage.fsync_interval_s`, 15 s),
fsync of completed video segments, segment length reduced 120 s → 60 s, and the
storage root re-resolved at every session start.

### SIGKILL verification (154 s session, killed with no warning)

| file | recovered | last row |
|---|---|---|
| `gps/gps.csv` | 297 rows | 5 s before the kill |
| `imu/imu.csv` | 1535 rows | 1 s before the kill |
| `camera/frame_timestamps.csv` | 14970 rows | 4 s before |
| `pose/features.csv` | 1474 rows | 2 s before |
| `health/health.csv` | 149 rows | 5 s before |
| `imu/raw_000.bin` | 67497 bytes | — |
| `gps/raw_nmea.log` | 792 lines | — |

All three Matroska segments playable, **including the in-progress one** —
`ffprobe` counted 1500 + 1500 + 821 frames. `INCOMPLETE` present. systemd
restarted the daemon automatically (`NRestarts=1`, service `active`).

**A true power-interruption test has not been run** — see `BLOCKERS.md` §6.

### Defect found and fixed: fsync stalled capture

The segment fsync was initially done on the capture thread. Over a 10-minute
run it produced **exactly 10 stalls of 583–743 ms, each ~1 s after a 60 s
segment boundary** — fsyncing a finished 56 MB file on exFAT/USB2 blocks that
long, losing ~70 frames per rollover (1.15 % of frames).

```
t_since_start   gap_ms    nearest segment boundary
        60.63    583.3    1.15 s away
       120.75    653.6    1.27 s away
       ...       ...      ...
       601.21    700.4    0.90 s away
```

Interval distribution before the fix: 99.880 % of frames < 15 ms, 0.102 %
15–25 ms, **0.016 % > 105 ms** — capture was essentially perfect apart from
these periodic stalls.

Moving the fsync to its own thread helped but did not finish the job: the
32-minute endurance run still showed **9 stalls of 114–297 ms**, because the
blocking `stdin.write()` to ffmpeg and the timestamp-CSV fsync were both still
on the capture thread. A writer thread now owns both, fed through the bounded
queue `camera_write_maxsize` already described but nothing used.

Measured across the three states, over 3 segment rollovers each:

| capture path does... | stalls > 105 ms | max interval | frames dropped |
|---|---|---|---|
| fsync inline (original) | 1 per 60 s segment | 742.7 ms | 1.15 % |
| fsync on its own thread | ~1 per 217 s | 296.9 ms | 0.20 % |
| **nothing but enqueue** | **0** | **22.9 ms** | **0.039 %** |

In the final state 99.961 % of intervals are under 15 ms, the writer queue never
exceeded depth 0, and `camera_write_drops` stayed at 0.

---

## 5b. Raw archives are replayable

The point of storing raw bytes alongside the parsed CSV is that a parser change
can be re-run against the original stream. That was verified against the
SIGKILL session, not assumed.

**IMU** — the 67 497-byte raw archive was re-parsed from scratch:

```
frames: ok=6136  bad_checksum=0  bytes_discarded=0     (100.000% valid)
samples reconstructed from raw : 1534
samples in imu.csv (written live): 1535
compared 1534 samples x 6 raw fields -> 0 mismatches
```

**GPS** — `raw_nmea.log` was replayed through the assembler:

```
792 sentences, 0 checksum failures
rows replayed from raw : 299
rows in gps.csv        : 297
```

In both cases the raw archive is **at least as complete as** the parsed CSV —
the raw writer runs before parsing, so it is flushed with data the CSV had not
yet reached. The one- and two-row differences are the expected boundary effect
of killing the process without warning. Nothing recorded is unrecoverable from
the raw stream.

---

## 5c. Endurance — 32 min 31 s, zero errors

Run through the systemd daemon, recording to the SSD, with the dashboard polled
throughout.

| | |
|---|---|
| duration / final state | **32:31**, `COMPLETE`, `INCOMPLETE` removed |
| **errors** | **0** across every worker |
| camera | 196 556 frames @ 100.77 Hz; **49 139 recorded — exactly 196 556 / 4** |
| IMU | 19 592 samples @ 10.04 Hz; **0 bad checksums in 78 368 frames** |
| GPS | 3 894 rows @ 2.00 Hz; 4 checksum failures in ~15 000 sentences (0.03 %) |
| pose | 19 262 frames @ 9.88 Hz; latency p50 45.1 ms / p95 59.6 ms |
| storage | 33 segments, 1 600 MB → **2.95 GB/h** |
| timestamps | every stream `monotonic = true` |

Resource trend over 1 940 one-second health samples:

| metric | min | mean | max |
|---|---|---|---|
| CPU | 15.0 % | 27.3 % | 75.2 % |
| SoC temperature | 44.1 °C | 48.6 °C | **53.5 °C** |
| memory used | 21.1 % | 23.9 % | 25.2 % |
| camera capture | 82.2 fps | 100.9 fps | 101.5 fps |
| pose | — | 9.84 fps | 9.88 fps |
| IMU | — | 10.05 Hz | 10.05 Hz |
| SSD write latency | 4.7 ms | 18.9 ms | 159.9 ms |

`vcgencmd get_throttled = 0x0` before and after — no under-voltage, no thermal
throttling in 32 minutes at ~100 fps capture plus continuous pose inference.

### What this run exposed

**Capture stalls, before the writer thread.** This session predates that fix, so
it still shows the blocking pipe write: **9 stalls of 114–297 ms in 1 949 s**,
no longer aligned to segment boundaries but on a ~120 s cycle.
`sum(dropped_since_previous)` was 403 of 196 556 frames — **0.20 %**, already
down from 1.15 % before the fsync-thread fix. 99.883 % of intervals were under
15 ms.

**A phantom loss counter.** The summary reported `imu: dropped=11400`. Nothing
was lost: `imu.csv` holds **19 592 rows for 19 592 samples**. `GpsWorker` and
`ImuWorker` pushed every row into a `DropCountingQueue` that **nothing ever
drained** — the CSV write is synchronous and the dashboard reads a
newest-wins slot, so the queue had no consumer. It filled at its 8 192 bound
and then rejected everything, inflating `dropped` without losing a byte. An
operator reading the summary would have concluded a 32-minute session lost
11 400 IMU samples. The vestigial queues are removed; `health.csv` now reports
the one queue that genuinely exists, the camera writer's.

---

## 5d. Verification run — every fix confirmed on hardware

A 7 min 24 s session on the fully fixed code, ended by `systemctl restart`
*during recording* so the graceful-SIGTERM path was exercised too.

| | |
|---|---|
| final state | `COMPLETE`, `clean_close: true`, `workers_not_joined: []` |
| **errors / dropped** | **0 / 0** on every worker |
| camera | 44 659 frames @ 100.59 Hz, 11 164 recorded (exactly /4), `write_queue_drops: 0` |
| **capture stalls > 105 ms** | **0** — max interval **22.9 ms**, 99.973 % under 15 ms |
| IMU | 4 454 samples @ 10.03 Hz, **0 bad checksums in 17 816 frames** |
| GPS | 885 rows @ 1.99 Hz, **0 checksum failures** |
| pose | 4 368 @ 9.84 Hz, latency p50 41.8 ms / p95 52.9 ms |
| `events.jsonl` | **0 strict-JSON rejects** (was 1 per session) |
| warnings | 1 — `neutral_calibration_failed`, correct with no subject present |

The timestamp audit now reads the receiver correctly:

```
gps           443 rows   1.000 Hz  mono=True dup=0 gaps=0  jitter p50 0.674 ms p95 16.5 ms
gps_all_rows  885 rows   2.000 Hz  bimodal=true, gap statistics suppressed
imu          4454 rows  10.050 Hz  mono=True dup=1 gaps=1
camera      44659 rows 100.918 Hz  mono=True dup=0 gaps=25
pose         4368 rows   9.885 Hz  mono=True dup=0 gaps=1
```

### Graceful restart during recording

```
01:03:17  Stopping oasis-daq.service...
01:03:17  state: RECORDING -> SAVING (daemon shutdown)
01:03:18  state: SAVING -> READY (session saved)
01:03:18  daemon exited cleanly
01:03:19  Started oasis-daq.service
01:03:24  state: INIT -> READY
```

1.4 s from SIGTERM to a finalised session. `INCOMPLETE` removed, all six
artefacts written, and the new daemon back to READY 7 s later.

### Two further defects this run exposed

**`clean_close` never reached the file.** The flag was set on the summary dict
*after* `write_json` had already serialised it, so `session_summary.json` never
contained it — the logic was right and the evidence was missing. Now decided
before the write, and a test asserts the field is present in the file.

**Bimodal series got nonsense gap statistics.** `gps_all_rows` reported
`gaps: 103, estimated_missed_samples: 5745` on a perfectly healthy 105-second
recording, because GGA and RMC arrive 0.3 ms apart and then wait a second. A
series whose p90/p10 interval ratio exceeds 10 is now flagged `bimodal` and its
gap statistics are suppressed with a note, rather than published as fact.

---

## 6. Dashboard

Tested over the LAN from another host (`192.168.35.32:8080`) while a session was
recording.

| endpoint | result |
|---|---|
| `GET /` | 200, 16 363 B, 5.8 ms |
| `GET /api/status` | 200, 1.7 ms, live values for all four workers |
| `GET /api/health` | 200 — CPU 48.4 %, 46.3 °C, disk write latency 7.15 ms |
| `GET /api/preview.jpg` | 200, 8 048 B, decodes to 640x400 |
| `GET /api/stream.mjpg` | 23 frames in 3 s ≈ 7.7 fps (configured 8) |
| `GET /api/latest`, `/api/route.geojson` | 200 |
| `GET /api/nonexistent` | 404 |

### Disconnect does not stop recording

| event | state | elapsed | frames recorded |
|---|---|---|---|
| MJPEG stream killed mid-transfer | RECORDING | 00:00:42 | 1033 |
| port 8080 dropped with iptables for 20 s | RECORDING | 00:01:02 | 1532 |

499 frames in 20 s = exactly the 25 fps recorded rate, 0 errors. The dashboard
page itself shows a banner stating that recording is unaffected when it cannot
reach the Pi.

---

## 7. systemd

* **Boot start verified for real.** After an unplanned reboot, the journal shows
  `-- Boot ... --` immediately followed by `Started oasis-daq.service` and
  `state: INIT -> READY`, with no operator action.
* **`Restart=on-failure` verified** by SIGKILL: service returned to `active`
  with `NRestarts=1`.
* **Graceful SIGTERM verified**: `systemctl restart` during a session closes it
  cleanly — `RECORDING -> SAVING -> READY`, `INCOMPLETE` removed, all artefacts
  written.
* A unit-file warning (`StartLimitIntervalSec` in `[Service]`) was found in the
  journal and fixed by moving it to `[Unit]`.
* **A crashed daemon never resumes into an old session** — every start
  allocates a new directory.

---

## 8. Real hardware failures encountered (unplanned, and useful)

Two genuine hardware events occurred during testing. Both are consistent with
the **5 V / 3 A supply**, which caps total USB current at 600 mA across four
USB devices (GPS, IMU, camera, SSD). `vcgencmd get_throttled` stayed `0x0`
throughout — it reports the Pi's input rail, not a USB port budget.

### SSD disconnect + reboot

`usb 2-2: USB disconnect` → `Synchronize Cache(10) failed` →
`exFAT-fs (sda1): Volume was not properly unmounted`, and the machine rebooted.
The SSD re-enumerated on a different bus. **Pre-existing user data on the SSD
was intact.** This exposed the durability defect in §5.

It also exposed a second defect: the daemon starts at boot but udisks2 mounts
the SSD only after login, so the daemon selected the local fallback root and
never re-checked. Fixed by re-resolving storage at session start
(`BLOCKERS.md` §5).

### Camera disconnect during the endurance run

At 00:17:50, `usb 2-1: USB disconnect` and
`uvcvideo: Failed to resubmit video URB (-19)`; the camera re-enumerated one
second later as a new USB device.

**This was an unplanned sensor-unplug test, and the system behaved as designed:**

| requirement | observed |
|---|---|
| detect device loss | 50 consecutive failed reads → `device_lost` event |
| close the session cleanly | `RECORDING → SAVING → ERROR`, reason recorded |
| write all artefacts | summary, manifest, metadata, route, events all present |
| keep `INCOMPLETE` | present (error close) |
| **not take other sensors down** | camera `errors=1`; **gps, imu, pose all `errors=0`** |
| preserve data | 64612 camera / 6504 imu / 1292 gps / 6343 pose samples |
| video integrity | **all 11 segments playable, 0 unreadable** |
| stable naming survives replug | `/dev/oasis_camera` recreated by udev |

---

## 9. Resource use

Sampled at 1 Hz from `health.csv` over a 10-minute session (611 samples):

| metric | min | mean | max |
|---|---|---|---|
| CPU | 13.8 % | 27.6 % | 61.4 % |
| SoC temperature | 42.5 °C | 46.0 °C | 49.1 °C |
| memory used | 20.0 % | 22.3 % | 23.4 % |
| camera capture | — | 100.13 fps | 101.28 fps |
| pose | — | 9.67 fps | 9.80 fps |
| IMU | — | 10.01 Hz | 10.05 Hz |
| pose latency p50 | 50.2 ms | 50.7 ms | 53.8 ms |

`vcgencmd get_throttled = 0x0` throughout — no under-voltage, no thermal
throttling. Storage consumption **≈3.25 GB/h** at 25 fps recorded.

**Process separation confirmed.** Measured live: the MediaPipe child process
used ~60 % of one core while the main acquisition process used ~11 % and held
101 fps. Vision load does not reach the recording path.

---

## 9b. Independent review

An independent reviewer read the whole package against six named risk areas,
cross-checking the parser and timestamp claims against the live session's own
CSVs rather than against the docstrings. It found defects in five of the six.

**Confirmed sound, with no defect found:**

* All IMU scale factors and unit conversions, all NMEA field indices and
  coordinate/speed conversions, and the `v_veh = R · v_sens` convention.
* IMU host timestamping after the read-batching fix — measured on a live
  session: 7961 rows, **0 backward steps, 5 duplicate stamps (0.06 %), median
  Δt 99.61 ms**.
* Storage safety: nothing formats, partitions, mounts, unmounts, truncates or
  deletes recorded data; the only `unlink` calls are the two probe files and
  the `INCOMPLETE` marker. `SessionPaths.allocate` cannot reuse a directory
  even across a backwards clock step.
* Segment bookkeeping: ffmpeg's PTS segmentation gives exactly 1500 frames per
  60 s segment, matching `_frames_per_segment`.
* Pose/preview isolation from capture: `LatestSlot` holds its lock for four
  assignments, MediaPipe is a spawned process, the preview releases its lock
  before decoding, and the recording path never decodes.
* **Dashboard disconnect (risk area 6): no path found from the dashboard into
  the acquisition path.** A client that disconnects mid-`stop_session` does not
  abort it; only the final response write fails, harmlessly.

**Defects found, all fixed** — see the table in §10, entries 14–26. The four
graded *high* were:

| defect | why it mattered |
|---|---|
| ffmpeg's stderr pipe was never drained | 64 KiB of error output would block ffmpeg, then block `stdin.write()` on the capture thread **forever** — recording stops with no symptom but a stale-sensor warning |
| `shutdown()` was a no-op during `SAVING` | a `systemctl stop` arriving while a stop was already running killed the daemon thread mid-save: no summary, no manifest, unflushed writers, **and exit code 0 so systemd did not restart** |
| the `INCOMPLETE` marker was removed regardless | a worker that failed to join left writers unclosed and the final segment unfinalised, yet the session was indistinguishable from a clean one |
| the udev path skipped the ambiguity guard | with two CH340 adapters — the exact case the udev rules warn about — the wrong port was opened as the IMU silently, **and only when the rules were installed** |

The GPS timestamp-audit fix is worth showing, because it changed what the data
appeared to say about the receiver:

| | before (all rows) | after (GGA epoch rows) |
|---|---|---|
| measured rate | 2.0 Hz | **1.0 Hz** (`rate_error 0.0 %`) |
| jitter p50 | 964 ms | **0.69 ms** |
| jitter p95 | 982 ms | **16.0 ms** |
| gap threshold | 2.78 s — a three-fix outage went uncounted | 1.75 s, and a 3 s outage is reported with `estimated_missed_samples: 3` |

The receiver was always this good; the audit was measuring the wrong series.

---

## 10. Defects found and fixed during testing

Every one of these was found by running the system or by reading its own
output, not by inspection. Listed in the order they were found.

| # | defect | how it surfaced | severity |
|---|---|---|---|
| 1 | `BaseWorker._stop` shadowed `threading.Thread._stop`, so `join()` raised `TypeError: 'Event' object is not callable` | first integrated run crashed on shutdown | critical — no session could close cleanly |
| 2 | Raw NMEA and raw IMU logs were never flushed until close | the crashed session left both at 0 bytes | high |
| 3 | Serial reads batched a whole second of frames under one timestamp | timestamp audit reported 669 duplicates in 809 IMU samples | high — makes fine alignment impossible |
| 4 | `flush()` without `fsync` is not durable on removable exFAT | a real SSD disconnect destroyed ~10 minutes of acquisition | critical |
| 5 | Storage root resolved once at boot, never re-checked | after a reboot the daemon sat on the microSD fallback while the SSD was mounted | high |
| 6 | Segment `fsync` ran on the capture thread | 10 stalls of 583–743 ms, one per 60 s segment, losing 1.15 % of frames | high |
| 7 | `lean_velocity` disabled by a truthiness check on a timestamp of `0` | unit test | medium |
| 8 | `sensor_stale` fired before a worker's first sample | every session logged a spurious GPS warning | low |
| 9 | `events.jsonl` contained a bare `Infinity` | strict JSON parsers reject the line; verified on a real file | medium — breaks downstream tooling |
| 10 | `git status --porcelain` parsed with `.strip()`, shifting paths by one character | `metadata.json` recorded `cquisition/...` | medium — silently wrong provenance |
| 11 | `route_summary()` omitted `distance_km`/`start`/`end` when the track was empty | `summarize` printed `None km` | low |
| 12 | The udev symlink was followed without checking it against the configured matcher | a degraded-mode test resolved a device it should not have | high — device misidentification |
| 13 | RMC `course_deg` read from field 8 (the date) instead of field 7 | caught while writing the parser tests | high |
| 14 | ffmpeg's stderr pipe never drained → capture-thread deadlock | independent review | **high** |
| 15 | `shutdown()` a no-op while a stop was already running | independent review | **high** |
| 16 | `INCOMPLETE` removed even when a worker did not join or artefacts failed | independent review | **high** |
| 17 | udev path bypassed the ambiguity guard | independent review | **high** |
| 18 | a matcher with only `device:` matched every port | independent review | medium |
| 19 | RMC rows silently carried the previous epoch's altitude/fix_quality | independent review | medium |
| 20 | GPS timestamp audit measured the wrong series; gap threshold 2.8 s | independent review | medium |
| 21 | blocking pipe write and CSV fsync still on the capture thread | independent review | medium |
| 22 | mounting rotation accepted unvalidated (`det = −1` mirrors silently) | independent review | medium |
| 23 | IMU parse guard outside the frame loop → one bad frame aborts every chunk | independent review | medium |
| 24 | unguarded `fsync` on chunk rollover could kill the IMU worker | independent review | medium |
| 25 | `is_dir()` used where the comment promised a mountpoint check | independent review | low |
| 26 | `lean_velocity` differentiated against inference-result time | independent review | low |
| 27 | GPS/IMU rows pushed into a queue nothing drained → 11 400 phantom "dropped" samples | 32-minute endurance run | medium — a false data-loss signal is worse than none |
| 28 | `clean_close` set after the summary was serialised, so it never reached the file | verification run | medium — the fix worked but left no evidence |
| 29 | bimodal series reported 103 gaps and 5745 missed samples on healthy data | verification run | medium — another false data-loss signal |

Defects 1–6 and 12–13 would each have produced silently wrong or missing data
in a real drive.

---

## 11. What was NOT tested

| | why | where |
|---|---|---|
| GPS with a real fix | no sky view indoors | `BLOCKERS.md` §2 |
| MediaPipe on a real person | no subject at the bench | `BLOCKERS.md` §3 |
| Neutral-posture calibration succeeding | needs a person | `BLOCKERS.md` §3 |
| True power-cut durability | needs power physically removed | `BLOCKERS.md` §6 |
| Disk-threshold stop on a real full disk | 362 GB free; covered by unit test instead | — |
| Deliberate sensor unplug | happened for real instead (§8) | — |
| GPIO button | pin not confirmed; module left disabled | `BLOCKERS.md` |
| Hailo | not installed, not a dependency | `BLOCKERS.md` |
| Push / PR to GitHub | no credential on this machine | `BLOCKERS.md` §1 |
