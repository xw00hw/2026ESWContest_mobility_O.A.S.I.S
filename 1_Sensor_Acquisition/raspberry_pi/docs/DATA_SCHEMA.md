# OASIS DAQ v0.1 — data schema

`schema_version: oasis-daq/0.1`

## Session layout

```
<storage_root>/sessions/YYYY-MM-DD/session_YYYYMMDD_HHMMSS[_<suffix>]/
├── camera/
│   ├── driver_000.mkv          60 s Matroska segments, MJPEG, stream-copied
│   ├── driver_001.mkv
│   ├── segments.csv            written by ffmpeg: name,start_s,end_s
│   └── frame_timestamps.csv    one row per CAPTURED frame
├── pose/
│   ├── landmarks.jsonl         one object per frame that produced landmarks
│   └── features.csv            one row per processed frame
├── imu/
│   ├── raw_000.bin             unmodified serial bytes, ~1 MB chunks
│   └── imu.csv                 one row per assembled sample
├── gps/
│   ├── raw_nmea.log            "<monotonic_ns> <wall_ns> <sentence>" per line
│   └── gps.csv                 one row per GGA/RMC
├── health/
│   └── health.csv              1 Hz system + worker health
├── events.jsonl                structured events (state changes, faults)
├── metadata.json               everything needed to interpret the session
├── manifest.json               file inventory with SHA-256
├── session_summary.json        durations, worker stats, timestamp audit
├── route.geojson               LineString of the driven track
├── route_map.html              self-contained map (OSM + offline fallback)
├── route_summary.json          distance, bbox, endpoints
└── INCOMPLETE                  present ONLY while the session is open or if it crashed
```

`INCOMPLETE` is the recovery marker: written when the session opens, removed
only on a clean close. **If it is present, the session was interrupted.**

## Common columns

Every row of every stream carries:

| column | type | meaning |
|---|---|---|
| `host_monotonic_ns` | int | `CLOCK_MONOTONIC_RAW` ns — **the alignment key** |
| `host_wall_time_ns` | int | UNIX epoch ns, for human correlation only |

The actual clock source is recorded in `metadata.json → clock.source`. Wall time
can step (NTP); monotonic cannot. **Always align on `host_monotonic_ns`.**

Nothing is resampled during acquisition. Each stream is written at its own rate;
alignment happens in post-processing against whichever timeline you choose
(usually camera frames).

## `gps/gps.csv`

| column | type | notes |
|---|---|---|
| `host_monotonic_ns`, `host_wall_time_ns` | int | |
| `gps_utc` | str\|null | ISO-8601 from RMC date+time, e.g. `1994-03-23T12:35:19.000Z` |
| `latitude_deg`, `longitude_deg` | float\|null | signed decimal degrees, WGS-84 |
| `altitude_m` | float\|null | GGA antenna altitude above the geoid |
| `ground_speed_mps` | float\|null | knots→m/s (×0.514444) from RMC, or VTG |
| `course_deg` | float\|null | true course. **Meaningless at rest or low speed** |
| `fix_quality` | int\|null | GGA field 6: 0 = no fix, 1 = GPS, 2 = DGPS, … |
| `fix_valid` | 0/1 | GGA quality ≥1, or RMC status `A` |
| `satellite_count` | int\|null | satellites *used* (GGA), not in view |
| `hdop`, `vdop`, `pdop` | float\|null | HDOP from GGA/GSA; VDOP/PDOP from GSA only |
| `source_sentence` | str | `GPGGA` or `GPRMC` — which sentence produced the row |
| `sequence_id` | int | monotonic counter within the session |
| `checksum_valid` | 0/1 | always 1 — failed sentences produce no row (counted in the summary) |
| `carried_fields` | str | `\|`-separated names of fields on this row that came from an **earlier** sentence, not the one named in `source_sentence` |

**Row cadence.** One row per GGA and one per RMC → **2 rows/s at a 1 Hz fix
rate**. Fields absent from the producing sentence are carried over from the
previous row (altitude and fix quality onto RMC rows; speed and course onto GGA
rows). `source_sentence` names the producing sentence and `carried_fields`
names exactly which values are **not** fresh.

This matters more than it looks. The attached receiver emits `RMC` **first** in
each epoch, so an RMC row's `altitude_m`, `fix_quality` and `satellite_count`
come from the *previous second's* GGA. At a fix transition the row is therefore
internally inconsistent — the first RMC after a fix is acquired reports
`fix_valid=1` next to `fix_quality=0`. Read `carried_fields` before trusting a
value, or work from the GGA rows only.

**Expected artefact:** the two rows of a burst arrive ~5 ms apart, then ~995 ms
of silence. The timestamp audit therefore reports `gaps` and a jitter p95 near
1000 ms for GPS. That is the receiver's 1 Hz burst behaviour, not a defect.

`raw_nmea.log` holds every received sentence, valid or not, prefixed with both
host timestamps — the parser can always be re-run against it.

## `imu/imu.csv`

Raw counts and engineering units are stored side by side, and the sensor frame
is kept separate from the vehicle frame. Nothing is overwritten in place.

| group | columns |
|---|---|
| identity | `host_monotonic_ns`, `host_wall_time_ns`, `sensor_sequence` |
| raw counts | `raw_acc_{x,y,z}`, `raw_gyro_{x,y,z}`, `raw_mag_{x,y,z}`, `raw_{roll,pitch,yaw}`, `raw_q{0,1,2,3}` |
| sensor frame, SI | `acc_{x,y,z}_mps2`, `gyro_{x,y,z}_radps`, `mag_{x,y,z}`, `{roll,pitch,yaw}_rad`, `quaternion_{w,x,y,z}` |
| vehicle frame | `veh_acc_{x,y,z}_mps2`, `veh_gyro_{x,y,z}_radps` |
| other | `temperature_c`, `packet_type`, `packets_in_sample`, `checksum_valid`, `sample_valid` |

**Scaling** (WIT standard protocol, full-scale from `configs/daq.yaml`):

```
acc   = raw/32768 * 16 g     * 9.80665   -> m/s²
gyro  = raw/32768 * 2000 °/s * π/180     -> rad/s
angle = raw/32768 * 180°     * π/180     -> rad
quat  = raw/32768
temp  = raw/100                          -> °C
mag   = raw counts (uncalibrated; the unit reports no scale factor)
```

**Vehicle frame is `+x forward, +y left, +z up`, right-handed.**
`veh_* = R · (sensor − bias)` where `R` is `imu.mounting.rotation_matrix`.
**In v0.1 `R` is the identity matrix** because the IMU is not yet mounted in a
vehicle, so `veh_*` currently equals the sensor-frame values. Both are stored so
the transform can be applied later without re-acquiring.

`sample_valid` requires acceleration *and* gyro present with a good checksum.
`packet_type` lists which WIT packets contributed, e.g. `acc+angle+gyro+mag`.

`raw_*.bin` is the unmodified byte stream. Concatenating the chunks in name
order reproduces exactly what arrived on the wire.

## `camera/frame_timestamps.csv`

One row per **captured** frame, including frames that were not written to disk.

| column | notes |
|---|---|
| `frame_id` | 1-based, counts every captured frame |
| `capture_monotonic_ns` | taken the instant `read()` returned — **use this, not the container's timestamps** |
| `host_wall_time_ns` | |
| `segment_id` | index of the `driver_NNN.mkv` this frame is in; `-1` if not recorded |
| `frame_index_in_segment` | position within that segment; `-1` if not recorded |
| `recorded` | 1 if written to disk, 0 if dropped by decimation |
| `capture_valid` | 1 if the buffer was a well-formed JPEG (SOI + EOI present) |
| `dropped_since_previous` | frames inferred missing from the capture interval |
| `jpeg_bytes` | size after padding was trimmed |

With `record_decimation: 4`, every 4th row has `recorded=1`. To map a recorded
frame back to the video: filter `recorded==1`, then `(segment_id,
frame_index_in_segment)` addresses it exactly.

**The container's timestamps are nominal.** ffmpeg is given a constant nominal
rate (`capture_fps / record_decimation`); real time lives only in this CSV.

`dropped_since_previous` is *inferred* (interval > 1.5× expected), because V4L2
does not report drops on this path. Treat it as an indicator, not a count.

**Known limitation — timestamps are dequeue times, not exposure times.**
`capture_monotonic_ns` is taken the moment `read()` returns. OpenCV does not
expose the V4L2 buffer timestamp, and the driver queues up to
`CAP_PROP_BUFFERSIZE` (4) frames. So if the capture thread is ever delayed, the
queued frames dequeue back-to-back and carry stamps up to the length of the
stall *later* than the sensor actually exposed them. The signature is
unmistakable in the data — a long interval followed by two or three
sub-millisecond ones:

```
gap 289.0 ms, then intervals (ms): 0.228, 0.095, 0.094, 18.793 ...
```

`dropped_since_previous` also attributes the whole gap to the first frame of
such a burst. Roughly 20 frames per session were affected before the writer
thread was introduced; it should be rarer now, but the property is inherent to
this capture API. **When sub-millisecond intervals appear, treat that burst's
timestamps as upper bounds.** Everything else — `segment_id`,
`frame_index_in_segment`, ordering, monotonicity — remains exact.

## `pose/features.csv`

One row per frame the pose process returned, including frames with no person.

| group | columns |
|---|---|
| identity | `host_monotonic_ns`, `host_wall_time_ns`, `frame_id`, `capture_monotonic_ns` |
| geometry | `shoulder_center_{x,y}`, `hip_center_{x,y}`, `head_center_{x,y}`, `shoulder_width`, `torso_angle_deg` |
| neutral-relative | `shoulder_shift_{x,y}`, `hip_shift_{x,y}`, `head_shift_{x,y}`, `shoulder_shift_px`, `head_shift_px` |
| derived | `lean_score`, `lean_velocity` |
| quality | `pose_confidence`, `landmark_valid_mask`, `landmarks_visible`, `pose_processing_latency_ms`, `neutral_valid`, `vision_valid` |

* Coordinates are **MediaPipe-normalised** (0–1 of the image). `*_px` variants
  are scaled by `pose.input_width/height`.
* `head_center` = mean of nose + both ears when visible, nose alone otherwise.
* `torso_angle_deg` = `atan2(shoulder_x − hip_x, hip_y − shoulder_y)` in
  degrees. **0 = upright**, positive = leaning toward +x in image space.
* `lean_score` = `shoulder_shift_x / neutral.shoulder_width` — normalising by
  shoulder width makes it comparable across drivers and camera distances.
* `lean_velocity` = d(`lean_score`)/dt using the host clock.
* `landmark_valid_mask` is one character per tracked landmark, MSB first, in
  ascending MediaPipe index order (`0,7,8,11,12,13,14,15,16,23,24`); `1` means
  visibility ≥ 0.5.
* `vision_valid` requires both shoulders and both hips at visibility ≥ 0.5.
* **All `*_shift_*` and `lean_*` columns are null until neutral calibration
  succeeds.** `neutral_valid` says whether it did; the reason is in
  `session_summary.json → workers.pose.neutral.reason`.

`landmarks.jsonl` — one object per frame **that produced landmarks** (so it is
empty when no one is in frame):

```json
{"frame_id": 1234, "capture_monotonic_ns": …, "host_wall_time_ns": …,
 "latency_ms": 72.4,
 "landmarks": [{"i":11,"n":"left_shoulder","x":0.54,"y":0.48,"z":-0.10,"v":1.0,"p":1.0}, …]}
```

`i` MediaPipe index, `n` name, `v` visibility, `p` presence.

## `health/health.csv`

1 Hz. `state`, `cpu_percent`, `cpu_temp_c`, `load_1m`, memory,
`disk_free_gb`, `disk_write_latency_ms` (a real 64 KiB write+fsync, sampled at
most every 30 s), plus per-worker liveness, rate, age, checksum errors, camera
FPS/recorded/drops, pose FPS/latency/valid-ratio, queue depths and cumulative
warning/error counts.

## `events.jsonl`

`{host_monotonic_ns, host_wall_time_ns, level, kind, source, …}` with
`level ∈ {info, warning, error}`. Kinds include `session_started`,
`state_changed`, `worker_started/stopped/failed`, `device_lost`,
`recorder_died`, `sensor_stale`, `disk_space_low/exhausted`,
`storage_root_changed`, `neutral_calibrated`, `neutral_calibration_failed`.

## `metadata.json`

OS/kernel/arch/Pi model, Python and package versions, git commit + dirty state,
clock source, storage root and mount (device, fstype, UUID, options), the full
device inventory with USB identities and actual baud/format, camera installation
(`driver_side_a_pillar_upper`), the vehicle-frame definition and IMU mounting
transform, and a verbatim copy of the config used.

## `session_summary.json`

Duration, final state, per-worker counters, storage state, event counts, route
statistics, and a **timestamp audit** per stream:

```json
"imu": {"count": 1535, "monotonic": true, "backward_steps": 0,
        "duplicates": 0, "gaps": 0, "max_gap_s": 0.6,
        "measured_hz": 10.05, "jitter_ms_p50": 0.81, "jitter_ms_p95": 3.22}
```

`duplicates > 0` means several samples share one host timestamp — that is how
the serial read-batching defect was caught. Check this before trusting an
analysis.

`gaps` counts intervals long enough to imply **at least one missed sample**
(threshold `median × 1.75`, reported as `gap_threshold_s`); `large_gaps` uses
`× 3`, and `estimated_missed_samples` totals how many samples the gaps account
for. The GPS entry audits the **GGA rows only** — one row per epoch — because
auditing both rows of a 1 Hz epoch reports the receiver as 2 Hz with ~900 ms of
jitter. `gps_all_rows` keeps the unfiltered view alongside it.

## What is never written to git

Session data never enters the repository. `.gitignore` excludes
`OASIS_DAQ/`, `sessions/`, `oasis-daq-fallback/`, video, images, `*.bin`,
`raw_nmea.log`, `landmarks.jsonl` and the per-stream CSVs. Recorded data stays
on the SSD and is uploaded separately by the operator.
