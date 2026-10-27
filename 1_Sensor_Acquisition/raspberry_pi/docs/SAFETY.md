# OASIS DAQ v0.1 — safety

## Vehicle safety

**The DAQ is a passive recorder.** It reads sensors and writes files. It has no
actuator, no vehicle-bus connection, and no control authority of any kind.

* **Never operate the dashboard while driving.** START/STOP before moving, or
  have a passenger do it. The dashboard is a browser page — treat it exactly
  like a phone.
* **The camera must not obstruct the driver's view.** It is mounted at the upper
  driver-side A-pillar, aimed inward at the driver. Verify the field of view and
  local regulations before driving.
* **Secure every device and cable.** An unsecured Pi, SSD or camera is a
  projectile in a collision. Route cables clear of pedals, the steering column,
  and airbag deployment paths.
* **Do not debug while driving.** SSH in, start the session, then leave it alone.

## Electrical safety

The Pi is currently on a **5 V / 3 A (15 W) PD supply**. Raspberry Pi 5 expects
**5 V / 5 A**; at 15 W the firmware limits **total USB current to 600 mA**, and
four USB devices are attached (GPS, IMU, camera, SSD).

* A Samsung T7 can draw more than 600 mA on its own during sustained writes.
* An unexplained SSD disconnect plus a reboot occurred during the first
  endurance run — consistent with exceeding the USB power budget.
* **Check `vcgencmd get_throttled` before and after every session.** `0x0` is
  healthy; bit 0 means under-voltage now, bit 16 means it happened since boot.
* Do not hot-plug USB devices during a session.
* For vehicle use, a supply that browns out during cranking will corrupt
  in-flight data. Use a supply rated for the load, and prefer powering the Pi
  from a circuit that is not shared with the starter.

Under-voltage is the single most likely cause of data loss in this setup — more
likely than any software fault.

## Data safety — what this software will not do

By construction, the DAQ:

* **never formats, partitions or repartitions** any device;
* **never deletes or overwrites** existing files — a session directory is always
  new, and if a name somehow exists the next free name is used instead;
* **never resumes into a previous session directory** after a crash;
* **never writes recorded data into git** (`.gitignore` excludes video, raw
  streams and per-stream CSVs);
* **never modifies `/etc/fstab`**, and never mounts or unmounts the SSD;
* **never changes the GPS or IMU configuration** — baud probing is read-only;
* **never touches the Hailo driver or runtime.**

`scripts/install.sh` backs up any udev rule or unit file it replaces.

## Data loss: what is actually at risk

`flush()` only reaches the kernel page cache. This was not theoretical — an
abrupt SSD disconnect during the first endurance run destroyed **~10 minutes** of
acquisition: every CSV had zero data rows and the video file was zero bytes.

Mitigations now in place:

| stream | worst-case loss on a power cut |
|---|---|
| CSV / JSONL | `storage.fsync_interval_s` (default **15 s**) |
| IMU raw bytes | 15 s, or one chunk boundary |
| GPS raw NMEA | 15 s |
| video, completed segments | none — fsynced as ffmpeg finishes each one |
| video, in-progress segment | up to `camera.segment_seconds` (default **60 s**) |

Verified by `SIGKILL`: all streams held data to within 1–5 s of the kill, and
every segment including the in-progress one stayed playable. **A true
power-interruption test has not been run** — see `BLOCKERS.md` §6.

Always check for the `INCOMPLETE` marker before treating a session as complete.

## Disk exhaustion

* A session **refuses to start** below `storage.min_free_gb_start` (20 GB).
* A running session **closes cleanly and goes to ERROR** below
  `storage.min_free_gb_stop` (5 GB) — files are finalised first, then the state
  changes. It does not keep writing until the filesystem fails.
* At the default 25 fps recording rate, expect **≈3.9 GB/h**. With 363 GB free
  that is roughly 90 hours. `record_decimation: 1` raises it to ≈16 GB/h.
* The dashboard shows free space and estimated recording time remaining.

## Privacy

**This system records video of a person's face and body, continuously, together
with precise location and time.** In most jurisdictions that is personal data,
and in-vehicle camera recording of a driver may be separately regulated.

* Obtain **informed consent** from anyone recorded, before recording.
* Recorded sessions must **not** be committed to the repository or shared
  casually. `.gitignore` blocks the obvious paths, but it cannot stop a
  deliberate `git add -f`.
* `route.geojson` and `route_map.html` contain a precise travel history. Treat
  them as sensitive.
* Decide retention and deletion policy before collecting at scale.
* If a session is to be shared for analysis, prefer sharing `pose/features.csv`
  and the sensor CSVs **without** `camera/` — the posture features are derived
  and far less identifying than the video.

## Credentials

No credential — sudo password, GitHub token, or any other secret — is written to
any file, log, commit message or session artefact. `.gitignore` additionally
excludes `.env`, `*.token`, `*credentials*`, `*.pem` and `*.key`.

The dashboard has **no authentication**. Bind it only to a trusted network; do
not port-forward it.

## Before the first drive

- [ ] `oasis-daqctl check --probe` reports 0 problems
- [ ] `vcgencmd get_throttled` returns `0x0`
- [ ] GPS has achieved a real fix outdoors (`fix_valid` true)
- [ ] MediaPipe produces landmarks with the actual driver seated
- [ ] Neutral posture calibration succeeds (`neutral_valid` = 1)
- [ ] IMU mounting rotation measured and written into `configs/daq.yaml`
- [ ] SSD mounted, ≥20 GB free, and `status` shows the SSD (not the fallback)
- [ ] All hardware physically secured; camera does not obstruct the view
- [ ] Consent obtained from everyone who will be recorded
- [ ] A test session recorded, stopped, and verified with `summarize`
