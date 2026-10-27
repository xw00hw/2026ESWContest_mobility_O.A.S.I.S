# OASIS DAQ v0.1 — blockers

Everything that could not be completed, why, and the exact command to finish it.
Written 2026-08-28. Nothing below is a code defect; each item needs a credential,
a physical action, or an environment this bench does not have.

---

## BLOCKER 1 — GitHub authentication (RESOLVED 2026-08-28)

**Status:** resolved. The branch is pushed and this section is kept only as a
record of what the delay cost.

For most of this work the Pi had **no GitHub credential of any kind** — no
`gh auth`, no `GH_TOKEN`, no `~/.ssh`, no `~/.git-credentials` — and `git` and
`gh` were not even installed (both were installed as part of this work:
`git 2.43.0`, `gh 2.45.0`). Network to github.com was fine throughout
(`HTTP 200`); only the credential was missing, and `xw00hw/ESW` is private.

**Consequence while it lasted.** The repository could not be read, so
`AGENTS.md`, `CONTRIBUTING.md`, `TASKS.md` and `docs/*.md` were unavailable and
the work was built in a standalone local repository against the task brief
alone.

**How it was resolved.** The operator ran `gh auth login` (account `xw00hw`,
scopes `gist, read:org, repo, workflow`). The repository conventions were then
read *before* reconciling, and this work was replanted onto `origin/main` as
`feat/oasis-daq`, which is why the branch name dropped the `-v0.1` suffix —
`CONTRIBUTING.md` specifies `feat/<short-topic>`.

**What reading the conventions changed:**

* `docs/DECISIONS.md` gained ADR-010…013 — AGENTS.md requires that a mismatch
  between plan and reality be recorded rather than hidden.
* `docs/HARDWARE.md` gained a measured-hardware section: the BOM lists a
  NEO-M8N/M10 and a Hailo AI HAT+, but the attached GNSS is a **u-blox 7** and
  **no Hailo device is present**; the camera was not in the BOM at all.
* `docs/INTERFACES.md` gained §3b mapping every DAQ column onto the existing
  §3 pose-feature contract, with units and frames as AGENTS.md requires. This
  surfaced a real mismatch: §3 specifies `torso_angle_rad`, the DAQ stores
  `torso_angle_deg`.
* `TASKS.md`: `PER-001` and `DATA-001` were already-open tasks this work
  advances; `DAQ-001…004` were added for what remains.
* ADR-009 (BATON IMU axes) turned out to be the same lesson as ADR-012 —
  the team had already learned that guessing IMU axes silently reverses the
  bolsters.

## BLOCKER 2 — GPS has never achieved a position fix (indoor bench)

**Status:** environmental, not a defect.

> **Update 2026-10-24:** resolved in the real-vehicle drive — 1,315 valid fixes in the main
> session and a real `route.geojson`. See `2_Data_Processing/`.

The u-blox 7 streams valid NMEA continuously with **zero checksum failures**,
reports `ANTSTATUS=OK`, and tracks satellites (`GPGSV` shows 3–4 with SNR 22–23).
It has never reported `fix_valid`, because the receiver has no sky view:

```
$GPGGA,,,,,,0,00,99.99,,,,,,*48      <- fix quality 0, 0 satellites used
$GPRMC,,V,,,,,,,,,,N*53              <- status V (void)
```

Consequently these are **untested against real data**, though all are exercised
by unit tests using canonical NMEA vectors:

* `latitude_deg` / `longitude_deg` / `altitude_m` from live hardware
* `ground_speed_mps`, `course_deg` from live hardware
* `route.geojson`, `route_map.html`, `route_summary.json` with a real track
  (they are generated correctly, but with `point_count: 0`)
* the dashboard map's live track rendering

**To finish:** move the receiver where it can see sky (a window is usually
enough; outdoors is reliable), wait 1–15 minutes for a cold-start fix, then:

```bash
oasis-daqctl check --probe --probe-seconds 60     # expect fix=yes
oasis-daqctl start --notes "gps fix validation"
# drive or walk for a few minutes
oasis-daqctl stop
# then confirm the artefacts have real coordinates:
python3 -c "import json;d=json.load(open('<session>/route_summary.json'));print(d)"
xdg-open <session>/route_map.html
```

---

## BLOCKER 3 — MediaPipe has never seen a person (no subject at the bench)

**Status:** environmental, not a defect.

> **Update 2026-10-24:** partly resolved in the real-vehicle drive — a seated person was detected
> (`vision_valid` 99.9%), but live neutral calibration never locked (`neutral_valid` = 0 for the
> whole session); the neutral pose was computed offline. See `2_Data_Processing/`.

The pose pipeline runs end to end and is proven to produce landmarks, but only
against a still image: MediaPipe's own sample photo yields all 33 landmarks
through the **exact grayscale→RGB path the OV9281 uses**, with results
essentially identical to the colour path (normalised coordinates differ by
<0.01, visibility >0.96). See `TEST_RESULTS.md`.

Against the live camera — which points at an empty bench — `frames_with_pose`
is `0`, so these paths have **not** run on real hardware data:

* `pose/landmarks.jsonl` (written only when landmarks exist; currently 0 bytes)
* neutral-posture calibration accepting a window (it correctly reports
  `only 0 valid frames in window, need 30`)
* `lean_score`, `lean_velocity`, `*_shift_*`, `torso_angle_deg` on live data
* the dashboard skeleton overlay

**To finish:** sit a person in the driver's seat, in frame, and:

```bash
oasis-daqctl start --notes "pose validation with subject"
# hold a neutral posture for the first ~20 s so calibration can lock,
# then lean left/right and forward/back
oasis-daqctl stop
python3 -c "
import csv;r=list(csv.DictReader(open('<session>/pose/features.csv')))
print('rows',len(r),'valid',sum(int(x['vision_valid']) for x in r))
print('neutral', r[-1]['neutral_valid'], 'lean', r[-1]['lean_score'])"
```

Expect `vision_valid` ≈ 1 for most rows and a non-empty `landmarks.jsonl`.

---

## BLOCKER 4 — IMU is limited to 10 Hz at its factory settings

**Status:** a real limitation of the current configuration, deliberately not
changed in v0.1.

Measured on the attached unit: **9600 baud, 10 Hz** for each of the four WIT
packet types (`acc`, `gyro`, `angle`, `mag`), 0 checksum failures. Four 11-byte
frames per sample at 9600 baud is ~440 B/s, and the line saturates near 21 Hz —
so 10 Hz cannot be meaningfully raised **without changing the receiver's baud
rate**, which v0.1 does not do (the brief was to record the confirmed current
configuration).

10 Hz is adequate for posture and gross vehicle dynamics but is **too slow for
sharp transients** (pothole strikes, hard braking onset, impact detection).

**To finish (only if higher rate is required):** this needs a WIT
configuration write, which must be done deliberately and verified:

1. Record the current configuration first.
2. Send the WIT unlock + baud-rate + rate commands (see
   `sensor-drivers/WitStandardProtocol_JY901`, the WITMOTION official
   repository already present on this machine).
3. Re-probe: `oasis-daqctl check --probe --probe-imu-baud` must select the new
   baud with `frame_ratio > 0.99`.
4. Update `imu.baud` and `imu.expected_hz` in `configs/daq.yaml`.
5. Save the before/after configuration into the session metadata.

`configs/daq.yaml` already carries `baud_candidates` and the probe is read-only,
so a wrong guess is recoverable by power-cycling the unit.

---

## BLOCKER 5 — the external SSD is mounted by udisks2, not by fstab

**Status:** known operational hazard, mitigated in software, not fixed at the
system level (fixing it means editing `/etc/fstab`, which was out of scope).

The Samsung T7 is mounted at `/media/esw/T7` by **udisks2, after a desktop
login**. `oasis-daq.service` starts at boot, well before that. This was observed
for real: after an unplanned reboot the daemon came up with the SSD absent and
selected the local fallback root.

**Mitigation already in place:** the supervisor re-resolves the storage root on
every `start_session()` (`_refresh_storage_root`), so pressing START after the
SSD appears puts the session on the SSD and logs a `storage_root_changed` event.
`oasis-daqctl status` always shows the root in use, and the dashboard shows it
too.

**Residual risk:** with `session.autostart_recording: true`, a session started at
boot would land on the microSD fallback. Autostart is `false` by default for
exactly this reason.

**To fix properly** (requires a considered `/etc/fstab` change):

```bash
sudo cp /etc/fstab /etc/fstab.bak.$(date +%Y%m%d)
# UUID confirmed on this machine: 8E1B-119F  (exFAT, label T7)
echo 'UUID=8E1B-119F /media/esw/T7 exfat defaults,nofail,uid=1000,gid=1000,x-systemd.device-timeout=10 0 0' \
  | sudo tee -a /etc/fstab
sudo mount -a          # MUST succeed before rebooting
findmnt /media/esw/T7  # verify
# then uncomment RequiresMountsFor= in systemd/oasis-daq.service
```

`nofail` and the device timeout are mandatory: without them a missing SSD blocks
boot.

---

## BLOCKER 6 — true power-loss durability is only partly verified

**Status:** partially verified; the remaining test needs someone to physically
cut power.

An **unplanned** SSD disconnect during the first endurance run destroyed ~10
minutes of acquisition (every CSV had 0 data rows, the video file was 0 bytes),
because `flush()` only reaches the kernel page cache. That was a genuine defect
and is fixed: writers now `fsync` on an interval (`storage.fsync_interval_s`,
default 15 s), completed video segments are fsynced as ffmpeg finishes them, and
segments were shortened to 60 s.

What has been verified since: **SIGKILL** of the daemon, and a **32-minute
endurance run that closed cleanly with zero errors** — see
`TEST_RESULTS.md`. What has **not** been verified: pulling the Pi's
power mid-session, which is the only way to prove the page cache is genuinely
bypassed.

**To finish:**

```bash
oasis-daqctl start --notes "power loss test"
sleep 180                      # let 2-3 segments complete
# physically remove power from the Pi (do not shut down)
# power back on, then:
oasis-daqctl summarize /media/esw/T7/OASIS_DAQ/sessions/<date>/<session>
```

Expect: `INCOMPLETE` present, all CSVs holding data up to ~15 s before the cut,
and every *completed* `driver_*.mkv` playable. The in-progress segment is
expected to be lost or truncated — that is the documented worst case.

---

## BLOCKER 7 — no vehicle, so the mounting transform is identity

**Status:** cannot be resolved on a bench.

`imu.mounting.rotation_matrix` in `configs/daq.yaml` is the identity matrix, and
is labelled as such (`"identity -- sensor axes not yet measured against vehicle
axes"`). Until the IMU is physically fixed in the car, the `veh_*` columns equal
the sensor-frame columns. Both are stored separately in every row, so nothing is
lost — the transform can be applied in post-processing once measured.

The camera position is recorded as `driver_side_a_pillar_upper` from the brief;
it has not been physically verified either.

**To finish:** mount the IMU in its final orientation, then with the vehicle
level and stationary read `imu.csv` and solve for the rotation that maps the
measured gravity vector onto vehicle `-z`, and forward acceleration onto `+x`.
Write it into `configs/daq.yaml` and record the date in
`configs/imu_calibration.yaml`.

---

---

## BLOCKER 8 — the 5 V / 3 A supply is under-specified for this device set

**Status:** hardware, needs a different power supply before vehicle use.

Raspberry Pi 5 expects **5 V / 5 A**. On the attached **5 V / 3 A (15 W) PD
supply** the firmware caps **total USB current at 600 mA**, and four USB devices
are attached: GPS, IMU, camera and the Samsung T7 — which alone can exceed
600 mA during sustained writes.

Two unexplained USB dropouts occurred during one night of testing:

| time | event | consequence |
|---|---|---|
| ~23:44 | SSD disconnected, machine rebooted | ~10 min of acquisition lost (this exposed the missing `fsync`) |
| 00:17:50 | camera disconnected, re-enumerated 1 s later | session correctly closed to ERROR; all 11 segments stayed playable |

`vcgencmd get_throttled` read `0x0` throughout — but it reports the Pi's **input
rail**, not the USB port budget, so it does not rule this out.

**Neither dropout was caused by software**, and both were handled correctly by
the DAQ. But a dropout mid-drive costs the in-flight data, and in a vehicle the
supply is also exposed to cranking transients.

**To resolve, in order of preference:**

1. Use a **5 V / 5 A (25 W) USB-C PD supply** — the official Pi 5 supply or
   equivalent. This is the real fix.
2. Failing that, move the SSD to a **self-powered USB hub** so it draws from the
   hub rather than the Pi.
3. Only after (1), consider `usb_max_current_enable=1` in
   `/boot/firmware/config.txt` to lift the 600 mA cap. **Do not set this on a
   15 W supply** — it removes the guard without adding the current.

Check before and after every session:

```bash
vcgencmd get_throttled       # 0x0 is healthy
                             # bit 0  = under-voltage now
                             # bit 16 = under-voltage has occurred since boot
```

The SSD currently sits on a **USB 2.0** port (480 Mbps). That is ample for the
4.4 MB/s the DAQ produces, and re-plugging it is itself a dropout risk, so it
was deliberately left alone.

---

## Not blockers, but unverified

* **GPIO START/STOP button** — implemented as a config-gated module and left
  `enabled: false`; no pin has been confirmed on real wiring, and the brief
  forbids enabling it against an unverified pin.
* **Hailo AI HAT+** — `hailortcli` is **not installed** on this machine and no
  Hailo device appears in `lspci`. Nothing Hailo-related was installed, removed
  or reconfigured. MediaPipe runs on CPU via XNNPACK, which meets the v0.1
  target, so Hailo is not a dependency.
* **Sensor unplug/replug during a session** — this happened **for real** at
  00:17:50 when the camera dropped off the USB bus, and the system behaved as
  designed: `device_lost` after 50 failed reads, session closed cleanly to
  ERROR, every artefact written, all 11 video segments playable, and GPS, IMU
  and pose continued with **zero** errors while only the camera worker failed.
  udev recreated `/dev/oasis_camera` when the device re-enumerated. A
  *deliberate* unplug of the GPS or IMU has still not been performed.
