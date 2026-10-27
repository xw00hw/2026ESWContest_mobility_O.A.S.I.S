# Experiment A runtime integration

V7 was deployed on 2026-10-27 through the same desktop shortcut/bootstrap.
All 273 host tests (14.867s), `git diff --check`, and the installed `--check-ui`
passed. The UI check opens no UART/UDP acquisition workers. The release hash is
`dbed8c068b6e0cf0bdc6dfcdad82c89082556510c70359595f7e7a896bdfacd1`.
Settings were backed up as `trial_settings.before_v7_20261027_171011.json`;
all five hardware confirmations and firmware were retained. The application was
left closed after verification, so the next ordinary launch selects V7. No
physical V7 motor test or validated positive collision trial was performed.
The V6 verification record below is historical. V7 builds on V6 commit
`6bcf848` in `BeamNG_Bolster_FB`, preserving the V6 history and installed release.
Git integration does not automatically update another PC installation or board firmware.

## Input boundary

Only BeamNG collector v0.3 vehicle rows enter the vehicle-response/control calculation.
`vehicle.py` shares the existing pressure-independent adapter and adds optional
`specific_force_x/y/z_mps2`, `roll_rad`, `pitch_rad`, and `gear_index` fields. Vehicle coordinates
are +x forward, +y left, +z up; orientation fields describe the vehicle and are not
occupant measurements. Optional fields are preserved as `None` when missing and
are rejected if present but nonfinite. Selecting specific force requires its x/y
fields before ARM; there is no silent substitution between acceleration sources.
V6 also uses validated steering, forward gear, brake and vehicle orientation for
the bounded steering advance described below. Throttle is recorded but unused.

The existing collector bridge archives the original row first, then forwards a
copy on loopback UDP 64257. The collector retains ownership of UDP 64256. Source
receive timestamps, session/stream identifiers, configuration hash, derivative
validity, sequence continuity and simulation time are preserved. Runtime reads
the existing bridge envelope `oasis.beamng.fallback.v1`; the legacy name describes
the compatible transport envelope and does not enable the pressure fallback.

`runner.py` never passes pressure to the policy or computes a pressure baseline,
contact gate or pressure-based target. Optional `record_pressure=True` starts
acquisition only for evaluation logs; the desktop launcher enables this option.
Default CLI behavior still discards residual scanner bytes. There is no camera,
pose-estimator or RL input. Offline angle choices remain static configuration.

## V7: saturation and a separate impact-suspect latch

Experiment A no longer treats a single `abs(ay)>12` or `abs(ax)>15` sample as
a collision or automatic session end. Its adapter uses a broad integrity bound
of 200 m/s² per linear x/y component; nonfinite values, invalid derivatives,
clocks, sequence changes and ordinary freshness checks remain faults. The
shared pressure-fallback validator's defaults are unchanged. Qualified large
ay enters the normal response filter and saturates the requested Yaw at the
effective STM limit (initially 15°), with Pitch/CAD coupling, 2°/s initial
rates, PWM slew and neutral passage unchanged. Saturation is a target bound,
not a guarantee of instantaneous physical motion or unlimited applied load.

`impact.py` observes every original qualified row before policy downsampling.
It uses gravity-free linear ax/ay, independently of the selected response
acceleration source. Let `a=hypot(ax,ay)` and `j=hypot(delta_ax,delta_ay)/dt`.
Default settings are:

| Setting | Value |
|---|---:|
| `impact_planar_threshold_mps2` | 25m/s² |
| `impact_jerk_threshold_mps3` | 150m/s³ |
| `impact_min_duration_s` | 0.06s |
| `impact_min_delta_v_mps` | 1.5m/s |
| `impact_extreme_planar_mps2` | 60m/s² |

A sustained suspicion requires **all**: a continuous segment with `a>=25`,
`j>=150` observed within that segment, duration >=0.06s, and the magnitude of
the local-axis vector acceleration integral >=1.5m/s. Duration/integration use
original simulation time. Integration is trapezoidal only between two high
endpoints; it never backdates a new peak over a preceding low sample. A low,
unqualified or discontinuous sample clears the segment. Repeated held samples
earn no duration. Local vehicle axes rotate, so this short vector integral is
an acceleration-derived proxy, not independent measured world-frame delta-v.
Separately, `a>=60` immediately requests motor OFF. The detector latches until
the session ends and does not use pressure, body measurements or motor position.

These are provisional shock heuristics, not collision ground truth. The user's
confirmed non-collision V6 session `A_20261027_165300_984bb9af` peaked at ay
12.87136, ax +7.25147, a 14.77348m/s², then fell to a 6.93867m/s² after 13.5ms.
The earlier 14:41 a=32.93081m/s² peak is unlabelled and also isolated. Neither
record is a validated positive collision case; no detection-rate claim follows.

On suspicion the runner logs the raw trigger and `ImpactState` diagnostics,
latches out motor command/ARM authority, and enters `IMPACT_OFF_PENDING`.
After a matching OFF acknowledgement and a fresh OFF STATE within 300ms it
enters `RECORD_ONLY`. The same vehicle and pressure recording continues until
manual Stop or another acquisition/device/logging fault. There is no motor
re-ARM in this session. Failure to confirm OFF ends the full run; a stale prior
OFF STATE cannot satisfy confirmation. Manual Stop ends the workers and then
the user explicitly saves or discards the pending record.

After the impact latch, missing/invalid vehicle samples remain diagnostic data
and do not restore motor authority or automatically end the record-only phase.
Board transport, OFF verification and recording failures still end the run.
The ordinary vehicle freshness gate remains mandatory whenever control is active.

Offline verification currently includes 17 focused tests covering saturation,
isolated peaks, sustained front/side shocks, extreme input, stopped input,
duplicate/gap handling, coherent-vector accumulation and replay latching. The
939-row user-confirmed non-collision record and 1,527-row unlabelled earlier
record both replayed without an impact event. These tests opened no hardware;
synthetic positive cases do not establish real collision detection performance.

## V6 bounded steering advance

`steering_preview.py` adds a small cue before primary lateral-acceleration
support. Its maximum requested Yaw is 2° and requested Pitch is always 0°.
It never sums with the acceleration target. The existing initial primary limits
of Yaw 15°, Pitch ±6°, 2°/s, CAD coupling, PWM slew, and neutral passage remain in
force. A 2° target is not a guarantee of immediate 2° shaft motion.

The declared profile is stock `vehicles/vivace/vivace_160_DCT.pc` (Cherrier
Vivace 160 DCT, 1.6L turbo gasoline/FWD/7DCT), SHA256
`4eb3d9c52cb02b1a8885779090215f6b631671a3fee03296a0d32303d367ce85`.
`metadata.declared_vehicle_profile.identity_source` explicitly reads
`operator_selected_profile_not_live_vehicle_verification`, and
`steering_gain_calibrated=false`. No V5 steering gain or inferred road-wheel
angle is transferred; this profile is a comparison candidate, not a validated
dynamic equivalent of a real vehicle or a live vehicle-identity interlock.

Default configuration:

| Setting | Value |
|---|---:|
| `steering_preview_enabled` | true |
| `steering_to_ay_sign` | +1 |
| `steering_preview_max_yaw_deg` | 2° |
| `steering_preview_deadband_deg` / `steering_preview_reference_deg` | 5° / 45° |
| `steering_preview_max_abs_deg` | 560° |
| `steering_preview_min_speed_mps` / `steering_preview_full_speed_mps` | 5 / 15m/s |
| `steering_preview_dwell_s` / `steering_preview_timeout_s` | 0.1 / 1.5s |
| `steering_preview_speed_lookahead_s` | 0.25s |

Eligibility requires finite steering within the bound, forward integer gear >0,
brake in [0,1], absolute vehicle roll/pitch <20°, valid gravity-free longitudinal
acceleration, and actual speed >=5m/s. Missing qualifiers disable this small cue;
they do not manufacture data or bypass the ordinary input validator. A stopped
vehicle or steering inside the 5° deadband resets the episode. Otherwise only
the dwell may restart: dropped qualifiers or direction flips never renew the
1.5s episode. Both clocks use intervals between same-PC source receipt monotonic
timestamps, exactly once per new source sample, rather than simulation-time
slow motion or repeated policy ticks.

Let `clip01(x)=max(0,min(1,x))`. The cue uses:

```text
v_preview = clip(max(0, v + linear_ax * 0.25), 0.8*v, 1.1*v)
speed_fraction = clip01((v_preview**2 - 5**2) / (15**2 - 5**2))
steering_fraction = clip01((abs(steering_deg) - 5) / (45 - 5))
brake_factor = 1 - 0.5*brake
yaw = min(2, effective_max_yaw) * speed_fraction * steering_fraction * brake_factor
pitch = 0
```

Positive steering times `steering_to_ay_sign` requests right support. Existing
primary support, raw or filtered absolute ay >= the 0.03g entry threshold, or
opposite-direction ay reaching the 0.02g release threshold blocks the cue. Brake
dampens only this cue and never attenuates the primary actual-acceleration
response. Vehicle orientation is an eligibility bound, not occupant posture.

`diag.target_source` is `actual_acceleration` or `steering_advance`.
`diag.steering_preview` logs `active`, `reason`, `side`, `yaw_deg`, `pitch_deg`,
`steering_fraction`, `speed_fraction`, `projected_speed_mps`, `brake_factor`,
`dwell_elapsed_s`, `episode_age_s`, and `throttle_used=false`. These explain
decisions; they are not proof of road-wheel angle, future ay, actual contact
pressure, or human motion prediction. The declared profile and settings are
preserved in each selected recording.

## Startup and authority

1. Settings validation and explicit hardware confirmations run **before** any
   UART or UDP resource opens. The desktop app enumerates ST-LINK ports without
   opening unrelated devices; it never flashes firmware or invents confirmations.
2. `CONTROL_HELLO` must match the existing fallback motor firmware contract,
   including geometry/calibration, 270° servo mapping and 2 µs/20 ms slew.
   The A v2 identity, CAD profile hash and STM committed tuning revision must
   match; the effective STM values are frozen for this session and rechecked at ARM.
3. `FB_OFF → DISABLE → STATE` verifies PWM OFF and disables the separate scanner.
   With observation enabled, this becomes `FB_OFF → DISABLE → HELLO(FSR) →
   START_12,15 → STATE`. Pressure PING is scheduled every second through the same
   single pending-request slot, after higher-priority motor/state work.
4. The launcher requires at least two continuous seconds of valid fresh vehicle
   data and verified OFF board telemetry. A bad sample resets the full interval.
5. One explicit session-ID ARM is sent. A fresh armed STATE must report neutral
   commanded pose/target and no ongoing move before model outputs or leases begin.

Launching the commissioned desktop app authorizes this one automatic start;
no repeated ARM button is required. Its pre-ARM BeamNG wait is unlimited. The
supervisor waits for the original collector socket/archive to open, then waits
for USB and starts the controller. Either worker ending stops the other;
the V7 impact latch keeps both workers alive with motor output OFF. No
automatic restart or re-ARM follows an error. Stop is checked both at loop entry
and again after receiving rows, before any ARM/control action. The validation
clock is refreshed after each receive so concurrent fresh rows are not mistaken
for future timestamps.

`BoardState` is firmware command/transport evidence, **not measured shaft position
or human posture**. Its use for device health, independent angle limits and
neutral-transition checks does not add an occupant-feedback input to Experiment A.

At most one UART request is outstanding, with exact response matching and a
200 ms ACK timeout. Requests are not queued for delayed motion. The same loop
that validates vehicle input computes targets and renews motor authority: motor
PING every 100 ms, STATE approximately every 100 ms, changing targets no faster
than every 40 ms, policy ticks no faster than every 20 ms. Firmware independently
retains its 500 ms motor lease and fixed PWM slew limit. Faster configured body
response does not increase that firmware actuator bandwidth.

## Invalid data and shutdown

During control, every received vehicle row is checked, including multiple rows in the same poll.
A subsequent good row cannot hide an earlier bad derivative, stream/session
change, timestamp regression, stale row or sequence error after ARM. Input or
board faults cause immediate OFF initiation and terminate the run; the V7
impact-suspect path above is a distinct record-only transition. There is no
automatic reconnect or re-ARM. Before ARM, invalid input restarts qualification.

Control freshness uses host monotonic time. Simulation time is separately
validated and the model owns its simulation integration. Repeated policy ticks
must not integrate a retained vehicle sample twice. Replay must preserve these
clocks and never disguise recorded data as a fresh live source.

Shutdown sends `FB_OFF`, disables acquisition, requests STATE and waits up to
300 ms for fresh OFF evidence. The manifest reports whether OFF telemetry was
actually observed; an old OFF STATE cannot prove shutdown. PWM OFF is not a
claim that an unpowered/loaded mechanism releases contact or holds its position.

## Records and verification scope

For V6 and later desktop runs, acquisition writes to a pending directory outside the
permanent output root: `install_root/pending_sessions/<version>/<session_id>`.
`STOPPED` or `ERROR` does not mean saved. Only an explicit Save after **all**
workers finish promotes a pending directory to `output_root/<version>/<session_id>`.
Explicit Discard removes only that selected owned pending tree. Close and errors
do neither; the next application start discovers recoverable pending sessions and
requires the operator to resolve them before starting or reconnecting. A crash
releases the OS lease so its unfinished record can be recovered without opening
hardware. Active records cannot be recovered, moved, or discarded.

`DesktopSession.snapshot()` exposes `retention`, `recording_id`, `staging_path`,
`persisted_path`, `intended_output`, and the active `output` path. The retention
states are `pending`, `saved`, and `discarded` (`none` before acquisition).
`save_recording()` and `discard_recording()` serialize against each other and
reject live workers. Same-volume save uses directory rename; cross-volume save
copies to a private destination staging directory, verifies file hashes, then
renames it into place. Copy or promotion failure leaves the original pending
record available. Existing destinations are never overwritten. Every recursive
move/delete validates the owned root, identifier, ownership marker, resolved
path, and absence of symlinks/junctions/reparse children.

Raw CSV/JSONL and existing capture manifests remain byte-identical after save.
`recording.json` is the retention authority; `recording_retention_at_capture`
in `experiment.json` describes its original capture state. Absolute paths in
historical records are resolved with `recording_relocation.json` and
`resolve_recording_path(saved_root, historical_reference)` rather than rewriting
their content. The relocated reference must stay inside the current recording.

Each new run directory receives `manifest.json` and `control.jsonl` with config,
local module hashes, source context, decisions and board command state. Existing
session files are never overwritten. The collector independently retains its
original vehicle archive. With observation enabled, `pressure.jsonl`,
`pressure_raw.csv` and `pressure_sweeps.csv` preserve all raw ADC/status samples,
complete sweeps, independent MCU/host clocks and quality events. Pressure is
not calibrated to force. Bounded asynchronous logging performs writes/fsync in
its own worker; queue overflow or disk failure ends motor authority and retains
`INCOMPLETE`. Video is not captured. The enclosing desktop session adds
`experiment.json` with collector path and comparison labels; see [DESKTOP.md](DESKTOP.md).
The offending vehicle frame is logged before an armed-session input exception
is raised; impact-trigger samples and diagnostics are also retained if the
operator chooses Save. Invalid input remains distinct from valid large
acceleration and is not clamped into a usable motion request. A high value or
heuristic suspicion is not proof of a collision.

Historical V6 was deployed through the same desktop bootstrap/shortcut on 2026-10-27.
All 238 host tests and the installed `--check-ui` check passed; the latter opens
no UART. Release identity is recorded in [README.md](README.md). Physical
steering-advance/contact-pressure verification remains outstanding. No firmware
change was made for this host release. Earlier V1~V5 records are never
automatically migrated or removed.

Transport/launcher tests inject a fake UART, loopback receiver and monotonic clock.
The fixture policy tests orchestration independently of the biomechanical model;
additional end-to-end tests use the actual response calculation and command limiter. They cover
no-pressure startup and identical output with absent/corrupt residual pressure
bytes, strict ACKs, session/clock errors, startup qualification, neutral ARM
verification and fresh shutdown evidence. Current target tests respect the CAD
envelope and pass commanded neutral before changing support side. Historical
25° reversal tests describe the superseded V1 limits, not the current envelope.
The fake board acknowledges position immediately;
these tests are not measurements of real servos, actual pressure or torso motion.
