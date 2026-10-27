# Pressure-only bench profile — 2026-10-25

`bench_20261025.json` contains measured ADC distributions and commanded motor states for initial deterministic fallback development. It does **not** establish an angle-to-force calibration, a monotonic pressure-control gain, or a safe human-contact limit.

## Source and exclusions

- Drive ZIP: `bench_20261025_pose_v1.zip`, file ID `1HTNr1ZExkWfbuYSdfRaxxZawIA2wcV_h`, 742,536,978 bytes.
- Profile session: `20261025_013054_fdbdd0b4`, independent left/right yaw 0–27°, both pitches 0°.
- Earlier comparison: `20261025_005459_3512cd6e`, left yaw 0–20° / pitch ±8°, right at 0°.
- Additional investigation: `20261025_003530_9c4e477c`, equal left/right yaw 0–20° / pitch ±8°.
- Each session visits each full four-angle command once. The 357 + 357 + 784 points do not form repeated randomized trials or a full four-dimensional calibration.

Only selected ZIP members were retrieved by HTTP byte ranges. Per-entry ZIP CRC checks and local SHA256 provenance are retained. No video, image, frame table, landmark, inferred body pose, or merged pose/pressure table was downloaded or used. `points_source.parquet` is the original motor experiment result table; the builder explicitly selects motor/pressure/timing columns and excludes its video-count columns. Source experiment metadata is used for configuration, clocks, and provenance only.

Raw extracted files remain outside Git at the separately supplied dataset directory. Each profile source file's path, byte count, and SHA256 is recorded in `provenance`.

## Measurement meaning

Pressure channels are `A0_C0` … `A2_C3`: ADC bank and MUX input identifiers. The supplied METHODS document explicitly says physical sensor positions and force/pressure calibration are absent. Do not infer left/right, upper/lower, or contact area from these identifiers.

Values use `raw × 3.3 / 4095`. Vref is nominal, not measured. Units are estimated volts, not N, kg, kPa, or a calibrated seat load. The baseline at commanded (0,0,0,0) is the session's loaded/contact reference, not an unloaded ADC zero.

`p05_v`, `p50_v`, and `p95_v` are sample quantiles of valid raw ADC readings within each exact two-second dwell, using NumPy's linear quantile definition. They are not confidence intervals, safe pressure thresholds, or quantiles of a group sum. A sum of channel p95 values is not a measured p95 of that sum.

## Hardware configuration must match

The source identifies DS5180SSG 270° servos, C4_R8 inverse kinematics, crank zero 1200 µs, and these zero-command PWM values:

| Pin | Joint | Zero-command PWM | Direction |
|---|---|---:|---:|
| D3 | Right crank 1 | 1800 µs | −1 |
| D4 | Left crank 1 | 1200 µs | +1 |
| D5 | Right crank 2 | 1200 µs | +1 |
| D6 | Left crank 2 | 1800 µs | −1 |

This differs from the earlier SMART_SEAT assumption of 180° travel with four 1500 µs assembly centers. The profile must not silently transfer between those configurations. Actual shaft/plate position was not measured; the logged “arrived” and “moving” fields describe controller command state.

The independent yaw experiment reports PWM ranges D3/D6 = 1296–1800 µs and D4/D5 = 1200–1704 µs. Observed maximum yaw 27° is a dataset boundary, not an independently verified clearance or human-contact limit.

## Verified profile subset

The JSON has left-only and right-only branches, each with yaw 0–27° at 1° spacing; their shared zero point gives 55 unique commanded points. Opposite yaw and both pitches remain zero.

- 132,660 valid ADC readings: 55 points × 12 channels × 201 readings.
- Every point has exactly 0.5 seconds between reported arrival and dwell start, followed by a two-second half-open dwell interval.
- Ten motor state reports per point inside the dwell all have armed=1, moving=0, fault=NONE, matching current/target commanded angles, and matching expected/reported PWM.
- No ADC errors or saturated readings occur in this selected subset. Recomputed means match supplied `point_pressure.csv` to less than 1e−10 V.
- `motion_check` and per-channel sample/quality fields retain this evidence. They confirm telemetry consistency, not measured physical position.

## Pressure response is not reliably monotonic

For the selected one-side branches, the observed mean change from yaw 0° to 27° is:

| ADC channel | Left-only yaw change | Right-only yaw change |
|---|---:|---:|
| A1_C0 | +0.032 V | −0.093 V |
| A1_C1 | +0.156 V | −0.021 V |
| A1_C2 | −0.443 V | −0.101 V |
| A1_C3 | −0.202 V | −0.295 V |

These are electrical channel observations, not assertions about which side of the seat they measure. The profile contains weighted increasing and decreasing PAVA fits plus their residual error. Neither fit establishes actuator polarity or a causal pressure gain. For example, the left-only A1_C2 increasing fit is flat. A1_C3 has no point p95 above its neutral p95 in either branch.

Consequently, an integral controller that assumes increasing yaw must increase a mapped pressure signal can run into its output limit. Use the profile as an empirical candidate lookup, with no extrapolation. Until mapped response direction is confirmed, pressure feedback should limit or inhibit further commands when fresh-baseline increments exceed the configured cap; it should not keep increasing yaw to chase an assumed pressure target.

## Session variation and unmeasured hysteresis

At 21 identical full commands shared by the first and third sessions, A0_C3 absolute mean differences have median 0.542 V and maximum 0.999 V. A1_C0 has median absolute difference 0.322 V. For the left-only comparison against the third session, those median absolute differences are 0.544 V and 0.402 V, respectively.

Typical within-dwell standard deviations on active channels are around 0.03–0.04 V. The much larger between-session differences cannot be treated as measurement noise alone. Occupant/contact changes, preload, drift, scan order, and hysteresis were not independently controlled or identified. Each command is visited once per session, so this dataset cannot separate these effects.

Re-establish the current neutral/contact baseline before using pressure increments. Do not use the old session's absolute voltage as a new occupant's pressure target. A2_C0 and A2_C3 were mostly near zero; their low signal does not prove that they are broken or unoccupied without the physical sensor mapping.

## Commissioning candidates, not safety limits

The profile records an initial 5° yaw candidate, obtained by flooring 20% of the observed 0–27° span. This is a conservative software commissioning choice and requires hardware/configuration/clearance confirmation.

Each channel's optional pressure-increment candidate is 20% of the positive span between its baseline p95 and the largest observed point p95 on that same branch. If that span is zero, the candidate is zero. It is not a calibrated pressure limit, evidence of a positive gain, or permission to ignore an unexpected physical response. The runtime controller must apply its own tighter angle, rate, freshness, and feedback gates.

## Reproduction

The offline builder requires NumPy and PyArrow. The JSON consumer does not need PyArrow. The dataset directory contains `raw/bench_20261025_pose_v1/<session>/...`; its `analysis_deps` may hold an isolated PyArrow install.

```powershell
python build_bench_profile.py --dataset-dir C:\path\to\bench_20261025_pose_v1 --output profiles\bench_20261025.json
python -m unittest discover -s tests -p test_bench_profile.py -v
```

An optional `--pyarrow-dir C:\path\to\analysis_deps` keeps the offline dependency separate. The builder reads only allowlisted pressure/motor files and explicitly projected columns. Four tests cover weighted pooling/invalid inputs, measured branch completeness, sample quantile ordering, telemetry evidence, monotonic fit consistency, and the separation between candidates and safety limits.

Remaining commissioning inputs: physical channel-to-seat mapping, confirmation of unchanged servo/mount/zero configuration relative to this bench experiment, and the current neutral/contact baseline. These cannot be recovered from the excluded video or inferred from channel names.

## User-confirmed mapping and direction update (2026-10-25 KST)

The user subsequently supplied this physical mapping in the active conversation:

| Channel | Position | Side group |
|---|---|---|
| A1_C0 | DR, lower right | Right |
| A1_C1 | UR, upper right | Right |
| A1_C2 | DL, lower left | Left |
| A1_C3 | UL, upper left | Left |

The user reported that the mounts, horns, links, and sensors had not changed, and that the recorded experiment caused no noticed discomfort or binding. The user separately confirmed that **positive yaw from 0 to 27 degrees closes both sides toward the body**. These statements resolve the mapping and mechanical-direction questions above. They are user reports; they do not supply a force calibration, an encoder measurement, or a general safe contact limit. The current participant, neutral contact baseline, and clear initial motion still require the live-session checks.

The original `bench_20261025.json` retains the source document's unknown mapping field to preserve its provenance. `mapped_analysis.json` records the later user statement separately, with source hashes, all 56 branch rows including their shared neutral, signed changes, sample statistics, original dwell times, and matched earlier-session comparisons. No image or pose-derived evidence was used to obtain this mapping.

### Signed response under the confirmed mapping

The following numbers are sums of the two mapped **channel means**, relative to that session's own loaded neutral. They are useful electrical summaries, not total contact force or a measured quantile of the channel sum. MUX channels are sampled sequentially within the same dwell; their covariance and the sum's variance are not inferred here.

| Single-side yaw command | Left mean-voltage sum change | Right mean-voltage sum change |
|---|---:|---:|
| Left 1 deg, right 0 | -0.5491 V | +0.0616 V |
| Left 5 deg, right 0 | -0.5232 V | +0.0392 V |
| Left 27 deg, right 0 | -0.6448 V | +0.1882 V |
| Right 1 deg, left 0 | -0.1418 V | -0.0158 V |
| Right 5 deg, left 0 | -0.9022 V | +0.1109 V |
| Right 27 deg, left 0 | -0.3957 V | -0.1146 V |

In the later session, every nonzero point of either independent branch has a lower left mapped sum than the common neutral. Right-side changes are nonmonotonic and the first positive right yaw does not increase its mapped sum. Averaging per-channel historical p95 maxima would hide these negative changes and combine peaks recorded at different angles. That construction remains useful as an upper guard envelope; it cannot be inverted as a pressure plant response.

This does **not** contradict the user's mechanical-direction observation or prove that closing reduces pressure. The experiment does not isolate angle from time and contact changes. In the earlier left-only session, the same full commands gave the opposite sign:

| Left yaw, all other angles zero | Earlier left sum change | Later left sum change |
|---|---:|---:|
| 1 deg | +0.7896 V | -0.5491 V |
| 5 deg | +0.4157 V | -0.5232 V |
| 20 deg | +0.5889 V | -0.4479 V |

Both columns use their **own** session's neutral; this is not merely a fixed voltage offset. The later independent left points were separated by long intervening right-yaw scans: left 1 deg was measured 176.944 seconds after neutral, left 5 deg 536.407 seconds after neutral, and left 27 deg 2516.960 seconds after neutral. The initial right branch spans 0 to 86.985 seconds. These timing differences prevent a clean causal direction or hysteresis estimate.

For a further first-versus-last check, the 21 shared equal-left/right-yaw commands at zero pitch have the following absolute same-command mean differences:

| Channel | Median difference | Maximum difference |
|---|---:|---:|
| A1_C0 / DR | 0.3222 V | 0.5601 V |
| A1_C1 / UR | 0.2317 V | 0.4057 V |
| A1_C2 / DL | 0.0487 V | 0.2578 V |
| A1_C3 / UL | 0.1117 V | 0.4956 V |

Both actuators moved in that comparison; it demonstrates between-session variability, not independent actuator gains.

### Consequence for the deterministic fallback

The confirmed inward direction allows a **position feedforward cue**: filter the vehicle lateral acceleration, choose the outside bolster, and scale its initial yaw cap by normalized demand. The pressure evidence does not justify pressure-target inversion, PI pressure tracking, or a claim that the cue cancels acceleration or reduces sway. The initial contiguous p95 guard estimates (about 0.497 deg left and 0.300 deg right under the four-A1 guard configuration) are commissioning candidates based on interpolation, not guaranteed safe limits or proof of a 20% pressure change.

Fresh neutral baselines, live per-channel increment/hotspot checks, contact-change checks, rate limits, reversal through commanded neutral, stale-input shutdown, and latched faults remain necessary parts of that position-cue policy. Signed pressure-response qualification is advisory for this policy. It must become a prerequisite if a later controller attempts to track a pressure target.

### Proposed follow-up measurement plan; documentation only

Do not automatically expand to 1 or 2 degrees on the strength of this note. Those are optional measurement candidates above the current initial software caps and require a separate deliberate commissioning decision. Start within the already checked local range, verify the actual direction, and increase only when the operator chooses it.

A future plan CSV could use columns `trial_id,block_id,condition,left_yaw_deg,left_pitch_deg,right_yaw_deg,right_pitch_deg,settle_s,dwell_s,operator_gate`. Use neutral `(0,0,0,0)` immediately before and after every single-side target; keep the opposite yaw and both pitches zero. Candidate conditions are one left small-angle target and one right small-angle target, later adding 1 and 2 degrees if commissioned. Repeat each condition at least five times, randomizing condition order with a recorded seed and balancing left/right order. Record any aborted trial instead of silently dropping it. This is an identification proposal, not an automatically executable motor schedule; no plan or commands have been uploaded.

Record only raw pressure, command/ACK/state/PWM, clocks, configuration, and operator events. The earlier 0.5-second settle and 2-second dwell are starting choices, not proven physical settling times; record fresh samples and actual interval boundaries and lengthen the dwell if contact is still drifting. Compare each target against its adjacent before/after neutral measurements. Check sign consistency, repeated-trial spread, channel hotspots, and neutral-return drift before fitting a local response. Repeated outward/inward pairs can then test hysteresis. Do not fit an inverse pressure law when those checks remain unresolved.

To reproduce the mapped analysis from the preserved pressure-only point table:

```powershell
python -m fallback.build_mapped_analysis --point-table C:\path\to\bench_20261025_pose_v1\pressure_command_points.csv --output fallback\profiles\mapped_analysis.json
```

Run this from `4_Seat_Actuation`. The builder validates the pressure/motor-only CSV schema and hashes the profile, derived table, and builder source. It never opens a serial port, socket, camera, or video/pose table.
