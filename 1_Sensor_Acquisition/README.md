# 1_Sensor_Acquisition

Driving-data collection for O.A.S.I.S. Three devices record **separately and at
the same time**; their recordings are joined afterwards on a common UTC timeline.

| recorder | hardware | runs on | code | output |
|---|---|---|---|---|
| GPS + IMU + camera + MediaPipe pose | Raspberry Pi 5 | the Pi | [`raspberry_pi/`](raspberry_pi/README.md) | `imu/imu.csv`, `gps/gps.csv`, `pose/features.csv`, `camera/…` |
| driver seat pressure, 12 FSR | NUCLEO-H723ZG | laptop (ST-LINK USB) | [`pressure_seat/`](pressure_seat/README.md) | `seat/pressure.csv` |
| vehicle data | commercial OBD tools (Launch X431, Infocar app screen) | scanner / phone | OCR + parsing in [`../2_Data_Processing/`](../2_Data_Processing/README.md) (part A) | `obd/` |

`align_sessions.py` in this folder merges the three into one CSV.

The final seat board firmware, which scans the same 12 channels, is in
[`../4_Seat_Actuation/stm32/`](../4_Seat_Actuation/stm32/README.md); nothing here imports from it.

## Common format

All three recorders write CSV into
`<data root>/sessions/YYYY-MM-DD/session_YYYYMMDD_HHMMSS[_suffix]/`, with
`metadata.json`, `session_summary.json` and an `INCOMPLETE` marker that is
removed only on a clean stop. Every row starts with the same two columns:

| column | meaning |
|---|---|
| `host_monotonic_ns` | the recording host's own monotonic clock. Ordering and intervals **within one device** |
| `host_wall_time_ns` | UNIX epoch ns (UTC) on that host. The link **between devices** |

## Time synchronisation

There is no wire between the devices, so the shared reference is UTC.

1. Each device's monotonic clock is mapped to its wall clock with one constant
   per session (`median(wall − monotonic)`), which keeps the smooth monotonic
   spacing and ignores wall-clock steps.
2. Each device's wall-clock error is then removed:
   - **laptop (seat, OBD)** — the recorders measure the offset to an NTP server
     at session start and end and store it in `metadata.json` /
     `session_summary.json`. Needs internet at those two moments (phone hotspot).
   - **Pi** — unchanged code. If its clock is NTP-synced it is used as-is. If it
     is not (no network in the car), `align_sessions.py` detects that from the
     GPS time already logged in `gps/gps.csv` and corrects with it.

Expected accuracy is **tens of milliseconds**, limited by NTP over a mobile
link and by USB/serial receive latency. It is not sub-millisecond: for that the
devices would need a shared hardware trigger. The GPS fallback additionally
includes the receiver's NMEA output delay (unmeasured, typically 0.1–0.4 s);
override with `--pi-offset-s` once it has been measured.

## Drive-day procedure

1. Laptop and Pi on the phone hotspot, so both clocks are NTP-synced and the
   recorders can measure their offset.
2. Start the Pi DAQ as usual ([`raspberry_pi/docs/OPERATIONS.md`](raspberry_pi/docs/OPERATIONS.md)).
3. On the laptop, in two terminals:
   ```powershell
   py "1_Sensor_Acquisition\pressure_seat\collect_seat.py" --suffix driverA_run1
   ```
4. Drive. Stop the laptop recorders with Ctrl+C and the Pi session as usual.
   Start order does not matter; only the time all three overlap is merged.
5. Copy the Pi session folder to the laptop, then:
   ```powershell
   py "1_Sensor_Acquisition\align_sessions.py" --pi <pi session> --seat <seat session> --obd <obd session> --out aligned.csv
   ```

`align_sessions.py` prints each stream's UTC range and the clock offset it
applied, then writes one row per grid step (`--hz`, default 20) holding the most
recent sample of every stream — no interpolation — with a `<stream>_age_ms`
column per stream. Values older than `--max-age-s` (default 1.5) are left blank.
The raw session folders are never modified; rerun with different options freely.

```powershell
py "1_Sensor_Acquisition\align_sessions.py" --selftest
```

## Field use

First real-vehicle drive. The Pi DAQ and the pressure-seat recorder
ran in the car (the seat recorder on the real NUCLEO board, 6 sessions). OBD data
came from commercial tools (Launch X431, Infocar app screen recording), and the streams were aligned by the refinement pipeline in
[`../2_Data_Processing/`](../2_Data_Processing/README.md)
(GPS-disciplined clock; measured seat ±0.1 s, OBD ±0.15 s) rather than
`align_sessions.py`.

**For that drive the IMU and the seat sensors were on the passenger side.**
Record which seat every sensor and the subject are on, and the IMU-to-seat
distance, on each drive.

Still not run in a vehicle: `align_sessions.py` on real sessions.
