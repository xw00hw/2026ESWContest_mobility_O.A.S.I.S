# pressure_seat — driver seat pressure recorder

Headless recorder for the 12-FSR seat (NUCLEO-H723ZG, firmware `FSR_12_SCAN v9`,
3 MUX × C0–C3 on A0/A1/A2, 100 Hz per sensor, 460800 bps over the ST-LINK USB
port). It records a driving session to CSV in the same layout and with the same
time columns as the Raspberry Pi DAQ. Synchronisation with the other recorders
is described in [`../README.md`](../README.md).

This folder is self-contained (no imports from the firmware folders). Only one
program can hold the COM port at a time.

## Run

```powershell
py -m pip install -r requirements.txt        # pyserial, once
py collect_seat.py                           # auto-detects the ST-LINK port; Ctrl+C stops
py collect_seat.py --port COM7 --suffix driverA_run1
py collect_seat.py --list-ports
```

One status line per second shows the frame count, lost packets, ADC errors and
the latest 12 raw values. If the USB cable drops, the board resets, or data
stops for 3 s, the fault is logged and the recorder reconnects into the **same**
session; it does not exit until Ctrl+C (or `--seconds`).

## Output

```
data/sessions/YYYY-MM-DD/session_YYYYMMDD_HHMMSS[_suffix]/
├── seat/pressure.csv       one row per scan of the 12 sensors (100 Hz nominal)
├── seat/raw_uart.log       every received line: "<monotonic_ns> <wall_ns> <line>"
├── events.jsonl            recording_started, link_fault, …
├── metadata.json           clock + NTP offset at start, port, firmware, config copy
├── session_summary.json    counters, measured rate, NTP offset at end
└── INCOMPLETE              present only while recording or after a crash
```

`data/` is not committed. `--data-root` moves it.

### `seat/pressure.csv`

| column | notes |
|---|---|
| `host_monotonic_ns`, `host_wall_time_ns` | laptop receive time of the frame's first packet |
| `sequence_id` | frame counter within the session |
| `packet_seq`, `mcu_t_ms` | firmware packet counter and `HAL_GetTick()` ms of the first packet |
| `packets_in_frame` | 4 when every enabled MUX channel arrived |
| `frame_valid` | 1 only if all packets arrived and every enabled ADC read succeeded |
| `<id>_raw` ×12 | 12-bit ADC counts, 0–4095. **Blank** on an ADC error or a lost packet, never 0 |
| `<id>_v` ×12 | `raw × vref / 4095` volts |

Sensor ids (`A0_C0` … `A2_C3`) and `vref` come from `seat_config.json`: A0 = seat
base, A1 = side, A2 = back, as wired for the 2026-10-07 measurement.

Read before using the values:

- **Volts, not pressure.** No N / kg / kPa calibration is applied, `vref` is the
  nominal 3.3 V rather than a measured one, and no baseline is subtracted.
- **A frame is not one instant.** The four MUX channels are scanned in turn
  (about 2.5 ms apart) and the three ADCs of a channel are converted one after
  another, so a row spans up to ~10 ms.
- **Time stamps are USB receive times** with a few ms of jitter. `mcu_t_ms` and
  `raw_uart.log` are kept so the timing can be refined later if needed.
- Left/right and front/back positions of the sensors are not recorded here;
  `seat_config.json` still lists `side` as `unknown`. The real-vehicle drive gives a
  data-derived left/right guess from each channel's response to lateral
  acceleration (`2_Data_Processing/README.md` §6); confirm it on the seat.

## Test

```powershell
py -m pip install pytest
py -m pytest tests -q        # fake board, opens no serial port
```

Covered: handshake and heartbeat, wide-frame assembly, blank-not-zero on ADC
errors, lost-packet counting, reconnect after a board reset, clean shutdown.
Not covered: the real board, real USB timing.
