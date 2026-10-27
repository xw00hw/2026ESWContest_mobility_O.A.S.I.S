"""A2: run1 X431 xlsx -> 01_refined/run1/obd.csv (+ checks json). No timestamps exist in the source."""
import json

import numpy as np
import openpyxl
import pandas as pd

from common import OUT1, QC, RAW_XLSX

COLS = {"Sample": "sample_idx", "Relative throttle (%)": "throttle_rel_pct",
        "Absolute throttle (%)": "throttle_abs_pct", "Reference torque (Nm)": "reference_torque_nm",
        "Engine RPM (rpm)": "engine_rpm", "Vehicle speed (km/h)": "vehicle_speed_kmh"}
RANGE = {"throttle_rel_pct": (0, 100), "throttle_abs_pct": (0, 100), "reference_torque_nm": (0, 5000),
         "engine_rpm": (0, 8000), "vehicle_speed_kmh": (0, 250)}


def main():
    ws = openpyxl.load_workbook(RAW_XLSX, data_only=True)["측정 데이터"]
    rows = list(ws.iter_rows(values_only=True))
    h = next(i for i, r in enumerate(rows) if r[0] == "Sample")
    assert list(rows[h]) == list(COLS), rows[h]
    df = pd.DataFrame(rows[h + 1:], columns=list(COLS.values()))
    df = df[~df.isna().all(axis=1)].reset_index(drop=True)
    num = df.apply(pd.to_numeric, errors="coerce")
    valid = ~num.isna().any(axis=1)
    for c, (lo, hi) in RANGE.items():
        valid &= num[c].between(lo, hi)
    df["valid"] = valid.astype(int)
    df["flag_rpm_zero_moving"] = ((num.engine_rpm == 0) & (num.vehicle_speed_kmh > 0)).astype(int)
    OUT1.mkdir(parents=True, exist_ok=True); QC.mkdir(parents=True, exist_ok=True)
    df.to_csv(OUT1 / "obd.csv", index=False, lineterminator="\n")

    chk = {"rows": len(df), "sample_contiguous_1_to_N": bool((num.sample_idx.values == np.arange(1, len(df) + 1)).all()),
           "nan_cells": int(num.isna().sum().sum()), "invalid_rows": int((~valid).sum()),
           "rpm_zero_while_moving": int(df.flag_rpm_zero_moving.sum()),
           "rpm_zero_total": int((num.engine_rpm == 0).sum()), "columns": {}}
    for c in list(COLS.values())[1:]:
        v = num[c].values
        u = np.unique(v)
        step = float(np.min(np.diff(u))) if len(u) > 1 else None
        chk["columns"][c] = {"min": float(v.min()), "max": float(v.max()), "n_unique": int(len(u)),
                             "min_step_between_unique": step, "constant": bool(len(u) == 1),
                             "max_abs_sample_jump": float(np.abs(np.diff(v)).max()),
                             "all_integer": bool((v == np.round(v)).all())}
    for c in ("throttle_rel_pct", "throttle_abs_pct"):  # 8-bit PID scaling n*100/255 ?
        n = num[c].values * 255 / 100
        chk["columns"][c]["max_dev_from_n_x_100_over_255"] = float(np.abs(n - np.round(n)).max())
    r = num.engine_rpm.values
    chk["columns"]["engine_rpm"]["max_dev_from_multiple_of_0.25"] = float(np.abs(r * 4 - np.round(r * 4)).max())
    # longest run of identical consecutive rows (hint at the logger's sampling vs ECU update rate)
    same = (num.iloc[:, 1:].diff().abs().sum(axis=1) == 0).values
    run = best = 0
    for s in same:
        run = run + 1 if s else 0; best = max(best, run)
    chk["rows_identical_to_previous"] = int(same.sum()); chk["longest_identical_run"] = int(best + 1)
    (QC / "run1_checks.json").write_text(json.dumps(chk, indent=1), encoding="utf-8")
    print(json.dumps(chk, indent=1))
    return df


if __name__ == "__main__":
    main()
