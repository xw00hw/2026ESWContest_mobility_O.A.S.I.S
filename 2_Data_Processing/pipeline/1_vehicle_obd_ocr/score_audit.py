"""Score a blind transcription (answers.csv: item_id,value,min,max) against the audit key.
Usage: score_audit.py --key <dir>_key/key.csv --answers a.csv [b.csv ...] [--sheets <dir>] [--out res.json]"""
import argparse
import json
from pathlib import Path

import cv2
import pandas as pd


def norm(s):
    return str(s).strip().replace("−", "-").replace("–", "-").replace("—", "-").replace(" ", "")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--key", required=True)
    ap.add_argument("--answers", nargs="+", required=True)
    ap.add_argument("--sheets")
    ap.add_argument("--out")
    a = ap.parse_args()
    key = pd.read_csv(a.key, dtype=str, keep_default_na=False)
    ans = pd.concat([pd.read_csv(p, dtype=str, keep_default_na=False) for p in a.answers])
    assert not ans.item_id.duplicated().any(), "duplicate item ids in answers"
    m = key.merge(ans, on="item_id", how="left", indicator=True)
    missing = m[m._merge == "left_only"].item_id.tolist()
    m = m[m._merge == "both"].copy()
    for c, k in (("value", "csv_value_text"), ("min", "csv_min_text"), ("max", "csv_max_text")):
        m[c + "_match"] = m[c].map(norm) == m[k].map(norm)
    bad = m[~(m.value_match & m.min_match & m.max_match)]
    res = {"items_in_key": len(key), "items_answered": len(m), "items_missing": missing,
           "tile_values_compared": int(3 * len(m)),
           "value_mismatch": int((~m.value_match).sum()), "min_mismatch": int((~m.min_match).sum()),
           "max_mismatch": int((~m.max_match).sum()),
           "by_stratum_items": m.stratum.value_counts().to_dict(),
           "mismatches": bad[["item_id", "frame_idx", "tile", "stratum", "csv_value_text", "value",
                              "csv_min_text", "min", "csv_max_text", "max"]].to_dict("records")}
    print(json.dumps(res, indent=1, ensure_ascii=False))
    if a.out:
        Path(a.out).write_text(json.dumps(res, indent=1, ensure_ascii=False), encoding="utf-8")
    if a.sheets and len(bad):  # enlarge mismatching crops for review
        md = Path(a.sheets) / "mismatch"
        md.mkdir(exist_ok=True)
        ids = key.item_id.tolist()
        for it in bad.item_id:
            j = ids.index(it)
            sh = cv2.imread(str(Path(a.sheets) / f"sheet_{j // 6:03d}.png"))
            h = sh.shape[0] // min(6, len(ids) - (j // 6) * 6)
            cv2.imwrite(str(md / f"{it}.png"), cv2.resize(sh[(j % 6) * h:(j % 6 + 1) * h], None, fx=2, fy=2,
                                                          interpolation=cv2.INTER_CUBIC))


if __name__ == "__main__":
    main()
