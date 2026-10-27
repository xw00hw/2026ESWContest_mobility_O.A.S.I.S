"""Render blind-audit contact sheets (no OCR answers on them) + a key file kept in a separate folder.
Frames are decoded with OpenCV VideoCapture (a different decode path than the ffmpeg pipe of stage 1),
so the audit also cross-checks the frame_idx alignment.
Usage: make_audit_sheets.py --seed 1 --n 360 --out _work/1_vehicle_obd_ocr/audit/a1
"""
import argparse
import pickle
from pathlib import Path

import cv2
import numpy as np
import pandas as pd

from common import CACHE, OUT2, PITCH, RAW_MP4, ROOT, SIGNALS

PER_SHEET = 6
CROP = (98, 246, 190, 890)  # y0,y1,x0,x1 of tile 0: value line + range line (no title, no dot)


def sample(seed, n):
    rng = np.random.default_rng(seed)
    txt = pd.read_csv(OUT2 / "obd_text.csv", dtype=str, keep_default_na=False)
    obd = pd.read_csv(OUT2 / "obd.csv")
    ch = pd.read_csv(OUT2 / "obd_changes.csv", dtype={"old": str, "new": str}, keep_default_na=False)
    r = pickle.load(open(CACHE / "stage2.pkl", "rb"))
    d = pickle.load(open(CACHE / "stage1.pkl", "rb"))
    N = len(txt)
    items = {}

    def add(f, k, stratum):
        if 0 <= f < N:
            items.setdefault((int(f), int(k)), stratum)

    def quota(frac):
        return max(1, int(n * frac))

    # hard strata first
    for k, sig in enumerate(SIGNALS):  # every flagged cell
        for f in np.flatnonzero(obd[sig + "_ok"].values == 0)[:50]:
            add(f, k, "flagged")
    dis = [(kind, k, i) for kind in ("val", "rng") for k in range(6) for i, x in enumerate(r[kind][k])
           if not (x["g"] == x["t"] == x["r"])]
    for j in rng.permutation(len(dis))[:quota(0.17)]:  # crops where an engine (Tesseract) disagreed
        kind, k, i = dis[j]
        fr = np.flatnonzero(d[kind + "_id"][:, k] == i)
        add(rng.choice(fr), k, "engine_disagreement")
    ch["k"] = ch.signal.map({s: i for i, s in enumerate(SIGNALS)})
    ch["fo"] = pd.to_numeric(ch.old)
    ch["fn"] = pd.to_numeric(ch.new)
    ch["jump"] = (ch.fn - ch.fo).abs()
    for k in range(6):
        c = ch[ch.k == k]
        for f in c.nlargest(quota(0.02), "jump").frame_idx:  # largest jumps: frame of change and the one before
            add(f, k, "big_jump")
            add(f - 1, k, "big_jump_prev")
        for f in c.nlargest(2, "fn").frame_idx:
            add(f, k, "extreme_max")
        for f in c.nsmallest(2, "fn").frame_idx:
            add(f, k, "extreme_min")
        sc = c[(c.old.str.startswith("-")) != (c.new.str.startswith("-"))]
        for f in sc.frame_idx.sample(min(len(sc), quota(0.015)), random_state=seed):
            add(f, k, "sign_change")
        nz = c[c.new.str.fullmatch(r"-0(\.0+)?")]
        for f in nz.frame_idx.sample(min(len(nz), quota(0.01)), random_state=seed):
            add(f, k, "negative_zero")
        neg = c[c.new.str.startswith("-")]
        for f in neg.frame_idx.sample(min(len(neg), quota(0.015)), random_state=seed):
            add(f, k, "negative")
        for f in c.frame_idx.sample(quota(0.015), random_state=seed + 1):
            add(f, k, "at_change")
    while len(items) < n:  # uniform random (frame, tile)
        add(rng.integers(N), rng.integers(6), "random")
    keys = list(items)
    order = rng.permutation(len(keys))
    rows = []
    for j, o in enumerate(order):
        f, k = keys[o]
        sig = SIGNALS[k]
        mn, mx = (txt.at[f, sig + "_range_text"].split(" - ") + ["", ""])[:2]
        rows.append(dict(item_id=f"A{j:03d}", frame_idx=f, tile=k, stratum=items[(f, k)],
                         csv_value_text=txt.at[f, sig + "_text"], csv_min_text=mn, csv_max_text=mx))
    return pd.DataFrame(rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--n", type=int, default=360)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    out = (ROOT / a.out) if not Path(a.out).is_absolute() else Path(a.out)
    keydir = out.parent / (out.name + "_key")
    out.mkdir(parents=True, exist_ok=True)
    keydir.mkdir(parents=True, exist_ok=True)
    key = sample(a.seed, a.n)
    key.to_csv(keydir / "key.csv", index=False, lineterminator="\n")
    want = {}
    for _, r in key.iterrows():
        want.setdefault(r.frame_idx, []).append((r.item_id, r.tile))
    crops = {}
    cap = cv2.VideoCapture(str(RAW_MP4))
    f = 0
    last = max(want)
    while f <= last:
        ok, fr = cap.read()
        assert ok, f"decode ended at {f}"
        for item, k in want.get(f, []):
            y0, y1, x0, x1 = CROP
            c = cv2.resize(fr[y0 + PITCH * k:y1 + PITCH * k, x0:x1], None, fx=1.5, fy=1.5,
                           interpolation=cv2.INTER_CUBIC)
            c = cv2.copyMakeBorder(c, 34, 6, 6, 6, cv2.BORDER_CONSTANT, value=(60, 60, 60))
            cv2.putText(c, item, (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
            crops[item] = c
        f += 1
    ids = list(key.item_id)
    for s in range(0, len(ids), PER_SHEET):
        cv2.imwrite(str(out / f"sheet_{s // PER_SHEET:03d}.png"),
                    np.vstack([crops[i] for i in ids[s:s + PER_SHEET]]))
    (out / "README.txt").write_text(
        "Each sheet shows 6 tile crops captioned with an item id. Big blue number = current value; small grey "
        "line below = min - max.\nTranscribe exactly as displayed (keep minus signs, decimal points, trailing "
        "zeros; ignore the unit) into answers.csv with header:\nitem_id,value,min,max\n", encoding="utf-8")
    print(len(ids), "items,", (len(ids) + PER_SHEET - 1) // PER_SHEET, "sheets ->", out)
    print(key.stratum.value_counts().to_string())


if __name__ == "__main__":
    main()
