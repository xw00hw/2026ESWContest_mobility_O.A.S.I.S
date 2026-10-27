"""Stage 4: end-to-end self check of the written CSV.
Decodes the video again with OpenCV (different decode path than stage 1), runs the template engine
directly on freshly cropped ROIs of randomly chosen frames (no change detection / inheritance) and
compares with obd_text.csv.  Catches frame_idx misalignment and missed change detections.
Also asserts the known probe readings near t=2 s and t=800 s (read by eye by the lead).
"""
import json

import cv2
import numpy as np
import pandas as pd

from common import OUT2, QC, RAW_MP4, RNG_ROI, SIGNALS, VAL_ROI, roi
from ocr import engine_g, split_range

N_RANDOM = 3000
PROBES = {2.0: (["0", "9", "0.05", "-261.2", "-0.04", "-17.9"],
                ["0 - 1875", "0 - 11", "-0.02 - 0.08", "-261.2 - 402.3", "-0.04 - 0.07", "-17.9 - 9.4"]),
          800.0: (["0", "3", "-0.06", "-0.5", "0.02", "0.0"],
                  ["0 - 2700", "0 - 57", "-0.06 - 0.30", "-0.5 - 402.3", "-0.01 - 0.19", "-0.1 - 25.0"])}


def main():
    txt = pd.read_csv(OUT2 / "obd_text.csv", dtype=str, keep_default_na=False)
    obd = pd.read_csv(OUT2 / "obd.csv")
    N = len(txt)
    t = obd.t_video_s.values
    assert (np.diff(t) > 0).all() and (obd.frame_idx.values == np.arange(N)).all()
    res = {"rows": int(N), "probe_checks": {}}
    for tp, (vals, rngs) in PROBES.items():
        near = np.flatnonzero(np.abs(t - tp) < 0.5)  # ffmpeg -ss seek is only frame-approximate
        hit = [i for i in near if [txt.at[i, s + "_text"] for s in SIGNALS] == vals
               and [txt.at[i, s + "_range_text"] for s in SIGNALS] == rngs]
        res["probe_checks"][str(tp)] = {"frames_within_0.5s": int(len(near)), "frames_matching_probe": int(len(hit))}
        assert hit, f"probe at {tp}s not reproduced"
    pick = set(np.random.default_rng(7).choice(N, N_RANDOM, replace=False).tolist())
    cap = cv2.VideoCapture(str(RAW_MP4))
    cells = bad = 0
    examples = []
    for f in range(N):
        ok, fr = cap.read()
        assert ok, f"OpenCV decode ended early at frame {f}"
        if f not in pick:
            continue
        for k, sig in enumerate(SIGNALS):
            v = engine_g(roi(fr, VAL_ROI, k), "val")[0]
            a, b = split_range(engine_g(roi(fr, RNG_ROI, k), "rng")[0])
            got = (v or "", f"{a} - {b}")
            exp = (txt.at[f, sig + "_text"], txt.at[f, sig + "_range_text"])
            cells += 2
            if got != exp:
                bad += (got[0] != exp[0]) + (got[1] != exp[1])
                examples.append((f, sig, got, exp))
    assert not cap.read()[0], "OpenCV decodes more frames than the CSV has rows"
    res.update(random_frames=N_RANDOM, cells_compared=cells, cells_mismatch=bad, mismatch_examples=examples[:20])
    (QC / "run2_selfcheck.json").write_text(json.dumps(res, indent=1), encoding="utf-8")
    print(json.dumps(res, indent=1))
    assert bad == 0, "self check found mismatches"


if __name__ == "__main__":
    main()
