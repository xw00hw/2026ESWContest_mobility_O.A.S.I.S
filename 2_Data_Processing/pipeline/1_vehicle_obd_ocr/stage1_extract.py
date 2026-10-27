"""Stage 1: decode every frame once; per tile detect ROI changes and store unique ROI crops.

Output cache (_work/1_vehicle_obd_ocr/cache/stage1.pkl):
  pts            int64[N]      frame pts (1/90000 s)
  val_id/rng_id  int32[N,6]    index into val_crops[k] / rng_crops[k] (crop inherited while unchanged)
  val_crops/rng_crops  list[6] of list of PNG bytes (BGR crop at the frame where it changed)
  val_first/rng_first  list[6] of list of frame_idx where each crop first appeared
  title_diff     float32[N,6]  mean abs diff of the static title ROI vs reference (layout check)
  static_diff    float32[N]    mean abs diff of whole-frame static mask vs reference
  dot            uint8[N,6,3]  mean BGR of the green dot
  frame_diff     float32[N]    mean abs diff of whole frame vs previous frame
"""
import pickle
import sys
import time

import cv2
import numpy as np

from common import (CACHE, DOT_ROI, RNG_ROI, TITLE_ROI, VAL_ROI, iter_frames,
                    probe_pts, roi)

DIFF_LEVEL = 40   # per-pixel abs gray difference regarded as real change
DIFF_COUNT = 4    # number of such pixels needed to call the ROI "changed"


def main():
    CACHE.mkdir(parents=True, exist_ok=True)
    pts = probe_pts()
    N = len(pts)
    val_id = np.zeros((N, 6), np.int32); rng_id = np.zeros((N, 6), np.int32)
    title_diff = np.zeros((N, 6), np.float32); dot = np.zeros((N, 6, 3), np.uint8)
    frame_diff = np.zeros(N, np.float32)
    crops = {"val": [[] for _ in range(6)], "rng": [[] for _ in range(6)]}
    first = {"val": [[] for _ in range(6)], "rng": [[] for _ in range(6)]}
    last = {"val": [None] * 6, "rng": [None] * 6}
    ids = {"val": val_id, "rng": rng_id}
    rois = {"val": VAL_ROI, "rng": RNG_ROI}
    ref_title = None; prev_small = None
    t0 = time.time(); n = 0
    for n, fr in enumerate(iter_frames()):
        if n >= N:
            raise SystemExit("decoded more frames than pts")
        g = cv2.cvtColor(fr, cv2.COLOR_BGR2GRAY)
        if ref_title is None:
            ref_title = [roi(g, TITLE_ROI, k).astype(np.int16) for k in range(6)]
        small = g[::4, ::4].astype(np.int16)
        if prev_small is not None:
            frame_diff[n] = np.abs(small - prev_small).mean()
        prev_small = small
        for k in range(6):
            title_diff[n, k] = np.abs(roi(g, TITLE_ROI, k).astype(np.int16) - ref_title[k]).mean()
            dot[n, k] = roi(fr, DOT_ROI, k).reshape(-1, 3).mean(0)
            for kind in ("val", "rng"):
                c = roi(g, rois[kind], k).astype(np.int16)
                l = last[kind][k]
                # compare against the last STORED crop so slow drift cannot accumulate unnoticed
                if l is None or (np.abs(c - l) > DIFF_LEVEL).sum() >= DIFF_COUNT:
                    last[kind][k] = c
                    crops[kind][k].append(cv2.imencode(".png", roi(fr, rois[kind], k))[1].tobytes())
                    first[kind][k].append(n)
                ids[kind][n, k] = len(crops[kind][k]) - 1
        if n % 2000 == 0:
            print(f"frame {n}/{N}  {time.time()-t0:.0f}s  val crops {[len(c) for c in crops['val']]}", flush=True)
    assert n + 1 == N, f"decoded {n+1} frames but ffprobe lists {N} pts"
    with open(CACHE / "stage1.pkl", "wb") as f:
        pickle.dump(dict(pts=pts, val_id=val_id, rng_id=rng_id, val_crops=crops["val"],
                         rng_crops=crops["rng"], val_first=first["val"], rng_first=first["rng"],
                         title_diff=title_diff, dot=dot, frame_diff=frame_diff), f)
    print("done", N, "frames;", "val crops", [len(c) for c in crops["val"]],
          "rng crops", [len(c) for c in crops["rng"]])


if __name__ == "__main__":
    sys.exit(main())
