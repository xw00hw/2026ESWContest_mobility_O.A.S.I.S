"""Stage 2: run engines G (templates), T (Tesseract), R (RapidOCR) on every unique ROI crop.
Output cache stage2.pkl: {kind: [per tile list of dict(g, g_score, g_margin, t, r)]}"""
import pickle
import sys
from multiprocessing import Pool

import cv2
import numpy as np

from common import CACHE
from ocr import engine_g, engine_r, engine_t


def work(args):
    kind, k, i, b = args
    im = cv2.imdecode(np.frombuffer(b, np.uint8), 1)
    g, sc, mg = engine_g(im, kind)
    return kind, k, i, dict(g=g, g_score=sc, g_margin=mg, t=engine_t(im, kind), r=engine_r(im, kind))


def main():
    d = pickle.load(open(CACHE / "stage1.pkl", "rb"))
    jobs = [(kind, k, i, b) for kind in ("val", "rng") for k in range(6)
            for i, b in enumerate(d[kind + "_crops"][k])]
    res = {kind: [[None] * len(d[kind + "_crops"][k]) for k in range(6)] for kind in ("val", "rng")}
    with Pool(8) as p:
        for n, (kind, k, i, r) in enumerate(p.imap_unordered(work, jobs, chunksize=16)):
            res[kind][k][i] = r
            if n % 1000 == 0:
                print(n, "/", len(jobs), flush=True)
    pickle.dump(res, open(CACHE / "stage2.pkl", "wb"))
    print("done", len(jobs))


if __name__ == "__main__":
    sys.exit(main())
