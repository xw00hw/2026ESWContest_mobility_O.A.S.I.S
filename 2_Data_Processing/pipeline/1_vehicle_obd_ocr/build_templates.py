"""Build glyph templates for the Infocar font (engine G) by unsupervised clustering of connected
components of all unique ROI crops.  Cluster labels were assigned by the lead by eye from
02_qc/1_vehicle_obd_ocr/templates_{val,rng}.png (the sheet shows the cluster index) and are embedded below;
the clustering is deterministic, and the script asserts the cluster structure it was labelled for.
Output: pipeline/1_vehicle_obd_ocr/templates.npz (committed asset used by ocr.py).
"""
import pickle
from pathlib import Path

import cv2
import numpy as np

from common import CACHE, QC
from ocr import TH, soft

MAXW = {"val": 34, "rng": 17}   # wider components are touching glyph pairs -> not templates
MIN_N = 5
# labels of the single-glyph clusters, in order of decreasing cluster size (see templates_*.png)
LABELS = {"val": "0.-2131564798318405", "rng": "0-.23.41596781"}


def cluster(d, kind):
    clusters = []
    for k in range(6):
        for b in d[kind + "_crops"][k]:
            s = soft(cv2.imdecode(np.frombuffer(b, np.uint8), 1), kind)
            n, lab, st, _ = cv2.connectedComponentsWithStats((s > TH[kind]).astype(np.uint8), 8)
            for i in range(1, n):
                x, y, w, h, a = st[i]
                if a < 4 or w > MAXW[kind]:
                    continue
                p = np.zeros((s.shape[0], MAXW[kind]), np.float32)  # full ROI height keeps y position
                p[y:y + h, :w] = s[y:y + h, x:x + w] * (lab[y:y + h, x:x + w] == i)
                for c in clusters:
                    if abs(c["h"] - h) <= 1 and abs(c["w0"] - w) <= 1:
                        m = c["sum"] / c["n"]
                        if (m * p).sum() / np.sqrt((m * m).sum() * (p * p).sum()) > 0.9:
                            c["sum"] += p; c["n"] += 1; c["w"] = max(c["w"], w)
                            break
                else:
                    clusters.append(dict(h=h, w=w, w0=w, sum=p.copy(), n=1))
    clusters = [c for c in clusters if c["n"] >= MIN_N]
    clusters.sort(key=lambda c: -c["n"])
    return clusters


def main():
    d = pickle.load(open(CACHE / "stage1.pkl", "rb"))
    out = {}
    for kind in ("val", "rng"):
        cl = cluster(d, kind)
        tiles = []
        for i, c in enumerate(cl):
            m = (c["sum"] / c["n"]).astype(np.uint8)[:, :c["w"]]
            lab = LABELS[kind][i] if LABELS[kind] else "?"
            im = cv2.resize(255 - m, None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST)
            im = cv2.copyMakeBorder(im, 44, 4, 6, 6, cv2.BORDER_CONSTANT, value=200)
            cv2.putText(im, f"#{i} '{lab}'", (2, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, 0, 1)
            cv2.putText(im, f"n={c['n']}", (2, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.4, 0, 1)
            tiles.append(im)
            out[f"{kind}_{i}"] = m
        wmax = max(t.shape[1] for t in tiles)
        tiles = [cv2.copyMakeBorder(t, 0, 0, 0, wmax - t.shape[1], cv2.BORDER_CONSTANT, value=200) for t in tiles]
        rows = [np.hstack(tiles[i:i + 10] + [np.full_like(tiles[0], 200)] * (10 - len(tiles[i:i + 10])))
                for i in range(0, len(tiles), 10)]
        QC.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(QC / f"templates_{kind}.png"), np.vstack(rows))
        print(kind, [(i, c["h"], c["w"], c["n"]) for i, c in enumerate(cl)])
        if LABELS[kind]:
            assert len(LABELS[kind]) == len(cl), (kind, len(cl))
            out[f"{kind}_labels"] = np.array(list(LABELS[kind]))
    if all(LABELS.values()):
        np.savez_compressed(Path(__file__).with_name("templates.npz"), **out)
        print("templates.npz written")


if __name__ == "__main__":
    main()
