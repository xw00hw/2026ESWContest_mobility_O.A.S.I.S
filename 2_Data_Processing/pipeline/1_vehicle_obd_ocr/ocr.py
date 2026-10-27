"""OCR engines for the Infocar tiles.

Engine G: greedy left-to-right glyph-template parser (fixed font, fixed pixel grid).
Engine T: Tesseract (digit whitelist) on an upscaled binarised crop.
Engine R: RapidOCR (used as tie-breaker / third opinion).
"""
import re
from pathlib import Path

import cv2
import numpy as np

TH = {"val": 100, "rng": 60}
GRAMMAR = [r"-?\d+", r"-?\d+", r"-?\d+\.\d\d", r"-?\d+\.\d", r"-?\d+\.\d\d", r"-?\d+\.\d"]
G_MIN_SCORE = 0.90   # minimum normalised correlation for a glyph match
G_MIN_MARGIN = 0.05  # required lead over the best differently-labelled template


def soft(im, kind):
    """Text 'ink' image (0..255). val: blueness (grey unit text vanishes); rng: darkness."""
    im = im.astype(np.int16)
    if kind == "val":
        return np.clip(im[:, :, 0] - im[:, :, 2], 0, 255).astype(np.uint8)
    return np.clip(250 - im.max(2), 0, 255).astype(np.uint8)


_T = None


def templates():
    global _T
    if _T is None:
        z = np.load(Path(__file__).with_name("templates.npz"))
        _T = {}
        for kind in ("val", "rng"):
            labs = z[f"{kind}_labels"]
            _T[kind] = [(str(l), z[f"{kind}_{i}"].astype(np.float32)) for i, l in enumerate(labs)]
    return _T


def engine_g(im, kind):
    """Return (string, min_score, min_margin) or (None, score, margin) when rejected."""
    s = soft(im, kind).astype(np.float32)
    s[s <= TH[kind] * 0.3] = 0  # suppress compression haze
    cols = (s > TH[kind]).any(0)
    xs = np.flatnonzero(cols)
    if len(xs) == 0:
        return None, 0.0, 0.0
    pos, end = xs[0], xs[-1]
    out, worst, worst_m = [], 1.0, 1.0
    W = s.shape[1]
    while pos <= end:
        best = {}
        for lab, t in templates()[kind]:
            w = t.shape[1]
            for dx in (-2, -1, 0, 1, 2):
                x0 = pos + dx
                if x0 < 0 or x0 + w > W:
                    continue
                win = s[:, x0:x0 + w]
                den = np.sqrt((win * win).sum() * (t * t).sum())
                sc = float((win * t).sum() / den) if den > 0 else 0.0
                if sc > best.get(lab, (0, 0))[0]:
                    best[lab] = (sc, x0 + w)
        rank = sorted(best.items(), key=lambda kv: -kv[1][0])
        if not rank:
            return None, 0.0, 0.0
        (lab, (sc, nxt)) = rank[0]
        margin = sc - (rank[1][1][0] if len(rank) > 1 else 0)
        worst, worst_m = min(worst, sc), min(worst_m, margin)
        if sc < G_MIN_SCORE or margin < G_MIN_MARGIN:
            return None, sc, margin
        out.append(lab)
        for j in (0, 1, 2):  # skip a <=2 px ink tail of this glyph (template may be 1-2 px narrower)
            if nxt + j >= W or not cols[nxt + j]:
                nxt += j
                break
        rest = np.flatnonzero(cols[nxt:])  # next ink column (dx search absorbs touching/overlap)
        if len(rest) == 0:
            break
        pos = nxt + rest[0]
    return "".join(out), worst, worst_m


def split_range(s):
    """'-95.7-402.3' -> ('-95.7', '402.3'); the separator is the '-' preceded by a digit."""
    m = re.fullmatch(r"(-?\d+(?:\.\d+)?)-(-?\d+(?:\.\d+)?)", s or "")
    return (m.group(1), m.group(2)) if m else (None, None)


def _prep(im, kind, scale):
    s = 255 - soft(im, kind)
    xs = np.flatnonzero((s < 255 - TH[kind]).any(0))
    if len(xs):
        s = s[:, max(xs[0] - 12, 0):xs[-1] + 13]
    s = cv2.normalize(s, None, 0, 255, cv2.NORM_MINMAX)
    s = cv2.resize(s, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
    return cv2.copyMakeBorder(s, 20, 20, 20, 20, cv2.BORDER_CONSTANT, value=255)


def engine_t(im, kind, scale=None):
    import pytesseract
    pytesseract.pytesseract.tesseract_cmd = r"C:\Program Files\Tesseract-OCR\tesseract.exe"
    img = _prep(im, kind, scale or (2 if kind == "val" else 4))
    cfg = "--psm 7 -c tessedit_char_whitelist=0123456789.- "
    txt = pytesseract.image_to_string(img, config=cfg)
    return re.sub(r"\s+", "", txt.replace("—", "-").replace("–", "-"))


_R = None


def engine_r(im, kind):
    global _R
    if _R is None:
        from rapidocr import RapidOCR
        _R = RapidOCR()
    img = cv2.cvtColor(_prep(im, kind, 2 if kind == "val" else 3), cv2.COLOR_GRAY2BGR)
    res = _R(img, use_det=False, use_cls=False)
    txt = "".join(res.txts) if getattr(res, "txts", None) else ""
    txt = txt.replace("—", "-").replace("–", "-").replace("−", "-").replace("~", "-")
    return re.sub(r"[^0-9.\-]", "", txt)
