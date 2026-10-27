"""Stage 3: combine engines, run consistency checks, write run2 CSVs, stats json and plots.

Empirically determined behaviour of the grey range line (verified on every frame, see stats):
  max  == running maximum of the displayed current value (starting from the max shown in frame 0)
  min  == the most recent current value whose TEXT starts with '-' (incl. '-0.00'); an Infocar quirk,
          it is NOT a running minimum.  Both laws are used as checksums between two independent crops.
"""
import json
import pickle
import re

import cv2
import matplotlib
import numpy as np
import pandas as pd

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from common import CACHE, DECIMALS, OUT2, QC, SIGNALS  # noqa: E402
from ocr import GRAMMAR, split_range  # noqa: E402

SPIKE = [800, 8, 0.15, 60, 0.15, 8]  # per-refresh jump regarded as "large" (listed for audit, not an error)


def law_check(vtxt, mn, mx):
    """Return bool arrays (max_ok, min_ok) per frame for the two range laws. Inputs are per-frame lists of str/None."""
    n = len(vtxt)
    max_ok = np.zeros(n, bool); min_ok = np.zeros(n, bool)
    run_max = None; last_neg = None
    for i in range(n):
        v, a, b = vtxt[i], mn[i], mx[i]
        if v is None or a is None or b is None:
            run_max = last_neg = None  # chain broken: re-anchor on the next readable frame
            continue
        fv = float(v)
        if run_max is None:
            run_max, last_neg = float(b), a  # anchor (first frame: history before the recording is unknown)
        run_max = max(run_max, fv)
        if v.startswith("-"):
            last_neg = v
        max_ok[i] = float(b) == run_max
        min_ok[i] = a == last_neg
        if not max_ok[i]:
            run_max = float(b)
        if not min_ok[i]:
            last_neg = a
    return max_ok, min_ok


def per_frame(d, r, engine):
    """Per-frame texts of value/min/max for one engine ('g','t','r'); None where grammar fails."""
    out = []
    for k in range(6):
        vt = [x[engine] if x[engine] and re.fullmatch(GRAMMAR[k], x[engine]) else None for x in r["val"][k]]
        rt = []
        for x in r["rng"][k]:
            a, b = split_range(x[engine])
            ok = a is not None and re.fullmatch(GRAMMAR[k], a) and re.fullmatch(GRAMMAR[k], b)
            rt.append((a, b) if ok else (None, None))
        vi, ri = d["val_id"][:, k], d["rng_id"][:, k]
        out.append(([vt[i] for i in vi], [rt[i][0] for i in ri], [rt[i][1] for i in ri]))
    return out


def violations(pf):
    res = {}
    for k, (v, a, b) in enumerate(pf):
        mxo, mno = law_check(v, a, b)
        unread = sum(x is None for x in v) + sum(x is None for x in a)
        inr = sum(1 for x, lo, hi in zip(v, a, b) if x and lo and hi and not (min(float(lo), float(x)) <= float(x) <= float(hi)))
        res[SIGNALS[k]] = {"frames_unreadable_value_or_range": int(unread), "value_above_max_frames": int(inr),
                           "max_law_violation_frames": int((~mxo).sum()), "min_law_violation_frames": int((~mno).sum())}
    return res


def num(s):
    return "" if s is None else ("0" + s[2:] if re.fullmatch(r"-0(\.0+)?", s) else s)


def main():
    d = pickle.load(open(CACHE / "stage1.pkl", "rb"))
    r = pickle.load(open(CACHE / "stage2.pkl", "rb"))
    t = d["pts"] / 90000.0
    N = len(t)
    OUT2.mkdir(parents=True, exist_ok=True); (QC / "plots").mkdir(parents=True, exist_ok=True)
    (QC / "flagged").mkdir(exist_ok=True)
    layout_bad = d["title_diff"].max(1) > 8  # static titles moved/covered (mean abs gray diff)

    pf = {e: per_frame(d, r, e) for e in "gtr"}
    stats = {"frames_total": int(N)}
    dt = np.diff(t)
    stats["pts"] = {"first_s": float(t[0]), "last_s": float(t[-1]), "strictly_increasing": bool((dt > 0).all()),
                    "interval_min_s": float(dt.min()), "interval_median_s": float(np.median(dt)),
                    "interval_max_s": float(dt.max()), "intervals_gt_0.2s": int((dt > 0.2).sum()),
                    "mean_fps": float((N - 1) / (t[-1] - t[0]))}
    stats["layout"] = {"title_roi_mean_abs_diff_max_per_tile": d["title_diff"].max(0).round(2).tolist(),
                       "frames_layout_deviation": int(layout_bad.sum()),
                       "whole_frame_diff_max": float(d["frame_diff"].max())}
    stats["constraint_violations_if_single_engine"] = {
        "tesseract_only": violations(pf["t"]), "rapidocr_only": violations(pf["r"]), "template_only": violations(pf["g"])}

    cols = {"frame_idx": np.arange(N), "t_video_s": [f"{x:.6f}" for x in t]}
    text = {"frame_idx": np.arange(N)}
    okc, rngc, changes, per_tile, flagged = {}, {}, [], {}, []
    for k, sig in enumerate(SIGNALS):
        vi, ri = d["val_id"][:, k], d["rng_id"][:, k]
        V, R = r["val"][k], r["rng"][k]
        v, a, b = pf["g"][k]
        # engine agreement per unique crop: G (templates) must equal R (RapidOCR); T (Tesseract) is reported
        v_agree = np.array([x["g"] is not None and x["g"] == x["r"] for x in V])[vi]
        r_agree = np.array([x["g"] is not None and x["g"] == x["r"] for x in R])[ri]
        mxo, mno = law_check(v, a, b)
        readable = np.array([x is not None for x in v])
        rreadable = np.array([x is not None for x in a])
        v_ok = readable & v_agree & mxo & mno & ~layout_bad
        r_ok = rreadable & r_agree & mxo & mno & ~layout_bad
        cols[sig] = [num(x) for x in v]
        okc[sig + "_ok"] = v_ok.astype(int)
        rngc[sig + "_min"] = [num(x) for x in a]; rngc[sig + "_max"] = [num(x) for x in b]
        rngc[sig + "_range_ok"] = r_ok.astype(int)
        text[sig + "_text"] = [x or "" for x in v]
        text[sig + "_range_text"] = [f"{x} - {y}" if x else "" for x, y in zip(a, b)]
        fv = np.array([float(x) if x else np.nan for x in v])
        # compare normalised values: "-0.00" -> "0.00" is a display sign flip, not a value change
        nv = cols[sig]
        ch = np.flatnonzero(np.array([nv[i] != nv[i - 1] for i in range(1, N)])) + 1
        for i in ch:
            changes.append((sig, int(i), f"{t[i]:.6f}", v[i - 1] or "", v[i] or ""))
        gaps = np.diff(t[ch])
        jumps = np.abs(np.diff(fv[np.r_[0, ch]]))
        spike = [int(ch[j]) for j in range(len(ch) - 1)
                 if abs(fv[ch[j]] - fv[ch[j] - 1]) > SPIKE[k] and abs(fv[ch[j + 1]] - fv[ch[j] - 1]) < SPIKE[k] / 4]
        per_tile[sig] = {
            "unique_value_crops_ocrd": len(V), "frames_value_inherited": int(N - len(V)),
            "unique_range_crops_ocrd": len(R), "frames_range_inherited": int(N - len(R)),
            "value_G_rejects": sum(x["g"] is None for x in V),
            "value_G_min_score": float(min(x["g_score"] for x in V)), "value_G_min_margin": float(min(x["g_margin"] for x in V)),
            "value_G_eq_R": sum(x["g"] == x["r"] for x in V), "value_G_eq_T": sum(x["g"] == x["t"] for x in V),
            "value_G_ne_R": sum(x["g"] != x["r"] for x in V), "value_G_ne_T": sum(x["g"] != x["t"] for x in V),
            "range_G_eq_R": sum(x["g"] == x["r"] for x in R), "range_G_eq_T": sum(x["g"] == x["t"] for x in R),
            "range_G_ne_R": sum(x["g"] != x["r"] for x in R), "range_G_ne_T": sum(x["g"] != x["t"] for x in R),
            "grammar_fail_frames": int((~readable).sum()),
            "max_law_violation_frames": int((~mxo).sum()), "min_law_violation_frames": int((~mno).sum()),
            "flagged_value_frames": int((~v_ok).sum()), "flagged_range_frames": int((~r_ok).sum()),
            "value_min": float(np.nanmin(fv)), "value_max": float(np.nanmax(fv)),
            "n_negative_text_changes_verified_by_min_line": int(sum(1 for i in ch if v[i].startswith("-"))),
            "n_new_max_verified_by_max_line": int((np.diff(np.array([float(x) for x in b])) > 0).sum()),
            "value_changes": int(len(ch)), "change_interval_median_s": float(np.median(gaps)),
            "change_interval_p05_p95_s": np.percentile(gaps, [5, 95]).round(3).tolist(),
            "longest_hold_s": float(gaps.max()), "max_abs_step_per_change": float(jumps.max()),
            "p99_abs_step_per_change": float(np.percentile(jumps, 99)),
            "isolated_spike_candidates(frame_idx)": spike,
            "range_change_not_on_value_change_frame": int(len(set(d["rng_first"][k][1:]) - set(d["val_first"][k][1:]))),
        }
        for i in np.flatnonzero(~v_ok)[:100]:
            flagged.append((int(i), k))
    # display refresh events: frames on which at least one tile's value text changed
    chf = np.array(sorted({c[1] for c in changes}))
    refresh = np.zeros(N, int); refresh[chf] = 1
    gi = np.diff(t[chf])
    ntiles = pd.Series([c[1] for c in changes]).value_counts()
    bright = d["dot"][:, :, 2] < 115
    on = np.flatnonzero(bright[1:, 0] & ~bright[:-1, 0]) + 1
    j = np.clip(np.searchsorted(on, chf), 1, len(on) - 1)
    stats["update_behaviour"] = {
        "refresh_frames(any value changed)": int(len(chf)),
        "refresh_interval_median_s": float(np.median(gi)), "refresh_interval_p05_p25_p75_p95_s": np.percentile(gi, [5, 25, 75, 95]).round(3).tolist(),
        "refresh_interval_max_s": float(gi.max()), "estimated_refresh_rate_hz": float(1 / np.median(gi)),
        "fraction_of_value_changes_on_frame_shared_with_another_tile": float((ntiles[[c[1] for c in changes]].values > 1).mean()),
        "tiles_changed_per_refresh_mean": float(ntiles.mean()),
        "frames_without_any_roi_change_fraction": float(1 - len(set(np.concatenate([np.array(x) for x in d["val_first"] + d["rng_first"]]))) / N),
        "dot_blink": {"bright_fraction": float(bright[:, 0].mean()), "blink_on_events_tile0": int(len(on)),
                      "blink_interval_median_s": float(np.median(np.diff(t[on]))),
                      "blink_interval_p05_p95_s": np.percentile(np.diff(t[on]), [5, 95]).round(3).tolist(),
                      "refresh_delay_after_previous_blink_p05_p50_p95_s": np.percentile(t[chf] - t[on[j - 1]], [5, 50, 95]).round(3).tolist(),
                      "refresh_on_blink_frame_fraction": float(np.isin(chf, on).mean())},
    }
    stats["per_signal"] = per_tile
    stats["flagged_total_value_cells"] = int(sum(p["flagged_value_frames"] for p in per_tile.values()))
    stats["flagged_total_range_cells"] = int(sum(p["flagged_range_frames"] for p in per_tile.values()))

    df = pd.DataFrame({**cols, **okc, **rngc, "any_value_changed": refresh, "flag_layout": layout_bad.astype(int)})
    assert len(df) == N and (np.diff(d["pts"]) > 0).all()
    df.to_csv(OUT2 / "obd.csv", index=False, lineterminator="\n")
    pd.DataFrame(text).to_csv(OUT2 / "obd_text.csv", index=False, lineterminator="\n")
    pd.DataFrame(changes, columns=["signal", "frame_idx", "t_video_s", "old", "new"]).sort_values(
        ["frame_idx", "signal"]).to_csv(OUT2 / "obd_changes.csv", index=False, lineterminator="\n")
    (QC / "run2_stats.json").write_text(json.dumps(stats, indent=1, ensure_ascii=False), encoding="utf-8")
    for i, k in flagged[:500]:
        cv2.imwrite(str(QC / "flagged" / f"f{i}_t{k}.png"),
                    cv2.imdecode(np.frombuffer(d["val_crops"][k][d["val_id"][i, k]], np.uint8), 1))

    # plots
    num_df = df[SIGNALS].apply(pd.to_numeric)
    for name, (lo, hi) in {"all": (0, t[-1]), "zoom_300_360s": (300, 360)}.items():
        fig, ax = plt.subplots(6, 1, figsize=(16, 14), sharex=True)
        m = (t >= lo) & (t <= hi)
        for k, sig in enumerate(SIGNALS):
            ax[k].step(t[m], num_df[sig][m], where="post", lw=0.7)
            bad = m & (df[sig + "_ok"].values == 0)
            ax[k].plot(t[bad], num_df[sig][bad], "r.", ms=5)
            ax[k].set_ylabel(sig); ax[k].grid(alpha=0.3)
        ax[-1].set_xlabel("t_video_s"); ax[0].set_title(f"run2 OBD (Infocar OCR) - {name}; red = flagged")
        fig.tight_layout(); fig.savefig(QC / "plots" / f"run2_6signals_{name}.png", dpi=90); plt.close(fig)
    for sig in SIGNALS:
        fig, ax = plt.subplots(figsize=(16, 3.5))
        ax.step(t, num_df[sig], where="post", lw=0.6)
        bad = df[sig + "_ok"].values == 0
        ax.plot(t[bad], num_df[sig][bad], "r.", ms=5)
        ax.set_xlabel("t_video_s"); ax.set_ylabel(sig); ax.grid(alpha=0.3)
        fig.tight_layout(); fig.savefig(QC / "plots" / f"run2_{sig}.png", dpi=90); plt.close(fig)
    print(json.dumps({k: stats[k] for k in ("frames_total", "pts", "layout", "update_behaviour",
                                             "flagged_total_value_cells", "flagged_total_range_cells")}, indent=1))
    print(json.dumps(stats["constraint_violations_if_single_engine"]))


if __name__ == "__main__":
    main()
