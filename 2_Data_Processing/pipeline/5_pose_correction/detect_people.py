"""Person boxes (YOLOX-x HumanArt, rtmlib ONNX) on every recorded frame -> _work/5_pose_correction/cache/yolox_person/seg_XXX.npz.

Used by correct.py's `occluded_by_other_person` rule (a second person - the passenger reaching or leaning into
the camera view - covering the driver's joints). Cached like infer.py; `--force` recomputes.
npz: boxes float32 [n, 10, 5] = (x1, y1, x2, y2, score) in 1280x800 pixels, NaN padded, score >= 0.3, sorted by score.
"""
import argparse, glob, time
import numpy as np
import onnxruntime as ort
from common import ROOT, CACHE, MODELS, load_camera_frames, iter_segment_frames

OUT = CACHE / "yolox_person"
MAXB = 10

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--force", action="store_true"); ap.add_argument("--segments")
    a = ap.parse_args()
    cf = load_camera_frames(); OUT.mkdir(parents=True, exist_ok=True)
    segs = sorted(cf.segment_id.unique()) if not a.segments else [int(s) for s in a.segments.split(",")]
    todo = [s for s in segs if a.force or not (OUT / f"seg_{s:03d}.npz").exists()]
    if not todo:
        print("SKIP person detection: cache present", flush=True); return
    ort.preload_dlls()
    from rtmlib import YOLOX
    path = glob.glob(str(MODELS / "yolox_x_8xb8-300e_humanart-a39d44ed/**/end2end.onnx"), recursive=True)[0]
    det = YOLOX(path, model_input_size=(640, 640), backend="onnxruntime", device="cuda")
    det.session = ort.InferenceSession(path, providers=[("CUDAExecutionProvider", {"device_id": 0, "cudnn_conv_algo_search": "DEFAULT"}), "CPUExecutionProvider"])
    for s in todo:
        t0 = time.time(); part = cf[cf.segment_id == s]; boxes = np.full((len(part), MAXB, 5), np.nan, np.float32)
        for i, im in enumerate(iter_segment_frames(part.segment_file.iloc[0])):
            x, ratio = det.preprocess(im)
            out = det.inference(x)[0][0]            # onnx with NMS: [k, 5] = x1, y1, x2, y2, score (input scale)
            out = out[out[:, 4] >= 0.3]
            out = out[np.argsort(-out[:, 4], kind="stable")][:MAXB]
            out[:, :4] /= ratio
            boxes[i, :len(out)] = out
        tmp = OUT / f"seg_{s:03d}.tmp.npz"; np.savez(tmp, boxes=boxes); tmp.replace(OUT / f"seg_{s:03d}.npz")
        print(f"person boxes seg {s:03d}: {len(part)} frames {time.time()-t0:.1f}s", flush=True)

if __name__ == "__main__":
    main()
