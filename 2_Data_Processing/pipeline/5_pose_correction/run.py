r"""Offline Part E pipeline. Reproduce C(.venv) -> D(.venv) -> E(.venv_pose).

E runs inference, correction, QC, then D again in the main venv to add common
time columns and rebuild synced tables, then checks handoff.
Use --no-handoff while the external QA team owns segment handoff.
--skip-d leaves E outputs without D time columns until D is run separately.

Prerequisites (already installed; never install or download during this run):
uv venv .venv_pose --python 3.12
uv pip install --python .venv_pose\Scripts\python.exe mediapipe==1.1.0 rtmlib==0.0.16 "onnxruntime-gpu[cuda,cudnn]==1.31.0" numpy pandas scipy matplotlib opencv-python
The main venv must also exist, and C and D outputs must already be present.
infer.py calls ort.preload_dlls() before creating its CUDA session.
Four local model files under _work/5_pose_correction/models (hashes: cache/infer_meta.json):
full: https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_full/float16/1/pose_landmarker_full.task
sha256: 4eaa5eb7a98365221087693fcc286334cf0858e2eb6e15b506aa4a7ecdcec4ad
heavy: https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_heavy/float16/1/pose_landmarker_heavy.task
sha256: 64437af838a65d18e5ba7a0d39b465540069bc8aae8308de3e318aad31fcbc7b
RTMPose-x: https://download.openmmlab.com/mmpose/v1/projects/rtmposev1/onnx_sdk/rtmpose-x_simcc-body7_pt-body7_700e-384x288-71d7b7e9_20230629.zip
sha256 (extracted end2end.onnx): df0c0fa91e9870b1515dcaff741fd76cc753dcfb12786862f61b243bae81cd52
RTMW-x: https://download.openmmlab.com/mmpose/v1/projects/rtmw/onnx_sdk/rtmw-dw-x-l_simcc-cocktail14_270e-384x288_20231122.zip
sha256 (extracted end2end.onnx): bd033156e5104c4f5d2edfe0453e02661e30a2f3da453ec93c8764d561b83054

CSV outputs use Pi wall capture time first; correction always rewrites them
without D columns. D is therefore enabled by default. Elapsed times go only to
stdout; this runner never writes runtime.json.
"""
import argparse
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]


def run_stage(name, python, script, args=()):
    started = time.perf_counter()
    print(f"START {name}", flush=True)
    env = {**os.environ, "PYTHONDONTWRITEBYTECODE": "1"}
    # uv's Python 3.12 launcher exports PYTHONHOME; D uses Python 3.13.
    for key in ("PYTHONHOME", "PYTHONPATH", "__PYVENV_LAUNCHER__"):
        env.pop(key, None)
    result = subprocess.run([str(python), "-B", "-u", str(ROOT / script), *args], cwd=ROOT,
                            env=env)
    print(f"END {name}: exit={result.returncode}, elapsed={time.perf_counter()-started:.2f}s", flush=True)
    if result.returncode:
        raise SystemExit(result.returncode)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--force", action="store_true", help="Recompute inference caches")
    parser.add_argument("--skip-d", action="store_true", help="Leave D time columns and integration for a separate D run")
    parser.add_argument("--no-handoff", action="store_true", help="Do not write or check segment handoff")
    args = parser.parse_args()
    from infer import VARIANTS, output_path
    from common import load_camera_frames
    segments = sorted(int(s) for s in load_camera_frames().segment_id.unique())
    if args.force or any(not output_path(v, s).is_file() for v in VARIANTS for s in segments):
        run_stage("inference", sys.executable, "pipeline/5_pose_correction/infer.py", ["--force"] if args.force else [])
    else:
        print("SKIP inference: all variant/segment caches present", flush=True)
    run_stage("person detection", sys.executable, "pipeline/5_pose_correction/detect_people.py", ["--force"] if args.force else [])
    run_stage("correction", sys.executable, "pipeline/5_pose_correction/correct.py", [] if args.no_handoff else ["--handoff", "all"])
    run_stage("QC", sys.executable, "pipeline/5_pose_correction/qc.py")
    if not args.skip_d:
        run_stage("D integration", ROOT.parent / ".venv/Scripts/python.exe", "pipeline/4_time_sync/run.py")
    if not args.no_handoff:
        run_stage("handoff check", sys.executable, "pipeline/5_pose_correction/check_handoff.py")
    print("Part E COMPLETE", flush=True)


if __name__ == "__main__":
    main()
