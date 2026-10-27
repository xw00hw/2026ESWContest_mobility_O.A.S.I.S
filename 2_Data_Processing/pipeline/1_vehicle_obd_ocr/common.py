"""Shared constants/helpers for Part A (OBD)."""
import json
import subprocess
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
RAW_XLSX = ROOT / "00_raw" / "obd" / "run1_x431_measurements.xlsx"
RAW_MP4 = ROOT / "00_raw" / "obd" / "run2_infocar_screenrec.mp4"
OUT1 = ROOT / "01_refined" / "run1"
OUT2 = ROOT / "01_refined" / "run2"
QC = ROOT / "02_qc" / "1_vehicle_obd_ocr"
WORK = ROOT / "_work" / "1_vehicle_obd_ocr"
CACHE = WORK / "cache"

W, H = 1068, 1620
PITCH = 256  # vertical distance between tiles
SIGNALS = ["engine_rpm", "vehicle_speed_kmh", "accel_long_g",
           "steering_angle_deg", "accel_lat_g", "yaw_rate_dps"]
DECIMALS = [0, 0, 2, 1, 2, 1]
# ROIs for tile 0; add PITCH*k to y for tile k.  (y0, y1, x0, x1)
VAL_ROI = (104, 190, 200, 880)   # big blue value + grey unit
RNG_ROI = (194, 240, 300, 780)   # small grey "min - max" line
TITLE_ROI = (50, 98, 300, 780)   # static tile title (layout signature)
DOT_ROI = (128, 164, 98, 134)    # green status dot
TILE_ROI = (30, 262, 58, 1022)   # whole tile (for audit crops)


def roi(frame, r, k):
    y0, y1, x0, x1 = r
    return frame[y0 + PITCH * k:y1 + PITCH * k, x0:x1]


def probe_pts():
    """Real presentation timestamps of every video frame (seconds), in presentation order."""
    out = subprocess.run(
        ["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries",
         "frame=pts", "-of", "json", str(RAW_MP4)],
        capture_output=True, check=True).stdout
    pts = np.array([int(f["pts"]) for f in json.loads(out)["frames"]], dtype=np.int64)
    return pts  # in 1/90000 s


def iter_frames():
    """Sequentially decode every frame (no frame dropping/duplication) as BGR arrays."""
    p = subprocess.Popen(
        ["ffmpeg", "-v", "error", "-i", str(RAW_MP4), "-fps_mode", "passthrough",
         "-f", "rawvideo", "-pix_fmt", "bgr24", "-"],
        stdout=subprocess.PIPE, bufsize=W * H * 3 * 4)
    n = W * H * 3
    while True:
        buf = p.stdout.read(n)
        if len(buf) < n:
            break
        yield np.frombuffer(buf, np.uint8).reshape(H, W, 3)
    p.wait()
