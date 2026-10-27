"""Shared frame access and drawing helpers for offline driver pose."""
from pathlib import Path
import atexit
import cv2
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[2]
R2 = ROOT / '01_refined/run2'
CACHE = ROOT / '_work/5_pose_correction/cache'
MODELS = ROOT / '_work/5_pose_correction/models'
LM = ['nose', 'left_ear', 'right_ear', 'left_shoulder', 'right_shoulder', 'left_elbow', 'right_elbow', 'left_wrist', 'right_wrist', 'left_hip', 'right_hip']
W, H = 1280, 800
MP_IDX = [0, 7, 8, 11, 12, 13, 14, 15, 16, 23, 24]
COCO_IDX = [0, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]
BONES = [('nose', 'left_ear'), ('nose', 'right_ear'), ('left_shoulder', 'right_shoulder'), ('left_shoulder', 'left_elbow'), ('left_elbow', 'left_wrist'), ('right_shoulder', 'right_elbow'), ('right_elbow', 'right_wrist'), ('left_shoulder', 'left_hip'), ('right_shoulder', 'right_hip'), ('left_hip', 'right_hip')]
_caps = {}
_frames = None


def load_camera_frames():
    global _frames
    if _frames is None:
        _frames = pd.read_csv(R2 / 'camera_frames.csv', dtype={'t_utc_ns': 'int64', 'capture_monotonic_ns': 'int64'})
        for _, group in _frames.groupby('segment_id', sort=True):
            if group.segment_file.nunique() != 1 or not np.array_equal(group.frame_in_segment.to_numpy(), np.arange(len(group))):
                raise ValueError('Camera frame manifest has inconsistent segment order')
    return _frames.copy()


def iter_segment_frames(segment_file):
    expected = int((load_camera_frames().segment_file == segment_file).sum())
    if not expected:
        raise ValueError(f'Unknown segment: {segment_file}')
    cap = cv2.VideoCapture(str(ROOT / segment_file))
    count = 0
    try:
        if not cap.isOpened():
            raise OSError(f'Cannot open {segment_file}')
        while True:
            ok, image = cap.read()
            if not ok:
                break
            if count >= expected:
                raise ValueError(f'{segment_file}: decoded more than {expected} frames')
            if image.shape[:2] != (H, W):
                raise ValueError(f'Unexpected frame shape: {image.shape}')
            count += 1
            yield image
        if count != expected:
            raise ValueError(f'{segment_file}: decoded {count}, expected {expected}')
    finally:
        cap.release()


def read_frame(segment_file, frame_in_segment):
    count = int((load_camera_frames().segment_file == segment_file).sum())
    idx = int(frame_in_segment)
    if not 0 <= idx < count:
        raise IndexError((segment_file, idx))
    if segment_file not in _caps:
        _caps[segment_file] = cv2.VideoCapture(str(ROOT / segment_file))
    cap = _caps[segment_file]
    if not cap.set(cv2.CAP_PROP_POS_FRAMES, idx):
        raise OSError(f'Cannot seek {segment_file}:{idx}')
    ok, image = cap.read()
    if not ok:
        raise OSError(f'Cannot decode {segment_file}:{idx}')
    return image


@atexit.register
def close_captures():
    for cap in _caps.values():
        cap.release()


def color(name):
    return (255, 160, 0) if name.startswith('left') else (0, 90, 255) if name.startswith('right') else (0, 255, 0)


def draw(image, pts, vis=None, label=True, thr=0.5, scale=1.0):
    image = image.copy()
    for a, b in BONES:
        if a in pts and b in pts and all(np.isfinite(pts[k]).all() for k in (a, b)):
            low = vis is not None and (vis.get(a, 1) < thr or vis.get(b, 1) < thr)
            cv2.line(image, tuple(int(v) for v in pts[a]), tuple(int(v) for v in pts[b]), color(b), 1 if low else 3, cv2.LINE_AA)
    for name in LM:
        if name not in pts or not np.isfinite(pts[name]).all():
            continue
        q = tuple(int(v) for v in pts[name])
        low = vis is not None and vis.get(name, 1) < thr
        cv2.circle(image, q, 7, color(name), 2 if low else -1, cv2.LINE_AA)
        cv2.circle(image, q, 7, (255, 255, 255), 1, cv2.LINE_AA)
        if label:
            text = name[0].upper() + name.split('_')[-1][:2] if '_' in name else 'nose'
            for c, thickness in [((0, 0, 0), 3), ((255, 255, 255), 1)]:
                cv2.putText(image, text, (q[0] + 8, q[1] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.55 * scale, c, thickness, cv2.LINE_AA)
    return image


def sheet(panels, cols, w=640):
    if not panels or cols < 1:
        raise ValueError('Nonempty panels and positive cols required')
    images = [cv2.resize(p, (w, int(p.shape[0] * w / p.shape[1]))) for p in panels]
    if len({p.shape for p in images}) != 1:
        raise ValueError('Panels must have the same aspect ratio')
    while len(images) % cols:
        images.append(np.zeros_like(images[0]))
    return np.vstack([np.hstack(images[i:i + cols]) for i in range(0, len(images), cols)])
