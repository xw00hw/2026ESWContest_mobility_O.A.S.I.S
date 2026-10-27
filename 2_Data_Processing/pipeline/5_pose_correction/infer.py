"""Offline VIDEO MediaPipe and fixed-box CUDA RTM inference."""
import os
os.environ.setdefault('OMP_NUM_THREADS', '1')
os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
import argparse
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor, as_completed
import hashlib
from itertools import islice
from importlib.metadata import version
import json
import time
import numpy as np
import cv2
from common import ROOT, CACHE, MODELS, LM, MP_IDX, COCO_IDX, W, H, load_camera_frames, iter_segment_frames

MP_VARIANTS = ['mp_full_raw', 'mp_heavy_raw', 'mp_heavy_clahe', 'mp_heavy_roi']
RTM_VARIANTS = ['rtm_x', 'rtm_x_flip', 'rtm_x_bbox2', 'rtmw_x']
VARIANTS = MP_VARIANTS + RTM_VARIANTS


def log(message):
    print(message, flush=True)


def model_paths():
    paths = {'full': MODELS / 'pose_landmarker_full.task', 'heavy': MODELS / 'pose_landmarker_heavy.task'}
    for key, pattern in [('rtm', 'rtmpose-x*/**/end2end.onnx'), ('rtmw', 'rtmw-dw-x-l*/end2end.onnx')]:
        found = sorted(MODELS.glob(pattern))
        if len(found) != 1:
            raise ValueError(f'Expected one {key} model, got {found}')
        paths[key] = found[0]
    for path in paths.values():
        if not path.is_file():
            raise FileNotFoundError(path)
    return paths


def output_path(variant, segment):
    return CACHE / variant / f'seg_{segment:03d}.npz'


def allocate(rows):
    n = len(rows)
    return {'xy': np.full((n, len(LM), 2), np.nan, np.float32),
            'score': np.full((n, len(LM)), np.nan, np.float32),
            'pres': np.full((n, len(LM)), np.nan, np.float32),
            'z': np.full((n, len(LM)), np.nan, np.float32),
            'detected': np.zeros(n, np.uint8),
            'frame_in_segment': rows.frame_in_segment.to_numpy(dtype=np.int32)}


def save(variant, segment, data):
    path = output_path(variant, segment)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix('.npz.tmp')
    try:
        with tmp.open('wb') as f:
            np.savez(f, **data)
        tmp.replace(path)
    finally:
        tmp.unlink(missing_ok=True)


def mp_job(segment, variant):
    import mediapipe as mp
    from mediapipe.tasks import python as mpp
    from mediapipe.tasks.python import vision
    cv2.setNumThreads(1)
    np.random.seed(0)
    started = time.perf_counter()
    rows = load_camera_frames().query('segment_id == @segment').reset_index(drop=True)
    data = allocate(rows)
    model = model_paths()['full' if variant == 'mp_full_raw' else 'heavy']
    options = vision.PoseLandmarkerOptions(
        base_options=mpp.BaseOptions(model_asset_path=str(model), delegate=mpp.BaseOptions.Delegate.CPU),
        running_mode=vision.RunningMode.VIDEO, num_poses=1,
        min_pose_detection_confidence=0.3, min_pose_presence_confidence=0.3, min_tracking_confidence=0.3)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    previous = -1
    first = int(rows.capture_monotonic_ns.iloc[0])
    with vision.PoseLandmarker.create_from_options(options) as landmarker:
        for i, image in enumerate(iter_segment_frames(rows.segment_file.iloc[0])):
            if variant == 'mp_heavy_clahe':
                image = cv2.cvtColor(clahe.apply(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)), cv2.COLOR_GRAY2BGR)
            elif variant == 'mp_heavy_roi':
                image = cv2.resize(image[:, :960], (1200, 1000), interpolation=cv2.INTER_CUBIC)
            timestamp = max(previous + 1, round((int(rows.capture_monotonic_ns.iloc[i]) - first) / 1e6))
            previous = timestamp
            rgb = np.ascontiguousarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
            result = landmarker.detect_for_video(mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb), timestamp)
            if result.pose_landmarks:
                landmarks = [result.pose_landmarks[0][j] for j in MP_IDX]
                # ROI normalized coordinates map directly back to its original extent.
                width, height = (960, H) if variant == 'mp_heavy_roi' else (W, H)
                data['xy'][i] = [(p.x * width, p.y * height) for p in landmarks]
                for key, attribute in [('score', 'visibility'), ('pres', 'presence'), ('z', 'z')]:
                    data[key][i] = [getattr(p, attribute) for p in landmarks]
                data['detected'][i] = 1
            if (i + 1) % 250 == 0:
                log(f'{variant} seg {segment:03d}: {i + 1}/{len(rows)}')
    save(variant, segment, data)
    elapsed = time.perf_counter() - started
    log(f'DONE {variant} seg {segment:03d}: {len(rows)} frames, {elapsed:.1f}s')
    return {'variant': variant, 'segment': segment, 'frames': len(rows), 'seconds': elapsed}


def make_rtm(path):
    import onnxruntime as ort
    from rtmlib import RTMPose
    ort.set_default_logger_severity(3)
    ort.preload_dlls()
    pose = RTMPose(str(path), model_input_size=(288, 384), backend='onnxruntime', device='cpu')
    options = ort.SessionOptions()
    options.log_severity_level = 3
    options.intra_op_num_threads = 1
    options.inter_op_num_threads = 1
    pose.session = ort.InferenceSession(str(path), sess_options=options, providers=[
        ('CUDAExecutionProvider', {'device_id': 0, 'cudnn_conv_algo_search': 'DEFAULT'}), 'CPUExecutionProvider'])
    if pose.session.get_providers()[0] != 'CUDAExecutionProvider':
        raise RuntimeError('CUDA provider unavailable')
    log(f'CUDA session: {path.name}, {pose.session.get_provider_options()}')
    return pose


def gpu_jobs(segments, variants, force):
    cv2.setNumThreads(1)
    np.random.seed(0)
    pending = {s: [v for v in variants if force or not output_path(v, s).exists()] for s in segments}
    if not any(pending.values()):
        return []
    paths = model_paths()
    poses = {}
    if any(v != 'rtmw_x' for values in pending.values() for v in values):
        poses['rtm'] = make_rtm(paths['rtm'])
    if any('rtmw_x' in values for values in pending.values()):
        poses['rtmw'] = make_rtm(paths['rtmw'])
    records = []
    swap = [0, 2, 1, 4, 3, 6, 5, 8, 7, 10, 9]
    for segment in segments:
        active = pending[segment]
        if not active:
            continue
        rows = load_camera_frames().query('segment_id == @segment').reset_index(drop=True)
        arrays = {v: allocate(rows) for v in active}
        seconds = {v: 0.0 for v in active}
        segment_start = time.perf_counter()
        frames = iter_segment_frames(rows.segment_file.iloc[0])
        offset = 0
        while images := list(islice(frames, 64)):
            for variant in active:
                start = time.perf_counter()
                pose = poses['rtmw' if variant == 'rtmw_x' else 'rtm']
                box = [0, 40, 1060, 800]
                if variant == 'rtm_x_flip':
                    box = [W - 1 - 1060, 40, W - 1, 800]
                elif variant == 'rtm_x_bbox2':
                    box = [0, 0, W, H]
                prepared = [pose.preprocess(cv2.flip(image, 1) if variant == 'rtm_x_flip' else image, box) for image in images]
                inputs = np.ascontiguousarray(np.stack([item[0].transpose(2, 0, 1) for item in prepared]), dtype=np.float32)
                outputs = pose.session.run(None, {pose.session.get_inputs()[0].name: inputs})
                for j, (_, center, scale) in enumerate(prepared):
                    points, scores = pose.postprocess([out[j:j + 1] for out in outputs], center, scale)
                    xy, score = points[0, COCO_IDX].copy(), scores[0, COCO_IDX].copy()
                    if variant == 'rtm_x_flip':
                        xy[:, 0] = W - 1 - xy[:, 0]
                        xy, score = xy[swap], score[swap]
                    valid = np.isfinite(xy).all() and np.isfinite(score).all() and (score > 0).any()
                    if valid:
                        arrays[variant]['xy'][offset + j] = xy
                        arrays[variant]['score'][offset + j] = score
                        arrays[variant]['detected'][offset + j] = 1
                elapsed = time.perf_counter() - start
                seconds[variant] += elapsed
                if offset == 0:
                    log(f'{variant} seg {segment:03d}: first batch {len(images)} frames, {elapsed:.3f}s')
            offset += len(images)
            if offset % 64 == 0 or offset == len(rows):
                log(f'RTM seg {segment:03d}: {offset}/{len(rows)}')
        for variant in active:
            save(variant, segment, arrays[variant])
            records.append({'variant': variant, 'segment': segment, 'frames': len(rows), 'seconds': seconds[variant]})
        log(f'DONE RTM seg {segment:03d}: {len(rows)} frames, {time.perf_counter() - segment_start:.1f}s')
    return records


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--force', action='store_true')
    parser.add_argument('--variants', default=','.join(VARIANTS))
    parser.add_argument('--segments')
    parser.add_argument('--workers', type=int, default=12)
    args = parser.parse_args()
    variants = args.variants.split(',')
    available = sorted(int(s) for s in load_camera_frames().segment_id.unique())
    segments = available if args.segments is None else sorted(set(int(s) for s in args.segments.split(',')))
    if args.workers < 1 or not variants or len(set(variants)) != len(variants) or any(v not in VARIANTS for v in variants) or any(s not in available for s in segments):
        parser.error('Invalid variants, segments, or workers')
    paths = model_paths()
    started = time.perf_counter()
    jobs = [(s, v) for s in segments for v in variants if v in MP_VARIANTS and (args.force or not output_path(v, s).exists())]
    records = []
    executor_kind = 'ProcessPoolExecutor'
    try:
        pool = ProcessPoolExecutor(max_workers=args.workers)
    except PermissionError as error:
        if getattr(error, 'winerror', None) != 5:
            raise
        log('WARNING: sandbox denies multiprocessing pipe; using ThreadPoolExecutor')
        executor_kind = 'ThreadPoolExecutor (WinError 5 fallback)'
        pool = ThreadPoolExecutor(max_workers=args.workers)
    with pool:
        futures = [pool.submit(mp_job, s, v) for s, v in jobs]
        records.extend(gpu_jobs(segments, [v for v in variants if v in RTM_VARIANTS], args.force))
        for future in as_completed(futures):
            records.append(future.result())
    meta_path = CACHE / 'infer_meta.json'
    prior = json.loads(meta_path.read_text(encoding='utf-8')) if meta_path.exists() else {}
    retained = {(r['variant'], r['segment']): r for r in prior.get('tasks', [])}
    retained.update({(r['variant'], r['segment']): r for r in records})
    tasks = [retained[k] for k in sorted(retained)]
    totals = {v: {'seconds': sum(r['seconds'] for r in tasks if r['variant'] == v), 'frames': sum(r['frames'] for r in tasks if r['variant'] == v)} for v in VARIANTS}
    meta = {'packages': {p: version(p) for p in ['numpy', 'opencv-python', 'mediapipe', 'onnxruntime-gpu', 'rtmlib', 'pandas']},
            'models': {k: {'path': str(p.relative_to(ROOT)), 'sha256': hashlib.sha256(p.read_bytes()).hexdigest()} for k, p in paths.items()},
            'tasks': tasks, 'variants': totals, 'last_run': {'segments': segments, 'variants': variants, 'wall_seconds': time.perf_counter() - started, 'computed_tasks': len(records), 'mp_executor': executor_kind},
            'rtm_batch_size': 64, 'timing_definition': 'MP task wall time; RTM inference time excludes shared decode and model setup'}
    CACHE.mkdir(parents=True, exist_ok=True)
    tmp = meta_path.with_suffix('.json.tmp')
    tmp.write_text(json.dumps(meta, indent=2, ensure_ascii=False) + '\n', encoding='utf-8', newline='\n')
    tmp.replace(meta_path)
    log(f'COMPLETE: {len(records)} computed tasks, {meta["last_run"]["wall_seconds"]:.1f}s')


if __name__ == '__main__':
    main()
