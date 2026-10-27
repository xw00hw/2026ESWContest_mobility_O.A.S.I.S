"""Immutable-source bench dataset preparation; see README.md for clocks/units."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import time

import cv2
import numpy as np
import pandas as pd

from pose_core import NAMES, ROI, THRESHOLD, draw, estimate, features, load_model

SESSIONS = {
    '20261025_003530_9c4e477c': ('both_yaw_pitch_357', 357),
    '20261025_005459_3512cd6e': ('left_yaw_pitch_357', 357),
    '20261025_013054_fdbdd0b4': ('both_yaw_28x28_784', 784),
}
FEATURES = ['shoulder_tilt_deg', 'torso_lean_deg', 'head_offset_shoulder_ratio',
            'shoulder_span_px', 'torso_height_px']
STRIDE = 3


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')


def sha256(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def infer_session(source, dest, model, model_hash, decoder='opencv'):
    """Every third CFR frame; infer a repeated source capture only once."""
    marker = dest / 'pose_complete.json'
    if marker.exists():
        print(f'{source.name}: pose already complete', flush=True)
        return
    dest.mkdir(parents=True, exist_ok=True)
    chunks = dest / 'pose_checkpoints'
    chunks.mkdir(exist_ok=True)
    frames = pd.read_csv(source / 'frame_timestamps.csv')
    meta = json.loads((source / 'metadata.json').read_text(encoding='utf-8'))
    assert meta['status'] == meta['auto_scan']['status'] == 'complete'
    assert len(frames) == meta['counts']['video_frames']
    assert np.array_equal(frames.video_frame_index.to_numpy(), np.arange(len(frames)))
    selected = frames.iloc[::STRIDE]
    existing = sorted(chunks.glob('part_*.parquet'))
    saved = pd.concat([pd.read_parquet(p) for p in existing], ignore_index=True) if existing else pd.DataFrame()
    cache = {int(row['capture_index']): row for row in saved.to_dict('records')}
    for row in cache.values():
        for key,default in [('inference_backend','openvino'),('image_decoder','opencv_ffmpeg')]:
            if key not in row or pd.isna(row[key]):
                row[key] = default
    old_count = len(saved)
    capture = cv2.VideoCapture(str(source / 'camera.avi'))
    assert capture.isOpened()
    assert int(capture.get(cv2.CAP_PROP_FRAME_COUNT)) == len(frames)
    if decoder == 'jpeg':
        assert meta['video']['codec'] == 'MJPG'
        assert capture.set(cv2.CAP_PROP_FORMAT, -1), 'FFmpeg raw packet mode unavailable'
    import imageio_ffmpeg
    video = dest / 'pose_overlay.working.mp4'
    command = [imageio_ffmpeg.get_ffmpeg_exe(), '-hide_banner', '-loglevel', 'error', '-y',
               '-f', 'rawvideo', '-pix_fmt', 'bgr24', '-s', f'{ROI[2]-ROI[0]}x{ROI[3]-ROI[1]}',
               '-r', '10', '-i', '-', '-an', '-c:v', 'libx264', '-preset', 'veryfast',
               '-crf', '23', '-threads', '1', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(video)]
    encoder = subprocess.Popen(command, stdin=subprocess.PIPE)
    rows, pending = [], []
    start = time.monotonic()
    next_frame = 0
    try:
        for sample_i, frame_row in enumerate(selected.to_dict('records')):
            index = int(frame_row['video_frame_index'])
            while next_frame <= index:
                if not capture.grab():
                    raise RuntimeError(f'Video decode/grab failed at {next_frame}')
                next_frame += 1
            ok, image = capture.retrieve()
            if not ok:
                raise RuntimeError(f'Video decode failed at {index}')
            if decoder == 'jpeg':
                image = cv2.imdecode(image, cv2.IMREAD_COLOR)
                if image is None:
                    raise RuntimeError(f'JPEG packet decode failed at {index}')
            capture_id = int(frame_row['capture_index'])
            record = cache.get(capture_id)
            if record is None:
                xy, score, valid, target = estimate(model, image)
                record = {**frame_row, 'target_valid': target, 'valid_upper_joints': int(valid[:13].sum()),
                          'inference_backend': getattr(model, 'inference_backend', model.backend),
                          'image_decoder': 'opencv_libjpeg' if decoder == 'jpeg' else 'opencv_ffmpeg',
                          **features(xy, valid)}
                for k, name in enumerate(NAMES):
                    record.update({f'{name}_x_px': float(xy[k, 0]), f'{name}_y_px': float(xy[k, 1]),
                                   f'{name}_score': float(score[k]), f'{name}_valid': bool(valid[k])})
                cache[capture_id] = record
            else:
                xy = np.array([[record[f'{k}_x_px'], record[f'{k}_y_px']] for k in NAMES])
                score = np.array([record[f'{k}_score'] for k in NAMES])
                valid = np.array([record[f'{k}_valid'] for k in NAMES])
            current = {**record, **frame_row, 'pose_sample_index': sample_i,
                       'overlay_time_s': sample_i / 10,
                       'source_session_s': (int(frame_row['host_monotonic_ns']) - meta['origin_monotonic_ns']) / 1e9}
            rows.append(current)
            if sample_i >= old_count:
                pending.append(current)
            overlay = draw(image, xy, score, valid, labels=(source.name,
                f"video {index/30:.1f}s | capture {current['source_session_s']:.1f}s",
                'RTMPose-m / 10 Hz / 2D upper body'))
            encoder.stdin.write(overlay.tobytes())
            if pending and (len(pending) >= 1000 or sample_i == len(selected)-1):
                pd.DataFrame(pending).to_parquet(chunks / f'part_{sample_i+1-len(pending):06d}.parquet', index=False)
                pending = []
            if (sample_i+1) % 250 == 0 or sample_i == len(selected)-1:
                elapsed = time.monotonic()-start
                status = dict(session=source.name, samples=sample_i+1, total=len(selected),
                              elapsed_s=round(elapsed, 1), throughput_hz=round((sample_i+1)/elapsed, 2))
                write_json(dest / 'progress.json', status)
                print(json.dumps(status), flush=True)
        encoder.stdin.close()
        if encoder.wait() != 0:
            raise RuntimeError('FFmpeg encoding failed')
    finally:
        capture.release()
        if encoder.poll() is None:
            encoder.kill()
            encoder.wait()
    result = pd.DataFrame(rows)
    result.to_parquet(dest / 'pose_samples.parquet', index=False)
    unique = result.drop_duplicates('capture_index').sort_values('host_monotonic_ns')
    unique.to_parquet(dest / 'pose_unique.parquet', index=False)
    video.replace(dest / 'pose_overlay.mp4')
    frames.to_parquet(dest / 'frame_index.parquet', index=False)
    write_json(marker, dict(session=source.name, cfr_frames=len(frames), pose_samples=len(result),
        unique_inferences=len(unique), model_sha256=model_hash, stride=STRIDE,
        overlay_fps=10, crop_xyxy=ROI, confidence_threshold=THRESHOLD,
        source_capture_timestamps=True, lower_legs_observable=False))


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--threads', type=int, default=4)
    parser.add_argument('--backend', choices=['openvino', 'onnxruntime', 'directml'], default='openvino')
    parser.add_argument('--decoder', choices=['opencv', 'jpeg'], default='opencv')
    parser.add_argument('--session', choices=list(SESSIONS), action='append')
    args = parser.parse_args()
    cv2.setNumThreads(1)
    args.output.mkdir(parents=True, exist_ok=True)
    model = load_model(args.model, args.threads, args.backend)
    model_hash = sha256(args.model)
    for session in args.session or SESSIONS:
        infer_session(args.input / session, args.output / session, model, model_hash, args.decoder)


if __name__ == '__main__':
    main()
