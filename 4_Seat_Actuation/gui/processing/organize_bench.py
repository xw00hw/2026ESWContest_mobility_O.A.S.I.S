"""Clean sensor tables, align host timestamps, and summarize measured dwell windows."""
import argparse
import importlib.metadata
import json
from pathlib import Path
import platform
import shutil
import sys

import cv2
import numpy as np
import pandas as pd

from prepare_bench import FEATURES, SESSIONS, sha256, write_json
from pose_core import NAMES, ROI, THRESHOLD

BIN_NS = 100_000_000
SENSORS = [f'A{bank}_C{channel}' for bank in range(3) for channel in range(4)]
# Conservative visual-review interval, not an inferred motor response.
CAMERA_REVIEW = {'20261025_005459_3512cd6e': (239.0, 280.0)}


def annotate(times, points):
    """Half-open dwell intervals; transition rows never inherit a dwell label."""
    times = np.asarray(times, dtype=np.int64)
    starts = points.dwell_start_ns.to_numpy(np.int64)
    ends = points.dwell_end_ns.to_numpy(np.int64)
    slot = np.searchsorted(starts, times, side='right') - 1
    safe = np.maximum(slot, 0)
    valid = (slot >= 0) & (times < ends[safe])
    return np.where(valid, points.point_index.to_numpy()[safe], -1)


def sensors(source, dest):
    marker = dest / 'sensors_complete.json'
    if marker.exists():
        print(f'{source.name}: sensors already complete', flush=True)
        return
    dest.mkdir(parents=True, exist_ok=True)
    meta = json.loads((source / 'metadata.json').read_text(encoding='utf-8'))
    label, expected = SESSIONS[source.name]
    assert meta['status'] == meta['auto_scan']['status'] == 'complete'
    assert not (source / 'INCOMPLETE').exists()
    points = pd.read_csv(source / 'auto_results.csv')
    assert len(points) == expected and points.status.eq('complete').all()
    assert points.point_index.nunique() == expected
    angles = ['left_yaw_deg', 'left_pitch_deg', 'right_yaw_deg', 'right_pitch_deg']
    assert not points.duplicated(angles).any()
    assert np.all(np.diff(points.dwell_start_ns) > 0)
    assert np.all(points.dwell_start_ns < points.dwell_end_ns)
    assert np.all(points.dwell_end_ns.to_numpy()[:-1] < points.dwell_start_ns.to_numpy()[1:])
    points.to_parquet(dest / 'points_source.parquet', index=False)
    pressure = pd.read_csv(source / 'pressure.csv', dtype={'sensor_id': 'category', 'bank': 'int8',
        'channel': 'int8', 'raw': 'int16', 'status': 'int16'})
    assert len(pressure) == meta['counts']['pressure_rows']
    assert set(pressure.sensor_id) == set(SENSORS)
    origin = meta['origin_monotonic_ns']
    pressure['dwell_point_index'] = annotate(pressure.host_monotonic_ns, points)
    pressure.to_parquet(dest / 'pressure_long.parquet', index=False)
    # USB receipt clock is shared by all streams. Do not align using uncorrected MCU time.
    pressure['bin'] = (pressure.host_monotonic_ns-origin)//BIN_NS
    valid = pressure.status.eq(0) & pressure.raw.between(0, 4095)
    p = pressure.loc[valid].copy()
    p['raw_zero'] = p.raw.eq(0).astype(float)
    p['raw_full_scale'] = p.raw.eq(4095).astype(float)
    stats = p.groupby(['bin', 'sensor_id'], observed=True).agg(
        mean_v=('voltage_v', 'mean'), std_v=('voltage_v', lambda x: x.std(ddof=0)),
        min_v=('voltage_v', 'min'), max_v=('voltage_v', 'max'), samples=('raw', 'size'),
        zero_fraction=('raw_zero', 'mean'), saturation_fraction=('raw_full_scale', 'mean'))
    wide = stats.unstack('sensor_id')
    wide.columns = [f'pressure_{sensor}_{stat}' for stat, sensor in wide.columns]
    all_bins = np.arange(0, int(pressure.bin.max())+1)
    wide = wide.reindex(all_bins).reset_index(names='bin')
    wide['host_monotonic_ns'] = origin + wide.bin*BIN_NS + BIN_NS//2
    wide['session_s'] = (wide.host_monotonic_ns-origin)/1e9
    wide['dwell_point_index'] = annotate(wide.host_monotonic_ns, points)
    wide.to_parquet(dest / 'pressure_10hz.parquet', index=False)
    motion = pd.read_csv(source / 'motion.csv')
    assert motion.host_monotonic_ns.is_monotonic_increasing
    motion['dwell_point_index'] = annotate(motion.host_monotonic_ns, points)
    motion.to_parquet(dest / 'motion.parquet', index=False)
    # Recompute dwell statistics rather than treating session-wide averages as point measurements.
    measured = p[p.dwell_point_index > 0]
    dwell = measured.groupby(['dwell_point_index', 'sensor_id'], observed=True).agg(
        samples=('raw', 'size'), mean_v=('voltage_v', 'mean'),
        std_v=('voltage_v', lambda x: x.std(ddof=0)), min_v=('voltage_v', 'min'),
        max_v=('voltage_v', 'max'), mean_raw=('raw', 'mean'),
        zero_fraction=('raw_zero', 'mean'), saturation_fraction=('raw_full_scale', 'mean')).reset_index()
    dwell.rename(columns={'dwell_point_index': 'point_index'}, inplace=True)
    dwell.to_csv(dest / 'point_pressure.csv', index=False)
    assert len(dwell) == expected*12
    stored = pd.read_csv(source / 'auto_pressure_summary.csv')
    comparison = dwell.merge(stored, on=['point_index', 'sensor_id'], suffixes=('_new', '_stored'), validate='one_to_one')
    max_summary_difference = float((comparison.mean_v_new-comparison.mean_v_stored).abs().max())
    assert max_summary_difference < 1e-8
    packets = pressure[pressure.bank.eq(0)]
    diffs = np.diff(packets.seq.to_numpy(np.int64))
    sensor_quality = []
    for sensor, group in pressure.groupby('sensor_id', observed=True):
        sensor_quality.append(dict(sensor_id=str(sensor), samples=len(group),
            mean_v=float(group.voltage_v.mean()), zero_fraction=float(group.raw.eq(0).mean()),
            saturated_fraction=float(group.raw.eq(4095).mean()), errors=int(group.status.ne(0).sum())))
    pd.DataFrame(sensor_quality).to_csv(dest / 'sensor_quality.csv', index=False)
    # MCU drift is diagnostic only; no affine clock correction silently changes source time.
    x = (packets.mcu_extended_ms.to_numpy()-int(packets.mcu_extended_ms.iloc[0]))/1000
    y = (packets.host_monotonic_ns.to_numpy()-int(packets.host_monotonic_ns.iloc[0]))/1e9
    host_s_per_mcu_s = float(np.polyfit(x, y, 1)[0])
    mismatches = sum(int((points[f'expected_{pin}_us'] != points[f'reported_{pin}_us']).sum()) for pin in ['D3','D4','D5','D6'])
    qc = dict(session=source.name, label=label, points=expected, pressure_rows=len(pressure),
        pressure_packets=len(packets), packet_missing=int(np.maximum(diffs-1, 0).sum()),
        packet_duplicate_or_backward=int((diffs <= 0).sum()), adc_errors=int((~valid).sum()),
        host_time_backward=int((np.diff(pressure.host_monotonic_ns) < 0).sum()),
        voltage_max_conversion_error_v=float((pressure.voltage_v-pressure.raw*3.3/4095).abs().max()),
        summary_max_difference_v=max_summary_difference, pwm_point_mismatches=mismatches,
        dwell_min_s=float(points.measurement_s.min()), dwell_max_s=float(points.measurement_s.max()),
        motion_states=len(motion), host_seconds_per_mcu_second=host_s_per_mcu_s,
        mcu_faster_percent=(1/host_s_per_mcu_s-1)*100,
        source_metadata_commit=meta['source_commit'], source_errors=meta['errors'])
    assert qc['packet_missing'] == qc['packet_duplicate_or_backward'] == qc['adc_errors'] == mismatches == 0
    for filename in ['metadata.json', 'auto_plan.csv', 'auto_summary.json']:
        shutil.copyfile(source / filename, dest / f'source_{filename}')
    write_json(marker, qc)
    print(json.dumps(qc), flush=True)


def merge(source, dest):
    assert (dest / 'pose_complete.json').exists()
    metadata = json.loads((source / 'metadata.json').read_text(encoding='utf-8'))
    pose = pd.read_parquet(dest / 'pose_unique.parquet').sort_values('host_monotonic_ns')
    samples = pd.read_parquet(dest / 'pose_samples.parquet')
    for table in (pose,samples):
        if 'inference_backend' not in table:
            table['inference_backend'] = 'openvino'
        table['inference_backend'] = table.inference_backend.fillna('openvino')
        if 'image_decoder' not in table:
            table['image_decoder'] = 'opencv_ffmpeg'
        table['image_decoder'] = table.image_decoder.fillna('opencv_ffmpeg')
        interval = CAMERA_REVIEW.get(source.name)
        table['camera_reposition_flag'] = table.video_time_s.between(*interval, inclusive='left') if interval else False
        table['camera_epoch'] = np.where(table.video_time_s >= interval[1], 2,
            np.where(table.video_time_s >= interval[0], 1, 0)).astype(np.int8) if interval else 0
        table['pose_review_required'] = ~table.target_valid | table.camera_reposition_flag | (table.valid_upper_joints < 9)
        table['pose_analysis_valid'] = table.target_valid & ~table.pose_review_required
    points = pd.read_parquet(dest / 'points_source.parquet')
    angle_columns = ['left_yaw_deg','left_pitch_deg','right_yaw_deg','right_pitch_deg']
    actual = set(map(tuple, points[angle_columns].to_numpy()))
    if SESSIONS[source.name][1] == 784:
        wanted = {(left,0,right,0) for left in range(28) for right in range(28)}
    elif source.name == '20261025_003530_9c4e477c':
        wanted = {(yaw,pitch,yaw,pitch) for yaw in range(21) for pitch in range(-8,9)}
    else:
        wanted = {(yaw,pitch,0,0) for yaw in range(21) for pitch in range(-8,9)}
    assert actual == wanted, 'Missing/unexpected structure-angle combinations'
    angle_mismatches = sum(int((points[f'reported_{name[:-4]}_mdeg'] != points[name]*1000).sum()) for name in angle_columns)
    assert angle_mismatches == 0
    pose['dwell_point_index'] = annotate(pose.host_monotonic_ns, points)
    samples['dwell_point_index'] = annotate(samples.host_monotonic_ns, points)
    pose.to_parquet(dest / 'pose_unique.parquet', index=False)
    samples.to_parquet(dest / 'pose_samples.parquet', index=False)
    wide = pd.read_parquet(dest / 'pressure_10hz.parquet')
    motion = pd.read_parquet(dest / 'motion.parquet')
    motion = motion.drop(columns=['session_s', 'host_wall_time_ns', 'dwell_point_index']).rename(columns={
        'host_monotonic_ns': 'motion_host_ns'})
    merged = pd.merge_asof(wide, motion, left_on='host_monotonic_ns', right_on='motion_host_ns',
                           direction='backward', tolerance=500_000_000)
    merged['motion_age_ms'] = (merged.host_monotonic_ns-merged.motion_host_ns)/1e6
    pose_join = pose.drop(columns=['session_s', 'host_wall_time_ns', 'dwell_point_index']).rename(columns={
        'host_monotonic_ns': 'pose_host_ns'})
    merged = pd.merge_asof(merged, pose_join, left_on='host_monotonic_ns', right_on='pose_host_ns',
                           direction='nearest', tolerance=100_000_000)
    merged['pose_age_ms'] = (merged.host_monotonic_ns-merged.pose_host_ns).abs()/1e6
    merged['pose_time_delta_ms'] = (merged.pose_host_ns-merged.host_monotonic_ns)/1e6
    merged['session_id'] = source.name
    merged.to_parquet(dest / 'aligned_10hz.parquet', index=False)
    measured = pose[pose.dwell_point_index > 0]
    pose_stats = measured.groupby('dwell_point_index').agg(
        pose_samples=('capture_index', 'size'), pose_target_valid_fraction=('target_valid', 'mean'),
        valid_upper_joints_mean=('valid_upper_joints', 'mean'),
        pose_review_required_fraction=('pose_review_required', 'mean'),
        pose_analysis_valid_fraction=('pose_analysis_valid', 'mean'),
        camera_reposition_fraction=('camera_reposition_flag', 'mean'))
    pose_stats['camera_epoch_min'] = measured.groupby('dwell_point_index').camera_epoch.min()
    pose_stats['camera_epoch_max'] = measured.groupby('dwell_point_index').camera_epoch.max()
    for feature in FEATURES:
        group = measured.groupby('dwell_point_index')[feature]
        pose_stats[f'{feature}_mean'] = group.mean()
        pose_stats[f'{feature}_std'] = group.std(ddof=0)
        pose_stats[f'{feature}_n'] = group.count()
        clean_group = measured[measured.pose_analysis_valid].groupby('dwell_point_index')[feature]
        pose_stats[f'{feature}_analysis_mean'] = clean_group.mean()
        pose_stats[f'{feature}_analysis_n'] = clean_group.count().reindex(pose_stats.index, fill_value=0)
    for name in NAMES:
        group = measured.groupby('dwell_point_index')[f'{name}_valid']
        pose_stats[f'{name}_valid_fraction'] = group.mean()
    pressure = pd.read_csv(dest / 'point_pressure.csv').set_index(['point_index', 'sensor_id'])
    pressure = pressure.unstack('sensor_id')
    pressure.columns = [f'pressure_{sensor}_{stat}' for stat, sensor in pressure.columns]
    summary = points.merge(pressure, left_on='point_index', right_index=True, how='left', validate='one_to_one')
    summary = summary.merge(pose_stats, left_on='point_index', right_index=True, how='left', validate='one_to_one')
    for column in ['pose_samples'] + [c for c in summary.columns if c.endswith(('_analysis_n','_deg_n','_ratio_n','_px_n'))]:
        summary[column] = summary[column].fillna(0).astype(np.int32)
    summary.insert(0, 'session_id', source.name)
    summary['scan_label'] = SESSIONS[source.name][0]
    summary.to_csv(dest / 'points.csv', index=False)
    summary.to_parquet(dest / 'points.parquet', index=False)
    frame_index = pd.read_parquet(dest / 'frame_index.parquet')
    # Full 30 fps source index gets a nearest estimated capture and explicit age (no interpolation).
    frame_index = frame_index.sort_values('host_monotonic_ns')
    lookup = pose[['host_monotonic_ns', 'pose_sample_index', 'capture_index']].rename(columns={
        'host_monotonic_ns': 'pose_host_ns', 'capture_index': 'pose_capture_index'})
    full = pd.merge_asof(frame_index, lookup, left_on='host_monotonic_ns', right_on='pose_host_ns',
                         direction='nearest', tolerance=100_000_000)
    full['pose_age_ms'] = (full.host_monotonic_ns-full.pose_host_ns).abs()/1e6
    full['dwell_point_index'] = annotate(full.host_monotonic_ns, points)
    full.sort_values('video_frame_index').to_parquet(dest / 'frame_pose_index.parquet', index=False)
    cap = cv2.VideoCapture(str(dest / 'pose_overlay.mp4'))
    overlay_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    overlay_fps = float(cap.get(cv2.CAP_PROP_FPS))
    assert cap.isOpened() and overlay_frames == len(samples) and overlay_fps == 10
    cap.release()
    qc = dict(**json.loads((dest / 'sensors_complete.json').read_text(encoding='utf-8')),
        **{k:v for k,v in json.loads((dest / 'pose_complete.json').read_text()).items() if k not in ['session']},
        source_unique_frames=int(frame_index.capture_index.nunique()),
        repeated_cfr_frames=int(frame_index.repeated.sum()),
        target_valid_fraction=float(pose.target_valid.mean()),
        shoulder_valid_fraction=float((pose.left_shoulder_valid & pose.right_shoulder_valid).mean()),
        hip_valid_fraction=float((pose.left_hip_valid & pose.right_hip_valid).mean()),
        min_point_pose_samples=int(summary.pose_samples.min()), max_point_pose_samples=int(summary.pose_samples.max()),
        min_point_target_valid_fraction=float(summary.pose_target_valid_fraction.min()),
        points_without_pose=int(summary.pose_samples.eq(0).sum()),
        aligned_rows=len(merged), aligned_pose_missing=int(merged.pose_host_ns.isna().sum()),
        aligned_motion_missing=int(merged.motion_host_ns.isna().sum()),
        aligned_pose_age_p95_ms=float(merged.pose_age_ms.quantile(.95)),
        source_duration_s=len(frame_index)/30, overlay_duration_s=overlay_frames/overlay_fps,
        overlay_frames=overlay_frames)
    qc['inference_backend_unique_counts'] = {str(k):int(v) for k,v in pose.inference_backend.value_counts().items()}
    qc['image_decoder_unique_counts'] = {str(k):int(v) for k,v in pose.image_decoder.value_counts().items()}
    qc['pose_review_required_fraction'] = float(pose.pose_review_required.mean())
    qc['camera_reposition_pose_samples'] = int(pose.camera_reposition_flag.sum())
    qc['camera_reposition_video_interval_s'] = list(CAMERA_REVIEW[source.name]) if source.name in CAMERA_REVIEW else None
    qc['point_command_angle_mismatches'] = angle_mismatches
    qc['created_utc'] = metadata['created_utc']
    qc['stopped_utc'] = metadata['stopped_utc']
    qc['camera_name'] = metadata['context']['camera']['name']
    assert sum(qc['inference_backend_unique_counts'].values()) == len(pose)
    assert sum(qc['image_decoder_unique_counts'].values()) == len(pose)
    write_json(dest / 'quality_report.json', qc)
    print(json.dumps(qc), flush=True)
    return qc


def provenance(source, dest):
    path = dest / 'source_manifest.json'
    if path.exists():
        return
    entries = []
    for file in sorted(source.iterdir()):
        if file.is_file():
            entries.append(dict(filename=file.name, bytes=file.stat().st_size, sha256=sha256(file)))
    write_json(path, dict(session=source.name, immutable_source=str(source.resolve()), files=entries))


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--stage', choices=['sensors', 'merge', 'manifest', 'all'], default='all')
    parser.add_argument('--session', action='append', choices=list(SESSIONS))
    args = parser.parse_args()
    for session in args.session or SESSIONS:
        src, dst = args.input / session, args.output / session
        if args.stage in ['sensors', 'all']:
            sensors(src, dst)
        if args.stage in ['manifest', 'all']:
            provenance(src, dst)
        if args.stage in ['merge', 'all']:
            merge(src, dst)
    if args.stage in ['merge', 'all'] and not args.session:
        points = pd.concat([pd.read_parquet(args.output / s / 'points.parquet') for s in SESSIONS], ignore_index=True)
        assert len(points) == 1498
        points.to_csv(args.output / 'all_points.csv', index=False)
        points.to_parquet(args.output / 'all_points.parquet', index=False)
        compact = ['session_id','scan_label','point_index','left_yaw_deg','left_pitch_deg','right_yaw_deg','right_pitch_deg',
            'dwell_start_ns','dwell_end_ns'] + [f'pressure_{s}_mean_v' for s in SENSORS] + [
            f'{feature}_analysis_mean' for feature in FEATURES[:3]] + [
            'pose_samples','pose_analysis_valid_fraction','pose_review_required_fraction','camera_reposition_fraction',
            'camera_epoch_min','camera_epoch_max']
        points[compact].to_csv(args.output / 'analysis_points.csv', index=False)
        reports = [json.loads((args.output / s / 'quality_report.json').read_text()) for s in SESSIONS]
        pd.DataFrame(reports).to_csv(args.output / 'sessions.csv', index=False)
        versions = {p:importlib.metadata.version(p) for p in ['rtmlib','onnxruntime','openvino','opencv-python','numpy','pandas','pyarrow','imageio-ffmpeg']}
        versions.update(python=sys.version, platform=platform.platform())
        write_json(args.output / 'environment.json', versions)


if __name__ == '__main__':
    main()
