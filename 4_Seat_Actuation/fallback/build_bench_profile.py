"""Build the deterministic pressure profile from sensor/motor-only bench files.

No camera, image, landmark, inferred-pose, or merged multimodal table is read.
Requires numpy and pyarrow only for this offline builder; the output is plain JSON.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys

SENSORS = [f'A{bank}_C{channel}' for bank in range(3) for channel in range(4)]
AXES = ['left_yaw_deg', 'left_pitch_deg', 'right_yaw_deg', 'right_pitch_deg']
SESSION = '20261025_013054_fdbdd0b4'
COMPARISON = '20261025_005459_3512cd6e'
DATASET = 'bench_20261025_pose_v1'


def pava(values, weights=None, increasing=True):
    """Weighted least-squares isotonic regression; preserve the input order."""
    import math
    values = list(values)
    weights = [1.0]*len(values) if weights is None else list(weights)
    if len(weights) != len(values) or not values:
        raise ValueError('Nonempty values and matching weights required')
    if not all(math.isfinite(x) for x in values+weights) or min(weights) <= 0:
        raise ValueError('Finite values and positive weights required')
    sign = 1.0 if increasing else -1.0
    blocks = []
    for value, weight in zip(values, weights):
        blocks.append([sign*value*weight, weight, 1])
        while len(blocks) > 1 and blocks[-2][0]/blocks[-2][1] > blocks[-1][0]/blocks[-1][1]:
            b = blocks.pop(); a = blocks.pop()
            blocks.append([a[0]+b[0], a[1]+b[1], a[2]+b[2]])
    return [sign*total/weight for total, weight, count in blocks for _ in range(count)]


def csv_rows(path):
    with path.open(encoding='utf-8', newline='') as stream:
        return list(csv.DictReader(stream))


def build(dataset_dir, output):
    import numpy as np
    import pyarrow.parquet as pq
    dataset_dir, output = Path(dataset_dir), Path(output)
    raw = dataset_dir/'raw'/DATASET
    source = raw/SESSION
    meta = json.loads((source/'source_metadata.json').read_text(encoding='utf-8'))
    plan = csv_rows(source/'source_auto_plan.csv')
    mean_csv = {(int(r['point_index']),r['sensor_id']):r for r in csv_rows(source/'point_pressure.csv')}
    # Explicit projection avoids even reading video-count columns in source auto_results.
    point_columns = ['point_index', *AXES, 'command_id', 'arrived_ns', 'dwell_start_ns', 'dwell_end_ns',
        'status', 'measurement_s', 'pressure_nominal_samples', 'pressure_valid_samples',
        'pressure_adc_errors', 'pressure_missing_sensors',
        *['expected_'+pin+'_us' for pin in ('D3','D4','D5','D6')],
        *['reported_'+pin+'_us' for pin in ('D3','D4','D5','D6')]]
    all_points = {r['point_index']:r for r in pq.read_table(source/'points_source.parquet', columns=point_columns).to_pylist()}
    selected = {int(r['point_index']):r for r in plan if
        float(r['left_pitch_deg']) == float(r['right_pitch_deg']) == 0 and
        (float(r['left_yaw_deg']) == 0 or float(r['right_yaw_deg']) == 0)}
    assert len(selected) == 55
    pressure_columns = ['host_monotonic_ns','sensor_id','raw','status','voltage_v','dwell_point_index']
    data = pq.read_table(source/'pressure_long.parquet', columns=pressure_columns,
                         filters=[('dwell_point_index','in',list(selected))]).to_pydict()
    pressure_ids = np.asarray(data['dwell_point_index'])
    sensor_ids = np.asarray(data['sensor_id'])
    raw_values = np.asarray(data['raw'], dtype=float)
    statuses = np.asarray(data['status'])
    host_ns = np.asarray(data['host_monotonic_ns'], dtype=np.int64)
    saved_v = np.asarray(data['voltage_v'],dtype=float)
    assert np.max(np.abs(saved_v-raw_values*3.3/4095)) < 1e-10
    motion_columns = ['host_monotonic_ns','armed','moving','fault','dwell_point_index',
        'left_yaw_mdeg','left_pitch_mdeg','right_yaw_mdeg','right_pitch_mdeg',
        'target_left_yaw_mdeg','target_left_pitch_mdeg','target_right_yaw_mdeg','target_right_pitch_mdeg',
        'D3_us','D4_us','D5_us','D6_us']
    motion = pq.read_table(source/'motion.parquet', columns=motion_columns,
                           filters=[('dwell_point_index','in',list(selected))]).to_pylist()
    records = {}
    for point_id, command in selected.items():
        completed = all_points[point_id]
        angles = [float(command[name]) for name in AXES]
        begin, end = completed['dwell_start_ns'], completed['dwell_end_ns']
        states = [r for r in motion if r['dwell_point_index'] == point_id and begin <= r['host_monotonic_ns'] < end]
        state_angles = [name.replace('_deg','_mdeg') for name in AXES]
        matches = all(all(r[name] == round(value*1000) and r['target_'+name] == round(value*1000)
            for name,value in zip(state_angles,angles)) for r in states)
        pwm_match = all(completed['expected_'+pin+'_us'] == completed['reported_'+pin+'_us']
            for pin in ('D3','D4','D5','D6')) and all(all(r[pin+'_us'] == completed['expected_'+pin+'_us']
            for pin in ('D3','D4','D5','D6')) for r in states)
        settle_s = (begin-completed['arrived_ns'])/1e9
        state_check = dict(state_samples=len(states), all_armed=all(r['armed']==1 for r in states),
            all_moving_false=all(r['moving']==0 for r in states), all_fault_none=all(r['fault']=='NONE' for r in states),
            reported_and_target_angles_match=matches, expected_reported_pwm_match=pwm_match,
            settling_delay_s=settle_s, passed=bool(states) and matches and pwm_match and settle_s >= .5 and
            all(r['armed']==1 and r['moving']==0 and r['fault']=='NONE' for r in states))
        assert state_check['passed'], (point_id,state_check)
        sensor = {}
        for name in SENSORS:
            mask = (pressure_ids==point_id)&(sensor_ids==name)
            valid = mask&(statuses==0)&(raw_values>=0)&(raw_values<=4095)
            values = raw_values[valid]*3.3/4095
            times = host_ns[mask]
            assert len(values) >= 180 and np.all((times>=begin)&(times<end))
            row = mean_csv[point_id,name]
            assert len(values) == int(row['samples'])
            assert abs(values.mean()-float(row['mean_v'])) < 1e-10
            p05,p50,p95 = np.quantile(values,[.05,.5,.95],method='linear')
            sensor[name] = dict(samples=int(len(values)), mean_v=float(values.mean()),
                std_v=float(values.std(ddof=0)), min_v=float(values.min()), max_v=float(values.max()),
                p05_v=float(p05), p50_v=float(p50), p95_v=float(p95),
                adc_errors=int(np.sum(mask&~valid)), zero_fraction=float(np.mean(values==0)),
                saturation_fraction=float(np.mean(values>=3.3)),
                sample_count_fraction_of_nominal=float(len(values)/200),
                quality_ok=bool(len(values)>=180 and np.sum(mask&~valid)==0 and np.all(values<3.3)))
        assert completed['status']=='complete' and completed['pressure_adc_errors']==0 and completed['pressure_missing_sensors']==0
        records[tuple(angles)] = dict(point_index=point_id, command_deg=dict(zip(AXES,angles)),
            interval=dict(start_host_monotonic_ns=begin,end_host_monotonic_ns=end,duration_s=(end-begin)/1e9,
                clock='original host perf_counter_ns',bounds='[start,end)'),
            commanded_pwm_us={pin:int(command['expected_'+pin+'_us']) for pin in ('D3','D4','D5','D6')},
            motion_check=state_check, channels=sensor, quality_ok=state_check['passed'] and all(x['quality_ok'] for x in sensor.values()))
    branches = {}
    for side in ('left','right'):
        points = []
        for angle in range(28):
            key = (angle,0,0,0) if side=='left' else (0,0,angle,0)
            points.append(dict(yaw_deg=angle,**records[key]))
        fits = {}
        for name in SENSORS:
            means = [p['channels'][name]['mean_v'] for p in points]
            weights = [p['channels'][name]['samples'] for p in points]
            increasing, decreasing = pava(means,weights),pava(means,weights,False)
            rmse = lambda fit:float(np.sqrt(np.average((np.asarray(means)-fit)**2,weights=weights)))
            base = points[0]['channels'][name]
            p95_peak = max(p['channels'][name]['p95_v'] for p in points)
            p95_positive_span = max(0.0,p95_peak-base['p95_v'])
            fits[name] = dict(increasing_mean_v=increasing,decreasing_mean_v=decreasing,
                increasing_weighted_rmse_v=rmse(increasing),decreasing_weighted_rmse_v=rmse(decreasing),
                observed_mean_endpoint_delta_v=means[-1]-means[0],
                observed_peak_point_p95_v=p95_peak, baseline_point_p95_v=base['p95_v'],
                positive_p95_span_from_baseline_v=p95_positive_span,
                candidate_twenty_percent_positive_delta_v=.2*p95_positive_span,
                candidate_is_not_human_pressure_limit=True)
        branches[side] = dict(other_yaw_deg=0,both_pitch_deg=0,yaw_knots_deg=list(range(28)),
            points=points,per_channel_monotonic_fits=fits,
            no_extrapolation=True,initial_angle_candidate_deg=5,
            initial_angle_candidate_basis='Floor of 20 percent of observed 0..27deg span. Software commissioning candidate only; clearance and human pressure safety not measured.')
    previous_plan = {int(r['point_index']):r for r in csv_rows(raw/COMPARISON/'source_auto_plan.csv')}
    previous_pressure = {(int(r['point_index']),r['sensor_id']):r for r in csv_rows(raw/COMPARISON/'point_pressure.csv')}
    comparison = []
    for point_id,row in previous_plan.items():
        if [float(row[name]) for name in AXES[1:]] != [0,0,0]:
            continue
        yaw = int(float(row['left_yaw_deg']))
        comparison.append(dict(left_yaw_deg=yaw,
            earlier_mean_v={s:float(previous_pressure[point_id,s]['mean_v']) for s in SENSORS},
            later_mean_v={s:branches['left']['points'][yaw]['channels'][s]['mean_v'] for s in SENSORS}))
    files = [source/name for name in ['source_metadata.json','source_auto_plan.csv','point_pressure.csv',
        'pressure_long.parquet','motion.parquet','points_source.parquet']]
    files += [raw/COMPARISON/name for name in ['source_metadata.json','source_auto_plan.csv','point_pressure.csv']]
    provenance = [dict(path=str(p.relative_to(raw)).replace('\\','/'),size_bytes=p.stat().st_size,
        sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in files]
    profile = dict(schema='oasis-bench-pressure-profile-v1',dataset_id=DATASET,session_id=SESSION,
        source_drive_file_id='1HTNr1ZExkWfbuYSdfRaxxZawIA2wcV_h', source_commit=meta['source_commit'],
        input_policy='Only pressure ADC, commanded motor state, experiment config, and clock metadata. No image/video/landmark/inferred-pose or combined multimodal tables.',
        sensors=SENSORS,physical_channel_mapping=None,pressure_unit='estimated_voltage_v',vref_v=3.3,vref_measured=False,
        force_calibrated=False,actual_position_feedback=False,physical_clearance_verified=False,
        samples_nominal_hz=100,quantile_method='numpy linear on valid raw ADC * 3.3 / 4095, inside exact 2s dwell',
        motor_configuration=meta['config']['mapping'],firmware_capabilities=meta['context']['firmware_capabilities'],
        hardware_compatibility_required=True,baseline=records[(0,0,0,0)],branches=branches,
        qualitative_earlier_left_only=dict(session_id=COMPARISON,points=sorted(comparison,key=lambda r:r['left_yaw_deg']),
            warning='Different session baseline/contact state; not interchangeable calibration and not repeated trials.'),
        known_low_signal_channels=['A2_C0','A2_C3'],
        limitations=['No physical sensor-position map or force/pressure calibration was supplied.',
            'Neutral is a loaded/contact reference in this session, not an unloaded ADC zero.',
            'Each four-angle command was visited once per session. Hysteresis and repeatability are not identified.',
            'Channel PAVA fits are statistical monotonic approximations, not proof of positive actuator gain or human safety.',
            'Cross-session same-command ADC means differ substantially. Re-establish current neutral baseline.',
            'Independent branch control is represented only at opposite yaw=0 and both pitch=0. Do not extrapolate.',
            'Maximum observed yaw is an observation boundary, not a safe mechanical or human-contact limit.',
            'Sum of per-channel p95 values is not a measured p95 of the sum.'],
        selected_unique_points=len(records), selected_pressure_readings=len(pressure_ids),
        quality_all_points_ok=all(r['quality_ok'] for r in records.values()),provenance=provenance)
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(profile,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps(dict(output=str(output),sha256=hashlib.sha256(output.read_bytes()).hexdigest(),
        unique_points=len(records),pressure_readings=len(pressure_ids),quality_ok=profile['quality_all_points_ok']),indent=2))
    return profile


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset-dir',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--pyarrow-dir',type=Path,help='Optional isolated dependency directory')
    args=parser.parse_args()
    if args.pyarrow_dir: sys.path.insert(0,str(args.pyarrow_dir))
    build(args.dataset_dir,args.output)


if __name__=='__main__': main()
