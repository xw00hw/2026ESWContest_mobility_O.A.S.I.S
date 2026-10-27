"""Derive per-pose data from immutable raw CSVs using half-open host time windows."""
from bisect import bisect_right
import csv
import json
import math
from pathlib import Path

from auto_scan import POINT_FIELDS,POSE_DEG_FIELDS


def summarize(directory):
    root = Path(directory)
    marker = root/'AUTO_SUMMARY_INCOMPLETE'
    marker.write_text('processing\n',encoding='utf-8')
    try:
        return _summarize(root,marker)
    except Exception as error:
        (root/'auto_summary_error.json').write_text(json.dumps(dict(error=str(error))),encoding='utf-8')
        raise


def _summarize(root,marker):
    def rows(name):
        with (root/(name+'.csv')).open(encoding='utf-8',newline='') as file:
            yield from csv.DictReader(file)

    meta = json.loads((root/'metadata.json').read_text(encoding='utf-8'))
    plan = list(rows('auto_plan'))
    expected = {int(r['point_index']):r for r in plan}
    if len(expected)!=len(plan):
        raise ValueError('Duplicate AUTO plan points')
    attempts = list(rows('auto_points'))
    points = {int(r['point_index']):r for r in attempts}
    if len(points)!=len(attempts):
        raise ValueError('Duplicate AUTO point results')
    for i,r in points.items():
        if i not in expected or (r['yaw_deg'],r['pitch_deg'])!=(expected[i]['yaw_deg'],expected[i]['pitch_deg']):
            raise ValueError('AUTO result does not match its plan')
        for name in ('left_yaw_deg','left_pitch_deg','right_yaw_deg','right_pitch_deg'):
            if r.get(name)!=expected[i].get(name):
                raise ValueError('AUTO left/right result does not match its plan')
        if r['status']=='complete' and not (
            int(r['queued_ns'])<=int(r['tx_ns'])<=int(r['ack_ns'])<=int(r['arrived_ns'])
            <=int(r['dwell_start_ns'])<int(r['dwell_end_ns'])<=int(r['end_state_ns'])):
            raise ValueError('Invalid AUTO point time ordering')
    windows = sorted((int(r['dwell_start_ns']),int(r['dwell_end_ns']),i)
                     for i,r in points.items() if r['status']=='complete')
    if any(a[1]>b[0] for a,b in zip(windows,windows[1:])):
        raise ValueError('AUTO measurement windows overlap')
    starts = [w[0] for w in windows]

    def point_at(ns):
        i = bisect_right(starts,ns)-1
        return windows[i][2] if i>=0 and ns<windows[i][1] else None

    stats = {}
    for r in rows('pressure'):
        point = point_at(int(r['host_monotonic_ns']))
        if point is None:
            continue
        key = point,r['sensor_id']
        s = stats.setdefault(key,dict(samples=0,valid=0,errors=0,mean=0.,m2=0.,
            min_v=None,max_v=None,raw_sum=0,raw_min=None,raw_max=None))
        s['samples'] += 1
        if int(r['status'])!=0 or not r['raw'] or not r['voltage_v']:
            s['errors'] += 1
            continue
        voltage,raw = float(r['voltage_v']),int(r['raw'])
        if not math.isfinite(voltage) or not 0<=raw<=4095:
            raise ValueError('Invalid raw pressure CSV value')
        s['valid'] += 1
        delta = voltage-s['mean']
        s['mean'] += delta/s['valid']
        s['m2'] += delta*(voltage-s['mean'])
        s['raw_sum'] += raw
        for name,value in [('v',voltage),('raw',raw)]:
            low,high = ('min_v','max_v') if name=='v' else ('raw_min','raw_max')
            s[low] = value if s[low] is None else min(s[low],value)
            s[high] = value if s[high] is None else max(s[high],value)

    video = {}
    origin = int(meta['origin_monotonic_ns'])
    for r in rows('frame_timestamps'):
        point = point_at(origin+round(float(r['video_time_s'])*1e9))
        if point is None:
            continue
        v = video.setdefault(point,dict(frames=0,repeated=0,source_ids=set(),first=None,last=None))
        index = int(r['video_frame_index'])
        v['first'] = index if v['first'] is None else min(v['first'],index)
        v['last'] = index
        v['frames'] += 1
        v['repeated'] += int(r['repeated'])
        if not int(r['repeated']) and point_at(int(r['host_monotonic_ns']))==point:
            v['source_ids'].add(r['capture_index'])

    selected = meta['context'].get('pressure',False)
    result_rows,pressure_rows = [],[]
    for target in plan:
        point = int(target['point_index'])
        result = {**target,**points.get(point,{}),'status':points.get(point,{}).get('status','not_attempted')}
        complete = result['status']=='complete'
        duration = (int(result['dwell_end_ns'])-int(result['dwell_start_ns']))/1e9 if complete else None
        expected = round(duration*100) if selected and complete else 0
        valid_total,error_total,missing_sensors = 0,0,0
        for bank in range(3):
            for channel in range(4):
                sensor = f'A{bank}_C{channel}'
                s = stats.get((point,sensor),{})
                valid = s.get('valid',0)
                valid_total += valid
                error_total += s.get('errors',0)
                missing_sensors += int(selected and complete and not valid)
                pressure_rows.append(dict(point_index=point,yaw_deg=target['yaw_deg'],pitch_deg=target['pitch_deg'],
                    **{name:target.get(name) for name in POSE_DEG_FIELDS},
                    point_status=result['status'],sensor_id=sensor,bank=bank,channel=channel,
                    nominal_samples=expected,samples=s.get('samples',0),valid_samples=valid,
                    adc_errors=s.get('errors',0),valid_coverage=valid/expected if expected else None,
                    mean_v=s['mean'] if valid else None,
                    std_v=math.sqrt(max(0,s['m2']/valid)) if valid else None,
                    min_v=s.get('min_v'),max_v=s.get('max_v'),
                    mean_raw=s['raw_sum']/valid if valid else None, min_raw=s.get('raw_min'),max_raw=s.get('raw_max'),
                    source_status='not_selected' if not selected else ('not_completed' if not complete else
                        ('no_data' if not valid else ('adc_errors' if s['errors'] else 'ok')))))
        v = video.get(point,{})
        result.update(measurement_s=duration,pressure_nominal_samples=12*expected,
            pressure_valid_samples=valid_total,pressure_adc_errors=error_total,
            pressure_missing_sensors=missing_sensors,video_frames=v.get('frames',0),
            video_repeated_frames=v.get('repeated',0),video_unique_source_frames=len(v.get('source_ids',())),
            first_video_frame=v.get('first'),last_video_frame=v.get('last'))
        result_rows.append(result)

    extra = ['measurement_s','pressure_nominal_samples','pressure_valid_samples','pressure_adc_errors',
             'pressure_missing_sensors','video_frames','video_repeated_frames','video_unique_source_frames',
             'first_video_frame','last_video_frame']
    for name,data,fields in [('auto_results',result_rows,POINT_FIELDS+extra),
                             ('auto_pressure_summary',pressure_rows,list(pressure_rows[0]))]:
        path = root/(name+'.csv.tmp')
        with path.open('w',encoding='utf-8',newline='') as file:
            writer = csv.DictWriter(file,fieldnames=fields,extrasaction='ignore')
            writer.writeheader(); writer.writerows(data)
        path.replace(root/(name+'.csv'))
    summary = dict(processing_status='complete',scan_status=meta['auto_scan']['status'],
        side=meta['auto_scan'].get('side','both'),
        mode=meta['auto_scan'].get('mode','grid'),
        recording_status=meta['status'],recording_errors=meta.get('errors',[]),
        planned=len(plan),completed=sum(r['status']=='complete' for r in result_rows),
        not_attempted=sum(r['status']=='not_attempted' for r in result_rows),
        pressure_selected=selected,video_selected=meta['context'].get('video',False),
        clock='host perf_counter_ns; [dwell_start_ns,dwell_end_ns)',
        arrival='Commanded position only; no measured shaft feedback',std_v='population standard deviation')
    path = root/'auto_summary.json.tmp'
    path.write_text(json.dumps(summary,indent=2),encoding='utf-8')
    path.replace(root/'auto_summary.json')
    marker.unlink()
    return summary
