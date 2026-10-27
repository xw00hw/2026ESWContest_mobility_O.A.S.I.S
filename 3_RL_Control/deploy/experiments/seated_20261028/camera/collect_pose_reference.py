"""Collect bounded geometry from the existing Pi camera owner; never opens a camera.

Run on that same Pi so CLOCK_MONOTONIC timestamps have the same clock domain.
Only explicit --operator-confirmed permits writing a seated neutral. No images,
serial devices, UDP commands, or motor/controller configuration are touched.
"""
import argparse
import json
import math
from pathlib import Path
import time
import urllib.request

import numpy as np
from oasis_rt.rtmpose_math import BodyGeometry, image_transform, make_neutral, relative_features


def observation(status, now_ns, camera_id, model_sha256):
    sample = status.get('calibration_observation')
    if not isinstance(sample, dict) or sample.get('schema') != 'oasis.pose.calibration_observation.v1':
        raise ValueError('no current validated body geometry')
    if sample.get('camera_id') != camera_id or sample.get('model_sha256') != model_sha256:
        raise ValueError('camera/model mismatch')
    stamp, generation = sample.get('capture_ns'), sample.get('generation')
    if type(stamp) is not int or not 0 <= now_ns-stamp < 300_000_000:
        raise ValueError('stale or future geometry')
    if type(generation) is not int or generation < 1 or not sample.get('stream_id'):
        raise ValueError('invalid capture generation')
    raw = sample['geometry']
    geometry = BodyGeometry(**{k: tuple(v) if isinstance(v,list) else v for k,v in raw.items()})
    transform = image_transform(geometry.frame_hw, geometry.roi_xyxy)
    reference = make_neutral(geometry,transform,model_sha256,operator_confirmed=True)
    relative_features(geometry,reference,transform,model_sha256)
    if geometry.pose_confidence < .5:
        raise ValueError('body confidence below .5')
    return dict(sample, geometry=geometry, transform=transform)


def summarize(samples, *, stable=True):
    if len(samples) < 20:
        raise ValueError('need at least 20 unique fresh observations')
    first=samples[0]
    identity=lambda s:(s['stream_id'],s['generation'],s['camera_id'],s['model_sha256'],
                        s['geometry'].frame_hw,s['geometry'].roi_xyxy)
    if any(identity(s)!=identity(first) for s in samples):
        raise ValueError('camera/stream/generation changed during collection')
    stamps=[s['capture_ns'] for s in samples]
    if any(b<=a for a,b in zip(stamps,stamps[1:])) or (stamps[-1]-stamps[0])/1e9<2.5:
        raise ValueError('need increasing captures over at least 2.5 seconds')
    shoulders=np.array([s['geometry'].shoulder_center_px for s in samples])
    hips=np.array([s['geometry'].hip_center_px for s in samples])
    widths=np.array([s['geometry'].shoulder_width_px for s in samples])
    angles=np.array([s['geometry'].torso_angle_deg for s in samples])
    width=float(np.median(widths))
    quality=dict(samples=len(samples),span_s=(stamps[-1]-stamps[0])/1e9,
                 shoulder_std_width=float(np.max(shoulders.std(axis=0))/width),
                 hip_std_width=float(np.max(hips.std(axis=0))/width),
                 width_cv=float(widths.std()/width),torso_std_deg=float(angles.std()))
    if stable and (quality['shoulder_std_width']>.04 or quality['hip_std_width']>.04 or
                   quality['width_cv']>.05 or quality['torso_std_deg']>2.):
        raise ValueError('reference moved during capture: '+json.dumps(quality))
    shoulder,hip=np.median(shoulders,axis=0),np.median(hips,axis=0)
    angle=math.degrees(math.atan2(float(shoulder[0]-hip[0]),float(hip[1]-shoulder[1])))
    g=BodyGeometry(first['geometry'].frame_hw,first['geometry'].roi_xyxy,tuple(shoulder),tuple(hip),
                   width,angle,min(s['geometry'].pose_confidence for s in samples))
    neutral=make_neutral(g,first['transform'],first['model_sha256'],operator_confirmed=True)
    relative_features(g,neutral,first['transform'],first['model_sha256'])
    neutral.update(camera_id=first['camera_id'],collection=quality,
                   evidence_stream_id=first['stream_id'],evidence_generation=first['generation'],
                   first_capture_ns=stamps[0],last_capture_ns=stamps[-1])
    return neutral,g,first['transform']


def collect(camera_id,model_sha256,seconds=5.,timeout=15.):
    start=time.monotonic(); samples=[]; seen=set(); rejected={}
    while time.monotonic()-start<timeout:
        try:
            with urllib.request.urlopen('http://127.0.0.1:8081/status.json',timeout=.5) as response:
                data=response.read(65537)
                if len(data)>65536:raise ValueError('status too large')
                sample=observation(json.loads(data),time.monotonic_ns(),camera_id,model_sha256)
            stamp=sample['capture_ns']
            if stamp not in seen:
                if samples and (sample['stream_id'],sample['generation']) != (samples[0]['stream_id'],samples[0]['generation']):
                    raise RuntimeError('camera reconnected during reference capture; retry after stable')
                if samples and stamp<=samples[-1]['capture_ns']:raise RuntimeError('capture timestamp moved backward')
                seen.add(stamp);samples.append(sample)
            if len(samples)>=20 and (samples[-1]['capture_ns']-samples[0]['capture_ns'])/1e9>=seconds:
                return samples,rejected
        except (ValueError,KeyError,TypeError,OSError) as exc:
            key=str(exc)[:180];rejected[key]=rejected.get(key,0)+1
        time.sleep(.05)
    raise RuntimeError('reference timeout: '+json.dumps(dict(accepted=len(samples),rejected=rejected)))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--camera-id',required=True);p.add_argument('--model-sha256',required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--operator-confirmed',action='store_true')
    p.add_argument('--neutral',type=Path,help='Existing neutral for a direction check, never modified')
    p.add_argument('--side',choices=['left','right'],help='Occupant side during explicit direction check')
    args=p.parse_args()
    if not args.operator_confirmed:p.error('explicit seated posture confirmation required')
    if bool(args.neutral)!=bool(args.side):p.error('--neutral and --side must be used together')
    if args.output.exists():p.error('refusing to overwrite existing reference/evidence')
    samples,rejected=collect(args.camera_id,args.model_sha256)
    reference,geometry,transform=summarize(samples)
    if args.neutral:
        neutral=json.loads(args.neutral.read_text(encoding='utf-8'))
        if neutral.get('camera_id')!=args.camera_id:raise ValueError('neutral camera mismatch')
        features=relative_features(geometry,neutral,transform,args.model_sha256)
        lean,angle=features['lean_score'],features['torso_lean_deg']
        if abs(lean)<.06 or abs(angle)<2 or lean*angle<=0:
            raise ValueError('direction check requires a clear consistent torso lean and shoulder shift')
        expected=1 if args.side=='left' else -1  # Runtime +y = occupant/vehicle left.
        reference=dict(schema='oasis.pose.direction_check.v1',side=args.side,
                       recommended_lean_sign=expected*(1 if lean>0 else -1),
                       features=features,collection=reference['collection'],
                       camera_id=args.camera_id,model_sha256=args.model_sha256,
                       source_stream_id=samples[0]['stream_id'],generation=samples[0]['generation'],
                       final_capture_ns=samples[-1]['capture_ns'],rejected=rejected)
    else:
        reference['rejected_observations']=rejected
    with args.output.open('x',encoding='utf-8') as out:
        json.dump(reference,out,indent=2,allow_nan=False)
    print(json.dumps(reference,indent=2,allow_nan=False))


if __name__=='__main__':main()
