"""Select historical seated camera frames without changing source recordings."""
import csv
import hashlib
import json
import os
from pathlib import Path

STAGE=Path(__file__).resolve().parent
GUI=Path(os.environ.get('OASIS_GUI_ROOT', STAGE.parents[5]/'4_Seat_Actuation'/'gui'))  # gui/processed/ holds the bench recordings
BENCH=GUI/'processed/bench_rtmpose_v1'
SESSIONS=('S09','S10')
def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for data in iter(lambda:f.read(8*1024*1024),b''):h.update(data)
    return h.hexdigest()
def wsl(p):
    return '/mnt/'+p.drive[0].lower()+'/'+str(p).replace('\\','/')[3:]
report=dict(schema='oasis.rtmpose.research_calibration.v1', purpose='historical_seated_research_candidate',
            target_arch='hailo10h', deployment_approved=False, current_ov9281_representativeness_verified=False,
            neutral_calibration_established=False, camera_control_enable_authorized=False,
            excluded=['current ceiling frame','random images','solid images','contact sheets','pose overlays'],
            calibration_count=128, holdout_count=32, roi_xyxy=[240,0,960,720],
            normalized_layout='NHWC', tensor_shape=[256,192,3], channels='BGR',
            mean=[123.675,116.28,103.53], std=[58.395,57.12,57.375],
            padding_scale=1.25, source_videos=[], frames=[])
for session in SESSIONS:
    src=GUI/'sessions'/session
    meta=json.loads((src/'metadata.json').read_text(encoding='utf-8'))
    known=json.loads((BENCH/'integrity'/session/'source_manifest.json').read_text(encoding='utf-8'))
    known_files={r['name']:r for r in known['files']}
    video=src/'camera.avi'
    before=video.stat()
    digest=sha(video)
    after=video.stat()
    if (before.st_size,before.st_mtime_ns)!=(after.st_size,after.st_mtime_ns) or digest!=known_files['camera.avi']['sha256']:
        raise RuntimeError('Source changed or fails prior full-video SHA256: '+session)
    ts=src/'frame_timestamps.csv'
    seen=set(); unique=[]
    with ts.open(newline='',encoding='utf-8') as f:
        rows=list(csv.DictReader(f))
    last=int(rows[-1]['video_frame_index'])
    for row in rows:
        index=int(row['video_frame_index']); capture=int(row['capture_index'])
        if capture in seen:continue
        seen.add(capture)
        if index<300 or index>last-300:continue
        if session=='S10' and 1250*30<=index<=1260*30:continue
        unique.append(row)
    chosen=[unique[round(i*(len(unique)-1)/79)] for i in range(80)]
    for i,row in enumerate(chosen):
        report['frames'].append(dict(session=session, source_video=wsl(video),
            video_frame_index=int(row['video_frame_index']), capture_index=int(row['capture_index']),
            host_monotonic_ns=int(row['host_monotonic_ns']),
            split='holdout' if i%5==0 else 'calibration', roi_xyxy=report['roi_xyxy']))
    report['source_videos'].append(dict(session=session, path_windows=str(video), path_wsl=wsl(video),
        bytes=before.st_size, sha256=digest, mtime_ns=before.st_mtime_ns,
        camera=meta.get('context',{}).get('camera'), timestamp_csv_sha256=sha(ts),
        prior_integrity_manifest=str(BENCH/'integrity'/session/'source_manifest.json'),
        matched_timestamp_frames=len(rows), distinct_capture_count=len(seen),
        status='incomplete_recording_but_video_decoded_prior' if session.endswith('f3d90508') else 'completed_recording'))
path=STAGE/'research_calibration_manifest.json'
with path.open('x',encoding='utf-8') as f:json.dump(report,f,indent=2)
print(json.dumps({'manifest':str(path),'sources':report['source_videos'],
                  'calibration_frames':sum(r['split']=='calibration' for r in report['frames']),
                  'holdout_frames':sum(r['split']=='holdout' for r in report['frames'])},indent=2))
