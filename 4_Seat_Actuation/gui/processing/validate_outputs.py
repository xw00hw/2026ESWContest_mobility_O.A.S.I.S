"""Read back derived tables and decode every output video frame."""
import argparse
import json
from pathlib import Path
import subprocess

import imageio_ffmpeg
import numpy as np
import pandas as pd

from pose_core import NAMES, ROI, THRESHOLD
from prepare_bench import SESSIONS, write_json


def validate(dest):
    qc=json.loads((dest/'quality_report.json').read_text())
    pose=pd.read_parquet(dest/'pose_unique.parquet')
    samples=pd.read_parquet(dest/'pose_samples.parquet')
    aligned=pd.read_parquet(dest/'aligned_10hz.parquet')
    points=pd.read_parquet(dest/'points.parquet')
    frame_index=pd.read_parquet(dest/'frame_pose_index.parquet')
    assert len(pose)==pose.capture_index.nunique()==qc['unique_inferences']
    assert len(samples)==qc['pose_samples']
    assert samples.video_frame_index.mod(3).eq(0).all()
    assert len(frame_index)==qc['cfr_frames']
    assert np.array_equal(frame_index.video_frame_index,np.arange(len(frame_index)))
    assert len(points)==SESSIONS[dest.name][1]
    assert points.point_index.nunique()==len(points)
    assert points.pose_samples.sum() <= len(pose)
    assert aligned.pose_age_ms.dropna().between(0,100).all()
    assert aligned.motion_age_ms.dropna().between(0,500).all()
    for i,name in enumerate(NAMES):
        mask=pose[f'{name}_valid']
        if i>=13:
            assert not mask.any()
        assert pose.loc[mask,f'{name}_score'].ge(THRESHOLD).all()
        assert pose.loc[mask,f'{name}_x_px'].between(ROI[0],ROI[2],inclusive='left').all()
        assert pose.loc[mask,f'{name}_y_px'].between(ROI[1],ROI[3],inclusive='left').all()
    # Fail on recoverable decoder errors as well as a nonzero exit code.
    result=subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(),'-hide_banner','-loglevel','error',
        '-threads','1','-i',str(dest/'pose_overlay.mp4'),'-map','0:v:0','-an',
        '-progress','pipe:1','-nostats','-f','null','-'],capture_output=True,text=True,check=True)
    assert not result.stderr.strip(), result.stderr
    progress=dict(line.split('=',1) for line in result.stdout.splitlines() if '=' in line)
    assert int(progress['frame'])==len(samples)
    assert progress['progress']=='end'
    report=dict(session=dest.name,tables='passed',video_full_decode='passed',
        decoded_video_frames=int(progress['frame']),points=len(points),pose_samples=len(samples),
        pose_unique_captures=len(pose),all_17_joint_masks_checked=True,
        aligned_pose_tolerance_ms=100,motion_max_age_ms=500)
    write_json(dest/'validation.json',report)
    print(json.dumps(report),flush=True)


def main():
    parser=argparse.ArgumentParser(__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--session',action='append',choices=list(SESSIONS))
    parser.add_argument('--finalize',action='store_true',help='Check completed validation records and root index only')
    args=parser.parse_args()
    if not args.finalize:
        for session in args.session or SESSIONS:
            validate(args.output/session)
    if not args.session:
        for session in SESSIONS:
            checked=json.loads((args.output/session/'validation.json').read_text())
            assert checked['tables']==checked['video_full_decode']=='passed'
            assert checked['points']==SESSIONS[session][1]
        all_points=pd.read_parquet(args.output/'all_points.parquet')
        compact=pd.read_csv(args.output/'analysis_points.csv')
        assert len(all_points)==len(compact)==1498
        assert not all_points.duplicated(['session_id','point_index']).any()
        write_json(args.output/'COMPLETE.json',dict(dataset='bench_pose_v1',
            status='complete',sessions=3,measurement_windows=1498,
            output_videos_full_decode='passed',raw_data_modified=False))


if __name__=='__main__':
    main()
