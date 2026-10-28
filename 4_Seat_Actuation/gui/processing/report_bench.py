"""Visual audit, data dictionary, and a local entry point for processed bench data."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from organize_bench import SENSORS, CAMERA_REVIEW
from pose_core import NAMES, draw
from prepare_bench import FEATURES, SESSIONS, sha256, write_json


def heatmaps(dest, session):
    points = pd.read_parquet(dest / 'points.parquet')
    if SESSIONS[session][1] == 784:
        x,y='right_yaw_deg','left_yaw_deg'
    else:
        x,y='left_yaw_deg','left_pitch_deg'
    fig, axes = plt.subplots(3,4,figsize=(16,11),layout='constrained')
    for ax,sensor in zip(axes.flat,SENSORS):
        matrix=points.pivot(index=y,columns=x,values=f'pressure_{sensor}_mean_v').sort_index()
        image=ax.imshow(matrix.to_numpy(),origin='lower',aspect='auto',
            extent=[matrix.columns.min()-.5,matrix.columns.max()+.5,matrix.index.min()-.5,matrix.index.max()+.5],
            cmap='viridis')
        ax.set(title=sensor,xlabel=x,ylabel=y)
        fig.colorbar(image,ax=ax,label='V (uncalibrated)')
    fig.suptitle(f'{SESSIONS[session][0]}: exact dwell-window pressure means')
    fig.savefig(dest/'pressure_by_pose.png',dpi=140)
    plt.close(fig)
    columns=['shoulder_tilt_deg_analysis_mean','torso_lean_deg_analysis_mean','head_offset_shoulder_ratio_analysis_mean','pose_analysis_valid_fraction']
    fig,axes=plt.subplots(2,2,figsize=(12,9),layout='constrained')
    for ax,column in zip(axes.flat,columns):
        matrix=points.pivot(index=y,columns=x,values=column).sort_index()
        image=ax.imshow(matrix.to_numpy(),origin='lower',aspect='auto',
            extent=[matrix.columns.min()-.5,matrix.columns.max()+.5,matrix.index.min()-.5,matrix.index.max()+.5],
            cmap='coolwarm' if column!='pose_target_valid_fraction' else 'viridis')
        ax.set(title=column,xlabel=x,ylabel=y)
        fig.colorbar(image,ax=ax)
    fig.suptitle('2D image measurements; single scan, no causal attribution')
    fig.savefig(dest/'pose_by_command.png',dpi=140)
    plt.close(fig)


def audit(source,dest):
    pose=pd.read_parquet(dest/'pose_unique.parquet')
    even=pose.iloc[np.linspace(0,len(pose)-1,8,dtype=int)].copy()
    low=pose.sort_values(['target_valid','valid_upper_joints']).head(4).copy()
    even['audit_reason']='even_time';low['audit_reason']='lowest_joint_validity'
    selected=pd.concat([even,low]).drop_duplicates('video_frame_index')
    flagged=pose[pose.camera_reposition_flag]
    if len(flagged):
        extra=flagged.iloc[[len(flagged)//2]].copy()
        extra['audit_reason']='camera_reposition'
        selected=pd.concat([selected,extra]).drop_duplicates('video_frame_index')
    selected.to_csv(dest/'visual_audit_samples.csv',index=False)
    cap=cv2.VideoCapture(str(source/'camera.avi'))
    tiles=[]
    for row in selected.to_dict('records'):
        cap.set(cv2.CAP_PROP_POS_FRAMES,int(row['video_frame_index']))
        ok,frame=cap.read()
        assert ok
        xy=np.array([[row[f'{k}_x_px'],row[f'{k}_y_px']] for k in NAMES])
        score=np.array([row[f'{k}_score'] for k in NAMES])
        valid=np.array([row[f'{k}_valid'] for k in NAMES])
        image=draw(frame,xy,score,valid,labels=(f"{row['source_session_s']:.1f}s / {row['audit_reason']}",
            f"valid joints: {row['valid_upper_joints']}/13"))
        tiles.append(cv2.resize(image,(310,300)))
    cap.release()
    while len(tiles)%4:
        tiles.append(np.zeros_like(tiles[0]))
    sheet=np.vstack([np.hstack(tiles[i:i+4]) for i in range(0,len(tiles),4)])
    (dest/'visual_audit.jpg').write_bytes(cv2.imencode('.jpg',sheet)[1].tobytes())
    fig,axes=plt.subplots(3,1,figsize=(15,9),sharex=True,layout='constrained')
    t=pose.source_session_s
    axes[0].plot(t,pose.nose_x_px,lw=.5,label='nose x')
    axes[0].plot(t,(pose.left_shoulder_x_px+pose.right_shoulder_x_px)/2,lw=.5,label='shoulder center x')
    axes[0].set_ylabel('full-image pixel');axes[0].legend()
    axes[1].plot(t,pose.valid_upper_joints,lw=.5)
    axes[1].set(ylabel='valid upper joints',ylim=(-.5,13.5))
    for feature in FEATURES[:2]:
        axes[2].plot(t,pose[feature],lw=.5,label=feature)
    axes[2].set(xlabel='host capture session time (s)',ylabel='2D degrees');axes[2].legend()
    fig.savefig(dest/'pose_time_qc.png',dpi=140);plt.close(fig)


def dictionary(output):
    table=pd.read_parquet(output/next(iter(SESSIONS))/'aligned_10hz.parquet')
    rows=[]
    for column in table.columns:
        if column.endswith('_x_px') or column.endswith('_y_px'):
            unit='pixel';definition='Raw COCO keypoint; full image x right/y down; use matching *_valid'
        elif column.endswith('_score'):
            unit='model score';definition='Raw SimCC confidence; threshold 0.5, not calibrated probability'
        elif column.endswith('_valid') or column=='target_valid':
            unit='boolean';definition='Observability and confidence gate; knees/ankles always false'
        elif column.startswith('pressure_'):
            if column.endswith(('_mean_v','_std_v','_min_v','_max_v')):
                unit='V';definition='Valid raw ADC within host 100ms bin; std uses ddof=0'
            elif column.endswith('_samples'):
                unit='count';definition='Valid sensor observations in bin; NaN if missing'
            else:
                unit='fraction';definition='Valid observations equal to raw zero/full-scale in bin'
        elif column.endswith('_ns'):
            unit='ns';definition='Host QPC unless explicitly MCU; retain int64 in source tables'
        elif column.endswith('_mdeg'):
            unit='0.001 degree';definition='Reported firmware command state/target; no mechanical feedback'
        elif column.endswith('_us'):
            unit='microsecond';definition='Firmware-commanded servo pulse width'
        elif column.endswith('_ms'):
            unit='millisecond';definition='Pose/motion age or raw MCU uptime, according to column name'
        elif column.endswith('_s'):
            unit='second';definition='Source/display/session time as described in METHODS.md'
        elif column.endswith('_deg'):
            unit='degree';definition='2D image feature; no camera/3D calibration'
        elif column.endswith('_px'):
            unit='pixel';definition='2D body size feature; uncalibrated distance'
        else:
            unit='see METHODS';definition='Source ID/state or derived feature; no imputation'
        rows.append(dict(column=column,dtype=str(table[column].dtype),unit=unit,definition=definition))
    pd.DataFrame(rows).to_csv(output/'aligned_data_dictionary.csv',index=False)


def main():
    parser=argparse.ArgumentParser(__doc__)
    parser.add_argument('--input',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--model',type=Path,required=True)
    args=parser.parse_args()
    cv2.setNumThreads(1)
    reports=[]
    for session in SESSIONS:
        dest=args.output/session
        heatmaps(dest,session);audit(args.input/session,dest)
        reports.append(json.loads((dest/'quality_report.json').read_text()))
        print(f'{session}: plots/audit complete',flush=True)
    dictionary(args.output)
    model_dir=args.output/'models';model_dir.mkdir(exist_ok=True)
    shutil.copyfile(args.model,model_dir/'rtmpose_m.onnx')
    code_dir=args.output/'processing_source';code_dir.mkdir(exist_ok=True)
    here=Path(__file__).resolve().parent
    source_manifest=[]
    for file in sorted(here.iterdir()):
        if file.is_file() and file.suffix in ['.py','.md','.txt']:
            shutil.copyfile(file,code_dir/file.name)
            source_manifest.append(dict(filename=file.name,sha256=sha256(file)))
    shutil.copyfile(here/'README.md',args.output/'METHODS.md')
    commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=here,text=True).strip()
    write_json(args.output/'processing_manifest.json',dict(dataset_id='bench_pose_v1',
        processing_parent_commit=commit,source_files=source_manifest,
        model=dict(name='RTMPose-m body7 COCO17 256x192',sha256=sha256(args.model),
        source='https://download.openmmlab.com/mmpose/v1/projects/rtmposev1/onnx_sdk/rtmpose-m_simcc-body7_pt-body7_420e-256x192-e48f03d0_20230504.zip'),
        sessions=list(SESSIONS),sampling=dict(video_fps=30,pose_stride=3,overlay_fps=10),
        raw_modified=False,processing='offline OpenVINO CPU / DirectML GPU; no imputation or smoothing'))
    pd.DataFrame([dict(session_id=s,start_video_s=a,end_video_s=b,
        reason='Camera reposition observed visually; conservative review interval, raw values retained',
        review_frames='7344,7359,7386,7518,7800,8100') for s,(a,b) in CAMERA_REVIEW.items()]).to_csv(args.output/'manual_quality_flags.csv',index=False)
    lines=['# 벤치 실측 정리','',
        '정상 종료된 3개 세션, **총 1,498개 측정 구간**의 압력·명령각·영상 포즈를 정리했다.',
        '원본 세션 파일은 수정하지 않았다. 사용법과 단위·시계·결측 규칙은 [METHODS.md](METHODS.md)를 따른다.','',
        '- [분석용 간단 표: 각도·압력 12채널·2D 포즈·품질](analysis_points.csv)',
        '- [전체 자세별 통합 CSV](all_points.csv) / [Parquet](all_points.parquet)',
        '- [세션 요약](sessions.csv) / [10Hz 통합 표 컬럼 사전](aligned_data_dictionary.csv)',
        '- 포즈 모델: RTMPose-m, 중앙 크롭 620×600, COCO 17관절, 10Hz 추정.',
        '- 무릎·발목은 화면 밖이므로 invalid. 골반은 가림/신뢰도를 함께 확인한다.',
        '- 왼쪽 시험 239–280초는 카메라 재조준/손 뻗기 구간으로 표시했다. `*_analysis_mean`은 검토 대상 샘플을 제외한 평균이다.',
        '- 카메라 조정 전·중·후를 `camera_epoch=0/1/2`로 구분했다. 전후 영상 좌표계는 재보정되지 않았다.',
        '- 압력은 보정 전 ADC·V이며 인체 2D 스켈레톤은 실측 기구 각도가 아니다.','',
        '| 시험 | 자세 | 영상 길이 | 포즈 샘플 | 착석자 유효율 | 어깨 유효율 | 골반 유효율 |',
        '|---|---:|---:|---:|---:|---:|---:|']
    for r in reports:
        lines.append(f"| [{r['label']}]({r['session']}/points.csv) | {r['points']} | {r['source_duration_s']/60:.2f}분 | {r['pose_samples']:,} | {r['target_valid_fraction']:.2%} | {r['shoulder_valid_fraction']:.2%} | {r['hip_valid_fraction']:.2%} |")
    lines+=['','총 영상 표시 프레임: '+f"{sum(r['cfr_frames'] for r in reports):,}",
        '총 10Hz 포즈 샘플: '+f"{sum(r['pose_samples'] for r in reports):,}",
        '전체 압력 원시 값: '+f"{sum(r['pressure_rows'] for r in reports):,}",'',
        '세 세션 모두 압력 패킷 누락·ADC 오류·dwell PWM 불일치 0. 기존 자세별 평균과 재집계값 차이 <1e−8V.',
        'MCU 시계와 PC 시계의 속도 차이 때문에 통합 표는 공통 PC QPC 시각으로 맞췄다.',
        '최근접 포즈는 ±100ms, 과거 상태 보고는 최대 500ms까지만 결합하며 시간차를 열로 남긴다.','']
    for r in reports:
        s=r['session']
        lines += [f"## {r['label']}",'',
            f'- [스켈레톤 영상]({s}/pose_overlay.mp4)',
            f'- [압력·명령·포즈 10Hz 통합 표]({s}/aligned_10hz.parquet)',
            f'- [압력 히트맵]({s}/pressure_by_pose.png) / [포즈 히트맵]({s}/pose_by_command.png)',
            f'- [육안 확인용 샘플]({s}/visual_audit.jpg) / [시계열 QC]({s}/pose_time_qc.png)',
            f'- [검사 결과]({s}/quality_report.json) / [입력 SHA-256]({s}/source_manifest.json)','']
    (args.output/'README.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')


if __name__=='__main__':
    main()
