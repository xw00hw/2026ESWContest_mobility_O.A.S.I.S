"""Read-only pose QC inputs and identical skeleton rendering."""
from pathlib import Path
import json
import cv2
import numpy as np
import pandas as pd
from common import LM, BONES, read_frame, color

ROOT = Path(__file__).resolve().parents[2]
R2 = ROOT / '01_refined/run2'
QC = ROOT / '02_qc/5_pose_correction'
WORK = ROOT / '_work/5_pose_correction'
GROUPS = {g: [n for n in LM if n.endswith(s)] for g, s in [('shoulders', 'shoulder'), ('elbows', 'elbow'), ('wrists', 'wrist'), ('hips', 'hip')]}
GROUPS['head'] = ['nose', 'left_ear', 'right_ear']
LABELS = dict(zip(LM, ['nose', 'Lea', 'Rea', 'Lsh', 'Rsh', 'Lel', 'Rel', 'Lwr', 'Rwr', 'Lhi', 'Rhi']))
VARIANTS = ['rtm_x', 'rtm_x_flip', 'rtm_x_bbox2', 'rtmw_x', 'mp_heavy_raw', 'mp_heavy_clahe', 'mp_heavy_roi', 'mp_full_raw']

def log(message):
    print(message, flush=True)

def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8', newline='\n')

def write_csv(path, data):
    data.to_csv(path, index=False, encoding='utf-8', lineterminator='\n', float_format='%.6f', na_rep='')

def load():
    names = ['camera_frames', 'pose', 'pose_landmarks', 'pose_landmarks_corrected', 'pose_landmarks_reinferred', 'pose_corrected', 'pose_before_after', 'pose_frame_diagnosis', 'obd', 'imu']
    d = {n: pd.read_csv(R2 / (n + '.csv'), low_memory=False).copy() for n in names}
    camera = d['camera_frames'].set_index('frame_id')
    before = d['pose'].set_index('frame_id')
    for name in names[2:8]:
        table = d[name]
        table['t_run_s'] = table.frame_id.map(camera.t_run_s).fillna(table.frame_id.map(before.t_run_s))
        assert table.t_run_s.notna().all(), name
    d['b'] = d['pose_frame_diagnosis'].query('has_before == 1').reset_index(drop=True).fillna('')
    d['a'] = d['pose_frame_diagnosis'].query('is_recorded == 1').reset_index(drop=True).fillna('')
    assert len(d['b']) == len(d['pose_before_after']) == len(before)
    assert len(d['a']) == len(camera)
    d['camera_index'] = camera
    d['after_index'] = d['pose_landmarks_corrected'].set_index('frame_id')
    d['before_index'] = d['pose_landmarks'].set_index('frame_id')
    d['steering'] = np.interp(d['pose_before_after'].t_run_s, d['obd'].t_run_s, d['obd'].steering_angle_deg)
    return d

def points(row, after=False):
    return {n: np.array([row[n+'_x']*1280, row[n+'_y']*800]) for n in LM
            if (row.get(n+'_reliable', 0) == 1 if after else row.get('detected', 1) == 1 and row.get(n+'_vis', 0) >= .5)
            and np.isfinite([row[n+'_x'], row[n+'_y']]).all()}

def skeleton(image, pts, width, title='', raw=None, thin=False, long_labels=False):
    scale = width / 1280
    out = cv2.resize(image, (width, int(width*800/1280)))
    coords = {n: tuple(np.rint(p*scale).astype(int)) for n, p in pts.items()}
    for a, b in BONES:
        if a in coords and b in coords:
            cv2.line(out, coords[a], coords[b], color(b), 1 if thin else 2, cv2.LINE_AA)
    for n, q in coords.items():
        cv2.circle(out, q, 4, color(n), -1, cv2.LINE_AA)
        cv2.circle(out, q, 4, (255,255,255), 1, cv2.LINE_AA)
        label = n if long_labels else LABELS[n]
        for c, th in [((0,0,0),3), ((255,255,255),1)]:
            cv2.putText(out, label, (q[0]+6,q[1]-5), cv2.FONT_HERSHEY_SIMPLEX, .4, c, th, cv2.LINE_AA)
    if raw is not None:
        for n in LM:
            p = np.array([raw[n+'_x_reinfer'],raw[n+'_y_reinfer']])*[1280,800]*scale
            if raw[n+'_reliable'] == 0 and np.isfinite(p).all():
                q = tuple(np.rint(p).astype(int))
                cv2.circle(out,q,6,(150,150,150),2,cv2.LINE_AA)
                cv2.putText(out,(n if long_labels else LABELS[n])+'?',(q[0]+8,q[1]),cv2.FONT_HERSHEY_SIMPLEX,.4,(180,180,180),1,cv2.LINE_AA)
    if title:
        cv2.rectangle(out,(0,0),(width,32),(0,0,0),-1)
        cv2.putText(out,title,(10,24),cv2.FONT_HERSHEY_SIMPLEX,.65,(255,255,255),2,cv2.LINE_AA)
    return out

def pair(d, before_id, after_id, width, titles, raw=False, flip=False, long_labels=False, skip=()):
    frame = d['camera_index'].loc[after_id]
    image = read_frame(frame.segment_file, frame.frame_in_segment)
    br = d['before_index'].loc[before_id] if before_id is not None else None
    ar = d['after_index'].loc[after_id]
    bp = points(br) if br is not None else {}
    ap = points(ar, True)
    bp, ap = [{k: v for k, v in p.items() if k not in skip} for p in (bp, ap)]   # joints hidden in both panels
    panels = [bp, ap] if not flip else [ap, bp]
    result = [skeleton(image,p,width,titles[i],raw=ar if raw and ((i==1) != flip) else None,long_labels=long_labels) for i,p in enumerate(panels)]
    return np.hstack([result[0],np.full((result[0].shape[0],4,3),255,np.uint8),result[1]]), panels

def sample_strata(times, strata, spacing, n, rng):
    selected = []
    for name, indices, quota in strata:
        if quota <= 0:
            continue
        taken = 0
        for i in indices:
            i = int(i)
            if all(abs(times[i]-times[j]) >= spacing for j,_ in selected):
                selected.append((i,name))
                taken += 1
                if taken >= quota or len(selected) >= n:
                    break
        if len(selected) >= n:
            break
    if len(selected) < n:
        for i in rng.permutation(len(times)):
            if all(abs(times[i]-times[j]) >= spacing for j,_ in selected):
                selected.append((int(i),'random_fill'))
                if len(selected) >= n:
                    break
    return selected
