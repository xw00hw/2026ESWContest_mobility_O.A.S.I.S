"""Diagnose and correct cached poses without inference or modifying BEFORE files."""
import os
os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
os.environ.setdefault('OMP_NUM_THREADS', '1')
import sys
sys.dont_write_bytecode = True
import argparse
import csv
import hashlib
import importlib.util
import json
import time
import warnings
from decimal import Decimal
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pandas as pd
from scipy.ndimage import binary_dilation
from common import LM, MP_IDX, BONES, CACHE, load_camera_frames

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / '01_refined/run2'
QC = ROOT / '02_qc/5_pose_correction'
WORK = ROOT / '_work/5_pose_correction'
VARIANTS = ['rtm_x', 'rtm_x_flip', 'rtm_x_bbox2', 'rtmw_x', 'mp_heavy_raw',
            'mp_heavy_clahe', 'mp_heavy_roi', 'mp_full_raw']
DETECTORS = ['low_conf', 'tta', 'roi', 'model', 'mp', 'jump', 'bone', 'angle', 'swap', 'region', 'overlap', 'person']
BDETECTORS = ['not_detected', 'low_vis', 'jump', 'bone', 'angle', 'swap', 'region', 'ref', 'mp']
HARD = ['tta', 'roi', 'model', 'jump', 'bone', 'angle', 'swap', 'region', 'overlap', 'person']
PAIRS = [(LM.index('left_' + p), LM.index('right_' + p)) for p in ['ear', 'shoulder', 'elbow', 'wrist', 'hip']]
EDGES = [(3, 5), (5, 7), (4, 6), (6, 8), (3, 9), (4, 10), (3, 4), (9, 10), (0, 2)]
ARMS = [(3, 5, 7), (4, 6, 8)]
GEOMETRY = [f'{p}_center_{a}' for p in ['shoulder', 'hip', 'head'] for a in ['x', 'y']] + ['shoulder_width', 'torso_angle_deg']
FEATURES = GEOMETRY + ['shoulder_shift_x', 'shoulder_shift_y', 'hip_shift_x', 'hip_shift_y',
                     'head_shift_x', 'head_shift_y', 'shoulder_shift_px', 'head_shift_px',
                     'lean_score', 'lean_velocity', 'pose_confidence', 'landmark_valid_mask', 'landmarks_visible']
COMPARE = [f for f in FEATURES if f not in ['lean_velocity', 'landmark_valid_mask', 'landmarks_visible']]
SCALE = np.array([1280., 800.])


def log(message):
    print(message, flush=True)


def clean(value):
    if isinstance(value, dict):
        return {str(k): clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, np.ndarray)):
        return [clean(v) for v in value]
    if isinstance(value, np.generic):
        return clean(value.item())
    if isinstance(value, float) and not np.isfinite(value):
        return None
    return value


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(clean(value), ensure_ascii=False, indent=2, allow_nan=False) + '\n',
                    encoding='utf-8', newline='\n')


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for part in iter(lambda: f.read(1024 * 1024), b''):
            h.update(part)
    return h.hexdigest()


def fixed(value, digits=None):
    if value is None or value is pd.NA:
        return ''
    if isinstance(value, str):
        return value
    if isinstance(value, (bool, np.bool_)):
        return str(int(value))
    if isinstance(value, (int, np.integer)):
        return str(value)
    if not np.isfinite(value):
        return ''
    text = f'{value:.{digits}f}' if digits is not None else format(Decimal(str(value)), 'f')
    text = text.rstrip('0').rstrip('.') if '.' in text else text
    return '0' if text == '-0' else text


def write_csv(path, columns, rows, digits):
    path.parent.mkdir(parents=True, exist_ok=True)
    precision = [digits.get(c) for c in columns]
    temporary = path.with_suffix('.csv.tmp')
    with temporary.open('w', encoding='utf-8', newline='', buffering=1024*1024) as f:
        writer = csv.writer(f, lineterminator='\n')
        writer.writerow(columns)
        for row in rows:
            assert len(row) == len(columns), path
            writer.writerow([fixed(v, d) for v, d in zip(row, precision)])
    temporary.replace(path)


def feature_digits(column):
    return (4 if column == 'pose_confidence' or 'torso_angle' in column
            else 3 if column.endswith(('_px', '_ms'))
            else 5 if column.endswith(('_x', '_y')) or column in ['shoulder_width', 'lean_score', 'lean_velocity']
            else None)


def blocks(labels):
    cuts = np.r_[0, np.flatnonzero(np.diff(labels) != 0) + 1, len(labels)]
    return list(zip(cuts[:-1], cuts[1:]))


def neighbors(t, runs, window, exclude_self=True):
    # Actual time bounds and run bounds both constrain each padded neighborhood.
    left, right = np.empty(len(t), int), np.empty(len(t), int)
    radius = int(round(window * 1e9))
    for a, b in blocks(runs):
        left[a:b] = a + np.searchsorted(t[a:b], t[a:b] - radius, side='left')
        right[a:b] = a + np.searchsorted(t[a:b], t[a:b] + radius, side='right')
    width = int(np.max(right - left))
    ids = left[:, None] + np.arange(width)
    valid = ids < right[:, None]
    if exclude_self:
        valid &= ids != np.arange(len(t))[:, None]
    return np.minimum(ids, len(t)-1), valid


def neighbor_median(xy, ids, valid, accepted=None):
    gathered = xy[ids].copy()
    good = valid[..., None] & np.isfinite(gathered).all(axis=-1)
    if accepted is not None:
        good &= accepted[ids]
    gathered[~good] = np.nan
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', RuntimeWarning)
        median = np.nanmedian(gathered, axis=1)
    return median, good.sum(axis=1)


def distance(a, b):
    return np.linalg.norm(a - b, axis=-1)


def elbow_angle(a, b, c):
    u, v = a - b, c - b
    denom = np.linalg.norm(u, axis=-1) * np.linalg.norm(v, axis=-1)
    cosine = np.divide(np.sum(u*v, axis=-1), denom, out=np.full(denom.shape, np.nan), where=denom > 0)
    return np.degrees(np.arccos(np.clip(cosine, -1, 1)))


def learn_limits(xy, base, params):
    limits = {'bones': [], 'angles': [], 'regions': [], 'reference_rule': 'finite P and low_conf/tta/roi/model all false'}
    for a, b in EDGES:
        values = distance(xy[:, a], xy[:, b])[base[:, a] & base[:, b]]
        values = values[np.isfinite(values)]
        if not len(values):
            raise ValueError(f'No reference frames for bone {LM[a]}-{LM[b]}')
        lo, hi = np.quantile(values, [params['bone_q_lo'], params['bone_q_hi']])
        limits['bones'].append({'joints': [LM[a], LM[b]], 'reference_count': len(values),
                                'quantiles_px': [lo, hi], 'limits_px': [lo * params['bone_margin_lo'], hi * params['bone_margin_hi']]})
    for a, b, c in ARMS:
        values = elbow_angle(xy[:, a], xy[:, b], xy[:, c])[base[:, a] & base[:, b] & base[:, c]]
        values = values[np.isfinite(values)]
        if not len(values):
            raise ValueError(f'No reference angles for {LM[b]}')
        lo, hi = np.quantile(values, [params['bone_q_lo'], params['bone_q_hi']])
        limits['angles'].append({'joints': [LM[a], LM[b], LM[c]], 'reference_count': len(values),
                                 'quantiles_deg': [lo, hi], 'limits_deg': [lo - params['angle_margin_deg'], hi + params['angle_margin_deg']]})
    masks = {}
    for k, name in enumerate(LM):
        points = xy[base[:, k], k]
        if not len(points):
            raise ValueError(f'No reference region for {name}')
        cells = np.floor(points / params['region_bin']).astype(int)
        pad = params['region_dilate']
        origin = cells.min(axis=0) - pad
        shape = cells.max(axis=0) - origin + pad + 1
        hist = np.zeros(tuple(shape), np.int64)
        np.add.at(hist, tuple((cells-origin).T), 1)
        mask = hist >= params['region_min_count']
        if pad:
            mask = binary_dilation(mask, structure=np.ones((3, 3), bool), iterations=pad)
        masks[name] = mask
        masks[name + '_origin'] = origin
        limits['regions'].append({'joint': name, 'reference_count': len(points), 'origin_bin': origin,
                                  'shape': shape, 'supported_cells': int(mask.sum()), 'bin_px': params['region_bin'],
                                  'min_count': params['region_min_count'], 'dilate_cells': pad,
                                  'dilation': 'square (8-connected)'})
    return limits, masks


def region_inside(points, k, masks, params):
    finite = np.isfinite(points).all(axis=-1)
    cells = np.floor(np.nan_to_num(points) / params['region_bin']).astype(np.int64) - masks[LM[k] + '_origin']
    mask = masks[LM[k]]
    inside = finite & (cells[:, 0] >= 0) & (cells[:, 1] >= 0) & (cells[:, 0] < mask.shape[0]) & (cells[:, 1] < mask.shape[1])
    result = np.zeros(len(points), bool)
    result[inside] = mask[cells[inside, 0], cells[inside, 1]]
    return result


def geometric_flags(xy, limits):
    bone, angle = np.zeros(xy.shape[:2], bool), np.zeros(xy.shape[:2], bool)
    for (a, b), item in zip(EDGES, limits['bones']):
        d = distance(xy[:, a], xy[:, b])
        lo, hi = item['limits_px']
        bad = np.isfinite(d) & ((d < lo) | (d > hi))
        bone[:, a] |= bad
        bone[:, b] |= bad
    for (a, b, c), item in zip(ARMS, limits['angles']):
        d = elbow_angle(xy[:, a], xy[:, b], xy[:, c])
        lo, hi = item['limits_deg']
        bad = np.isfinite(d) & ((d < lo) | (d > hi))
        angle[:, b] |= bad
        angle[:, c] |= bad
    return bone, angle


def candidate_geometry(candidate, k, primary, limits):
    valid = np.isfinite(candidate).all(axis=-1)
    for (a, b), item in zip(EDGES, limits['bones']):
        if k in (a, b):
            other = b if k == a else a
            d = distance(candidate, primary[:, other])
            lo, hi = item['limits_px']
            valid &= (d >= lo) & (d <= hi)
    for (a, b, c), item in zip(ARMS, limits['angles']):
        if k in (a, b, c):
            d = elbow_angle(candidate if k == a else primary[:, a],
                            candidate if k == b else primary[:, b],
                            candidate if k == c else primary[:, c])
            lo, hi = item['limits_deg']
            valid &= (d >= lo) & (d <= hi)
    return valid


def swap_flags(xy, ref, margin, reliable=None):
    result = np.zeros(xy.shape[:2], bool)
    for a, b in PAIRS:
        crossed = distance(xy[:, a], ref[:, b]) + distance(xy[:, b], ref[:, a])
        same = distance(xy[:, a], ref[:, a]) + distance(xy[:, b], ref[:, b])
        bad = crossed + margin < same
        if reliable is not None:
            bad &= reliable[:, a] & reliable[:, b]
        result[:, a] = result[:, b] = bad
    return result


def diagnose_after(data, t, runs, params):
    p, f, b, w = [data[v]['xy'] for v in VARIANTS[:4]]
    score = data['rtm_x']['score']
    det = {'low_conf': (score < params['t_conf']) | ~np.isfinite(score) | ~np.isfinite(p).all(axis=-1),
           'tta': distance(p, f) > params['t_tta'], 'roi': distance(p, b) > params['t_roi'],
           'model': distance(p, w) > params['t_model']}
    # Far (left) ear: hidden behind the head in this right-profile view; the model drops it on the brow,
    # the hairline or the near ear even with a high score (QA findings seg_004_r2, seg_000_r1), so it is
    # treated as structurally occluded and never marked reliable.
    base = ~(det['low_conf'] | det['tta'] | det['roi'] | det['model'])
    limits, masks = learn_limits(p, base, params)
    det['low_conf'][:, LM.index('left_ear')] = True   # applied after the limits were learned from score-based references
    mp = np.stack([data[v]['xy'] for v in VARIANTS[4:7]])
    mp_score = np.stack([data[v]['score'] for v in VARIANTS[4:7]])
    consistent = np.isfinite(mp).all(axis=(0, 3)) & (np.min(mp_score, axis=0) >= params['t_mp_vis'])
    for a, b in [(0, 1), (0, 2), (1, 2)]:
        consistent &= distance(mp[a], mp[b]) <= params['t_mp_self']
    det['mp'] = consistent & (distance(p, np.median(mp, axis=0)) > params['t_mp'])
    ids, valid = neighbors(t, runs, params['jump_win_s'])
    med, count = neighbor_median(p, ids, valid)
    det['jump'] = (count >= params['jump_min_neighbors']) & (distance(p, med) > params['t_jump'])
    det['bone'], det['angle'] = geometric_flags(p, limits)
    det['swap'] = swap_flags(p, f, params['t_swap_margin'])
    det['region'] = np.column_stack([~region_inside(p[:, k], k, masks, params) for k in range(len(LM))])
    # Overlap: a far-side arm joint drawn on top of a near-side arm joint is hidden behind it, and an elbow drawn
    # on its own wrist is a collapsed limb (QA finding seg_000_r1) -> the hidden/collapsed joint is flagged.
    ix = {n: LM.index(n) for n in ['left_elbow', 'right_elbow', 'left_wrist', 'right_wrist']}
    det['overlap'] = np.zeros(score.shape, bool)
    for hidden, other in [('left_elbow', 'right_elbow'), ('left_elbow', 'right_wrist'), ('left_wrist', 'right_elbow'),
                          ('left_wrist', 'right_wrist'), ('left_elbow', 'left_wrist'), ('right_elbow', 'right_wrist')]:
        det['overlap'][:, ix[hidden]] |= distance(p[:, ix[hidden]], p[:, ix[other]]) < params['t_overlap']
    return det, limits, masks, ids, valid


def interpolate(xy, score, reliable, action, source, t, runs, params):
    for a, b in blocks(runs):
        for k in range(len(LM)):
            good = np.flatnonzero(reliable[a:b, k]) + a
            for left, right in zip(good[:-1], good[1:]):
                if right-left <= 1 or t[right]-t[left] > params['interp_max_gap_s']*1e9:
                    continue
                ids = np.arange(left+1, right)
                weight = (t[ids]-t[left]) / (t[right]-t[left])
                xy[ids, k] = xy[left, k] + weight[:, None]*(xy[right, k]-xy[left, k])
                score[ids, k] = score[left, k] + weight*(score[right, k]-score[left, k])
                reliable[ids, k] = True
                action[ids, k] = 'interpolated'
                source[ids, k] = 'interp'


def smooth(xy, reliable, t, runs, params):
    result = xy.copy()
    for k in range(len(LM)):
        # Every missing point and every camera gap starts a new smoothing group.
        cut = np.r_[True, (runs[1:] != runs[:-1]) | ~reliable[1:, k] | ~reliable[:-1, k]]
        groups = np.cumsum(cut)
        ids, valid = neighbors(t, groups, params['smooth_half_win_s'], exclude_self=False)
        valid &= reliable[ids, k] & reliable[:, k, None]
        count = valid.sum(axis=1)
        use = np.flatnonzero(count >= params['smooth_min_pts'])
        if not len(use):
            continue
        ids, valid = ids[use], valid[use]
        dt = (t[ids]-t[use, None]) / 1e9 / params['smooth_half_win_s']
        design = np.stack([np.ones_like(dt), dt, dt*dt], axis=-1)
        design *= valid[..., None]
        coords = np.nan_to_num(xy[ids, k])
        matrix = np.einsum('nwi,nwj->nij', design, design)
        rhs = np.einsum('nwi,nwc->nic', design, coords)
        result[use, k] = np.linalg.solve(matrix, rhs)[:, 0]
    return result


def person_occlusion(frames, p, t, runs, params):
    """occluded_by_other_person: a joint inside the box of a person other than the driver (YOLOX person boxes,
    pipeline/5_pose_correction/detect_people.py; QA findings seg_000_r4 / seg_001_r4 - passenger in front of the camera).
    Driver box = box with the largest IoU with the fixed driver ROI (if IoU >= person_driver_iou); boxes overlapping it
    by IoU >= person_driver_iou are duplicates. Other boxes must be >= person_min_height_px tall and touch the left or
    bottom frame edge (within person_edge_px) - people seen through the windows are ignored. Gaps <= person_fill_gap_s inside a
    camera run are filled so an occlusion episode is flagged as a whole."""
    roi = np.array([0, 40, 1060, 800], float)
    hit = np.zeros(p.shape[:2], bool)
    cache = {}
    for i, (sid, fis) in enumerate(zip(frames.segment_id.to_numpy(), frames.frame_in_segment.to_numpy())):
        if sid not in cache:
            cache = {sid: np.load(CACHE / 'yolox_person' / f'seg_{int(sid):03d}.npz')['boxes']}
        b = cache[sid][fis]
        b = b[np.isfinite(b[:, 4])]
        if not len(b):
            continue
        ix = np.clip(np.minimum(b[:, 2], roi[2]) - np.maximum(b[:, 0], roi[0]), 0, None) * np.clip(np.minimum(b[:, 3], roi[3]) - np.maximum(b[:, 1], roi[1]), 0, None)
        iou = ix / ((b[:, 2]-b[:, 0])*(b[:, 3]-b[:, 1]) + (roi[2]-roi[0])*(roi[3]-roi[1]) - ix)
        keep = np.ones(len(b), bool)
        if iou.max() >= params['person_driver_iou']:
            keep[int(np.argmax(iou))] = False
        if not keep.all():   # duplicate boxes of the driver are not another person
            d = b[~keep][0]
            jx = np.clip(np.minimum(b[:, 2], d[2]) - np.maximum(b[:, 0], d[0]), 0, None) * np.clip(np.minimum(b[:, 3], d[3]) - np.maximum(b[:, 1], d[1]), 0, None)
            keep &= jx / ((b[:, 2]-b[:, 0])*(b[:, 3]-b[:, 1]) + (d[2]-d[0])*(d[3]-d[1]) - jx) < params['person_driver_iou']
        keep &= (b[:, 3]-b[:, 1]) >= params['person_min_height_px']
        # the passenger enters the view from the camera side (left / bottom edge); people seen through the windows do not
        keep &= (b[:, 0] <= params['person_edge_px']) | (b[:, 3] >= 800 - params['person_edge_px'])
        for x1, y1, x2, y2, _ in b[keep]:
            hit[i] |= (p[i, :, 0] >= x1) & (p[i, :, 0] <= x2) & (p[i, :, 1] >= y1) & (p[i, :, 1] <= y2)
    for k in range(hit.shape[1]):
        idx = np.flatnonzero(hit[:, k])
        for a, b in zip(idx[:-1], idx[1:]):
            if b - a > 1 and runs[a] == runs[b] and t[b] - t[a] <= params['person_fill_gap_s'] * 1e9:
                hit[a:b, k] = True
    return hit


def choose(data, det, limits, masks, ids, valid, t, runs, params, manual=None):
    primary = data['rtm_x']['xy']
    estimates = np.stack([data[v]['xy'] for v in VARIANTS[:4]], axis=1)
    scores = np.stack([data[v]['score'] for v in VARIANTS[:4]], axis=1)
    c0 = ~np.logical_or.reduce([det[d] for d in ['low_conf'] + HARD])
    reliable = c0.copy()
    xy = np.where(c0[..., None], primary, np.nan)
    vis = np.clip(data['rtm_x']['score'], 0, 1)
    action = np.where(c0, 'smoothed_only', 'flagged_unreliable').astype('U32')
    source = np.where(c0, 'rtm_x', '').astype('U24')
    med, count = neighbor_median(primary, ids, valid, c0)
    candidates = [('median4', np.median(estimates, axis=1), None),
                  ('rtmw_x', estimates[:, 3], 3), ('rtm_x_flip', estimates[:, 1], 1),
                  ('rtm_x_bbox2', estimates[:, 2], 2)]
    for name, candidate, idx in candidates:
        agree = (distance(estimates, candidate[:, None]) <= params['t_agree']) & np.isfinite(scores)
        num = agree.sum(axis=1)
        mean_score = np.divide(np.where(agree, scores, 0).sum(axis=1), num,
                               out=np.zeros(num.shape, float), where=num > 0)
        passing = (num >= params['min_agree']) & (mean_score >= params['t_conf'])
        passing &= (count < params['jump_min_neighbors']) | (distance(candidate, med) <= params['t_jump'])
        for k in range(len(LM)):
            passing[:, k] &= region_inside(candidate[:, k], k, masks, params)
            passing[:, k] &= candidate_geometry(candidate[:, k], k, primary, limits)
        take = ~reliable & passing & ~det['low_conf'] & ~det['overlap'] & ~det['person']   # occluded (low score / hidden behind another joint): never rescued by other models
        xy[take] = candidate[take]
        vis[take] = np.clip(mean_score[take] if idx is None else scores[:, idx][take], 0, 1)
        reliable[take] = True
        action[take] = 'corrected_by_constraint'
        source[take] = name
    interpolate(xy, vis, reliable, action, source, t, runs, params)
    # Reliable islands shorter than min_track_frames next to an unreliable frame are flicker (QA finding seg_000_r1).
    short = np.zeros(reliable.shape, bool)
    for k in range(len(LM)):
        r = reliable[:, k]
        edge = np.flatnonzero(np.r_[True, (r[1:] != r[:-1]) | (runs[1:] != runs[:-1]), True])
        for a, b in zip(edge[:-1], edge[1:]):
            touches = (a > 0 and runs[a-1] == runs[a] and not r[a-1]) or (b < len(r) and runs[b] == runs[b-1] and not r[b])
            if r[a] and b-a < params['min_track_frames'] and touches:
                short[a:b, k] = True
    if manual is not None:   # joints QA confirmed wrong on enlarged renders (params qa_overrides); exact ranges only
        short |= manual & reliable
    reliable[short] = False
    xy[short] = np.nan
    vis[short] = np.clip(data['rtm_x']['score'], 0, 1)[short]
    action[short] = 'flagged_unreliable'
    source[short] = ''
    presmooth = xy.copy()
    xy = smooth(xy, reliable, t, runs, params)
    reason = np.full(reliable.shape, '', dtype='U32')
    for d, word in reversed([('low_conf', 'low_confidence'), ('person', 'occluded_by_other_person'), ('overlap', 'overlap_occluded'), ('swap', 'lr_ambiguous'),
                            ('region', 'off_body'), ('bone', 'bone_length'),
                            ('angle', 'joint_angle'), ('jump', 'temporal_jump')]):
        reason[det[d] & ~reliable] = word
    reason[~reliable & (reason == '')] = 'model_disagreement'
    reason[short] = 'short_track'
    if manual is not None:
        reason[manual & short] = 'qa_manual'
    assert np.array_equal(~reliable, reason != '')
    assert np.array_equal(~reliable, action == 'flagged_unreliable')
    return xy, vis, reliable, action, source, reason, distance(xy, presmooth)


def status_after(det):
    status = np.full(det['low_conf'].shape, 'ok', dtype='U40')
    status[det['tta'] | det['roi'] | det['model']] = 'misplaced_model_disagree'
    for d, word in reversed([('low_conf', 'occluded'), ('person', 'occluded'), ('overlap', 'occluded'), ('swap', 'misplaced_swap_lr'),
                            ('region', 'misplaced_off_body'), ('bone', 'misplaced_bone_length'),
                            ('angle', 'misplaced_joint_angle'), ('jump', 'misplaced_jump')]):
        status[det[d]] = word
    return status


def detector_strings(det, order):
    result = np.full(next(iter(det.values())).shape, '', dtype=object)
    for d in order:
        mask = det[d]
        prev = result[mask]
        result[mask] = [v + ('+' if v else '') + d for v in prev]
    return result


def load_collector():
    spec = importlib.util.spec_from_file_location('e_pose_collector', ROOT / '_work/3_pi_gps_imu_camera/src_ref/src_oasis_daq_pose_features.py')
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def nearest_speed(frames, gps, params):
    valid = gps.loc[(gps.valid == 1) & gps.speed_kmh.notna()]
    if valid.empty:
        return np.full(len(frames), np.nan)
    t, query = valid.t_utc_ns.to_numpy(np.int64), frames.t_utc_ns.to_numpy(np.int64)
    right = np.minimum(np.searchsorted(t, query), len(t)-1)
    left = np.maximum(right-1, 0)
    near = np.where(np.abs(t[left]-query) <= np.abs(t[right]-query), left, right)
    speed = valid.speed_kmh.to_numpy()[near].copy()
    speed[np.abs(t[near]-query) > params['gps_match_max_s']*1e9] = np.nan
    return speed


def feature_pass(frames, xy, vis, reliable, primary, primary_score, collector, params, neutral=None):
    records, previous = [], None
    runs = frames['_run'].to_numpy()
    for i, row in enumerate(frames.itertuples(index=False)):
        if row.flag_gap_before or (i and runs[i] != runs[i-1]):
            previous = None
        coords = np.where(reliable[i, :, None], xy[i], primary[i]) / SCALE
        visibility = np.where(reliable[i], np.maximum(vis[i], params['feature_min_vis']),
                              np.minimum(np.nan_to_num(primary_score[i]), params['feature_unreliable_max_vis']))
        points = [SimpleNamespace(x=0., y=0., z=0., visibility=0., presence=0.) for _ in range(33)]
        for k, mp_idx in enumerate(MP_IDX):
            points[mp_idx] = SimpleNamespace(x=float(coords[k, 0]), y=float(coords[k, 1]), z=0.,
                                             visibility=float(visibility[k]), presence=float(visibility[k]))
        f = collector.extract_features(points, frame_id=int(row.frame_id),
            capture_monotonic_ns=int(row.capture_monotonic_ns), host_monotonic_ns=int(row.capture_monotonic_ns),
            host_wall_time_ns=int(row.t_utc_ns), neutral=neutral, previous=previous,
            image_width=640, image_height=400, latency_ms=None)
        records.append(f.to_row())
        previous = f
        if i and i % 5000 == 0:
            log(f'features {i}/{len(frames)} neutral={neutral is not None}')
    return pd.DataFrame(records)


def kst(ns):
    return pd.Timestamp(int(ns), unit='ns', tz='UTC').tz_convert('Asia/Seoul').isoformat()


def estimate_neutral(base, frames, gps, collector, params):
    valid = base.loc[base.vision_valid == 1]
    driving = valid.loc[valid.speed_kmh_nearest >= params['neutral_min_speed_kmh']]
    typical = driving[GEOMETRY].median()
    t = frames.capture_monotonic_ns.to_numpy(np.int64)
    wall = frames.t_utc_ns.to_numpy(np.int64)
    def wall_at(mono):
        pos = min(np.searchsorted(t, mono), len(t)-1)
        return int(wall[pos]) + int(mono) - int(t[pos])
    windows = []
    window_ns, step_ns = int(params['neutral_window_s']*1e9), int(params['neutral_step_s']*1e9)
    for start in range(int(t[0]), int(t[-1])-window_ns+1, step_ns):
        end = start+window_ns
        all_mask = (t >= start) & (t < end)
        total = int(all_mask.sum())
        if not total or frames.loc[all_mask, '_run'].nunique() != 1:
            continue
        part = valid.loc[(valid.capture_monotonic_ns >= start) & (valid.capture_monotonic_ns < end)]
        if len(part) < max(params['neutral_min_frames'], params['neutral_valid_fraction']*total):
            continue
        scatter = max(part.shoulder_center_x.std(ddof=1)*640, part.shoulder_center_y.std(ddof=1)*400)
        if scatter > params['neutral_max_scatter_px']:
            continue
        sw, ew = wall_at(start), wall_at(end)
        speed = gps.loc[(gps.valid == 1) & (gps.t_utc_ns >= sw) & (gps.t_utc_ns < ew), 'speed_kmh'].dropna()
        steady = len(speed) >= params['neutral_min_gps_epochs'] and speed.min() >= params['neutral_min_speed_kmh'] and speed.max()-speed.min() <= params['neutral_max_speed_range_kmh']
        values = part[GEOMETRY].mean()
        if not values.notna().all() or not typical.notna().all() or typical.shoulder_width <= 0:
            continue
        score = np.sum(((values.iloc[:6]-typical.iloc[:6])*np.array([640, 400]*3))**2)
        score += ((values.shoulder_width-typical.shoulder_width)/typical.shoulder_width*100)**2
        score += (values.torso_angle_deg-typical.torso_angle_deg)**2
        windows.append({'start_capture_monotonic_ns': start, 'end_capture_monotonic_ns': end,
                        'start_t_utc_ns': sw, 'end_t_utc_ns': ew, 'start_kst': kst(sw), 'end_kst': kst(ew),
                        'stored_frames': total, 'valid_frames': len(part), 'scatter_px': scatter,
                        'valid_gps_epochs': len(speed), 'speed_min_kmh': speed.min(), 'speed_max_kmh': speed.max(),
                        'steady': bool(steady), 'score': score, 'values': values.to_dict()})
    candidates = sorted([w for w in windows if w['steady']], key=lambda w: (w['score'], w['start_capture_monotonic_ns']))
    fallback = False
    if candidates:
        selected = candidates[0]
        values = selected['values']
        method, used, established = 'stable_window_steady_speed', selected['valid_frames'], selected['end_capture_monotonic_ns']
    else:
        fallback = True
        stable = sorted(windows, key=lambda w: (w['score'], w['start_capture_monotonic_ns']))
        selected = stable[0] if stable else None
        values = selected['values'] if selected else typical.to_dict()
        method = 'stable_window_any_speed' if selected else 'session_median_fallback'
        used, established = (selected['valid_frames'], selected['end_capture_monotonic_ns']) if selected else (len(driving), int(t[-1]))
    complete = all(np.isfinite(values.get(c, np.nan)) for c in GEOMETRY)
    neutral = collector.NeutralPosture(shoulder_center=tuple([values['shoulder_center_x'], values['shoulder_center_y'], 0.]),
        hip_center=tuple([values['hip_center_x'], values['hip_center_y'], 0.]),
        head_center=tuple([values['head_center_x'], values['head_center_y'], 0.]),
        shoulder_width=values['shoulder_width'], torso_angle_deg=values['torso_angle_deg'], frames_used=used,
        established_monotonic_ns=established, valid=complete, reason=method)
    before = json.loads((OUT/'pose_neutral.json').read_text(encoding='utf-8'))['neutral']
    delta = {p+'_center_delta_px': [(values[p+'_center_x']-before[p+'_center'][0])*640,
                                   (values[p+'_center_y']-before[p+'_center'][1])*400] for p in ['shoulder', 'hip', 'head']}
    delta['shoulder_width_delta_px'] = (values['shoulder_width']-before['shoulder_width'])*640
    delta['torso_angle_delta_deg'] = values['torso_angle_deg']-before['torso_angle_deg']
    info = {'method': method, 'rules': {k: v for k, v in params.items() if k.startswith(('neutral_', 'gps_match_'))},
            'rule_note': '첫 저장 프레임 기준 [start,end) 창. 표본 표준편차, 전형 주행 자세와 중심 거리 제곱+너비 백분율 차 제곱+몸통 각도 차 제곱 최소, 동률이면 이른 창. 후보 없으면 파트 C와 같은 any-speed/주행 중앙값 fallback.',
            'candidate_windows': len(candidates), 'stable_windows': len(windows), 'selected_window': selected,
            'neutral': neutral.to_dict(), 'before_neutral': before, 'difference_from_before': delta,
            'fallback': fallback}
    return neutral, info


def diagnose_before(before, landmark_text, data, near, frames, xy, reliable, action, limits, masks, params):
    n = len(before)
    bxy = np.array([[[float(row.get(lm+'_'+a) or 'nan') for a in ['x', 'y']] for lm in LM]
                    for row in landmark_text], float) * SCALE
    bvis = np.array([[float(row.get(lm+'_vis') or 'nan') for lm in LM] for row in landmark_text])
    detected = np.array([int(row['detected']) for row in landmark_text])
    t = before.capture_monotonic_ns.to_numpy(np.int64)
    # BEFORE points inside a camera gap belong to the interval following its start.
    camera_t = frames.capture_monotonic_ns.to_numpy(np.int64)
    prev = np.clip(np.searchsorted(camera_t, t, side='right')-1, 0, len(camera_t)-1)
    bruns = frames['_run'].to_numpy()[prev]
    ids, valid = neighbors(t, bruns, params['b_jump_win_s'])
    med, count = neighbor_median(bxy, ids, valid)
    ref, rgood = xy[near], reliable[near]
    disp = distance(bxy, ref)
    det = {'not_detected': np.repeat((detected == 0)[:, None], len(LM), axis=1),
           'low_vis': (bvis < params['b_min_vis']) | ~np.isfinite(bvis),
           'jump': (count >= params['jump_min_neighbors']) & (distance(bxy, med) > params['t_b_jump'])}
    det['bone'], det['angle'] = geometric_flags(bxy, limits)
    det['swap'] = swap_flags(bxy, ref, params['t_swap_margin'], rgood)
    det['region'] = np.column_stack([~region_inside(bxy[:, k], k, masks, params) for k in range(len(LM))])
    det['ref'] = rgood & (disp > params['t_mis'])
    det['mp'] = (distance(bxy, data['mp_heavy_raw']['xy'][near]) > params['t_mis']) & (data['mp_heavy_raw']['score'][near] >= params['b_min_vis'])
    status = np.full((n, len(LM)), 'ok', dtype='U40')
    status[det['low_vis']] = 'low_visibility'
    status[~rgood] = 'occluded'
    for d, word in reversed([('not_detected', 'not_detected'), ('swap', 'misplaced_swap_lr'),
                            ('region', 'misplaced_off_body'), ('bone', 'misplaced_bone_length'),
                            ('angle', 'misplaced_joint_angle'), ('jump', 'misplaced_jump'),
                            ('ref', 'misplaced_model_disagree')]):
        status[det[d]] = word
    outcome = action[near].copy()
    outcome[rgood & (outcome == 'smoothed_only')] = 'replaced_by_reinference'
    outcome[rgood & (status == 'ok') & (disp <= params['t_mis'])] = 'kept'
    assert np.all(status != '')
    return bxy, bvis, detected, det, status, outcome, disp


def counts(array, vocabulary=None):
    words = vocabulary if vocabulary is not None else sorted(np.unique(array).tolist())
    return {lm: {word: int(np.count_nonzero(array[:, k] == word)) for word in words}
            for k, lm in enumerate(LM)}


def quantiles(values):
    finite = np.asarray(values)[np.isfinite(values)]
    return {'n': len(finite), **{f'p{q}': float(np.percentile(finite, q)) if len(finite) else None for q in [50, 90, 99]}}


def stats_table(frames, adet, astatus, action, reason, reliable, movement, bdet, bstatus, outcome, disp, near):
    detector_matrix = np.stack([adet[d] for d in DETECTORS], axis=-1).reshape(-1, len(DETECTORS)).astype(np.int64)
    after = {'detectors': {lm: {d: int(adet[d][:, k].sum()) for d in DETECTORS} for k, lm in enumerate(LM)},
             'detector_pair_order': DETECTORS, 'detector_pair_coactivation': detector_matrix.T @ detector_matrix,
             'frames_any_detector': {name: int(np.logical_or.reduce([adet[d] for d in ds]).any(axis=1).sum())
                 for name, ds in [('hard', HARD), ('hard_with_mp', HARD+['mp']),
                                  ('hard_with_low_conf', HARD+['low_conf']), ('all', DETECTORS)]},
             'after_status': counts(astatus), 'action': counts(action), 'reason': counts(reason),
             'reliable_ratio': {lm: float(reliable[:, k].mean()) for k, lm in enumerate(LM)},
             'reliable_ratio_by_segment': {str(int(s)): {lm: float(reliable[frames.segment_id.to_numpy() == s, k].mean())
                                                        for k, lm in enumerate(LM)} for s in sorted(frames.segment_id.unique())},
             'smoothing_displacement_px': {lm: quantiles(movement[:, k]) for k, lm in enumerate(LM)}}
    before = {'detectors': {lm: {d: int(bdet[d][:, k].sum()) for d in BDETECTORS} for k, lm in enumerate(LM)},
              'before_status': counts(bstatus), 'before_outcome': counts(outcome),
              'displacement_px': {lm: quantiles(disp[:, k]) for k, lm in enumerate(LM)},
              'status_x_after_final': {word: {'reliable': int(((bstatus == word) & reliable[near]).sum()),
                                            'unreliable': int(((bstatus == word) & ~reliable[near]).sum())}
                                       for word in sorted(np.unique(bstatus).tolist())}}
    a_sums = {lm: sum(after['after_status'][lm].values()) for lm in LM}
    b_sums = {lm: sum(before['before_status'][lm].values()) for lm in LM}
    assert all(v == len(frames) for v in a_sums.values())
    assert all(v == len(near) for v in b_sums.values())
    assert all(sum(after['action'][lm].values()) == len(frames) for lm in LM)
    assert all(sum(before['before_outcome'][lm].values()) == len(near) for lm in LM)
    return {'segments': sorted(int(s) for s in frames.segment_id.unique()), 'after': after, 'before': before,
            'assertions': {'after_rows': len(frames), 'before_rows': len(near), 'after_status_sums': a_sums,
                           'before_status_sums': b_sums, 'status_action_reason_invariants': True}}


def handoff(path, segments):
    header, *lines = path.read_bytes().splitlines(keepends=True)
    cols = next(csv.reader([header.decode('utf-8')]))
    sid_col = cols.index('segment_id')
    grouped = {s: [] for s in segments}
    for line in lines:
        sid = int(next(csv.reader([line.decode('utf-8')]))[sid_col])
        if sid in grouped:
            grouped[sid].append(line)
    for sid in segments:
        if not grouped[sid]:
            raise ValueError(f'No output rows for handoff segment {sid}')
        content = header + b''.join(grouped[sid])
        digest = hashlib.sha256(content).hexdigest()
        folder = WORK/'handoff'/f'seg_{sid:03d}'
        folder.mkdir(parents=True, exist_ok=True)
        target = folder/f'pose_landmarks_corrected_seg_{sid:03d}.csv'
        done = folder/'DONE.json'
        previous = json.loads(done.read_text(encoding='utf-8')) if done.exists() else None
        if previous and previous['csv_sha256'] == digest and target.exists() and sha(target) == digest:
            log(f'handoff {sid:03d}: unchanged revision {previous["revision"]}')
            continue
        revision = 1 if previous is None else previous['revision'] + int(previous['csv_sha256'] != digest)
        temporary = target.with_suffix('.csv.tmp')
        temporary.write_bytes(content)
        temporary.replace(target)
        payload = {'segment': sid, 'rows': len(grouped[sid]), 'revision': revision,
                   'schema_note': 'x/y는 1280x800 프레임의 정규화 좌표(x/1280,y/800)입니다. unreliable 관절은 x/y가 빈 칸이며 *_x_reinfer/*_y_reinfer는 주 모델 원시 추정입니다. reliable=1은 채택 또는 짧은 보간 후 신뢰 가능한 값, 0은 결측입니다. reason은 unreliable 사유(신뢰 가능하면 빈 칸), action은 smoothed_only/corrected_by_constraint/interpolated/flagged_unreliable 처리, source는 채택 모델/median4/interp(결측이면 빈 칸)입니다.',
                   'landmark_order': LM, 'skeleton_edges': [list(edge) for edge in BONES], 'csv_sha256': digest}
        temporary = done.with_suffix('.json.tmp')
        write_json(temporary, payload)
        temporary.replace(done)
        log(f'handoff {sid:03d}: {len(grouped[sid])} rows revision {revision}')


def output_tables(out, frames, data, xy, vis, reliable, action, source, reason, features,
                  before, before_text, landmark_text, near, detected, bstatus, outcome, disp,
                  astatus, astrings, bstrings):
    meta = ['t_utc_ns', 'frame_id', 'capture_monotonic_ns', 'segment_id', 'frame_in_segment']
    metadata = frames[meta].to_numpy(np.int64)
    cols = meta.copy()
    digits = {}
    for v in VARIANTS:
        cols.append(v+'_detected')
        for lm in LM:
            for field, places in [('x', 5), ('y', 5), ('score', 4)]:
                col = f'{v}_{lm}_{field}'
                cols.append(col)
                digits[col] = places
    def reinferred_rows():
        for i in range(len(frames)):
            row = list(metadata[i])
            for v in VARIANTS:
                row.append(data[v]['detected'][i])
                for k in range(len(LM)):
                    row.extend([data[v]['xy'][i, k, 0]/1280, data[v]['xy'][i, k, 1]/800, data[v]['score'][i, k]])
            yield row
    log('write pose_landmarks_reinferred.csv')
    write_csv(out/'pose_landmarks_reinferred.csv', cols, reinferred_rows(), digits)
    meta2 = ['t_utc_ns', 'frame_id', 'capture_monotonic_ns', 'segment_id', 'segment_file', 'frame_in_segment', 'flag_gap_before']
    columns = meta2 + ['n_reliable']
    digits = {}
    for lm in LM:
        columns += [f'{lm}_{f}' for f in ['x', 'y', 'vis', 'reliable', 'reason', 'action', 'source', 'x_reinfer', 'y_reinfer', 'score_reinfer']]
        digits.update({f'{lm}_{f}': p for f, p in [('x', 5), ('y', 5), ('vis', 4), ('x_reinfer', 5), ('y_reinfer', 5), ('score_reinfer', 4)]})
    def corrected_rows():
        for i, meta_row in enumerate(frames[meta2].itertuples(index=False, name=None)):
            row = list(meta_row) + [int(reliable[i].sum())]
            for k in range(len(LM)):
                row.extend([xy[i, k, 0]/1280, xy[i, k, 1]/800, vis[i, k], reliable[i, k], reason[i, k],
                            action[i, k], source[i, k], data['rtm_x']['xy'][i, k, 0]/1280,
                            data['rtm_x']['xy'][i, k, 1]/800, data['rtm_x']['score'][i, k]])
            yield row
    log('write pose_landmarks_corrected.csv')
    write_csv(out/'pose_landmarks_corrected.csv', columns, corrected_rows(), digits)
    fcols = FEATURES + ['torso_angle_delta_deg', 'neutral_valid', 'vision_valid', 'shift_valid', 'dt_prev_s',
                       'flag_gap_before', 'flag_low_visibility', 'flag_neutral_fallback', 'speed_kmh_nearest']
    def feature_rows():
        for m, f in zip(metadata, features[fcols].itertuples(index=False, name=None)):
            yield list(m) + list(f)
    write_csv(out/'pose_corrected.csv', meta+fcols, feature_rows(), {c: feature_digits(c) for c in fcols})
    columns = ['t_utc_ns', 'frame_id', 'before_detected', 'after_frame_id', 'after_t_utc_ns', 'after_dt_ms', 'after_exact_frame']
    digits = {'after_dt_ms': 3}
    for lm in LM:
        columns += [f'{lm}_{c}' for c in ['x_before', 'y_before', 'vis_before', 'x_after', 'y_after', 'vis_after',
                                         'reliable_after', 'disp_px', 'before_status', 'before_outcome']]
        digits.update({f'{lm}_{c}': d for c, d in [('x_after', 5), ('y_after', 5), ('vis_after', 4), ('disp_px', 1)]})
    for f in COMPARE:
        columns += [f+'_before', f+'_after']
        digits[f+'_after'] = feature_digits(f)
    columns += ['vision_valid_before', 'vision_valid_after', 'shift_valid_before', 'shift_valid_after']
    frows = features.to_dict('records')
    def comparison_rows():
        for i, original in enumerate(before_text):
            j = near[i]
            row = [original['t_utc_ns'], original['frame_id'], detected[i], metadata[j, 1], metadata[j, 0],
                   (metadata[j, 2]-int(original['capture_monotonic_ns']))/1e6, int(metadata[j, 1] == int(original['frame_id']))]
            for k, lm in enumerate(LM):
                row += [landmark_text[i][lm+'_x'], landmark_text[i][lm+'_y'], landmark_text[i][lm+'_vis'],
                        xy[j, k, 0]/1280, xy[j, k, 1]/800, vis[j, k], reliable[j, k], disp[i, k], bstatus[i, k], outcome[i, k]]
            for f in COMPARE:
                row += [original[f], frows[j][f]]
            row += [original['vision_valid'], frows[j]['vision_valid'], original['shift_valid'], frows[j]['shift_valid']]
            yield row
    write_csv(out/'pose_before_after.csv', columns, comparison_rows(), digits)
    columns = ['t_utc_ns', 'frame_id', 'has_before', 'is_recorded', 'after_frame_id', 'segment_id', 'frame_in_segment']
    for lm in LM:
        columns += [f'{lm}_{c}' for c in ['before_status', 'before_detectors', 'after_status', 'after_detectors',
                                         'disp_px', 'action', 'before_outcome', 'final', 'reason']]
    by_before = {int(row['frame_id']): i for i, row in enumerate(before_text)}
    by_after = {int(v): i for i, v in enumerate(metadata[:, 1])}
    union = sorted(set(by_before) | set(by_after))
    def diagnosis_rows():
        has_before_count = 0
        for fid in union:
            bi, ai = by_before.get(fid), by_after.get(fid)
            if ai is None:
                ai = near[bi]
            has_before, recorded = bi is not None, fid in by_after
            timestamp = int(before_text[bi]['t_utc_ns']) if has_before else metadata[ai, 0]
            row = [timestamp, fid, has_before, recorded, metadata[ai, 1], metadata[ai, 3], metadata[ai, 4]]
            has_before_count += has_before
            for k in range(len(LM)):
                assert astatus[ai, k] and action[ai, k]
                assert (not reliable[ai, k]) == bool(reason[ai, k]) == (action[ai, k] == 'flagged_unreliable')
                assert not has_before or bstatus[bi, k]
                row += [bstatus[bi, k] if has_before else '', bstrings[bi, k] if has_before else '',
                        astatus[ai, k], astrings[ai, k], disp[bi, k] if has_before else np.nan, action[ai, k],
                        outcome[bi, k] if has_before else '', 'reliable' if reliable[ai, k] else 'unreliable', reason[ai, k]]
            yield row
        assert has_before_count == len(before)
    write_csv(out/'pose_frame_diagnosis.csv', columns, diagnosis_rows(), {lm+'_disp_px': 1 for lm in LM})
    return len(union)


def selftest(params):
    t = np.arange(40, dtype=np.int64)*40_000_000
    runs = np.r_[np.zeros(20, int), np.ones(20, int)]
    xy = np.repeat(np.column_stack([np.arange(40)*2., np.arange(40)*3.])[:, None], len(LM), axis=1)
    xy[20:] += 500
    raw = xy.copy()
    raw[8] += 200
    ids, valid = neighbors(t, runs, params['jump_win_s'])
    median, count = neighbor_median(raw, ids, valid)
    jump = (count >= params['jump_min_neighbors']) & (distance(raw, median) > params['t_jump'])
    assert jump[8].all() and not jump[19:21].any()
    reliable = np.ones((40, len(LM)), bool)
    reliable[10:13] = False
    reliable[19:21] = False
    xy[~reliable] = np.nan
    score = np.ones(reliable.shape)
    action = np.full(reliable.shape, 'smoothed_only', dtype='U32')
    source = np.full(reliable.shape, 'rtm_x', dtype='U24')
    interpolate(xy, score, reliable, action, source, t, runs, params)
    assert reliable[10:13].all() and (action[10:13] == 'interpolated').all()
    assert not reliable[19:21].any()
    expected = np.column_stack([np.arange(10, 13)*2., np.arange(10, 13)*3.])
    assert np.allclose(xy[10:13, 0], expected)
    smoothed = smooth(xy, reliable, t, runs, params)
    assert np.allclose(smoothed[reliable], xy[reliable])
    assert np.isnan(smoothed[19:21]).all()
    # With valid boundary samples, a large run offset must still remain intact.
    complete = np.repeat(np.column_stack([np.arange(40)*2., np.arange(40)*3.])[:, None], len(LM), axis=1)
    complete[20:] += 500
    assert np.allclose(smooth(complete, np.ones(reliable.shape, bool), t, runs, params), complete)
    log('SELFTEST PASS: jump, 3-frame interpolation, run-boundary interpolation/smoothing isolation')


def parse_segments(value, available):
    result = available if value == 'all' else sorted(set(int(s) for s in value.split(',')))
    if not result or any(s not in available for s in result):
        raise ValueError(f'Invalid segments: {value}; available={available}')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--segments', help='Subset development outputs go to _work/5_pose_correction/subset')
    parser.add_argument('--handoff', help='Comma-separated segment IDs or all')
    parser.add_argument('--selftest', action='store_true')
    args = parser.parse_args()
    params = json.loads((Path(__file__).with_name('params.json')).read_text(encoding='utf-8-sig'))
    if any(not isinstance(v, (int, float)) or not np.isfinite(v) or v < 0 for k, v in params.items() if k != 'qa_overrides'):
        parser.error('All parameters must be finite nonnegative numbers')
    if not (0 <= params['bone_q_lo'] < params['bone_q_hi'] <= 1):
        parser.error('Bone quantiles must satisfy 0 <= lo < hi <= 1')
    for key in ['region_bin', 'jump_win_s', 'b_jump_win_s', 'smooth_half_win_s', 'neutral_window_s', 'neutral_step_s']:
        if params[key] <= 0:
            parser.error(f'{key} must be positive')
    for key in ['region_dilate', 'region_min_count', 'min_agree', 'smooth_min_pts', 'jump_min_neighbors']:
        if params[key] != int(params[key]):
            parser.error(f'{key} must be an integer')
        params[key] = int(params[key])
    if not (1 <= params['min_agree'] <= 4) or params['smooth_min_pts'] < 3 or params['jump_min_neighbors'] < 2:
        parser.error('min_agree must be 1..4, smooth_min_pts >=3, jump_min_neighbors >=2')
    if args.selftest:
        selftest(params)
        return
    started = time.perf_counter()
    originals = {name: sha(OUT/name) for name in ['pose_landmarks.csv', 'pose.csv']}
    all_frames = load_camera_frames()
    available = sorted(int(s) for s in all_frames.segment_id.unique())
    segments = parse_segments(args.segments or 'all', available)
    selected = all_frames.segment_id.isin(segments).to_numpy()
    original_runs = np.cumsum(all_frames.flag_gap_before.to_numpy())
    indices = np.flatnonzero(selected)
    frames = all_frames.loc[selected].reset_index(drop=True)
    runs = original_runs[selected].copy()
    # Non-adjacent selected segments must not become temporal neighbors.
    extra = np.cumsum(np.r_[False, np.diff(indices) != 1])
    runs = runs + extra*(int(original_runs.max())+1)
    frames['_run'] = runs
    t = frames.capture_monotonic_ns.to_numpy(np.int64)
    assert np.all(np.diff(t) > 0) and frames.frame_id.is_unique
    data = {}
    for v in VARIANTS:
        chunks = []
        for sid in segments:
            path = WORK/'cache'/v/f'seg_{sid:03d}.npz'
            with np.load(path, allow_pickle=False) as cache:
                expected = frames.loc[frames.segment_id == sid, 'frame_in_segment'].to_numpy()
                assert np.array_equal(cache['frame_in_segment'], expected), path
                chunk = {k: cache[k].copy() for k in ['xy', 'score', 'detected']}
                assert chunk['xy'].shape == (len(expected), len(LM), 2), path
                assert chunk['score'].shape == (len(expected), len(LM)), path
                assert chunk['detected'].shape == (len(expected),), path
                chunks.append(chunk)
        data[v] = {k: np.concatenate([c[k] for c in chunks]).astype(float if k != 'detected' else np.uint8)
                   for k in ['xy', 'score', 'detected']}
    log(f'loaded {len(frames)} frames; segments={segments}; diagnose AFTER')
    det, limits, masks, ids, valid = diagnose_after(data, t, runs, params)
    det['person'] = person_occlusion(frames, data['rtm_x']['xy'], t, runs, params)
    manual = np.zeros((len(frames), len(LM)), bool)
    for o in params.get('qa_overrides', []):
        rows = ((frames.segment_id == o['segment']) & frames.frame_in_segment.between(o['frames'][0], o['frames'][1])).to_numpy()
        manual[rows, slice(None) if o['landmark'] == 'all' else LM.index(o['landmark'])] = True   # 'all' = frame mask
    xy, vis, reliable, action, source, reason, movement = choose(data, det, limits, masks, ids, valid, t, runs, params, manual)
    astatus = status_after(det)
    astrings = detector_strings(det, DETECTORS)
    log('compute collector features and neutral')
    collector = load_collector()
    gps = pd.read_csv(OUT/'gps.csv', usecols=['t_utc_ns', 'valid', 'speed_kmh'], dtype={'t_utc_ns': 'int64'})
    features = feature_pass(frames, xy, vis, reliable, data['rtm_x']['xy'], data['rtm_x']['score'], collector, params)
    features['speed_kmh_nearest'] = nearest_speed(frames, gps, params)
    neutral, neutral_info = estimate_neutral(features, frames, gps, collector, params)
    features = feature_pass(frames, xy, vis, reliable, data['rtm_x']['xy'], data['rtm_x']['score'], collector, params, neutral)
    features['speed_kmh_nearest'] = nearest_speed(frames, gps, params)
    features['torso_angle_delta_deg'] = (features.torso_angle_deg-neutral.torso_angle_deg).round(4)
    features['shift_valid'] = ((features.neutral_valid == 1) & features.shoulder_shift_x.notna()).astype(int)
    features['dt_prev_s'] = frames.capture_monotonic_ns.diff().astype(float)/1e9
    features['flag_gap_before'] = frames.flag_gap_before
    features['flag_low_visibility'] = (~reliable[:, [3, 4, 9, 10]].all(axis=1)).astype(int)
    features['flag_neutral_fallback'] = int(neutral_info['fallback'])
    before_all = pd.read_csv(OUT/'pose.csv', dtype={'t_utc_ns': 'int64', 'capture_monotonic_ns': 'int64'})
    by_frame = {int(fid): i for i, fid in enumerate(frames.frame_id)}
    mask = before_all.nearest_recorded_frame_id.isin(by_frame)
    before = before_all.loc[mask].reset_index(drop=True)
    with (OUT/'pose.csv').open(encoding='utf-8', newline='') as f:
        before_text = [row for row, keep in zip(csv.DictReader(f), mask) if keep]
    with (OUT/'pose_landmarks.csv').open(encoding='utf-8', newline='') as f:
        landmark_lookup = {int(row['frame_id']): row for row in csv.DictReader(f)}
    landmark_text = [landmark_lookup[int(row['frame_id'])] for row in before_text]
    near = np.array([by_frame[int(fid)] for fid in before.nearest_recorded_frame_id], int)
    assert len(before) == len(before_text)
    if not args.segments:
        assert len(before) == len(before_all)
    log(f'diagnose BEFORE {len(before)} frames')
    _, _, detected, bdet, bstatus, outcome, disp = diagnose_before(before, landmark_text, data, near, frames,
                                                               xy, reliable, action, limits, masks, params)
    bstrings = detector_strings(bdet, BDETECTORS)
    out = WORK/'subset' if args.segments else OUT
    qc = WORK/'subset/qc' if args.segments else QC
    out.mkdir(parents=True, exist_ok=True)
    qc.mkdir(parents=True, exist_ok=True)
    limits['segments'] = segments
    limits['params'] = params
    write_json(qc/'kinematic_limits.json', limits)
    np.savez(qc/'region_masks.npz', **masks)
    write_json(out/'pose_neutral_corrected.json', neutral_info)
    union_rows = output_tables(out, frames, data, xy, vis, reliable, action, source, reason, features,
                              before, before_text, landmark_text, near, detected, bstatus, outcome, disp,
                              astatus, astrings, bstrings)
    stats = stats_table(frames, det, astatus, action, reason, reliable, movement, bdet, bstatus, outcome, disp, near)
    stats['assertions']['diagnosis_rows'] = union_rows
    stats['assertions']['original_sha256'] = originals
    stats['params'] = params
    write_json(qc/'diag_stats.json', stats)
    assert originals == {name: sha(OUT/name) for name in originals}, 'BEFORE files changed'
    if args.handoff:
        handoff(out/'pose_landmarks_corrected.csv', parse_segments(args.handoff, segments))
    log(f'COMPLETE {time.perf_counter()-started:.2f}s after={len(frames)} before={len(before)} union={union_rows} out={out}')


if __name__ == '__main__':
    main()
