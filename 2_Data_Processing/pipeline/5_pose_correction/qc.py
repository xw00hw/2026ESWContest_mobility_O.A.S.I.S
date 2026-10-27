"""Deterministic read-only QC and report for S2 driver pose outputs."""
import json
import subprocess
import cv2
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from qc_helpers import *

def quantile(values, q):
    values = np.asarray(values)
    values = values[np.isfinite(values)]
    return float(np.quantile(values,q)) if len(values) else None

def motion(xy, times, runs):
    dt = np.diff(times)
    valid = (dt > 0) & (dt <= .5) & (np.diff(runs) == 0)
    speed = np.linalg.norm(np.diff(xy,axis=0),axis=1) / np.where(dt>0,dt,np.nan)
    speed = speed[valid & np.isfinite(speed)]
    triple = valid[:-1] & valid[1:]
    jitter = np.linalg.norm(xy[2:]-2*xy[1:-1]+xy[:-2],axis=1)
    jitter = jitter[triple & np.isfinite(jitter)]
    return {'speed_p50_px_s':quantile(speed,.5),'speed_p95_px_s':quantile(speed,.95),'speed_p99_px_s':quantile(speed,.99),
            'jump_ratio':float(np.mean(speed>1500)) if len(speed) else None,'jitter_median_px':quantile(jitter,.5), 'n_speed_pairs':len(speed)}

def bone_stats(values):
    med = quantile(values,.5)
    return {'bone_median_px':med, 'bone_mad_over_median':quantile(np.abs(values-med),.5)/med if med else None,
            'bone_p99_p1_over_median':(quantile(values,.99)-quantile(values,.01))/med if med else None}

def coordinates(table, xfield='x', yfield='y', prefix=''):
    return np.stack([table[[prefix+n+'_'+xfield,prefix+n+'_'+yfield]].to_numpy(float)*[1280,800] for n in LM],axis=1)

def proxy(d):
    b,a,m = d['pose_landmarks'],d['pose_landmarks_corrected'],d['pose_before_after']
    camera = d['camera_index']
    runmap = pd.Series(np.cumsum(d['camera_frames'].flag_gap_before).to_numpy(), index=d['camera_frames'].frame_id)
    datasets = [ ('before',b,coordinates(b),np.stack([b[n+'_vis'].to_numpy() for n in LM],1),
                  np.stack([b[n+'_vis'].to_numpy()>=.5 for n in LM],1)&(b.detected.to_numpy()[:,None]==1),d['b'],'before'),
                 ('after_full',a,coordinates(a),np.stack([a[n+'_vis'].to_numpy() for n in LM],1),
                  np.stack([a[n+'_reliable'].to_numpy()==1 for n in LM],1),d['a'],'after'),
                 ('after_matched',m,coordinates(m,'x_after','y_after'),np.stack([m[n+'_vis_after'].to_numpy() for n in LM],1),
                  np.stack([m[n+'_reliable_after'].to_numpy()==1 for n in LM],1),d['b'],'after')]
    rows=[]
    for name,table,xy,score,good,diag,side in datasets:
        if name.startswith('after'):
            xy[~good]=np.nan
        else:
            xy[b.detected.to_numpy()!=1]=np.nan
        fid = table.after_frame_id if name=='after_matched' else (d['pose'].nearest_recorded_frame_id if name=='before' else table.frame_id)
        runs = fid.map(runmap).to_numpy()
        assert np.isfinite(runs).all()
        for k,lm in enumerate(LM):
            counts = diag[lm+'_'+side+'_detectors'].str.split('+',regex=False).map(lambda v:'swap' in v).sum()
            rows.append({'dataset':name,'kind':'landmark','name':lm,**motion(xy[:,k],table.t_run_s.to_numpy(),runs),
                         'swap_count':int(counts),'visibility_median':quantile(score[:,k],.5),'usable_ratio':float(good[:,k].mean())})
        for left,right in BONES:
            if 'ear' in left or 'ear' in right or left=='nose':
                continue
            values=np.linalg.norm(xy[:,LM.index(left)]-xy[:,LM.index(right)],axis=1)
            rows.append({'dataset':name,'kind':'bone','name':left+'--'+right,**bone_stats(values)})
    result=pd.DataFrame(rows)
    write_csv(QC/'proxy_metrics.csv',result)
    return result

def savefig(name):
    plt.savefig(QC/name,dpi=120,metadata={'Software':None},bbox_inches='tight')
    plt.close()

def plots(d):
    b,a=d['pose_landmarks'],d['pose_landmarks_corrected']
    for group in ['shoulders','elbows','wrists']:
        fig,axs=plt.subplots(2,1,figsize=(16,7),sharex=True)
        for axis,field,size in zip(axs,['x','y'],[1280,800]):
            for lm in GROUPS[group]:
                c='tab:blue' if lm.startswith('left') else 'tab:orange'
                axis.scatter(b.t_run_s,b[lm+'_'+field]*size,s=1,c=c,alpha=.35,label=lm+' BEFORE')
                axis.plot(a.t_run_s,a[lm+'_'+field]*size,c=c,lw=.6,label=lm+' AFTER')
            axis.set_ylabel(field+' (px)'); axis.legend(fontsize=7,ncol=2); axis.grid(alpha=.2)
        axs[-1].set_xlabel('t_run_s'); savefig('ts_'+group+'_full.png')
    obd=d['obd']; centers=[]
    for i in np.argsort(-np.abs(obd.steering_angle_deg.fillna(0).to_numpy()),kind='stable'):
        t=float(obd.t_run_s.iloc[i])
        if all(abs(t-c)>=60 for c in centers): centers.append(t)
        if len(centers)==3: break
    for num,t in enumerate(centers,1):
        fig,axs=plt.subplots(3,1,figsize=(15,9),sharex=True)
        for axis,field,size in zip(axs[:2],['x','y'],[1280,800]):
            for k,lm in enumerate(sum([GROUPS[g] for g in ['shoulders','elbows','wrists']],[])):
                bb=b[b.t_run_s.between(t-10,t+10)]; aa=a[a.t_run_s.between(t-10,t+10)]
                axis.scatter(bb.t_run_s,bb[lm+'_'+field]*size,s=5,color=plt.cm.tab10(k),alpha=.5)
                axis.plot(aa.t_run_s,aa[lm+'_'+field]*size,lw=.8,color=plt.cm.tab10(k),label=lm)
            axis.set_ylabel(field+' (px)'); axis.legend(ncol=3,fontsize=7)
        oo=obd[obd.t_run_s.between(t-10,t+10)]
        axs[2].plot(oo.t_run_s,oo.steering_angle_deg,color='black'); axs[2].set_ylabel('steering (deg)'); axs[2].set_xlabel('t_run_s')
        savefig(f'ts_arms_zoom_{num}.png')

def timeline(d):
    rows=[]
    lo=int(np.floor(d['a'].t_run_s.min()/60)); hi=int(np.floor(d['a'].t_run_s.max()/60))
    for minute in range(lo,hi+1):
        b=d['b'][np.floor(d['b'].t_run_s/60)==minute]; a=d['a'][np.floor(d['a'].t_run_s/60)==minute]
        for group,joints in GROUPS.items():
            bs=np.concatenate([b[n+'_before_status'].to_numpy(str) for n in joints]); ast=np.concatenate([a[n+'_after_status'].to_numpy(str) for n in joints])
            final=np.concatenate([a[n+'_final'].to_numpy(str) for n in joints]); action=np.concatenate([a[n+'_action'].to_numpy(str) for n in joints])
            rate=lambda mask:float(np.mean(mask)) if len(mask) else np.nan
            rows.append({'minute':minute,'start_t_run_s':minute*60,'group':group,'n_before_joint_frames':len(bs),'n_after_joint_frames':len(ast),
                         'before_misplaced_rate':rate(np.char.startswith(bs,'misplaced_')),'before_unusable_rate':rate(np.isin(bs,['not_detected','low_visibility','occluded'])),
                         'after_unreliable_rate':rate(final=='unreliable'),'after_raw_misplaced_rate':rate(np.char.startswith(ast,'misplaced_')),
                         'after_corrected_rate':rate(np.isin(action,['corrected_by_constraint','interpolated']))})
    table=pd.DataFrame(rows); write_csv(QC/'timeline_per_minute.csv',table)
    fig,axs=plt.subplots(5,1,figsize=(14,13),sharex=True)
    for ax,(g,_) in zip(axs,GROUPS.items()):
        sub=table[table.group==g]
        for field in [c for c in table if c.endswith('_rate')]: ax.plot(sub.start_t_run_s,sub[field],label=field)
        ax.set_ylabel(g); ax.set_ylim(0,1); ax.grid(alpha=.2)
    axs[0].legend(fontsize=7,ncol=3); axs[-1].set_xlabel('t_run_s'); savefig('timeline_per_minute.png')
    return table

def distribution(values):
    counts=pd.Series(values).value_counts().sort_index()
    return ';'.join(f'{key}:{int(value)}' for key,value in counts.items())

def episodes(d):
    rows=[]
    runmap=pd.Series(np.cumsum(d['camera_frames'].flag_gap_before).to_numpy(),index=d['camera_frames'].frame_id)
    for kind,table in [('before',d['b']),('after',d['a'])]:
        for lm in LM:
            status=table[lm+'_'+('before_status' if kind=='before' else 'after_status')].to_numpy(str)
            mask=np.char.startswith(status,'misplaced_') if kind=='before' else table[lm+'_action'].to_numpy()!='smoothed_only'
            t=table.t_run_s.to_numpy(); runs=table.after_frame_id.map(runmap).to_numpy()
            assert np.isfinite(runs).all()
            adjacent=(np.diff(t)<=.5)&(np.diff(runs)==0) if kind=='before' else np.diff(runs)==0
            starts=np.flatnonzero(mask & ~np.r_[False,mask[:-1]&adjacent])
            ends=np.flatnonzero(mask & ~np.r_[mask[1:]&adjacent,False])
            for start,end in zip(starts,ends):
                sub=table.iloc[start:end+1]; freq=pd.Series(status[start:end+1]).value_counts()
                typ=sorted(freq.index,key=lambda s:(-int(freq[s]),s))[0]
                rows.append({'kind':kind,'landmark':lm,'start_t_run_s':t[start],'end_t_run_s':t[end],'duration_s':t[end]-t[start],
                             'n_frames':end-start+1,'start_frame_id':int(sub.frame_id.iloc[0]),'end_frame_id':int(sub.frame_id.iloc[-1]),
                             'type':typ,'types':distribution(status[start:end+1]),'resolution':distribution(sub[lm+('_before_outcome' if kind=='before' else '_action')]),
                             'max_disp_px':pd.to_numeric(sub[lm+'_disp_px'],errors='coerce').max() if kind=='before' else np.nan})
    result=pd.DataFrame(rows); write_csv(QC/'episodes.csv',result)
    return result

def confusion(d):
    rows=[]
    for lm in LM:
        b=d['b']; final=np.where(b[lm+'_final']=='reliable','reliable','unreliable('+b[lm+'_reason']+')')
        counts=pd.crosstab(b[lm+'_before_status'],final)
        assert int(counts.to_numpy().sum())==len(b)
        for status,row in counts.iterrows():
            for outcome,count in row.items(): rows.append({'landmark':lm,'before_status':status,'after_final':outcome,'count':int(count)})
    table=pd.DataFrame(rows); write_csv(QC/'confusion_before_after.csv',table)
    return table

def model_compare(d):
    table=d['pose_landmarks_reinferred']; times=table.t_run_s.to_numpy(); runs=np.cumsum(d['camera_frames'].flag_gap_before).to_numpy()
    main=coordinates(table,prefix='rtm_x_'); matched=d['pose_before_after']; lookup=table.set_index('frame_id')
    results={}
    for variant in VARIANTS:
        xy=coordinates(table,prefix=variant+'_'); score=np.stack([table[f'{variant}_{n}_score'].to_numpy() for n in LM],1)
        detected=table[variant+'_detected'].to_numpy()==1; xy[~detected]=np.nan
        mm=lookup.loc[matched.after_frame_id]; mx=coordinates(mm,prefix=variant+'_'); mx[mm[variant+'_detected'].to_numpy()!=1]=np.nan
        bx=coordinates(matched,'x_before','y_before'); bx[matched.before_detected.to_numpy()!=1]=np.nan
        bone={}
        for a,b in BONES:
            if 'ear' not in a+b and a!='nose': bone[a+'--'+b]=bone_stats(np.linalg.norm(xy[:,LM.index(a)]-xy[:,LM.index(b)],axis=1))
        groups={}
        for group,joints in GROUPS.items():
            ids=[LM.index(n) for n in joints]; dist=np.linalg.norm(xy[:,ids]-main[:,ids],axis=2); finite=dist[np.isfinite(dist)]
            groups[group]={'distance_to_rtm_x_median_px':quantile(dist,.5),'over_40px_ratio':float(np.mean(finite>40)) if len(finite) else None,
                           'distance_to_before_median_px':quantile(np.linalg.norm(mx[:,ids]-bx[:,ids],axis=2),.5)}
        results[variant]={'detection_rate':float(detected.mean()),'landmarks':{lm:motion(xy[:,k],times,runs) for k,lm in enumerate(LM)},'bones':bone,'groups':groups}
    write_json(QC/'model_compare.json',results)
    return results

def overlays(d):
    m=d['pose_before_after']; rng=np.random.default_rng(2026); times=m.t_run_s.to_numpy()
    disp=m[[n+'_disp_px' for n in LM]].max(axis=1).fillna(-1).to_numpy()
    low=(m[[n+'_vis_before' for n in LM]].to_numpy()<.5).any(axis=1)|(m.before_detected.to_numpy()==0)
    unrel=(m[[n+'_reliable_after' for n in LM]].to_numpy()==0).any(axis=1)
    strata=[('random',rng.permutation(len(m)),20),('steering',np.argsort(-np.abs(d['steering']),kind='stable'),10),
            ('displacement',np.argsort(-disp,kind='stable'),10),('before_low',rng.permutation(np.flatnonzero(low)),10),('after_unreliable',rng.permutation(np.flatnonzero(unrel)),10)]
    chosen=sample_strata(times,strata,2,60,rng); folder=QC/'overlays'; folder.mkdir(exist_ok=True)
    for num,(i,kind) in enumerate(chosen,1):
        row=m.iloc[i]; title=f'f{int(row.frame_id)} t={row.t_run_s:.3f}s steering={d["steering"][i]:.1f}deg'
        im,_=pair(d,int(row.frame_id),int(row.after_frame_id),960,[title+' BEFORE',title+' AFTER'],raw=True,long_labels=True)
        if not cv2.imwrite(str(folder/f'{num:03d}_{kind}_f{int(row.frame_id)}.jpg'),im,[cv2.IMWRITE_JPEG_QUALITY,92]): raise OSError('JPEG write failed')
    write_json(QC/'overlay_selection.json',[{'item':num,'stratum':kind,'frame_id':int(m.frame_id.iloc[i]),'t_run_s':float(times[i])} for num,(i,kind) in enumerate(chosen,1)])
    log(f'overlays: {len(chosen)}')

def clip(d):
    camera=d['camera_frames']; frames=camera[(camera.segment_id==4)&camera.t_run_s.between(135,185,inclusive='left')]
    if frames.empty: raise ValueError('No segment 4 frames at 135..185s')
    target=135+np.arange(1250)/25
    ft=frames.t_run_s.to_numpy(); upper=np.clip(np.searchsorted(ft,target),0,len(ft)-1); lower=np.maximum(0,upper-1)
    nearest=np.where(np.abs(ft[lower]-target)<=np.abs(ft[upper]-target),lower,upper)
    assert np.max(np.abs(ft[nearest]-target)) < .05
    frames=frames.iloc[nearest]
    path=QC/'before_after_clip.mp4'; error=WORK/'clip_ffmpeg.log'
    cmd=['ffmpeg','-y','-hide_banner','-loglevel','error','-f','rawvideo','-pixel_format','bgr24','-video_size','1284x400','-framerate','25','-i','pipe:0','-an','-c:v','libx264','-threads','1','-bitexact','-map_metadata','-1','-fflags','+bitexact','-pix_fmt','yuv420p','-crf','23',str(path)]
    b=d['pose']; bt=b.t_run_s.to_numpy()
    with error.open('wb') as err:
        process=subprocess.Popen(cmd,stdin=subprocess.PIPE,stderr=err)
        try:
            for num,frame in enumerate(frames.itertuples(index=False)):
                idx=int(np.searchsorted(bt,frame.t_run_s,side='right')-1)
                bid=int(b.frame_id.iloc[idx]) if idx>=0 and frame.t_run_s-bt[idx]<=.15 else None
                image,_=pair(d,bid,int(frame.frame_id),640,['BEFORE' if bid is not None else 'BEFORE no pose','AFTER'],raw=True)
                process.stdin.write(image.tobytes())
                if num%200==0: log(f'clip {num}/{len(frames)}')
        finally:
            process.stdin.close()
            code=process.wait()
        if code: raise RuntimeError(error.read_text())
    return {'frames':len(frames),'duration_s':len(frames)/25,'first_t_run_s':float(frames.t_run_s.iloc[0]),'last_t_run_s':float(frames.t_run_s.iloc[-1])}

def markdown(table):
    table=table.copy().fillna('')
    def fmt(v):
        return f'{v:.6f}'.rstrip('0').rstrip('.') if isinstance(v,(float,np.floating)) else str(v).replace('|','/').replace('\n',' ')
    return '| '+' | '.join(map(str,table.columns))+' |\n| '+' | '.join(['---']*len(table.columns))+' |\n'+'\n'.join('| '+' | '.join(fmt(v) for v in row)+' |' for row in table.itertuples(index=False,name=None))+'\n'

def dict_table(value):
    if isinstance(value,dict):
        return markdown(pd.DataFrame([{'항목':k,'값':json.dumps(v,ensure_ascii=False,sort_keys=True)} for k,v in value.items()]))
    return '```json\n'+json.dumps(value,ensure_ascii=False,indent=2)+'\n```\n'

def limits_table(limits):
    text='기준은 finite P이고 low_conf/tta/roi/model이 모두 False인 관절-프레임이다.\n'
    for field in ['bones','angles','regions']:
        table=pd.DataFrame(limits[field])
        for col in table:
            table[col]=table[col].map(lambda v:json.dumps(v,ensure_ascii=False) if isinstance(v,(list,dict)) else v)
        text+='\n'+field+'\n'+markdown(table)
    return text

def diagnostic_tables(stats,diag,side):
    text=''
    for field in ['detectors','before_outcome','after_status','action','reason','reliable_ratio_by_segment','displacement_px']:
        if field in stats:
            text+='\n'+field+'\n'+markdown(pd.DataFrame.from_dict(stats[field],orient='index').rename_axis('landmark / segment').reset_index().fillna(0))
    order=sorted({v for lm in LM for s in diag[lm+'_'+side+'_detectors'] for v in s.split('+') if v})
    matrix=np.array([[name in s.split('+') for name in order] for lm in LM for s in diag[lm+'_'+side+'_detectors']],dtype=np.int64)
    text+='\n검출기 겹침(관절-프레임 동시 발동 수)\n'+markdown(pd.DataFrame(matrix.T@matrix,index=order,columns=order).rename_axis('detector').reset_index())
    return text

def report(d,p,t,e,c,models,video):
    stats=json.loads((QC/'diag_stats.json').read_text(encoding='utf-8-sig'))
    params=json.loads((ROOT/'pipeline/5_pose_correction/params.json').read_text(encoding='utf-8-sig'))
    limits=json.loads((QC/'kinematic_limits.json').read_text(encoding='utf-8-sig'))
    before=d['b']; after=d['a']; reliable=np.array([(after[n+'_final']=='reliable').mean() for n in LM])
    misplaced=np.concatenate([before[n+'_before_status'].str.startswith('misplaced_').to_numpy() for n in LM]).mean()
    sections=['# 운전자 pose 보정 QC 보고서\n',f'## 0. 요약\nBEFORE {len(before):,}프레임, AFTER {len(after):,} 저장 프레임을 분석했다. BEFORE 오배치 진단율은 관절-프레임 기준 {misplaced:.6%}, AFTER reliable 비율은 {reliable.mean():.6%}다. 재추론·제약 후보 선택·짧은 결측 보간·시간 평활을 적용했다. 오배치 진단은 모델과 기하 프록시이며 정답 판정이 아니다.\n',
              '## 1. 입력/산출물\n'+markdown(pd.DataFrame([{'파일':name+'.csv','행수':len(table),'열수':len(set(table.columns)-{'t_run_s','in_obd_window'})} for name,table in d.items() if name in ['camera_frames','pose','pose_landmarks','pose_landmarks_corrected','pose_landmarks_reinferred','pose_corrected','pose_before_after','pose_frame_diagnosis','obd','imu']]))]
    sections.append(markdown(pd.DataFrame([{'파일':n,'행수':len(v),'열수':len(v.columns)} for n,v in [('proxy_metrics.csv',p),('timeline_per_minute.csv',t),('episodes.csv',e),('confusion_before_after.csv',c)]])))
    sections.append('## 2. BEFORE 진단\n'+markdown(pd.DataFrame({n:before[n+'_before_status'].value_counts() for n in LM}).fillna(0).astype(int).rename_axis('status').reset_index())+diagnostic_tables(stats['before'],before,'before'))
    rows=[]
    for variant,value in models.items():
        for group,g in value['groups'].items():
            rows.append({'variant':variant,'group':group,'검출률':value['detection_rate'],**g,
                         '속도_p50_중앙값':quantile([v['speed_p50_px_s'] for v in value['landmarks'].values() if v['speed_p50_px_s'] is not None],.5),
                         '속도_p99_중앙값':quantile([v['speed_p99_px_s'] for v in value['landmarks'].values() if v['speed_p99_px_s'] is not None],.5),
                         '지터_중앙값':quantile([v['jitter_median_px'] for v in value['landmarks'].values() if v['jitter_median_px'] is not None],.5),
                         '뼈_MAD비_중앙값':quantile([v['bone_mad_over_median'] for v in value['bones'].values() if v['bone_mad_over_median'] is not None],.5)})
    sections.append('## 3. 방법 비교\n'+markdown(pd.DataFrame(rows))+'관절별·뼈별 전체 수치는 model_compare.json에 있다. score 단위는 모델마다 달라 직접 확률 비교가 불가능하다.\n')
    definitions=[('low_conf','P.score < t_conf'),('tta','P와 반전 복원 F의 거리 > t_tta'),('roi','P와 bbox2의 거리 > t_roi'),('model','P와 RTMW의 거리 > t_model'),
                 ('mp','MediaPipe 3종의 상호 거리 ≤ t_mp_self, 최소 visibility ≥ t_mp_vis인데 P와 중앙값 거리 > t_mp; hard 제외'),
                 ('jump','같은 run의 jump_win_s 이내 이웃(최소 jump_min_neighbors개) 성분별 중앙값과 거리 > t_jump'),
                 ('bone','붙은 뼈가 기준 분위수 × bone_margin_lo/hi 범위 밖; 양 끝 관절 표시'),('angle','팔꿈치 내각이 기준 분위수 ± angle_margin_deg 밖; elbow/wrist 표시'),
                 ('swap','TTA의 교차 좌우 거리 합 + t_swap_margin < 동일 좌우 거리 합'),('region','기준 2D 히스토그램에서 region_min_count 이상인 칸을 region_dilate 팽창한 마스크 밖'),('overlap','먼 쪽 팔 관절(left elbow/wrist)이 가까운 쪽 팔 관절과 t_overlap 이내로 겹침(뒤에 가려짐) 또는 팔꿈치가 같은 팔 손목과 겹침(팔 붕괴) → 가려진/붕괴된 관절 표시'),('person','YOLOX 사람 bbox 중 운전자가 아닌 사람(높이 ≥ person_min_height_px) 상자 안의 관절 → occluded_by_other_person, 0.25 s 이하 틈 메움')]
    sections.append('## 4. 보정 방법\n'+markdown(pd.DataFrame(definitions,columns=['AFTER 검출기','정의']))+'\nBEFORE는 not_detected, low_vis(vis < 0.5), 자체 이웃 jump(b_jump_win_s/t_b_jump), 동일한 bone/angle/region, reliable AFTER 참조와의 swap/ref(t_mis), 고신뢰 mp_heavy_raw와의 차이를 검사한다. before_status는 not_detected → swap → off_body → bone → angle → jump → ref → AFTER unreliable(occluded) → low_visibility → ok 순으로 판정한다.\n\n임계값과 시간창(params.json):\n'+dict_table(params)+'\n후보 순서는 주 모델 → median4 → RTMW → flip → bbox2이며 합의 수·점수·지지영역·뼈·각도·시간 조건을 모두 만족해야 한다. 같은 run의 짧은 결측만 실제 시각으로 보간하고 reliable 연속 구간 안에서 2차 시간 다항식으로 평활한다. run 경계를 넘지 않는다.\n학습 한계(kinematic_limits.json):\n'+limits_table(limits))
    sections.append('## 5. AFTER 결과\n'+markdown(pd.DataFrame({'landmark':LM,'reliable_ratio':reliable}))+diagnostic_tables(stats['after'],after,'after')+'\n프레임 단위 검출기 발동 범위:\n'+dict_table(stats['after']['frames_any_detector']))
    sections.append('## 6. 프록시 지표 BEFORE vs AFTER\nAFTER full은 25 fps 전체, matched는 BEFORE 시각의 대응값이며 reliable 좌표만 쓴다. BEFORE 좌표는 검출된 원값을 쓰고 사용 가능율은 vis ≥ 0.5다. 속도는 실제 시간차 ≤ 0.5 s이고 run 안인 이웃 쌍, 지터는 연속 세 점의 2차 차분 벡터 크기다. 지터는 시간 간격을 정규화하지 않으므로 서로 다른 fps 간 직접 우열 판단에 쓰지 않는다. 뼈 통계는 유한한 양 끝점만 사용한다.\n'+markdown(p)+'\n'+''.join(f'![{g}](ts_{g}_full.png)\n' for g in ['shoulders','elbows','wrists'])+''.join(f'![zoom {i}](ts_arms_zoom_{i}.png)\n' for i in range(1,4)))
    summary=e.groupby(['kind','landmark'],sort=True).duration_s.agg(['count','sum','median',lambda x:x.quantile(.9),lambda x:x.quantile(.99)]).reset_index()
    summary.columns=['kind','landmark','episode_count','total_duration_s','duration_p50_s','duration_p90_s','duration_p99_s']
    sections.append('## 7. 에피소드\n지속시간은 마지막 시각−시작 시각이며 단일 프레임은 0초다. 전체 목록은 episodes.csv.\n'+markdown(summary)+'\n최장 30개:\n'+markdown(e.sort_values(['duration_s','kind','landmark','start_frame_id'],ascending=[False,True,True,True],kind='stable').head(30)))
    sections.append('## 8. 분당 타임라인\n![timeline](timeline_per_minute.png)\n'+markdown(t))
    sections.append('## 9. before×after 교차표\n관절별 전체 합은 BEFORE 프레임 수와 같다. 각 before_status 행합은 해당 상태의 BEFORE 수다.\n'+markdown(c))
    blind=QC/'blind_audit.json'
    sections.append('## 10. 블라인드 감사 결과\n'+(dict_table(json.loads(blind.read_text(encoding='utf-8'))) if blind.exists() else '미실시\n'))
    qa=[]; reviewed=sorted((QC/'qa/findings').glob('*.REVIEWED.json'))
    for sid in sorted(d['camera_frames'].segment_id.unique()):
        done=WORK/f'handoff/seg_{sid:03d}/DONE.json'; info=json.loads(done.read_text(encoding='utf-8-sig')) if done.exists() else {}
        matches=[]
        for path in reviewed:
            item=json.loads(path.read_text(encoding='utf-8-sig'))
            if int(item.get('segment',item.get('segment_id',-1)))==sid:
                finding=path.with_name(path.name.replace('.REVIEWED.json','.json'))
                details=json.loads(finding.read_text(encoding='utf-8-sig')) if finding.exists() else item
                matches.append((item,details))
        current=[x for x in matches if x[0].get('revision')==info.get('revision') and 'SUPERSEDED' not in str(x[0].get('status','')).upper()]
        unresolved=sum(int(x.get('unresolved_count',len([f for f in detail.get('findings',[]) if f.get('status','open') not in ['resolved','closed']]))) for x,detail in current)
        latest=max([x[0].get('revision',0) for x in matches],default=0)
        qa.append({'segment':int(sid),'revision':info.get('revision',''),'검수_revision':latest or '',
                   '검수':('완료' if all(x.get('frames_reviewed',0)>=x.get('frames_rendered',float('inf')) for x,_ in current) else '부분') if current else '대기',
                   '미해결_finding':unresolved if current else ''})
    sections.append('## 11. QA 팀 검수 현황\n'+markdown(pd.DataFrame(qa)))
    detectors=sorted({name for lm in LM for value in after[lm+'_after_detectors'] for name in value.split('+') if name})
    sections.append(f'## 12. 모든 프레임 분석의 의미\n자동 분석은 저장 프레임 {len(after):,}개(100%), 관절 {len(LM)}개, 검출기 {len(detectors)}종이다. BEFORE {len(before):,}개도 전부 진단했다. 자동 분석은 모든 프레임의 시각 검수 완료를 뜻하지 않는다. QC 오버레이 60장과 블라인드 표본은 별도이며 실제 채점 완료 범위는 10절, QA 완료 범위는 11절에 기록한다. 잔여 오류율은 clean AFTER wrong 율과 Wilson 95% CI로 추정하며 감사 미실시 시 추정 불가다. 영상: {json.dumps(video,ensure_ascii=False)}.\n')
    sections.append('## 13. 남은 한계\n카메라는 운전자 우측면이며 right는 가까운 쪽, left는 먼 쪽이다. 특히 left_ear는 가림이 심하다. 모델 간 합의와 학습 기하 한계는 실제 해부학 정답을 보장하지 않는다. 가려진 관절의 위치는 확인 불가하며 unreliable은 자동 누락 표시다. 표본 간 최소 시간 간격 때문에 시각 감사는 독립 단순 무작위 표본이 아니고 Wilson 구간은 표본 내 이항 근사다. 영상 BEFORE는 직전 0.15초 이내 결과를 유지 표시한다.\n')
    sections.append('## 14. 재현 방법\n```powershell\npython pipeline\\5_pose_correction\\qc.py\npython pipeline\\5_pose_correction\\audit.py render --set hard\npython pipeline\\5_pose_correction\\audit.py render --set clean\npython pipeline\\5_pose_correction\\audit.py render-unreliable\npython pipeline\\5_pose_correction\\audit.py unblind\n```\n채점 후 qc.py를 다시 실행하면 감사 결과가 보고서에 반영된다. 입력은 읽기 전용이며 난수 seed=2026, Agg/dpi=120, JPEG 품질=92, 단일 스레드 libx264로 출력한다.\n')
    note=WORK/'notes_lead.md'
    if note.exists(): sections.append('## 팀장 메모\n'+note.read_text(encoding='utf-8-sig'))
    (QC/'report.md').write_text('\n'.join(sections),encoding='utf-8',newline='\n')

def selftest():
    xy=np.array([[0.,0],[3.,4],[6.,8]])
    m=motion(xy,np.array([0.,.1,.2]),np.zeros(3))
    assert abs(m['speed_p50_px_s']-50)<1e-9 and m['jitter_median_px']==0
    assert motion(xy,np.array([0.,1.,2.]),np.zeros(3))['n_speed_pairs']==0
    assert motion(xy,np.array([0.,.1,.2]),np.arange(3))['n_speed_pairs']==0
    assert bone_stats(np.array([10.,10.,10.]))['bone_mad_over_median']==0

def main():
    selftest(); QC.mkdir(parents=True,exist_ok=True)
    log('QC load inputs'); d=load()
    log('QC metrics'); p=proxy(d); t=timeline(d); e=episodes(d); c=confusion(d); models=model_compare(d)
    log('QC plots'); plots(d)
    log('QC overlays'); overlays(d)
    log('QC clip'); video=clip(d)
    report(d,p,t,e,c,models,video); log('QC COMPLETE')

if __name__=='__main__': main()
