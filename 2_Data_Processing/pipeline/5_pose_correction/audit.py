"""Seeded blind A/B rendering and validated unblinding of human scores."""
import argparse
import json
import math
import cv2
import numpy as np
import pandas as pd
from qc_helpers import *

AUDIT=WORK/'audit'
GRADES=['correct','slightly_off','wrong','not_visible','absent']
VERDICTS=['visible_and_correct','visible_but_wrong','occluded_or_ambiguous']
AB_INSTRUCTIONS='''이미지는 운전자를 우측에서 본 차량 실내 모노 영상입니다. 두 패널은 같은 프레임에 서로 다른 방법의 스켈레톤을 표시합니다. 어느 패널이 어떤 방법인지 모릅니다.
R = 운전자의 오른쪽 = 카메라에 가까운 쪽, L = 먼 쪽입니다. Rsh/Lsh 어깨, Rel/Lel 팔꿈치, Rwr/Lwr 손목, Rhi/Lhi 엉덩이, Rea 귀(가까운 쪽 귀만 그림), nose 코입니다.
패널별·관절군별(shoulders, elbows, wrists, head, hips)로 그려진 관절을 평가하세요.
correct: 관절 중심이 실제 해부학적 위치에서 대략 손바닥 반 폭 이내
slightly_off: 같은 신체 부위 위지만 눈에 띄게 어긋남
wrong: 다른 신체 부위/물체/배경 위, 또는 좌우가 바뀜
not_visible: 해당 관절이 가려지거나 화면 밖이라 판단 불가(점이 그려져 있어도)
absent: 해당 관절군의 점이 하나도 그려지지 않음
관절군에 점이 여러 개면 가장 나쁜 것을 기준으로 평가하고 worst_joint 라벨을 worst의 해당 관절군에 적으세요. not_visible은 정확도 분모에서 제외됩니다.
출력: scores/batch_NN.json
[{"item":1,"A":{"shoulders":"correct","elbows":"wrong","wrists":"correct","head":"not_visible","hips":"absent"},"B":{"shoulders":"correct","elbows":"correct","wrists":"correct","head":"not_visible","hips":"correct"},"worst":{"A":{"elbows":"Lel"},"B":{}},"note":""}]
'''
UNREL_INSTRUCTIONS='''운전자를 우측에서 본 차량 실내 영상입니다. 노란 원으로 표시한 관절 하나의 원시 추정 위치를 평가하세요. R은 운전자의 오른쪽(카메라에 가까운 쪽), L은 먼 쪽입니다.
visible_and_correct: 관절이 뚜렷이 보이고 노란 점이 맞는 위치
visible_but_wrong: 관절은 보이는데 점이 틀림
occluded_or_ambiguous: 관절이 가려짐·화면 밖·모호
출력: scores/batch_NN.json = [{"item":1,"verdict":"visible_and_correct","note":""}]
'''

def write_batches(setname, keys, instructions):
    folder=AUDIT/setname
    (folder/'scores').mkdir(exist_ok=True)
    (folder/'INSTRUCTIONS.txt').write_text(instructions,encoding='utf-8',newline='\n')
    ids=sorted(int(k) for k in keys)
    write_json(folder/'batches.json',[{'batch':num+1,'items':ids[i:i+30],'score_file':f'scores/batch_{num+1:02d}.json'} for num,i in enumerate(range(0,len(ids),30))])
    write_json(AUDIT/'key'/f'{setname}_key.json',keys)

def render(setname,n,seed):
    d=load(); m=d['pose_before_after']; rng=np.random.default_rng(seed)
    m=m[m.before_detected==1].reset_index(drop=True)
    times=m.t_run_s.to_numpy(); steer=np.interp(times,d['obd'].t_run_s,d['obd'].steering_angle_deg)
    yaw=np.abs(np.interp(times,d['imu'].t_run_s,d['imu'].veh_gyro_z_radps))
    arms=sum([GROUPS[g] for g in ['shoulders','elbows','wrists']],[])
    important=arms+GROUPS['hips']
    if setname=='clean':
        diag=d['a'].set_index('frame_id').loc[m.after_frame_id]
        mask=(diag[[lm+'_after_detectors' for lm in LM if lm!='left_ear']].to_numpy()=='').all(axis=1)
        eligible=np.flatnonzero(mask)
        # The fill pool is restricted to detector-clean frames too.
        pool=m.iloc[eligible].reset_index(drop=True)
        chosen=sample_strata(pool.t_run_s.to_numpy(),[('clean',rng.permutation(len(pool)),n)],.5,n,rng)
        m=pool
    else:
        low=(m[[lm+'_vis_before' for lm in LM]].to_numpy()<.5).sum(axis=1)>=3
        disp=m[[lm+'_disp_px' for lm in arms]].max(axis=1).fillna(-1).to_numpy()
        unrel=(m[[lm+'_reliable_after' for lm in important]].to_numpy()==0).any(axis=1)
        turning=(np.abs(steer)>=90)|(yaw>=np.quantile(yaw,.9))
        quotas=np.array([80,60,40,70,70])*n/320
        quotas=np.floor(quotas).astype(int); quotas[0]+=n-int(quotas.sum())
        strata=[('random',rng.permutation(len(m)),quotas[0]),('turning',rng.permutation(np.flatnonzero(turning)),quotas[1]),
                ('before_low_visibility',rng.permutation(np.flatnonzero(low)),quotas[2]),('arm_displacement',np.argsort(-disp,kind='stable'),quotas[3]),
                ('after_unreliable',rng.permutation(np.flatnonzero(unrel)),quotas[4])]
        chosen=sample_strata(times,[s for s in strata if s[2]>0],1,n,rng)
    folder=AUDIT/setname/'img'; folder.mkdir(parents=True,exist_ok=True)
    if list((AUDIT/setname/'scores').glob('*.json')): raise ValueError('Existing scores: render into a reviewed set would invalidate its keys')
    for stale in folder.glob(f'{setname}_*.jpg'): stale.unlink()
    keys={}
    for num,(idx,stratum) in enumerate(chosen,1):
        row=m.iloc[idx]; flip=bool(rng.integers(2))
        image,panels=pair(d,int(row.frame_id),int(row.after_frame_id),784,[f'A   {num:04d}',f'B   {num:04d}'],flip=flip,skip=('left_ear',))  # far ear is almost never usable AFTER: drawing it would unblind the panels
        path=folder/f'{setname}_{num:04d}.jpg'
        if not cv2.imwrite(str(path),image,[cv2.IMWRITE_JPEG_QUALITY,92]): raise OSError(path)
        keys[str(num)]={'frame_id':int(row.frame_id),'after_frame_id':int(row.after_frame_id),'before_panel':'B' if flip else 'A','stratum':stratum,
                        'drawn_joints':{panel:[lm for lm in LM if lm in pts] for panel,pts in zip(['A','B'],panels)}}
        if num%30==0: log(f'{setname} render {num}/{len(chosen)}')
    write_batches(setname,keys,AB_INSTRUCTIONS)
    write_json(AUDIT/'key'/f'{setname}_selection.json',{'seed':seed,'requested':n,'rendered':len(chosen),'shortfall':n-len(chosen),'strata':{s:sum(k['stratum']==s for k in keys.values()) for s in sorted({k['stratum'] for k in keys.values()})}})
    log(f'{setname} COMPLETE {len(chosen)}/{n}; keys separated in audit/key')

def render_unreliable(n,seed):
    d=load(); rng=np.random.default_rng(seed); a=d['pose_landmarks_corrected']; diag=d['a'].set_index('frame_id')
    candidates={}; ear=[]
    for lm in LM:
        for idx in np.flatnonzero(a[lm+'_reliable'].to_numpy()==0):
            row=a.iloc[idx]
            if not np.isfinite([row[lm+'_x_reinfer'],row[lm+'_y_reinfer']]).all(): continue
            candidate=(int(idx),lm,str(row[lm+'_reason']))
            if lm=='left_ear': ear.append(candidate)
            else: candidates.setdefault((lm,candidate[2]),[]).append(candidate)
    queues={key:[values[i] for i in rng.permutation(len(values))] for key,values in sorted(candidates.items())}
    selected=[]; ear_quota=min(30,n//5); target=n-ear_quota
    while len(selected)<target and any(queues.values()):
        for key in sorted(queues):
            if queues[key] and len(selected)<target: selected.append(queues[key].pop())
    selected.extend([ear[i] for i in rng.permutation(len(ear))[:ear_quota]])
    folder=AUDIT/'unrel/img'; folder.mkdir(parents=True,exist_ok=True)
    if list((AUDIT/'unrel/scores').glob('*.json')): raise ValueError('Existing unrel scores; rendering would invalidate keys')
    for stale in folder.glob('unrel_*.jpg'): stale.unlink()
    keys={}
    for num,(idx,lm,reason) in enumerate(selected,1):
        row=a.iloc[idx]; fid=int(row.frame_id); frame=d['camera_index'].loc[fid]
        image=read_frame(frame.segment_file,frame.frame_in_segment)
        out=skeleton(image,points(row,True),1100,f'{num:04d}  {LABELS[lm]}',thin=True)
        q=tuple(np.rint(np.array([row[lm+'_x_reinfer']*1280,row[lm+'_y_reinfer']*800])*1100/1280).astype(int))
        cv2.circle(out,q,12,(0,255,255),3,cv2.LINE_AA)
        cv2.putText(out,LABELS[lm],(q[0]+15,q[1]),cv2.FONT_HERSHEY_SIMPLEX,.7,(0,255,255),2,cv2.LINE_AA)
        path=folder/f'unrel_{num:04d}.jpg'
        if not cv2.imwrite(str(path),out,[cv2.IMWRITE_JPEG_QUALITY,92]): raise OSError(path)
        keys[str(num)]={'frame_id':fid,'landmark':lm,'reason':reason,'detectors':diag.loc[fid,lm+'_after_detectors']}
        if num%30==0: log(f'unrel render {num}/{len(selected)}')
    write_batches('unrel',keys,UNREL_INSTRUCTIONS)
    log(f'unrel COMPLETE {len(selected)}/{n}')

def wilson(success,total):
    if not total: return {'rate':None,'n':0,'count':success,'ci95':[None,None]}
    z=1.959963984540054; p=success/total; den=1+z*z/total
    center=(p+z*z/(2*total))/den; half=z*math.sqrt(p*(1-p)/total+z*z/(4*total*total))/den
    return {'rate':p,'n':total,'count':success,'ci95':[max(0,center-half),min(1,center+half)]}

def summarize(rows):
    result={}
    rank={'correct':2,'slightly_off':1,'wrong':0}
    for group in GROUPS:
        sub=[r for r in rows if r['group']==group]; out={}
        for side in ['before','after']:
            counts={g:sum(r[side]==g for r in sub) for g in GRADES}; total=sum(counts[g] for g in rank)
            out[side]={'grades':counts,'accuracy':wilson(counts['correct'],total),'wrong_rate':wilson(counts['wrong'],total),
                       'available_rate':wilson(len(sub)-counts['absent'],len(sub))}
        wins={'after_win':0,'tie':0,'before_win':0,'not_comparable':0}
        for row in sub:
            if row['before'] not in rank or row['after'] not in rank: wins['not_comparable']+=1
            else:
                diff=rank[row['after']]-rank[row['before']]
                wins['after_win' if diff>0 else 'before_win' if diff<0 else 'tie']+=1
        out['paired_outcomes']=wins; result[group]=out
    return result

def validate_score(item,key,setname):
    if setname=='unrel':
        if item.get('verdict') not in VERDICTS: raise ValueError('Invalid verdict')
        return
    if not isinstance(item.get('note',''),str): raise ValueError('note must be a string')
    worst=item.get('worst',{})
    if not isinstance(worst,dict): raise ValueError('worst must be an object')
    for panel in ['A','B']:
        grades=item.get(panel)
        if not isinstance(grades,dict) or set(grades)!=set(GROUPS): raise ValueError(f'{panel} must contain exactly five groups')
        for group in GROUPS:
            grade=grades[group]
            if grade not in GRADES: raise ValueError(f'{panel}/{group}: invalid grade')
            drawn=[n for n in key['drawn_joints'][panel] if n in GROUPS[group]]
            if (grade=='absent') != (len(drawn)==0): raise ValueError(f'{panel}/{group}: absent inconsistent with drawn joints')
            label=worst.get(panel,{}).get(group)
            if label is not None and label not in [LABELS[n] for n in drawn]: raise ValueError(f'{panel}/{group}: invalid worst joint')

def unblind():
    rows=[]; unrel=[]; errors=[]; missing=[]; totals={}; item_results=[]
    for setname in ['hard','clean','unrel']:
        keypath=AUDIT/'key'/f'{setname}_key.json'
        if not keypath.exists():
            missing.append({'set':setname,'issue':'key missing'}); continue
        keys=json.loads(keypath.read_text(encoding='utf-8-sig')); seen=set(); candidates={}; invalid=set()
        for path in sorted((AUDIT/setname/'scores').glob('*.json')):
            try:
                scores=json.loads(path.read_text(encoding='utf-8-sig'))
                if not isinstance(scores,list): raise ValueError('batch must be a JSON list')
            except (ValueError,OSError) as exc:
                errors.append({'set':setname,'file':path.name,'error':str(exc)}); continue
            for item in scores:
                try:
                    if not isinstance(item,dict): raise ValueError('score must be an object')
                    number=item.get('item')
                    if not isinstance(number,int) or isinstance(number,bool): raise ValueError('item must be an integer')
                    sid=str(number)
                    if sid not in keys: raise ValueError('unknown item')
                    if sid in seen:
                        invalid.add(sid); raise ValueError('duplicate item; excluded from aggregation')
                    seen.add(sid); validate_score(item,keys[sid],setname)
                    candidates[sid]=item
                except (ValueError,TypeError,AttributeError) as exc:
                    errors.append({'set':setname,'file':path.name,'item':item.get('item') if isinstance(item,dict) else None,'error':str(exc)})
        valid={sid:item for sid,item in candidates.items() if sid not in invalid}
        for sid in sorted(keys,key=int):
            if sid not in valid: missing.append({'set':setname,'item':int(sid),'issue':'missing or invalid score'}); continue
            key=keys[sid]; item=valid[sid]
            if setname=='unrel':
                unrel.append({'set':setname,'item':int(sid),**key,'verdict':item['verdict'],'note':item.get('note','')}); continue
            bp=key['before_panel']; ap='B' if bp=='A' else 'A'
            item_rows=[]
            for group in GROUPS:
                row={'set':setname,'item':int(sid),'frame_id':key['frame_id'],'after_frame_id':key['after_frame_id'],'stratum':key['stratum'],
                     'group':group,'before':item[bp][group],'after':item[ap][group],'before_worst_joint':item.get('worst',{}).get(bp,{}).get(group,''),
                     'after_worst_joint':item.get('worst',{}).get(ap,{}).get(group,''),'note':item.get('note','')}
                rows.append(row); item_rows.append(row)
            rank={'correct':2,'slightly_off':1,'wrong':0}
            if any(r['before'] not in rank or r['after'] not in rank for r in item_rows): outcome='not_comparable'
            else:
                diff=sum(rank[r['after']]-rank[r['before']] for r in item_rows)
                outcome='after_win' if diff>0 else 'before_win' if diff<0 else 'tie'
            item_results.append({'set':setname,'item':int(sid),'outcome':outcome})
        totals[setname]={'key_items':len(keys),'valid_scored_items':len(valid)}
    results={'sets':{s:summarize([r for r in rows if r['set']==s]) for s in ['hard','clean']},'combined':summarize(rows),'coverage':totals,
             'missing_items':missing,'format_errors':errors,'item_outcomes':item_results,
             'item_outcome_counts':{s:{o:sum(x['outcome']==o and (s=='combined' or x['set']==s) for x in item_results) for o in ['after_win','tie','before_win','not_comparable']} for s in ['hard','clean','combined']}}
    clean=[r for r in rows if r['set']=='clean' and r['after'] in ['correct','slightly_off','wrong']]
    results['clean_after_residual_wrong_rate']=wilson(sum(r['after']=='wrong' for r in clean),len(clean))
    results['clean_after_residual_wrong_rate']['unit']='joint_group_item; correlated groups, descriptive binomial CI'
    def verdict_summary(sub):
        counts={v:sum(r['verdict']==v for r in sub) for v in VERDICTS}
        return {'verdicts':counts,'overflag_rate':wilson(counts['visible_and_correct'],len(sub)), 'visible_but_wrong_rate':wilson(counts['visible_but_wrong'],len(sub))}
    results['unreliable']={'combined':verdict_summary(unrel),'by_landmark':{k:verdict_summary([r for r in unrel if r['landmark']==k]) for k in sorted({r['landmark'] for r in unrel})},
                           'by_reason':{k:verdict_summary([r for r in unrel if r['reason']==k]) for k in sorted({r['reason'] for r in unrel})}}
    QC.mkdir(parents=True,exist_ok=True)
    write_json(QC/'blind_audit.json',results)
    columns=['set','item','frame_id','after_frame_id','stratum','group','before','after','before_worst_joint','after_worst_joint','note','landmark','reason','detectors','verdict']
    write_csv(QC/'blind_audit_items.csv',pd.DataFrame(rows+unrel).reindex(columns=columns))
    log(f'unblind COMPLETE: {len(rows)} A/B group ratings, {len(unrel)} unreliable ratings, {len(missing)} missing, {len(errors)} errors')

def selftest():
    w=wilson(5,10); assert w['ci95'][0]<.5<w['ci95'][1]
    assert wilson(0,0)['rate'] is None
    rows=[{'group':'elbows','before':'wrong','after':'correct'},{'group':'elbows','before':'absent','after':'correct'}]
    out=summarize(rows)['elbows']; assert out['paired_outcomes']['after_win']==1 and out['paired_outcomes']['not_comparable']==1
    key={'drawn_joints':{'A':[],'B':[]}}
    validate_score({'A':{g:'absent' for g in GROUPS},'B':{g:'absent' for g in GROUPS}},key,'hard')

def main():
    parser=argparse.ArgumentParser(description=__doc__); sub=parser.add_subparsers(dest='command',required=True)
    p=sub.add_parser('render'); p.add_argument('--set',choices=['hard','clean'],required=True); p.add_argument('--n',type=int); p.add_argument('--seed',type=int,default=2026)
    p=sub.add_parser('render-unreliable'); p.add_argument('--n',type=int,default=150); p.add_argument('--seed',type=int,default=2026)
    sub.add_parser('unblind'); sub.add_parser('selftest')
    args=parser.parse_args(); selftest()
    if getattr(args,'n',None) is not None and args.n<=0: parser.error('--n must be positive')
    if args.command=='render': render(args.set,args.n or (320 if args.set=='hard' else 640),args.seed)
    elif args.command=='render-unreliable': render_unreliable(args.n,args.seed)
    elif args.command=='unblind': unblind()
    else: log('SELFTEST PASS')

if __name__=='__main__': main()
