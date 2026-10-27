"""Structure poses: ACK -> commanded arrival -> settle -> measurement."""
import csv
import json
import math
import time

from kinematics import inverse,pair_pulses,POSE_FIELDS,YAW_LIMIT_DEG,PITCH_LIMIT_DEG,YAW_SCAN_LIMIT_DEG

POSE_DEG_FIELDS = tuple(name.replace('_mdeg','_deg') for name in POSE_FIELDS)

PLAN_FIELDS = ['point_index','yaw_deg','pitch_deg','theta1_deg','theta2_deg',
               *POSE_DEG_FIELDS,
               'left_theta1_deg','left_theta2_deg','right_theta1_deg','right_theta2_deg',
               'expected_D3_us','expected_D4_us','expected_D5_us','expected_D6_us']
POINT_FIELDS = PLAN_FIELDS+['command_id','queued_ns','tx_ns','ack_ns','duration_ms',
    'arrived_ns','arrived_mcu_ms',*('reported_'+name for name in POSE_FIELDS),
    'reported_D3_us','reported_D4_us','reported_D5_us','reported_D6_us',
    'dwell_start_ns','dwell_end_ns','end_state_ns','end_mcu_ms',
    'status','reason']


def grid(mode='grid'):
    if mode=='yaw':
        return [(left,0,right,0) for left in range(YAW_SCAN_LIMIT_DEG+1) for right in
                (range(YAW_SCAN_LIMIT_DEG+1) if left%2==0 else range(YAW_SCAN_LIMIT_DEG,-1,-1))]
    if mode!='grid':
        raise ValueError('AUTO mode: grid / yaw')
    return [(y,p) for y in range(YAW_LIMIT_DEG+1) for p in
            (range(-PITCH_LIMIT_DEG,PITCH_LIMIT_DEG+1) if y%2==0 else
             range(PITCH_LIMIT_DEG,-PITCH_LIMIT_DEG-1,-1))]


class AutoScan:
    def __init__(self,session,send,send_pose,settle_s=.5,dwell_s=2,clock=time.perf_counter_ns,
                 side='both',initial_pose=(0,0,0,0),mode='grid'):
        if not all(math.isfinite(x) and 0<=x<=60 for x in (settle_s,dwell_s)) or dwell_s<=0:
            raise ValueError('AUTO 안정 시간 0~60초, 측정 시간 0초 초과~60초가 필요합니다.')
        self.session,self.send,self.send_pose,self.clock = session,send,send_pose,clock
        if side not in ('both','left','right'):
            raise ValueError('AUTO 대상: both / left / right')
        pair_pulses(*(round(x*1000) for x in initial_pose))
        side = 'both' if mode=='yaw' else side
        self.side,self.initial_pose = side,tuple(initial_pose)
        self.settle_ns,self.dwell_ns = round(settle_s*1e9),round(dwell_s*1e9)
        self.plan = []
        for i,point in enumerate(grid(mode),1):
            y,p = point[:2]
            motors = inverse(y,p)['motors']
            pose = point if mode=='yaw' else self.pose(y,p)
            right = inverse(*pose[2:])['motors']
            left = inverse(*pose[:2])['motors']
            self.plan.append(dict(point_index=i,yaw_deg=y,pitch_deg=p,
                **dict(zip(POSE_DEG_FIELDS,pose)),
                left_theta1_deg=left[0]['phi_deg'],left_theta2_deg=left[1]['phi_deg'],
                right_theta1_deg=right[0]['phi_deg'],right_theta2_deg=right[1]['phi_deg'],
                theta1_deg=motors[0]['phi_deg'],theta2_deg=motors[1]['phi_deg'],
                **dict(zip([f'expected_D{i}_us' for i in (3,4,5,6)],pair_pulses(*(round(x*1000) for x in pose))))))
        with (session.directory/'auto_plan.csv').open('x',encoding='utf-8',newline='') as file:
            writer = csv.DictWriter(file,fieldnames=PLAN_FIELDS)
            writer.writeheader(); writer.writerows(self.plan)
        session.add_table('auto_points',POINT_FIELDS)
        session.add_table('auto_events',['point_index','command_id','phase','event','detail'])
        self.info = dict(schema='oasis-auto-v2',status='preparing',total=len(self.plan),completed=0,
            side=side,initial_pose_deg=list(initial_pose),mode=mode,
            step_deg=1,settle_s=settle_s,dwell_s=dwell_s,order='serpentine',
            arrival='MCU command state only; no measured shaft/structure feedback',
            measurement_window='[dwell_start_ns,dwell_end_ns), host perf_counter_ns')
        session.metadata['auto_scan'] = self.info
        session._json('metadata.json',session.metadata)
        self.active,self.current,self.returning = True,None,False
        self.stage,self.index,self.tx_ns,self.last_state_ns = 'idle',0,None,self.clock()
        self.last_state = None
        self._control('HOLD','holding')

    def pose(self,y,p):
        return ((y,p) if self.side!='right' else self.initial_pose[:2])+(
                (y,p) if self.side!='left' else self.initial_pose[2:])

    def timing(self,ns=None):
        ns = self.clock() if ns is None else ns
        return ns,self.session.metadata['origin_wall_time_ns']+ns-self.session.origin_ns

    def log(self,event,timing=None,**detail):
        current = self.current or {}
        self.session.row('auto_events',dict(point_index=current.get('point_index',0),
            command_id=current.get('command_id'),phase=self.stage,event=event,
            detail=json.dumps(detail,ensure_ascii=False)),timing or self.timing())

    def _control(self,command,stage):
        self.stage,self.awaited,self.tx_ns = stage,command,None
        self.deadline = self.clock()+5_000_000_000
        self.send(command)
        self.log('control_queued',command=command)

    def _pose(self,point,returning=False):
        self.current,self.returning = dict(point),returning
        self.current['queued_ns'] = self.clock()
        pose = tuple(point[name] for name in POSE_DEG_FIELDS) if all(name in point for name in POSE_DEG_FIELDS) else self.pose(point['yaw_deg'],point['pitch_deg'])
        self.current.update(zip(POSE_DEG_FIELDS,pose))
        self.current['command_id'] = self.send_pose(*pose)
        self.awaited = f"SET,{self.current['command_id']},"+','.join(str(round(x*1000)) for x in pose)
        self.stage,self.tx_ns = 'sending',None
        self.deadline = self.clock()+5_000_000_000
        self.log('pose_queued')

    def event(self,event):
        if not self.active:
            return
        timing = event.get('timing',self.timing())
        ns = timing[0]
        if event['type']=='tx' and event['text']==self.awaited:
            self.tx_ns = ns
            self.deadline = ns+2_000_000_000
            if self.stage=='sending':
                self.stage = 'ack'
                self.current['tx_ns'] = ns
            self.log('tx',timing,text=event['text'])
        elif event['type']=='reply' and self.tx_ns is not None and ns>=self.tx_ns:
            text = event['text']
            if text.startswith('#ERR,'):
                self.abort(text); return
            if self.stage=='holding' and text=='#ACK,HOLD':
                self.log('hold_ack',timing)
                self._control('ARM','arming')
            elif self.stage=='arming' and text=='#ACK,ARM':
                self.stage,self.deadline = 'armed',ns+2_000_000_000
                self.log('arm_ack',timing)
                self.send('STATE')
            elif self.stage=='finish_hold' and text=='#ACK,HOLD':
                self.active = False
                self.info['status'] = 'complete'
                self.stage = 'complete'
                self.log('complete',timing)
                self.session._json('auto_progress.json',self.info)
            elif self.stage=='ack' and text.startswith('#ACK,SET,'):
                fields = text.split(',')
                c = self.current
                expected=[str(c['command_id']),*(str(round(c[name]*1000)) for name in POSE_DEG_FIELDS)]
                if len(fields)!=8 or fields[2:7]!=expected:
                    self.abort('SET_ACK_MISMATCH'); return
                if not fields[7].isdigit():
                    self.abort('INVALID_DURATION'); return
                duration = int(fields[7])
                if not 0<duration<=120000:
                    self.abort('INVALID_DURATION'); return
                c.update(ack_ns=ns,duration_ms=duration)
                self.stage,self.deadline = 'moving',ns+duration*1_000_000+10_000_000_000
                self.log('set_ack',timing,duration_ms=duration)
        elif event['type']=='state':
            state = event['state']
            self.last_state,self.last_state_ns = state,ns
            if self.stage=='armed':
                if state['armed'] and not state['moving'] and state['fault']=='NONE':
                    self._pose(self.plan[0])
                return
            if self.stage not in ('moving','settle','dwell'):
                return
            c = self.current
            if not state['armed'] or state['fault']!='NONE':
                self.abort('MCU_DISARMED_OR_FAULT'); return
            wanted=tuple(round(c[name]*1000) for name in POSE_DEG_FIELDS)
            if state['command_id']!=c['command_id'] or tuple(state['target_'+name] for name in POSE_FIELDS)!=wanted:
                self.abort('COMMAND_AUTHORITY_CHANGED'); return
            arrived = not state['moving'] and tuple(state[name] for name in POSE_FIELDS)==wanted
            if self.stage!='moving' and not arrived:
                self.abort('POSE_CHANGED_DURING_MEASUREMENT'); return
            if self.stage=='moving' and arrived:
                if [state[f'D{i}_us'] for i in (3,4,5,6)]!=pair_pulses(*wanted):
                    self.abort('PWM_MISMATCH'); return
                c.update(arrived_ns=ns,arrived_mcu_ms=state['mcu_ms'])
                c.update(**{'reported_'+name:state[name] for name in POSE_FIELDS},
                         **{f'reported_D{i}_us':state[f'D{i}_us'] for i in (3,4,5,6)})
                self.log('command_arrived',timing)
                if self.returning:
                    self._control('HOLD','finish_hold')
                else:
                    c['dwell_start_ns'] = ns+self.settle_ns
                    c['dwell_end_ns'] = c['dwell_start_ns']+self.dwell_ns
                    self.stage = 'settle'

    def tick(self):
        if not self.active:
            return
        now = self.clock()
        if now-self.last_state_ns>1_000_000_000:
            self.abort('STATE_STALE'); return
        if self.stage in ('holding','arming','armed','sending','ack','moving','finish_hold') and now>self.deadline:
            self.abort('TIMEOUT_'+self.stage.upper()); return
        c = self.current
        if self.stage=='settle' and now>=c['dwell_start_ns']:
            self.stage = 'dwell'
            self.log('dwell_begin',self.timing(c['dwell_start_ns']))
        if self.stage=='dwell' and now>=c['dwell_end_ns'] and self.last_state_ns>=c['dwell_end_ns']:
            c.update(end_state_ns=self.last_state_ns,end_mcu_ms=self.last_state['mcu_ms'],status='complete',reason='')
            self.session.row('auto_points',c,self.timing())
            self.log('dwell_end',self.timing(c['dwell_end_ns']))
            self.index += 1
            self.info['completed'] = self.index
            self.info['status'] = 'running'
            self.session._json('auto_progress.json',self.info)
            if self.index<len(self.plan):
                self._pose(self.plan[self.index])
            else:
                self._pose(dict(point_index=0,yaw_deg=0,pitch_deg=0),returning=True)

    def abort(self,reason,status='failed'):
        if not self.active:
            return
        self.active = False
        self.info.update(status=status,reason=reason)
        hold_error = None
        try:
            self.send('HOLD')
        except Exception as error:
            hold_error = str(error)
        if self.current and not self.returning:
            self.current.update(status=status,reason=reason)
            self.session.row('auto_points',self.current,self.timing())
        self.log(status,reason=reason)
        self.session.error('auto_'+status+': '+reason)
        self.session._json('auto_progress.json',self.info)
        if hold_error:
            self.log('hold_not_sent',reason=hold_error)
