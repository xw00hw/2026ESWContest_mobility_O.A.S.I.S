"""UNAPPROVED CANDIDATE: one occupied G-hybrid RL trial, yaw20, pitch0, 300s.

This file does nothing on import. Hardware execution requires the exact
occupied-trial approval and all normal session confirmations. Source model
range stays 20 deg; independent downstream and raw-UART guards impose 20 deg.
Measured1G requests20 degrees BEFORE one signed filter and all existing limits.
This hybrid changes the base request; it is not an unchanged/retrained RL actor.
Fresh valid pressure input and the existing interlock remain. Baseline variance
above the 40-count diagnostic threshold is logged as a warning for this trial.
Initial FB_ARM applies neutral signals. Its movement from an unknown physical
shaft position is NOT bounded by the later 20-degree target clamp: current
mechanical neutral alignment and clearance must be confirmed before approval.
"""
from pathlib import Path
import argparse
import hashlib
import json
import math
import re
import signal
import sys
import time
import uuid
from raw_count_guard import RawCountGuard, attach_pressure_observer, attach_service_gate
from startup_alignment import align_startup_input, StartupAlignmentError


TRIAL_SECONDS = 300
BASELINE_STD_COUNTS = 40
BASELINE_VARIABILITY_MODE = 'warn'
APPROVAL = 'occupied_g_hybrid1g20_preview10_res1p5_pitch0_rate10_300s_raw2500_neutral18181212_optionalpose_baselinewarn_v3'
HYBRID_VERSION = 'measured_g_bounded_residual_v1'
HYBRID_CONFIG = dict(enabled=True,deadband_g=.01,measured_on_g=.02,measured_off_g=.01,
    knee_g=.0625,knee_yaw_deg=10.,full_g=1.,full_yaw_deg=20.,preview_on_g=.03,
    preview_off_g=.015,preview_full_g=.0625,preview_max_yaw_deg=10.,
    preview_confirm_s=.1,residual_max_deg=1.5,speed_min_mps=1.)
OPTIONAL_POSE_CONFIG = dict(source='udp',source_id='front_seat_pose',udp_port=47802,
    min_confidence=.5,lean_sign=1.,stale_s=1.)
MONITOR_LINK = '/home/esw/ESW_RL_FB/reconnect_20261028/live_control.jsonl'
CONFIRM = ('hardware_mapping_checked','neutral_start_clear','stm_tuning_reviewed',
           'steer_sign_checked','trial_settings_reviewed')
SAFE = frozenset((v+'\n').encode('ascii') for v in
    ('CONTROL_HELLO','FB_OFF','CAD_INFO','TUNE_STATE','DISABLE','HELLO','START_12,15','STATE','PING'))
SAFE = SAFE | {b'\nFB_OFF\nDISABLE\n'}


def validate_monitor_link(value):
    if value is None:return None
    if value!=MONITOR_LINK:
        raise ValueError('Monitor link is restricted to this experiment live_control.jsonl')
    return Path(value)


def publish_monitor_link(session_out, link):
    """Publish only an already-created journal, before ARM, by atomic symlink swap."""
    if link is None:return
    workspace=link.parent.resolve(strict=True)
    if workspace!=link.parent:
        raise ValueError('Monitor workspace must not redirect through a symlink')
    target=(Path(session_out)/'control.jsonl').resolve(strict=True)
    target.relative_to(workspace/'logs')
    if not target.is_file():raise ValueError('Session control journal is not a file')
    if link.exists() and not link.is_symlink():
        raise FileExistsError('Refusing to overwrite a regular monitor-path file')
    temporary=link.with_name('.live_control.'+uuid.uuid4().hex+'.tmp')
    try:
        temporary.symlink_to(target)
        temporary.replace(link)
    finally:
        if temporary.is_symlink():temporary.unlink()


def validate_optional_pose(cfg):
    """Allow disabled or exact optional same-Pi pose; no physical calibration claim.

    Runtime oasis.pose.v1 capture-time, confidence and neutral-valid gates remain
    responsible for rejecting invalid/stale input. Missing pose is not an ARM
    prerequisite and does not abort the otherwise unchanged BeamNG/RL pipeline.
    """
    if cfg.get('profile',{}).get('from_camera') is not False:
        raise ValueError('Metric body estimation from camera must stay explicitly false')
    pose=cfg.get('pose',{})
    if not isinstance(pose,dict):
        raise ValueError('Optional pose configuration must be an object')
    if pose.get('source')=='none':
        return
    if set(pose)!=set(OPTIONAL_POSE_CONFIG) or pose.get('source')!='udp' or pose.get('source_id')!='front_seat_pose':
        raise ValueError('Only disabled or reviewed local front_seat_pose UDP input is allowed')
    # Image-to-seat direction is measured during commissioning. Either sign
    # is valid; this does not assert that a physical alignment check happened.
    if type(pose['lean_sign']) not in (int,float) or pose['lean_sign'] not in (-1,1):
        raise ValueError('Optional pose lean_sign must be a measured +1 or -1')
    for key in ('udp_port','min_confidence','stale_s'):
        value=pose[key]
        if type(value) not in (int,float) or not math.isfinite(value) or value!=OPTIONAL_POSE_CONFIG[key]:
            raise ValueError('Optional pose freshness/confidence/port/sign differs from reviewed settings')
    if type(pose['udp_port']) is not int:
        raise ValueError('Optional pose UDP port must be an integer')


def validate_candidate(cfg, *, require_approval=True):
    hybrid=cfg.get('g_residual_control')
    if not isinstance(hybrid,dict) or set(hybrid)!=set(HYBRID_CONFIG) or hybrid.get('enabled') is not True:
        raise ValueError('Exact reviewed G-hybrid configuration must be explicitly enabled')
    if any(type(hybrid[k]) not in (int,float) or not math.isfinite(hybrid[k])
           or hybrid[k]!=v for k,v in HYBRID_CONFIG.items() if k!='enabled'):
        raise ValueError('G-hybrid parameters differ from reviewed 1G20/preview10/residual1.5 defaults')
    if cfg.get('dt_s')!=.05 or cfg.get('filter_tau_s')!=.3:
        raise ValueError('Reviewed 20Hz / single signed 0.3-second filter required')
    if float(cfg.get('theta_max_deg',0)) != 20:
        raise ValueError('Model theta_max_deg must stay 20; physical limit is downstream')
    if cfg.get('pitch_deg') != {'left':0.,'right':0.}:
        raise ValueError('Zero pitch presets required')
    validate_optional_pose(cfg)
    if cfg.get('rig') != 'static_bench': raise ValueError('static_bench required')
    if cfg.get('link',{}).get('firmware') != 'fallback_v2': raise ValueError('fallback_v2 required')
    if cfg.get('control',{}).get('mode') != 'rl': raise ValueError('RL required')
    if cfg.get('control',{}).get('cold_fallback') is not None:
        raise ValueError('Startup cold fallback must be disabled for this trial')
    p=cfg.get('pressure',{}); s=cfg.get('safety',{})
    exact_map={'side_hi_R':'A1_C1','side_hi_L':'A1_C3','side_lo_R':'A1_C0','side_lo_L':'A1_C2'}
    if any(p.get('channels',{}).get(k)!=v for k,v in exact_map.items()):
        raise ValueError('Actual side-channel map must match the fixed raw guard')
    if s.get('pad_saturation_counts') != 2500:
        raise ValueError('Existing interlock saturation must match absolute raw guard 2500')
    if float(p.get('zero_max_std_counts',-1)) != BASELINE_STD_COUNTS or not 0 < float(p.get('zero_window_s',0)) <= 3:
        raise ValueError('Existing 40-count baseline diagnostic threshold and bounded window required')
    if p.get('baseline_variability_mode') != BASELINE_VARIABILITY_MODE:
        raise ValueError('This trial requires explicit baseline variance warn mode')
    if float(p.get('no_pressure_yaw_cap_deg',99)) != 20:
        raise ValueError('Valid zero-contact readings retain yaw20; raw freshness guard remains independent')
    if s.get('arm_deg') != 0 or s.get('mode') != 'raw':
        raise ValueError('Raw pressure interlock armed above 0 deg required')
    soft=float(s.get('fsr_force_soft_N',float('nan'))); hard=float(s.get('fsr_force_hard_N',float('nan')))
    if not (math.isfinite(soft) and math.isfinite(hard) and 0 < soft < hard):
        raise ValueError('Finite existing pressure protection limits required')
    dep=cfg.get('deployment',{})
    if dep.get('baseline_variability_mode') != BASELINE_VARIABILITY_MODE:
        raise ValueError('Deployment baseline variance mode must explicitly match warn')
    if dep.get('pressure_masking') is not False or dep.get('physical_trial_yaw_cap_deg') != 20:
        raise ValueError('Explicit real-pressure, 20-degree candidate required')
    if dep.get('raw_release_counts') != 2500 or dep.get('raw_release_latched') is not True:
        raise ValueError('Exact per-pad 2500 raw-count latched release required')
    if dep.get('neutral_pulses_us') != [1800,1800,1200,1200] or dep.get('neutral_settle_s') != 2.:
        raise ValueError('New confirmed neutral pulses and 2-second settle required')
    if float(p.get('zero_window_s',0)) != 2.:
        raise ValueError('Existing exact 2-second pressure baseline required')
    revision=dep.get('expected_tuning_revision')
    if type(revision) is not int or revision!=2:
        raise ValueError('Reviewed STM tuning revision2 required; a reset must be requalified')
    if dep.get('arm_time_budget_s')!=TRIAL_SECONDS or dep.get('policy')!='g_residual_rl':
        raise ValueError('Explicit 300-second G-hybrid trial identity required')
    if require_approval:
        if any(cfg.get('confirmations',{}).get(k) is not True for k in CONFIRM):
            raise ValueError('Normal hardware confirmations not all true; no device opened')
        if dep.get('approved_trial') != APPROVAL or dep.get('occupied_trial_accepted') is not True:
            raise ValueError('Specific occupied 20-degree/raw2500 trial not accepted; no device opened')
        if dep.get('operator_stop_ready') is not True:
            raise ValueError('Operator stop readiness not confirmed; no device opened')


def validate_hybrid_policy(ctl):
    """Reject an old runtime which silently ignored the optional hybrid section."""
    governor=getattr(ctl,'g_governor',None)
    expected={k:v for k,v in HYBRID_CONFIG.items() if k!='enabled'}
    if governor is None or getattr(governor,'version',None)!=HYBRID_VERSION:
        raise RuntimeError('Actual reviewed G-hybrid governor missing; refusing ARM')
    if governor.config.to_dict()!=expected or getattr(ctl,'g_filter',None) is None:
        raise RuntimeError('Actual G-hybrid defaults/signed filter mismatch; refusing ARM')
    if not callable(getattr(ctl,'cfg',{}).get('_training_prior')):
        raise RuntimeError('Pinned model training prior missing; refusing ARM')


class StopMonitor:
    def __init__(self,path,clock=time.monotonic):
        self.path=Path(path); self.clock=clock; self.arm_at=None; self.reason=None
    def arm_attempted(self):
        if self.arm_at is not None: raise RuntimeError('One ARM only')
        self.arm_at=self.clock()
    def check(self):
        if self.reason: return self.reason
        if self.path.exists(): self.reason='operator_stop_file'
        elif self.arm_at is not None and self.clock()-self.arm_at >= TRIAL_SECONDS:
            self.reason='occupied_trial_300s_elapsed'
        return self.reason


class LimitedTransport:
    """Exact safe commands and one authorized session; every SET <=20/0/20/0."""
    def __init__(self,raw,monitor,pressure_guard,*,alignment_clock=time.monotonic,alignment_sleep=time.sleep):
        if raw.timeout != 0 or raw.write_timeout != 0: raise ValueError('Nonblocking UART required')
        self.__raw=raw; self.monitor=monitor; self.pressure_guard=pressure_guard; self.timeout=self.write_timeout=0
        self.authorized_arm=None; self.arm_sent=False; self.allowed=[]; self.blocked=[]; self.closed=False
        self._alignment_clock,self._alignment_sleep=alignment_clock,alignment_sleep
        self._alignment_attempted=self._reset_consumed=False
        self.startup_alignment=None; self.startup_trace=[]; self._trace_bytes=0
        self._trace_done=self._hello_requested=False; self._hello_tail=b''
        self.startup_trace_truncated=False
    @property
    def in_waiting(self): return self.__raw.in_waiting
    def _capture_startup(self,data,phase):
        if self._trace_done or self.arm_sent: return
        remaining=8192-self._trace_bytes
        if remaining<=0 or len(self.startup_trace)>=256:
            self.startup_trace_truncated=True;self._trace_done=True;return
        chunk=data[:remaining];self._trace_bytes+=len(chunk)
        self.startup_trace.append(dict(phase=phase,monotonic_s=self._alignment_clock(),
            rx=chunk.decode('ascii',errors='backslashreplace')))
        if len(chunk)<len(data):self.startup_trace_truncated=True;self._trace_done=True
        if phase=='control_hello':
            self._hello_tail=(self._hello_tail+data)[-1024:]
            begin=self._hello_tail.find(b'#HELLO,OASIS_FALLBACK,')
            if begin>=0 and b'\n' in self._hello_tail[begin:]:self._trace_done=True
    def read(self,n):
        data=self.__raw.read(n)
        if data and self._hello_requested:self._capture_startup(data,'control_hello')
        return data
    def prime_startup(self):
        if self._alignment_attempted or self.allowed or self.arm_sent or self.closed:
            raise RuntimeError('Startup alignment allowed once, before every UART command')
        self._alignment_attempted=True
        try:
            self.startup_alignment=align_startup_input(self.__raw,
                clock=self._alignment_clock,sleep=self._alignment_sleep,
                trace=lambda data:self._capture_startup(data,'pre_handshake_discard'))
        except StartupAlignmentError as exc:
            self.startup_alignment=exc.report
            raise
    def reset_input_buffer(self):
        # BoardLink captures its ACK clock before this call. Priming just before
        # Session.run completes the bounded wait before that clock is captured.
        if self._reset_consumed or self.allowed or self.arm_sent or not self.startup_alignment or not self.startup_alignment.get('success'):
            raise RuntimeError('Startup must be successfully primed once before protocol start')
        self._reset_consumed=True
    def startup_diagnostics(self):
        return dict(alignment=self.startup_alignment,trace=self.startup_trace,
                    trace_bytes=self._trace_bytes,trace_truncated=self.startup_trace_truncated)
    def close(self):
        if not self.closed: self.closed=True; self.__raw.close()
    def authorize_one_arm(self,sid,revision):
        if self.authorized_arm is not None or type(sid) is not int or type(revision) is not int:
            raise ValueError('One exact session authorization required')
        if not 1 <= sid <= 0xffffffff or not 0 <= revision <= 0xffffffff: raise ValueError('Invalid session/revision')
        self.authorized_arm=(sid,revision)
    def _reject(self,payload,reason):
        self.blocked.append(dict(payload=repr(payload),reason=reason))
        raise PermissionError('Occupied trial UART guard: '+reason)
    def write(self,payload):
        if type(payload) is not bytes: return self._reject(payload,'bytes required')
        if payload in SAFE:
            if payload==b'CONTROL_HELLO\n':self._hello_requested=True
            self.allowed.append(payload.decode('ascii')); return self.__raw.write(payload)
        if self.monitor.check(): return self._reject(payload,self.monitor.reason)
        if self.authorized_arm is None: return self._reject(payload,'motion session not authorized')
        sid,rev=self.authorized_arm
        arm=f'FB_ARM,{sid},{rev}\n'.encode('ascii')
        if payload==arm:
            if self.arm_sent: return self._reject(payload,'second ARM denied')
            self.pressure_guard.activate()
            self.arm_sent=True; self.monitor.arm_attempted()
        else:
            if not self.arm_sent: return self._reject(payload,'ARM not sent')
            try: text=payload.decode('ascii')
            except UnicodeError: return self._reject(payload,'ASCII required')
            m=re.fullmatch(r'FB_PING,([0-9]{1,10}),([0-9]{1,10})\n',text)
            if m:
                if int(m[1])!=sid or not 1 <= int(m[2]) <= 0xffffffff:
                    return self._reject(payload,'invalid lease session/id')
            else:
                m=re.fullmatch(r'FB_SET,([0-9]{1,10}),([0-9]{1,10}),([0-9]{1,5}),0,([0-9]{1,5}),0\n',text)
                if not m or int(m[1])!=sid or not 1<=int(m[2])<=0xffffffff or not (0<=int(m[3])<=20000 and 0<=int(m[4])<=20000):
                    return self._reject(payload,'target outside 20-degree yaw / zero pitch or unsupported command')
                if not self.pressure_guard.permits_mdeg(int(m[3]),int(m[4])):
                    return self._reject(payload,'latched or newly detected raw-pressure release forbids closing')
        self.allowed.append(payload.decode('ascii'))
        return self.__raw.write(payload)


class TargetLimiter:
    def __init__(self,journal,pressure_guard,clock_ns=time.monotonic_ns):
        self.journal,self.clock_ns=journal,clock_ns
        self.pressure_guard=pressure_guard
        self.calls=self.clamps=0
    def degrees(self,pose):
        if len(pose)!=4 or not all(math.isfinite(float(x)) for x in pose):
            raise ValueError('Finite LY,LP,RY,RP required')
        value=tuple(map(float,pose)); out=(max(0.,min(20.,value[0])),0.,max(0.,min(20.,value[2])),0.)
        out=self.pressure_guard.gate(out)
        self.calls+=1
        if out!=value:
            self.clamps+=1
            self.journal.record('occupied_trial_target_clamped',host_monotonic_ns=self.clock_ns(),
                requested_LY_LP_RY_RP_deg=value,limited_LY_LP_RY_RP_deg=out)
        return out


def attach_neutral_settle(session, clock_ns=time.monotonic_ns):
    """One 2s neutral hold before unchanged baseline, inside the ARM time budget."""
    original_start=session.policy.start
    started=False
    def start(s):
        nonlocal started
        if started: raise RuntimeError('No baseline/settle retry allowed')
        started=True
        s.phase='NEUTRAL_SETTLE'
        began=clock_ns()
        s.link.pressure_log.record('neutral_settle_started',host_monotonic_ns=began,duration_s=2.)
        s.wait_neutral(2.)
        if s.link.fault: raise RuntimeError(s.link.fault)
        ended=clock_ns()
        s.link.pressure_log.record('neutral_settle_completed',host_monotonic_ns=ended,elapsed_s=(ended-began)/1e9)
        s.phase='BASELINE'
        return original_start(s)
    session.policy.start=start


def run_trial(args):
    runtime=args.runtime_root.resolve(strict=True); sys.path.insert(0,str(runtime))
    from oasis_rt import app,controller as C
    cfg=C.load_config(str(args.config.resolve(strict=True))); cfg['_config_path']=str(args.config.resolve())
    validate_candidate(cfg)
    monitor_link=validate_monitor_link(getattr(args,'monitor_link',None))
    if args.stop_file.exists(): raise RuntimeError('STOP file already exists; no device opened')
    man,obs,actor=C.prepare_policy(cfg,print)
    if cfg['control']['mode']!='rl' or getattr(actor,'name',None) not in ('c','numpy') or man['action_dim']!=2:
        raise RuntimeError('Actual verified yaw-only RL actor required before opening hardware')
    import fcntl
    lock=(Path.home()/'ESW_RL_FB/rl_trial.lock').open('a+')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    monitor=StopMonitor(args.stop_file)
    session=None; guarded=None; fb=None; limiter=None; result=None; cleanup=[]
    pressure_guard=RawCountGuard(lambda: (session.link.pressure, session.link.pressure_at) if session else ({},float("nan")))
    signal_seen=False
    def stop_signal(_sig,_frame):
        nonlocal signal_seen
        if signal_seen: return
        signal_seen=True; monitor.reason='operator_signal'; raise KeyboardInterrupt
    previous={sig:signal.signal(sig,stop_signal) for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP)}
    # Guard is installed at the only serial factory while assembling; no unguarded link exists.
    from oasis_rt import fbstack
    fb=fbstack.load(cfg.get('fb_root')); serial_factory=fb.serial_link.open_serial
    def guarded_factory(port):
        nonlocal guarded
        if guarded is not None: raise RuntimeError('No reconnect or second UART allowed')
        raw=serial_factory(port)
        try: guarded=LimitedTransport(raw,monitor,pressure_guard)
        except BaseException: raw.close(); raise
        return guarded
    try:
        fb.serial_link.open_serial=guarded_factory
        try: fb,session,_=app.assemble(cfg,'hw',print)
        finally: fb.serial_link.open_serial=serial_factory
        publish_monitor_link(session.out,monitor_link)
        pressure_guard.journal=session.link.pressure_log
        attach_pressure_observer(session.link.pressure_log,pressure_guard)
        limiter=TargetLimiter(session.link.pressure_log,pressure_guard)
        send=session.link.send_targets; set_pose=session.link.set_pose_mdeg
        session.link.send_targets=lambda *p:send(*limiter.degrees(p))
        session.link.set_pose_mdeg=lambda p:set_pose(tuple(round(x*1000) for x in limiter.degrees(tuple(v/1000 for v in p))))
        attach_service_gate(session.link,pressure_guard,limiter)
        original_pump=session.pump
        def pump():
            why=monitor.check()
            if why:
                session.link.stop(why)
                raise RuntimeError(why)
            original_pump()
        session.pump=pump
        original_bind=session.policy.bind
        def bind(s):
            original_bind(s)
            if not isinstance(s.policy,app.RlPolicy) or s.policy.ctl.mode!='rl' or s.policy.man['action_dim']!=2:
                raise RuntimeError('Actual RL model/actuator qualification failed; refusing ARM')
            validate_hybrid_policy(s.policy.ctl)
            if s.link.hello.get('neutral_us') != '1800:1800:1200:1200':
                raise RuntimeError('New confirmed four-motor neutral calibration required; refusing ARM')
            tune=s.link.tuning
            if tune['yaw_max_mdeg']!=20000 or tune['yaw_rate_mdeg_s']!=10000 or tune['revision']!=cfg['deployment']['expected_tuning_revision']:
                raise RuntimeError('Board must match reviewed yaw20 / rate10 / exact revision; refusing ARM')
            if s.link.env.yaw_max!=20000:
                raise RuntimeError('Actual board envelope changed; refusing ARM')
            if s.policy.ctl.ilock.arm!=0 or s.policy.ctl.ilock.model:
                raise RuntimeError('Real raw pressure interlock not active at low angles')
            guarded.authorize_one_arm(s.link.hello['session_floor']+1,tune['revision'])
        session.policy.bind=bind
        attach_neutral_settle(session)
        snapshot={k:v for k,v in cfg.items() if not k.startswith('_')}
        with (Path(session.out)/'occupied_trial_config.json').open('x') as h: json.dump(snapshot,h,indent=2)
        try:
            guarded.prime_startup()  # Only pre-handshake; never reset an active protocol stream.
        finally:
            with (Path(session.out)/'startup_uart_before_protocol.json').open('x') as h:
                json.dump(guarded.startup_diagnostics(),h,indent=2)
        result=session.run(TRIAL_SECONDS,startup_timeout_s=15)
        manifest=app.manifest(cfg,fb,session,result,session.policy)
        manifest['occupied_limited_trial']=dict(approval=APPROVAL,occupied=True,
            policy='g_residual_rl',g_residual_control=cfg['g_residual_control'],
            full_g_endpoint_semantics='1G requests20deg before signed filter and all independent limits',
            physical_yaw_max_deg=20,pitch_deg=0,arm_time_budget_s=TRIAL_SECONDS,arm_budget_includes_baseline=True,
            model_theta_max_deg=20,pressure_masking=False,pressure_force_calibrated=False,
            neutral_pulses_us=[1800,1800,1200,1200],neutral_settle_s=2.,baseline_window_s=2.,
            baseline_max_std_counts=BASELINE_STD_COUNTS,baseline_variability_mode=BASELINE_VARIABILITY_MODE,
            raw_pressure_guard=pressure_guard.summary(),startup_uart=guarded.startup_diagnostics(),
            initial_arm_applies_neutral=True,
            initial_unknown_shaft_displacement_not_bounded_by_target_clamp=True,
            stop_file=str(args.stop_file),stop_reason=monitor.reason,
            arm_attempted_monotonic_s=monitor.arm_at,clamp_calls=limiter.calls,clamped=limiter.clamps,
            allowed_uart_writes=guarded.allowed,blocked_uart_writes=guarded.blocked,
            script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            raw_guard_sha256=hashlib.sha256(Path(__file__).with_name('raw_count_guard.py').read_bytes()).hexdigest(),
            startup_alignment_sha256=hashlib.sha256(Path(__file__).with_name('startup_alignment.py').read_bytes()).hexdigest(),
            config_sha256=hashlib.sha256(args.config.read_bytes()).hexdigest())
        with (Path(session.out)/'manifest.json').open('x') as h: json.dump(manifest,h,indent=2,default=str)
        print(json.dumps({'log':session.out,'result':result,'occupied_limited_trial':manifest['occupied_limited_trial']},indent=2))
    finally:
        fb.serial_link.open_serial=serial_factory
        for sig in previous: signal.signal(sig,signal.SIG_IGN)
        if session is not None:
            actions=(('board',lambda:session.link.close(time.sleep) if not session.link.b._closed else None),
                     ('vehicle',session.vin.close),('pose',lambda:app.close_pose(session.pose)),
                     ('journal',session.link.pressure_log.close))
            for name,action in actions:
                try: action()
                except Exception as exc: cleanup.append(name+': '+repr(exc))
        elif guarded is not None:
            try: guarded.close()
            except Exception as exc: cleanup.append('serial: '+repr(exc))
        if cleanup: print(json.dumps({'cleanup_errors':cleanup}))
        for sig,old in previous.items(): signal.signal(sig,old)
        lock.close()
    return 0 if result and result.get('off_verified') and not cleanup and not guarded.blocked and result['result'] in ('completed','operator_stop','occupied_trial_300s_elapsed','operator_stop_file') else 1


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--runtime-root',required=True,type=Path)
    p.add_argument('--config',required=True,type=Path)
    p.add_argument('--stop-file',required=True,type=Path)
    p.add_argument('--monitor-link',help='Optional fixed reconnect experiment live_control.jsonl symlink')
    args=p.parse_args(argv)
    return run_trial(args)


if __name__=='__main__': raise SystemExit(main())
