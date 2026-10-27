"""20 Hz bolster control loop on the Raspberry Pi 5.

    python -m oasis_rt.controller --config config.yaml              # car / bench, real STM32
    python -m oasis_rt.controller --config config.yaml --mock       # PC/Pi SIL: MockStm32 + replayed drive

tick (every dt_s, absolute schedule):
  poll STM32 -> vehicle + pose sources -> predictor -> obs_layout.build_obs (the training contract)
  -> history -> actor (C / numpy / Hailo) -> 0.3 s low-pass -> [c, d(, cp, dp)] -> yaw L/R + pitch L/R
     (v3 model: obs_layout.action_to_targets, the training mapping; v2 model: yaw only, pitch preset)
  -> interlock (wing force over its pad zones) -> stale-input gates -> Stm32Link.send_targets
  -> one log row (obs + actions + feedback: closed-loop data with all three axes for the rebuild).
The STM32 turns (yaw, pitch) into crank angles / servo pulses (C4_R8 IK) and enforces bounds,
rate and the 1.5 s heartbeat; this loop never talks PWM.
"""
import argparse, csv, gc, hashlib, importlib.util, json, os, sys, time
import numpy as np

from .actor import History, OutputFilter, RuleActor, make_actor
from .safety import Interlock, FsrInverse
from .stm32_link import Stm32Link, SerialTransport
from . import predictor as PR, sources as SRC

G = 9.81


def _sha(p): return hashlib.sha256(open(p, 'rb').read()).hexdigest()


def load_model(model_dir):
    man = json.load(open(os.path.join(model_dir, 'manifest.json')))
    p = os.path.join(model_dir, 'obs_layout.py')
    if _sha(p) != man['files']['obs_layout.py']:
        raise RuntimeError('obs_layout.py does not match the manifest -- re-export the model')
    spec = importlib.util.spec_from_file_location('oasis_obs_layout', p)
    obs = importlib.util.module_from_spec(spec); spec.loader.exec_module(obs)
    if obs.VERSION != man['obs_version'] or obs.OBS_DIM != man['OD']:
        raise RuntimeError(f'observation contract {obs.VERSION}/{obs.OBS_DIM} != model {man["obs_version"]}/{man["OD"]}')
    return man, obs


def realtime_setup(rt, log):
    """Best effort: pin to an isolated core, SCHED_FIFO, lock memory, freeze the GC heap."""
    if not rt: return
    if rt.get('cpu') is not None and hasattr(os, 'sched_setaffinity'):
        try: os.sched_setaffinity(0, {int(rt['cpu'])}); log(f'rt: pinned to cpu {rt["cpu"]}')
        except OSError as e: log(f'rt: affinity failed ({e})')
    if rt.get('fifo_priority') and hasattr(os, 'sched_setscheduler'):
        try: os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(int(rt['fifo_priority']))); log('rt: SCHED_FIFO')
        except (OSError, PermissionError) as e: log(f'rt: SCHED_FIFO failed ({e}); run with CAP_SYS_NICE (see systemd unit)')
    if rt.get('mlock') and sys.platform.startswith('linux'):
        try:
            import ctypes
            if ctypes.CDLL('libc.so.6', use_errno=True).mlockall(3) != 0: raise OSError(ctypes.get_errno(), 'mlockall')
            log('rt: memory locked')
        except OSError as e: log(f'rt: mlockall failed ({e})')
    if rt.get('gc_freeze'):
        gc.collect(); gc.freeze(); gc.disable(); log('rt: gc frozen/disabled (collected in slack time)')


class Controller:
    def __init__(self, cfg, link, actor, obs, predictor, vehicle, pose, clock, log=print):
        self.cfg, self.link, self.actor, self.obs, self.pred = cfg, link, actor, obs, predictor
        self.vehicle, self.pose, self.clock, self.log = vehicle, pose, clock, log
        self.th_max = float(cfg['theta_max_deg']); self.dt = float(cfg['dt_s'])
        self.pitch = (float(cfg['pitch_deg']['left']), float(cfg['pitch_deg']['right']))   # v2 models: preset
        self.v3 = hasattr(obs, 'action_to_targets')                                     # v3: policy drives pitch
        self.pmax = float(getattr(obs, 'PITCH_MAX_DEG', 8.0))
        self.hist = History(actor.OD if hasattr(actor, 'OD') else obs.OBS_DIM, cfg['_K'])
        self.filt = OutputFilter(cfg['filter_tau_s'], self.dt)
        P = cfg['pressure']; self.ch = P['channels']
        # pad zones of each wing used for the interlock force: v3 lower + upper pad, v2 the one side pad
        self.wing = {'L': ['side_lo_L', 'side_hi_L'], 'R': ['side_lo_R', 'side_hi_R']} if self.v3 else {'L': ['side_L'], 'R': ['side_R']}
        miss = [k for w in self.wing.values() for k in w if k not in self.ch]
        if miss: raise RuntimeError(f'pressure.channels lacks {miss} for this model')
        self.cpn = {k: float(P['counts_per_N'][self.ch[k]]) for w in self.wing.values() for k in w}
        # interlock force: nonlinear FSR curve if configured (the pads saturate), else linear counts_per_N
        self.fsr = FsrInverse(cfg['_fsr_curve']) if cfg.get('_fsr_curve') else None
        # static bench (BeamNG demo): the occupant feels NO inertial load, so subtracting the model load of the
        # (virtual) a_y would raise the effective limit by up to 222 N/g -> raw pad force there
        sc = dict(cfg['safety']); self.rig = cfg.get('rig', 'car')
        if self.rig == 'static_bench' and sc.get('mode', 'model') == 'model':
            sc['mode'] = 'raw'; log('rig static_bench: interlock on the raw pad force (no inertial-load subtraction)')
        self.ilock = Interlock(sc, cfg['console_N_per_g'])
        pr = cfg['profile']; self.profile = [pr['mass_kg'], pr['shoulder_width_m'], pr['sitting_torso_m']]
        self.cam_body = bool(pr.get('from_camera', False))   # pose process may send shoulder_width_m / sitting_torso_m
        self.A = getattr(actor, 'A', 2)
        ctl = cfg.get('control', {}); self.mode = ctl.get('mode', 'rl'); self.auto_fallback = ctl.get('fallback', True)
        self.rule = cfg.get('_rule'); self.fallbacks = 0; self.pred_fallbacks = 0
        if self.mode == 'rule' and self.rule is None: raise RuntimeError('control.mode rule needs rule parameters')
        self.base = None; self.first = True; self.a_prev = self._a_rest()
        self.state = 'INIT'; self.n = 0; self.overruns = 0; self.stale_ticks = 0; self.row = None
        self.g_governor = self.g_filter = None; self.g_snapshot = None; self.body_scale = None
        gcfg = dict(cfg.get('g_residual_control') or {})
        enabled = gcfg.pop('enabled', False)
        if type(enabled) is not bool: raise ValueError('g_residual_control.enabled must be boolean')
        if enabled:
            from .g_residual_control import GResidualGovernor
            if not self.v3 or self.A != 2 or self.th_max != 20 or cfg.get('_training_prior') is None:
                raise ValueError('G residual control requires verified v3 yaw-only model, theta20 and training prior')
            from .body_scale import session_scale
            scale, self.body_scale = session_scale(cfg)       # v9: frozen per session from the sitter's camera neutral
            self.g_governor = GResidualGovernor(gcfg, body_scale=scale)
            if self.body_scale['enabled']:
                log(f"body scale {scale:.3f} (shoulder ratio {self.body_scale['width_ratio']:.3f}, "
                    f"torso ratio {self.body_scale['torso_ratio']:.3f} vs the reference user)")
            if self.g_governor.config.full_g != 1 or self.g_governor.config.full_yaw_deg != 20:
                raise ValueError('This experiment requires the reviewed 1g / 20-degree endpoint')
            self.g_filter = OutputFilter(cfg['filter_tau_s'], self.dt)
            self.g_filter.y = np.zeros(1, np.float32)
        elif (cfg.get('body_scale') or {}).get('enabled'):
            raise ValueError('body_scale scales the G base curve: enable g_residual_control too')

    def _g_action(self, a_raw, history, measured_g, predicted_g, speed_mps):
        """One signed-yaw filter; its actual policy action feeds the next observation.

        The bounded correction is the clipped actor's effective angle difference
        from its own training prior, not the unclipped neural-network residual.
        This is a hybrid policy, not an unchanged/retrained RL actor.
        """
        prior_action = self.cfg['_training_prior'](history)
        raw_pose = self.obs.action_to_targets(a_raw, self.th_max, self.pmax)
        prior_pose = self.obs.action_to_targets(prior_action, self.th_max, self.pmax)
        raw_lr = raw_pose[:2] if self.mode == 'rl' else prior_pose[:2]
        requested, snapshot = self.g_governor.step(measured_g, predicted_g, speed_mps,
                                                  raw_lr, prior_pose[:2], self.dt)
        target = requested[1] - requested[0]
        if snapshot['speed_gate']:
            self.g_filter.y = np.zeros(1, np.float32)
        filtered = float(self.g_filter([target])[0])
        if abs(filtered-target) < .001:
            filtered = target; self.g_filter.y[0] = target
        left, right = max(-filtered, 0.), max(filtered, 0.)
        # Exact preimage of one-sided yaw, with sum <= theta_max. Filtering a
        # signed scalar avoids closing both wings during a direction reversal.
        action = np.array([2*(left+right)/self.th_max-1, (left-right)/self.th_max], np.float32)
        snapshot.update(filtered_yaw_lr_deg=[left, right], actor_yaw_lr_deg=list(raw_pose[:2]),
                        training_prior_yaw_lr_deg=list(prior_pose[:2]))
        self.g_snapshot = snapshot
        return action

    def _record_g_tick(self, now, rep, sent, pressure_valid):
        journal = getattr(self.link, 'pressure_log', None)
        if self.g_snapshot is None or journal is None: return
        s = self.g_snapshot; side = s['selected_side']
        pair = lambda value: {'left':float(value[0]), 'right':float(value[1])}
        sided = lambda v: pair((v if side == 'left' else 0., v if side == 'right' else 0.))
        guards = []
        if s['speed_gate']: guards.append('speed_gate')
        if self.ilock.flag: guards.append('pressure_interlock')
        if not pressure_valid: guards.append('not_all_pads_above_5_counts')
        filtered = s['filtered_yaw_lr_deg']
        if max(abs(sent[0]-filtered[0]), abs(sent[2]-filtered[1])) > .002:
            guards.append('downstream_limit_or_release')
        p = self.link.pressure
        journal.record('g_control_tick', host_monotonic_ns=int(now*1e9),
            monotonic_s=now, measured_g=s['measured_g'], predicted_g=s['predicted_g'],
            base_yaw_deg=sided(s['effective_base_yaw_deg']),
            rl_delta_yaw_deg=sided(s['residual_applied_deg']),
            requested_yaw_deg=pair(s['requested_yaw_lr_deg']), filtered_yaw_deg=pair(filtered),
            final_sent_yaw_deg=pair((sent[0],sent[2])), board_command_yaw_deg=pair((rep[0],rep[2])),
            final_sent_semantics='queued_after_limits_not_wire_ack',
            pressure_counts={name:p.get(ch) for name,ch in [('UR','A1_C1'),('UL','A1_C3'),('DR','A1_C0'),('DL','A1_C2')]},
            guards=guards, status=self.state, source=self.cfg.get('_display_source','simulation'),
            policy='g_residual_rl' if self.mode == 'rl' else 'g_base_without_rl',
            direction_source=s['direction_source'], pressure_valid=pressure_valid,
            governor=s)

    def _a_rest(self):
        a = np.zeros(self.A, np.float32); a[0] = -1.0; return a

    # -------------------------------------------------- session start
    def zero_pressure(self, window_s=None, wait=None):
        """Rest baseline (bolsters open, occupant seated, car still). Noisy baselines are rejected
        by default; an explicit diagnostic mode records variability without rejecting it.
        Only distinct, fresh pressure snapshots count; their acquisition timestamps
        must cover the full window, so polling a cached sweep cannot qualify the baseline."""
        P = self.cfg['pressure']; window_s = P['zero_window_s'] if window_s is None else float(window_s)
        variability_mode = P.get('baseline_variability_mode', 'reject')
        if variability_mode not in ('reject', 'warn'):
            raise ValueError('pressure baseline_variability_mode must be reject or warn')
        stale_s = float(P['stale_s'])
        if not (np.isfinite(window_s) and window_s > 0 and np.isfinite(stale_s) and stale_s > 0):
            raise ValueError('pressure baseline window and stale limit must be positive and finite')
        names = sorted({n for v in self.ch.values() for n in (v if isinstance(v, list) else [v])})
        acc = {n: [] for n in names}; t0 = self.clock()
        first_stamp = last_stamp = None
        self.base = None
        while True:
            (wait or time.sleep)(0.02); self.link.poll()
            now = self.clock(); stamp = self.link.pressure_at
            if self.link.fault:
                raise RuntimeError(f'pressure baseline interrupted: {self.link.fault}')
            if not np.isfinite(stamp) or stamp > now + 1e-9:
                raise RuntimeError('pressure baseline rejected: invalid acquisition timestamp')
            if last_stamp is not None and stamp < last_stamp:
                raise RuntimeError('pressure baseline rejected: acquisition timestamp regressed')
            # A handshake-era snapshot is not part of this neutral calibration. Give the scanner
            # one freshness interval to supply the first complete snapshot, never an unbounded wait.
            if first_stamp is None and stamp < t0:
                if now - t0 >= stale_s:
                    raise RuntimeError('pressure baseline rejected: no fresh pressure snapshot')
                continue
            if now - stamp > stale_s:
                raise RuntimeError('pressure baseline rejected: stale pressure snapshot')
            missing = [n for n in names if n not in self.link.pressure or not np.isfinite(self.link.pressure[n])]
            if missing:
                raise RuntimeError(f'pressure baseline rejected: missing/invalid channels {missing}')
            if last_stamp is None or stamp > last_stamp:
                if first_stamp is None: first_stamp = stamp
                last_stamp = stamp
                for n in names: acc[n].append(self.link.pressure[n])
                if last_stamp - first_stamp >= window_s - 1e-9: break
            if now - t0 > window_s + stale_s:
                raise RuntimeError('pressure baseline rejected: insufficient distinct pressure coverage')
        stats = {n: (len(v), float(np.std(v)) if v else None) for n, v in acc.items()}
        noisy = {n: s for n, s in stats.items() if s[1] is not None and s[1] > P['zero_max_std_counts']}
        bad = {n: s for n, s in stats.items() if s[0] < 10 or (variability_mode == 'reject' and n in noisy)}
        if bad: raise RuntimeError(f'pressure baseline rejected (n, std): {bad}')
        if noisy:
            self.log(f'pressure baseline variability warning (n, std): {noisy}; explicitly requested warn mode')
        journal = getattr(self.link, 'pressure_log', None)
        if journal is not None:
            journal.record('pressure_baseline_quality', host_monotonic_ns=round(self.clock()*1e9), variability_mode=variability_mode,
                           diagnostic_std_counts=P['zero_max_std_counts'], channel_statistics=stats,
                           noisy_channels=list(noisy), acquisition_valid=True)
        self.base = {n: float(np.mean(v)) for n, v in acc.items()}
        self.log('pressure baseline: ' + ', '.join(f'{n}={v:.0f}' for n, v in self.base.items()))
        # where would the interlock fire? (one pad carrying the whole wing force, no inertial load)
        S = self.cfg['safety']
        for k in (k for w in self.wing.values() for k in w):
            ch, b = self.ch[k], self.base[self.ch[k]]
            if self.fsr:
                inv = lambda F: next((c for c in range(int(b), 4096) if self.fsr(c) - self.fsr(b) >= F), None)
                soft, hard = inv(S['fsr_force_soft_N']), inv(S['fsr_force_hard_N'])
            else:
                soft, hard = b + S['fsr_force_soft_N'] * self.cpn[k], b + S['fsr_force_hard_N'] * self.cpn[k]
            warn = '' if (hard is not None and hard <= 4095) else '  <- cannot trip on this pad'
            self.log(f'interlock reach {k} ({ch}): rest {b:.0f}, soft at {soft}, hard at {hard} counts{warn}')
        return self.base

    # -------------------------------------------------- one control tick
    def _pressure(self, now):
        p = self.link.pressure; fresh = now - self.link.pressure_at < self.cfg['pressure']['stale_s']
        def get(name):
            names = name if isinstance(name, list) else [name]
            v = [p[n] - self.base[n] for n in names if n in p]
            return float(np.mean(v)) if (v and fresh) else None
        vals = {k: get(v) for k, v in self.ch.items()}
        raw = [p.get(self.ch[k]) for w in self.wing.values() for k in w]
        valid = fresh and all(r is not None and r > 5 for r in raw)
        return vals, valid

    def _wing_force(self, pv):
        """pad force above rest, summed over the wing's pads [N] (None if a pad has no data)"""
        out = []
        for s in ('L', 'R'):
            v = []
            for k in self.wing[s]:
                if pv[k] is None: continue
                if self.fsr:
                    b = self.base[self.ch[k]]; v.append(self.fsr(b + pv[k]) - self.fsr(b))
                else:
                    v.append(pv[k] / self.cpn[k])
            out.append(sum(v) if len(v) == len(self.wing[s]) else None)
        return out

    def tick(self):
        now = self.clock(); c = self.cfg
        self.link.poll()
        if self.link.fault:
            if self.state != 'FAULT': self.log(f'STM32 fault: {self.link.fault} -> stopped (operator re-arm)')
            self.state = 'FAULT'; return None
        veh, vage = self.vehicle.latest()
        if veh is None or vage > c['vehicle']['stale_s']:
            # no vehicle state: open the bolsters (yaw 0 = open, pitch 0) and restart the history when it returns
            self.stale_ticks += 1; self.first = True; self.filt.reset(); self.a_prev = self._a_rest()
            if self.g_governor is not None:
                self.g_governor.reset(); self.g_filter.y = np.zeros(1, np.float32); self.g_snapshot = None
            p0 = (0.0, 0.0) if self.v3 else self.pitch
            self.link.send_targets(0.0, p0[0], 0.0, p0[1]); self.state = 'STALE'
            return None
        self.state = 'RUN'
        pv, p_ok = self._pressure(now)
        ps, page = self.pose.latest()
        pose_ok = ps is not None and page < c['pose']['stale_s'] and ps.get('neutral_valid', True)
        if pose_ok and self.cam_body:                       # body build from the camera (scaled RTMPose spans), slow EMA
            for j, k in ((1, 'shoulder_width_m'), (2, 'sitting_torso_m')):
                if ps.get(k): self.profile[j] += 0.02 * (float(ps[k]) - self.profile[j])
        lean = c['pose']['lean_sign'] * ps['torso_lean_deg'] if pose_ok else 0.0
        shift = c['pose']['lean_sign'] * ps['lean_score'] * self.profile[1] * 100 if pose_ok else 0.0
        ay_p, ax_p = self.pred(veh)
        if not (np.all(np.isfinite(ay_p)) and np.all(np.isfinite(ax_p))):   # predictor failure: current a_y held
            self.pred_fallbacks += 1; ay_p, ax_p = PR.HoldPredictor()(veh)
        rep = self.link.reported() or (0.0, 0.0, 0.0, 0.0)
        z = lambda k: pv[k] if pv.get(k) is not None else 0.0
        rest = lambda k: self.base.get(self.ch[k], 0.0) if isinstance(self.ch.get(k), str) else 0.0
        d = dict(ay_pred_g=ay_p, ax_pred_g=ax_p, ay_g=veh['ay_mps2'] / G, ax_g=veh['ax_mps2'] / G,
                 yaw_rate_radps=veh['yaw_rate_radps'], speed_mps=veh['speed_mps'], steer_rad=veh.get('steer_input_rad', veh['steer_rad']),
                 side_R_counts=z('side_R'), side_L_counts=z('side_L'),                                  # v2 contract
                 side_lo_R_counts=z('side_lo_R'), side_lo_L_counts=z('side_lo_L'),                      # v3 contract
                 side_hi_R_counts=z('side_hi_R'), side_hi_L_counts=z('side_hi_L'),
                 side_rest_counts=[rest('side_lo_R'), rest('side_lo_L'), rest('side_hi_R'), rest('side_hi_L')],
                 back_R_counts=z('back_R'), back_L_counts=z('back_L'), cush_R_counts=z('cush_R'), cush_L_counts=z('cush_L'),
                 torso_lean_deg=lean, shoulder_shift_cm=shift,
                 yaw_reported_L_deg=rep[0], yaw_reported_R_deg=rep[2], pitch_reported_L_deg=rep[1], pitch_reported_R_deg=rep[3],
                 prev_action=self.a_prev,
                 mass_kg=self.profile[0], shoulder_width_m=self.profile[1], sitting_torso_m=self.profile[2],
                 pose_age_s=min(page, 1.0) if pose_ok else 1.0, pressure_valid=p_ok, safety_flag=self.ilock.flag)
        o = self.obs.build_obs(d, self.th_max)
        x = self.hist.reset(o) if self.first else self.hist.push(o); self.first = False
        a_raw = None
        if self.mode == 'rl':
            try:
                a_raw = self.actor(x)
            except Exception as e:                                   # backend failure (device / library)
                self.log(f'actor failed: {type(e).__name__}: {e}')
            if a_raw is None or not np.all(np.isfinite(a_raw)):
                self.fallbacks += 1
                if not (self.auto_fallback and self.rule is not None):
                    raise RuntimeError('actor output invalid and no fallback rule')
                self.mode = 'rule'; self.log('-> FALLBACK to the tuned rule for the rest of the session (latched)')
        if self.mode == 'rule': a_raw = self.rule(x)
        if self.g_governor is not None:
            a = self._g_action(a_raw, x, veh['ay_mps2']/G,
                               ay_p[self.cfg['_training_lead_idx']], veh['speed_mps'])
        else:
            a = self.filt(a_raw)
        if self.v3:
            yL, yR, pL, pR = self.obs.action_to_targets(a, self.th_max, self.pmax)       # same mapping as training
        else:
            cc = (a[0] + 1) / 2 * self.th_max / 2; dd = a[1] * self.th_max / 2
            (yL, yR), (pL, pR) = np.clip([cc + dd, cc - dd], 0, self.th_max), self.pitch
        force = self._wing_force(pv); sat = c['safety'].get('pad_saturation_counts')
        if sat:                                                          # unit-free guard: a saturated pad
            for j, s_ in enumerate(('L', 'R')):
                if any(self.link.pressure.get(self.ch[k], 0) >= sat for k in self.wing[s_]): force[j] = float('inf')
        cmd = self.ilock(now, (yL, yR), (rep[0], rep[2]), force, veh['ay_mps2'] / G)
        if not p_ok:                                                          # no interlock without the pads
            cmd = [min(v, c['pressure']['no_pressure_yaw_cap_deg']) for v in cmd]
        sent = self.link.send_targets(cmd[0], pL, cmd[1], pR)
        self._record_g_tick(now, rep, sent, p_ok)
        if hasattr(self.link, 'authorize'): self.link.authorize()      # OASIS_FALLBACK v2: lease only after a healthy tick
        self.a_prev = np.asarray(a, np.float32)
        self.row = (now, o, a_raw, a, (cmd[0], cmd[1], pL, pR), sent, rep, self.ilock.flag, self.ilock.force)
        return sent

    # -------------------------------------------------- loop
    def run(self, duration_s=None, writer=None, sleep=time.sleep, hold_on_exit=True):
        t0 = self.clock(); k = 0; worst = 0.0
        try:
            while duration_s is None or self.clock() - t0 < duration_s:
                k += 1; due = t0 + k * self.dt
                ts = self.clock(); self.tick(); te = self.clock()
                busy = te - ts; worst = max(worst, busy); self.n += 1
                if writer and self.row: writer(self.row, busy, ts - (due - self.dt))
                if self.state == 'FAULT': break
                slack = due - self.clock()
                if slack < 0: self.overruns += 1
                elif slack > 0.01 and k % 20 == 0: gc.collect(0)              # GC only in slack
                if slack > 0: sleep(slack)
        finally:
            if hold_on_exit: self.link.hold()
        return dict(ticks=self.n, overruns=self.overruns, stale_ticks=self.stale_ticks, worst_tick_ms=1e3 * worst)


class CsvLog:
    def __init__(self, path, od):
        self.f = open(path, 'w', newline=''); self.w = csv.writer(self.f)
        self.w.writerow(['t', 'tick_ms', 'late_ms'] + [f'o{i}' for i in range(od)] +
                        ['a_raw_c', 'a_raw_d', 'a_raw_cp', 'a_raw_dp', 'a_c', 'a_d', 'a_cp', 'a_dp',
                         'cmd_yL', 'cmd_yR', 'cmd_pL', 'cmd_pR', 'sent_yL', 'sent_pL', 'sent_yR', 'sent_pR',
                         'rep_yL', 'rep_pL', 'rep_yR', 'rep_pR', 'safety_flag', 'F_L', 'F_R'])
    def __call__(self, row, busy, late):
        now, o, a_raw, a, cmd, sent, rep, flag, force = row
        pad = lambda v: list(v) + [float('nan')] * (4 - len(v))
        self.w.writerow([f'{now:.4f}', f'{1e3 * busy:.3f}', f'{1e3 * late:.3f}'] + [f'{v:.5g}' for v in o] +
                        [f'{v:.4f}' for v in (*pad(a_raw), *pad(a), *cmd, *sent, *rep, flag, *force)])
    def close(self): self.f.close()


# ------------------------------------------------------------------ assembly
def load_config(path):
    import yaml
    with open(path, encoding='utf-8') as handle:
        cfg = yaml.safe_load(handle)
    base = os.path.dirname(os.path.abspath(path))
    rel = lambda p: p if (p is None or os.path.isabs(p)) else os.path.normpath(os.path.join(base, os.path.expanduser(p)))
    cfg['model_dir'] = rel(cfg['model_dir'])
    if cfg.get('fb_root'): cfg['fb_root'] = rel(cfg['fb_root'])
    for k in ('c_lib', 'hef', 'check_set'): cfg['actor'][k] = rel(cfg['actor'].get(k))
    cfg['vehicle']['replay'] = rel(cfg['vehicle'].get('replay')); cfg['log_dir'] = rel(os.path.expanduser(cfg['log_dir']))
    pc = cfg['pressure'].get('pad_calib')
    cfg['_pad_calib'] = rel(pc) if pc and os.path.exists(rel(pc)) else None
    rj = ((cfg.get('control') or {}).get('rule') or {}).get('from_json')
    if rj: cfg['control']['rule']['from_json'] = rel(rj)
    fc = cfg['pressure'].get('fsr_curve')
    cfg['_fsr_curve'] = rel(fc) if fc and os.path.exists(rel(fc)) else None
    return cfg


def pad_gains(cfg, man, log=print):
    """counts per N for every pad channel: measured (pad_calib.json) where available, otherwise the value
    the simulator assumed for this model (manifest sim_counts_per_N)."""
    g = {ch: man['sim_counts_per_N'] for ch in {v for v in cfg['pressure']['channels'].values() if isinstance(v, str)}}
    if cfg.get('_pad_calib'):
        meas = json.load(open(cfg['_pad_calib']))['controller_counts_per_N']; g.update(meas)
        log(f'pad gains: measured {sorted(meas)} from {cfg["_pad_calib"]}, model {man["sim_counts_per_N"]:.1f} for the rest')
    else:
        log(f'pad gains: model value {man["sim_counts_per_N"]:.1f} counts/N (no pad_calib.json yet)')
    for k, v in man.get('channel_map', {}).items():
        if cfg['pressure']['channels'].get(k) not in (None, v):
            log(f'WARNING: config maps {k} -> {cfg["pressure"]["channels"][k]}, the model was trained with {v}')
    return g


def make_predictor(cfg, vehicle, man=None):
    p = dict(cfg['predictor']); kind = p['kind']
    mp = (man or {}).get('predictor')
    if kind == 'model':                                   # the preview the model was trained with (manifest, v6+)
        if not mp: raise RuntimeError('predictor kind model: the manifest has no predictor (pre-v6 model)')
        p = dict(mp, **{k: v for k, v in p.items() if k != 'kind'}); kind = p['kind']
    elif mp and mp['kind'] != kind:
        print(f'WARNING: predictor {kind}, the model was trained with {mp["kind"]} ({mp})')
    if kind == 'sequence' and isinstance(vehicle, SRC.ReplayVehicle):
        return PR.SequencePredictor(vehicle.t, vehicle.cols['ax'], vehicle.cols['ay'])
    if kind == 'kinematic':
        return PR.KinematicPredictor(**{k: v for k, v in p.items() if k != 'kind'})
    if kind == 'wheel':                                   # BeamNG: raw wheel input preview (the model's training predictor)
        return PR.WheelPreviewPredictor(**{k: v for k, v in p.items() if k not in ('kind', 'src')})
    if kind == 'beamng':                                  # v7: learned 1 s preview (wheel + road ahead + state), npz in the model dir
        npz = p.get('npz'); npz = os.path.join(cfg['model_dir'], npz) if npz and not os.path.isabs(npz) else npz
        if not (npz and os.path.exists(npz)): raise RuntimeError(f'predictor beamng: weights {npz} missing')
        return PR.BeamNGPredictor(npz, **{k: v for k, v in p.items() if k not in ('kind', 'src', 'npz')})
    return PR.HoldPredictor()


def prepare_policy(cfg, log=print):
    """Model, observation contract, actor backend and fallback rule -- everything except the I/O.
    Raises if the model package is unusable (the app then decides about the cold fallback)."""
    man, obs = load_model(cfg['model_dir']); cfg['_K'] = man['K']
    for k in ('cush_R', 'cush_L'):                       # seat-pad grouping is model-specific (v4 fixed the L/R mix)
        if k in man.get('channel_map', {}) and cfg['pressure']['channels'].get(k) != man['channel_map'][k]:
            log(f'{k}: using the model grouping {man["channel_map"][k]} (config had {cfg["pressure"]["channels"].get(k)})')
            cfg['pressure']['channels'][k] = man['channel_map'][k]
    cfg['pressure']['counts_per_N'] = pad_gains(cfg, man, log)
    acfg = dict(cfg['actor'], npz=os.path.join(cfg['model_dir'], 'actor.npz'))
    rc = (cfg.get('control') or {}).get('rule') or {}
    if rc.get('from_json'):                              # e.g. models/v6/rule.json (best of the robust grid)
        rc = dict(json.load(open(rc['from_json']))['best'], **{k: v for k, v in rc.items() if k != 'from_json'})
    rp = dict(gain=man['ff_gain'], lead_idx=man['ff_idx'], both=man.get('ff_both', 0.0), deadband_g=man.get('ff_deadband', 0.0),
              pitch=man.get('pitch_prior', 0.0))
    cfg['_training_prior'] = RuleActor(man['OD'], man['action_dim'], **rp)
    cfg['_training_lead_idx'] = int(man['ff_idx'])
    rp.update({k: rc[k] for k in rp if k in rc})
    cfg['_rule'] = RuleActor(man['OD'], man['action_dim'], **rp)
    try:
        actor, _ = make_actor(acfg, log=log)
        log(f'actor: {actor.name} backend, model {man["source"]} ({man["source_sha256"][:12]}), obs {man["obs_version"]}')
    except Exception as e:                               # no valid backend: the rule runs the session (if allowed)
        if not (cfg.get('control') or {}).get('fallback', True): raise
        log(f'actor unavailable ({type(e).__name__}: {e}) -> control.mode rule'); actor = cfg['_rule']
        cfg.setdefault('control', {})['mode'] = 'rule'
    log(f'fallback rule: {rp} (control.mode {(cfg.get("control") or {}).get("mode", "rl")})')
    return man, obs, actor


def make_controller(cfg, man, obs, actor, link, vehicle, pose, clock, log=print):
    predictor = make_predictor(cfg, vehicle, man)
    if (cfg.get('wheel') or {}).get('priority_prediction', False):
        predictor = PR.MozaPriorityPredictor(predictor)
    return Controller(cfg, link, actor, obs, predictor, vehicle, pose, clock, log)


def build(cfg, mock=False, log=print):
    """Legacy assembly: OASIS_CONTROL v6 firmware (Stm32Link) or its MockStm32. OASIS_FALLBACK v2: oasis_rt.app."""
    man, obs, actor = prepare_policy(cfg, log)
    if mock:
        here = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(here, '..', 'tests'))
        from mock_stm32 import MockClock, MockStm32
        clock = MockClock(); dev = MockStm32(clock, crank_rate=cfg['link'].get('mock_crank_rate_dps', 15.0),
                                             trk=cfg['link']['send_mode'] == 'track')
        sleep = dev.advance
    else:
        clock = time.monotonic; dev = SerialTransport(cfg['link']['port'], cfg['link']['baud']); sleep = time.sleep
    link = Stm32Link(dev, cfg['link'], clock)
    v = cfg['vehicle']
    vehicle = SRC.ReplayVehicle(v['replay'], clock, loop=v.get('loop', False)) if (mock or v['source'] == 'replay') \
        else SRC.UdpJsonSource(v['udp_port'], clock)
    pz = cfg['pose']
    pose = SRC.UdpJsonSource(pz['udp_port'], clock) if (pz['source'] == 'udp' and not mock) else SRC.NoPose()
    ctl = make_controller(cfg, man, obs, actor, link, vehicle, pose, clock, log)
    return ctl, link, dev, sleep


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--config', default=os.path.join(os.path.dirname(__file__), '..', 'config.yaml'))
    ap.add_argument('--mock', action='store_true', help='MockStm32 + replayed drive, simulated clock')
    ap.add_argument('--duration', type=float)
    a = ap.parse_args(argv)
    cfg = load_config(a.config); os.makedirs(cfg['log_dir'], exist_ok=True)
    ctl, link, dev, sleep = build(cfg, a.mock)
    caps = link.connect(); print('firmware:', caps)
    ctl.zero_pressure(wait=sleep)
    if not a.mock: realtime_setup(cfg.get('realtime'), print)
    link.arm()
    stamp = time.strftime('%Y%m%d_%H%M%S'); logp = os.path.join(cfg['log_dir'], f'oasis_rt_{stamp}.csv')
    w = CsvLog(logp, ctl.obs.OBS_DIM)
    try:
        res = ctl.run(a.duration, w, sleep)
    finally:
        link.close(); w.close()
    res.update(sent=link.sent, rejected=link.rejected, projected=link.projected, log=logp)
    print(json.dumps(res, indent=1))


if __name__ == '__main__':
    main()
