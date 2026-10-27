"""Raspberry Pi main: BeamNG.drive (BeamNG_Bolster_FB collector path) -> RL controller / fallback -> NUCLEO-H723ZG
running OASIS_FALLBACK v2 (BeamNG_Bolster_FB stm32).

  python -m oasis_rt.app --config config.yaml                     hardware session (config.confirmations all true)
  python -m oasis_rt.app --config config.yaml --check-vehicle 20  BeamNG stream check, no serial: rate, gaps, PC age,
                                                                  steering sign, invalid-row reasons
  python -m oasis_rt.app --config config.yaml --sil 60            software-in-the-loop: MockFbBoard + synthetic
                                                                  collector rows on a simulated clock (no devices)

Session (BeamNG_Bolster_FB runner semantics; ESW AGENTS.md safety rules):
  1 policy assembled BEFORE any device opens: control.mode rl | rule | fb_v6
      rl    residual SAC actor + BeamNG 1 s predictor; actor failure -> tuned rule for the rest of the session (latched)
      rule  tuned rule only (the RL prior, same predictor and interlock)
      fb_v6 BeamNG_Bolster_FB V6 vehicle-only controller (experiment_a.controller, unchanged)
      rl/rule model package unusable -> control.cold_fallback (fb_v6) or refuse to start
  2 board handshake (fb_link.FbLink) -> committed STM tuning frozen -> envelope (CAD n tuning n IK)
  3 >= beamng.startup_s of continuously valid fresh BeamNG rows + verified PWM OFF -> one FB_ARM
  4 fresh armed STATE at neutral (pose = target = 0) within 0.5 s
  5 rl/rule: pressure baseline at neutral (2 s; lease renewed only while BeamNG stays valid)
  6 control loop; every input / board / policy fault -> FB_OFF, OFF telemetry verified, session ends, no re-ARM
"""
import argparse, csv, hashlib, json, os, sys, tempfile, time

from . import controller as C, fbstack
from .beamng_rx import RelayReceiver, VehicleInput
from .fb_link import FbLink
from .pose_input import close_pose, make_pose_input, pose_summary, validate_config as validate_pose_config
from .sources import NoPose
from .wheel_input import make_wheel, validate_config as validate_wheel_config

CONFIRM = ('hardware_mapping_checked', 'neutral_start_clear', 'stm_tuning_reviewed', 'steer_sign_checked',
           'trial_settings_reviewed')


def _sha(p):
    with open(p, 'rb') as handle:
        return hashlib.sha256(handle.read()).hexdigest()


# ------------------------------------------------------------------ policies
class RlPolicy:
    """controller.Controller (20 Hz) on FbLink; mode rl or rule (cfg control.mode, may degrade rl -> rule)."""
    def __init__(self, cfg, man, obs, actor, log):
        self.cfg, self.man, self.obs, self.actor, self.log = cfg, man, obs, actor, log
        self.name = (cfg.get('control') or {}).get('mode', 'rl'); self.ctl = None; self.res = {}

    def bind(self, s):
        self.ctl = C.make_controller(self.cfg, self.man, self.obs, self.actor, s.link, s.vin, s.pose, s.clock, self.log)
        th = float(self.cfg['theta_max_deg']); ymax = s.link.env.yaw_max / 1000; rate = s.link.tuning['yaw_rate_mdeg_s'] / 1000
        if ymax < th: self.log(f'NOTE: STM yaw_max {ymax:g} deg < policy range {th:g} deg -> targets clipped (raise '
                               f'g_experiment_a_tune.values.yaw_max_mdeg to {int(th * 1000)} for the full RL range)')
        # the actuator the policy was trained with vs this board (sim: v7, trained on the faster FW-proposal actuator,
        # falls below the rule on OASIS_FALLBACK v2): mismatch -> the tuned rule runs the session (control.actuator_mismatch)
        act = self.man.get('actuator') or {}; why = None
        if act.get('mode') != 'fb_a2':
            why = f'model trained for {act.get("firmware", "the OASIS_CONTROL v6 / FW-proposal actuator")}, board is OASIS_FALLBACK v2'
        elif rate < (act.get('min_tune_rate_dps') or 0):
            why = f'STM yaw_rate {rate:g} deg/s < {act["min_tune_rate_dps"]:g} deg/s the policy was trained with'
        elif act.get('yaw_caps_deg') and ymax < min(act['yaw_caps_deg']):
            why = f'STM yaw_max {ymax:g} deg < {min(act["yaw_caps_deg"]):g} deg the policy was trained with'
        self.actuator_note = why
        if why and self.ctl.mode == 'rl' and (self.cfg.get('control') or {}).get('actuator_mismatch', 'rule') == 'rule':
            self.ctl.mode = 'rule'; self.log(f'NOTE: {why} -> tuned rule for this session (control.actuator_mismatch)')
        elif why:
            self.log(f'NOTE: {why}')

    def start(self, s):
        self.ctl.zero_pressure(wait=s.wait_neutral)

    def run(self, s, duration):
        logp = os.path.join(s.out, 'control.csv'); w = C.CsvLog(logp, self.obs.OBS_DIM)
        try:
            self.res = self.ctl.run(duration, w, sleep=s.wait, hold_on_exit=False)
        finally:
            w.close()
        return logp

    def summary(self):
        c = self.ctl
        return dict(self.res, mode_final=c.mode if c else None, actor_fallbacks=c.fallbacks if c else 0,
                    predictor_fallbacks=c.pred_fallbacks if c else 0, state=c.state if c else None,
                    actuator_note=getattr(self, 'actuator_note', None),
                    predictor=c.pred.summary() if c and hasattr(c.pred,'summary') else None,
                    g_residual_control=c.g_governor.config.to_dict() if c and c.g_governor else None,
                    body_scale=c.body_scale if c else None,
                    g_residual_last=c.g_snapshot if c else None)


class FbV6Policy:
    """BeamNG_Bolster_FB Experiment A V6 controller, unchanged: vehicle-only (no pressure, no pose), CAD-coupled
    targets, its own 2 deg steering advance, neutral passage between sides; STM tuning replaces its six parameters."""
    name = 'fb_v6'

    def __init__(self, cfg, fb, log):
        self.fb, self.log = fb, log
        p = (cfg.get('fb_v6') or {}).get('settings')
        self.settings = fb.settings.Settings.load(p) if p else fb.settings.Settings()
        self.settings.steering_to_ay_sign = int((cfg.get('beamng') or {}).get('steer_sign', 1))
        self.settings.validate(); self.ctl = None; self.n = 0; self.reasons = {}

    def bind(self, s):
        self.settings, self.frozen = self.fb.tuning.apply_firmware_tuning(self.settings, s.link.tuning)
        self.ctl = self.fb.controller.ExperimentAController(self.settings)

    def start(self, s): pass

    def run(self, s, duration):
        logp = os.path.join(s.out, 'control_fb_v6.csv'); f = open(logp, 'w', newline=''); w = csv.writer(f)
        w.writerow(['t', 'ay_mps2', 'steering_deg', 'speed_mps', 'reason', 'target_source',
                    'cmd_LY', 'cmd_LP', 'cmd_RY', 'cmd_RP', 'rep_LY', 'rep_LP', 'rep_RY', 'rep_RP'])
        t0 = nxt = s.clock_ns()
        try:
            while (duration is None or s.clock_ns() - t0 < duration * 1e9) and not s.link.fault:
                now = s.clock_ns(); fr, st = s.vin.frame, s.link.b.state
                d = self.ctl.tick(fr, st, now)
                self.n += 1; self.reasons[d.reason] = self.reasons.get(d.reason, 0) + 1
                if d.emergency_off or not d.permit_lease:
                    s.link.stop('policy:' + d.reason); break
                s.link.set_pose_mdeg(d.pose_mdeg); s.link.authorize()
                w.writerow([f'{now / 1e9:.4f}', fr.ay_mps2, fr.steering_deg, fr.speed_mps, d.reason,
                            d.diag.get('target_source'), *d.pose_mdeg, *(st.pose_mdeg if st else (None,) * 4)])
                nxt += 20_000_000; s.wait(max(0.0, (nxt - s.clock_ns()) / 1e9))
        finally:
            f.close()
        return logp

    def summary(self): return dict(ticks=self.n, reasons=self.reasons)


def make_policy(cfg, fb, log):
    mode = (cfg.get('control') or {}).get('mode', 'rl')
    if mode == 'fb_v6': return FbV6Policy(cfg, fb, log)
    try:
        man, obs, actor = C.prepare_policy(cfg, log)
        return RlPolicy(cfg, man, obs, actor, log)
    except Exception as e:
        cold = (cfg.get('control') or {}).get('cold_fallback')
        if cold != 'fb_v6': raise
        log(f'COLD FALLBACK: model package unusable ({type(e).__name__}: {e}) -> fb_v6 (BeamNG_Bolster_FB V6)')
        return FbV6Policy(cfg, fb, log)


# ------------------------------------------------------------------ session
class Session:
    def __init__(self, cfg, fb, link, vin, policy, clock_ns, sleep, out, log=print, pose=None):
        self.cfg, self.fb, self.link, self.vin, self.policy = cfg, fb, link, vin, policy
        self.clock_ns, self.sleep, self.out, self.log = clock_ns, sleep, out, log
        self.pose = pose if pose is not None else NoPose()
        self.pose_cleanup_error = None
        b = cfg.get('beamng') or {}
        self.startup_ns = int(float(b.get('startup_s', 2.0)) * 1e9)
        self.need_sign = policy.name != 'fb_v6'           # rl / rule read the wheel through the predictor
        self.hold_auth = False; self.phase = 'INIT'; self.skipped = {}

    def clock(self): return self.clock_ns() / 1e9

    def pump(self):
        self.link.poll(); self.vin.poll()
        if hasattr(self.pose, 'poll'): self.pose.poll()
        if self.link.armed and not self.link.fault:
            if self.vin.issue: self.link.stop('vehicle:' + self.vin.issue)
            elif not self.vin.good(): self.link.stop('vehicle_stale')
            elif self.need_sign and self.vin.sign_mon.verdict == 'mismatch':
                self.link.stop(f'steer_sign_mismatch (agreement {self.vin.sign_mon.fraction:.0%}): flip beamng.steer_sign')
            elif self.hold_auth:
                self.link.authorize()
        self.link.service()

    def wait(self, dt):
        end = self.clock_ns() + int(dt * 1e9)
        while True:
            self.pump(); rem = end - self.clock_ns()
            if rem <= 0 or self.link.fault: return
            self.sleep(min(0.002, rem / 1e9))

    def wait_neutral(self, dt):
        """Baseline phase: hold neutral, renew the lease while BeamNG stays valid (pump stops it otherwise)."""
        self.hold_auth = True
        try: self.wait(dt)
        finally: self.hold_auth = False

    def _until(self, cond, timeout_s, what):
        t0 = self.clock_ns()
        while not cond():
            if self.link.fault: raise RuntimeError(self.link.fault)
            if timeout_s is not None and self.clock_ns() - t0 > timeout_s * 1e9: raise TimeoutError(what)
            self.wait(0.005)

    def run(self, duration=None, startup_timeout_s=None):
        result, off_ok, logp = 'startup_error', False, None
        try:
            self.link.start(); self.phase = 'HANDSHAKE'
            self._until(lambda: self.link.ready, 5.0, 'board handshake (OASIS_FALLBACK v2 HELLO/CAD/TUNE/STATE)')
            tun = self.link.freeze()
            self.log(f'board: {self.link.hello.get("firmware")} {self.link.hello.get("version")}, tuning rev {tun["revision"]}: '
                     f'yaw <= {tun["yaw_max_mdeg"] / 1000:g} deg, pitch <= {tun["pitch_max_mdeg"] / 1000:g} deg, '
                     f'{tun["yaw_rate_mdeg_s"] / 1000:g} deg/s')
            self.policy.bind(self)
            self.phase = 'QUALIFY'; good_since = None; t0 = self.clock_ns()
            self.log(f'waiting for {self.startup_ns / 1e9:g} s of valid BeamNG rows + verified PWM OFF ...')
            while True:
                self.wait(0.01)
                if self.link.fault: raise RuntimeError(self.link.fault)
                if self.vin.issue:
                    self.skipped[self.vin.issue] = self.skipped.get(self.vin.issue, 0) + 1; self.vin.reset_issue(); good_since = None
                ok = self.vin.good() and self.link.board_off() and not (self.need_sign and self.vin.sign_mon.verdict == 'mismatch')
                if not ok: good_since = None
                else:
                    good_since = good_since or self.clock_ns()
                    if self.clock_ns() - good_since >= self.startup_ns and self.link.arm(): break
                if startup_timeout_s is not None and self.clock_ns() - t0 > startup_timeout_s * 1e9:
                    raise TimeoutError(f'no qualified BeamNG input within {startup_timeout_s} s ({self.skipped})')
            t_arm = self.clock_ns(); self.vin.reset_issue(); self.phase = 'ARM'
            self._until(lambda: self.link.armed_neutral(t_arm), 0.5, 'fresh neutral armed STATE')
            self.log(f'armed (session {self.link.b.armed_session}), policy {self.policy.name}')
            self.phase = 'BASELINE'; self.policy.start(self)
            if self.link.fault: raise RuntimeError(self.link.fault)
            self.phase = 'CONTROL'; logp = self.policy.run(self, duration)
            result = self.link.fault or 'completed'
        except KeyboardInterrupt:
            result = 'operator_stop'
        except Exception as e:
            result = self.link.fault or f'{type(e).__name__}: {e}'
        finally:
            try:
                off_ok = self.link.close(self.sleep)
            finally:
                try:
                    self.vin.close()
                finally:
                    try:
                        close_pose(self.pose)
                    except Exception as e:
                        self.pose_cleanup_error = f'{type(e).__name__}: {e}'
                        self.log('pose close error: ' + self.pose_cleanup_error)
                        if result in ('completed', 'operator_stop'):
                            result = 'pose_close:' + self.pose_cleanup_error
                    finally:
                        if self.link.pressure_log is not None:
                            try:
                                self.link.pressure_log.close()
                            except Exception as e:
                                self.link.logging_error = self.link.logging_error or f'{type(e).__name__}: {e}'
                                self.link.stop('pressure_logging:' + self.link.logging_error)
                            if self.link.logging_error and result in ('completed', 'operator_stop', 'startup_error'):
                                result = 'pressure_logging:' + self.link.logging_error
            self.log(f'session ended in {self.phase}: {result}; PWM OFF verified: {off_ok}. No automatic re-ARM.')
        return dict(result=result, phase=self.phase, off_verified=off_ok, control_log=logp,
                    logging_error=self.link.logging_error)


# ------------------------------------------------------------------ assembly
def manifest(cfg, fb, session, res, policy):
    md = cfg['model_dir']; mp = os.path.join(md, 'manifest.json')
    l, v = session.link, session.vin
    return dict(schema='oasis-rt-app-v1', result=res, policy=policy.name, policy_summary=policy.summary(),
                wall_time=time.strftime('%Y-%m-%dT%H:%M:%S%z'), config=cfg.get('_config_path'),
                model=json.load(open(mp)).get('source_sha256') if os.path.exists(mp) else None,
                fb_root=fb.root, fb_sha256={n: _sha(os.path.join(fb.root, n)) for n in (
                    'experiment_a/transport.py', 'experiment_a/controller.py', 'experiment_a/cad_limits.py',
                    'experiment_a/vehicle.py', 'experiment_a/pressure_logging.py', 'experiment_a/session_logging.py',
                    'fallback/beamng.py', 'fallback/serial_link.py')},
                firmware=l.hello, stm_tuning=l.tuning, envelope=dict(yaw_max_mdeg=l.env.yaw_max, pitch_max_mdeg=l.env.pitch_max) if l.env else None,
                board=dict(sent=l.sent, projected=l.projected, sweeps=l.sweeps, bad_sweeps=l.bad_sweeps,
                           final_state=str(l.b.state)),
                vehicle=dict(rows=v.n, reasons=dict(v.reasons), steer_sign=v.sign, steer_agreement=v.sign_mon.fraction,
                             steer_evidence_rows=v.sign_mon.n, startup_skipped=session.skipped),
                pressure_role='rl/rule: observation + interlock; fb_v6: recorded only (BeamNG_Bolster_FB ADR-022)',
                pressure_logging=session.link.pressure_log.summary() if session.link.pressure_log is not None else None,
                pose=dict(pose_summary(session.pose), cleanup_error=session.pose_cleanup_error),
                wheel=dict(v.wheel.summary(), cleanup_error=v.wheel_cleanup_error,
                           expected_vehicle_id=v.wheel_vehicle_id) if v.wheel is not None else None,
                physical_effect_verified=False)


def _sil_sleep(clock, link):
    """Pace only virtual time against the real writer, with bounded error handling.

    0.25 simulated seconds generates about 125 pressure records. Let that
    batch drain before simulating another; wall-clock I/O never advances the
    mock board or its lease. Hardware retains its nonblocking journal path.
    """
    last_sync = clock()
    def sleep(dt):
        nonlocal last_sync
        clock.sleep(dt)
        if clock() - last_sync < 250_000_000 or link.fault: return
        last_sync = clock()
        try:
            link.pressure_log.flush(wait=True)  # SIL only; existing bounded writer wait.
        except link.fb.session_logging.SessionLogError:
            # Latch the real failure, then permit virtual OFF/STATE shutdown.
            link.check_logging()
    return sleep


def assemble(cfg, mode, log=print, sil_tune=None, sil_receiver=None, sil_pose=None, sil_wheel=None):
    """mode: 'hw' (serial + UDP relay) or 'sil' (MockFbBoard + synthetic rows on a simulated clock)."""
    cfg['_display_source'] = 'live' if mode == 'hw' else 'simulation'
    fb = fbstack.load(cfg.get('fb_root'))
    validate_pose_config(cfg)                                         # reject unsupported metric body input before devices
    validate_wheel_config(cfg)
    if sil_pose is not None and mode != 'sil': raise ValueError('sil_pose injection is only available in SIL')
    if sil_wheel is not None and mode != 'sil': raise ValueError('sil_wheel injection is only available in SIL')
    b = cfg.get('beamng') or {}
    policy = make_policy(cfg, fb, log)                                  # before any device opens
    from .pressure_log import PressureLogger                           # FB import path selected above
    stamp = time.strftime('%Y%m%d_%H%M%S')
    os.makedirs(cfg['log_dir'], exist_ok=True)
    out = tempfile.mkdtemp(prefix=f'app_{stamp}_{mode}_', dir=cfg['log_dir'])
    journal = PressureLogger(out)                                      # exclusive logs BEFORE UDP/UART opening
    rx = dev = pose = wheel = None
    try:
        journal.check()
        if mode == 'sil':
            here = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(here, '..', 'tests'))
            from mock_fb_board import MockClockNs, MockFbBoard
            from .synthetic import SyntheticReceiver
            clock = MockClockNs(); dev = MockFbBoard(clock, tune=sil_tune)
            rx = sil_receiver(clock) if sil_receiver else SyntheticReceiver(clock)
            clock_ns, sleep, synth = clock, clock.sleep, True
            pose = sil_pose(clock_ns, journal) if sil_pose else make_pose_input(cfg, clock_ns, journal, allow_udp=False)
            wheel = sil_wheel(clock_ns, journal) if sil_wheel else make_wheel(cfg, clock_ns, journal, allow_udp=False)
        else:
            clock_ns, sleep, synth = time.monotonic_ns, time.sleep, bool(b.get('allow_synthetic', False))
            pose = make_pose_input(cfg, clock_ns, journal)
            wheel = make_wheel(cfg, clock_ns, journal)
            listen = b.get('listen', ['0.0.0.0', 64257])
            rx = RelayReceiver(fb, (listen[0], int(listen[1])), b.get('allow_from') or None, clock_ns, b.get('max_pc_age_ms', 50.0))
            dev = fb.serial_link.open_serial(cfg['link']['port'])
        link = FbLink(dev, fb, clock_ns, log, pressure_log=journal)
        if mode == 'sil': sleep = _sil_sleep(clock_ns, link)
        vin = VehicleInput(fb, rx, clock_ns, fresh_ms=float(b.get('fresh_ms', 200.0)), steer_sign=int(b.get('steer_sign', 1)),
                           allow_synthetic=synth, require_steering=policy.name != 'fb_v6', wheel=wheel,
                           wheel_priority=bool((cfg.get('wheel') or {}).get('priority_prediction',False)), journal=journal,
                           wheel_vehicle_id=((cfg.get('wheel') or {}).get('calibration') or {}).get('beamng_vehicle_id'))
        journal.record('session_created', host_monotonic_ns=clock_ns(), wall_time_ns=time.time_ns(), mode=mode)
        return fb, Session(cfg, fb, link, vin, policy, clock_ns, sleep, out, log, pose=pose), dev
    except BaseException:
        if pose is not None:
            try: close_pose(pose)
            except Exception: pass
        for resource in (dev, rx, wheel, journal):
            if resource is not None:
                try: resource.close()
                except Exception: pass
        raise


def run(cfg, mode='hw', duration=None, log=print, **kw):
    startup_timeout_s = kw.pop('startup_timeout_s', 30.0 if mode == 'sil' else None)
    fb, s, dev = assemble(cfg, mode, log, **kw)
    if mode == 'hw': C.realtime_setup(cfg.get('realtime'), log)
    res = s.run(duration, startup_timeout_s=startup_timeout_s)
    man = manifest(cfg, fb, s, res, s.policy)
    with open(os.path.join(s.out, 'manifest.json'), 'x', encoding='utf-8') as handle:
        json.dump(man, handle, indent=1, default=str)
    return man, s, dev


def check_vehicle(cfg, seconds, log=print):
    """BeamNG stream check (no serial): what the session would see."""
    fb = fbstack.load(cfg.get('fb_root')); b = cfg.get('beamng') or {}
    listen = b.get('listen', ['0.0.0.0', 64257])
    rx = RelayReceiver(fb, (listen[0], int(listen[1])), b.get('allow_from') or None, time.monotonic_ns, b.get('max_pc_age_ms', 50.0))
    vin = VehicleInput(fb, rx, time.monotonic_ns, fresh_ms=float(b.get('fresh_ms', 200.0)), steer_sign=int(b.get('steer_sign', 1)),
                       allow_synthetic=bool(b.get('allow_synthetic', False)))
    t0 = time.monotonic(); last = None; gaps = []; issues = {}; nxt = t0 + 2
    while time.monotonic() - t0 < seconds:
        f = vin.poll()
        if vin.issue: issues[vin.issue] = issues.get(vin.issue, 0) + 1; vin.reset_issue()
        if f is not None and (last is None or f.received_monotonic_ns != last):
            if last is not None: gaps.append((f.received_monotonic_ns - last) / 1e6)
            last = f.received_monotonic_ns
        if time.monotonic() > nxt:
            nxt += 2; sm = vin.sign_mon
            log(f'rows {vin.n} ({vin.n / (time.monotonic() - t0):.0f}/s), good {vin.good()}, steer agreement '
                f'{sm.fraction if sm.fraction is None else round(sm.fraction, 2)} over {sm.n} cornering rows, issues {issues}')
        time.sleep(0.002)
    import numpy as np
    g = np.array(gaps) if gaps else np.zeros(1); age = np.array(rx.pc_age_ms) if rx.pc_age_ms else np.zeros(1)
    rep = dict(rows=vin.n, rate_hz=vin.n / seconds, gap_ms_p95=float(np.percentile(g, 95)), gap_ms_max=float(g.max()),
               pc_age_ms_p95=float(np.percentile(age, 95)), relay_gaps=rx.relay_gaps, foreign_datagrams=rx.foreign,
               reasons=dict(vin.reasons), issues=issues, steer_sign=vin.sign, steer_verdict=vin.sign_mon.verdict,
               steer_agreement=vin.sign_mon.fraction, steer_evidence_rows=vin.sign_mon.n)
    vin.close(); return rep


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--config', default=os.path.join(os.path.dirname(__file__), '..', 'config.yaml'))
    ap.add_argument('--duration', type=float, help='seconds of control (default: until fault / Ctrl+C)')
    ap.add_argument('--sil', type=float, metavar='S', help='software-in-the-loop for S simulated seconds')
    ap.add_argument('--check-vehicle', type=float, metavar='S', help='BeamNG stream check for S seconds (no serial)')
    a = ap.parse_args(argv)
    cfg = C.load_config(a.config); cfg['_config_path'] = os.path.abspath(a.config); os.makedirs(cfg['log_dir'], exist_ok=True)
    if a.check_vehicle:
        print(json.dumps(check_vehicle(cfg, a.check_vehicle), indent=1)); return 0
    if a.sil:
        man, s, dev = run(cfg, 'sil', a.sil)
    else:
        if (cfg.get('link') or {}).get('firmware', 'fallback_v2') != 'fallback_v2':
            sys.exit('link.firmware is not fallback_v2: use python -m oasis_rt.controller (legacy OASIS_CONTROL v6)')
        missing = [k for k in CONFIRM if not (cfg.get('confirmations') or {}).get(k)]
        if missing:
            sys.exit('hardware session blocked before opening any device; confirm in config.yaml: ' + ', '.join(missing))
        man, s, dev = run(cfg, 'hw', a.duration)
    print(json.dumps({k: man[k] for k in ('result', 'policy', 'policy_summary', 'board', 'vehicle')}, indent=1, default=str))
    print('log:', s.out)
    return 0 if man['result']['result'] in ('completed', 'operator_stop') else 1


if __name__ == '__main__':
    sys.exit(main())
