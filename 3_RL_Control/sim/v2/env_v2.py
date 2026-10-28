"""O.A.S.I.S L3 environment v2 -- calibrated to the real-car data (passenger on the
mat) and the bench session 1 data (servo bolsters, one subject).

Differences from v0.2 (sim/bolster_env.py):
  * bolster = hobby servo per side: command latency, firmware ramp 15 deg/s, servo lag,
    backlash, compliance under load; no current sensing -> no motor-torque observation
  * contact geometry from calib_v2: effective pad travel (lever arm ~1.5 cm) re-solved per
    episode so every sample keeps the measured bolster/inertia pressure ratio x authority_scale
  * pressure = side contact sensors (counts per N) + cushion/back load-distribution readout
    from the body state (structural readout fitted on the car), FSR noise/hysteresis/pad lag
  * pose = frontal camera torso lean + shoulder shift (lateral lean is observable there)
  * body build profile (mass, shoulder width, sitting torso) drives the physics and is
    observed as a noisy estimate (policy can condition on it)
  * optional door/armrest support on the outboard side (passenger right / driver left)
  * interlock on the side-FSR force estimate: hold above soft, release above hard if closed
Same API as BolsterEnv (reset/step/privileged_vec/obs_dim/PRIV_DIM).
"""
import os, sys, json
import numpy as np

H = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(H, '..'))
from human_model import HumanLateral, HumanParams  # noqa: E402
from sensors import DelayLine, SampledSignal, AccelPredictor  # noqa: E402
from bolster_env import Randomizer, load_cfg  # noqa: E402
import scenario as scn  # noqa: E402
from obs_layout import build_obs, OBS_DIM  # noqa: E402  (shared observation contract)

G = 9.81
CFG = os.path.join(H, 'dr_v2.yaml')
CAL = json.load(open(os.environ.get('OASIS_CALIB') or os.path.join(H, 'calib_v2.json')))   # pipeline: versioned calib
L1_SEQ = os.environ.get('OASIS_L1_SEQ') or os.path.join(H, 'l1_refined_seq.csv')   # make_l1_seq.py


class Servo:
    """Command -> latency -> firmware ramp (reported angle) -> servo lag -> backlash -> compliance."""
    def __init__(self, rng, S, dt):
        self.dt = dt; self.delay = DelayLine(rng.uniform(*S['servo_cmd_latency_s']), dt)
        self.rate = S['servo_rate_dps'] * dt; self.tau = rng.uniform(*S['servo_tau_s'])
        self.bl = rng.uniform(*S['servo_backlash_deg']) / 2; self.comp = rng.uniform(*S['servo_compliance_deg_per_Nm'])
        self.target = 0.0; self.reported = 0.0; self.shaft = 0.0; self.out = 0.0
    def __call__(self, cmd_deg, torque_Nm):
        c = self.delay(cmd_deg)
        if c is not None: self.target = c
        self.reported += np.clip(self.target - self.reported, -self.rate, self.rate)        # firmware state
        self.shaft += (self.reported - self.shaft) * min(1.0, self.dt / self.tau)
        self.out = min(max(self.out, self.shaft - self.bl), self.shaft + self.bl)
        return max(0.0, self.out - self.comp * max(torque_Nm, 0.0))                          # physical angle [deg]


class FirmwareServo(Servo):
    """Actuator as the real STM32 firmware moves it (firmware_model): SET restarts a quintic path
    from the current pose, path time from the C4_R8 crank-rate limit. mode 'fw_event': the Pi sends
    SET only when the target moved >= deadband (and keep-alive); 'fw_event_hb': same, with the proposed
    firmware fix that a same-target SET only refreshes the heartbeat; 'fw_stream': every control tick;
    'fw_track': PROPOSED firmware tracking mode (rate/accel limited, no restart)."""
    def __init__(self, rng, S, dt, mode):
        super().__init__(rng, S, dt)
        from firmware_model import Side
        self.side, self.mode = Side(), mode; self.crank_rate = S.get('crank_rate_dps', 15.0)
        self.now = 0.0; self.last_tx = -1e9; self.last_sent = None; self.ctl_dt = 0.05
        self.deadband = S.get('fw_deadband_deg', 0.5); self.keepalive = 1.0
    def __call__(self, cmd_deg, torque_Nm):
        c = self.delay(cmd_deg)
        if c is not None: self.target = float(np.clip(c, 0, 20))
        self.now += self.dt
        if self.mode == 'fw_track':
            vmax = 0.8 * self.crank_rate / 2.3                           # structure deg/s (crank/yaw ~2.3)
            pose = self.side.track([self.target, 0.0], self.dt, vmax=vmax, amax=8 * vmax)[0]
        else:
            due = self.now - self.last_tx >= self.ctl_dt - 1e-9
            send = due and (self.mode == 'fw_stream' or self.last_sent is None or
                            abs(self.target - self.last_sent) >= self.deadband or self.now - self.last_tx >= self.keepalive)
            same = self.last_sent is not None and abs(self.target - self.last_sent) < 1e-6
            if send and self.mode == 'fw_event_hb' and same:
                self.last_tx = self.now                      # FW fix: same-target SET = heartbeat, no path restart
            elif send and self.side.set(self.now, [self.target, 0.0], self.crank_rate):
                self.last_tx, self.last_sent = self.now, self.target
            pose = self.side.update(self.now)[0]
        self.reported = float(pose)
        self.shaft += (self.reported - self.shaft) * min(1.0, self.dt / self.tau)
        self.out = min(max(self.out, self.shaft - self.bl), self.shaft + self.bl)
        return max(0.0, self.out - self.comp * max(torque_Nm, 0.0))


class SideFSR:
    """Contact force -> counts: pad low-pass, play hysteresis, gain, noise, dead channel."""
    def __init__(self, rng, S, F_rest):
        self.rng = rng; self.g = S['side_counts_per_N'] * rng.uniform(*S['side_gain_scale'])
        self.base = rng.uniform(*S['side_base_counts']); self.noise = rng.uniform(*S['noise_counts'])
        self.h = rng.uniform(*S['hyst_frac']) * 15.0; self.tau = rng.uniform(*S['pad_tau_s'])
        self.dead = rng.random() < S['p_dead']; self.F_rest = F_rest; self.f = F_rest; self.play = F_rest
    def __call__(self, F, dt):
        self.f += (F - self.f) * min(1.0, dt / self.tau)
        self.play = min(max(self.play, self.f - self.h), self.f + self.h)
        if self.dead: return float(self.rng.integers(0, 6))
        c = self.base + self.g * (0.5 * (self.f + self.play) - self.F_rest) + self.rng.normal(0, self.noise)
        return float(np.clip(c, 0, 4095))


class BolsterEnvV2:
    PRIV_DIM = 27
    cmd_in_deg = True
    OBS_DIM = OBS_DIM

    def __init__(self, level=0, seed=None, cfg_path=CFG, scenario_kind=None, sequence=None, overlays=None, policy_off=False):
        self.cfg = load_cfg(cfg_path, overlays)
        self.level, self.rng = level, np.random.default_rng(seed)
        self.kind_override, self.sequence, self.policy_off = scenario_kind, sequence, policy_off
        s = self.cfg['sim']; self.dt, self.dtc = s['dt_physics'], s['dt_control']
        self.sub = int(round(self.dtc / self.dt)); self.T = s['episode_s']; self.nh = 10
        if sequence is not None:
            self.T = float(scn.load_sequence(sequence, self.dt)['t'][-1]) - 2.5
        self.seq_files = scn.library()
        self.obs_dim, self.action_dim = self.OBS_DIM, 2
        self.th_max = float(self.cfg['seat_bolster']['theta_max_deg'])

    # ------------------------------------------------------------------ setup
    def _hparams(self, hp_, pr, sb, door_k, door_side):
        h_com = 0.55 * pr['sitting_torso_m']
        return HumanParams(mass=pr['mass_kg'], torso_frac=0.55, h_com=h_com, fn_torso=hp_['fn_torso_hz'],
                           zeta_torso=hp_['zeta_torso'], passive_frac=hp_['passive_frac'], reflex_delay=hp_['reflex_delay_s'],
                           fn_pelvis=hp_['fn_pelvis_hz'], zeta_pelvis=hp_['zeta_pelvis'], mu_cushion=hp_['mu_cushion'],
                           brace_k=hp_['brace_k_Nm_rad'], ff_gain=hp_['ff_gain'], ff_lead=hp_['ff_lead_s'],
                           lean_bias=np.deg2rad(hp_['lean_bias_deg']), relax_gain=hp_['relax_gain'],
                           h_bolster=sb['h_bolster_m'], k_contact=sb['contact_k'], n_contact=1.0, c_contact=sb['c_contact'],
                           door_k=door_k, door_side=door_side)

    def _static(self, hp, preload, arm, ay_g=0.0, thR=0.0, T=3.0):
        b = HumanLateral(hp, self.dt); gap0 = -(preload / hp.k_contact)
        for i in range(int(T / self.dt)):
            r = min(1.0, i * self.dt / 0.5)
            o = b.step(ay_g * G * r, ay_g * G * r, gap0, gap0 - arm * np.sin(np.deg2rad(thR * r)))
        return o['F_R'] - o['F_L']

    def _preload(self, sb, pr):
        """body build -> preload (wider torso closer to the pads)"""
        half_w = 0.85 * pr['shoulder_width_m'] / 2
        return sb['side_preload_N'] + sb['contact_k'] * max(0.0, half_w - self.cfg['profile']['chair_half_gap_m'])

    def _solve_arm(self, hp, preload, authority):
        """Pad travel per degree such that tightening/inertia matches the data ratio x authority."""
        from dataclasses import replace
        hp0 = replace(hp, door_k=0.0)
        d0 = self._static(hp0, preload, 0.17)
        console = (self._static(hp0, preload, 0.17, ay_g=-0.2) - d0) / -0.2
        tight = (self._static(hp0, preload, 0.17, thR=self.th_max) - d0) / self.th_max
        ratio = CAL['target_ratio_g_per_deg'] * authority
        return float(np.clip(0.17 * ratio / (tight / console), 0.003, 0.17)), console

    def reset(self, seed=None):
        if seed is not None: self.rng = np.random.default_rng(seed)
        r = self.rng
        from bolster_env import BolsterEnv
        for _ in range(50):
            R = Randomizer(self.cfg, self.level, r)
            sp, vp, pr, sb = R.section('scenario'), R.section('vehicle'), R.section('profile'), R.section('seat_bolster')
            hp_ = {k: R.draw(v) for k, v in self.cfg['human'].items() if k not in ('driver', 'passenger', 'door')}
            arch = hp_.pop('archetype'); hp_.update({k: R.draw(v) for k, v in self.cfg['human'][arch].items()})
            door = r.random() < self.cfg['human']['door']['p_present']
            # door stiffness scales with the sampled contact stiffness (calibrated ratio 6.6 x a spread)
            door_k = (CAL['door_k_over_contact_k'] * sb['contact_k'] * float(np.exp(r.normal(0, 0.35)))) if door else 0.0
            door_side = -1 if arch == 'passenger' else +1
            hp = self._hparams(hp_, pr, sb, door_k, door_side)
            if BolsterEnv._plausible(hp, self.dt): break
        mix = self.cfg['scenario']['mix']
        kind = self.kind_override or r.choice(list(mix), p=np.array(list(mix.values())) / sum(mix.values()))
        if self.sequence is not None:
            kind = 'sequence'; self.ex = scn.load_sequence(self.sequence, self.dt)
        elif kind == 'beamng' and self.seq_files:
            self.ex = scn.seq_replay(r, sp, self.T + 2, self.dt, self.seq_files)
        elif kind in ('l1_replay', 'beamng'):          # refined real drive, time/amplitude scaled window
            src = scn.load_sequence(L1_SEQ, self.dt); ts = sp['l1_time_scale']
            n = int((self.T + 2) / self.dt); t = np.arange(n) * self.dt
            t0 = r.uniform(0, src['t'][-1] - (self.T + 2) * ts - 1)
            self.ex = {k: np.interp(t0 + t * ts, src['t'], src[k]) for k in ('ax', 'ay', 'yaw', 'v', 'sw')}
            self.ex['t'] = t; self.ex['ay'] *= sp['l1_amp_scale']; self.ex['yaw'] *= sp['l1_amp_scale']
        else:
            self.ex = scn.synth(kind, r, sp, vp, self.T + 2, self.dt)
        preload = self._preload(sb, pr)
        self.arm, self.console_N_per_g = self._solve_arm(hp, preload, sb['authority_scale'])
        self.preload = preload
        # counts per newton anchored to the car: the NOMINAL passenger at this contact stiffness gives
        # the measured 7234 counts/g; this episode's occupant then differs by its own physics
        tab = np.array(CAL['console_N_per_g_vs_k_nominal'])
        cn = float(np.exp(np.interp(np.log(sb['contact_k']), np.log(tab[:, 0]), np.log(tab[:, 1]))))
        self.G_side = CAL['targets']['car_console'] / cn
        self.params = dict(kind=kind, scenario=sp, vehicle=vp, profile=pr, archetype=arch, human=hp_, seat=sb,
                           door=dict(present=door, k=door_k, side=door_side), arm_eff_m=self.arm, preload_N=preload)
        self.hp = hp; self.body = HumanLateral(hp, self.dt)
        self._setup_sensors(sb, pr)
        return self._obs(), self._info()

    def _setup_sensors(self, sb, pr):
        r, S = self.rng, self.cfg['sensors']; SB = self.cfg['seat_bolster']
        mode = SB.get('actuator_mode', 'ramp')
        self.servo = {s: (Servo(r, SB, self.dt) if mode == 'ramp' else FirmwareServo(r, SB, self.dt, mode)) for s in ('L', 'R')}
        self.gap0 = -(self.preload / self.hp.k_contact)
        self.th = np.zeros(2); self.cmd = np.zeros(2); self.a_prev = np.zeros(2)
        self.t = 0.0; self.trip = 0.0; self.hold = 0.0; self.hold_until = np.full(2, -1.0); self.hold_th = np.zeros(2)
        self.fsr = {}; self.out = None; self.ro_state0 = (0.0, self.hp.lean_bias)
        for k in ('imu_vals',):
            if hasattr(self, k): delattr(self, k)
        self.ro = {ch: CAL['readout'][ch] for ch in ('a0_c1', 'a0_c2', 'a0_c3', 'a2_c1', 'a2_c2')}
        self.ro_noise = r.uniform(*S['noise_counts']); self.ro_scale = r.uniform(0.7, 1.4)
        self.pose_period = 1 / r.uniform(*S['pose_rate_hz']); self.pose_delay = DelayLine(r.uniform(*S['pose_latency_s']), self.dt)
        self.pose_gain = r.uniform(*S['pose_lean_gain']); self.pose_noise = r.uniform(*S['pose_lean_noise_deg'])
        self.pose_next, self.pose, self.pose_age = 0.0, np.zeros(2), 1.0
        imu_rate = r.choice(S['imu_rate_hz']); self.mis = np.deg2rad(r.normal(0, S['imu_misalign_deg']))
        self.imu = {k: SampledSignal(r, imu_rate, r.uniform(*S['imu_latency_s']), r.uniform(*S['imu_ay_noise_mps2']),
                                     r.normal(0, S['imu_bias_mps2']), 0, self.dt) for k in ('ax', 'ay')}
        self.imu['wz'] = SampledSignal(r, imu_rate, r.uniform(*S['imu_latency_s']), 0.002, r.normal(0, 0.003), 0, self.dt)
        obr, obl = r.uniform(*S['obd_rate_hz']), r.uniform(*S['obd_latency_s'])
        self.obd_v = SampledSignal(r, obr, obl, 0, 0, S['speed_quant_kph'] / 3.6, self.dt)
        self.obd_sw = SampledSignal(r, obr, obl, 0, 0, np.deg2rad(0.1), self.dt)
        self.pred = AccelPredictor(r, S, self.nh, 0.1); self.pred_ax = AccelPredictor(r, S, self.nh, 0.1)
        est = self.cfg['profile']['estimate_noise_frac']
        self.profile_est = np.array([pr['mass_kg'], pr['shoulder_width_m'], pr['sitting_torso_m']]) * np.exp(r.normal(0, est, 3))
        self._advance(np.zeros(2), warmup=True)
        self.F_rest = np.array([self.out['F_L'], self.out['F_R']])
        S = dict(S, side_counts_per_N=self.G_side)
        self.fsr = {'L': SideFSR(r, S, self.F_rest[0]), 'R': SideFSR(r, S, self.F_rest[1])}
        self.counts = {'L': self.fsr['L'](self.F_rest[0], self.dt), 'R': self.fsr['R'](self.F_rest[1], self.dt)}
        self.base = {k: self.counts[k] for k in self.counts}
        self.ro_state0 = (self.body.y, self.body.phi); self.ro_last = {ch: 0.0 for ch in self.ro}
        self.com0 = self.body.com_lateral()
        # counterfactual twin: same occupant and inputs, bolsters held open. Sim-only; used for the
        # reward (variance reduction: the uncontrollable inertial sway cancels) and for evaluation
        import copy
        self.twin = copy.deepcopy(self.body); self.twin_gap = self.gap0
        self._sense(0)                                          # first sensor frame at t = 0

    # ------------------------------------------------------------------ physics
    def _ay(self, t): return float(np.interp(t, self.ex['t'], self.ex['ay']))
    def _ax(self, t): return float(np.interp(t, self.ex['t'], self.ex['ax']))

    def _advance(self, cmd, warmup=False):
        n = int(2.0 / self.dt) if warmup else self.sub
        S = self.cfg['safety']
        for _ in range(n):
            i = min(int(self.t / self.dt), len(self.ex['t']) - 1)
            ay = 0.0 if warmup else self.ex['ay'][i]
            ayf = 0.0 if warmup else self._ay(self.t + self.hp.ff_lead)
            F = (self.out['F_L'], self.out['F_R']) if self.out else (0.0, 0.0)
            c = cmd.copy()
            for j in range(2):                                   # interlock hold: no further closing
                if self.t < self.hold_until[j]: c[j] = min(c[j], self.hold_th[j])
            thL = self.servo['L'](c[0], F[0] * self.arm); thR = self.servo['R'](c[1], F[1] * self.arm)   # pad force x lever
            gL = self.gap0 - self.arm * np.sin(np.deg2rad(thL)); gR = self.gap0 - self.arm * np.sin(np.deg2rad(thR))
            dgL = (gL - (self.gap0 - self.arm * np.sin(np.deg2rad(self.th[0])))) / self.dt
            dgR = (gR - (self.gap0 - self.arm * np.sin(np.deg2rad(self.th[1])))) / self.dt
            self.th = np.array([thL, thR])
            self.out = self.body.step(ay, ayf, gL, gR, dgL, dgR)
            if not warmup and hasattr(self, 'twin'):
                self.twin_out = self.twin.step(ay, ayf, self.twin_gap, self.twin_gap)
            if not warmup:
                self._sense(i)
                # interlock on what the rig can measure: side-FSR force estimate (servo -> no current)
                ay_meas = self.imu_vals['ay'][0] / G                    # what the MCU knows from the IMU [g]
                for j, s in enumerate(('L', 'R')):
                    Fh = (self.counts[s] - self.base[s]) / self.G_side     # firmware uses the nominal calibration
                    if S.get('mode', 'model') == 'model':
                        # subtract the occupant's own inertial load on this side (a_y < 0 loads the left,
                        # a_y > 0 the right; calibrated console gain), so only the bolster's squeeze trips it
                        load = -ay_meas if s == 'L' else ay_meas
                        Fh -= CAL['sim_console_N_per_g'] * max(load, 0.0)
                    if Fh > S['fsr_force_hard_N'] and self.th[j] > S['arm_deg']:
                        self.trip = 1.0; self.cmd[j] = 0.0; self.servo[s].target = 0.0
                        self.hold_until[j] = self.t + S['hold_s']; self.hold_th[j] = 0.0
                    elif Fh > S['fsr_force_soft_N'] and self.th[j] > S['arm_deg']:
                        self.hold = 1.0
                        self.hold_th[j] = min(self.hold_th[j], self.servo[s].reported) if self.t < self.hold_until[j] else self.servo[s].reported
                        self.hold_until[j] = self.t + S['hold_s']
                self.t += self.dt

    def _sense(self, i):
        o, t, ex = self.out, self.t, self.ex
        if self.fsr and int(t / self.dt) % 2 == 0:                # 100 Hz mat
            self.counts = {'L': self.fsr['L'](o['F_L'], 0.01), 'R': self.fsr['R'](o['F_R'], 0.01)}
            dy, dphi = 100 * (self.body.y - self.ro_state0[0]), np.rad2deg(self.body.phi - self.ro_state0[1])
            for ch, rr in self.ro.items():
                self.ro_last[ch] = self.ro_scale * (rr['weights'][0] * dy + rr['weights'][1] * dphi) + self.rng.normal(0, self.ro_noise)
        d = self.pose_delay((t, self.body.phi, self.body.y + self.hp.h_com * 1.3 * np.sin(self.body.phi)))
        self.pose_age += self.dt
        if d is not None and d[0] >= self.pose_next:
            self.pose_next = d[0] + self.pose_period
            self.pose = np.array([self.pose_gain * np.rad2deg(d[1] - self.ro_state0[1]) + self.rng.normal(0, self.pose_noise),
                                  self.pose_gain * 100 * (d[2] - self.ro_state0[0]) + self.rng.normal(0, self.pose_noise)])
            self.pose_age = t - d[0]
        ay_m = ex['ay'][i] * np.cos(self.mis) + ex['ax'][i] * np.sin(self.mis)
        ax_m = ex['ax'][i] * np.cos(self.mis) - ex['ay'][i] * np.sin(self.mis)
        self.imu_vals = {k: self.imu[k](t, v) for k, v in (('ax', ax_m), ('ay', ay_m), ('wz', ex['yaw'][i]))}
        self.v_obd = self.obd_v(t, ex['v'][i]); self.sw_obd = self.obd_sw(t, ex['sw'][i])

    # ------------------------------------------------------------------ observation
    def _obs(self):
        if not hasattr(self, 'imu_vals'):
            self._sense(0)
        iv, pe = self.imu_vals, self.profile_est
        d = dict(ay_pred_g=self.pred(self._ay, self.t) / G, ax_pred_g=self.pred_ax(self._ax, self.t) / G,
                 ay_g=iv['ay'][0] / G, ax_g=iv['ax'][0] / G, yaw_rate_radps=iv['wz'][0], speed_mps=self.v_obd[0],
                 steer_rad=self.sw_obd[0],
                 side_R_counts=self.counts['R'] - self.base['R'], side_L_counts=self.counts['L'] - self.base['L'],
                 back_R_counts=self.ro_last['a2_c2'], back_L_counts=self.ro_last['a2_c1'],
                 cush_R_counts=self.ro_last['a0_c3'], cush_L_counts=0.5 * (self.ro_last['a0_c1'] + self.ro_last['a0_c2']),
                 torso_lean_deg=self.pose[0], shoulder_shift_cm=self.pose[1],
                 yaw_reported_L_deg=self.servo['L'].reported, yaw_reported_R_deg=self.servo['R'].reported,
                 prev_action=self.a_prev, mass_kg=pe[0], shoulder_width_m=pe[1], sitting_torso_m=pe[2],
                 pose_age_s=self.pose_age, pressure_valid=self.counts['R'] > 5 and self.counts['L'] > 5,
                 safety_flag=max(self.trip, 0.5 * self.hold))
        return build_obs(d, self.th_max)           # shared with the Pi runtime (obs_layout.py)

    def privileged_vec(self):
        b, p = self.body, self.params
        fut = [self._ay(self.t + 0.1 * k) / G for k in range(10)]
        v = [(b.com_lateral() - self.com0) / 0.02, b.vy / 0.2, b.phi / 0.1, b.om / 0.5, b.y / 0.02,
             self.out['F_L'] / 50, self.out['F_R'] / 50, self.th[0] / self.th_max, self.th[1] / self.th_max] + fut + [
             p['profile']['mass_kg'] / 75 - 1, np.log(p['seat']['contact_k'] / 3000), np.log(p['seat']['authority_scale']),
             float(p['door']['present']), float(p['archetype'] == 'driver'), self.arm / 0.015, self.trip,
             (self.twin.com_lateral() - self.com0) / 0.02]
        return np.asarray(v, np.float32)

    def _info(self):
        b = self.body
        return dict(privileged=dict(y_com=b.com_lateral() - self.com0, y_com_twin=self.twin.com_lateral() - self.com0, phi=b.phi, y_pelvis=b.y, F_L=self.out['F_L'],
                                    F_R=self.out['F_R'], theta=np.deg2rad(self.th), ay=self._ay(self.t)),
                    params=self.params, trip=self.trip, hold=self.hold)

    # ------------------------------------------------------------------ step
    def step(self, action):
        a = np.clip(np.asarray(action, float), -1, 1)
        if self.policy_off: a = np.array([-1.0, 0.0])
        c = (a[0] + 1) / 2 * self.th_max / 2; d = a[1] * self.th_max / 2
        self.cmd = np.clip([c + d, c - d], 0, self.th_max)        # [left, right] target yaw, deg
        self.trip = 0.0; self.hold = 0.0
        self._advance(self.cmd)
        W = self.cfg['reward']
        y = (self.body.com_lateral() - self.com0) / W['com_scale_m']
        yt = (self.twin.com_lateral() - self.com0) / W['com_scale_m']
        com_term = (y ** 2 - yt ** 2) if W.get('counterfactual', True) else y ** 2
        # comfort: only the squeeze the bolsters ADD on top of what the occupant's own inertia produces
        dF = max(self.out['F_L'] - self.twin_out['F_L'], self.out['F_R'] - self.twin_out['F_R'], 0.0) / W.get('force_scale_N', 100.0)
        r = -(W['w_com'] * com_term + W['w_du'] * np.sum((a - self.a_prev) ** 2) + W['w_theta'] * np.sum((self.th / self.th_max) ** 2)
              + W['w_force'] * dF ** 2 + W['w_trip'] * self.trip)
        self.a_prev = a
        return self._obs(), float(r), False, bool(self.t >= self.T), self._info()


if __name__ == '__main__':
    import time
    for kind in ('l1_replay', 'urban_turn'):
        e = BolsterEnvV2(level=0, seed=3, scenario_kind=kind); t0 = time.time(); o, info = e.reset(); n = 0; R = 0
        while True:
            o, rr, te, tr, info = e.step([-1, np.clip(-4 * o[6], -1, 1)]); R += rr; n += 1
            if tr: break
        p = info['params']
        print(kind, 'steps', n, 'ret', round(R, 2), 'obs', o.shape, 'arm', round(100 * p['arm_eff_m'], 2), 'cm',
              'preload', round(p['preload_N'], 1), 'door', p['door']['present'], p['archetype'], 'wall', round(time.time() - t0, 2), 's')
