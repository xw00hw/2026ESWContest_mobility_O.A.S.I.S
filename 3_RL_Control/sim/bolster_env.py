"""O.A.S.I.S L3 environment with domain randomisation (gymnasium-style API,
numpy only so it runs without gymnasium installed).

    env = BolsterEnv(level=0, seed=0)
    obs, info = env.reset()
    obs, r, terminated, truncated, info = env.step(np.array([c, d]))

action  : [c, d] in [-1, 1]  (common clamp, differential support; review doc 1.1)
          theta_L = sat(c' + d'),  theta_R = sat(c' - d'),  c' = (c+1)/2 * theta_max/2
obs     : ~58-d vector (review doc 6.1), built from the *sensor models* only
info    : 'privileged' = true state + episode parameters (for an asymmetric critic)
reward  : camera/model CoM only -- pressure never enters the reward (review doc 5-D)
"""
import os
import numpy as np
import yaml

from human_model import HumanLateral, HumanParams
from sensors import FSRChannel, PoseCamera, SampledSignal, BolsterActuator, AccelPredictor
import scenario as scn

G = 9.81
CFG_PATH = os.path.join(os.path.dirname(__file__), 'dr_config.yaml')


def deep_merge(base, over):
    """Overlay files (e.g. dr_config.l2.yaml written by l2/update_dr.py) replace only
    the keys they name, so measured ranges supersede the L2 placeholders."""
    out = dict(base)
    for k, v in (over or {}).items():
        out[k] = deep_merge(base[k], v) if isinstance(v, dict) and isinstance(base.get(k), dict) and 'dist' not in v else v
    return out


def load_cfg(cfg_path=CFG_PATH, overlays=None):
    cfg = yaml.safe_load(open(cfg_path, encoding='utf-8'))
    env_ov = os.environ.get('OASIS_DR_OVERLAY')
    for ov in list(overlays or []) + ([p for p in env_ov.split(os.pathsep) if p] if env_ov else []):
        cfg = deep_merge(cfg, yaml.safe_load(open(ov, encoding='utf-8')))
    return cfg


# ----------------------------------------------------------------- randomiser
class Randomizer:
    """Samples one episode's parameters. `level` widens ranges from nominal to the
    full range (ADR-style curriculum); level >= spec.level -> full range."""
    def __init__(self, cfg, level, rng):
        self.cfg, self.level, self.rng = cfg, level, rng

    def draw(self, spec):
        r = self.rng
        if not isinstance(spec, dict):
            return spec
        lv = spec.get('level', 0)
        f = 1.0 if self.level >= lv else max(0.0, self.level / max(lv, 1))
        d = spec['dist']
        if d == 'choice':
            return r.choice(spec['values'], p=spec.get('p'))
        if d in ('uniform', 'loguniform'):
            lo, hi = spec['range']
            nom = spec.get('nominal', (lo + hi) / 2)
            nlo, nhi = (nom if isinstance(nom, list) else (nom, nom))
            lo, hi = nlo + f * (lo - nlo), nhi + f * (hi - nhi)
            if d == 'loguniform':
                return float(np.exp(r.uniform(np.log(lo), np.log(hi))))
            return float(r.uniform(lo, hi))
        if d == 'normal':
            x = r.normal(spec['mean'], spec['sd'] * max(f, 0.3))
        elif d == 'lognormal':
            x = spec['median'] * np.exp(r.normal(0, spec['logsd'] * max(f, 0.3)))
        else:
            raise ValueError(d)
        if 'clip' in spec:
            x = np.clip(x, *spec['clip'])
        return float(x)

    def section(self, name, skip=()):
        sec = self.cfg[name]
        out = {k: self.draw(v) for k, v in sec.items() if k not in skip and k not in ('mix', '_joint')}
        # jointly measured parameters (e.g. contact preload/k/n from one L2 fit) are drawn as a
        # row, so correlated parameters never form a combination that was not observed
        for J in sec.get('_joint', []):
            row = np.asarray(J['samples'][self.rng.integers(len(J['samples']))], float)
            row = row * np.exp(self.rng.normal(0, J.get('jitter_logsd', 0.0), len(row)))
            out.update(dict(zip(J['keys'], row.tolist())))
        return out


# ----------------------------------------------------------------------- env
class BolsterEnv:
    def __init__(self, level=0, seed=None, cfg_path=CFG_PATH, scenario_kind=None, policy_off=False, overlays=None,
                 sequence=None):
        """sequence: path to a recorded drive (BeamNG / converted L1). The episode then
        replays the WHOLE file from t = 0 (used to roll a policy out on a given drive)."""
        self.cfg = load_cfg(cfg_path, overlays)
        self.sequence = sequence
        self.seq_files = scn.library()
        self.level, self.rng = level, np.random.default_rng(seed)
        self.kind_override, self.policy_off = scenario_kind, policy_off
        s = self.cfg['sim']
        self.dt, self.dtc = s['dt_physics'], s['dt_control']
        self.sub = int(round(self.dtc / self.dt))
        self.T = s['episode_s']
        if sequence is not None:
            self.T = float(scn.load_sequence(sequence, self.dt)['t'][-1]) - 2.5
        self.nh = int(round(s['horizon_pred_s'] / 0.1))
        self.obs_dim = len(self._zero_obs())
        self.action_dim = 2

    # ---- episode setup
    def _sample(self):
        R = Randomizer(self.cfg, self.level, self.rng)
        sp = R.section('scenario'); vp = R.section('vehicle')
        mix = self.cfg['scenario']['mix']
        kind = self.kind_override or self.rng.choice(list(mix), p=np.array(list(mix.values())) / sum(mix.values()))
        hp_ = {k: R.draw(v) for k, v in self.cfg['human'].items() if k not in ('driver', 'passenger')}
        arch = hp_.pop('archetype')
        hp_.update({k: R.draw(v) for k, v in self.cfg['human'][arch].items()})
        sb = R.section('seat_bolster')
        sens = dict(self.cfg['sensors'])
        sens['fsr_noise_counts'] = sens['fsr_noise_counts'] * self.rng.uniform(*sens['fsr_noise_scale'])
        for k in ('act_delay_s', 'act_tau_s', 'act_rate_dps', 'act_backlash_deg'):
            sens[k] = sb[k]
        sens['theta_max_deg'] = sb['theta_max_deg']
        return kind, sp, vp, arch, hp_, sb, sens

    @staticmethod
    def _plausible(hp, dt):
        """Reject occupant parameter sets that are unstable with the reflex delay
        or deflect implausibly (humans do not fall over at 0.3 g): step a_y = 0.3 g
        with the seat wings retracted far away."""
        b = HumanLateral(hp, dt)
        ys = []
        for i in range(int(4.0 / dt)):
            o = b.step(0.3 * G, 0.3 * G, 1.0, 1.0)
            ys.append(o['phi'])
            if not np.isfinite(o['phi']) or abs(o['phi']) > 0.6:
                return False
        ys = np.array(ys[-int(1.0 / dt):])
        return ys.std() < 0.01            # settled (no sustained oscillation)

    def reset(self, seed=None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        for _ in range(50):
            kind, sp, vp, arch, hp_, sb, P = self._sample()
            if self._plausible(self._hparams(hp_, sb), self.dt):
                break
        self.n_rejected = _
        self.params = dict(kind=kind, scenario=sp, vehicle=vp, archetype=arch, human=hp_, seat=sb)
        if self.sequence is not None:
            kind = 'sequence'; self.params['kind'] = kind
            self.ex = scn.load_sequence(self.sequence, self.dt)
        elif kind == 'beamng' and self.seq_files:
            self.ex = scn.seq_replay(self.rng, sp, self.T + 2, self.dt, self.seq_files)
        elif kind in ('l1_replay', 'beamng'):        # no BeamNG library yet -> fall back to L1
            self.ex = scn.l1_replay(self.rng, sp, self.T + 2, self.dt)
        else:
            self.ex = scn.synth(kind, self.rng, sp, vp, self.T + 2, self.dt)
        self.hp = self._hparams(hp_, sb)
        self.body = HumanLateral(self.hp, self.dt)
        self._setup_sensors(P, sb)
        return self._obs(), self._info()

    @staticmethod
    def _hparams(hp_, sb):
        return HumanParams(mass=hp_['mass_kg'], torso_frac=hp_['torso_frac'], h_com=hp_['h_com_m'],
                              fn_torso=hp_['fn_torso_hz'], zeta_torso=hp_['zeta_torso'],
                              passive_frac=hp_['passive_frac'], reflex_delay=hp_['reflex_delay_s'],
                              fn_pelvis=hp_['fn_pelvis_hz'], zeta_pelvis=hp_['zeta_pelvis'], mu_cushion=hp_['mu_cushion'],
                              brace_k=hp_['brace_k_Nm_rad'], ff_gain=hp_['ff_gain'],
                              ff_lead=hp_['ff_lead_s'], lean_bias=np.deg2rad(hp_['lean_bias_deg']),
                              asym=hp_['asym_ratio'], relax_gain=hp_.get('relax_gain', 0.0), h_bolster=sb['h_bolster_m'],
                              k_contact=sb['k_contact'], n_contact=sb['n_contact'], c_contact=sb['c_contact'])

    def _setup_sensors(self, P, sb):
        r = self.rng
        # sensors (L1 layout: 4 cushion, 2 live side, 2 back channels used here)
        pre = self.hp.mass * G
        self.fsr = {'side_L': FSRChannel(r, P, 15), 'side_R': FSRChannel(r, P, 15),
                    'seat_LF': FSRChannel(r, P, pre * .15), 'seat_LR': FSRChannel(r, P, pre * .2),
                    'seat_RF': FSRChannel(r, P, pre * .15), 'seat_RR': FSRChannel(r, P, pre * .2),
                    'back_L': FSRChannel(r, P, 60), 'back_R': FSRChannel(r, P, 60)}
        self.fsr_period, self.fsr_next, self.fsr_last = 1 / P['fsr_rate_hz'], 0.0, None
        self.cam = PoseCamera(r, P, self.dt)
        imu_rate = r.choice(P['imu_rate_hz'])
        mis = np.deg2rad(r.normal(0, P['imu_misalign_deg']))
        self.mis = mis
        self.imu = {k: SampledSignal(r, imu_rate, r.uniform(*P['imu_latency_s']), r.uniform(*P['imu_ay_noise_mps2']),
                                     r.normal(0, P['imu_bias_mps2']), 0, self.dt) for k in ('ax', 'ay')}
        self.imu['wz'] = SampledSignal(r, imu_rate, r.uniform(*P['imu_latency_s']), 0.002, r.normal(0, 0.003), 0, self.dt)
        obd_r, obd_l = r.uniform(*P['obd_rate_hz']), r.uniform(*P['obd_latency_s'])
        self.obd_v = SampledSignal(r, obd_r, obd_l, 0.0, 0.0, P['speed_quant_kph'] / 3.6, self.dt)
        self.obd_sw = SampledSignal(r, obd_r, obd_l, 0.0, 0.0, np.deg2rad(0.1), self.dt)
        self.pred = AccelPredictor(r, P, self.nh, 0.1)
        self.pred_ax = AccelPredictor(r, P, self.nh, 0.1)
        self.act = {s: BolsterActuator(r, P, self.dt) for s in ('L', 'R')}
        # motor-side torque sensing (driver current sense): gain error, Coulomb friction, noise
        self.tq_gain = np.exp(r.normal(0, P['motor_torque_gain_logsd']))
        self.tq_fric = r.uniform(*P['motor_friction_Nm'])
        self.tq_noise = r.uniform(*P['motor_torque_noise_Nm'])
        self.tq = np.zeros(2)
        self.cmd = np.zeros(2); self._pose_ref = None; self.com0 = 0.0
        self.th_max = np.deg2rad(P['theta_max_deg'])
        self.gap0 = -(sb['side_preload_N'] / sb['k_contact']) ** (1 / sb['n_contact'])
        self.arm = sb['arm_len_m']
        self.k, self.t = 0, 0.0
        self.a_prev = np.zeros(2); self.th = np.zeros(2); self.th_prev = np.zeros(2)
        self.base = None; self.trip = 0.0; self.out = None; self.F_rest = None
        self.th_prev_phys = np.zeros(2)
        self._v2_init()
        self._advance(np.zeros(2), warmup=True)
        self.F_rest = (self.out['F_L'], self.out['F_R'])
        self.base = self._fsr_vec()          # session-start calibration (review doc 5.4)
        # v2 interlock rest reference: motor-side force at rest, and a full FSR history window
        self._v2_F_rest = np.array([self.tq[j] / (self.arm * np.cos(self.th[j])) for j in range(2)])
        self._v2_F = self._v2_F_rest.copy()
        for _ in range(5): self._v2_fsr_hist.append(self.fsr_last.copy())
        self.com0 = self.body.com_lateral()  # occupant's own neutral: habitual lean is not an error

    # ---- physics
    def _ay(self, t):
        return float(np.interp(t, self.ex['t'], self.ex['ay']))

    def _ax(self, t):
        return float(np.interp(t, self.ex['t'], self.ex['ax']))

    def _gap(self, th):
        return self.gap0 - self.arm * np.sin(th)

    def _advance(self, cmd, warmup=False):
        n = int(2.0 / self.dt) if warmup else self.sub
        hp = self.hp
        for _ in range(n):
            i = min(int(self.t / self.dt), len(self.ex['t']) - 1)
            ay = self.ex['ay'][i] if not warmup else 0.0
            ayf = self._ay(self.t + hp.ff_lead) if not warmup else 0.0
            thL, thR = self.act['L'](cmd[0]), self.act['R'](cmd[1])
            gL, gR = self._gap(thL), self._gap(thR)
            dgL = (gL - self._gap(self.th[0])) / self.dt; dgR = (gR - self._gap(self.th[1])) / self.dt
            self.th = np.array([thL, thR])
            o = self.body.step(ay, ayf, gL, gR, dgL, dgR)
            self.out = o
            self._motor_torque(o, thL, thR)
            # firmware safety layer -- only what the MCU can measure: side-FSR rise over the
            # session baseline (fraction of ADC headroom) and, as backup, motor torque. Per side.
            # The FSR cannot tell occupant inertia from bolster squeeze (review doc 5-D), so the
            # FSR path is armed only while that bolster is actually closed (> arm_deg); otherwise
            # the interlock would release support exactly when the occupant leans into a corner.
            if self.base is not None and self.cfg['safety'].get('mode', 'v1') == 'v2':
                self._interlock_v2(self.cfg['safety'], thL, thR)
            elif self.base is not None:
                S = self.cfg['safety']
                arm = np.deg2rad(S['fsr_trip_arm_deg'])
                for j, (s, th) in enumerate((('L', thL), ('R', thR))):
                    b0 = self.base[j]
                    fsr_trip = (th > arm) and (self.fsr_last[j] - b0) > S['side_fsr_trip_frac'] * (4095 - b0)
                    tq_trip = (th > arm) and self.tq[j] > S['motor_torque_trip_Nm']   # backdrive by the occupant is not a squeeze either
                    if fsr_trip or tq_trip:
                        self.trip = 1.0; self.act[s].cmd = 0.0; self.cmd[j] = 0.0
            if not warmup:
                self._sense(o, ay, i)
                self.t += self.dt
            else:
                self._sense(o, 0.0, 0)

    def _interlock_v2(self, S, thL, thR):
        """Force-based interlock (v2). Absolute force from the motor side (kt / friction
        calibrated on the L2 bench, so only a residual calibration error remains); the FSR
        is used only for what it is good at -- detecting a fast rise while the bolster closes
        (review doc 1.3-3, 5-A). Graded: soft limit -> hold (no further closing), hard -> release."""
        for j, (s, th) in enumerate((('L', thL), ('R', thR))):
            vel = th - self._v2_prev_th[j]
            fr = self._v2_fric * np.sign(vel) if abs(vel) > 1e-6 else 0.0
            F_hat = (self.tq[j] - fr) / (self.arm * np.cos(th))
            self._v2_F[j] += 0.2 * (F_hat - self._v2_F[j])           # ~25 ms firmware low-pass at 200 Hz
            rise = self._v2_F[j] - self._v2_F_rest[j]
            fsr_rate = (self.fsr_last[j] - self._v2_fsr_hist[0][j]) / 0.05
            closing = vel > 0
            if rise > S['v2_force_hard_N'] and th > np.deg2rad(S['fsr_trip_arm_deg']):   # release only if closed
                self.trip = 1.0; self.act[s].cmd = 0.0; self.cmd[j] = 0.0
                self._v2_hold_until[j] = self.t + S['v2_hold_s']; self._v2_hold_th[j] = 0.0
            elif rise > S['v2_force_soft_N'] or (closing and fsr_rate > S['v2_fsr_rise_cps']):
                self.hold = 1.0
                # freeze at the current angle (or keep an earlier, lower freeze still in force)
                self._v2_hold_th[j] = min(self._v2_hold_th[j], th) if self.t < self._v2_hold_until[j] else th
                self._v2_hold_until[j] = self.t + S['v2_hold_s']
            if self.t < self._v2_hold_until[j] and self.act[s].cmd > self._v2_hold_th[j]:
                self.act[s].cmd = self._v2_hold_th[j]
        self._v2_prev_th = np.array([thL, thR])
        if self._v2_step % 2 == 0:                                   # FSR history at 100 Hz, 50 ms window
            self._v2_fsr_hist.append(self.fsr_last.copy())
        self._v2_step += 1

    def _v2_init(self):
        from collections import deque
        r = self.rng
        self._v2_fric = self.tq_fric * np.exp(r.normal(0, 0.2))      # bench-calibrated friction, 20 % error
        self._v2_prev_th = np.zeros(2); self._v2_F = np.zeros(2); self._v2_F_rest = np.zeros(2)
        self._v2_hold_until = np.full(2, -1.0); self._v2_hold_th = np.zeros(2); self._v2_step = 0
        self._v2_fsr_hist = deque([np.zeros(2)] * 5, maxlen=5)
        self.hold = 0.0

    def _motor_torque(self, o, thL, thR):
        r = self.rng
        for j, (F, th) in enumerate(((o['F_L'], thL), (o['F_R'], thR))):
            vel = th - self.th_prev_phys[j]
            fr = self.tq_fric * np.sign(vel) if abs(vel) > 1e-6 else 0.0
            self.tq[j] = self.tq_gain * F * self.arm * np.cos(th) + fr + r.normal(0, self.tq_noise)
        self.th_prev_phys = np.array([thL, thR])

    def _sense(self, o, ay, i):
        t = self.t
        if t >= self.fsr_next:
            self.fsr_next = t + self.fsr_period
            # cushion: left/right normal loads split front/rear 43/57
            F = {'side_L': o['F_L'], 'side_R': o['F_R'],
                 'seat_LF': o['N_L'] * .43, 'seat_LR': o['N_L'] * .57,
                 'seat_RF': o['N_R'] * .43, 'seat_RR': o['N_R'] * .57,
                 'back_L': 60 + 0.4 * o['F_L'], 'back_R': 60 + 0.4 * o['F_R']}
            self.fsr_last = np.array([self.fsr[k](F[k], self.fsr_period) for k in self.fsr])
        b = self.body
        y_sh = b.y + self.hp.h_com * 1.3 * np.sin(b.phi)
        y_hd = b.y + self.hp.h_com * 1.9 * np.sin(b.phi)
        self.pose, self.pose_age = self.cam(t, y_sh, y_hd, b.y)
        ex = self.ex
        ay_m = ex['ay'][i] * np.cos(self.mis) + ex['ax'][i] * np.sin(self.mis)
        ax_m = ex['ax'][i] * np.cos(self.mis) - ex['ay'][i] * np.sin(self.mis)
        self.imu_vals = {k: self.imu[k](t, v) for k, v in (('ax', ax_m), ('ay', ay_m), ('wz', ex['yaw'][i]))}
        self.v_obd = self.obd_v(t, ex['v'][i]); self.sw_obd = self.obd_sw(t, ex['sw'][i])

    def _fsr_vec(self):
        return self.fsr_last.copy()

    # ---- observation (built only from sensor models, same rules as real time)
    def _zero_obs(self):
        return np.zeros(20 + 6 + 10 + 10 + 6 + 6)

    def _obs(self):
        o = self._zero_obs()
        fut = self.pred(self._ay, self.t) / G                                 # 10 x a_y [g]
        o[0:10] = fut
        o[10:20] = self.pred_ax(self._ax, self.t) / G                          # a_x forecast [g]
        iv = self.imu_vals
        o[20:26] = [iv['ay'][0] / G, iv['ax'][0] / G, iv['wz'][0], self.v_obd[0] / 20,
                    self.sw_obd[0] / 3, 0.0]
        if self.pose is not None:
            sh, hd, hip = self.pose
            if self._pose_ref is None: self._pose_ref = np.array([sh, hd, hip])
            # causal slow baseline (tau 20 s) -> neutral-relative shifts
            self._pose_ref = self._pose_ref + self.dtc / 20 * (np.array([sh, hd, hip]) - self._pose_ref)
            d = (np.array([sh, hd, hip]) - self._pose_ref) / 0.01
            o[26:29] = d; o[29] = d[0] - d[2]; o[30] = d[1] - d[0]
        o[31:33] = self.tq / self.cfg['safety']['motor_torque_trip_Nm']     # motor-side torque
        o[33] = self.hold                                                  # v2 interlock holding
        x = (self.fsr_last - self.base) / np.maximum(self.base, 50)
        live = self.fsr_last > 5
        x = np.where(live, x, 0.0)
        o[36:44] = np.clip(x, -3, 3)
        sl, sr = self.fsr_last[0], self.fsr_last[1]
        o[44] = (sr - sl) / (sr + sl + 50)
        cl = self.fsr_last[2] + self.fsr_last[3]; cr = self.fsr_last[4] + self.fsr_last[5]
        o[45] = (cr - cl) / (cr + cl + 50)                                   # cushion CoP proxy
        o[46:48] = self.th / self.th_max; o[48:50] = (self.th - self.th_prev) / self.dtc / 3
        o[50:52] = self.a_prev
        o[52:58] = [min(iv['ay'][1], 1), min(self.pose_age, 1), float(self.pose is not None),
                    min(self.v_obd[1], 2) / 2, float(live[:2].all()), self.trip]
        return o.astype(np.float32)

    PRIV_DIM = 32

    def privileged_vec(self):
        """Ground truth for the critic only (never the actor): occupant state, contact
        forces, true a_y now + next 1 s, and the episode's key parameters."""
        b, h, sb = self.body, self.params['human'], self.params['seat']
        fut = [self._ay(self.t + 0.1 * k) / G for k in range(10)]
        v = [(b.com_lateral() - self.com0) / 0.03, b.vy / 0.2, b.phi / 0.1, b.om / 0.5, b.y / 0.02,
             self.out['F_L'] / 100, self.out['F_R'] / 100, *(self.th / self.th_max)] + fut + [
             (h['mass_kg'] - 75) / 15, np.log(h['fn_torso_hz'] / 1.6), h['zeta_torso'] - 1.9,
             h['reflex_delay_s'] / 0.1, np.log(h['asym_ratio']), h['ff_gain'],
             float(self.params['archetype'] == 'driver'), np.log(sb['k_contact'] / 5000),
             sb['side_preload_N'] / 30, self.act['L'].tau / 0.2, sb['arm_len_m'] / 0.17, self.trip,
             self.out['slip']]
        return np.asarray(v, np.float32)

    def _info(self):
        b = self.body
        return dict(privileged=dict(y_com=b.com_lateral() - self.com0, phi=b.phi, y_pelvis=b.y,
                                    F_L=self.out['F_L'], F_R=self.out['F_R'], theta=self.th.copy(),
                                    ay=self._ay(self.t)), params=self.params)

    # ---- step
    def step(self, action):
        a = np.clip(np.asarray(action, float), -1, 1)
        if self.policy_off:
            a = np.array([-1.0, 0.0])
        c = (a[0] + 1) / 2 * self.th_max / 2
        d = a[1] * self.th_max / 2
        cmd = np.clip([c + d, c - d], 0, self.th_max)
        # rate limiter (firmware) on the command
        rl = np.deg2rad(self.cfg['safety']['rate_limit_dps']) * self.dtc
        self.cmd = np.clip(cmd, self.cmd - rl, self.cmd + rl)
        self.th_prev = self.th.copy(); self.trip = 0.0; self.hold = 0.0
        self._advance(self.cmd)
        W = self.cfg['reward']
        y = (self.body.com_lateral() - self.com0) / W['com_scale_m']
        dF = max(self.out['F_L'] - self.F_rest[0], self.out['F_R'] - self.F_rest[1], 0.0) / 100.0
        r = -(W['w_com'] * y ** 2 + W['w_du'] * np.sum((a - self.a_prev) ** 2)
              + W['w_theta'] * np.sum((self.th / self.th_max) ** 2)
              + W['w_slip'] * self.out['slip'] + W['w_trip'] * self.trip
              + W['w_force'] * dF ** 2)   # sim-only comfort cost on the TRUE squeeze force, not the FSR
        self.a_prev = a
        self.k += 1
        trunc = self.t >= self.T
        return self._obs(), float(r), False, bool(trunc), self._info()


if __name__ == '__main__':
    import time
    env = BolsterEnv(level=0, seed=1)
    t0 = time.time(); obs, info = env.reset(); R = 0; n = 0
    while True:
        obs, r, te, tr, info = env.step(np.zeros(2)); R += r; n += 1
        if te or tr: break
    print('kind', info['params']['kind'], 'archetype', info['params']['archetype'],
          'steps', n, 'return', round(R, 2), 'obs_dim', obs.shape, 'wall', round(time.time() - t0, 2), 's')
