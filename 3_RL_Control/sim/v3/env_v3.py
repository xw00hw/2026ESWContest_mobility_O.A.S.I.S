"""O.A.S.I.S L3 environment v3 = v2 (car/bench calibrated, firmware actuator) plus
  * bolster PITCH: action [c, d, cp, dp] -> yaw L/R and pitch L/R (obs_layout_v3.action_to_targets)
  * two pad zones per wing (lower / upper, h_bolster -/+ zone_dh): yaw moves both inward, pitch moves
    one in and the other out; an upper-zone force has the longer lever about the lumbar pivot
  * actuator: one firmware Side per bolster with the 2-D C4_R8 quintic path (crank 30 / 60 deg/s)
  * body shape couples into the contact: shoulder width -> preload and upper/lower share; sitting
    torso -> CoM height (as v2); the policy sees camera-grade estimates (3 %) and the rest pressure
    of all four side pads (absolute baseline = preload / taper cue)
Subclass of BolsterEnvV2: scenarios, randomiser, occupant draw, readout, IMU/OBD/pose sensors and the
counterfactual-twin reward are unchanged. Observation contract: obs_layout_v3.py.
"""
import copy, os, sys
from dataclasses import replace
import numpy as np

H = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(H, '..'), os.path.join(H, '..', 'v2'), os.path.join(H, '..', '..', 'deploy', 'runtime')]
from env_v2 import BolsterEnvV2, SideFSR, CAL, G  # noqa: E402
from human_model import HumanLateral  # noqa: E402
from sensors import DelayLine  # noqa: E402
from firmware_model import Side, cad_pitch_cap, fb_a2_step  # noqa: E402
from obs_layout_v3 import build_obs, action_to_targets, OBS_DIM  # noqa: E402
from oasis_rt.predictor import WheelPreviewPredictor, BeamNGPredictor, H as PRED_H  # noqa: E402  (the runtime's classes)

CFG2 = os.path.join(H, '..', 'v2', 'dr_v2.yaml')
OV3 = os.path.join(H, 'dr_v3.yaml')
ZONES = ('lo', 'hi')
ROOT = os.path.abspath(os.path.join(H, '..', '..'))


class Servo2D:
    """One bolster (yaw, pitch): command latency -> actuator -> servo lag -> backlash / compliance (yaw).
    ramp: structure-rate ramps per axis; fw_event_hb / fw_event / fw_stream / fw_track: the firmware Side
    (2-D quintic path restarted per SET, C4_R8 crank-rate timing) driven like the Pi does."""
    def __init__(self, rng, S, dt, mode, crank_rate, pitch_max):
        self.dt, self.mode, self.crank, self.pmax = dt, mode, crank_rate, pitch_max
        self.delay = DelayLine(rng.uniform(*S['servo_cmd_latency_s']), dt)
        self.rate = S['servo_rate_dps'] * dt; self.tau = rng.uniform(*S['servo_tau_s'])
        self.bl = rng.uniform(*S['servo_backlash_deg']) / 2; self.comp = rng.uniform(*S['servo_compliance_deg_per_Nm'])
        self.deadband = S.get('fw_deadband_deg', 0.5)
        self.side = Side(); self.now = 0.0; self.last_tx = -1e9; self.last_sent = None; self.rejected = 0
        self.target = np.zeros(2); self.reported = np.zeros(2); self.shaft = np.zeros(2); self.out = 0.0; self.defl = 0.0
        self.fb_rate, self.fb_cap, self.fb_t = 20.0, 20.0, 0.0          # fb_a2: STM tune rate / yaw_max (set per episode)

    def __call__(self, cmd, torque_Nm):
        c = self.delay(tuple(cmd))
        if c is not None:
            if self.mode == 'fb_a2':                                     # Pi envelope: STM yaw_max, CAD pitch cap
                y = float(np.clip(c[0], 0, self.fb_cap)); pc = min(self.pmax, cad_pitch_cap(y))
                self.target = np.array([y, np.clip(c[1], -pc, pc)])
            else:
                self.target = np.array([np.clip(c[0], 0, 20), np.clip(c[1], -self.pmax, self.pmax)])
        self.now += self.dt
        if self.mode == 'fb_a2':                                         # OASIS_FALLBACK v2: 20 ms firmware frames
            self.fb_t += self.dt
            while self.fb_t >= 0.02 - 1e-9:
                self.fb_t -= 0.02; self.reported = fb_a2_step(self.reported, self.target, (self.fb_rate, self.fb_rate))
        elif self.mode == 'ramp':
            self.reported = self.reported + np.clip(self.target - self.reported, -self.rate, self.rate)
        elif self.mode == 'fw_track':
            vmax = 0.8 * self.crank / 2.3
            self.reported = self.side.track(self.target, self.dt, vmax=vmax, amax=8 * vmax).copy()
        else:
            due = self.now - self.last_tx >= 0.05 - 1e-9
            moved = self.last_sent is None or np.max(np.abs(self.target - self.last_sent)) >= self.deadband
            keep = self.mode == 'fw_event' and self.now - self.last_tx >= 1.0      # v6 keep-alive SET (restarts path)
            if due and (self.mode == 'fw_stream' or moved or keep):
                if self.side.set(self.now, list(self.target), self.crank): self.last_tx, self.last_sent = self.now, self.target.copy()
                else: self.rejected += 1
            self.reported = self.side.update(self.now).copy()
        self.shaft += (self.reported - self.shaft) * min(1.0, self.dt / self.tau)
        self.out = min(max(self.out, self.shaft[0] - self.bl), self.shaft[0] + self.bl)
        # compliance under load with the servo's own lag: an instantaneous deflection closes an algebraic
        # loop with the pad force/damping and chatters at the physics rate (seen in v2 at large arms)
        self.defl += (self.comp * max(torque_Nm, 0.0) - self.defl) * min(1.0, self.dt / self.tau)
        return max(0.0, self.out - self.defl), float(self.shaft[1])


class ZoneFSR(SideFSR):
    """SideFSR whose rest reading carries the zone preload (absolute rest counts are a body cue)."""
    def __init__(self, rng, S, F_rest, offset):
        super().__init__(rng, S, F_rest)
        self.base = float(np.clip(offset + self.g * F_rest, 0, 4095))


class BolsterEnvV3(BolsterEnvV2):
    PRIV_DIM = 31
    OBS_DIM = OBS_DIM

    def __init__(self, level=0, seed=None, cfg_path=CFG2, scenario_kind=None, sequence=None, overlays=None, policy_off=False):
        super().__init__(level, seed, cfg_path, scenario_kind, sequence, [OV3] + list(overlays or []), policy_off)
        V = self.cfg['v3']; self.V = V
        self.pmax = float(V['pitch_max_deg']); self.pitch_on = bool(V['pitch_enabled'])
        self.obs_dim, self.action_dim = OBS_DIM, (4 if self.pitch_on else 2)
        self.cz = V['calib_zone']; self.oz = 'lo' if self.cz == 'hi' else 'hi'

    # ------------------------------------------------------------------ occupant / contact
    def reset(self, seed=None):
        """v2 reset, plus (with v3.clearance) a stability check of the drawn occupant WITH its real gaps: without
        bolster contact the delayed reflex loop of some draws (reflex 0.15 s) oscillates with growing amplitude
        (seed 1019: CoM +-9 cm, 400 N after 35 s) -- not a human behaviour; such draws are redrawn."""
        for _ in range(10):
            out = super().reset(seed); seed = None
            if not self.cfg['v3'].get('clearance') or self._stable(): return out
            self.redrawn = getattr(self, 'redrawn', 0) + 1
        return out

    def _stable(self, T=10.0):
        b = HumanLateral(self.hp, self.dt); g = tuple(self._gaps(0, 0)); n = int(T / self.dt); y = np.zeros(n)
        for i in range(n):
            t = i * self.dt; a = 0.3 * G if t < 1.0 else (-0.3 * G if t < 2.0 else 0.0)
            o = b.step(a, a, g, g); y[i] = o['y_com']
        k = int(2.0 / self.dt)
        early, late = np.ptp(y[-3 * k:-2 * k]), np.ptp(y[-k:])                # free response: must decay
        return bool(np.max(np.abs(y)) < 0.06 and late <= max(early, 1e-4))

    def _hparams(self, hp_, pr, sb, door_k, door_side):
        hp = super()._hparams(hp_, pr, sb, door_k, door_side)
        r, V = self.rng, self.cfg['v3']; zf = V['zone_frac_hi']
        # upper-zone share: narrower shoulders and a lower seating (shorter sitting torso, subject B) unload the upper pads
        frac = float(np.clip(zf['base'] + zf['per_m_shoulder'] * (pr['shoulder_width_m'] - 0.42)
                             + zf.get('per_m_torso', 0.0) * (pr['sitting_torso_m'] - 0.58) + r.normal(0, zf['sd']), *zf['clip']))
        self.v3p = dict(zone_dh=float(r.uniform(*V['zone_dh_m'])), zone_frac_hi=frac,
                        pitch_arm_ratio=float(r.uniform(*V['pitch_arm_ratio'])),
                        width_beta=float(r.uniform(*V['width_preload_beta'])),
                        crank=float(r.choice(V['crank_rate_choices'], p=V['crank_rate_p'])))
        fa = V.get('fb_a2')                                              # OASIS_FALLBACK v2 board: STM tune per episode
        if fa:
            self.v3p.update(fb_rate=float(r.choice(fa['rate_dps_choices'], p=fa['rate_p'])),
                            fb_cap=float(r.choice(fa['yaw_cap_choices'], p=fa['yaw_cap_p'])))
        cov = V.get('sensor_coverage')   # optional (off): closing moves the contact off the FSR area -- NOT the team model
        self.v3p['coverage_beta'] = {f'{s}_{z}': (float(r.uniform(*cov[f'beta_{z}'])) if cov else 0.0) for s in ('L', 'R') for z in ZONES}
        return replace(hp, zone_dh=self.v3p['zone_dh'], zone_frac_hi=frac)

    def _solve_arm(self, hp, preload, authority):
        """v2 solve on the calibrated zone, limited to a physical range: when a zone barely touches at rest the
        linear-ratio solve explodes (v3 p90 11 cm, clip 17 cm vs 1.5 cm calibrated) -> squeeze / reward spikes."""
        # the arm is a seat / bolster property: solve it at a contacting preload even for an occupant with clearance
        arm, console = super()._solve_arm(hp, max(preload, 5.0), authority)
        lo, hi = self.cfg['v3']['arm_range_m']
        return float(np.clip(arm, lo, hi)), console

    def _preload(self, sb, pr):
        """rest contact force of the side pads. With v3.clearance (v5, 2 subjects 10/26): an occupant narrower than
        the contact width w_c does not touch the pads at rest (subject B: side pads ~0-250 counts vs A ~2000) ->
        preload 0 and an effective clearance (w_c - w)/2 x scale the bolster must close before it supports."""
        self.clearance_m = 0.0
        C = self.cfg['v3'].get('clearance')
        if C:
            r = self.rng; w_c = float(r.uniform(*C['w_contact_m'])); sc = float(r.uniform(*C['scale']))
            self.v3p.update(w_contact_m=w_c, clearance_scale=sc)
            if pr['shoulder_width_m'] < w_c:
                self.clearance_m = (w_c - pr['shoulder_width_m']) / 2 * sc
                self.v3p['clearance_m'] = self.clearance_m
                return 0.0
        return sb['side_preload_N'] * float(np.exp(self.v3p['width_beta'] * (pr['shoulder_width_m'] - 0.42) / 0.06))

    def _static(self, hp, preload, arm, ay_g=0.0, thR=0.0, T=3.0):
        """side-diff of the CALIBRATED zone (the channels the car/bench gains were measured on)."""
        b = HumanLateral(hp, self.dt); g0 = -(preload / hp.k_contact); z = self.cfg['v3']['calib_zone']
        for i in range(int(T / self.dt)):
            r = min(1.0, i * self.dt / 0.5); gr = g0 - arm * np.sin(np.deg2rad(thR * r))
            o = b.step(ay_g * G * r, ay_g * G * r, (g0, g0), (gr, gr))
        return o[f'F_R_{z}'] - o[f'F_L_{z}']

    def _gaps(self, yaw, pitch):
        """(lower, upper) gap of one wing; +pitch (sign +1) moves the lower zone in, the upper out."""
        s = self.cfg['v3']['pitch_sign'] * self.arm * self.v3p['pitch_arm_ratio'] * np.sin(np.deg2rad(pitch))
        g = self.gap0 - self.arm * np.sin(np.deg2rad(yaw))
        return np.array([g - s, g + s])

    # ------------------------------------------------------------------ sensors
    def _setup_sensors(self, sb, pr):
        r, S = self.rng, self.cfg['sensors']; SB = self.cfg['seat_bolster']; V = self.cfg['v3']
        self.servo = {s: Servo2D(r, SB, self.dt, SB.get('actuator_mode', 'fw_event_hb'), self.v3p['crank'], self.pmax) for s in ('L', 'R')}
        for sv in self.servo.values():
            sv.fb_rate, sv.fb_cap = self.v3p.get('fb_rate', 20.0), self.v3p.get('fb_cap', 20.0)
        self.gap0 = -(self.preload / self.hp.k_contact) + getattr(self, 'clearance_m', 0.0)
        self.th = np.zeros(2); self.pt = np.zeros(2); self.cmd = np.zeros(4); self.a_prev = np.zeros(4)
        self.gz = {'L': self._gaps(0, 0), 'R': self._gaps(0, 0)}
        self.t = 0.0; self.trip = 0.0; self.hold = 0.0; self.hold_until = np.full(2, -1.0); self.hold_th = np.zeros(2)
        self.fsr = {}; self.out = None; self.ro_state0 = (0.0, self.hp.lean_bias)
        if hasattr(self, 'imu_vals'): delattr(self, 'imu_vals')
        self.cush = V.get('cush_groups') or {'R': ['a0_c3'], 'L': ['a0_c1', 'a0_c2']}     # v3 default (pre-map grouping)
        ro_ch = sorted({c for g in self.cush.values() for c in g} | {'a2_c1', 'a2_c2'})
        self.ro = {ch: CAL['readout'][ch] for ch in (('a0_c1', 'a0_c2', 'a0_c3', 'a2_c1', 'a2_c2') if not V.get('cush_groups') else ro_ch)}
        self.ro_noise = r.uniform(*S['noise_counts']); self.ro_scale = r.uniform(0.7, 1.4)
        from sensors import SampledSignal, AccelPredictor
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
        P = V.get('predictor') or {}; self.wheel = None
        self.ay_occ = self.ex.get('ay_occ', self.ex['ay'])           # dynamic car: occupant feels roll + bank on top
        self._road_setup()
        if P.get('kind') == 'beamng':
            # final (v7): learned 1 s preview = wheel prior + road curvature ahead + state (runtime class)
            npz = P.get('npz'); npz = os.path.join(ROOT, npz) if npz and not os.path.isabs(npz) else npz
            if npz and not os.path.exists(npz) and not getattr(BolsterEnvV3, '_warned_npz', False):
                import warnings; BolsterEnvV3._warned_npz = True
                warnings.warn(f'BeamNG predictor weights {npz} missing -> wheel prior only (run sim/v3/final/run_final.sh predictor)')
            self.wheel = BeamNGPredictor(npz if npz and os.path.exists(npz) else None, **P.get('params', {}))
        if P.get('kind') == 'wheel':
            # BeamNG profile (v6): the a_y preview is the runtime's WheelPreviewPredictor on the raw wheel input (fast
            # sampled through obd_sw by the BeamNG sensor overlay), IMU a_y and speed, with NOMINAL vehicle parameters
            self.wheel = WheelPreviewPredictor(**P.get('params', {}))
        if P.get('kind') in ('wheel', 'beamng') and self.params['kind'] == 'l1_replay':
            # the car log's OBD steering is ~0.55 s behind a_y (peak corr. of the steady-state a_y, real car);
            # the raw wheel leads the car's a_y by its yaw lag -> advance; same amplitude scale as a_y
            sp = self.params['scenario']; ex = self.ex
            adv = r.uniform(*P['l1_sw_advance_s']) / sp['l1_time_scale']
            ex['sw'] = np.interp(ex['t'] + adv, ex['t'], ex['sw']) * sp['l1_amp_scale']
        self.profile_est = np.array([pr['mass_kg'], pr['shoulder_width_m'], pr['sitting_torso_m']]) * \
            np.exp(r.normal(0, 1, 3) * np.asarray(V['profile_noise_frac']))
        self._advance(np.zeros(4), warmup=True)
        # counts per newton per zone: v2 anchoring (nominal passenger at this k) / nominal zone share
        self.G_zone = self.G_side / V['zone_share_nominal']
        S2 = dict(S, side_counts_per_N=self.G_zone)
        self.fsr = {(s, z): ZoneFSR(r, S2, self.out[f'F_{s}_{z}'], r.uniform(*V['side_offset_counts']))
                    for s in ('L', 'R') for z in ZONES}
        self.counts = {k: f(self.out[f'F_{k[0]}_{k[1]}'], self.dt) for k, f in self.fsr.items()}
        self.base = {k: f.base for k, f in self.fsr.items()}
        self.ro_state0 = (self.body.y, self.body.phi); self.ro_last = {ch: 0.0 for ch in self.ro}
        self.com0 = self.body.com_lateral()
        self.twin = copy.deepcopy(self.body); self.twin_gap = self._gaps(0, 0)
        if self.cfg['reward'].get('mode') == 'score': self._twin_energy()
        self._sense(0)

    def _twin_energy(self):
        """reward.mode score: the open-bolster twin does not depend on the policy, so run a copy of it once over the
        episode (same steps as _advance) for its CoM energy -- the denominator of the per-episode CoM reduction."""
        tw, W = copy.deepcopy(self.twin), self.cfg['reward']; t, E, n = self.t, 0.0, 0
        while True:
            for _ in range(self.sub):
                i = min(int(t / self.dt), len(self.ex['t']) - 1)
                tw.step(self.ay_occ[i], self._ay(t + self.hp.ff_lead), tuple(self.twin_gap), tuple(self.twin_gap)); t += self.dt
            yt = (tw.com_lateral() - self.com0) / W['com_scale_m']; E += yt * yt; n += 1
            if t >= self.T: break
        self.twin_E, self.n_ctrl = max(E, n * (W.get('com_floor_m', 0.001) / W['com_scale_m']) ** 2), n

    def _sense(self, i):
        o, t = self.out, self.t
        if self.fsr and int(t / self.dt) % 2 == 0:                # 100 Hz mat
            # sensed share of the zone force falls as that wing closes (bench 10/26: the right bolster closes onto
            # the body, yet its pads read LESS: -26.5 upper / -5.5 lower counts/deg) -> coverage 1 - beta yaw/th_max
            cov = {k: max(0.1, 1.0 - self.v3p['coverage_beta'][f'{k[0]}_{k[1]}'] * self.th[0 if k[0] == 'L' else 1] / self.th_max) for k in self.fsr}
            self.counts = {k: f(o[f'F_{k[0]}_{k[1]}'] * cov[k], 0.01) for k, f in self.fsr.items()}
            dy, dphi = 100 * (self.body.y - self.ro_state0[0]), np.rad2deg(self.body.phi - self.ro_state0[1])
            for ch, rr in self.ro.items():
                self.ro_last[ch] = self.ro_scale * (rr['weights'][0] * dy + rr['weights'][1] * dphi) + self.rng.normal(0, self.ro_noise)
        fsr, self.fsr = self.fsr, None                            # v2 _sense: pose / IMU / OBD part only
        super()._sense(i); self.fsr = fsr

    # ------------------------------------------------------------------ physics
    def _advance(self, cmd, warmup=False):
        n = int(2.0 / self.dt) if warmup else self.sub
        S = self.cfg['safety']
        for _ in range(n):
            i = min(int(self.t / self.dt), len(self.ex['t']) - 1)
            ay = 0.0 if warmup else self.ay_occ[i]
            ayf = 0.0 if warmup else self._ay(self.t + self.hp.ff_lead)
            F = (self.out['F_L'], self.out['F_R']) if self.out else (0.0, 0.0)
            c = np.array(cmd, float)
            for j in range(2):                                   # interlock hold: no further closing (yaw)
                if self.t < self.hold_until[j]: c[j] = min(c[j], self.hold_th[j])
            (yL, pL), (yR, pR) = self.servo['L']((c[0], c[2]), F[0] * self.arm), self.servo['R']((c[1], c[3]), F[1] * self.arm)
            gL, gR = self._gaps(yL, pL), self._gaps(yR, pR)
            dgL, dgR = (gL - self.gz['L']) / self.dt, (gR - self.gz['R']) / self.dt
            self.gz = {'L': gL, 'R': gR}; self.th = np.array([yL, yR]); self.pt = np.array([pL, pR])
            self.out = self.body.step(ay, ayf, tuple(gL), tuple(gR), tuple(dgL), tuple(dgR))
            if not warmup and hasattr(self, 'twin'):
                self.twin_out = self.twin.step(ay, ayf, tuple(self.twin_gap), tuple(self.twin_gap))
            if not warmup:
                self._sense(i)
                ay_meas = self.imu_vals['ay'][0] / G
                for j, s in enumerate(('L', 'R')):               # interlock: both zones of the wing
                    Fh = sum(self.counts[(s, z)] - self.base[(s, z)] for z in ZONES) / self.G_zone
                    if S.get('mode', 'model') == 'model':
                        load = -ay_meas if s == 'L' else ay_meas
                        Fh -= CAL['sim_console_N_per_g'] * max(load, 0.0)
                    rep = self.servo[s].reported[0]
                    if Fh > S['fsr_force_hard_N'] and self.th[j] > S['arm_deg']:
                        self.trip = 1.0; self.cmd[j] = 0.0; self.servo[s].target[0] = 0.0
                        self.hold_until[j] = self.t + S['hold_s']; self.hold_th[j] = 0.0
                    elif Fh > S['fsr_force_soft_N'] and self.th[j] > S['arm_deg']:
                        self.hold = 1.0
                        self.hold_th[j] = min(self.hold_th[j], rep) if self.t < self.hold_until[j] else rep
                        self.hold_until[j] = self.t + S['hold_s']
                self.t += self.dt

    # ------------------------------------------------------------------ observation
    def _obs(self):
        if not hasattr(self, 'imu_vals'): self._sense(0)
        iv, pe, c, b = self.imu_vals, self.profile_est, self.counts, self.base
        dz = lambda s, z: c[(s, z)] - b[(s, z)]
        if self.wheel is not None:
            ay_p, ax_p = self.wheel(self.veh_sample())
        else:
            ay_p, ax_p = self.pred(self._ay, self.t) / G, self.pred_ax(self._ax, self.t) / G
        d = dict(ay_pred_g=ay_p, ax_pred_g=ax_p,
                 ay_g=iv['ay'][0] / G, ax_g=iv['ax'][0] / G, yaw_rate_radps=self._yaw_rate(), speed_mps=self.v_obd[0],
                 steer_rad=self.sw_obd[0],
                 side_lo_R_counts=dz('R', 'lo'), side_lo_L_counts=dz('L', 'lo'), side_hi_R_counts=dz('R', 'hi'), side_hi_L_counts=dz('L', 'hi'),
                 back_R_counts=self.ro_last['a2_c2'], back_L_counts=self.ro_last['a2_c1'],
                 cush_R_counts=float(np.mean([self.ro_last[c] for c in self.cush['R']])),
                 cush_L_counts=float(np.mean([self.ro_last[c] for c in self.cush['L']])),
                 torso_lean_deg=self.pose[0], shoulder_shift_cm=self.pose[1],
                 yaw_reported_L_deg=self.servo['L'].reported[0], yaw_reported_R_deg=self.servo['R'].reported[0],
                 pitch_reported_L_deg=self.servo['L'].reported[1], pitch_reported_R_deg=self.servo['R'].reported[1],
                 prev_action=self.a_prev, mass_kg=pe[0], shoulder_width_m=pe[1], sitting_torso_m=pe[2],
                 side_rest_counts=[b[('R', 'lo')], b[('L', 'lo')], b[('R', 'hi')], b[('L', 'hi')]],
                 pose_age_s=self.pose_age, pressure_valid=all(v > 5 for v in c.values()),
                 safety_flag=max(self.trip, 0.5 * self.hold))
        return build_obs(d, self.th_max, self.pmax)

    def veh_sample(self):
        """what the BeamNG bridge sends the Pi (oasis_rt.sources schema): sensors as sampled, road preview"""
        iv = self.imu_vals; i = min(int(self.t / self.dt), len(self.ex['t']) - 1)
        road, valid = self._road_ay(i, self.v_obd[0])
        return dict(t=self.t, steer_rad=self.sw_obd[0], speed_mps=self.v_obd[0], ay_mps2=iv['ay'][0], ax_mps2=iv['ax'][0],
                    yaw_rate_radps=self._yaw_rate(), road_ay_g=road, road_valid=valid)

    def _yaw_rate(self):
        """v3.yaw_rate_source from_ay (v8): BeamNG Collector rows carry no yaw rate; the Pi estimates
        a_y / max(v, 5 m/s) (oasis_rt.beamng_rx.VehicleInput.sample) -- train on exactly that."""
        iv = self.imu_vals
        if self.V.get('yaw_rate_source') == 'from_ay':
            v = self.v_obd[0]; return float(iv['ay'][0] / max(v, 5.0)) if v > 0.5 else 0.0
        return iv['wz'][0]

    def _road_setup(self):
        """Road ahead as BeamNGpy's road network gives it (v3.road). The car follows the road in turns, curves and
        real drives (driven path = road smoothed, cut / turned in early or late, map noise); lane changes and slaloms
        happen on a straight road. At a junction without a known route the branch is only known once the car turns.
        Not every session has the road network (BeamNG.drive protocols carry none) -> road_valid 0."""
        R = self.V.get('road'); self.road = None
        if not R: return
        r, ex, dt, kind = self.rng, self.ex, self.dt, self.params['kind']
        v = np.maximum(ex['v'], 0.0); s = np.r_[0.0, np.cumsum(v[1:] * dt)]; kp = ex['yaw'] / np.maximum(v, 1.0)
        avail = bool(r.random() < R['p_available']); follows = kind in R['follow_kinds']
        sg = np.arange(0.0, s[-1] + 80.0, 0.5)
        if follows:
            kg = np.interp(sg, s, kp); w = max(1, int(r.uniform(*R['smooth_m']) / 0.5))
            kg = np.convolve(kg, np.ones(w) / w, 'same') * r.uniform(*R['gain'])
            kg = np.interp(sg + r.uniform(*R['shift_m']), sg, kg)
        else:
            kg = np.zeros_like(sg)
        e = r.normal(0, 1, len(sg)); a = 0.5 / 10.0
        for j in range(1, len(e)): e[j] = e[j - 1] + a * (e[j] - e[j - 1])
        kg = kg + r.uniform(*R['noise_kappa']) * e / (np.std(e) + 1e-9)
        commit = (np.abs(kp) > R['commit_kappa']) if (follows and kind == 'urban_turn' and r.random() > R['p_route_known']) else None
        self.road = dict(s=s, sg=sg, k=kg, avail=avail, commit=commit, s_err=float(r.uniform(*R['s_err_m']) * r.normal()))
        self.v3p.update(road_available=avail, road_follows=follows, route_known=commit is None)

    def _road_ay(self, i, v):
        R = self.road
        if R is None or not R['avail']: return np.zeros(10), 0.0
        if R['commit'] is not None and not R['commit'][i]: return np.zeros(10), 1.0     # straight on until it turns
        k = np.interp(R['s'][i] + R['s_err'] + max(v, 0.0) * PRED_H, R['sg'], R['k'])
        return v * v * k / G, 1.0

    def privileged_vec(self):
        v = super().privileged_vec()
        return np.r_[v, self.pt / self.pmax, self.out['F_L_hi'] / 50, self.out['F_R_hi'] / 50].astype(np.float32)

    def _info(self):
        i = super()._info(); i['privileged']['pitch'] = np.deg2rad(self.pt)
        i['privileged'].update({k: self.out[k] for k in ('F_L_lo', 'F_L_hi', 'F_R_lo', 'F_R_hi')})
        if hasattr(self, 'twin_out'): i['privileged']['added_squeeze_N'] = self.added_squeeze_N()
        i['params'] = dict(i['params'], v3=self.v3p); return i

    def added_squeeze_N(self):
        """force the bolsters ADD on the occupant vs the same occupant with them open (max over the wings)"""
        return max(self.out['F_L'] - self.twin_out['F_L'], self.out['F_R'] - self.twin_out['F_R'], 0.0)

    def over_press(self):
        """'pressed too hard': added squeeze beyond reward.over_F_N, relative to it (0 below the limit)"""
        F0 = self.cfg['reward'].get('over_F_N', 20.0)
        return max(self.added_squeeze_N() - F0, 0.0) / F0

    # ------------------------------------------------------------------ step
    def step(self, action):
        a = np.zeros(4); x = np.clip(np.asarray(action, float), -1, 1); a[:len(x)] = x
        if not self.pitch_on: a[2:] = 0.0
        if self.policy_off: a = np.array([-1.0, 0.0, 0.0, 0.0])
        yL, yR, pL, pR = action_to_targets(a, self.th_max, self.pmax)
        self.cmd = np.array([yL, yR, pL, pR]); self.trip = 0.0; self.hold = 0.0
        self._advance(self.cmd)
        W = self.cfg['reward']
        y = (self.body.com_lateral() - self.com0) / W['com_scale_m']
        yt = (self.twin.com_lateral() - self.com0) / W['com_scale_m']
        com_term = (y ** 2 - yt ** 2) if W.get('counterfactual', True) else y ** 2
        if W.get('mode') == 'score':
            # v8: the evaluation score itself (sim/v3/tune_rule.py), spread over the steps; sums to ~ CoM RMS
            # reduction [%] - 0.5 x time over over_F_N [%] - 100 x mean over-press^2, plus a small smoothness term
            F0 = W.get('over_F_N', 20.0)
            r = (-50.0 * com_term / self.twin_E - (50.0 * float(self.added_squeeze_N() > F0) + 100.0 * self.over_press() ** 2) / self.n_ctrl
                 - W['w_du'] * np.sum((a - self.a_prev) ** 2) - W['w_trip'] * self.trip)
            self.a_prev = a
            return self._obs(), float(r), False, bool(self.t >= self.T), self._info()
        dF = max(self.out['F_L'] - self.twin_out['F_L'], self.out['F_R'] - self.twin_out['F_R'], 0.0) / W.get('force_scale_N', 100.0)
        r = -(W['w_com'] * com_term + W['w_du'] * np.sum((a - self.a_prev) ** 2) + W['w_theta'] * np.sum((self.th / self.th_max) ** 2)
              + W.get('w_pitch', 0.0) * np.sum((self.pt / self.pmax) ** 2) + W['w_force'] * dF ** 2 + W['w_trip'] * self.trip
              + W.get('w_over', 0.0) * self.over_press() ** 2)
        self.a_prev = a
        return self._obs(), float(r), False, bool(self.t >= self.T), self._info()


if __name__ == '__main__':
    import time
    for kind in ('l1_replay', 'urban_turn'):
        e = BolsterEnvV3(level=1, seed=3, scenario_kind=kind); t0 = time.time(); o, info = e.reset(); n = 0; R = 0
        while True:
            o, rr, te, tr, info = e.step([-1, np.clip(-8 * o[6], -1, 1), 0, 0]); R += rr; n += 1
            if tr: break
        p = info['params']
        print(kind, 'steps', n, 'ret', round(R, 2), 'obs', o.shape, 'arm', round(100 * p['arm_eff_m'], 2), 'cm',
              'preload', round(p['preload_N'], 1), {k: round(v, 3) for k, v in p['v3'].items()}, 'wall', round(time.time() - t0, 2), 's')
