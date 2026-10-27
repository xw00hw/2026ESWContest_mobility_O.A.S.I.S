"""Lateral occupant model (vehicle frame, +y = left) used by the L3 simulator.

Two coupled DOF:
  y_p  pelvis lateral slide on the cushion  (cushion shear + passive side support,
       Coulomb friction cap)
  phi  torso roll about the lumbar joint    (+phi = torso tips to the left)

Turning left (a_y > 0) loads the body to the right (-y), which presses the RIGHT
backrest wing -- this matches the L1 log (A1_C1 r=+0.80 with a_y).

Stiffness is parameterised by a closed-loop natural frequency fn and damping ratio
zeta (identified from L1 / Fairley & Griffin 1990: lateral mode ~1.5 Hz with
backrest), split into a passive part and a delayed active (reflex) part. The
"driver" archetype adds steering-wheel bracing and a centripetal anticipatory lean
(Wada et al.); the "passenger" archetype has neither.
"""
from dataclasses import dataclass, field
import numpy as np

G = 9.81


@dataclass
class HumanParams:
    mass: float = 72.0            # kg, whole body
    torso_frac: float = 0.55      # upper-body mass / body mass
    h_com: float = 0.32           # m, torso CoM above lumbar pivot
    fn_torso: float = 1.5         # Hz, closed-loop torso roll mode
    zeta_torso: float = 1.0
    passive_frac: float = 0.5     # share of stiffness/damping that is passive
    reflex_delay: float = 0.08    # s
    fn_pelvis: float = 4.0        # Hz, cushion shear mode
    zeta_pelvis: float = 0.7
    mu_cushion: float = 0.5       # pelvis friction cap (x normal load)
    brace_k: float = 0.0          # N*m/rad extra roll stiffness from holding the wheel
    ff_gain: float = 0.0          # anticipatory lean torque, fraction of inertial torque
    ff_lead: float = 0.0          # s, how far ahead the occupant anticipates
    lean_bias: float = 0.0        # rad, habitual lean
    asym: float = 1.0             # right/left effective stiffness ratio (posture/seat asymmetry)
    relax_gain: float = 0.0       # interaction: active (muscle) stiffness drops as bolster support rises,
                                  # k_act / (1 + relax_gain * F_contact / 100 N)  (cf. Okiyama 2001: less EMG
                                  # with good lateral support). 0 = no interaction (v0.2 default)
    # contact with the backrest wings (bolsters)
    h_bolster: float = 0.20       # m above lumbar pivot where the wing touches
    half_width: float = 0.17      # m, torso half-width at h_bolster
    k_contact: float = 8000.0     # N/m^n
    n_contact: float = 1.5
    c_contact: float = 150.0      # N*s/m
    # un-instrumented support on one side (car door / armrest for a passenger): takes part of the
    # load when the torso moves toward it, so the seat sensors on that side see less (L1: 0.48x)
    door_k: float = 0.0           # N/m, 0 = no door (bench chair)
    door_side: int = -1           # -1 = right (+y is left), +1 = left
    door_gap: float = 0.0         # m
    # two contact zones per wing (v3, bolster pitch): lower/upper pad at h_bolster -/+ zone_dh, each
    # zone_frac / (1 - zone_frac) of k_contact and c_contact. zone_dh = 0 -> one zone (v0.2 / v2 behaviour)
    zone_dh: float = 0.0          # m
    zone_frac_hi: float = 0.5     # share of the wing stiffness in the upper zone


class HumanLateral:
    def __init__(self, p: HumanParams, dt: float):
        self.p, self.dt = p, dt
        mt = p.mass * p.torso_frac
        self.mt, self.mp = mt, p.mass - mt
        self.I = mt * p.h_com**2 * 1.3          # point mass + distributed inertia
        w = 2 * np.pi * p.fn_torso
        k_tot = self.I * w**2 + mt * G * p.h_com  # cancels the gravity toppling term
        c_tot = 2 * p.zeta_torso * w * self.I
        self.k_pas, self.c_pas = p.passive_frac * k_tot, p.passive_frac * c_tot
        self.k_act, self.c_act = (1 - p.passive_frac) * k_tot, (1 - p.passive_frac) * c_tot
        wp = 2 * np.pi * p.fn_pelvis
        self.kp_y, self.cp_y = p.mass * wp**2, 2 * p.zeta_pelvis * wp * p.mass
        nd = max(1, int(round(p.reflex_delay / dt)))
        self.buf = np.zeros((nd, 2))
        self.reset()

    def reset(self):
        self.y, self.vy, self.phi, self.om = 0.0, 0.0, self.p.lean_bias, 0.0
        self.buf[:] = 0
        self.slip = 0.0

    def com_lateral(self):
        """Upper-body CoM lateral displacement w.r.t. the seat centreline [m]."""
        return self.y + self.p.h_com * np.sin(self.phi)

    def step(self, a_y, a_y_future, gap_L, gap_R, dgap_L=0.0, dgap_R=0.0):
        """a_y: vehicle lateral accel [m/s2]; a_y_future: a_y at t+ff_lead (anticipation);
        gap_L/R: distance from torso side (neutral) to each wing surface [m] (>=0 = clear).
        Returns dict of contact forces and cushion loads."""
        p, dt = self.p, self.dt
        # wing contact (torso at h_bolster)
        yb = self.y + p.h_bolster * np.sin(self.phi)
        vb = self.vy + p.h_bolster * np.cos(self.phi) * self.om
        zones = None
        if p.zone_dh > 0:             # v3: gap_L/R = (lower, upper) per wing
            zones = {}; tau_z = 0.0; F_R = F_L = 0.0
            if not np.ndim(gap_R): gap_R = (gap_R, gap_R)          # scalar gap = same for both zones
            if not np.ndim(gap_L): gap_L = (gap_L, gap_L)
            for z, h, f in (('lo', p.h_bolster - p.zone_dh, 1 - p.zone_frac_hi), ('hi', p.h_bolster + p.zone_dh, p.zone_frac_hi)):
                j = 0 if z == 'lo' else 1
                yz = self.y + h * np.sin(self.phi); vz = self.vy + h * np.cos(self.phi) * self.om
                pr_, pl_ = -yz - gap_R[j], yz - gap_L[j]
                dR = dgap_R[j] if np.ndim(dgap_R) else dgap_R; dL = dgap_L[j] if np.ndim(dgap_L) else dgap_L
                # damping on the penetration RATE: d(pen_R)/dt = -v - dgap_R/dt (a closing pad adds force).
                # (the single-zone v0.2/v2 branch keeps its +dgap convention so v2 stays bit-identical)
                fr = (f * p.k_contact * pr_ ** p.n_contact + f * p.c_contact * max(-vz - dR, 0)) if pr_ > 0 else 0.0
                fl = (f * p.k_contact * pl_ ** p.n_contact + f * p.c_contact * max(vz - dL, 0)) if pl_ > 0 else 0.0
                zones[f'F_R_{z}'], zones[f'F_L_{z}'] = fr, fl; F_R += fr; F_L += fl
                tau_z += (fr - fl) * h * np.cos(self.phi)
        else:
            pen_R = -yb - gap_R           # torso moving right into right wing
            pen_L = yb - gap_L
            F_R = (p.k_contact * max(pen_R, 0) ** p.n_contact + p.c_contact * max(-vb + dgap_R, 0)) if pen_R > 0 else 0.0
            F_L = (p.k_contact * max(pen_L, 0) ** p.n_contact + p.c_contact * max(vb + dgap_L, 0)) if pen_L > 0 else 0.0
        F_door = 0.0
        if p.door_k > 0:
            pen_D = (yb * p.door_side) - p.door_gap
            if pen_D > 0: F_door = -p.door_side * p.door_k * pen_D
        F_wing = F_R - F_L + F_door   # net +y force on torso
        self._F_prev = F_L + F_R
        # delayed active torque (reflex) + anticipation + bracing
        phi_d, om_d = self.buf[0]
        self.buf = np.roll(self.buf, -1, axis=0); self.buf[-1] = (self.phi - p.lean_bias, self.om)
        k_side = p.asym if (self.phi < 0) else 1.0   # asymmetric posture stiffness
        relax = 1.0 / (1.0 + p.relax_gain * (self._F_prev / 100.0))
        tau_act = -relax * (self.k_act * k_side * phi_d + self.c_act * om_d)
        tau_ff = p.ff_gain * self.mt * p.h_com * a_y_future   # lean into the turn (+phi for a_y>0)
        tau_pas = -(self.k_pas * k_side * (self.phi - p.lean_bias) + self.c_pas * self.om) - p.brace_k * self.phi
        tau_in = -self.mt * p.h_com * np.cos(self.phi) * (a_y + self._ay_pel)  # inertial
        tau_g = self.mt * G * p.h_com * np.sin(self.phi)
        tau_w = F_wing * p.h_bolster * np.cos(self.phi) if zones is None else tau_z + F_door * p.h_bolster * np.cos(self.phi)
        al = (tau_in + tau_g + tau_pas + tau_act + tau_ff + tau_w) / self.I
        # pelvis: cushion shear spring with Coulomb cap
        N = p.mass * G
        f_sp = -self.kp_y * self.y - self.cp_y * self.vy
        f_cap = p.mu_cushion * N
        f_c = np.clip(f_sp, -f_cap, f_cap)
        self.slip = float(abs(f_sp) > f_cap)
        ay_p = (-p.mass * a_y + f_c + F_wing) / p.mass
        self._ay_pel = ay_p
        # semi-implicit Euler
        self.vy += ay_p * dt; self.y += self.vy * dt
        self.om += al * dt; self.phi += self.om * dt
        # cushion L/R normal loads (roll moment of the whole body about the cushion)
        h_seat = 0.10 + p.torso_frac * p.h_com
        M = -p.mass * a_y * h_seat + self.mt * G * p.h_com * np.sin(self.phi)
        w_c = 0.30
        N_L = N / 2 + M / w_c; N_R = N / 2 - M / w_c
        out = dict(F_L=F_L, F_R=F_R, N_L=max(N_L, 0), N_R=max(N_R, 0),
                   y_com=self.com_lateral(), y_pelvis=self.y, phi=self.phi, slip=self.slip)
        if zones is not None: out.update(zones)
        return out

    _ay_pel = 0.0
    _F_prev = 0.0
