"""Behavioural twin of the STM32 bolster firmware (ESW repo, branch
feat/seat-control-data-acquisition, 4_Seat_Actuation/stm32, OASIS_CONTROL v6):

  * C4_R8 inverse kinematics (port of gui/kinematics.py) -> crank angles / reachability
  * SET,id,ly,lp,ry,rp : rejects out-of-range / unreachable (PATH_UNREACHABLE latches a fault)
  * every accepted SET restarts a quintic (minimum-jerk) path from the CURRENT pose,
    duration = Joint_Duration (peak crank rate <= 15 deg/s x 0.8, min 200 ms)
  * 20 ms update loop; TIMEOUT fault if no accepted SET for 1.5 s while armed
  * proposed TRK mode (not in firmware): rate/accel-limited tracking without restart

Used to (a) put the real actuator behaviour into env v2, (b) choose how the Pi sends commands.
"""
from math import asin, atan2, cos, degrees, dist, hypot, radians, sin, ceil
import numpy as np

YAW_MAX, PITCH_MAX = 27.0, 8.0
GEOM = dict(radius=24.0, centers=[[-65, -36, 48], [-65, -36, -48]], anchors=[[-55, 12, 48], [-55, 12, -48]], signs=[1, -1])
RATE = 15.0            # MOTOR_RATE_MDEG_S / 1000
TIMEOUT_S = 1.5


def crank_angles(yaw, pitch):
    """C4_R8 IK: structure yaw/pitch [deg] -> two crank angles phi [deg], or None if unreachable."""
    if not (0 <= yaw <= YAW_MAX and -PITCH_MAX <= pitch <= PITCH_MAX):
        return None
    cy, sy = cos(radians(-yaw)), sin(radians(-yaw)); cp, sp = cos(radians(pitch)), sin(radians(pitch))
    R = [[cy, -sy * cp, sy * sp], [sy, cy * cp, -cy * sp], [0, sp, cp]]
    out = []
    for c, a, s in zip(GEOM['centers'], GEOM['anchors'], GEOM['signs']):
        r = GEOM['radius']; L = dist(a, [c[0], c[1], c[2] + s * r])
        p = [sum(R[i][j] * a[j] for j in range(3)) for i in range(3)]
        d = [p[i] - c[i] for i in range(3)]; rho = hypot(d[1], d[2])
        q = (sum(x * x for x in d) + r * r - L * L) / (2 * r * rho)
        if rho < 1e-10 or abs(q) > 1 + 1e-9: return None
        out.append(degrees(asin(max(-1, min(1, q))) - atan2(s * d[2], d[1])))
    return out


def reachable(yaw, pitch):
    return crank_angles(yaw, pitch) is not None


def duration_s(y0, p0, y1, p1, rate=RATE):
    """Port of Joint_Duration: sample the quintic path, peak crank rate -> duration (s), 0 = unreachable."""
    prev = crank_angles(y0, p0)
    if prev is None: return 0.0
    peak = 0.0
    for k in range(1, 65):
        t = k / 64; f = t ** 3 * (10 + t * (-15 + 6 * t))
        phi = crank_angles(y0 + (y1 - y0) * f, p0 + (p1 - p0) * f)
        if phi is None: return 0.0
        for i in range(2): peak = max(peak, abs(phi[i] - prev[i]) * 64); prev[i] = phi[i]
    return max(0.2, ceil(peak * 1250.0 / rate) / 1000.0)


class Side:
    """One bolster (yaw, pitch) as the firmware moves it."""
    def __init__(self):
        self.pose = np.zeros(2); self.frm = np.zeros(2); self.tgt = np.zeros(2); self.t0 = 0.0; self.T = 0.0; self.moving = False
        self.vel = np.zeros(2)

    def set(self, now, target, rate=RATE):            # firmware SET semantics (restart from current pose)
        T = duration_s(self.pose[0], self.pose[1], target[0], target[1], rate)
        if T == 0: return False
        self.frm = self.pose.copy(); self.tgt = np.asarray(target, float); self.t0 = now; self.T = T; self.moving = True
        return True

    def update(self, now):
        if not self.moving: return self.pose
        t = min(1.0, (now - self.t0) / self.T); f = t ** 3 * (10 + t * (-15 + 6 * t))
        self.pose = self.frm + (self.tgt - self.frm) * f
        if t >= 1: self.moving = False
        return self.pose

    def track(self, target, dt, vmax=12.0, amax=60.0):   # PROPOSED: no restart, rate + accel limited
        e = np.asarray(target, float) - self.pose
        v_des = np.clip(e / max(dt, 1e-3) * 0.25, -vmax, vmax)            # approach without overshoot
        self.vel = self.vel + np.clip(v_des - self.vel, -amax * dt, amax * dt)
        self.pose = self.pose + self.vel * dt
        return self.pose


US_PER_CRANK_DEG = 2000.0 / 270.0                   # DS5180 270 deg (OASIS_FALLBACK v2, ADR-026)
CAD_KNOTS = ((0, 0), (6, 6), (15, 6), (20, 2), (22, 0))  # BeamNG_Bolster_FB CAD yaw -> |pitch| cap [deg] (ADR-025)


def cad_pitch_cap(yaw):
    for (y0, p0), (y1, p1) in zip(CAD_KNOTS, CAD_KNOTS[1:]):
        if yaw <= y1: return p0 + (p1 - p0) * (max(yaw, 0.0) - y0) / (y1 - y0)
    return 0.0


def fb_a2_step(pose, target, rate_dps=(20.0, 20.0), step_us=2.0):
    """One 20 ms frame of OASIS_FALLBACK v2 Control_Poll for one side (yaw, pitch) [deg]
    (BeamNG_Bolster_FB stm32 fallback_control.c): per-axis STM tune rate on a common fraction (straight coupled
    path), then every servo pulse within step_us of the previous frame (Joint_LimitStep bisection), i.e.
    13.5 deg/s crank at 2 us -- no restart when the target changes."""
    pose, target = np.asarray(pose, float), np.asarray(target, float)
    d = target - pose
    if not np.any(d): return pose
    frac = min([1.0] + [r * 0.02 / abs(x) for x, r in zip(d, rate_dps) if x])
    cur = crank_angles(*pose)
    if cur is None: return pose
    def ok(f):
        c = crank_angles(*(pose + d * f))
        return c is not None and max(abs(a - b) for a, b in zip(c, cur)) * US_PER_CRANK_DEG <= step_us + 1e-9
    if ok(frac): return pose + d * frac
    c = crank_angles(*(pose + d * frac))                # locally linear: scale once, then bisect if needed
    if c is not None:
        f = frac * step_us / (max(abs(a - b) for a, b in zip(c, cur)) * US_PER_CRANK_DEG) * 0.999
        if ok(f): return pose + d * f
    lo, hi = 0.0, frac
    for _ in range(20):
        m = (lo + hi) / 2
        lo, hi = (m, hi) if ok(m) else (lo, m)
    return pose + d * lo


def simulate(t, target_deg, mode='stream', period=0.05, deadband=1.0, keepalive=1.0):
    """Drive one side's YAW (pitch 0) with a commanded trace; return the actual pose trace.
    mode: 'stream' = SET every `period`; 'event' = SET when |target-last_sent| >= deadband (min period)
          or for keep-alive; 'track' = proposed firmware tracking mode."""
    s = Side(); out = np.zeros(len(t)); last_tx = -1e9; last_sent = None; faults = 0
    for i, now in enumerate(t):
        tg = float(np.clip(target_deg[i], 0, 20))
        if mode == 'track':
            if i % int(round(0.02 / (t[1] - t[0]))) == 0 or True: out[i] = s.track([tg, 0.0], t[1] - t[0])[0]
            continue
        due = now - last_tx >= period
        if mode == 'stream' and due:
            s.set(now, [tg, 0.0]); last_tx = now; last_sent = tg
        elif mode == 'event' and due and (last_sent is None or abs(tg - last_sent) >= deadband or now - last_tx >= keepalive):
            s.set(now, [tg, 0.0]); last_tx = now; last_sent = tg
        if now - last_tx > TIMEOUT_S: faults += 1
        out[i] = s.update(now)[0]
    return out, faults
