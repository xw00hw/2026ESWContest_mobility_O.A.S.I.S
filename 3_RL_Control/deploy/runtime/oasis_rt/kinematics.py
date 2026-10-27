"""C4_R8 bolster kinematics for the Pi side (port of ESW repo
4_Seat_Actuation/gui/kinematics.py, revision C4_R8, same numbers).

The firmware REJECTS out-of-range / motor-bound commands (#ERR) and LATCHES a fault on an
unreachable path (PATH_UNREACHABLE -> disarmed, needs operator re-ARM). The Pi therefore
projects every policy target into the reachable set before sending it.
"""
from math import asin, atan2, cos, degrees, dist, hypot, radians, sin

YAW_MIN, YAW_FW_MAX, PITCH_MAX = 0.0, 27.0, 8.0
CRANK_ZERO_US = 1200
_R = 24.0
_CENTERS = ((-65, -36, 48), (-65, -36, -48))
_ANCHORS = ((-55, 12, 48), (-55, 12, -48))
_SIGNS = (1, -1)


def crank_angles(yaw_deg, pitch_deg):
    """Structure (yaw, pitch) [deg] -> two crank angles [deg], or None if unreachable."""
    if not (YAW_MIN <= yaw_deg <= YAW_FW_MAX and -PITCH_MAX <= pitch_deg <= PITCH_MAX):
        return None
    cy, sy = cos(radians(-yaw_deg)), sin(radians(-yaw_deg))
    cp, sp = cos(radians(pitch_deg)), sin(radians(pitch_deg))
    R = ((cy, -sy * cp, sy * sp), (sy, cy * cp, -cy * sp), (0.0, sp, cp))
    out = []
    for c, a, s in zip(_CENTERS, _ANCHORS, _SIGNS):
        L = dist(a, (c[0], c[1], c[2] + s * _R))
        p = [sum(R[i][j] * a[j] for j in range(3)) for i in range(3)]
        d = [p[i] - c[i] for i in range(3)]
        rho = hypot(d[1], d[2])
        if rho < 1e-10:
            return None
        q = (sum(x * x for x in d) + _R * _R - L * L) / (2 * _R * rho)
        if abs(q) > 1 + 1e-9:
            return None
        out.append(degrees(asin(max(-1.0, min(1.0, q)))) - degrees(atan2(s * d[2], d[1])))
    return out


def pulses_ok(yaw_deg, pitch_deg):
    """Same DS PWM bound check as kinematics.pulses(): 500..2500 us around the 1200 us crank zero."""
    phi = crank_angles(yaw_deg, pitch_deg)
    if phi is None:
        return False
    for p in phi:
        off = p * 2000 / 270
        delta = int(off + .5) if off >= 0 else -int(-off + .5)
        if not 500 <= CRANK_ZERO_US + delta <= 2500:
            return False
    return True


def project(yaw_deg, pitch_deg, yaw_max=20.0):
    """Closest commandable pose: clip to the box, then shrink |pitch| (then yaw) until reachable.
    Returns (yaw, pitch, changed)."""
    y = min(max(float(yaw_deg), YAW_MIN), yaw_max)
    p = min(max(float(pitch_deg), -PITCH_MAX), PITCH_MAX)
    changed = (y, p) != (yaw_deg, pitch_deg)
    for _ in range(64):
        if pulses_ok(y, p):
            return y, p, changed
        changed = True
        if abs(p) > 0.25:
            p -= 0.25 if p > 0 else -0.25
        else:
            p, y = 0.0, max(0.0, y - 0.25)
    return 0.0, 0.0, True
