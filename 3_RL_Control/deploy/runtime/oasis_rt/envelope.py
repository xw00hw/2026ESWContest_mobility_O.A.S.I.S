"""Commandable pose set of the OASIS_FALLBACK v2 board, enforced on the Pi BEFORE every FB_SET.

The firmware faults (PWM OFF, new session required) on any target outside
  CAD   : yaw 0..22 deg, |pitch| <= piecewise-linear cap (0,0) (6,6) (15,6) (20,2) (22,0) deg, integer floor
          (experiment_a/cad_limits.py, same sha256 as the firmware's fallback_geometry_profile.h)
  TUNE  : yaw <= yaw_max_mdeg, |pitch| <= pitch_max_mdeg of the committed STM Live-Expressions snapshot (#TUNE)
  IK    : C4_R8 reachability, 500..2500 us pulses (experiment_a/kinematics.py)
so policy targets are projected here: clip yaw, then |pitch| to the cap at that yaw (integer mdeg, rounded inward).
Order LY, LP, RY, RP (firmware pose order); yaw +closes toward the occupant on both sides.
"""
import math


class Envelope:
    def __init__(self, fb, tuning=None, yaw_cap_deg=None):
        self.fb = fb; cl = fb.cad_limits
        self.yaw_max = int(cl.CAD_YAW_MAX_MDEG); self.pitch_max = int(cl.CAD_PITCH_MAX_MDEG)
        if tuning:
            self.yaw_max = min(self.yaw_max, int(tuning['yaw_max_mdeg'])); self.pitch_max = min(self.pitch_max, int(tuning['pitch_max_mdeg']))
        if yaw_cap_deg is not None:
            self.yaw_max = min(self.yaw_max, int(math.floor(1000 * float(yaw_cap_deg))))
        self.knots = cl.CAD_CAP_POINTS

    def pitch_cap_mdeg(self, yaw_mdeg):
        for (y0, p0), (y1, p1) in zip(self.knots, self.knots[1:]):
            if yaw_mdeg <= y1:
                return min(self.pitch_max, (p0 * (y1 - y0) + (p1 - p0) * (yaw_mdeg - y0)) // (y1 - y0))
        return 0

    def side(self, yaw_deg, pitch_deg):
        if not (math.isfinite(yaw_deg) and math.isfinite(pitch_deg)):
            raise ValueError(f'non-finite target ({yaw_deg}, {pitch_deg})')
        y = min(max(int(math.floor(1000 * yaw_deg + 1e-6)), 0), self.yaw_max)
        cap = self.pitch_cap_mdeg(y)
        p = min(max(int(round(1000 * pitch_deg)), -cap), cap)
        return y, p

    def ok(self, pose):
        fb = self.fb
        if not fb.cad_limits.pose_within_cad(pose): return False
        if any(not 0 <= pose[i] <= self.yaw_max or abs(pose[i + 1]) > self.pitch_max for i in (0, 2)): return False
        try:
            fb.kinematics.pose_to_pulses_us([v / 1000 for v in pose])
        except ValueError:
            return False
        return True

    def project(self, yL, pL, yR, pR):
        """deg -> (integer mdeg pose inside all three sets, changed?)"""
        pose = (*self.side(yL, pL), *self.side(yR, pR))
        while not self.ok(pose):                          # IK guard (not reached inside the CAD set, kept fail-closed)
            pose = tuple(int(v * 0.9) for v in pose)
        changed = any(abs(a - 1000 * b) > 1.0 for a, b in zip(pose, (yL, pL, yR, pR)))
        return pose, changed
