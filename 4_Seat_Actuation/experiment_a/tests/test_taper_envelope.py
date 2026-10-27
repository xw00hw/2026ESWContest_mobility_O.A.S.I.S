"""Independent reachability and path checks for the STEP-derived CAD envelope."""
import math
import random
import unittest

from experiment_a.kinematics import (
    allowable_pitch_for_yaw, inverse_side, inward_offsets_mm,
    max_pitch_for_yaw, pose_to_pulses_us,
)


def max_abs_q(yaw, pitch):
    """Independent closure discriminant from the original anchor geometry."""
    psi, beta = math.radians(yaw), math.radians(pitch)
    cy, sy, cp, sp = math.cos(psi), math.sin(psi), math.cos(beta), math.sin(beta)
    result = []
    for sign in (1, -1):
        anchor = (-55*cy + (12*cp - sign*48*sp)*sy,
                  55*sy + (12*cp - sign*48*sp)*cy,
                  12*sp + sign*48*cp)
        d = (anchor[0] + 65, anchor[1] + 36, anchor[2] - sign*48)
        result.append(abs((sum(x*x for x in d) + 24**2 - 2980) / (48*math.hypot(d[1], d[2]))))
    return max(result)


class TaperEnvelopeTests(unittest.TestCase):
    def test_taper_table_and_existing_low_yaw_constraint(self):
        expected = (6, 5.2, 4.4, 3.6, 2.8, 2, 1, 0)
        for yaw, cap in zip(range(15, 23), expected):
            self.assertAlmostEqual(allowable_pitch_for_yaw(yaw, 8), cap, places=12)
        for yaw in (0, .1, 1, 2, 5, 10, 20):
            self.assertLessEqual(allowable_pitch_for_yaw(yaw, 8), max_pitch_for_yaw(yaw, 8))

    def test_exact_inverse_reachability_keeps_discriminant_margin_on_dense_envelope(self):
        peak = 0.
        for yi in range(2201):
            yaw = yi / 100
            cap = allowable_pitch_for_yaw(yaw, 8)
            for pitch in (-cap, 0., cap):
                inverse_side(yaw, pitch)
                self.assertGreaterEqual(min(inward_offsets_mm(yaw, pitch)), -1e-10)
                self.assertTrue(all(500 < p < 2500 for p in pose_to_pulses_us((yaw, pitch, yaw, -pitch))))
                peak = max(peak, max_abs_q(yaw, pitch))
        self.assertLess(peak, .95)  # Analytical command margin, not physical safety.

    def test_cap_is_concave_and_continuous_at_table_joins(self):
        caps = [allowable_pitch_for_yaw(i / 100, 8) for i in range(2201)]
        for a, b, c in zip(caps, caps[1:], caps[2:]):
            self.assertLessEqual(c - 2*b + a, 1e-10)
        h = .00001
        for yaw in (6, 15, 20):
            center = allowable_pitch_for_yaw(yaw, 8)
            self.assertAlmostEqual(center, allowable_pitch_for_yaw(yaw-h,8), delta=2*h)
            self.assertAlmostEqual(center, allowable_pitch_for_yaw(yaw+h,8), delta=2*h)

    def test_random_straight_paths_and_neutral_return_remain_inside_envelope(self):
        rng = random.Random(20261026)
        pairs = [((15., 6.), (22., 0.)), ((22., 0.), (0., 0.)),
                 ((21., -allowable_pitch_for_yaw(21)), (19., 2.8))]
        for _ in range(300):
            y0, y1 = rng.uniform(0, 22), rng.uniform(0, 22)
            p0 = rng.uniform(-1, 1) * allowable_pitch_for_yaw(y0)
            p1 = rng.uniform(-1, 1) * allowable_pitch_for_yaw(y1)
            pairs.append(((y0, p0), (y1, p1)))
        for (y0, p0), (y1, p1) in pairs:
            for step in range(101):
                t = step / 100
                yaw, pitch = y0 + t*(y1-y0), p0 + t*(p1-p0)
                self.assertLessEqual(abs(pitch), allowable_pitch_for_yaw(yaw) + 1e-10)
                inverse_side(yaw, pitch)

    def test_lower_configured_pitch_caps_do_not_weaken_default_envelope(self):
        for limit in (0, 1, 2, 4, 6):
            for yi in range(221):
                yaw = yi / 10
                cap = allowable_pitch_for_yaw(yaw, limit)
                self.assertLessEqual(cap, allowable_pitch_for_yaw(yaw) + 1e-10)
                self.assertLessEqual(cap, limit)
                inverse_side(yaw, cap)

    def test_invalid_envelope_angles_and_limits_fail_closed(self):
        for yaw, pitch_limit in ((True, 8), (-.01, 8), (22.01, 8), (math.nan, 8),
                                 (20, True), (20, 8.01), (20, -.01), (20, math.inf)):
            with self.assertRaises(ValueError):
                allowable_pitch_for_yaw(yaw, pitch_limit)


if __name__ == '__main__':
    unittest.main()
