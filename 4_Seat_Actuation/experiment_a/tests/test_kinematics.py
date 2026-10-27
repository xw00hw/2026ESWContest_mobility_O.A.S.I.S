import math
import unittest

from experiment_a.kinematics import (
    inverse_side, inward_offsets_mm, max_pitch_for_yaw, pose_to_pulses_us,
)


class KinematicsTests(unittest.TestCase):
    def test_neutral_matches_recorded_firmware_centers(self):
        self.assertEqual(inverse_side(0, 0), (0.0, 0.0))
        self.assertEqual(pose_to_pulses_us((0, 0, 0, 0)), (1800, 1800, 1200, 1200))

    def test_original_r8_reference_points(self):
        # Independent golden values published with original CAD derivation.
        upper, lower = inverse_side(5, 2)
        self.assertAlmostEqual(upper, 7.249115, places=5)
        self.assertAlmostEqual(lower, 15.712434, places=5)
        self.assertAlmostEqual(inverse_side(1, 0)[0], 2.372851, places=5)

    def test_corrected_270_degree_motor_gain(self):
        pulse = pose_to_pulses_us((1, 0, 1, 0))
        delta = 2.372851 * 2000 / 270
        for actual, expected in zip(pulse,(1800-delta,1800+delta,1200+delta,1200-delta)):
            self.assertAlmostEqual(actual,expected,places=4)

    def test_positive_pitch_reduces_upper_inward_projection(self):
        upper, lower = inward_offsets_mm(10, 4)
        self.assertLess(upper, lower)
        self.assertEqual(inward_offsets_mm(10, -4), (lower, upper))
        self.assertLess(inward_offsets_mm(0, 4)[0], 0)

    def test_channel_mapping_is_mirrored_without_side_cross_talk(self):
        l = pose_to_pulses_us((10, 2, 0, 0))
        r = pose_to_pulses_us((0, 0, 10, 2))
        self.assertEqual((l[0], l[2]), (1800, 1200))
        self.assertEqual((r[1], r[3]), (1800, 1200))
        self.assertAlmostEqual(l[1] + r[0], 3600)
        self.assertAlmostEqual(l[3] + r[2], 2400)

    def test_original_link_lengths_recovered_across_candidate_envelope(self):
        for yaw in range(21):
            for pitch in range(-8, 9):
                phi = inverse_side(yaw, pitch)
                cy, sy = math.cos(math.radians(-yaw)), math.sin(math.radians(-yaw))
                cp, sp = math.cos(math.radians(pitch)), math.sin(math.radians(pitch))
                for sign, angle in zip((1, -1), phi):
                    z = 48 * sign
                    anchor = (-55*cy - 12*sy*cp + z*sy*sp,
                              -55*sy + 12*cy*cp - z*cy*sp, 12*sp + z*cp)
                    ball = (-65, -36 + 24*math.sin(math.radians(angle)),
                            z + sign*24*math.cos(math.radians(angle)))
                    self.assertAlmostEqual(math.dist(anchor, ball)**2, 2980, places=9)
                self.assertTrue(all(500 < p < 2500 for p in pose_to_pulses_us((yaw, pitch, yaw, -pitch))))

    def test_exact_coupled_bound_forbids_neutral_retraction(self):
        for yi in range(201):
            yaw = yi / 10
            for cap in (2, 4, 8):
                limit = max_pitch_for_yaw(yaw, cap)
                self.assertGreaterEqual(limit, 0)
                self.assertLessEqual(limit, cap)
                self.assertGreaterEqual(min(inward_offsets_mm(yaw, limit)), -1e-12)
                self.assertGreaterEqual(min(inward_offsets_mm(yaw, -limit)), -1e-12)
                if limit < cap:
                    self.assertLess(min(inward_offsets_mm(yaw, limit + 1e-5)), 0)
        self.assertEqual(max_pitch_for_yaw(0, 8), 0)
        self.assertEqual(max_pitch_for_yaw(20, 8), 8)

    def test_nonfinite_bounds_and_nonphysical_combination_rejected(self):
        for yaw, pitch in ((math.nan, 0), (0, math.inf), (-.001, 0), (27.001, 0), (0, 8.001), (True, 0)):
            with self.assertRaises(ValueError):
                inverse_side(yaw, pitch)
        for cap in (-1, 8.001, math.nan, True):
            with self.assertRaises(ValueError):
                max_pitch_for_yaw(10, cap)
        with self.assertRaises(ValueError):
            pose_to_pulses_us((0, 0, 0))
        # Firmware rectangular hard limits still require nonlinear reachability.
        with self.assertRaises(ValueError):
            inverse_side(27, 8)


if __name__ == "__main__":
    unittest.main()
