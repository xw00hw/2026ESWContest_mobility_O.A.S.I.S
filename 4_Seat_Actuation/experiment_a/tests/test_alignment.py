import math
import unittest

from experiment_a.alignment import alignment_error_deg, closest_face_pitch_deg


class AlignmentTests(unittest.TestCase):
    def test_zero_yaw_preserves_torso_angle(self):
        for theta in (-30., -8., 0., 8., 30.):
            self.assertAlmostEqual(closest_face_pitch_deg(0., theta), theta, places=12)
            self.assertAlmostEqual(alignment_error_deg(0., theta, theta), 0., places=6)

    def test_zero_roll_and_nonzero_yaw_has_residual(self):
        self.assertEqual(closest_face_pitch_deg(20., 0.), 0.)
        self.assertAlmostEqual(alignment_error_deg(20., 0., 0.), 20., places=12)

    def test_projection_compensation_and_signed_symmetry(self):
        pitch = closest_face_pitch_deg(20., 5.)
        self.assertAlmostEqual(pitch, 5.319103338346746, places=10)
        self.assertAlmostEqual(closest_face_pitch_deg(20., -5.), -pitch, places=12)
        self.assertAlmostEqual(closest_face_pitch_deg(-20., 5.), pitch, places=12)
        self.assertGreater(alignment_error_deg(20., pitch, 5.), 0.)

    def test_analytic_optimum_dominates_dense_grid(self):
        for yaw in (0., 10., 20., 27.):
            for theta in (-20., -5., 0., 5., 20.):
                with self.subTest(yaw=yaw, theta=theta):
                    ideal = closest_face_pitch_deg(yaw, theta)
                    optimum_error = alignment_error_deg(yaw, ideal, theta)
                    brute_error = min(alignment_error_deg(yaw, i/10., theta) for i in range(-890,891))
                    self.assertLessEqual(optimum_error, brute_error+1e-7)

    def test_upper_axis_projection_matches_target_but_full_axis_need_not(self):
        yaw, theta = 20., 5.
        pitch = closest_face_pitch_deg(yaw, theta)
        psi, beta = map(math.radians, (yaw,pitch))
        upper_axis=(-math.sin(psi)*math.sin(beta),-math.cos(psi)*math.sin(beta),math.cos(beta))
        projected = math.degrees(math.atan2(-upper_axis[1],upper_axis[2]))
        self.assertAlmostEqual(projected, theta, places=12)
        self.assertNotEqual(upper_axis[0],0.)

    def test_clipped_pitch_increases_alignment_residual(self):
        ideal = closest_face_pitch_deg(20., 10.)
        self.assertGreater(ideal,8.)
        self.assertGreater(alignment_error_deg(20.,4.,10.),alignment_error_deg(20.,ideal,10.))

    def test_invalid_and_degenerate_inputs_rejected(self):
        for invalid in (float('nan'),float('inf'),True,'5',90.,-90.):
            with self.subTest(invalid=invalid):
                with self.assertRaises(ValueError):closest_face_pitch_deg(invalid,0.)
                with self.assertRaises(ValueError):closest_face_pitch_deg(0.,invalid)
                with self.assertRaises(ValueError):alignment_error_deg(invalid,0.,0.)
                with self.assertRaises(ValueError):alignment_error_deg(0.,invalid,0.)
                with self.assertRaises(ValueError):alignment_error_deg(0.,0.,invalid)


if __name__=='__main__':unittest.main()
