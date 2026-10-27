"""Analytic checks of body-independent dynamics, separate from controller code."""
import math
import unittest
from experiment_a.vehicle_response import GRAVITY_MPS2, VehicleResponse


class VehicleResponseTests(unittest.TestCase):
    def test_closed_form_filter_and_static_vector_angle(self):
        model = VehicleResponse(.15)
        for _ in range(10):
            state = model.step(1, .3*GRAVITY_MPS2, .02)
        expected = .3*GRAVITY_MPS2*(1-math.exp(-.2/.15))
        self.assertAlmostEqual(state.filtered_ay_mps2, expected, places=12)
        self.assertAlmostEqual(state.roll_deg, math.degrees(math.atan(expected/GRAVITY_MPS2)), places=12)
        for _ in range(200):
            state = model.step(1, .3*GRAVITY_MPS2, .02)
        self.assertAlmostEqual(state.roll_deg, math.degrees(math.atan(.3)), places=9)
        self.assertLess(state.pitch_deg, 0)

    def test_time_partition_invariance_symmetry_and_reset(self):
        a,b = VehicleResponse(.15),VehicleResponse(.15)
        for _ in range(20): x=a.step(0,2,.01)
        for _ in range(10): y=b.step(0,-2,.02)
        self.assertAlmostEqual(x.roll_deg,-y.roll_deg,places=12)
        self.assertEqual(x.preview_roll_deg,x.roll_deg)
        self.assertEqual(a.reset().roll_deg,0)

    def test_nonfinite_outlier_and_gap_rejected_before_mutation(self):
        model=VehicleResponse(.15)
        before=model.state
        for values in ((0,math.nan,.02),(0,201,.02),(0,0,.2),(True,0,.02)):
            with self.assertRaises(ValueError):model.step(*values)
            self.assertEqual(model.state,before)

    def test_high_valid_acceleration_remains_finite(self):
        state=VehicleResponse(.15).step(20., 30., .02)
        self.assertTrue(math.isfinite(state.roll_deg))
        self.assertGreater(state.filtered_ay_mps2, 2.942)
