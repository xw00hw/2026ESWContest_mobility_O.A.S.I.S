"""Analytic and invariance checks of the unobserved human surrogate."""
from dataclasses import replace
import math
import unittest

from experiment_a.human_model import HumanModel, HumanModelConfig, nominal_design_config


class HumanModelTests(unittest.TestCase):
    def setUp(self):
        self.config = nominal_design_config()

    def settle(self, model, ax=0., ay=0., seconds=10., dt=.02):
        for _ in range(round(seconds/dt)):
            result = model.step(ax, ay, dt)
        return result

    def test_neutral_and_reset(self):
        model = HumanModel(self.config)
        neutral = self.settle(model)
        self.assertEqual(neutral.roll_deg, 0.)
        self.assertEqual(neutral.pitch_deg, 0.)
        self.settle(model, ax=-2., ay=2.)
        self.assertEqual(model.reset(), neutral)

    def test_signs_and_independent_axes(self):
        right = self.settle(HumanModel(self.config), ay=2.)
        left = self.settle(HumanModel(self.config), ay=-2.)
        acceleration = self.settle(HumanModel(self.config), ax=2.)
        braking = self.settle(HumanModel(self.config), ax=-2.)
        self.assertGreater(right.roll_deg, 0.)
        self.assertEqual(right.pitch_deg, 0.)
        self.assertAlmostEqual(right.roll_deg, -left.roll_deg, places=12)
        self.assertLess(acceleration.pitch_deg, 0.)
        self.assertEqual(acceleration.roll_deg, 0.)
        self.assertAlmostEqual(acceleration.pitch_deg, -braking.pitch_deg, places=12)

    def test_static_gain_matches_equation(self):
        state = self.settle(HumanModel(self.config), ax=-2., ay=2.)
        parameters = self.config.effective_parameters()
        self.assertAlmostEqual(state.roll_deg, 2 * parameters["roll_static_gain_deg_per_mps2"], places=8)
        self.assertAlmostEqual(state.pitch_deg, 2 * parameters["pitch_static_gain_deg_per_mps2"], places=8)

    def test_reference_anthropometry_matches_primary_table_reduction(self):
        parameters = self.config.effective_parameters()
        self.assertAlmostEqual(parameters["upper_body_mass_kg"], 35.16, places=10)
        self.assertAlmostEqual(parameters["com_distance_m"], .28729368031854385, places=12)
        self.assertAlmostEqual(parameters["roll_inertia_kg_m2"], 5.447788137970001, places=10)
        self.assertAlmostEqual(parameters["pitch_inertia_kg_m2"], 5.310393044243201, places=10)

    def test_mass_cancels_when_stiffness_and_damping_scale_with_inertia(self):
        # Explicit invariant: assigning a heavier body does not manufacture a
        # larger angle when its selected modal frequency remains unchanged.
        larger = replace(self.config, body_mass_kg=110.)
        a = self.settle(HumanModel(self.config), ay=2., seconds=.6)
        b = self.settle(HumanModel(larger), ay=2., seconds=.6)
        self.assertAlmostEqual(a.roll_deg, b.roll_deg, places=12)
        self.assertGreater(larger.effective_parameters()["roll_net_stiffness_nm_per_rad"],
                           self.config.effective_parameters()["roll_net_stiffness_nm_per_rad"])

    def test_height_and_recline_change_derived_gain_transparently(self):
        tall = replace(self.config, body_height_m=2.)
        reclined = replace(self.config, seat_recline_deg=30.)
        gain = self.config.effective_parameters()["roll_static_gain_deg_per_mps2"]
        self.assertAlmostEqual(tall.effective_parameters()["roll_static_gain_deg_per_mps2"] / gain,
                               self.config.body_height_m / 2., places=12)
        self.assertAlmostEqual(reclined.effective_parameters()["roll_static_gain_deg_per_mps2"] / gain,
                               math.cos(math.radians(30)), places=12)

    def test_time_partition_agrees_at_valid_steps(self):
        states = [self.settle(HumanModel(self.config), ax=-1., ay=1., seconds=.4, dt=dt)
                  for dt in (.05, .02, .01, .005)]
        for state in states[1:]:
            self.assertAlmostEqual(state.roll_deg, states[0].roll_deg, places=9)
            self.assertAlmostEqual(state.pitch_deg, states[0].pitch_deg, places=9)
            self.assertAlmostEqual(state.filtered_ay_mps2, states[0].filtered_ay_mps2, places=12)

    def test_causal_filter_matches_closed_form(self):
        state = HumanModel(self.config).step(1., 2., .02)
        fraction = 1 - math.exp(-.02 / self.config.acceleration_filter_tau_s)
        self.assertAlmostEqual(state.filtered_ax_mps2, fraction, places=13)
        self.assertAlmostEqual(state.filtered_ay_mps2, 2*fraction, places=13)
        self.assertGreater(state.roll_deg, 0.)

    def test_preview_is_bounded_causal_rate_projection_and_does_not_change_dynamics(self):
        cfg = replace(self.config, preview_horizon_s=.15)
        plain, preview = HumanModel(self.config), HumanModel(cfg)
        a = plain.step(0., 2., .02)
        b = preview.step(0., 2., .02)
        self.assertEqual(a.roll_deg, b.roll_deg)
        self.assertEqual(a.roll_rate_deg_s, b.roll_rate_deg_s)
        self.assertEqual(a.roll_deg, a.preview_roll_deg)
        self.assertAlmostEqual(b.preview_roll_deg, b.roll_deg + .15*b.roll_rate_deg_s, places=12)
        self.assertGreater(b.preview_roll_deg, b.roll_deg)

    def test_damped_response_returns_to_neutral(self):
        model = HumanModel(replace(self.config, roll_damping_ratio=.3, pitch_damping_ratio=1.8))
        self.settle(model, ax=2., ay=2., seconds=.2)
        result = self.settle(model, seconds=15.)
        self.assertAlmostEqual(result.roll_deg, 0., places=8)
        self.assertAlmostEqual(result.pitch_deg, 0., places=8)

    def test_extreme_valid_configuration_stays_finite_and_bounded(self):
        cfg = replace(self.config, roll_natural_frequency_hz=5., pitch_natural_frequency_hz=.1,
                      roll_damping_ratio=2., pitch_damping_ratio=.1,
                      preview_horizon_s=.25, max_angle_deg=5., max_rate_deg_s=10.)
        model = HumanModel(cfg)
        was_limited = False
        for i in range(1000):
            state = model.step(15. if i < 500 else -15., 12. if i < 500 else -12., .05)
            was_limited |= state.limited
            for value in (state.roll_deg, state.pitch_deg, state.preview_roll_deg, state.preview_pitch_deg):
                self.assertTrue(math.isfinite(value))
                self.assertLessEqual(abs(value), cfg.max_angle_deg + 1e-12)
            self.assertLessEqual(abs(state.roll_rate_deg_s), cfg.max_rate_deg_s + 1e-12)
            self.assertLessEqual(abs(state.pitch_rate_deg_s), cfg.max_rate_deg_s + 1e-12)
        self.assertTrue(was_limited)

    def test_invalid_input_does_not_mutate_state(self):
        for values in ((float("nan"), 0., .02), (0., float("inf"), .02), (0., 0., 0.),
                       (0., 0., .051), (16., 0., .02), (0., -13., .02), (True, 0., .02)):
            with self.subTest(values=values):
                model, reference = HumanModel(self.config), HumanModel(self.config)
                model.step(1., 1., .02)
                reference.step(1., 1., .02)
                with self.assertRaises(ValueError):
                    model.step(*values)
                self.assertEqual(model.step(1., 1., .02), reference.step(1., 1., .02))

    def test_invalid_configuration_rejected(self):
        for field, value in (("body_mass_kg", 0.), ("body_height_m", float("nan")),
                             ("seat_recline_deg", 90.), ("roll_natural_frequency_hz", 0.),
                             ("pitch_damping_ratio", -1.), ("preview_horizon_s", 1.),
                             ("max_rate_deg_s", True)):
            with self.subTest(field=field), self.assertRaises(ValueError):
                replace(self.config, **{field: value})
        with self.assertRaises(TypeError):
            HumanModel({})
        with self.assertRaises(TypeError):
            HumanModelConfig()


if __name__ == "__main__":
    unittest.main()
