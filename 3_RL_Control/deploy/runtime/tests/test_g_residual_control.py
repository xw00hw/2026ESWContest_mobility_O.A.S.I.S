"""Pure high-level G governor; no device/serial/network imports or operations."""
from pathlib import Path
import json
import math
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from oasis_rt.g_residual_control import GResidualConfig, GResidualGovernor


class GResidualControlTests(unittest.TestCase):
    def step(self, control=None, g=.4, predicted=.4, speed=10., raw=(0., 10.),
             prior=(0., 10.), dt=.05):
        return (control or GResidualGovernor()).step(g, predicted, speed, raw, prior, dt)

    def test_one_g_endpoint_overrides_opposed_prediction_and_negative_residual(self):
        for g, expected in ((1., (0., 20.)), (2., (0., 20.)),
                            (-1., (20., 0.)), (-2., (20., 0.))):
            governor = GResidualGovernor()
            self.step(governor, g=0., predicted=-g)
            self.step(governor, g=0., predicted=-g)
            out, snap = self.step(governor, g=g, predicted=-g,
                                  raw=(0., 0.), prior=(20., 20.))
            self.assertEqual(out, expected)
            self.assertEqual(snap['residual_limit_deg'], 0.)
            self.assertEqual(snap['direction_source'], 'measured')

    def test_monotone_base_has_explicit_noise_knee_and_full_endpoints(self):
        governor = GResidualGovernor()
        self.assertEqual(governor.base_yaw(.01), 0.)
        self.assertEqual(governor.base_yaw(.0625), 10.)
        self.assertEqual(governor.base_yaw(1.), 20.)
        curve = [governor.base_yaw(i/1000) for i in range(2001)]
        self.assertTrue(all(a <= b for a, b in zip(curve, curve[1:])))

    def test_preview_requires_stable_onset_and_total_preview_cannot_exceed_ten(self):
        control = GResidualGovernor()
        out, snap = self.step(control, g=0., predicted=.3, raw=(0., 20.))
        self.assertEqual(out, (0., 0.))
        out, snap = self.step(control, g=0., predicted=.3, raw=(0., 20.))
        self.assertEqual(out, (0., 10.))
        self.assertEqual(snap['direction_source'], 'preview')
        self.assertEqual(snap['residual_applied_deg'], 0.)

    def test_measured_sign_dominates_opposed_stable_preview(self):
        control = GResidualGovernor()
        for _ in range(2):
            self.step(control, g=0., predicted=.3)
        out, snap = self.step(control, g=-.04, predicted=.3,
                              raw=(0., 10.), prior=(0., 10.))
        self.assertGreater(out[0], 0.)
        self.assertEqual(out[1], 0.)
        self.assertTrue(snap['preview_opposed_to_measured'])
        self.assertEqual(snap['preview_yaw_deg'], 0.)

    def test_measured_hysteresis_releases_without_latching_wrong_side(self):
        control = GResidualGovernor()
        self.assertEqual(self.step(control, g=.019, predicted=0.)[0], (0., 0.))
        self.assertGreater(self.step(control, g=.021, predicted=0.)[0][1], 0.)
        self.assertGreater(self.step(control, g=.019, predicted=0.)[0][1], 0.)
        self.assertEqual(self.step(control, g=-.015, predicted=0.)[0], (0., 0.))
        self.assertGreater(self.step(control, g=-.021, predicted=0.,
                                     raw=(0., 0.), prior=(0., 0.))[0][0], 0.)
        self.assertEqual(self.step(control, g=-.01, predicted=0.)[0], (0., 0.))

    def test_new_preview_direction_reconfirms_and_zero_removes_old_preview(self):
        control = GResidualGovernor()
        for _ in range(2):
            self.step(control, g=0., predicted=.3)
        self.assertEqual(self.step(control, g=0., predicted=-.3)[0], (0., 0.))
        self.assertEqual(self.step(control, g=0., predicted=-.3,
                                   raw=(10., 0.), prior=(10., 0.))[0], (10., 0.))
        self.assertEqual(self.step(control, g=0., predicted=0.)[0], (0., 0.))

    def test_noise_below_preview_on_cannot_accumulate_confirmation(self):
        control = GResidualGovernor()
        for predicted in [.04, .02, .04, .02] * 3:
            self.assertEqual(self.step(control, g=0., predicted=predicted)[0], (0., 0.))

    def test_zero_speed_resets_all_direction_history_and_requests_zero(self):
        control = GResidualGovernor()
        for _ in range(2):
            self.step(control, g=.3, predicted=.4)
        out, snap = self.step(control, g=2., predicted=2., speed=0.)
        self.assertEqual(out, (0., 0.))
        self.assertEqual(snap['direction_source'], 'stopped')
        self.assertEqual(snap['preview_pending_s'], 0.)
        self.assertEqual(self.step(control, g=0., predicted=.4, speed=1.)[0], (0., 0.))

    def test_effective_residual_is_bounded_and_tapers_to_full_g(self):
        for g in (.04, .1, .4, .9):
            out, snap = self.step(g=g, predicted=0., raw=(0., 20.), prior=(0., 0.))
            expected = 1.5 * min(1., (1.-g)/(1.-.0625))
            self.assertAlmostEqual(snap['residual_applied_deg'], expected)
            self.assertAlmostEqual(out[1], snap['base_yaw_deg'] + expected)
            negative, ns = self.step(g=g, predicted=0., raw=(0., 0.), prior=(0., 20.))
            self.assertAlmostEqual(ns['residual_applied_deg'], -expected)
            self.assertLess(negative[1], out[1])

    def test_identical_actor_and_own_prior_have_zero_correction(self):
        out, snap = self.step(g=.4, predicted=0., raw=(3., 7.), prior=(3., 7.))
        self.assertEqual(snap['residual_raw_lr_deg'], [0., 0.])
        self.assertEqual(snap['residual_applied_deg'], 0.)
        self.assertEqual(out, (0., snap['base_yaw_deg']))

    def test_residual_cannot_initiate_motion_without_measured_or_preview_evidence(self):
        control = GResidualGovernor()
        for _ in range(10):
            out, snap = self.step(control, g=.001, predicted=.006,
                                  raw=(20., 20.), prior=(0., 0.))
            self.assertEqual(out, (0., 0.))

    def test_output_one_sided_and_bounded_across_direction_changes(self):
        control = GResidualGovernor()
        for i in range(401):
            g = 1.5 * math.sin(i * .1)
            predicted = 2 * math.cos(i * .13)
            out, snap = self.step(control, g=g, predicted=predicted,
                                  raw=(20., 20.), prior=(0., 0.))
            self.assertTrue(all(0 <= a <= 20 for a in out))
            self.assertEqual(out[0] * out[1], 0.)
            self.assertLessEqual(sum(out), 20.)
            json.dumps(snap, allow_nan=False)

    def test_explicit_reset_for_stale_recovery_clears_confirmation(self):
        control = GResidualGovernor()
        for _ in range(2):
            self.step(control, g=0., predicted=.3)
        control.reset()
        self.assertEqual(self.step(control, g=0., predicted=.3)[0], (0., 0.))

    def test_invalid_input_is_rejected_before_mutating_direction_state(self):
        control = GResidualGovernor()
        self.step(control, g=.2)
        for override in ({'g': float('nan')}, {'predicted': float('inf')},
                         {'speed': '1'}, {'dt': 0}, {'dt': .6},
                         {'raw': (0., float('nan'))}, {'raw': (0., -1)},
                         {'prior': (0., 20.01)}, {'prior': (1.,)}, {'g': True}):
            with self.subTest(override=override):
                with self.assertRaises(ValueError):
                    self.step(control, **override)
                self.assertEqual(control.measured_side, 1)

    def test_config_is_finite_consistent_and_unknown_fields_rejected(self):
        for values in ({'full_g': float('nan')}, {'knee_g': 1},
                       {'measured_on_g': .01}, {'full_yaw_deg': 21},
                       {'preview_off_g': .04}, {'preview_max_yaw_deg': 11},
                       {'residual_max_deg': -1}, {'speed_min_mps': -1}, {'speed_min_mps': 0},
                       {'preview_confirm_s': 0}, {'enabled': True}):
            with self.subTest(values=values):
                with self.assertRaises(ValueError):
                    GResidualGovernor(values)
        config = GResidualConfig()
        self.assertEqual(GResidualGovernor(config.to_dict()).config, config)


if __name__ == '__main__':
    unittest.main()
