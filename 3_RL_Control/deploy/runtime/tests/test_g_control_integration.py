"""Offline Controller integration against the actual exported model/obs contract.

Set OASIS_TEST_MODEL_DIR to model_v8. No UART, camera, network or hardware opens.
The link double uses the real CAD/STM/IK Envelope and captures queued targets;
its reported state is deliberately independent of those target requests.
"""
import copy
import os
from pathlib import Path
import sys
import unittest

import numpy as np

RT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(RT), str(RT/'tests')]
from oasis_rt import controller as C, sources as SRC, fbstack  # noqa: E402
from oasis_rt.actor import OutputFilter  # noqa: E402
from oasis_rt.envelope import Envelope  # noqa: E402
from mock_stm32 import MockClock  # noqa: E402


class Journal:
    def __init__(self, fail=False):
        self.events, self.fail = [], fail

    def record(self, kind, **fields):
        if self.fail:
            raise RuntimeError('injected_bounded_journal_failure')
        self.events.append((kind, fields))


class Link:
    def __init__(self, clock, fb, cap=20., journal=None):
        self.clock, self.pressure_log = clock, journal
        self.env = Envelope(fb, dict(yaw_max_mdeg=int(cap*1000), pitch_max_mdeg=6000))
        self.pressure = {f'A{bank}_C{channel}': 1000 for bank in range(3) for channel in range(4)}
        self.pressure_at = clock()
        self.fault, self.target = None, (0., 0., 0., 0.)
        self.report = (1., 0., 2., 0.)
        self.authorizations = self.holds = 0

    def poll(self):
        self.pressure_at = self.clock()

    def reported(self):
        return self.report

    def send_targets(self, yaw_L, pitch_L, yaw_R, pitch_R):
        pose, _ = self.env.project(yaw_L, pitch_L, yaw_R, pitch_R)
        self.target = tuple(value/1000 for value in pose)
        return self.target

    def authorize(self):
        self.authorizations += 1

    def hold(self):
        self.holds += 1
        self.target = (0., 0., 0., 0.)


class Vehicle:
    def __init__(self, clock):
        self.clock, self.g, self.pred_g, self.speed, self.available = clock, 1., 1., 10., True

    def latest(self):
        if not self.available:
            return None, 1.
        return dict(t=self.clock(), ay_mps2=self.g*C.G, ax_mps2=0.,
                    speed_mps=self.speed, yaw_rate_radps=0., steer_rad=0.), 0.

    def predict(self, _vehicle):
        return np.full(10, self.pred_g), np.zeros(10)


class TestGControlIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        model = os.environ.get('OASIS_TEST_MODEL_DIR') or os.environ.get('OASIS_TEST_MODEL_V3')
        if not model or not (Path(model)/'manifest.json').is_file():
            raise unittest.SkipTest('requires exported model_v8 via OASIS_TEST_MODEL_DIR')
        cls.cfg = C.load_config(str(RT/'config.yaml'))
        cls.cfg['model_dir'] = str(Path(model).resolve())
        cls.cfg['actor']['check_set'] = str(Path(model).resolve()/'check_set.npy')
        cls.cfg['actor']['backends'] = ['numpy']
        cls.cfg['control'].update(mode='rl', rule={})
        cls.cfg['g_residual_control'] = {'enabled': True}
        cls.cfg['_display_source'] = 'simulation'
        cls.man, cls.obs, cls.actor = C.prepare_policy(cls.cfg, log=lambda *_: None)
        if cls.man.get('action_dim') != 2 or cls.man.get('obs_version') != 'oasis-obs-v3.0':
            raise unittest.SkipTest('integration contract is the v8 yaw-only/v3 model')
        cls.fb = fbstack.load(cls.cfg.get('fb_root'))

    def build(self, *, enabled=True, missing=False, cap=20., journal=None, cfg=None):
        config = copy.deepcopy(cfg or self.cfg)
        if missing:
            config.pop('g_residual_control', None)
        else:
            config['g_residual_control'] = {'enabled': enabled}
        clock = MockClock()
        link = Link(clock, self.fb, cap, journal)
        vehicle = Vehicle(clock)
        ctl = C.Controller(config, link, self.actor, self.obs, vehicle.predict,
                           vehicle, SRC.NoPose(), clock, log=lambda *_: None)
        # Isolate policy integration from the independently tested baseline
        # acquisition procedure; current pressures equal the known rest baseline.
        ctl.base = dict(link.pressure)
        return ctl, link, vehicle, clock

    @staticmethod
    def tick(ctl, clock):
        clock.t += ctl.dt
        return ctl.tick()

    def test_zero_initialized_single_filter_and_one_g_steady_endpoint(self):
        ctl, link, _, clock = self.build()
        self.tick(ctl, clock)
        self.assertEqual(ctl.g_snapshot['requested_yaw_lr_deg'], [0., 20.])
        self.assertAlmostEqual(ctl.g_snapshot['filtered_yaw_lr_deg'][1], 20*ctl.dt/.3, places=5)
        self.assertLess(link.target[2], 4.)
        self.assertIsNone(ctl.filt.y)  # The old action filter is not applied a second time.
        for _ in range(100):
            self.tick(ctl, clock)
        self.assertEqual(ctl.g_snapshot['filtered_yaw_lr_deg'], [0., 20.])
        self.assertEqual(link.target, (0., 0., 20., 0.))
        self.assertEqual(ctl.g_snapshot['residual_applied_deg'], 0.)

    def test_reversal_has_no_bilateral_closing_targets(self):
        ctl, link, vehicle, clock = self.build()
        for direction in (1., -1., 1.):
            vehicle.g = vehicle.pred_g = direction
            for _ in range(90):
                self.tick(ctl, clock)
                left, right = ctl.g_snapshot['filtered_yaw_lr_deg']
                self.assertEqual(min(left, right), 0.)
                self.assertLessEqual(left+right, 20.)
                self.assertEqual(min(link.target[0], link.target[2]), 0.)
            self.assertEqual(link.target[0 if direction < 0 else 2], 20.)

    def test_next_observation_uses_hybrid_action_and_reported_command_state(self):
        ctl, link, _, clock = self.build()
        self.tick(ctl, clock)
        previous = ctl.a_prev.copy()
        mapped = self.obs.action_to_targets(previous, 20.)
        np.testing.assert_allclose(mapped[:2], ctl.g_snapshot['filtered_yaw_lr_deg'], atol=1e-6)
        self.tick(ctl, clock)
        np.testing.assert_allclose(ctl.row[1][43:45], previous, atol=1e-7)
        np.testing.assert_allclose(ctl.row[1][39:41], [link.report[0]/20, link.report[2]/20], atol=1e-7)
        self.assertEqual(ctl.row[6], link.report)
        self.assertFalse(np.allclose(ctl.row[2], ctl.row[3]))  # raw actor is separately logged.

    def test_training_prior_is_captured_before_fallback_rule_override(self):
        cfg = copy.deepcopy(self.cfg)
        cfg['control']['rule'] = {'gain': 1., 'both': .4, 'deadband_g': .01}
        man, _, _ = C.prepare_policy(cfg, log=lambda *_: None)
        history = np.zeros((man['K']+1)*man['OD'], np.float32)
        history[-man['OD']+man['ff_idx']] = .1
        expected = np.array([-1., -1.], np.float32)  # v8 prior gain16, both0, deadband0 at +0.1g.
        self.assertEqual(man['ff_gain'], 16.)
        np.testing.assert_array_equal(cfg['_training_prior'](history), expected)
        self.assertFalse(np.allclose(cfg['_rule'](history), expected))

    def test_disabled_and_absent_config_preserve_legacy_action_filter(self):
        a, la, va, ca = self.build(enabled=False)
        b, lb, vb, cb = self.build(missing=True)
        expected = OutputFilter(a.cfg['filter_tau_s'], a.dt)
        for g in (0., .08, .3, -.1, -1., .02):
            va.g = va.pred_g = vb.g = vb.pred_g = g
            self.tick(a, ca); self.tick(b, cb)
            self.assertIsNone(a.g_governor)
            self.assertIsNone(b.g_governor)
            np.testing.assert_allclose(a.row[3], expected(a.row[2]), atol=1e-7)
            np.testing.assert_array_equal(a.row[2], b.row[2])
            np.testing.assert_array_equal(a.row[3], b.row[3])
            self.assertEqual(la.target, lb.target)

    def test_stale_vehicle_resets_direction_filter_and_previous_action(self):
        ctl, link, vehicle, clock = self.build()
        for _ in range(60):
            self.tick(ctl, clock)
        vehicle.available = False
        self.assertIsNone(self.tick(ctl, clock))
        self.assertEqual(ctl.state, 'STALE')
        self.assertEqual(link.target, (0., 0., 0., 0.))
        self.assertIsNone(ctl.g_snapshot)
        self.assertEqual(ctl.g_governor.measured_side, 0)
        np.testing.assert_array_equal(ctl.a_prev, [-1., 0.])
        vehicle.available = True
        self.tick(ctl, clock)
        self.assertAlmostEqual(ctl.g_snapshot['filtered_yaw_lr_deg'][1], 20*ctl.dt/.3, places=5)

    def test_telemetry_separates_request_filter_envelope_and_board_state(self):
        journal = Journal()
        ctl, link, _, clock = self.build(cap=15., journal=journal)
        for _ in range(100):
            self.tick(ctl, clock)
        kind, event = journal.events[-1]
        self.assertEqual(kind, 'g_control_tick')
        self.assertEqual(event['requested_yaw_deg'], {'left': 0., 'right': 20.})
        self.assertEqual(event['filtered_yaw_deg'], {'left': 0., 'right': 20.})
        self.assertEqual(event['final_sent_yaw_deg'], {'left': 0., 'right': 15.})
        self.assertEqual(event['board_command_yaw_deg'], {'left': 1., 'right': 2.})
        self.assertEqual(event['final_sent_semantics'], 'queued_after_limits_not_wire_ack')
        self.assertIn('downstream_limit_or_release', event['guards'])
        self.assertEqual(event['pressure_counts'], dict(UR=1000, UL=1000, DR=1000, DL=1000))

    def test_body_scale_from_camera_neutral_scales_the_one_g_endpoint(self):
        import json, tempfile
        from oasis_rt import body_scale as B
        ref = B.REFERENCE; h, w = 800, 1280
        neutral = dict(schema=B.NEUTRAL_SCHEMA, operator_confirmed=True, frame_hw=[h, w], roi_xyxy=[0, 0, w, h],
                       model_sha256='0'*64, shoulder_center_px=[400., 250.], hip_center_px=[420., 250.+ref['torso_height_frac']*h],
                       shoulder_width_px=ref['shoulder_width_frac']*w*1.1, torso_angle_deg=0.)
        with tempfile.TemporaryDirectory() as d:
            path = Path(d)/'neutral.json'; path.write_text(json.dumps(neutral), encoding='utf-8')
            cfg = copy.deepcopy(self.cfg); cfg['body_scale'] = {'enabled': True, 'neutral': str(path)}
            ctl, link, _, clock = self.build(cfg=cfg)
        scale = B.scale_from_ratios(1.1, 1.)
        self.assertEqual((ctl.body_scale['scale'], ctl.g_governor.body_scale), (scale, scale))
        self.assertLess(scale, 1.)                      # wider than the reference user: slightly less closing
        for _ in range(120):
            self.tick(ctl, clock)
        self.assertAlmostEqual(link.target[2], 20.*scale, delta=.002)   # envelope rounds inward to integer mdeg
        self.assertEqual(ctl.g_snapshot['body_scale'], scale)
        cfg = copy.deepcopy(self.cfg); cfg['body_scale'] = {'enabled': True, 'neutral': str(path)}
        with self.assertRaises(ValueError):            # enabled with no neutral file -> refuse, never a silent 1.0
            self.build(cfg=cfg)
        with self.assertRaisesRegex(ValueError, 'g_residual_control'):
            self.build(enabled=False, cfg=cfg)

    def test_logging_failure_propagates_before_authorization_and_run_holds(self):
        ctl, link, _, clock = self.build(journal=Journal(fail=True))
        with self.assertRaisesRegex(RuntimeError, 'injected_bounded_journal_failure'):
            ctl.run(.1, sleep=lambda dt: setattr(clock, 't', clock.t+dt))
        self.assertEqual(link.authorizations, 0)
        self.assertEqual(link.holds, 1)
        self.assertEqual(link.target, (0., 0., 0., 0.))


if __name__ == '__main__':
    unittest.main(verbosity=2)
