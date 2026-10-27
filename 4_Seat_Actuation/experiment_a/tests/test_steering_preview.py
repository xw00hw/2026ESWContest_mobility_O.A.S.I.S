"""Advance-cue state/geometry tests; no hardware or occupant-force claims."""
from dataclasses import replace
import math
from types import SimpleNamespace
import unittest

from experiment_a.controller import ExperimentAController
from experiment_a.kinematics import pose_to_pulses_us
from experiment_a.cad_limits import pose_within_cad
from experiment_a.settings import Settings
from experiment_a.steering_preview import SteeringPreview
from experiment_a.vehicle import VehicleFrame
from fallback.serial_link import BoardState


def cue_vehicle(**changes):
    value = dict(speed_mps=15., steering_deg=45., gear_index=2., roll_rad=0.,
                 pitch_rad=0., brake=0., ax_mps2=0., reason='ok', throttle=0.)
    value.update(changes)
    return SimpleNamespace(**value)


def qualify(cue, vehicle=None, *, raw_ay=0., filtered_ay=0., primary_active=False):
    result = None
    for _ in range(7):
        result = cue.step(vehicle or cue_vehicle(), .02, raw_ay_mps2=raw_ay,
                          filtered_ay_mps2=filtered_ay, primary_active=primary_active)
    return result


class SteeringPreviewTests(unittest.TestCase):
    def test_bounded_advance_requires_dwell_and_has_no_pitch(self):
        cue = SteeringPreview(Settings())
        for _ in range(5):
            state = cue.step(cue_vehicle(), .02, raw_ay_mps2=0., filtered_ay_mps2=0.)
            self.assertFalse(state.active)
        state = qualify(cue)
        self.assertTrue(state.active)
        self.assertEqual((state.side, state.yaw_deg, state.pitch_deg), ('right', 2., 0.))
        self.assertEqual(qualify(SteeringPreview(Settings()), cue_vehicle(steering_deg=-45.)).side, 'left')
        self.assertEqual(qualify(SteeringPreview(Settings(steering_to_ay_sign=-1))).side, 'left')

    def test_straight_longitudinal_and_pedal_inputs_cannot_create_side_support(self):
        for steer in (-5., 0., 5.):
            result = qualify(SteeringPreview(Settings()), cue_vehicle(steering_deg=steer, ax_mps2=10., throttle=1.))
            self.assertFalse(result.active)
            self.assertEqual(result.yaw_deg, 0.)

    def test_optional_domain_forward_speed_and_attitude_guards(self):
        cases = [{'steering_deg': None}, {'steering_deg': float('nan')}, {'steering_deg': 560.001},
                 {'brake': None}, {'brake': -.001}, {'brake': 1.001}, {'brake': True},
                 {'gear_index': None}, {'gear_index': -1}, {'gear_index': 0}, {'gear_index': 1.5},
                 {'speed_mps': 4.999}, {'roll_rad': None}, {'pitch_rad': None},
                 {'roll_rad': math.radians(20)}, {'pitch_rad': math.radians(-20)}]
        for changes in cases:
            with self.subTest(changes=changes):
                state = qualify(SteeringPreview(Settings()), cue_vehicle(**changes))
                self.assertFalse(state.active)
                self.assertEqual(state.yaw_deg, 0.)

    def test_brake_attenuates_and_linear_acceleration_changes_bounded_speed_projection(self):
        base = qualify(SteeringPreview(Settings()), cue_vehicle(speed_mps=10.))
        braking = qualify(SteeringPreview(Settings()), cue_vehicle(speed_mps=10., brake=1.))
        speeding = qualify(SteeringPreview(Settings()), cue_vehicle(speed_mps=10., ax_mps2=15.))
        slowing = qualify(SteeringPreview(Settings()), cue_vehicle(speed_mps=10., ax_mps2=-15.))
        self.assertAlmostEqual(braking.yaw_deg, .5*base.yaw_deg)
        self.assertGreater(speeding.yaw_deg, base.yaw_deg)
        self.assertLess(slowing.yaw_deg, base.yaw_deg)
        self.assertEqual((slowing.projected_speed_mps, speeding.projected_speed_mps), (8., 11.))
        throttle = qualify(SteeringPreview(Settings()), cue_vehicle(speed_mps=10., throttle=1.))
        self.assertEqual(throttle, base)

    def test_primary_raw_or_filtered_ay_has_priority_and_opposite_small_ay_blocks(self):
        for raw, filtered, primary in ((.3, 0., False), (0., .3, False),
                                       (0., 0., True), (-.2, 0., False), (0., -.2, False)):
            with self.subTest(raw=raw, filtered=filtered, primary=primary):
                state = qualify(SteeringPreview(Settings()), raw_ay=raw, filtered_ay=filtered,
                                primary_active=primary)
                self.assertFalse(state.active)
        self.assertTrue(qualify(SteeringPreview(Settings()), raw_ay=.1, filtered_ay=.1).active)

    def test_fixed_episode_expires_and_requires_recentering_or_stopping(self):
        cue = SteeringPreview(Settings())
        self.assertTrue(qualify(cue).active)
        for _ in range(100):
            state = cue.step(cue_vehicle(), .02, raw_ay_mps2=0., filtered_ay_mps2=0.)
        self.assertFalse(state.active)
        self.assertEqual(state.reason, 'episode_expired_recenter_required')
        self.assertFalse(qualify(cue, cue_vehicle(steering_deg=-45.)).active)
        cue.step(cue_vehicle(steering_deg=0.), .02, raw_ay_mps2=0., filtered_ay_mps2=0.)
        self.assertTrue(qualify(cue).active)
        cue.step(cue_vehicle(reason='stopped'), .02, raw_ay_mps2=0., filtered_ay_mps2=0.)
        self.assertTrue(qualify(cue).active)

    def test_temporary_missing_input_does_not_renew_episode(self):
        cue = SteeringPreview(Settings())
        qualify(cue)
        for _ in range(100):
            cue.step(cue_vehicle(brake=None), .02, raw_ay_mps2=0., filtered_ay_mps2=0.)
        self.assertFalse(qualify(cue).active)

    def test_primary_turn_cannot_create_late_false_preview_when_acceleration_disappears(self):
        cue = SteeringPreview(Settings())
        for _ in range(100):
            cue.step(cue_vehicle(), .02, raw_ay_mps2=2., filtered_ay_mps2=2., primary_active=True)
        self.assertFalse(qualify(cue).active)

    def test_current_yaw_max_and_missing_side_are_respected(self):
        state = qualify(SteeringPreview(Settings(max_yaw_deg=.5)))
        self.assertEqual(state.yaw_deg, .5)
        state = qualify(SteeringPreview(Settings(available_bolsters='right_only')), cue_vehicle(steering_deg=-45.))
        self.assertFalse(state.active)

    def test_invalid_time_rejected(self):
        for dt in (-.01, .201, float('nan'), True):
            with self.subTest(dt=dt), self.assertRaises(ValueError):
                SteeringPreview(Settings()).step(cue_vehicle(), dt, raw_ay_mps2=0., filtered_ay_mps2=0.)


class PreviewLoop:
    def __init__(self, settings=None):
        self.controller = ExperimentAController(settings or Settings())
        self.now = 1_000_000_000
        self.sequence = 0
        self.pose = (0, 0, 0, 0)
        self.sim_time = 0.

    def step(self, ay=0., *, steer=45., sim_dt=.02, override_board=None, **fields):
        self.now += 20_000_000
        self.sequence += 1
        self.sim_time += sim_dt
        frame = VehicleFrame(self.now, self.sim_time, 'preview-test', 'car', self.sequence,
            ay, 0., 15., steer, 0., 0., True, 'ok', 'beamng',
            roll_rad=0., pitch_rad=0., gear_index=2.)
        frame = replace(frame, **fields)
        pulse = tuple(round(p) for p in pose_to_pulses_us([v/1000 for v in self.pose]))
        board = BoardState(self.now, True, False, self.pose, self.pose, pulse, self.sequence, 'NONE')
        if override_board:
            board = replace(board, **override_board)
        before = tuple(self.controller.pose)
        result = self.controller.tick(frame, board, self.now)
        if result.mode == 'RUN':
            self.pose = result.pose_mdeg
        return result, before


class PreviewControllerTests(unittest.TestCase):
    def check_limits(self, loop, result, before):
        self.assertEqual(result.mode, 'RUN', result.reason)
        self.assertTrue(pose_within_cad(result.pose_mdeg))
        self.assertLessEqual(max(abs(a-b) for a,b in zip(loop.controller.pose, before)), .04+1e-8)
        pulses = pose_to_pulses_us(loop.controller.pose)
        previous = pose_to_pulses_us(before)
        self.assertLessEqual(max(abs(a-b) for a,b in zip(pulses, previous)), 2.+1e-8)

    def test_presteering_moves_without_ay_but_never_above_two_and_returns_after_timeout(self):
        loop = PreviewLoop()
        maximum = 0
        for _ in range(200):
            result, before = loop.step()
            self.check_limits(loop, result, before)
            self.assertEqual(result.pose_mdeg[:2], (0,0))
            self.assertEqual(result.pose_mdeg[3], 0)
            maximum = max(maximum, result.pose_mdeg[2])
        self.assertGreater(maximum, 1000)
        self.assertLessEqual(maximum, 2000)
        self.assertEqual(result.pose_mdeg, (0,0,0,0))

    def test_simulation_slow_motion_does_not_extend_real_episode(self):
        loop = PreviewLoop()
        for _ in range(90):
            result, before = loop.step(sim_dt=.002)
            self.check_limits(loop, result, before)
        self.assertEqual(result.diag['steering_preview']['reason'], 'episode_expired_recenter_required')
        self.assertEqual(result.diag['requested_pose_deg'], [0.,0.,0.,0.])

    def test_opposite_primary_response_passes_neutral_before_switching(self):
        loop = PreviewLoop()
        for _ in range(45):
            loop.step()
        self.assertGreater(loop.pose[2], 0)
        reached_zero = False
        for _ in range(450):
            result, before = loop.step(ay=-4.)
            self.check_limits(loop, result, before)
            if result.pose_mdeg == (0,0,0,0): reached_zero = True
            if result.pose_mdeg[0] > 0:
                self.assertTrue(reached_zero)
                self.assertEqual(result.pose_mdeg[2:], (0,0))
        self.assertEqual(loop.pose, (15000,6000,0,0))

    def test_primary_steady_state_unchanged_with_or_without_preview(self):
        with_preview = PreviewLoop()
        without = PreviewLoop(Settings(steering_preview_enabled=False))
        for _ in range(600):
            a, before = with_preview.step(ay=4.)
            b, _ = without.step(ay=4.)
            self.check_limits(with_preview, a, before)
            self.assertEqual(a.pose_mdeg, b.pose_mdeg)
        self.assertEqual(a.pose_mdeg, (0,0,15000,6000))

    def test_missing_preview_fields_retains_primary_control_and_fault_contract(self):
        loop = PreviewLoop()
        for _ in range(10):
            result, before = loop.step(gear_index=None)
            self.check_limits(loop, result, before)
            self.assertEqual(result.pose_mdeg, (0,0,0,0))
        for _ in range(30):
            result, before = loop.step(ay=2., gear_index=None)
            self.check_limits(loop, result, before)
        self.assertGreater(result.pose_mdeg[2], 0)
        result, _ = loop.step(valid=False, reason='lateral_acceleration_outlier')
        self.assertTrue(result.emergency_off)
        self.assertEqual(result.reason, 'vehicle:lateral_acceleration_outlier')

    def test_preview_settings_limits_and_types(self):
        for fields in ({'steering_preview_enabled':1}, {'steering_to_ay_sign':True},
                       {'steering_to_ay_sign':0}, {'steering_preview_max_yaw_deg':2.001},
                       {'steering_preview_timeout_s':1.501}, {'steering_preview_dwell_s':.01},
                       {'steering_preview_deadband_deg':15., 'steering_preview_reference_deg':10.},
                       {'steering_preview_min_speed_mps':15., 'steering_preview_full_speed_mps':15.}):
            with self.subTest(fields=fields), self.assertRaises(ValueError):
                Settings(**fields).validate()


if __name__ == '__main__':
    unittest.main()
