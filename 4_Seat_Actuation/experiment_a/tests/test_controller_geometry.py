"""Closed command-path tests; synthetic board reports are not a physical plant."""
from dataclasses import asdict, replace
import math
import random
import unittest

from experiment_a.controller import ExperimentAController
from experiment_a.kinematics import allowable_pitch_for_yaw, inward_offsets_mm, pose_to_pulses_us
from experiment_a.settings import Settings
from fallback.beamng import VehicleFrame
from fallback.serial_link import BoardState


class CommandLoop:
    """Feed real model fresh vehicle rows and echo previous commands as telemetry.

    This ideal command echo tests controller sequencing/geometry only. It does
    not simulate servo lag, actual shaft feedback, occupant motion, or force.
    """
    def __init__(self, settings=None):
        self.control = ExperimentAController(settings or Settings())
        self.now = 1_000_000_000
        self.sequence = 0
        self.pose = (0, 0, 0, 0)
        self.last_vehicle = None

    def step(self, ay, *, ax=0.0, stopped=False, held=False, override_state=None):
        self.now += 20_000_000
        self.sequence += 1
        if held:
            vehicle = self.last_vehicle
        else:
            vehicle = VehicleFrame(self.now, self.sequence * .02, 'geometry-test', 'vehicle',
                                   self.sequence, ay, ax, 0.0 if stopped else 15.0,
                                   0.0, 0.0, 0.0, not stopped,
                                   'stopped' if stopped else 'ok', 'geometry-test')
            self.last_vehicle = vehicle
        pulse = tuple(round(p) for p in pose_to_pulses_us(tuple(x / 1000 for x in self.pose)))
        state = BoardState(self.now, True, False, self.pose, self.pose, pulse,
                           self.sequence, 'NONE', self.sequence * 20)
        if override_state:
            state = replace(state, **override_state)
        before = tuple(self.control.pose)
        decision = self.control.tick(vehicle, state, self.now)
        if decision.mode == 'RUN':
            self.pose = decision.pose_mdeg
        return decision, before, state


class ControllerGeometryTests(unittest.TestCase):
    def assert_geometry_and_rates(self, loop, decision, before, previous_command=None):
        self.assertEqual(decision.mode, 'RUN', decision.reason)
        settings = loop.control.settings
        after = loop.control.pose
        for side in (0, 2):
            self.assertGreaterEqual(after[side], 0)
            self.assertLessEqual(after[side], settings.max_yaw_deg)
            self.assertLessEqual(abs(after[side + 1]), settings.max_pitch_deg)
            self.assertLessEqual(abs(after[side+1]),allowable_pitch_for_yaw(after[side],settings.max_pitch_deg)+1e-8)
            self.assertGreaterEqual(min(inward_offsets_mm(*after[side:side + 2])), -1e-8)
            commanded = tuple(x / 1000 for x in decision.pose_mdeg[side:side + 2])
            self.assertGreaterEqual(min(inward_offsets_mm(*commanded)), -1e-10)
            self.assertLessEqual(abs(after[side] - before[side]), settings.max_yaw_rate_deg_s * .02 + 1e-10)
            self.assertLessEqual(abs(after[side + 1] - before[side + 1]), settings.max_pitch_rate_deg_s * .02 + 1e-10)
        before_p = pose_to_pulses_us(before)
        after_p = pose_to_pulses_us(after)
        self.assertLessEqual(max(abs(a - b) for a, b in zip(before_p, after_p)), settings.max_pulse_rate_us_s * .02 + 1e-8)
        self.assertTrue(all(500 <= p <= 2500 for p in after_p))
        # Quantizing output to millidegrees and clamping pitch inward can add
        # sub-microsecond differences. MCU independently rounds/limits at 2 us.
        if previous_command is not None:
            old = pose_to_pulses_us(tuple(x / 1000 for x in previous_command))
            new = pose_to_pulses_us(tuple(x / 1000 for x in decision.pose_mdeg))
            self.assertLessEqual(max(abs(a - b) for a, b in zip(old, new)), settings.max_pulse_rate_us_s * .02 + .2)
            self.assertLessEqual(max(abs(round(a) - round(b)) for a, b in zip(old, new)), math.ceil(settings.max_pulse_rate_us_s * .02 + .2))

    def test_sustained_left_and_right_acceleration_are_mirrored_yaw_and_pitch(self):
        plus, minus = CommandLoop(), CommandLoop()
        for _ in range(1001):
            a, before_a, _ = plus.step(6.0)
            b, before_b, _ = minus.step(-6.0)
            self.assert_geometry_and_rates(plus, a, before_a)
            self.assert_geometry_and_rates(minus, b, before_b)
            self.assertEqual(a.pose_mdeg, b.pose_mdeg[2:] + b.pose_mdeg[:2])
            self.assertEqual(a.pose_mdeg[:2], (0, 0))
            self.assertEqual(b.pose_mdeg[2:], (0, 0))
        self.assertEqual(a.pose_mdeg, (0, 0, 15000, 6000))
        self.assertEqual(b.pose_mdeg, (15000, 6000, 0, 0))
        upper, lower = inward_offsets_mm(20, 8)
        self.assertLess(upper, lower)  # Conform-lean lets the upper plate follow outward lean.

    def test_conform_mode_has_opposite_pitch_but_same_yaw(self):
        counter = CommandLoop(Settings(max_yaw_rate_deg_s=20, max_pitch_rate_deg_s=20, pitch_mode='counter_lean'))
        conform = CommandLoop(Settings(max_yaw_rate_deg_s=20, max_pitch_rate_deg_s=20, pitch_mode='conform_lean'))
        for _ in range(501):
            a, ba, _ = counter.step(6)
            b, bb, _ = conform.step(6)
            self.assert_geometry_and_rates(counter, a, ba)
            self.assert_geometry_and_rates(conform, b, bb)
        self.assertEqual(a.pose_mdeg[2], b.pose_mdeg[2])
        self.assertEqual(a.pose_mdeg[3], -b.pose_mdeg[3])
        self.assertGreater(b.pose_mdeg[3], 0)

    def test_zero_and_longitudinal_only_input_do_not_invent_lateral_support(self):
        for ax in (0., -4., 4.):
            loop = CommandLoop()
            for _ in range(201):
                result, before, _ = loop.step(0, ax=ax)
                self.assert_geometry_and_rates(loop, result, before)
                self.assertEqual(result.pose_mdeg, (0, 0, 0, 0))
            if ax:
                self.assertNotEqual(loop.control.predicted.pitch_deg, 0)
                self.assertEqual(loop.control.predicted.roll_deg, 0)

    def test_stopped_returns_existing_command_to_neutral_and_resets_prediction(self):
        loop = CommandLoop(Settings(max_yaw_rate_deg_s=20, max_pitch_rate_deg_s=20))
        for _ in range(501):
            result, _, _ = loop.step(6)
        self.assertGreater(result.pose_mdeg[2], 0)
        for _ in range(501):
            result, before, _ = loop.step(6, stopped=True)
            self.assert_geometry_and_rates(loop, result, before)
            self.assertEqual(loop.control.predicted.roll_deg, 0)
            self.assertEqual(result.reason, 'stopped_neutral')
        self.assertEqual(result.pose_mdeg, (0, 0, 0, 0))

    def test_six_degree_pitch_reaches_endpoint_using_combined_pulse_guard(self):
        loop = CommandLoop(Settings(max_yaw_deg=15,max_pitch_deg=6, max_yaw_rate_deg_s=20,
                                    max_pitch_rate_deg_s=20, pitch_mode='conform_lean'))
        limited = 0
        old_command = (0, 0, 0, 0)
        for _ in range(1001):
            result, before, _ = loop.step(10)
            self.assert_geometry_and_rates(loop, result, before, old_command)
            old_command = result.pose_mdeg
            limited += result.diag['pulse_slew_limited']
        self.assertEqual(result.pose_mdeg, (0, 0, 15000, 6000))
        self.assertGreater(limited, 0)

    def test_high_yaw_taper_does_not_stall_extension_or_return(self):
        # Start with substantial Pitch, extend through its shrinking envelope,
        # then return. Check the entire path, including quantized commands.
        for cap in (20., 22.):
            with self.subTest(yaw_cap=cap):
                loop = CommandLoop(Settings(max_yaw_deg=cap))
                previous = (0, 0, 0, 0)
                for ay in (2.0, 6., 0.):
                    for _ in range(801):
                        result, before, _ = loop.step(ay)
                        self.assert_geometry_and_rates(loop, result, before, previous)
                        previous = result.pose_mdeg
                    if ay == 2.0:
                        self.assertGreater(result.pose_mdeg[3], 4000)
                    elif ay == 6.:
                        self.assertEqual(result.pose_mdeg[2], int(cap * 1000))
                        self.assertEqual(result.pose_mdeg[3], math.floor(allowable_pitch_for_yaw(cap) * 1000))
                    else:
                        self.assertEqual(result.pose_mdeg, (0, 0, 0, 0))

    def test_reverse_waits_for_zero_board_target_before_changing_side(self):
        loop = CommandLoop(Settings(max_yaw_rate_deg_s=20, max_pitch_rate_deg_s=20))
        for _ in range(501):
            loop.step(6)
        saw_wait = False
        neutral_output_seen = False
        for _ in range(501):
            override = {'pose_mdeg': (0, 0, 100, 0), 'target_mdeg': (0, 0, 100, 0)}
            result, before, _ = loop.step(-6, override_state=override)
            self.assert_geometry_and_rates(loop, result, before)
            self.assertEqual(result.pose_mdeg[:2], (0, 0))
            saw_wait |= result.reason == 'returning_neutral_before_side_change'
            neutral_output_seen |= result.pose_mdeg == (0, 0, 0, 0)
        self.assertTrue(saw_wait)
        self.assertTrue(neutral_output_seen)
        result, before, state = loop.step(-6)
        self.assertEqual(state.pose_mdeg, (0, 0, 0, 0))
        self.assert_geometry_and_rates(loop, result, before)
        self.assertGreater(result.pose_mdeg[0], 0)
        self.assertEqual(result.pose_mdeg[2:], (0, 0))

    def test_repeated_latest_vehicle_sample_does_not_advance_human_prediction(self):
        loop = CommandLoop()
        for _ in range(26):
            loop.step(6)
        predicted = asdict(loop.control.predicted)
        original_pose = tuple(loop.control.pose)
        for _ in range(5):
            result, before, _ = loop.step(6, held=True)
            self.assert_geometry_and_rates(loop, result, before)
            self.assertEqual(asdict(loop.control.predicted), predicted)
        self.assertNotEqual(tuple(loop.control.pose), original_pose)

    def test_seeded_random_reversals_remain_feasible_and_never_close_both_sides(self):
        rng = random.Random(20261026)
        for mode in ('counter_lean', 'conform_lean'):
            loop = CommandLoop(Settings(max_pitch_deg=6, max_yaw_rate_deg_s=20,
                                        max_pitch_rate_deg_s=20, pitch_mode=mode))
            old_command = (0, 0, 0, 0)
            ay = 0
            for index in range(1601):
                if index % 71 == 0:
                    ay = rng.uniform(-11.5, 11.5)
                result, before, _ = loop.step(ay, ax=2 * math.sin(index / 50))
                self.assert_geometry_and_rates(loop, result, before, old_command)
                old_command = result.pose_mdeg
                self.assertFalse(result.pose_mdeg[0] > 0 and result.pose_mdeg[2] > 0)
                for side in (0, 2):
                    cap = allowable_pitch_for_yaw(result.pose_mdeg[side] / 1000, 8)
                    self.assertLessEqual(abs(result.pose_mdeg[side + 1]) / 1000, cap + 1e-12)


if __name__ == '__main__':
    unittest.main()
