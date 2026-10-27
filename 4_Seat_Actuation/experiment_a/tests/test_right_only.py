"""Right-only hardware commissioning without reversing support direction."""
from dataclasses import replace
import unittest
from experiment_a.tests.test_controller_geometry import CommandLoop
from experiment_a.settings import Settings
from experiment_a.kinematics import allowable_pitch_for_yaw, pose_to_pulses_us
from experiment_a.tuning import apply_firmware_tuning
from experiment_a.tests.test_tuning import SNAPSHOT


class RightOnlyTests(unittest.TestCase):
    def test_left_turn_press_right_turn_release_never_commands_absent_left(self):
        loop=CommandLoop(Settings(available_bolsters='right_only'))
        for ay in (6.,-6.,6.,0.):
            for _ in range(900):
                decision,before,_=loop.step(ay)
                self.assertEqual(decision.mode,'RUN',decision.reason)
                self.assertEqual(decision.pose_mdeg[:2],(0,0))
                self.assertNotEqual(decision.diag['active_side'],'left')
                self.assertFalse(loop.control.reversing)
                after=loop.control.pose
                self.assertLessEqual(abs(after[3]),allowable_pitch_for_yaw(after[2],6)+1e-9)
                self.assertLessEqual(max(abs(a-b) for a,b in zip(pose_to_pulses_us(before),pose_to_pulses_us(after))),2+1e-8)
            if ay>0:
                self.assertEqual(decision.pose_mdeg,(0,0,15000,6000))
                self.assertTrue(decision.diag['support_side_available'])
            else:
                self.assertEqual(decision.pose_mdeg,(0,0,0,0))
                if ay<0:
                    self.assertFalse(decision.diag['support_side_available'])
                    self.assertEqual(decision.reason,'unavailable_left_bolster_return_neutral')

    def test_unexpected_left_command_state_fails_closed(self):
        for field in ('pose_mdeg','target_mdeg'):
            loop=CommandLoop(Settings(available_bolsters='right_only'))
            decision,_,_=loop.step(2,override_state={field:(1,0,0,0)})
            self.assertTrue(decision.emergency_off)
            self.assertEqual(decision.reason,'unavailable_left_bolster_nonzero_command_state')

    def test_firmware_tuning_cannot_restore_missing_side(self):
        settings,_=apply_firmware_tuning(Settings(available_bolsters='right_only'),SNAPSHOT)
        self.assertEqual(settings.available_bolsters,'right_only')
        with self.assertRaises(ValueError):Settings(available_bolsters='guess').validate()
