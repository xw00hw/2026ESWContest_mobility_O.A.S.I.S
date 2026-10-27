"""Synthetic control-loop tests; channel assignments here are NOT hardware mapping."""
from types import SimpleNamespace
import unittest

from fallback.beamng import VehicleFrame
from fallback.controller import FallbackController, LateralDemand
from fallback.profile import BenchProfile, Settings


class ControllerTests(unittest.TestCase):
    def setUp(self):
        # Test-only association chosen to exercise positive envelope branches.
        self.settings = Settings(left_channels=("A1_C1",), right_channels=("A1_C0",))
        synthetic = BenchProfile()
        # Replace measured responses with an explicitly synthetic linear fixture.
        # The measured profile is separately tested to flag its unidentified
        # pressure gain; it must not be treated as a pressure inverse model.
        for c in self.settings.guard_channels:
            synthetic.base[c]["std_v"] = .001
        for side in ("left", "right"):
            xs, points = synthetic.points[side]
            for i, point in enumerate(points):
                for c in self.settings.guard_channels:
                    point["channels"][c]["mean_v"] = synthetic.base[c]["mean_v"]+.01*xs[i]
                    synthetic.envelopes[side][c][i] = .01*xs[i]
        self.cal = synthetic.derive(self.settings)
        self.control = FallbackController(self.cal)
        self.now = 1_000_000_000
        self.state = SimpleNamespace(armed=True, moving=False, fault="NONE", received_ns=self.now,
                                     pose_mdeg=(0,0,0,0), target_mdeg=(0,0,0,0))
        self.pressure = SimpleNamespace(valid=True, received_ns=self.now,
            voltages={c: self.cal.profile.base[c]["mean_v"] for c in self.settings.guard_channels})

    def step(self, ay=0, *, dt=.02, valid=True, reason="ok"):
        self.now += round(dt*1e9)
        self.state.received_ns = self.pressure.received_ns = self.now
        vehicle = VehicleFrame(self.now, self.now/1e9, "s", "v", self.now, ay, 0., 10.,
                               None, None, None, valid, reason, "beamng")
        return self.control.tick(vehicle, self.pressure, self.state, self.now)

    def baseline(self):
        for _ in range(102):
            decision = self.step()
        self.assertEqual(decision.mode, "RUN", decision)

    def test_no_guessed_mapping(self):
        model = BenchProfile().derive(Settings())
        self.assertTrue(model.reasons)
        self.assertEqual(model.angle_for_demand("left", 1), 0)

    def test_twenty_degree_command_cap_keeps_independent_live_pressure_guards(self):
        self.assertEqual(self.settings.max_yaw_deg, 20)
        self.assertEqual(self.cal.max_yaw, {"left":20, "right":20})
        for channel in self.settings.guard_channels:
            span=max(self.cal.profile.envelopes[side][channel][-1] for side in ("left","right"))
            self.assertAlmostEqual(self.cal.channel_caps_v[channel], .2*span)
            self.assertGreaterEqual(self.cal.noise_v[channel], .01)

    def test_real_mapped_data_is_position_cue_not_pressure_inverse(self):
        settings = Settings(left_channels=("A1_C2", "A1_C3"), right_channels=("A1_C0", "A1_C1"))
        model = BenchProfile().derive(settings)
        self.assertEqual(model.reasons, [])
        self.assertEqual(len(model.advisories), 2)
        self.assertEqual(model.max_yaw, {"left":20, "right":20})
        for side in ("left", "right"):
            self.assertEqual(model.angle_for_demand(side, 0), 0)
            self.assertEqual(model.angle_for_demand(side, .5), 10)
            self.assertEqual(model.angle_for_demand(side, 1), 20)
        # The user's commanded range replaces the earlier static p95-derived
        # angle clip. The recorded pressure increase guards remain independent.
        for channel in settings.guard_channels:
            span=max(model.profile.envelopes[side][channel][-1] for side in ("left","right"))
            self.assertAlmostEqual(model.channel_caps_v[channel], .2*span)

    def test_lower_explicit_yaw_setting_is_respected(self):
        settings=Settings(left_channels=("A1_C2","A1_C3"), right_channels=("A1_C0","A1_C1"),
                          max_yaw_deg=12.5)
        model=BenchProfile().derive(settings)
        self.assertEqual(model.max_yaw, {"left":12.5, "right":12.5})
        self.assertEqual(model.angle_for_demand("left",1),12.5)

    def reach_full_range(self, ay, index):
        """Fresh synthetic pressure plus immediate command-state following.

        This is a policy test, not measured pressure or actuator feedback at 20°.
        """
        self.baseline()
        previous=(0,0,0,0)
        for _ in range(650):
            decision=self.step(ay)
            self.assertEqual(decision.mode,"RUN")
            self.assertTrue(decision.permit_lease)
            self.assertFalse(decision.emergency_off)
            self.assertEqual(decision.pose_mdeg[1::2],(0,0))
            self.assertEqual(decision.pose_mdeg[2-index],0)
            for axis in (0,2):
                self.assertLessEqual(abs(decision.pose_mdeg[axis]-previous[axis]),40)
                self.assertTrue(0 <= decision.pose_mdeg[axis] <= 20000)
            self.state.pose_mdeg=self.state.target_mdeg=decision.pose_mdeg
            previous=decision.pose_mdeg
        self.assertEqual(decision.pose_mdeg[index],20000)
        return decision

    def test_each_side_reaches_twenty_with_zero_pitch_and_rate_limit(self):
        for ay,index in ((4.,2),(-4.,0)):
            with self.subTest(ay=ay):
                self.setUp()
                self.reach_full_range(ay,index)

    def test_plus_y_supports_right_and_slew_is_bounded(self):
        self.baseline()
        previous = (0,0,0,0)
        for _ in range(150):
            d = self.step(2.)
            self.assertEqual(d.mode, "RUN")
            self.assertEqual(d.pose_mdeg[0], 0)
            self.assertEqual(d.pose_mdeg[1::2], (0,0))
            self.assertLessEqual(abs(d.pose_mdeg[2]-previous[2]), 40)
            self.assertLessEqual(d.pose_mdeg[2], self.cal.max_yaw["right"]*1000)
            previous = d.pose_mdeg
        self.assertGreater(previous[2], 0)

    def test_reversal_waits_for_firmware_neutral(self):
        d=self.reach_full_range(4.,2)
        self.state.moving = True
        previous=d.pose_mdeg
        for _ in range(650):
            d = self.step(-4.)
            self.assertEqual(d.pose_mdeg[0], 0)
            self.assertEqual(d.pose_mdeg[1::2],(0,0))
            self.assertLessEqual(abs(d.pose_mdeg[2]-previous[2]),40)
            previous=d.pose_mdeg
        self.assertEqual(d.pose_mdeg, (0,0,0,0))
        self.assertEqual(d.reason,"returning_neutral_before_side_change")
        # The host's zero target alone is insufficient; firmware must report
        # its commanded neutral completion before the opposite side advances.
        self.state.moving = False
        self.state.pose_mdeg = self.state.target_mdeg = (0,0,0,0)
        for _ in range(650):
            d = self.step(-4.)
            self.assertLessEqual(abs(d.pose_mdeg[0]-previous[0]),40)
            self.assertEqual(d.pose_mdeg[1:],(0,0,0))
            self.state.pose_mdeg=self.state.target_mdeg=d.pose_mdeg
            previous=d.pose_mdeg
        self.assertEqual(d.pose_mdeg,(20000,0,0,0))

    def test_bad_source_fault_latches(self):
        self.baseline()
        d = self.step(valid=False, reason="stream_changed")
        self.assertTrue(d.emergency_off)
        self.assertFalse(d.permit_lease)
        self.assertEqual(self.step().reason, "vehicle:stream_changed")

    def test_stationary_is_neutral_not_latched_fault(self):
        self.baseline()
        for _ in range(50):
            d = self.step(2.)
        for _ in range(50):
            d = self.step(valid=False, reason="stopped")
        self.assertEqual(d.pose_mdeg, (0,0,0,0))
        self.assertTrue(d.permit_lease)

    def test_pressure_excess_does_not_increase_angle(self):
        self.baseline()
        c = "A1_C1"
        self.pressure.voltages[c] += self.cal.channel_caps_v[c]+2*self.cal.noise_v[c]+.01
        d = self.step(2.)
        self.assertEqual(d.reason, "pressure_hard_increment_limit")
        self.assertTrue(d.emergency_off)

    def test_pressure_hard_limit_at_twenty_latches_fault_on_either_side(self):
        for ay,index,channel in ((4.,2,"A1_C0"),(-4.,0,"A1_C1")):
            with self.subTest(index=index):
                self.setUp()
                self.reach_full_range(ay,index)
                self.pressure.voltages[channel] += (self.cal.channel_caps_v[channel]+
                                                   2*self.cal.noise_v[channel]+.001)
                decision=self.step(ay)
                self.assertEqual(decision.reason,"pressure_hard_increment_limit")
                self.assertTrue(decision.emergency_off)
                self.assertFalse(decision.permit_lease)
                self.pressure.voltages[channel]=self.control.baseline[channel]
                self.assertEqual(self.step(ay).reason,"pressure_hard_increment_limit")

    def test_persistent_pressure_excess_at_twenty_returns_then_faults(self):
        self.reach_full_range(4.,2)
        channel="A1_C0"
        self.pressure.voltages[channel] += self.cal.channel_caps_v[channel]+1.5*self.cal.noise_v[channel]
        decision=self.step(4.)
        self.assertEqual(decision.reason,"pressure_guard_returning_neutral")
        self.assertEqual(decision.pose_mdeg,(0,0,19960,0))
        for _ in range(5):
            decision=self.step(4.)
        self.assertEqual(decision.reason,"pressure_increment_limit_persisted")
        self.assertTrue(decision.emergency_off)
        self.assertFalse(decision.permit_lease)

    def test_nonzero_pitch_and_yaw_above_twenty_reject_board_state(self):
        for pose,reason in (((0,1,0,0),"unexpected_pitch"),
                            ((0,0,0,-1),"unexpected_pitch"),
                            ((21000,0,0,0),"board_outside_profile_range"),
                            ((0,0,21000,0),"board_outside_profile_range")):
            with self.subTest(pose=pose):
                self.setUp()
                self.baseline()
                self.state.pose_mdeg=self.state.target_mdeg=pose
                decision=self.step(4.)
                self.assertEqual(decision.reason,reason)
                self.assertTrue(decision.emergency_off)
                self.assertFalse(decision.permit_lease)

    def test_control_stall_does_not_catch_up(self):
        self.baseline()
        d = self.step(2, dt=.3)
        self.assertEqual(d.reason, "host_control_clock_gap")

    def test_adc_saturation_fault(self):
        self.pressure.voltages["A1_C1"] = 3.3
        self.assertTrue(self.step().emergency_off)

    def test_startup_requires_neutral(self):
        self.state.pose_mdeg = (100,0,0,0)
        self.state.moving = True
        self.assertEqual(self.step().reason, "startup_requires_commanded_neutral")

    def test_settings_reject_unsafe_or_nonfinite_override(self):
        self.assertEqual(Settings(max_yaw_deg=20).max_yaw_deg,20)
        for kwargs in ({"pressure_fraction":1}, {"max_yaw_deg":20.00001}, {"max_yaw_deg":27},
                       {"max_yaw_deg":float("nan")}, {"max_yaw_deg":True}, {"max_rate_deg_s":float("nan")},
                       {"left_channels":["A1_C0"],"right_channels":["A1_C0"]}):
            with self.assertRaises(ValueError):
                Settings(**kwargs)

    def test_hysteresis(self):
        demand = LateralDemand(self.settings)
        for i in range(100):
            side, amount = demand.update(.1, i*20_000_000)
        self.assertEqual(side, "none")
        self.assertEqual(amount, 0)

    def test_sign_crossing_inside_hysteresis_drops_old_side(self):
        demand = LateralDemand(self.settings)
        demand.filtered, demand.side, demand.last_ns = .30, "right", 0
        side, amount = demand.update(-5., 20_000_000)
        self.assertLess(demand.filtered, 0)
        self.assertEqual(side, "none")
        self.assertEqual(amount, 0)


if __name__ == "__main__":
    unittest.main()
