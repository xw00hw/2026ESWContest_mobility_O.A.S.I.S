"""Shock heuristic and replay regressions; synthetic shocks are not crash truth."""
from dataclasses import replace
import csv
import json
from pathlib import Path
import tempfile
import unittest

from experiment_a.impact import ImpactDetector
from experiment_a.replay import replay_vehicle
from experiment_a.settings import Settings
from experiment_a.vehicle import VehicleFrame
import test_controller_geometry as geometry
from test_vehicle import row


def sample(i, ax=0., ay=0., dt=.02, **changes):
    value=VehicleFrame(1_000_000_000+round(i*dt*1e9), i*dt, 'shock', 'car', i,
                       ay, ax, 15., 0., 0., 0., True, 'ok', 'beamng')
    return replace(value, **changes)


class ImpactTests(unittest.TestCase):
    def test_user_reported_noncollision_and_older_unlabeled_isolated_peak_do_not_trip(self):
        # Rounded historical neighborhoods: no claim that the older peak was a crash.
        for peak in ((7.251468709664833,12.871359949150792),(-12.970685,30.2687)):
            guard=ImpactDetector(Settings())
            for i, (ax,ay) in enumerate(((0.,0.),(3.8777,4.3251),peak,(3.,6.),(2.,5.))):
                self.assertFalse(guard.observe(sample(i,ax,ay)).triggered)

    def test_front_side_and_diagonal_sustained_shocks_trip_and_latch(self):
        for ax,ay in ((-30.,0.),(30.,0.),(0.,30.),(0.,-30.),(20.,20.)):
            with self.subTest(ax=ax,ay=ay):
                guard=ImpactDetector(Settings());guard.observe(sample(0))
                for i in range(1,4):
                    self.assertFalse(guard.observe(sample(i,ax,ay)).triggered)
                impact=guard.observe(sample(4,ax,ay))
                self.assertTrue(impact.triggered)
                self.assertEqual(impact.reason,'impact_suspected_sustained')
                self.assertGreaterEqual(impact.vector_delta_v_mps,1.5)
                self.assertIs(guard.observe(sample(5)),impact)

    def test_extreme_planar_guard_uses_both_axes_and_is_immediate(self):
        for ax,ay in ((0.,60.),(-60.,0.),(43.,43.)):
            guard=ImpactDetector(Settings())
            impact=guard.observe(sample(1,ax,ay))
            self.assertTrue(impact.triggered)
            self.assertEqual(impact.reason,'impact_suspected_extreme')

    def test_duration_and_impulse_both_required(self):
        guard=ImpactDetector(Settings(impact_min_delta_v_mps=3.))
        guard.observe(sample(0))
        for i in range(1,5):self.assertFalse(guard.observe(sample(i,0.,25.)).triggered)
        self.assertAlmostEqual(guard.state.high_duration_s,.06)
        self.assertAlmostEqual(guard.state.vector_delta_v_mps,1.5)
        self.assertTrue(guard.observe(sample(7,0.,25.)).triggered is False)  # missing seq resets history

    def test_alternating_high_samples_do_not_accumulate_absolute_impulse(self):
        guard=ImpactDetector(Settings());guard.observe(sample(0))
        for i in range(1,20):
            state=guard.observe(sample(i,0.,30. if i%2 else -30.))
            self.assertFalse(state.triggered)
            self.assertLess(state.vector_delta_v_mps,.01)

    def test_duplicate_source_does_not_earn_duration(self):
        guard=ImpactDetector(Settings());guard.observe(sample(0))
        first=guard.observe(sample(1,0.,30.))
        for _ in range(100):self.assertIs(guard.observe(sample(1,0.,30.)),first)
        self.assertEqual(first.high_duration_s,0.)

    def test_stream_gap_and_invalid_quality_clear_partial_evidence(self):
        changes=({'sequence_id':10},{'sim_time_s':1.}, {'stream_id':'changed'},
                 {'received_monotonic_ns':1},{'valid':False,'reason':'invalid_derivative'})
        for change in changes:
            guard=ImpactDetector(Settings());guard.observe(sample(0))
            guard.observe(sample(1,0.,30.));guard.observe(sample(2,0.,30.))
            state=guard.observe(sample(3,0.,30.,**change))
            self.assertFalse(state.triggered)
            self.assertEqual(state.high_duration_s,0.)

    def test_shock_while_vehicle_stopped_still_detected(self):
        guard=ImpactDetector(Settings());guard.observe(sample(0))
        for i in range(1,5):
            state=guard.observe(sample(i,-30.,0.,valid=False,reason='stopped',speed_mps=0.))
        self.assertTrue(state.triggered)

    def test_slow_high_g_ramp_has_no_abrupt_onset(self):
        guard=ImpactDetector(Settings());guard.observe(sample(0))
        for i in range(1,101):
            self.assertFalse(guard.observe(sample(i,0.,min(40.,i*.4))).triggered)

    def test_abrupt_change_does_not_borrow_old_high_g_duration(self):
        guard=ImpactDetector(Settings())
        for i in range(20):self.assertFalse(guard.observe(sample(i,0.,30.)).triggered)
        state=guard.observe(sample(20,10.,30.))
        self.assertFalse(state.triggered)
        self.assertEqual(state.high_duration_s,0.)
        self.assertEqual(state.vector_delta_v_mps,0.)

    def test_nonfinite_or_outside_integrity_range_rejected(self):
        for value in (float('nan'),float('inf'),201.):
            with self.assertRaises(ValueError):ImpactDetector(Settings()).observe(sample(0,0.,value))

    def test_settings_cannot_remove_finite_shock_bounds(self):
        for changes in ({'impact_planar_threshold_mps2':0.}, {'impact_min_duration_s':0.},
                        {'impact_min_delta_v_mps':float('nan')},
                        {'impact_planar_threshold_mps2':50.,'impact_extreme_planar_mps2':50.}):
            with self.assertRaises(ValueError):Settings(**changes).validate()

    def test_simulation_time_not_host_rate_drives_duration(self):
        guard=ImpactDetector(Settings());guard.observe(sample(0))
        for i in range(1,4):
            v=sample(i,0.,30.,received_monotonic_ns=1_000_000_000+i*100_000_000)
            self.assertFalse(guard.observe(v).triggered)
        self.assertTrue(guard.observe(sample(4,0.,30.,received_monotonic_ns=1_400_000_000)).triggered)


class HighAccelerationControllerTests(unittest.TestCase):
    def test_above_old_threshold_saturates_target_without_widening_angles_or_rates(self):
        checks=geometry.ControllerGeometryTests()
        for sign in (-1,1):
            loop=geometry.CommandLoop()
            for i in range(1001):
                decision,before,_=loop.step(sign*20.,ax=5.)
                checks.assert_geometry_and_rates(loop,decision,before)
            expected=(15000,6000,0,0) if sign<0 else (0,0,15000,6000)
            self.assertEqual(decision.pose_mdeg,expected)
            self.assertEqual(decision.diag['demand_fraction'],1.)
            self.assertFalse(decision.diag['impact']['triggered'])

    def test_pure_longitudinal_above_old_limit_does_not_create_lateral_motion(self):
        loop=geometry.CommandLoop()
        for i in range(50):
            decision,_,_=loop.step(0.,ax=-20.)
            self.assertEqual(decision.mode,'RUN')
            self.assertEqual(decision.pose_mdeg,(0,0,0,0))

    def test_standalone_controller_latches_shock(self):
        loop=geometry.CommandLoop();loop.step(0.)
        for i in range(4):decision,_,_=loop.step(30.)
        self.assertTrue(decision.emergency_off)
        self.assertEqual(decision.reason,'impact_suspected_sustained')
        self.assertTrue(decision.diag['impact']['triggered'])


class ReplayImpactTests(unittest.TestCase):
    def test_replay_keeps_rows_after_impact_but_does_not_resume_commands(self):
        values=[row(i,linear_acc_y_mps2=2.) for i in range(1,261)]
        for i in range(150,154):values[i]['linear_acc_y_mps2']=30.
        with tempfile.TemporaryDirectory() as tmp:
            source=Path(tmp)/'vehicle.csv'
            with source.open('w',newline='',encoding='utf-8') as f:
                writer=csv.DictWriter(f,fieldnames=values[0]);writer.writeheader();writer.writerows(values)
            report=replay_vehicle(source,Path(tmp)/'out')
            with (Path(tmp)/'out/commands.csv').open(encoding='utf-8',newline='') as f:
                output=list(csv.DictReader(f))
            self.assertEqual(len(output),len(values))
            self.assertIsNotNone(report['impact_event'])
            first=next(i for i,r in enumerate(output) if r['impact_suspected']=='1')
            self.assertTrue(all(r['policy_reason']=='impact_record_only' for r in output[first:]))
            self.assertTrue(all(float(r['command_RY_deg'])==0 for r in output[first:]))
            self.assertEqual(report['hardware_commands_sent'],0)


if __name__=='__main__':unittest.main()
