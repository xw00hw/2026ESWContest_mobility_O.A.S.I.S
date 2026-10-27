from dataclasses import replace
import json
from pathlib import Path
import tempfile
import unittest

from fallback.serial_link import BoardState
from experiment_a.controller import ExperimentAController
from experiment_a.settings import Settings
from experiment_a.vehicle import VehicleFrame


def frame(now=1_000_000_000,seq=1,sim=0.,**changes):
    v=VehicleFrame(now,sim,'stream','car',seq,2.,0.,10.,0.,0.,0.,True,'ok','beamng')
    return replace(v,**changes)

def board(now=1_000_000_000,**changes):
    b=BoardState(now,True,False,(0,0,0,0),(0,0,0,0),(1800,1800,1200,1200),0,'NONE')
    return replace(b,**changes)


class ControllerFaultTests(unittest.TestCase):
    def test_high_yaw_pitch_violation_in_pose_or_target_latches_off(self):
        for key in ('pose_mdeg', 'target_mdeg'):
            with self.subTest(key=key):
                c=ExperimentAController(Settings())
                c.tick(frame(),board(),1_000_000_000)
                d=c.tick(frame(1_020_000_000,2,.02),
                         board(1_020_000_000,**{key:(0,0,25000,8000)}),1_020_000_000)
                self.assertTrue(d.emergency_off)
                self.assertEqual(d.reason,'board_pose_outside_CAD_envelope')
                again=c.tick(frame(1_040_000_000,3,.04),board(1_040_000_000),1_040_000_000)
                self.assertEqual(again.reason,d.reason)

    def test_input_and_board_failures_latch(self):
        now=1_000_000_000
        variants=[(None,board()),(frame(),None),
                  (frame(valid=False,reason='sequence_gap'),board()),
                  (frame(received_monotonic_ns=now-300_000_000),board()),
                  (frame(),board(received_ns=now-300_000_000)),
                  (frame(),board(armed=False,fault='OFF')),
                  (frame(),board(pose_mdeg=(25101,0,0,0),target_mdeg=(25101,0,0,0))),
                  (frame(),board(pose_mdeg=(0,5000,0,0),target_mdeg=(0,5000,0,0)))]
        for v,b in variants:
            with self.subTest(v=v,b=b):
                c=ExperimentAController(Settings())
                d=c.tick(v,b,now)
                self.assertTrue(d.emergency_off)
                again=c.tick(frame(),board(),now+20_000_000)
                self.assertTrue(again.emergency_off)
                self.assertEqual(d.reason,again.reason)

    def test_first_armed_pose_must_be_neutral(self):
        c=ExperimentAController(Settings())
        d=c.tick(frame(),board(pose_mdeg=(1000,0,0,0),target_mdeg=(1000,0,0,0)),1_000_000_000)
        self.assertEqual(d.reason,'nonneutral_initial_state')

    def test_source_clock_identity_and_model_gap_fault(self):
        for change in ({'sim_time_s':0.},{'sim_time_s':.2},{'stream_id':'new'},
                       {'sequence_id':0},{'received_monotonic_ns':999_999_999}):
            with self.subTest(change=change):
                c=ExperimentAController(Settings());c.tick(frame(),board(),1_000_000_000)
                v=replace(frame(1_020_000_000,2,.02),**change)
                self.assertTrue(c.tick(v,board(1_020_000_000),1_020_000_000).emergency_off)

    def test_no_silent_specific_force_substitution(self):
        c=ExperimentAController(Settings(acceleration_source='specific_force'))
        self.assertEqual(c.tick(frame(),board(),1_000_000_000).reason,'selected_acceleration_missing_or_invalid')

    def test_host_gap_fails_instead_of_catching_up(self):
        c=ExperimentAController(Settings());c.tick(frame(),board(),1_000_000_000)
        d=c.tick(frame(1_150_000_000,2,.02),board(1_150_000_000),1_150_000_000)
        self.assertEqual(d.reason,'host_loop_gap_or_reversal')

    def test_observed_pressure_or_pose_not_accepted(self):
        c=ExperimentAController(Settings())
        with self.assertRaises(TypeError):
            c.tick(frame(),board(),1_000_000_000,pressure={'UL':1.0})


class SettingsTests(unittest.TestCase):
    def test_shipped_candidate_has_no_participant_parameters(self):
        candidate=Settings.load(Path(__file__).resolve().parents[1]/'config.candidate.json')
        self.assertEqual(candidate.to_dict(),Settings().validate().to_dict())
        self.assertNotIn('human',candidate.to_dict())
        self.assertEqual(candidate.metadata['participant_source'],'none_no_body_compensation')
        self.assertEqual(candidate.pitch_mode,'conform_lean')
        self.assertEqual((candidate.max_yaw_deg,candidate.max_pitch_deg),(15.,6.))
        self.assertEqual(len(candidate.missing_confirmations()),5)

    def test_live_defaults_are_unconfirmed(self):
        self.assertEqual(len(Settings().missing_confirmations()),5)

    def test_limits_unknown_keys_and_duplicate_keys_rejected(self):
        for kwargs in ({'max_yaw_deg':27.001},{'max_pitch_deg':9},{'max_pulse_rate_us_s':101},
                       {'pitch_mode':'guess'},{'full_scale_ay_mps2':float('nan')},
                       {'on_ay_mps2':.1,'off_ay_mps2':.2}):
            with self.subTest(kwargs=kwargs),self.assertRaises(ValueError):Settings(**kwargs).validate()
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory)/'config.json'
            for text in ('{"max_yaw_deg":10,"max_yaw_deg":20}',
                         '{"pressure_fraction":0.2}','{"max_pitch_deg":NaN}'):
                p.write_text(text)
                with self.assertRaises(ValueError):Settings.load(p)

    def test_configuration_round_trip_preserves_model_and_bounds(self):
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory)/'config.json';s=Settings(max_pitch_deg=6)
            p.write_text(json.dumps(s.to_dict()),encoding='utf-8')
            self.assertEqual(Settings.load(p).to_dict(),s.to_dict())


if __name__=='__main__':unittest.main()
