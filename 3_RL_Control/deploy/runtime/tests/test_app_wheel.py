"""Model-backed MOZA receiver -> vehicle -> preview -> RL integration; mock I/O only."""
import csv
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

HERE=Path(__file__).resolve().parent
sys.path[:0]=[str(HERE.parent),str(HERE),str(HERE.parents[2]/'beamng')]
from oasis_rt import app, controller as C, predictor as P, wheel_input as W
from oasis_rt.synthetic import SyntheticReceiver
from mock_fb_board import default_pressure
import moza_sender as S

PC='192.0.2.10'
DEFAULT=C.load_config(str(HERE.parent/'config.yaml'))
MODEL=Path(os.environ.get('OASIS_TEST_MODEL_V3',DEFAULT['model_dir'])).resolve()


class TimedWheelTransport:
    """Fake independent Windows clock + raw WinMM packets, at 100 Hz after baseline."""
    def __init__(self, clock, behavior='valid', close_error=False):
        self.clock,self.behavior,self.close_error=clock,behavior,close_error
        self.epoch=None; self.last_bucket=-1; self.seq=0; self.closes=0

    def recvfrom(self,size):
        if self.epoch is None: raise BlockingIOError
        age=self.clock()-self.epoch
        if self.behavior=='stale' and age>=40_000_000: raise BlockingIOError
        bucket=age//10_000_000
        if bucket<=self.last_bucket: raise BlockingIOError
        self.last_bucket=bucket; self.seq+=1
        available=not (self.behavior=='unplugged' and age>=60_000_000)
        sample=S.WheelRead(True,65535,'') if available else S.WheelRead(False,None,'query_error:167')
        # Windows epoch is deliberately unrelated to the virtual Pi clock.
        send=10_000_000_000_000+self.clock()
        row=S.make_packet(sample,source_id='moza-r3',stream_id='fake-windows-session',
                          seq=self.seq,joy_id=0,pc_query_ns=send-1_000_000,pc_send_ns=send)
        return json.dumps(row,separators=(',',':')).encode()[:size],(PC,12345)

    def close(self):
        self.closes+=1
        if self.close_error: raise OSError('injected wheel close failure')


def config(directory, *, confirmed=True, sign=1, source='udp'):
    cfg=C.load_config(str(HERE.parent/'config.yaml'))
    cfg['model_dir']=str(MODEL)
    cfg['actor'].update(backends=['numpy'],check_set=str(MODEL/'check_set.npy'))
    cfg['control'].update(mode='rl',cold_fallback=None)
    cfg['beamng']['steer_sign']=sign
    cfg['pose']['source']='none'; cfg['profile']['from_camera']=False
    cfg['log_dir']=directory
    cfg['wheel']=dict(source=source,priority_prediction=source!='none',listen=['127.0.0.1',0],
                      allow_from=[PC],source_id='moza-r3',device_key=W.DEVICE_KEY,
                      physical_span_deg=450,fresh_ms=100,max_pc_age_ms=50,
                      calibration=dict(confirmed=confirmed,game_gain_deg=90.,game_offset_deg=0.))
    return cfg


@unittest.skipUnless((MODEL/'manifest.json').is_file(), 'requires actual exported v8 model')
class AppWheelTests(unittest.TestCase):
    def run_injected(self,directory,*,behavior='valid',confirmed=True,sign=1,duration=.35,
                     journal_failure=False,close_error=False,vehicle_id=None):
        cfg=config(directory,confirmed=confirmed,sign=sign)
        cfg['wheel']['calibration']['beamng_vehicle_id']=vehicle_id
        made=[]
        def inject(clock,journal):
            transport=TimedWheelTransport(clock,behavior,close_error)
            source=W.make_wheel(cfg,clock,journal,allow_udp=False,transport=transport)
            made.append((source,transport))
            return source
        with patch.object(W.socket,'socket',side_effect=AssertionError('real socket opened')):
            fb,session,device=app.assemble(cfg,'sil',log=lambda *args:None,
                sil_tune=dict(yaw_max_mdeg=20000,yaw_rate_mdeg_s=10000),
                sil_receiver=lambda clock:SyntheticReceiver(clock,route=lambda t:0.),sil_wheel=inject)
            source,transport=made[0]
            original_start=session.policy.start
            def start(s):
                original_start(s)
                transport.epoch=s.clock_ns()
                if journal_failure:
                    original_record=s.link.pressure_log.record
                    fired=[False]
                    def record(event,**fields):
                        if event=='wheel_sample' and not fired[0]:
                            fired[0]=True
                            raise RuntimeError('injected wheel journal failure')
                        return original_record(event,**fields)
                    s.link.pressure_log.record=record
            session.policy.start=start
            def pressure(pose):
                values=default_pressure(pose)
                if transport.epoch is not None and session.clock_ns()-transport.epoch>=120_000_000:
                    for channel,extra in enumerate((11,17,23,29)): values[f'A1_C{channel}']+=extra
                return values
            device.pressure_fn=pressure
            result=session.run(duration,startup_timeout_s=10)
            manifest=app.manifest(cfg,fb,session,result,session.policy)
        output=Path(session.out)
        control=[]
        if (output/'control.csv').is_file():
            with (output/'control.csv').open(newline='') as f:control=list(csv.DictReader(f))
        with (output/'pressure_sweeps.csv').open(newline='') as f:sweeps=list(csv.DictReader(f))
        events=[json.loads(line) for line in (output/'control.jsonl').read_text().splitlines()]
        return manifest,session,device,source,transport,control,sweeps,events

    def assert_clean_exit(self,manifest,session,device,source,transport):
        self.assertTrue(manifest['result']['off_verified'])
        self.assertFalse(device.armed); self.assertEqual(device.pulses,[0,0,0,0])
        self.assertTrue(device.closed); self.assertTrue(source.closed); self.assertEqual(transport.closes,1)
        self.assertTrue(manifest['pressure_logging']['complete'])
        self.assertEqual(manifest['pressure_logging']['accepted_records'],manifest['pressure_logging']['written_records'])
        self.assertEqual(manifest['policy_summary']['mode_final'],'rl')
        self.assertEqual(manifest['policy_summary']['actor_fallbacks'],0)
        self.assertEqual(manifest['policy_summary']['predictor_fallbacks'],0)

    def test_confirmed_raw_wheel_reaches_actual_rl_observation_and_preview_for_both_signs(self):
        for sign in (1,-1):
            with self.subTest(sign=sign),tempfile.TemporaryDirectory() as directory:
                m,s,d,w,t,rows,sweeps,events=self.run_injected(directory,sign=sign)
                self.assertEqual(m['result']['result'],'completed')
                self.assert_clean_exit(m,s,d,w,t)
                self.assertIsInstance(s.policy.ctl.pred,P.MozaPriorityPredictor)
                self.assertIsNotNone(s.policy.ctl.pred.fallback.W)  # actual learned predictor remains loaded
                self.assertGreater(len(rows),3)
                np.testing.assert_allclose([float(r['o24']) for r in rows],sign*math.pi/6,atol=5e-6)
                self.assertTrue(all(sign*float(r[f'o{i}'])>0 for r in rows for i in range(10)))
                reference=P.WheelPreviewPredictor(**s.policy.ctl.pred.params)
                expected,_=reference(dict(t=1.,ay_mps2=0.,ax_mps2=0.,speed_mps=14.,
                                          steer_rad=0.,steer_input_rad=sign*math.pi/2))
                np.testing.assert_allclose([float(rows[0][f'o{i}']) for i in range(10)],expected,atol=5e-6)
                self.assertTrue(all(float(r['o20'])==float(r['o21'])==0 for r in rows))
                self.assertTrue(all(float(r['o55'])==1 for r in rows))
                self.assertTrue(any(float(r['o25'])>0 for r in rows))
                self.assertTrue(any(int(r['A1_C0_raw'])>=1011 for r in sweeps))
                self.assertTrue(all(f'A{bank}_C{channel}_raw' in sweeps[0] for bank in range(3) for channel in range(4)))
                selections=[r for r in events if r['event']=='wheel_selection']
                self.assertTrue(selections)
                self.assertTrue(all(r['selected']=='moza_wheel' and r['beamng_steer_rad']==0 for r in selections))

    def test_stale_and_unplugged_wheel_revert_to_learned_predictor_without_control_fault(self):
        for behavior,reason in [('stale','stale_sample'),('unplugged','device_invalid')]:
            with self.subTest(behavior=behavior),tempfile.TemporaryDirectory() as directory:
                m,s,d,w,t,rows,_,events=self.run_injected(directory,behavior=behavior)
                self.assertEqual(m['result']['result'],'completed'); self.assert_clean_exit(m,s,d,w,t)
                self.assertAlmostEqual(float(rows[0]['o24']),math.pi/6,places=5)
                self.assertEqual(float(rows[-1]['o24']),0)
                counts=m['policy_summary']['predictor']['counts']
                self.assertGreater(counts['moza_wheel'],0); self.assertGreater(counts['beamng_predictor'],0)
                self.assertIn(reason,m['wheel']['reasons'])
                self.assertTrue(all(float(r['o55'])==1 for r in rows))
                reference=C.make_predictor(config(directory),s.vin,s.policy.man)
                expected,_=reference(dict(t=0.,ay_mps2=0.,ax_mps2=0.,speed_mps=14.,
                                          steer_rad=0.,steer_input_rad=0.,yaw_rate_radps=0.,road_valid=0.))
                np.testing.assert_allclose([float(rows[-1][f'o{i}']) for i in range(10)],expected,atol=5e-6)

    def test_unconfirmed_calibration_records_raw_wheel_but_does_not_control(self):
        with tempfile.TemporaryDirectory() as directory:
            m,s,d,w,t,rows,_,events=self.run_injected(directory,confirmed=False)
            self.assertEqual(m['result']['result'],'completed'); self.assert_clean_exit(m,s,d,w,t)
            self.assertEqual(m['policy_summary']['predictor']['counts']['moza_wheel'],0)
            self.assertEqual(w.accepted,0); self.assertGreater(w.validated,0)
            self.assertTrue(all(float(r['o24'])==0 and float(r['o55'])==1 for r in rows))
            self.assertTrue(any(r['event']=='wheel_sample' and r['packet']['axis_raw']==65535 for r in events))

    def test_disabled_source_opens_no_wheel_listener_and_leaves_model_predictor(self):
        with tempfile.TemporaryDirectory() as directory:
            cfg=config(directory,source='none')
            with patch.object(W.socket,'socket',side_effect=AssertionError('real socket opened')):
                m,s,d=app.run(cfg,'sil',.1,log=lambda *args:None,
                    sil_tune=dict(yaw_max_mdeg=20000,yaw_rate_mdeg_s=10000))
            self.assertEqual(m['result']['result'],'completed'); self.assertTrue(m['result']['off_verified'])
            self.assertIsInstance(s.policy.ctl.pred,P.BeamNGPredictor)
            self.assertFalse(m['wheel']['enabled']); self.assertTrue(m['wheel']['closed'])
            self.assertEqual(d.pulses,[0,0,0,0])

    def test_calibration_vehicle_guard_falls_back_without_losing_pressure_or_rl(self):
        for vehicle_id,match in [('1',True),('another-vehicle',False)]:
            with self.subTest(vehicle_id=vehicle_id),tempfile.TemporaryDirectory() as directory:
                m,s,d,w,t,rows,_,events=self.run_injected(directory,vehicle_id=vehicle_id)
                self.assertEqual(m['result']['result'],'completed')
                self.assert_clean_exit(m,s,d,w,t)
                self.assertTrue(all(float(r['o55'])==1 for r in rows))
                selected=[r for r in events if r['event']=='wheel_selection']
                self.assertTrue(selected)
                self.assertTrue(all(r['vehicle_matches_wheel_calibration']==match for r in selected))
                self.assertTrue(all(r['selected']==('moza_wheel' if match else 'beamng_predictor') for r in selected))
                expected=math.pi/6 if match else 0.
                np.testing.assert_allclose([float(r['o24']) for r in rows],expected,atol=5e-6)

    def test_armed_wheel_journal_failure_still_verifies_off_and_closes_resources(self):
        with tempfile.TemporaryDirectory() as directory:
            m,s,d,w,t,_,_,_=self.run_injected(directory,journal_failure=True)
            self.assertIn('injected wheel journal failure',m['result']['result'])
            self.assertTrue(m['result']['off_verified']); self.assertEqual(d.pulses,[0,0,0,0])
            self.assertFalse(d.armed); self.assertTrue(d.closed); self.assertTrue(w.closed)
            self.assertEqual(t.closes,1); self.assertTrue(m['pressure_logging']['complete'])

    def test_wheel_close_failure_keeps_off_result_and_records_cleanup_error(self):
        with tempfile.TemporaryDirectory() as directory:
            m,s,d,w,t,_,_,_=self.run_injected(directory,close_error=True)
            self.assertEqual(m['result']['result'],'completed'); self.assert_clean_exit(m,s,d,w,t)
            self.assertIn('injected wheel close failure',m['wheel']['cleanup_error'])


if __name__=='__main__': unittest.main()
