import copy
import os
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest

import occupied_trial as T
from prepare_candidate import candidate


class FakeRaw:
    timeout=write_timeout=0
    in_waiting=0
    def __init__(self):self.writes=[];self.closed=False
    def write(self,v):self.writes.append(v);return len(v)
    def read(self,n):return b''
    def close(self):self.closed=True
    def reset_input_buffer(self):pass


class GuardTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.now=0.;self.mon=T.StopMonitor(Path(self.temp.name)/'STOP',lambda:self.now)
        self.raw=FakeRaw();self.pressure=T.RawCountGuard(lambda:({k:100 for k in ('A1_C0','A1_C1','A1_C2','A1_C3')},self.now),clock=lambda:self.now)
        self.guard=T.LimitedTransport(self.raw,self.mon,self.pressure)
    def arm(self):
        self.guard.authorize_one_arm(7,3);self.guard.write(b'FB_ARM,7,3\n')
    def test_all_motion_denied_before_exact_session_authorization(self):
        for p in (b'FB_ARM,7,3\n',b'FB_SET,7,1,100,0,100,0\n',b'FB_PING,7,1\n'):
            with self.assertRaises(PermissionError):self.guard.write(p)
        self.assertEqual(self.raw.writes,[])
    def test_twenty_degree_boundary_allowed_pitch_and_overflow_rejected(self):
        self.arm();self.guard.write(b'FB_SET,7,1,20000,0,0,0\n')
        self.guard.write(b'FB_SET,7,2,0,0,20000,0\n')
        for p in (b'FB_SET,7,3,20001,0,0,0\n',b'FB_SET,7,3,0,1,0,0\n',
                  b'FB_SET,7,3,0,0,0,-1\n',b'FB_SET,8,3,0,0,0,0\n',b'SET,0,0\n'):
            before=len(self.raw.writes)
            with self.assertRaises(PermissionError):self.guard.write(p)
            self.assertEqual(len(self.raw.writes),before)
    def test_multiline_injection_and_partial_line_rejected_atomically(self):
        self.arm()
        for p in (b'STATE\nFB_SET,7,1,20001,0,0,0\n',b'FB_SET,7,1,0,0,0,0',b'FB_ARM,7,3\r\n'):
            before=len(self.raw.writes)
            with self.assertRaises(PermissionError):self.guard.write(p)
            self.assertEqual(len(self.raw.writes),before)
    def test_no_second_arm_or_wrong_revision(self):
        self.guard.authorize_one_arm(7,3)
        with self.assertRaises(PermissionError):self.guard.write(b'FB_ARM,7,4\n')
        self.guard.write(b'FB_ARM,7,3\n')
        with self.assertRaises(PermissionError):self.guard.write(b'FB_ARM,7,3\n')
    def test_stop_file_blocks_motion_but_allows_emergency_off(self):
        self.arm();self.mon.path.touch()
        with self.assertRaises(PermissionError):self.guard.write(b'FB_PING,7,1\n')
        self.guard.write(b'\nFB_OFF\nDISABLE\n')
        self.assertEqual(self.mon.reason,'operator_stop_file')
    def test_three_hundred_seconds_includes_initial_arm_and_baseline(self):
        self.arm();self.now=299.999;self.guard.write(b'FB_PING,7,1\n')
        self.now=300
        with self.assertRaises(PermissionError):self.guard.write(b'FB_SET,7,2,1,0,1,0\n')
        self.assertEqual(self.mon.reason,'occupied_trial_300s_elapsed')
        with self.assertRaises(PermissionError):self.guard.write(b'FB_PING,7,2\n')
        with self.assertRaises(PermissionError):self.guard.write(b'FB_ARM,7,3\n')
        self.guard.write(b'\nFB_OFF\nDISABLE\n')
        self.assertEqual(self.raw.writes[-1],b'\nFB_OFF\nDISABLE\n')


class LimiterTests(unittest.TestCase):
    def test_clamp_preserves_open_command_logs_all_clamps(self):
        rows=[];pg=T.RawCountGuard(lambda:({k:100 for k in ('A1_C0','A1_C1','A1_C2','A1_C3')},0),clock=lambda:0)
        lim=T.TargetLimiter(SimpleNamespace(record=lambda *a,**kw:rows.append((a,kw))),pg,lambda:123)
        self.assertEqual(lim.degrees((20,6,-1,-6)),(20.,0.,0.,0.))
        self.assertEqual(lim.degrees((0,0,0,0)),(0.,0.,0.,0.))
        self.assertEqual(lim.clamps,1);self.assertEqual(len(rows),1)
    def test_nonfinite_cannot_be_clamped_into_valid_motion(self):
        lim=T.TargetLimiter(SimpleNamespace(record=lambda *a,**kw:None),T.RawCountGuard(lambda:({},0)))
        for v in (float('nan'),float('inf'),float('-inf')):
            with self.assertRaises(ValueError):lim.degrees((v,0,0,0))


class CandidateTests(unittest.TestCase):
    def source(self):
        return dict(theta_max_deg=20,pitch_deg={'left':0.,'right':0.},pose={'source':'none'},
            profile={'from_camera':False},rig='static_bench',link={'firmware':'fallback_v2'},
            control={'mode':'rl','cold_fallback':'fb_v6'},pressure={'zero_window_s':2.,'zero_max_std_counts':40,
            'no_pressure_yaw_cap_deg':8,'channels':{'side_hi_R':'A1_C1','side_hi_L':'A1_C3','side_lo_R':'A1_C0','side_lo_L':'A1_C2'},'fsr_curve':'curve.json'},
            safety={'arm_deg':2,'mode':'raw','fsr_force_soft_N':60,'fsr_force_hard_N':120,'pad_saturation_counts':2200})
    def test_preparation_never_authorizes_and_does_not_change_pressure_calibration(self):
        source=self.source();before=copy.deepcopy(source);cfg=candidate(source,2)
        self.assertEqual(source,before)
        self.assertEqual(cfg['theta_max_deg'],20)
        self.assertTrue(all(v is False for v in cfg['confirmations'].values()))
        self.assertIsNone(cfg['deployment']['approved_trial'])
        self.assertEqual(cfg['pressure']['fsr_curve'],source['pressure']['fsr_curve'])
        self.assertEqual(cfg['safety']['pad_saturation_counts'],2500)
        self.assertEqual(cfg['safety']['fsr_force_hard_N'],120)
        with self.assertRaises(ValueError):T.validate_candidate(cfg)
    def test_all_specific_approval_and_stop_confirmation_required(self):
        cfg=candidate(self.source(),2);cfg['confirmations']={k:True for k in T.CONFIRM}
        cfg['deployment'].update(approved_trial=T.APPROVAL,occupied_trial_accepted=True,operator_stop_ready=True)
        T.validate_candidate(cfg)
        cfg['deployment']['approved_trial']='unoccupied_yaw15_rate10_pitch0_45s'
        with self.assertRaises(ValueError):T.validate_candidate(cfg)
    def test_reject_model_range_change_and_relaxed_baseline(self):
        source=self.source();source['theta_max_deg']=2
        with self.assertRaises(ValueError):candidate(source,2)
        source=self.source();source['pressure']['zero_max_std_counts']=1000
        with self.assertRaises(ValueError):candidate(source,2)


@unittest.skipUnless(os.environ.get('OASIS_TRIAL_TEST_RUNTIME'),'Explicit source for protocol twin')
class ProtocolTests(unittest.TestCase):
    def imports(self):
        runtime=Path(os.environ['OASIS_TRIAL_TEST_RUNTIME']);sys.path.insert(0,str(runtime));sys.path.insert(0,str(runtime/'tests'))
        from mock_fb_board import MockFbBoard,MockClockNs,FB
        from oasis_rt.fb_link import FbLink
        return MockFbBoard,MockClockNs,FB,FbLink
    def test_mock_board_never_receives_overrange_or_nonzero_pitch(self):
        Board,Clock,fb,Link=self.imports();clock=Clock();board=Board(clock,tune={'yaw_rate_mdeg_s':10000,'yaw_max_mdeg':20000})
        with tempfile.TemporaryDirectory() as tmp:
            mon=T.StopMonitor(Path(tmp)/'STOP',lambda:clock()/1e9)
            pg=T.RawCountGuard(lambda:(link.pressure,link.pressure_at),clock=lambda:clock()/1e9)
            guard=T.LimitedTransport(board,mon,pg,alignment_clock=lambda:clock()/1e9,alignment_sleep=clock.sleep)
            guard.prime_startup()
            link=Link(guard,fb,clock)
            link.start()
            for _ in range(100):
                link.poll();clock.sleep(.002)
                if link.ready:break
            self.assertTrue(link.ready);link.freeze()
            for _ in range(15):link.poll();link.service();clock.sleep(.002)
            guard.authorize_one_arm(link.hello['session_floor']+1,link.tuning['revision'])
            link.arm();link.poll()
            lim=T.TargetLimiter(SimpleNamespace(record=lambda *a,**kw:None),pg,clock)
            for _ in range(300):
                link.poll();link.send_targets(*lim.degrees((20,8,18,-8)));link.authorize();link.service();clock.sleep(.002)
            actual=[cmd.split(',') for _,cmd in board.commands if cmd.startswith('FB_SET,')]
            self.assertTrue(actual)
            self.assertTrue(all(0<=int(f[3])<=20000 and f[4]=='0' and 0<=int(f[5])<=20000 and f[6]=='0' for f in actual))
            self.assertTrue(link.close(clock.sleep));self.assertFalse(board.armed)
            self.assertEqual(board.pulses,[0,0,0,0])
    def test_low_angle_pressure_interlock_still_releases(self):
        self.imports()
        from oasis_rt.safety import Interlock
        ilock=Interlock(dict(fsr_force_soft_N=60,fsr_force_hard_N=120,arm_deg=0,hold_s=.5,mode='raw'),222)
        self.assertEqual(ilock(1,(2,2),(1,1),(121,0),0),[0.,2])


if __name__=='__main__':unittest.main()
