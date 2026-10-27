import os
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from raw_count_guard import RawCountGuard, attach_pressure_observer, attach_service_gate
import occupied_trial as T
from prepare_candidate import candidate


class RawTests(unittest.TestCase):
    def setUp(self):
        self.now=1.;self.stamp=1.;self.raw={f'A1_C{i}':100 for i in range(4)}
        self.pg=RawCountGuard(lambda:(self.raw,self.stamp),clock=lambda:self.now)
    def test_2499_allowed_2500_individual_right_releases(self):
        self.raw['A1_C0']=2499;self.pg.activate()
        self.assertEqual(self.pg.gate((20,0,20,0)),(20,0,20,0))
        self.raw['A1_C0']=2500
        self.assertEqual(self.pg.gate((20,0,20,0)),(20,0,0.,0.))
        self.assertEqual(self.pg.latched,[False,True])
    def test_each_of_four_channels_maps_correct_side(self):
        for c in range(4):
            raw={f'A1_C{i}':1 for i in range(4)}
            pg=RawCountGuard(lambda:(raw,1.),clock=lambda:1.)
            pg.activate();raw[f'A1_C{c}']=2500
            out=pg.gate((20,6,20,6))
            self.assertEqual(out,(20,6,0.,0.) if c<2 else (0.,0.,20,6))
    def test_latch_remains_after_low_sample_no_averaging_or_baseline(self):
        self.pg.activate();self.raw['A1_C3']=2500;self.pg.gate((20,0,20,0))
        self.raw['A1_C3']=0
        self.assertEqual(self.pg.gate((20,0,20,0)),(0.,0.,20,0))
        self.assertEqual(self.pg.events[0]['raw_counts'],2500)
    def test_zero_count_is_valid_no_contact(self):
        self.raw={k:0 for k in self.raw};self.pg.activate()
        self.assertEqual(self.pg.gate((20,0,20,0)),(20,0,20,0))
    def test_prearm_high_does_not_latch_and_refuses_arm(self):
        self.raw['A1_C1']=2500
        with self.assertRaises(RuntimeError):self.pg.activate()
        self.assertFalse(self.pg.active);self.assertEqual(self.pg.latched,[False,False])
        self.raw['A1_C1']=2499;self.pg.activate()
    def test_missing_invalid_out_of_range_or_stale_releases_both(self):
        for value in (None,float('nan'),float('inf'),-1,4096,True):
            raw={f'A1_C{i}':100 for i in range(4)}
            pg=RawCountGuard(lambda:(raw,1.),clock=lambda:1.);pg.activate();raw['A1_C0']=value
            self.assertEqual(pg.gate((20,0,20,0)),(0.,0.,0.,0.))
        self.pg.activate();self.now=1.301
        self.assertEqual(self.pg.gate((20,0,20,0)),(0.,0.,0.,0.))


class IndependentBoundaryTests(unittest.TestCase):
    def setUp(self): RawTests.setUp(self)
    def test_raw_uart_rechecks_high_at_send_time_with_zero_reported_angle(self):
        class Raw:
            timeout=write_timeout=0
            def __init__(self):self.writes=[]
            def write(self,p):self.writes.append(p);return len(p)
        raw=Raw();counts={f'A1_C{i}':0 for i in range(4)}
        pg=RawCountGuard(lambda:(counts,1.),clock=lambda:1.)
        with tempfile.TemporaryDirectory() as tmp:
            tr=T.LimitedTransport(raw,T.StopMonitor(Path(tmp)/'STOP',lambda:1.),pg)
            tr.authorize_one_arm(1,2);tr.write(b'FB_ARM,1,2\n')
            counts['A1_C2']=2500
            with self.assertRaises(PermissionError):tr.write(b'FB_SET,1,1,1,0,20000,0\n')
            self.assertEqual(raw.writes,[b'FB_ARM,1,2\n'])
            tr.write(b'FB_SET,1,1,0,0,20000,0\n')
            self.assertEqual(pg.latched,[True,False])
    def test_candidate_exact_map_and_raw_threshold_are_required(self):
        source=dict(theta_max_deg=20,pitch_deg={'left':0.,'right':0.},pose={'source':'none'},profile={'from_camera':False},
            rig='static_bench',link={'firmware':'fallback_v2'},control={'mode':'rl','cold_fallback':'fb_v6'},
            pressure={'zero_window_s':2.,'zero_max_std_counts':40,'no_pressure_yaw_cap_deg':8,
                      'channels':{'side_hi_R':'A1_C1','side_hi_L':'A1_C3','side_lo_R':'A1_C0','side_lo_L':'A1_C2'}},
            safety={'arm_deg':2,'mode':'raw','fsr_force_soft_N':60,'fsr_force_hard_N':120,'pad_saturation_counts':None})
        cfg=candidate(source,2)
        self.assertEqual(cfg['pressure']['no_pressure_yaw_cap_deg'],20)
        self.assertEqual(cfg['safety']['pad_saturation_counts'],2500)
        cfg['pressure']['channels']['side_hi_R']='A1_C0'
        with self.assertRaises(ValueError):T.validate_candidate(cfg,require_approval=False)
        cfg=candidate(source,2);cfg['safety']['pad_saturation_counts']=2501
        with self.assertRaises(ValueError):T.validate_candidate(cfg,require_approval=False)
    def test_future_timestamp_releases_both(self):
        self.pg.activate();self.stamp=1.1
        self.assertEqual(self.pg.gate((20,0,20,0)),(0.,0.,0.,0.))
    def test_batched_transient_high_then_low_cannot_be_overwritten(self):
        self.pg.activate()
        records=[dict(type='sample',valid=True,channel=0,raw1=r,status1=0,host_monotonic_ns=i) for i,r in enumerate((100,2500,100))]
        received=[];journal=SimpleNamespace(pressure=lambda r:received.extend(r))
        attach_pressure_observer(journal,self.pg);journal.pressure(records)
        self.assertEqual(received,records)
        self.assertEqual(self.pg.gate((20,0,20,0)),(20,0,0.,0.))
    def test_invalid_sample_latches_both_even_latest_retained_sweep_good(self):
        self.pg.activate();self.pg.observe_records([dict(type='sample',channel=2,valid=False,raw1=None,status1=1)])
        self.assertEqual(self.pg.gate((20,0,20,0)),(0.,0.,0.,0.))


@unittest.skipUnless(os.environ.get('OASIS_TRIAL_TEST_RUNTIME'),'Explicit runtime for real protocol twin')
class EndToEnd(unittest.TestCase):
    def test_full_real_link_high_then_drop_releases_and_records_to_off(self):
        runtime=Path(os.environ['OASIS_TRIAL_TEST_RUNTIME']);sys.path.insert(0,str(runtime));sys.path.insert(0,str(runtime/'tests'))
        from mock_fb_board import MockClockNs,MockFbBoard,FB
        from oasis_rt.fb_link import FbLink
        clock=MockClockNs();high={'on':False};events=[];records=[]
        def pressures(_pose):
            p={f'A{b}_C{c}':1000 for b in range(3) for c in range(4)}
            if high['on']:p['A1_C0']=2500
            return p
        board=MockFbBoard(clock,tune={'yaw_max_mdeg':20000,'yaw_rate_mdeg_s':10000},pressure_fn=pressures,revision=2)
        journal=SimpleNamespace(check=lambda:None,pressure=lambda r:records.extend(r),record=lambda *a,**kw:events.append((a,kw)))
        with tempfile.TemporaryDirectory() as tmp:
            pg=RawCountGuard(lambda:(link.pressure,link.pressure_at),clock=lambda:clock()/1e9,journal=journal)
            guard=T.LimitedTransport(board,T.StopMonitor(Path(tmp)/'STOP',lambda:clock()/1e9),pg,alignment_clock=lambda:clock()/1e9,alignment_sleep=clock.sleep)
            link=FbLink(guard,FB,clock,pressure_log=journal)
            attach_pressure_observer(journal,pg)
            lim=T.TargetLimiter(journal,pg,clock)
            original_send=link.send_targets
            link.send_targets=lambda *p:original_send(*lim.degrees(p))
            attach_service_gate(link,pg,lim)
            guard.prime_startup()
            link.start()
            for _ in range(100):link.poll();link.service();clock.sleep(.002)
            self.assertTrue(link.ready);self.assertTrue(link.board_off());link.freeze()
            guard.authorize_one_arm(1,2);link.arm();link.poll()
            for _ in range(120):
                link.poll();link.send_targets(20,0,20,0);link.authorize();link.service();clock.sleep(.002)
            self.assertTrue(any(cmd.startswith('FB_SET,1,') and ',20000,0,20000,0' in cmd for _,cmd in board.commands))
            high['on']=True
            for _ in range(20):
                link.poll();link.authorize();link.service();clock.sleep(.002)
            self.assertEqual(pg.latched,[False,True])
            count_before=len(records);high['on']=False
            for _ in range(120):
                link.poll();link.send_targets(20,0,20,0);link.authorize();link.service();clock.sleep(.002)
            self.assertGreater(len(records),count_before)
            self.assertEqual(link.target,(20000,0,0,0));self.assertEqual(board.target,[20000,0,0,0])
            self.assertTrue(any(a[0]=='raw_pressure_release_latched' for a,_ in events))
            self.assertTrue(link.close(clock.sleep));self.assertFalse(board.armed);self.assertEqual(board.pulses,[0,0,0,0])
            self.assertEqual(guard.blocked,[])


if __name__=='__main__':unittest.main()
