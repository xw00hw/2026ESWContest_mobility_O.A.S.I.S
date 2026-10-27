from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
import occupied_trial as T


class SettleTests(unittest.TestCase):
    def fake(self):
        self.now=0.;self.events=[];self.order=[]
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.mon=T.StopMonitor(Path(self.tmp.name)/'STOP',clock=lambda:self.now)
        self.mon.arm_attempted()
        self.session=SimpleNamespace(phase='BASELINE')
        self.session.link=SimpleNamespace(fault=None,pressure_log=SimpleNamespace(record=lambda *a,**k:self.events.append((a,k))))
        def wait(dt):
            self.order.append(('wait',dt,self.session.phase))
            end=self.now+dt
            while self.now<end:
                self.now=min(end,self.now+.01)
                if self.mon.check():raise RuntimeError(self.mon.reason)
        def baseline(s):
            self.order.append(('baseline',self.now,s.phase))
            wait(2.)
            return 'baseline_ok'
        self.session.wait_neutral=wait
        self.session.policy=SimpleNamespace(start=baseline)
        T.attach_neutral_settle(self.session,clock_ns=lambda:int(self.now*1e9))
    def test_two_second_settle_precedes_unchanged_baseline_once(self):
        self.fake()
        self.assertEqual(self.session.policy.start(self.session),'baseline_ok')
        self.assertEqual(self.order,[('wait',2.,'NEUTRAL_SETTLE'),('baseline',2.,'BASELINE'),('wait',2.,'BASELINE')])
        self.assertEqual(self.now,4.)
        self.assertEqual([a[0] for a,k in self.events],['neutral_settle_started','neutral_settle_completed'])
        with self.assertRaisesRegex(RuntimeError,'retry'):self.session.policy.start(self.session)
    def test_settle_respects_original_arm_three_hundred_second_budget(self):
        self.fake();self.now=299.
        with self.assertRaisesRegex(RuntimeError,'300s_elapsed'):self.session.policy.start(self.session)
        self.assertFalse(any(x[0]=='baseline' for x in self.order))
        self.assertLessEqual(self.now,300.011)
        self.assertEqual(self.mon.arm_at,0.)
    def test_fault_during_neutral_wait_does_not_enter_baseline(self):
        self.fake()
        def failwait(dt):self.session.link.fault='pressure_logging:test'
        self.session.wait_neutral=failwait
        with self.assertRaisesRegex(RuntimeError,'pressure_logging'):self.session.policy.start(self.session)
        self.assertEqual(self.order,[])
        self.assertEqual(self.session.phase,'NEUTRAL_SETTLE')


if __name__=='__main__':unittest.main()
