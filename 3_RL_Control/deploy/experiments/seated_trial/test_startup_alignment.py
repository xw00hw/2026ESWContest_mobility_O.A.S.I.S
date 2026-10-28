from pathlib import Path
import os
import sys
import tempfile
import unittest
from startup_alignment import align_startup_input, StartupAlignmentError
from raw_count_guard import RawCountGuard
from occupied_trial import LimitedTransport, StopMonitor


class Clock:
    def __init__(self): self.now=0.
    def __call__(self): return self.now
    def sleep(self,dt): self.now+=dt


class ScheduledRaw:
    timeout=write_timeout=0
    def __init__(self,clock,events=()):
        self.clock=clock;self.events=list(events);self.buf=bytearray(b'old incomplete prefix')
        self.resets=0;self.writes=[];self.closed=False
    @property
    def in_waiting(self):
        while self.events and self.events[0][0]<=self.clock():self.buf.extend(self.events.pop(0)[1])
        return len(self.buf)
    def read(self,n):
        data=bytes(self.buf[:n]);del self.buf[:n];return data
    def write(self,data):self.writes.append(data);return len(data)
    def reset_input_buffer(self):self.resets+=1;self.buf.clear()
    def close(self):self.closed=True


class AlignmentTests(unittest.TestCase):
    def test_partial_tail_waits_for_newline_then_quiet_without_sending(self):
        clock=Clock();raw=ScheduledRaw(clock,[(.001,b'errors=0'),(.006,b'\r\n')])
        report=align_startup_input(raw,clock=clock,sleep=clock.sleep)
        self.assertTrue(report['success']);self.assertEqual(raw.resets,1)
        self.assertEqual(report['discarded_newlines'],1);self.assertEqual(report['discarded_bytes'],10)
        self.assertGreaterEqual(clock(),.016);self.assertEqual(raw.writes,[])
    def test_empty_input_waits_quiet_interval(self):
        clock=Clock();raw=ScheduledRaw(clock)
        report=align_startup_input(raw,clock=clock,sleep=clock.sleep)
        self.assertEqual(report['reason'],'quiet_without_input');self.assertGreaterEqual(clock(),.01)
    def test_incomplete_tail_fails_closed_by_half_second(self):
        clock=Clock();raw=ScheduledRaw(clock,[(.002,b'partial tail')])
        with self.assertRaises(StartupAlignmentError) as caught:align_startup_input(raw,clock=clock,sleep=clock.sleep)
        self.assertLessEqual(clock(),.5);self.assertFalse(caught.exception.report['success']);self.assertEqual(raw.writes,[])
    def test_continuous_stream_has_bounded_timeout(self):
        clock=Clock();raw=ScheduledRaw(clock,[(i*.005,b'#STATS,x=0\r\n') for i in range(101)])
        with self.assertRaises(StartupAlignmentError):align_startup_input(raw,clock=clock,sleep=clock.sleep)
        self.assertLessEqual(clock(),.5);self.assertEqual(raw.writes,[])
    def test_one_actual_reset_before_protocol_and_no_reset_after_arm(self):
        clock=Clock();raw=ScheduledRaw(clock)
        pressure=RawCountGuard(lambda:({f'A1_C{i}':0 for i in range(4)},clock()),clock=clock)
        with tempfile.TemporaryDirectory() as tmp:
            guard=LimitedTransport(raw,StopMonitor(Path(tmp)/'STOP',clock),pressure,alignment_clock=clock,alignment_sleep=clock.sleep)
            guard.prime_startup();before=clock();guard.reset_input_buffer()
            self.assertEqual(clock(),before);self.assertEqual(raw.resets,1)
            guard.authorize_one_arm(1,2);guard.write(b'FB_ARM,1,2\n')
            with self.assertRaises(RuntimeError):guard.reset_input_buffer()
            with self.assertRaises(RuntimeError):guard.prime_startup()
            self.assertEqual(raw.resets,1)
    def test_trace_bounded_and_reads_remain_unfiltered_after_hello(self):
        clock=Clock();raw=ScheduledRaw(clock)
        pressure=RawCountGuard(lambda:({f'A1_C{i}':0 for i in range(4)},clock()),clock=clock)
        with tempfile.TemporaryDirectory() as tmp:
            guard=LimitedTransport(raw,StopMonitor(Path(tmp)/'STOP',clock),pressure,alignment_clock=clock,alignment_sleep=clock.sleep)
            guard.prime_startup();guard.reset_input_buffer();guard.write(b'CONTROL_HELLO\n')
            raw.buf.extend(b'#HELLO,OASIS_FALLBACK,v2,...\r\n')
            self.assertTrue(guard.read(len(raw.buf)).startswith(b'#HELLO'))
            size=len(guard.startup_trace);raw.buf.extend(b'#UNKNOWN,BAD\n')
            self.assertEqual(guard.read(len(raw.buf)),b'#UNKNOWN,BAD\n')
            self.assertEqual(len(guard.startup_trace),size)
            another=LimitedTransport(ScheduledRaw(clock),StopMonitor(Path(tmp)/'OTHER',clock),pressure)
            another._capture_startup(b'x'*10000,'pre_handshake_discard')
            self.assertEqual(another._trace_bytes,8192);self.assertTrue(another.startup_trace_truncated)


@unittest.skipUnless(os.environ.get('OASIS_TRIAL_TEST_RUNTIME'),'Explicit runtime for parser check')
class ParserPreservedTests(unittest.TestCase):
    def test_unknown_line_after_handshake_still_faults(self):
        runtime=Path(os.environ['OASIS_TRIAL_TEST_RUNTIME']);sys.path.insert(0,str(runtime));sys.path.insert(0,str(runtime/'tests'))
        from mock_fb_board import MockClockNs,MockFbBoard,FB
        from oasis_rt.fb_link import FbLink
        clock=MockClockNs();board=MockFbBoard(clock,tune={'yaw_max_mdeg':20000,'yaw_rate_mdeg_s':10000},revision=2)
        with tempfile.TemporaryDirectory() as tmp:
            pg=RawCountGuard(lambda:(link.pressure,link.pressure_at),clock=lambda:clock()/1e9)
            tr=LimitedTransport(board,StopMonitor(Path(tmp)/'STOP',lambda:clock()/1e9),pg,
                                alignment_clock=lambda:clock()/1e9,alignment_sleep=clock.sleep)
            tr.prime_startup();link=FbLink(tr,FB,clock);link.start()
            for _ in range(100):link.poll();link.service();clock.sleep(.002)
            self.assertTrue(link.ready)
            board.emit('#UNKNOWN,BAD');link.poll()
            self.assertIn('Unknown UART line',link.fault)
            link.close(clock.sleep)
            self.assertFalse(any(cmd.startswith('FB_ARM') for _,cmd in board.commands))


if __name__=='__main__':unittest.main()
