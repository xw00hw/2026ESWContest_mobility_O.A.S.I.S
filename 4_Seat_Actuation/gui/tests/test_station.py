from pathlib import Path
import csv
import json
import math
import queue
import sys
import tempfile
import threading
import time
import unittest

import cv2
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from protocol import (capabilities, motor_pulses, parse_state, set_command,
                      SequenceTracker, parse_packet)
from devices import SerialWorker
from session import Session, VideoSink, stamp

HELLO = '#HELLO,OASIS_CONTROL,v6,yaw_min_mdeg=0,yaw_limit_mdeg=27000,pitch_limit_mdeg=8000,timeout_ms=1500,motor_rate_mdeg_s=15000,ds_outputs=4,ik_revision=8,crank_zero_us=1200,joints=2'


class ProtocolTests(unittest.TestCase):
    def test_yaw_kinematics_and_upper_lower_sign(self):
        self.assertEqual(motor_pulses(15000,0),[1551,1449,1449,1551])
        self.assertEqual(motor_pulses(0,0),[1800,1200,1200,1800])

    def test_pitch_uses_two_distinct_crank_solutions(self):
        self.assertEqual(motor_pulses(0,8000),[1944,1056,1328,1672])
        self.assertEqual(motor_pulses(0,-8000),[1672,1328,1056,1944])

    def test_independent_motor_bounds(self):
        self.assertEqual(motor_pulses(20000,8000),[1594,1406,1713,1287])
        for y,p in [(-1,0),(27001,0),(0,-8001),(0,8001),(True,0),(0,1.5)]:
            with self.subTest(y=y,p=p),self.assertRaises(ValueError):
                motor_pulses(y,p)

    def test_command_validation(self):
        self.assertEqual(set_command(1,1.25,-.5,2,3),'SET,1,1250,-500,2000,3000')
        for seq,y,p in [(1,math.nan,0),(1,0,math.inf),(-1,0,0),(True,0,0)]:
            with self.subTest(seq=seq,y=y,p=p),self.assertRaises(ValueError):
                set_command(seq,y,p,0,0)

    def test_firmware_capability_mismatch(self):
        self.assertEqual(capabilities(HELLO)['timeout_ms'],1500)
        for line in [HELLO.replace('1500,','3000,'),HELLO.replace(',v6,',',v5,'),
                     HELLO.replace('crank_zero_us=1200,joints=2','crank_zero_us=1500'),'#HELLO,FSR_12_SCAN,v9']:
            with self.assertRaises(ValueError):
                capabilities(line)

    def test_motion_telemetry_integrity(self):
        self.assertEqual(parse_state('#STATE,10,1,0,15000,8000,15000,8000,15000,8000,15000,8000,1675,1325,1587,1413,2,NONE')['left_yaw_mdeg'],15000)
        with self.assertRaises(ValueError):
            parse_state('#STATE,10,1,0,15000,8000,15000,8000,15000,8000,15000,8000,1675,1325,1588,1414,2,NONE')

    def test_left_only_and_right_only_commands_validate_both_pairs(self):
        self.assertEqual(set_command(9,20,8,0,0),'SET,9,20000,8000,0,0')
        state=parse_state('#STATE,10,1,0,20000,8000,0,0,20000,8000,0,0,1800,1406,1200,1287,9,NONE')
        self.assertEqual((state['left_yaw_mdeg'],state['right_yaw_mdeg']),(20000,0))
        with self.assertRaises(ValueError):
            set_command(10,0,0,27.001,0)

    def test_reused_pressure_error_and_wrap(self):
        readings = parse_packet('0,0,0,,1,1200,0,4095,0')[-1]
        self.assertIsNone(readings[0][0])
        tracker = SequenceTracker()
        tracker.accept(0xffffffff,0xffffffff)
        self.assertEqual(tracker.accept(0,0)[0],0x100000000)
        self.assertIsNone(tracker.accept(0,0))
        with self.assertRaises(RuntimeError):
            tracker.accept(0xffffffff,0)


class FakeSerial:
    def __init__(self,*args,**kwargs):
        self.buffer = bytearray()
        self.commands = []
        self.closed = False

    @property
    def in_waiting(self):
        return len(self.buffer)

    def reset_input_buffer(self):
        self.buffer.clear()

    def write(self,data):
        text = data.decode().strip()
        self.commands.append(text)
        replies = {'CONTROL_HELLO':HELLO,'HOLD':'#ACK,HOLD',
                   'DISABLE':'#CONTROL,12,MASK=0,EN=1','PING':'#PONG,12',
                   'STATE':'#STATE,10,0,0,0,0,0,0,0,0,0,0,1800,1200,1200,1800,0,HOLD'}
        if text in replies:
            self.buffer.extend((replies[text]+'\n').encode())
        return len(data)

    def read(self,size):
        time.sleep(.001)
        result = self.buffer[:size]
        del self.buffer[:size]
        return bytes(result)

    def close(self):
        self.closed = True

    def flush(self):
        pass


class WorkerTests(unittest.TestCase):
    def test_interleaved_motion_pressure_and_ack_recording(self):
        with tempfile.TemporaryDirectory() as tmp:
            session = Session(tmp,{},dict(pressure=True))
            worker = SerialWorker('fake',queue.Queue())
            worker.attach_session(session)
            timing = (session.origin_ns,session.metadata['origin_wall_time_ns'])
            worker._line('#CONTROL,12,MASK=15,EN=0',timing)
            worker._line('#ACK,SET,1,15000,8000,10000',timing)
            for channel in range(4):
                worker._line(f'{channel},{10+channel},{channel},1000,0,2000,0,3000,0',timing)
            worker._line('#STATE,20,1,0,15000,8000,15000,8000,15000,8000,15000,8000,1675,1325,1587,1413,1,NONE',timing)
            worker.attach_session(None)
            session.close()
            self.assertEqual(session.counts['pressure_rows'],12)
            self.assertEqual(session.counts['motion_states'],1)
            self.assertEqual(session.counts['commands'],1)
            with (session.directory/'pressure.csv').open() as f:
                rows = list(csv.DictReader(f))
            self.assertEqual(len({row['sensor_id'] for row in rows}),12)

    def test_handshake_and_close_without_automatic_motion(self):
        fake = FakeSerial()
        worker = SerialWorker('fake',queue.Queue(),serial_factory=lambda *a,**kw:fake)
        worker.start()
        deadline = time.monotonic()+1
        while not worker.connected and time.monotonic()<deadline:
            worker.touch()
            time.sleep(.01)
        self.assertTrue(worker.connected)
        worker.close()
        self.assertFalse(any(c=='ARM' or c.startswith('SET,') for c in fake.commands))
        self.assertEqual(fake.commands[-2:],['HOLD','DISABLE'])
        self.assertTrue(fake.closed)

    def test_stop_clears_queued_arm_and_target(self):
        worker = SerialWorker('fake',queue.Queue())
        worker.connected = True
        worker.send('ARM')
        worker.send('SET,1,6000,0')
        worker.send('HOLD')
        self.assertTrue(worker.commands.empty())
        self.assertIsNone(worker._latest_set)
        self.assertTrue(worker.hold_requested.is_set())


class RecordingTests(unittest.TestCase):
    def test_video_pressure_and_motion_share_session_clock(self):
        with tempfile.TemporaryDirectory() as tmp:
            session = Session(tmp,dict(test='synthetic'),dict(pressure=True,video=True))
            sink = VideoSink(session,(64,48),10)
            timing = (session.origin_ns,session.metadata['origin_wall_time_ns'])
            session.row('pressure',dict(seq=1,mcu_ms=10,mcu_extended_ms=10,channel=0,
                bank=0,sensor_id='A0_C0',raw=1234,status=0,voltage_v=1234*3.3/4095),timing)
            session.row('commands',dict(direction='TX',text='SET,1,0,0'),timing)
            for i in range(5):
                sink.submit(np.full((48,64,3),i*30,dtype=np.uint8),i,
                            (timing[0]+i*100000000,timing[1]+i*100000000))
            sink.close(end_monotonic_ns=timing[0]+500000000)
            session.close()
            self.assertFalse((session.directory/'INCOMPLETE').exists())
            self.assertEqual(session.counts['video_frames'],5)
            with (session.directory/'frame_timestamps.csv').open() as f:
                frames = list(csv.DictReader(f))
            with (session.directory/'pressure.csv').open() as f:
                pressure = list(csv.DictReader(f))
            self.assertEqual(frames[0]['host_monotonic_ns'],pressure[0]['host_monotonic_ns'])
            self.assertEqual(json.loads((session.directory/'metadata.json').read_text())['status'],'complete')

    def test_slow_camera_preserves_video_duration(self):
        with tempfile.TemporaryDirectory() as tmp:
            session = Session(tmp,{})
            sink = VideoSink(session,(64,48),30)
            mono,wall = session.origin_ns,session.metadata['origin_wall_time_ns']
            for i in range(3):
                sink.submit(np.zeros((48,64,3),dtype=np.uint8),i,
                            (mono+i*100000000,wall+i*100000000))
            sink.close(end_monotonic_ns=mono+300000000)
            session.close()
            self.assertEqual(session.counts['video_frames'],9)
            self.assertEqual(session.counts['video_source_frames'],3)
            self.assertEqual(session.counts['video_repeated_frames'],6)
            self.assertEqual(session.errors,[])

    def test_failed_session_keeps_incomplete_marker(self):
        with tempfile.TemporaryDirectory() as tmp:
            session = Session(tmp,{})
            session.error('simulated_disk_or_device_failure')
            session.close()
            self.assertTrue((session.directory/'INCOMPLETE').exists())
            self.assertEqual(json.loads((session.directory/'metadata.json').read_text())['status'],'error')

    def test_sessions_are_unique_and_never_overwrite(self):
        with tempfile.TemporaryDirectory() as tmp:
            a,b = Session(tmp,{}),Session(tmp,{})
            self.assertNotEqual(a.directory,b.directory)
            a.close()
            b.close()


if __name__=='__main__':
    unittest.main()
