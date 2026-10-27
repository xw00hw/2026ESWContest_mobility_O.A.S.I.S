"""run_live integration using real policy/parsers, fake UART/socket and clock.

No device, UDP socket, board, game or camera is opened. Side assignments and
all confirmations below are TEST FIXTURES, not recorded hardware approvals.
The fake board acknowledges targets immediately; this tests runner authority
and transport integration, not motor dynamics or actual firmware instructions.
"""
from __future__ import annotations

import contextlib
import io
import json
from pathlib import Path
import socket
import sys
import tempfile
import unittest
from unittest.mock import patch

PACKAGE_ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(PACKAGE_ROOT))
from fallback import runner, serial_link
from fallback.beamng import BridgeReceiver, BRIDGE_HOST, BRIDGE_PORT, BRIDGE_SCHEMA, CLOCK_ID
from fallback.profile import BenchProfile, Settings


class FakeClock:
    def __init__(self, stop_after=5.8):
        self.started_ns=self.now_ns=1_000_000_000
        self.stop_ns=self.now_ns+round(stop_after*1e9)
        self.interrupted=False

    def perf_counter_ns(self): return self.now_ns
    def time_ns(self): return 1_800_000_000_000_000_000+self.now_ns
    def sleep(self, seconds):
        self.now_ns+=round(seconds*1e9)
        if self.now_ns>=self.stop_ns and not self.interrupted:
            self.interrupted=True
            raise KeyboardInterrupt()

    @property
    def elapsed_ns(self): return self.now_ns-self.started_ns


class FakeUART:
    timeout=write_timeout=0

    def __init__(self, clock, profile, *, pressure_delay=0, pressure_stop=None,
                 suppress_off_response=False):
        self.clock=clock; self.closed=False; self.buffer=bytearray(); self.commands=[]
        self.scan=False; self.armed=False; self.sid=0; self.command_id=0
        self.pose=(0,0,0,0); self.sequence=0; self.channel=0
        self.next_sample_ns=clock.now_ns; self.next_state_ns=clock.now_ns
        self.pressure_delay_ns=round(pressure_delay*1e9)
        self.pressure_stop_ns=None if pressure_stop is None else round(pressure_stop*1e9)
        self.suppress_off_response=suppress_off_response; self.silent=False
        self.first_complete_pressure_ns=None
        self.raw={c:round(profile.base[c]['mean_v']*4095/3.3) for c in serial_link.SENSORS}

    def emit(self,line): self.buffer.extend((line+'\r\n').encode('ascii'))

    def state_line(self):
        # Synthetic coherent command state; no actual shaft/force measurement.
        pulses=serial_link.NEUTRAL_US if self.armed else (0,0,0,0)
        fields=(self.clock.now_ns//1_000_000,int(self.armed),0,*self.pose,*self.pose,
                *pulses,self.command_id,'NONE' if self.armed else 'OFF')
        return '#STATE,'+','.join(map(str,fields))

    def write(self,data):
        for command in data.decode('ascii').splitlines():
            if not command: continue
            self.commands.append((self.clock.now_ns,command))
            fields=command.split(',')
            if self.silent: continue
            if command=='CONTROL_HELLO':
                self.emit('#HELLO,OASIS_FALLBACK,v1,'+','.join(
                    f'{key}={value}' for key,value in serial_link.HELLO_FIELDS.items())+
                    ',session_floor='+str(self.sid))
            elif command=='HELLO': self.emit(serial_link.FSR_HELLO)
            elif command=='DISABLE':
                self.scan=False; self.emit('#CONTROL,12,MASK=0,EN=1')
            elif command=='START_12,15':
                self.scan=True
                self.next_sample_ns=max(self.clock.now_ns,self.clock.started_ns+self.pressure_delay_ns)
                self.emit('#CONTROL,12,MASK=15,EN=0')
            elif command=='PING': self.emit('#PONG,12')
            elif command=='STATE': self.emit(self.state_line())
            elif fields[0]=='FB_ARM':
                self.sid=int(fields[1]); self.armed=True; self.pose=(0,0,0,0)
                self.emit('#ACK,'+command)
            elif fields[0]=='FB_SET':
                assert self.armed and int(fields[1])==self.sid
                self.command_id=int(fields[2]); self.pose=tuple(map(int,fields[3:]))
                self.emit('#ACK,'+command)
            elif fields[0]=='FB_PING':
                assert self.armed and int(fields[1])==self.sid
                self.emit('#ACK,'+command)
            elif command=='FB_OFF':
                if self.suppress_off_response:
                    self.silent=True; self.buffer.clear(); continue
                self.armed=False; self.emit('#ACK,FB_OFF')
            else: raise AssertionError('Unexpected runner command: '+command)
        return len(data)

    def generate(self):
        if self.silent: return
        now=self.clock.now_ns
        if now>=self.next_state_ns:
            self.emit(self.state_line()); self.next_state_ns=now+200_000_000
        active=self.pressure_stop_ns is None or self.clock.elapsed_ns<self.pressure_stop_ns
        while self.scan and active and self.next_sample_ns<=now:
            c=self.channel
            values=[self.sequence,self.next_sample_ns//1_000_000,c]
            for bank in range(3): values.extend((self.raw[f'A{bank}_C{c}'],0))
            self.emit(','.join(map(str,values)))
            if c==3 and self.first_complete_pressure_ns is None:
                self.first_complete_pressure_ns=now
            self.sequence+=1; self.channel=(c+1)%4; self.next_sample_ns+=2_500_000

    @property
    def in_waiting(self): self.generate(); return len(self.buffer)
    def read(self,count):
        data=bytes(self.buffer[:count]); del self.buffer[:count]; return data
    def reset_input_buffer(self): self.buffer.clear()
    def close(self): self.closed=True


class FakeSocket:
    def __init__(self,clock,*,source_invalid=None,source_stop=None,ay=2.):
        self.clock=clock; self.closed=False; self.bound=None
        self.next_ns=clock.now_ns; self.sequence=0; self.injected=False
        self.source_invalid_ns=None if source_invalid is None else round(source_invalid*1e9)
        self.source_stop_ns=None if source_stop is None else round(source_stop*1e9)
        self.ay=ay; self.queued=[]

    def setsockopt(self,*unused): pass
    def setblocking(self,value): self.blocking=value
    def bind(self,address):
        if address!=(BRIDGE_HOST,BRIDGE_PORT): raise AssertionError('Unexpected bind target')
        self.bound=address
    def close(self): self.closed=True

    def packet(self,*,invalid=False,offset_ns=0):
        self.sequence+=1
        row=dict(host_monotonic_ns=self.clock.now_ns+offset_ns,host_wall_time_ns=self.clock.time_ns(),
            sim_time_s=self.sequence*.02,stream_id='offline-stream',vehicle_id=42,
            sequence_id=self.sequence,source_kind='beamng',linear_acc_y_mps2=self.ay,
            linear_acc_x_mps2=0.,speed_mps=10.,steering_angle_deg=20.,throttle_input=.2,
            brake_input=0.,derivative_valid=0 if invalid else 1,
            derivative_reason='injected_invalid' if invalid else 'ok',config_match=1,
            sample_dt_s=.02,missing_packets_before=0)
        packet=dict(schema=BRIDGE_SCHEMA,collector_version='0.3',csv_revision=2,clock=CLOCK_ID,
            forwarded_monotonic_ns=self.clock.now_ns,
            provenance=dict(session_id='offline-bench',config_sha256='a'*64,
                            source_sha256={'fixture':'b'*64}),row=row)
        return json.dumps(packet,separators=(',',':')).encode('ascii'),(BRIDGE_HOST,12345)

    def recvfrom(self,count):
        if self.queued: return self.queued.pop(0)
        if self.source_stop_ns is not None and self.clock.elapsed_ns>=self.source_stop_ns:
            raise BlockingIOError()
        if self.clock.now_ns<self.next_ns: raise BlockingIOError()
        self.next_ns=self.clock.now_ns+20_000_000
        invalid=(self.source_invalid_ns is not None and
                 self.clock.elapsed_ns>=self.source_invalid_ns and not self.injected)
        if invalid:
            self.injected=True
            bad=self.packet(invalid=True,offset_ns=-1)
            # A subsequent good sample in the SAME receive batch cannot hide bad.
            self.queued.append(self.packet())
            return bad
        return self.packet()


def approved_test_settings():
    flags=('hardware_matches_bench','sensor_mapping_confirmed','positive_yaw_supports_occupant',
           'neutral_start_clear','same_recorded_participant','initial_range_checked')
    return Settings(left_channels=('A1_C1',),right_channels=('A1_C0',),
                    confirmations={name:True for name in flags})


def synthetic_positive_profile():
    """Invented linear response for orchestration tests, never bench evidence."""
    profile=BenchProfile()
    guards=approved_test_settings().guard_channels
    for channel in guards:
        profile.base[channel]['std_v']=.001
    for side in ('left','right'):
        angles,points=profile.points[side]
        for index,point in enumerate(points):
            for channel in guards:
                point['channels'][channel]['mean_v']=profile.base[channel]['mean_v']+.01*angles[index]
                profile.envelopes[side][channel][index]=.01*angles[index]
    return profile


class RunnerTests(unittest.TestCase):
    def run_fixture(self,*,settings=None,stop_after=5.8,pressure_delay=0,pressure_stop=None,
                    suppress_off_response=False,source_invalid=None,source_stop=None,ay=2.,profile=None):
        settings=settings or approved_test_settings()
        profile=profile or synthetic_positive_profile(); clock=FakeClock(stop_after)
        uart=FakeUART(clock,profile,pressure_delay=pressure_delay,pressure_stop=pressure_stop,
                      suppress_off_response=suppress_off_response)
        udp=FakeSocket(clock,source_invalid=source_invalid,source_stop=source_stop,ay=ay)
        error=None
        with tempfile.TemporaryDirectory() as temp:
            with patch.object(runner,'time',clock), \
                 patch.object(runner,'BridgeReceiver',lambda:BridgeReceiver(socket_factory=lambda *a:udp)), \
                 patch.object(serial_link,'open_serial',return_value=uart) as opening, \
                 patch.object(socket,'socket',side_effect=AssertionError('Real socket forbidden')), \
                 contextlib.redirect_stdout(io.StringIO()):
                try: runner.run_live('OFFLINE_FAKE_ONLY',temp,settings,profile)
                except Exception as exc: error=exc
            path=Path(temp)/'manifest.json'
            manifest=json.loads(path.read_text(encoding='utf-8')) if path.exists() else None
            log=Path(temp)/'control.jsonl'
            events=[json.loads(line) for line in log.read_text(encoding='utf-8').splitlines()] if log.exists() else []
        return dict(error=error,manifest=manifest,events=events,uart=uart,socket=udp,clock=clock,opens=opening.call_count)

    def assert_stopped(self,result,*,verified=True):
        self.assertTrue(result['uart'].closed)
        self.assertTrue(result['socket'].closed)
        self.assertEqual(result['manifest']['pwm_off_telemetry_verified'],verified)
        commands=[c for _,c in result['uart'].commands]
        off_index=commands.index('FB_OFF')
        self.assertFalse(any(c.startswith(('FB_ARM,','FB_SET,','FB_PING,')) for c in commands[off_index+1:]))

    def test_startup_arm_baseline_direction_and_ctrl_c_end_to_end(self):
        result=self.run_fixture()
        self.assertIsNone(result['error'])
        self.assertEqual(result['manifest']['result'],'operator_stop')
        commands=result['uart'].commands
        arms=[(ns,c) for ns,c in commands if c.startswith('FB_ARM,')]
        self.assertEqual(len(arms),1)
        self.assertGreaterEqual(arms[0][0]-result['clock'].started_ns,2_000_000_000)
        sets=[(ns,tuple(map(int,c.split(',')[3:]))) for ns,c in commands if c.startswith('FB_SET,')]
        self.assertTrue(sets,'Expected actual runner to send a feedforward target')
        self.assertGreaterEqual(sets[0][0]-arms[0][0],2_000_000_000)
        self.assertTrue(all(pose[0]==0 and pose[1]==0 and pose[3]==0 for _,pose in sets))
        self.assertTrue(any(pose[2]>0 for _,pose in sets))
        self.assertFalse(any(c.startswith('FB_PING,') and ns<arms[0][0] for ns,c in commands))
        modes={event['decision']['mode'] for event in result['events'] if event['event']=='control'}
        self.assertTrue({'BASELINE','RUN'}<=modes)
        self.assert_stopped(result)

    def test_negative_acceleration_selects_left_only(self):
        result=self.run_fixture(ay=-2.)
        self.assertIsNone(result['error'])
        poses=[tuple(map(int,c.split(',')[3:])) for _,c in result['uart'].commands if c.startswith('FB_SET,')]
        self.assertTrue(any(p[0]>0 for p in poses))
        self.assertTrue(all(p[1:]==(0,0,0) for p in poses))
        self.assert_stopped(result)

    def test_runner_transmits_twenty_degree_endpoint_on_each_side(self):
        # Artificial neutral pressure and immediate command-state ACKs prove
        # orchestration only; they do not certify physical pressure at 20°.
        for ay,index in ((4.,2),(-4.,0)):
            with self.subTest(ay=ay):
                result=self.run_fixture(ay=ay,stop_after=16.)
                self.assertIsNone(result['error'])
                targets=[(ns,tuple(map(int,command.split(',')[3:])))
                         for ns,command in result['uart'].commands if command.startswith('FB_SET,')]
                self.assertTrue(targets)
                self.assertEqual(max(pose[index] for _,pose in targets),20000)
                for _,pose in targets:
                    self.assertEqual(pose[1::2],(0,0))
                    self.assertEqual(pose[2-index],0)
                    self.assertTrue(0 <= pose[index] <= 20000)
                for (previous_ns,previous),(now_ns,pose) in zip(targets,targets[1:]):
                    # Quantization is 1 mdeg; actual controller advances no
                    # faster than 2°/s even when UART SETs skip control ticks.
                    self.assertLessEqual(abs(pose[index]-previous[index]),
                                         2000*(now_ns-previous_ns)/1e9+1)
                self.assert_stopped(result)

    def test_invalid_source_followed_by_good_same_batch_faults_and_stops(self):
        result=self.run_fixture(source_invalid=5.0)
        self.assertIsInstance(result['error'],RuntimeError)
        self.assertEqual(str(result['error']),'vehicle:invalid_derivative')
        self.assertTrue(result['socket'].queued,'Good successor must not hide the bad row')
        self.assert_stopped(result)

    def test_source_stale_and_pressure_stale_stop_without_further_lease(self):
        for kwargs,reason in ((dict(source_stop=5.),'vehicle_stale'),
                              (dict(pressure_stop=5.),'policy:pressure_invalid_or_stale')):
            with self.subTest(kwargs=kwargs):
                result=self.run_fixture(**kwargs)
                self.assertIsInstance(result['error'],RuntimeError)
                self.assertEqual(str(result['error']),reason)
                self.assert_stopped(result)

    def test_cached_old_off_state_is_not_verified_on_lost_off_response(self):
        result=self.run_fixture(stop_after=.35,suppress_off_response=True)
        self.assertIsNone(result['error'])
        self.assertFalse(result['manifest']['arm_attempted'])
        self.assert_stopped(result,verified=False)

    def test_missing_confirmation_blocks_before_socket_or_uart_open(self):
        result=self.run_fixture(settings=Settings(left_channels=('A1_C1',),right_channels=('A1_C0',)))
        self.assertIsInstance(result['error'],ValueError)
        self.assertIn('blocked before opening any device',str(result['error']))
        self.assertEqual(result['opens'],0)
        self.assertIsNone(result['socket'].bound)
        self.assertEqual(result['uart'].commands,[])

    def test_recorded_mapping_without_mechanical_direction_confirmation_blocks(self):
        settings=approved_test_settings()
        from dataclasses import replace
        confirmations={**settings.confirmations,'positive_yaw_supports_occupant':False}
        settings=replace(settings,left_channels=('A1_C2','A1_C3'),right_channels=('A1_C0','A1_C1'),
                         confirmations=confirmations)
        result=self.run_fixture(settings=settings,profile=BenchProfile())
        self.assertIsInstance(result['error'],ValueError)
        self.assertIn('confirm:positive_yaw_supports_occupant',str(result['error']))
        self.assertEqual(result['opens'],0)
        self.assertIsNone(result['socket'].bound)
        self.assertEqual(result['uart'].commands,[])

    def test_late_pressure_requires_qualified_two_second_joint_startup(self):
        result=self.run_fixture(pressure_delay=1.9,stop_after=4.3)
        self.assertIsNone(result['error'])
        arms=[ns for ns,c in result['uart'].commands if c.startswith('FB_ARM,')]
        self.assertEqual(len(arms),1)
        self.assertGreaterEqual(arms[0]-result['uart'].first_complete_pressure_ns,2_000_000_000)
        self.assert_stopped(result)

    def test_invalid_startup_row_resets_dwell_even_with_good_same_batch(self):
        result=self.run_fixture(source_invalid=1.,stop_after=5.4)
        self.assertIsNone(result['error'])
        arms=[ns for ns,c in result['uart'].commands if c.startswith('FB_ARM,')]
        self.assertEqual(len(arms),1)
        self.assertGreaterEqual(arms[0]-result['clock'].started_ns,3_000_000_000)
        self.assert_stopped(result)


if __name__=='__main__': unittest.main(verbosity=2)
