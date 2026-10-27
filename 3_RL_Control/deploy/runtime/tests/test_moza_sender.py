"""Raw wheel sender contracts using fake WinMM, clock and UDP; no real device."""
import json
from pathlib import Path
import socket
import sys
import tempfile
import unittest
from unittest.mock import patch

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parents[2]/'beamng'))
import moza_sender as S


class Api:
    def __init__(self):
        self.caps=dict(mid=0x346e,pid=5,xmin=0,xmax=65535,axes=6,name='test wheel')
        self.caps_code=self.x_code=0; self.raw=32768; self.calls=[]
    def get_caps(self, joy_id): self.calls.append(('caps',joy_id)); return self.caps_code,self.caps
    def get_x(self, joy_id): self.calls.append(('x',joy_id)); return self.x_code,self.raw


class Clock:
    def __init__(self): self.now=1_000_000_000
    def __call__(self): return self.now
    def sleep(self, seconds): self.now+=round(seconds*1e9)


class Reader:
    def __init__(self,clock,reads=None):
        self.clock=clock; self.calls=[]; self.reads=reads or [S.WheelRead(True,12345,'')]
    def read(self,joy_id):
        self.calls.append((self.clock.now,joy_id))
        self.clock.now+=700_000
        return self.reads[min(len(self.calls)-1,len(self.reads)-1)]


class Udp:
    def __init__(self,drop_first=False): self.rows=[]; self.blocking=None; self.closed=False; self.drop_first=drop_first
    def setblocking(self,value): self.blocking=value
    def sendto(self,data,dest):
        self.rows.append((data,dest))
        if self.drop_first and len(self.rows)==1: raise BlockingIOError('injected send drop')
        return len(data)
    def close(self): self.closed=True


class MozaTests(unittest.TestCase):
    def test_exact_caps_checked_every_read_without_selecting_another_device(self):
        api=Api(); reader=S.WinMMReader(api)
        self.assertEqual(reader.read().axis_raw,32768)
        api.caps['pid']=4
        read=reader.read()
        self.assertFalse(read.valid); self.assertIsNone(read.axis_raw)
        self.assertEqual(read.reason,'device_mismatch')
        self.assertEqual(api.calls,[('caps',0),('x',0),('caps',0)])

    def test_missing_device_recovers_at_same_explicit_index(self):
        api=Api(); reader=S.WinMMReader(api)
        api.caps_code=167
        self.assertFalse(reader.read().valid)
        api.caps_code=0; api.raw=65535
        self.assertEqual(reader.read().axis_raw,65535)
        api.x_code=167
        self.assertFalse(reader.read().valid)
        api.x_code=0; api.raw=0
        self.assertEqual(reader.read().axis_raw,0)
        self.assertTrue(all(joy==0 for _,joy in api.calls))

    def test_bad_range_or_value_never_becomes_valid(self):
        api=Api(); reader=S.WinMMReader(api)
        api.caps['xmax']=32767
        self.assertEqual(reader.read().reason,'axis_range_mismatch')
        api.caps['xmax']=65535
        for raw in (-1,65536,False,1.5):
            api.raw=raw
            with self.subTest(raw=raw): self.assertFalse(reader.read().valid)

    def test_packet_is_raw_exact_contract_with_conservative_query_time(self):
        packet=S.make_packet(S.WheelRead(True,16384,''),source_id='moza-r3',stream_id='abc',
                             seq=1,joy_id=0,pc_query_ns=100,pc_send_ns=200)
        self.assertEqual(set(packet),{'schema','source_id','stream_id','seq','device_key','pc_query_ns',
                                      'pc_send_ns','valid','axis_raw','axis_min','axis_max','reason'})
        self.assertEqual(packet['axis_raw'],16384)
        self.assertEqual(packet['device_key'],'winmm:346e:0005:0:x')
        self.assertEqual(packet['reason'],'')
        self.assertLess(packet['pc_query_ns'],packet['pc_send_ns'])
        missing=S.make_packet(S.WheelRead(False,None,'caps_error:167'),source_id='moza-r3',stream_id='abc',
                              seq=2,joy_id=0,pc_query_ns=300,pc_send_ns=400)
        self.assertIsNone(missing['axis_raw']); self.assertFalse(missing['valid'])

    def test_packet_rejects_regressing_clock_and_inconsistent_validity(self):
        args=dict(source_id='moza-r3',stream_id='abc',seq=1,joy_id=0,pc_query_ns=100,pc_send_ns=200)
        with self.assertRaises(ValueError):
            S.make_packet(S.WheelRead(True,1,''),**dict(args,pc_send_ns=99))
        for sample in (S.WheelRead(True,None,''),S.WheelRead(False,0,'missing'),
                       S.WheelRead(False,None,''),S.WheelRead(True,0,'unexpected')):
            with self.subTest(sample=sample),self.assertRaises(ValueError): S.make_packet(sample,**args)

    def run_sender(self,path,reader=None,udp=None,clock=None,**kwargs):
        clock=clock or Clock(); reader=reader or Reader(clock); udp=udp or Udp()
        result=S.serve('fake-destination',log_path=path,seconds=.05,reader=reader,
                       socket_factory=lambda *args:udp,
                       destination_resolver=lambda *args:(socket.AF_INET,('fake-destination',64258)),
                       clock_ns=clock,sleep=clock.sleep,log=lambda *args:None,**kwargs)
        return result,reader,udp

    def test_sender_timing_logging_and_cleanup(self):
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'wheel.jsonl'
            result,reader,udp=self.run_sender(path)
            packets=[json.loads(line) for line in path.read_text().splitlines()]
            self.assertEqual(result['samples'],5); self.assertEqual(result['sent'],5)
            self.assertEqual([row['seq'] for row in packets],[1,2,3,4,5])
            self.assertEqual([b[0]-a[0] for a,b in zip(reader.calls,reader.calls[1:])],[10_000_000]*4)
            self.assertTrue(all(row['pc_send_ns']-row['pc_query_ns']==700_000 for row in packets))
            self.assertEqual([json.loads(data) for data,_ in udp.rows],packets)
            self.assertFalse(udp.blocking); self.assertTrue(udp.closed)
            meta=json.loads(Path(str(path)+'.meta.json').read_text())
            self.assertFalse(meta['physical_degrees_in_packet'])
            self.assertIn('not hardware capture time',meta['timestamp_semantics'])
            self.assertEqual(json.loads(Path(str(path)+'.summary.json').read_text())['samples'],5)

    def test_unavailable_samples_and_send_drop_do_not_abort_optional_sender(self):
        with tempfile.TemporaryDirectory() as temp:
            clock=Clock(); reader=Reader(clock,[S.WheelRead(False,None,'missing'),S.WheelRead(True,30000,'')])
            result,_,udp=self.run_sender(Path(temp)/'wheel.jsonl',clock=clock,reader=reader,udp=Udp(True))
            self.assertEqual(result['invalid'],1); self.assertEqual(result['valid'],4)
            self.assertEqual(result['send_drops'],1); self.assertEqual(result['sent'],4)
            self.assertIsNone(json.loads(udp.rows[0][0])['axis_raw'])

    def test_existing_log_is_preserved_before_any_device_or_socket_is_opened(self):
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'wheel.jsonl'; path.write_text('existing evidence\n')
            with patch.object(S,'WinMMReader',side_effect=AssertionError('device opened')):
                with self.assertRaises(FileExistsError): S.serve('unused',log_path=path,seconds=1)
            self.assertEqual(path.read_text(),'existing evidence\n')
            self.assertFalse(Path(str(path)+'.summary.json').exists())

    def test_controlled_stop_and_invalid_configuration(self):
        with tempfile.TemporaryDirectory() as temp:
            clock=Clock(); reader=Reader(clock)
            result,_,udp=self.run_sender(Path(temp)/'wheel.jsonl',clock=clock,reader=reader,
                                         stop=lambda:len(reader.calls)>=2)
            self.assertEqual(result['samples'],2); self.assertTrue(udp.closed)
            for hz in (0,float('nan'),251,True):
                with self.subTest(hz=hz),self.assertRaises(ValueError):
                    self.run_sender(Path(temp)/'invalid.jsonl',hz=hz)


if __name__=='__main__': unittest.main()
