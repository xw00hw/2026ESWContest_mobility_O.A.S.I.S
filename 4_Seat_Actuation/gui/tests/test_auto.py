import csv
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from auto_scan import AutoScan,grid
from auto_summary import summarize
from kinematics import pair_pulses,POSE_FIELDS
from session import Session


class Rig:
    """Synthetic UART/clock only. Never opens a physical device."""
    def __init__(self,root,pressure=False,side='both',initial_pose=(0,0,0,0),mode='grid'):
        self.session = Session(root,dict(synthetic_test=True),dict(pressure=pressure,video=False,
                                                                  source='synthetic UART and clock'))
        self.now = self.session.origin_ns
        self.pending,self.id = [],0
        self.yaw,self.pitch,self.right_yaw,self.right_pitch = (round(x*1000) for x in initial_pose)
        self.scan = AutoScan(self.session,self.pending.append,self.pose,clock=lambda:self.now,
                             side=side,initial_pose=initial_pose,mode=mode)

    def pose(self,*pose):
        self.id += 1
        self.pending.append(f'SET,{self.id},'+','.join(str(round(x*1000)) for x in pose))
        return self.id

    def event(self,kind,**data):
        self.scan.event(dict(type=kind,timing=self.scan.timing(),**data))

    def state(self,**changes):
        pose = (self.yaw,self.pitch,self.right_yaw,self.right_pitch)
        state = dict(mcu_ms=(self.now-self.session.origin_ns)//1000000,armed=1,moving=0,
            command_id=self.id,fault='NONE',**dict(zip(POSE_FIELDS,pose)),
            **dict(zip(('target_'+name for name in POSE_FIELDS),pose)),
            **dict(zip([f'D{i}_us' for i in (3,4,5,6)],pair_pulses(*pose))))
        state.update(changes)
        self.event('state',state=state)

    def drain(self):
        while self.pending and self.scan.active:
            command = self.pending.pop(0)
            self.event('tx',text=command)
            self.now += 1000000
            if command=='HOLD':
                self.event('reply',text='#ACK,HOLD')
            elif command=='ARM':
                self.event('reply',text='#ACK,ARM')
            elif command=='STATE':
                self.state()
            else:
                _,seq,*pose = command.split(',')
                self.yaw,self.pitch,self.right_yaw,self.right_pitch = map(int,pose)
                self.event('reply',text=f'#ACK,SET,{seq},'+','.join(pose)+',200')
                self.now += 200000000
                self.state()

    def pressure(self):
        c = self.scan.current
        for time_ns,raw in [(c['dwell_start_ns']-1,4095),(c['dwell_start_ns'],1000),
                            (c['dwell_end_ns']-1,3000),(c['dwell_end_ns'],4095)]:
            for bank in range(3):
                for channel in range(4):
                    self.session.row('pressure',dict(seq=channel,mcu_ms=0,mcu_extended_ms=0,
                        channel=channel,bank=bank,sensor_id=f'A{bank}_C{channel}',raw=raw,
                        status=0,voltage_v=raw*3.3/4095),self.scan.timing(time_ns))

    def finish_point(self,pressure=False):
        if pressure:
            self.pressure()
        end = self.scan.current['dwell_end_ns']
        while self.scan.active and self.now<end:
            self.now = min(self.now+200000000,end)
            self.state()
            self.scan.tick()

    def close(self):
        self.session.close()


class AutoTests(unittest.TestCase):
    def test_yaw_cartesian_784_logs_both_pitch_zero_and_returns_both_to_zero(self):
        targets=grid('yaw')
        self.assertEqual(len(targets),784)
        self.assertEqual(set(targets),{(left,0,right,0) for left in range(28) for right in range(28)})
        self.assertTrue(all(sum(abs(x-y) for x,y in zip(a,b))==1 for a,b in zip(targets,targets[1:])))
        with tempfile.TemporaryDirectory() as tmp:
            rig=Rig(tmp,mode='yaw',side='left')
            self.assertEqual(rig.scan.side,'both')
            self.assertEqual(len(rig.scan.plan),784)
            while rig.scan.active:
                rig.drain()
                self.assertEqual((rig.pitch,rig.right_pitch),(0,0))
                if rig.scan.stage in ('settle','dwell'):
                    rig.finish_point()
            self.assertEqual(rig.scan.info['completed'],784)
            self.assertEqual((rig.yaw,rig.right_yaw),(0,0))
            rig.close()
            result=summarize(rig.session.directory)
            self.assertEqual((result['mode'],result['completed'],result['planned']),('yaw',784,784))
            with (rig.session.directory/'auto_results.csv').open() as file:
                rows=list(csv.DictReader(file))
            self.assertEqual(len(rows),784)
            self.assertEqual({(r['left_yaw_deg'],r['right_yaw_deg']) for r in rows},
                              {(str(a),str(b)) for a in range(28) for b in range(28)})

    def test_full_single_side_scans_hold_other_side_and_return_only_selected_side(self):
        initial=(5,2,7,-3)
        for side in ('left','right'):
            with self.subTest(side=side),tempfile.TemporaryDirectory() as tmp:
                rig=Rig(tmp,side=side,initial_pose=initial)
                fixed=(2,3) if side=='left' else (0,1)
                while rig.scan.active:
                    rig.drain()
                    pose=(rig.yaw,rig.pitch,rig.right_yaw,rig.right_pitch)
                    self.assertEqual(tuple(pose[i] for i in fixed),tuple(round(initial[i]*1000) for i in fixed))
                    if rig.scan.stage in ('settle','dwell'):
                        rig.finish_point()
                self.assertEqual(rig.scan.info['completed'],357)
                self.assertEqual((rig.yaw,rig.pitch,rig.right_yaw,rig.right_pitch),
                    (0,0,7000,-3000) if side=='left' else (5000,2000,0,0))
                rig.close()
                summary=summarize(rig.session.directory)
                self.assertEqual((summary['side'],summary['completed']),(side,357))

    def test_grid_covers_all_357_once_with_small_steps(self):
        points = grid()
        self.assertEqual(len(points),357)
        self.assertEqual(set(points),{(y,p) for y in range(21) for p in range(-8,9)})
        self.assertTrue(all(abs(y-y0)+abs(p-p0)==1 for (y,p),(y0,p0) in zip(points,points[1:])))

    def test_full_357_replay_and_exact_pressure_windows(self):
        with tempfile.TemporaryDirectory() as tmp:
            rig = Rig(tmp,pressure=True)
            while rig.scan.active:
                rig.drain()
                if rig.scan.stage in ('settle','dwell'):
                    rig.finish_point(pressure=True)
            self.assertEqual(rig.scan.info['status'],'complete')
            self.assertEqual(rig.scan.info['completed'],357)
            self.assertEqual((rig.yaw,rig.pitch),(0,0))
            rig.close()
            result = summarize(rig.session.directory)
            self.assertEqual((result['completed'],result['not_attempted']),(357,0))
            root = rig.session.directory
            with (root/'auto_results.csv').open() as f:
                rows = list(csv.DictReader(f))
            self.assertEqual(len(rows),357)
            self.assertTrue(all(r['status']=='complete' and r['measurement_s']=='2.0' for r in rows))
            with (root/'auto_pressure_summary.csv').open() as f:
                pressure = list(csv.DictReader(f))
            self.assertEqual(len(pressure),357*12)
            self.assertTrue(all(r['samples']=='2' and r['mean_raw']=='2000.0' for r in pressure))
            self.assertTrue(all(abs(float(r['mean_v'])-2000*3.3/4095)<1e-12 for r in pressure))
            self.assertFalse((root/'INCOMPLETE').exists())
            self.assertFalse((root/'AUTO_SUMMARY_INCOMPLETE').exists())

    def test_no_advance_without_set_ack(self):
        with tempfile.TemporaryDirectory() as tmp:
            rig = Rig(tmp)
            # Prepare the first target, but withhold its ACK.
            rig.event('tx',text='HOLD'); rig.event('reply',text='#ACK,HOLD')
            rig.event('tx',text='ARM'); rig.event('reply',text='#ACK,ARM')
            rig.state()
            rig.yaw,rig.pitch = 0,-8000
            rig.event('tx',text='SET,1,0,-8000,0,-8000')
            rig.state()
            self.assertEqual(rig.scan.stage,'ack')
            self.assertNotIn('arrived_ns',rig.scan.current)
            rig.scan.abort('test_stop','cancelled'); rig.close()

    def test_ack_mismatch_aborts_and_preserves_incomplete(self):
        with tempfile.TemporaryDirectory() as tmp:
            rig = Rig(tmp); rig.drain()
            rig.scan.stage = 'ack'
            rig.event('reply',text='#ACK,SET,999,0,-8000,0,-8000,200')
            self.assertFalse(rig.scan.active)
            self.assertEqual(rig.scan.info['reason'],'SET_ACK_MISMATCH')
            rig.close()
            self.assertTrue((rig.session.directory/'INCOMPLETE').exists())

    def test_wrong_command_or_changed_pose_cannot_pass(self):
        for change in [dict(command_id=999),dict(moving=1),dict(armed=0)]:
            with self.subTest(change=change),tempfile.TemporaryDirectory() as tmp:
                rig = Rig(tmp); rig.drain()
                rig.state(**change)
                self.assertFalse(rig.scan.active)
                self.assertEqual(rig.scan.info['completed'],0)
                rig.close()

    def test_stale_state_aborts(self):
        with tempfile.TemporaryDirectory() as tmp:
            rig = Rig(tmp); rig.drain()
            rig.now += 1000000001
            rig.scan.tick()
            self.assertEqual(rig.scan.info['reason'],'STATE_STALE')
            rig.close()

    def test_cancel_lists_unvisited_combinations(self):
        with tempfile.TemporaryDirectory() as tmp:
            rig = Rig(tmp); rig.drain(); rig.finish_point(); rig.drain()
            rig.scan.abort('user_stop','cancelled'); rig.close()
            summary = summarize(rig.session.directory)
            self.assertEqual(summary['completed'],1)
            self.assertEqual(summary['not_attempted'],355)
            self.assertEqual(summary['scan_status'],'cancelled')
            self.assertTrue((rig.session.directory/'INCOMPLETE').exists())

    def test_summary_keeps_adc_errors_and_aligns_video_display_time(self):
        with tempfile.TemporaryDirectory() as tmp:
            rig = Rig(tmp,pressure=True); rig.drain()
            rig.session.metadata['context']['video'] = True
            c = rig.scan.current
            begin,end = c['dwell_start_ns'],c['dwell_end_ns']
            rig.pressure()
            rig.session.row('pressure',dict(seq=999,mcu_ms=0,mcu_extended_ms=0,channel=0,
                bank=0,sensor_id='A0_C0',raw=None,status=1,voltage_v=None),rig.scan.timing(begin+1))
            for index,display_ns,source_ns,repeated in [(1,begin+100,begin-100,1),
                                                       (2,begin+200,begin+200,0),(3,end+100,end+100,0)]:
                rig.session.row('frame_timestamps',dict(video_frame_index=index,
                    video_time_s=(display_ns-rig.session.origin_ns)/1e9,capture_index=index,
                    repeated=repeated),rig.scan.timing(source_ns))
            rig.finish_point()
            rig.scan.abort('test_done','cancelled'); rig.close()
            summarize(rig.session.directory)
            with (rig.session.directory/'auto_results.csv').open() as f:
                result = next(csv.DictReader(f))
            self.assertEqual((result['video_frames'],result['video_repeated_frames'],
                              result['video_unique_source_frames']),('2','1','1'))
            self.assertEqual(result['pressure_adc_errors'],'1')
            for pin in (3,4,5,6):
                self.assertEqual(result[f'expected_D{pin}_us'],result[f'reported_D{pin}_us'])
            with (rig.session.directory/'auto_pressure_summary.csv').open() as f:
                sensor = next(csv.DictReader(f))
            self.assertEqual((sensor['samples'],sensor['valid_samples'],sensor['adc_errors']),('3','2','1'))

    def test_invalid_measurement_duration_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            session = Session(tmp,{})
            with self.assertRaises(ValueError):
                AutoScan(session,lambda c:None,lambda y,p:1,dwell_s=0)
            session.close()


if __name__=='__main__':
    unittest.main()
