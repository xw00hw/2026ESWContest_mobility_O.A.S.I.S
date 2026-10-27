"""Monitor-only tests; no Pi, serial or actuator commands."""
import http.client
import io
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from rl_monitor import Monitor, Server, normalize, bounded_lines, ssh_command, replay_file, tail_file, MAX_LINE


def row(stamp=1., **kwargs):
    data = dict(event='g_control_tick', monotonic_s=stamp, measured_g=.2, predicted_g=.3,
                base_yaw_deg=dict(left=0,right=6),rl_delta_yaw_deg=dict(left=0,right=1),
                final_sent_yaw_deg=dict(left=0,right=7),board_command_yaw_deg=dict(left=0,right=6.9),
                pressure_counts=dict(UR=1200,UL=800,DR=1250,DL=850),guards=['none'],status='running')
    data.update(kwargs)
    return json.dumps(data)


def wait_for(fn):
    deadline = time.monotonic()+2
    while time.monotonic()<deadline:
        if fn():return
        time.sleep(.01)
    raise AssertionError('timed out')


class TestMonitor(unittest.TestCase):
    def test_missing_nonfinite_never_displayed_as_zero(self):
        data=normalize(dict(event='g_control_tick',monotonic_s=1.,measured_g=float('nan'),predicted_g=True))
        self.assertIsNone(data['measured_g'])
        self.assertIsNone(data['predicted_g'])
        self.assertIsNone(data['final_sent_yaw_deg']['left'])
        self.assertIsNone(data['pressure_counts']['UR'])
        self.assertIsNone(normalize({'event':'other'}))

    def test_simulation_source_and_policy_preserved_without_claiming_live(self):
        data=normalize(dict(event='g_control_tick',monotonic_s=1.,source='simulation',policy='g_residual_rl',
                            requested_yaw_deg=dict(left=1,right=2),filtered_yaw_deg=dict(left=.9,right=1.9)))
        self.assertEqual(data['source'],'simulation')
        self.assertEqual(data['policy_mode'],'g_residual_rl')
        self.assertEqual(data['requested_yaw_deg']['right'],2.)
        self.assertEqual(data['filtered_yaw_deg']['right'],1.9)

    def test_stale_hides_current_values_keeps_history(self):
        clock=[10.]
        monitor=Monitor('live',clock=lambda:clock[0])
        self.assertEqual(monitor.snapshot()['connection'],'waiting')
        monitor.ingest(row())
        self.assertTrue(monitor.snapshot()['fresh'])
        clock[0]+=1.
        status=monitor.snapshot()
        self.assertFalse(status['fresh'])
        self.assertIsNone(status['latest'])
        self.assertEqual(len(status['history']),1)
        self.assertFalse(status['control_authority'])

    def test_disconnect_immediately_invalidates_current(self):
        monitor=Monitor('live')
        monitor.ingest(row())
        monitor.source_state('disconnected','SSH ended')
        self.assertIsNone(monitor.snapshot()['latest'])

    def test_bounded_history_time_and_count_and_new_run(self):
        monitor=Monitor('replay',max_rows=5)
        for i in range(100):monitor.ingest(row(float(i)))
        self.assertEqual(len(monitor.snapshot()['history']),5)
        monitor.ingest(row(200.))
        self.assertEqual(len(monitor.snapshot()['history']),1)

    def test_fast_source_chart_retains_sixty_seconds_with_latest_tick(self):
        monitor=Monitor('live')
        for i in range(7001):monitor.ingest(row(i/100))
        status=monitor.snapshot()
        self.assertLessEqual(len(status['history']),601)
        self.assertGreaterEqual(status['history'][-1]['monotonic_s']-status['history'][0]['monotonic_s'],59.8)
        self.assertEqual(status['latest']['monotonic_s'],70.)
        monitor.ingest(row(0.))
        self.assertEqual(monitor.snapshot()['source_epoch'],1)
        self.assertEqual(len(monitor.snapshot()['history']),1)

    def test_guard_html_treated_as_data_and_invalid_lines_rejected(self):
        monitor=Monitor()
        monitor.ingest(row(guards=['<script>bad()</script>']))
        self.assertEqual(monitor.snapshot()['latest']['guards'],['<script>bad()</script>'])
        self.assertFalse(monitor.ingest('invalid'))
        self.assertFalse(monitor.ingest('x'*(MAX_LINE+1)))
        self.assertEqual(monitor.snapshot()['invalid_lines'],2)

    def test_oversized_lines_dropped_bounded(self):
        stream=io.BytesIO(b'x'*(MAX_LINE*3)+b'\n'+row().encode()+b'\n')
        self.assertEqual(len(list(bounded_lines(stream))),1)

    def test_replay_mode_stays_replay_and_completion_not_live(self):
        monitor=Monitor('replay')
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'sample.jsonl'
            path.write_text(row(1)+'\n'+row(1.001)+'\n',encoding='utf-8')
            replay_file(path,monitor,threading.Event(),20.)
        status=monitor.snapshot()
        self.assertEqual(status['mode'],'replay')
        self.assertEqual(status['source_state'],'replay_complete')
        self.assertIsNone(status['latest'])
        self.assertEqual(len(status['history']),1)  # Same chart bin; newest source row wins.
        self.assertEqual(status['history'][0]['monotonic_s'],1.001)

    def test_live_file_skips_old_rows_and_waits_for_complete_record(self):
        monitor,stop=Monitor('live'),threading.Event()
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'live.jsonl'
            path.write_text(row(1)+'\n',encoding='utf-8')
            thread=threading.Thread(target=tail_file,args=(path,monitor,stop),daemon=True)
            thread.start()
            time.sleep(.15)
            self.assertEqual(len(monitor.snapshot()['history']),0)
            with path.open('ab') as stream:
                stream.write(row(2).encode());stream.flush()
                time.sleep(.15)
                self.assertEqual(len(monitor.snapshot()['history']),0)
                stream.write(b'\n');stream.flush()
            wait_for(lambda:len(monitor.snapshot()['history'])==1)
            stop.set();thread.join(1.)
        self.assertEqual(monitor.snapshot()['history'][0]['monotonic_s'],2.)

    def test_ssh_only_read_tail_and_quotes_path(self):
        cmd=ssh_command('fe80::123%5','esw',"/home/esw/a' ; touch bad/g_control.jsonl")
        self.assertIn('BatchMode=yes',cmd)
        self.assertNotIn('StrictHostKeyChecking=no',cmd)
        self.assertEqual(cmd[-2],'esw@fe80::123%5')
        self.assertTrue(cmd[-1].startswith('tail -n 0 -F -- '))
        self.assertIn("'\"'\"'",cmd[-1])
        for host in ('-option','bad host'):
            with self.assertRaises(ValueError):ssh_command(host,'esw','/tmp/log')
        with self.assertRaises(ValueError):ssh_command('pi','esw','relative/path')

    def test_http_loopback_no_write_endpoint(self):
        monitor=Monitor('demo');monitor.ingest(row())
        server=Server(0,monitor)
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        try:
            self.assertEqual(server.server_address[0],'127.0.0.1')
            conn=http.client.HTTPConnection('127.0.0.1',server.server_address[1],timeout=1)
            conn.request('GET','/status.json');response=conn.getresponse()
            self.assertEqual(response.status,200)
            self.assertEqual(json.loads(response.read())['mode'],'demo')
            conn.request('POST','/motor');response=conn.getresponse()
            self.assertEqual(response.status,501);response.read()
            conn.request('GET','/',headers={'Host':'attacker.test'});response=conn.getresponse()
            self.assertEqual(response.status,403);response.read();conn.close()
        finally:
            server.shutdown();server.server_close();thread.join(1.)


if __name__=='__main__':unittest.main()
