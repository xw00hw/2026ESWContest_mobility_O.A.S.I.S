"""Mixed valid journals and final manifests must not appear as control errors."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rl_monitor import Monitor, SESSION_READER, session_ssh_command


class LifecycleTests(unittest.TestCase):
    def test_valid_non_control_rows_do_not_count_as_invalid_or_control_fresh(self):
        monitor=Monitor('live')
        for name in ('session_created','wheel_sample','pose_sample','pose_rejected','neutral_settle_started'):
            self.assertFalse(monitor.ingest(json.dumps(dict(event=name,host_monotonic_ns=123))))
        data=monitor.snapshot()
        self.assertEqual(data['invalid_lines'],0)
        self.assertEqual(data['ignored_events'],5)
        self.assertEqual(data['session']['journal_phase'],'NEUTRAL_SETTLE')
        self.assertEqual(data['connection'],'waiting')
        self.assertIsNone(data['latest'])

    def test_malformed_json_and_bad_control_rows_still_invalid(self):
        monitor=Monitor()
        for line in ('not json','{}','[]','{"event":"g_control_tick"}'):
            self.assertFalse(monitor.ingest(line))
        self.assertEqual(monitor.snapshot()['invalid_lines'],4)

    def test_baseline_final_state_and_freshness_remain_separate_from_control(self):
        clock=[5.]
        monitor=Monitor(clock=lambda:clock[0])
        monitor.ingest(json.dumps(dict(event='neutral_settle_completed')))
        monitor.session_status(dict(session_id='trial',state='ended',phase='BASELINE',
            result='pressure baseline rejected',off_verified=True))
        data=monitor.snapshot()
        self.assertEqual(data['session']['journal_phase'],'BASELINE')
        self.assertTrue(data['session']['status']['off_verified'])
        self.assertFalse(data['fresh'])
        clock[0]+=1.5
        self.assertIsNone(monitor.snapshot()['session']['status'])

    def test_new_session_does_not_carry_old_control_values(self):
        monitor=Monitor()
        monitor.ingest(json.dumps(dict(event='g_control_tick',monotonic_s=3.)))
        self.assertTrue(monitor.snapshot()['fresh'])
        monitor.ingest(json.dumps(dict(event='session_created')))
        data=monitor.snapshot()
        self.assertIsNone(data['latest'])
        self.assertEqual(data['history'],[])
        self.assertEqual(data['session']['journal_phase'],'INIT')

    def test_manifest_reader_is_bounded_readonly_and_reports_actual_failure(self):
        once=SESSION_READER.replace('while True:', 'for _ in range(1):').replace('time.sleep(.5)','pass')
        with tempfile.TemporaryDirectory() as folder:
            log=Path(folder)/'control.jsonl';log.write_text('')
            manifest=Path(folder)/'manifest.json'
            content=json.dumps(dict(result=dict(phase='BASELINE',result='std exceeds limit',off_verified=True)))
            manifest.write_text(content)
            run=subprocess.run([sys.executable,'-c',once,str(log)],capture_output=True,text=True,timeout=3,check=True)
            result=json.loads(run.stdout)
            self.assertEqual(result['state'],'ended')
            self.assertEqual(result['phase'],'BASELINE')
            self.assertTrue(result['off_verified'])
            self.assertEqual(manifest.read_text(),content)
            self.assertEqual(log.read_text(),'')
            manifest.write_text('{partial')
            run=subprocess.run([sys.executable,'-c',once,str(log)],capture_output=True,text=True,timeout=3,check=True)
            self.assertEqual(json.loads(run.stdout)['state'],'unavailable')

    def test_remote_command_keeps_path_quoted(self):
        command=session_ssh_command(['ssh','esw@pi'],"/home/esw/a' ; extra/control.jsonl")
        self.assertEqual(command[:2],['ssh','esw@pi'])
        self.assertIn("'\"'\"'",command[-1])
        self.assertTrue(command[-1].startswith('python3 -u -c '))
        with self.assertRaises(ValueError):session_ssh_command(['ssh','esw@pi'],'relative')


if __name__=='__main__':unittest.main()
