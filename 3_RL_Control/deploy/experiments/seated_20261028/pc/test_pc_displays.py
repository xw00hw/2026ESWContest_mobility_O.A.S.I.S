import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from start_pc_displays import camera_command, monitor_command, validate, main, run, run_camera_tunnel


class TestPcDisplays(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        root = Path(self.directory.name)
        paths = []
        for name in ('identity', 'known_hosts', 'monitor.py'):
            path = root / name
            path.touch()
            paths.append(str(path))
        self.config = dict(pi_host='fe80::123%5',pi_user='esw',ipv6=True,
            host_key_alias='verified-original-alias',identity_file=paths[0],
            known_hosts_file=paths[1],monitor_source=paths[2],remote_control_log='',
            camera_remote_port=8081,camera_local_port=8081,monitor_local_port=8766)

    def test_default_preflight_never_starts_network(self):
        path = Path(self.directory.name)/'config.json'
        path.write_text(json.dumps(self.config),encoding='utf-8')
        with patch('start_pc_displays.subprocess.Popen') as process:
            self.assertEqual(main(['--config',str(path)]),0)
            process.assert_not_called()

    def test_start_requires_actual_log_path(self):
        validate(self.config,require_log=False)
        with self.assertRaisesRegex(ValueError,'remote_control_log'):
            validate(self.config)

    def test_camera_only_local_forward_and_verified_key(self):
        command = camera_command(self.config)
        self.assertNotIn('-6',command)  # IPv4 loopback listener with literal IPv6 destination.
        self.assertIn('-N',command)
        self.assertIn('BatchMode=yes',command)
        self.assertIn('StrictHostKeyChecking=yes',command)
        self.assertIn('HostKeyAlias=verified-original-alias',command)
        self.assertIn('127.0.0.1:8081:127.0.0.1:8081',command)
        self.assertEqual(command[-1],'esw@fe80::123%5')

    def test_log_tail_quotes_remote_shell_characters(self):
        self.config['remote_control_log']="/home/esw/test '; touch BAD/control.jsonl"
        validate(self.config)
        command = monitor_command(self.config)
        self.assertIn('-6',command)  # Tail has no IPv4 local forward to bind.
        self.assertTrue(command[-1].startswith('tail -n 0 -F -- '))
        self.assertIn("'\"'\"'",command[-1])
        self.assertNotIn('StrictHostKeyChecking=no',command)

    def test_existing_server_not_stopped_and_ssh_not_started_on_bind_error(self):
        self.config['remote_control_log']='/home/esw/run/control.jsonl'
        with patch('start_pc_displays.load_monitor') as module, patch('start_pc_displays.subprocess.Popen') as process:
            module.return_value.Server.side_effect=OSError('port occupied')
            with self.assertRaisesRegex(OSError,'port occupied'):
                run(self.config)
            process.assert_not_called()

    def test_tunnel_retries_exited_child_and_cleans_only_own_live_child(self):
        class Stop:
            stopped=False
            def is_set(self):return self.stopped
            def wait(self,seconds):
                if seconds==.5:self.stopped=True
                return self.stopped
        class Process:
            def __init__(self,pid,code):self.pid,self.returncode,self.terminated=pid,code,False
            def poll(self):return self.returncode
            def terminate(self):self.terminated=True;self.returncode=0
            def wait(self,_timeout):return self.returncode
        first,second=Process(1,255),Process(2,None)
        with patch('start_pc_displays.__file__',str(Path(self.directory.name)/'launcher.py')), \
             patch('start_pc_displays.socket.socket'),patch('start_pc_displays.socket.create_connection'), \
             patch('start_pc_displays.camera_status') as status, \
             patch('start_pc_displays.subprocess.Popen',side_effect=[first,second]) as launch:
            run_camera_tunnel(self.config,Stop())
        self.assertEqual(launch.call_count,2)
        self.assertFalse(first.terminated)
        self.assertTrue(second.terminated)
        self.assertEqual([c.args[1] for c in status.call_args_list],
                         ['connecting','retrying','connecting','forward_listening','stopped'])

    def test_tunnel_never_adopts_or_stops_an_existing_listener(self):
        with patch('start_pc_displays.socket.socket') as sock, \
             patch('start_pc_displays.camera_status') as status, \
             patch('start_pc_displays.subprocess.Popen') as launch:
            sock.return_value.__enter__.return_value.bind.side_effect=OSError('occupied')
            run_camera_tunnel(self.config,None)
            launch.assert_not_called()
            self.assertEqual(status.call_args.args[1],'port_already_in_use')


if __name__=='__main__':
    unittest.main()
