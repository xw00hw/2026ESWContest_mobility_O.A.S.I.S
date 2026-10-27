"""Read-only camera health must never qualify an RL input or motor command."""
import io
import json
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rl_monitor import Monitor, camera_status_loop, camera_url, normalize_camera


def status(**changes):
    value = dict(schema='oasis.pose.camera.status.v1', reason='pose_valid', inference_fps=8.,
        last_body_confidence=.8, neutral_provided=True, last_body_core_detected=True,
        camera_restarts=0, valid_packets=100, preview={'image_fresh': True})
    value.update(changes)
    return value


class Once:
    stopped = False
    def is_set(self): return self.stopped
    def wait(self, _): self.stopped = True


class CameraMonitorTests(unittest.TestCase):
    def test_valid_producer_never_claims_rl_or_motor_live(self):
        monitor = Monitor('live')
        monitor.camera_status(normalize_camera(status()))
        snapshot = monitor.snapshot()
        self.assertTrue(snapshot['camera']['fresh'])
        self.assertEqual(snapshot['camera']['rl_consumption'], 'not_reported_by_control_log')
        self.assertEqual(snapshot['connection'], 'waiting')
        self.assertIsNone(snapshot['latest'])
        self.assertFalse(snapshot['control_authority'])

    def test_producer_status_expires_and_recovers_without_control_data(self):
        clock = [10.]
        monitor = Monitor(clock=lambda: clock[0])
        monitor.camera_status(normalize_camera(status()))
        clock[0] += 1.5
        self.assertIsNone(monitor.snapshot()['camera']['status'])
        monitor.camera_status(normalize_camera(status(reason='neutral_required', neutral_provided=False)))
        self.assertFalse(monitor.snapshot()['camera']['status']['neutral_provided'])
        self.assertEqual(monitor.snapshot()['connection'], 'waiting')

    def test_invalid_schema_nonfinite_or_truthy_fields_not_accepted(self):
        with self.assertRaises(ValueError): normalize_camera({'reason': 'pose_valid'})
        value = normalize_camera(status(inference_fps=float('nan'), camera_restarts=-2,
            neutral_provided='true', preview={'image_fresh': 'true'}))
        self.assertIsNone(value['inference_fps'])
        self.assertIsNone(value['camera_restarts'])
        self.assertIsNone(value['neutral_provided'])
        self.assertFalse(value['image_fresh'])

    def test_only_explicit_loopback_read_endpoint(self):
        for url in ('https://127.0.0.1/status.json', 'http://remote/status.json',
                    'http://user@localhost/status.json', 'http://localhost/motor',
                    'http://localhost/status.json?arm=1'):
            with self.assertRaises(ValueError): camera_url(url)
        self.assertEqual(camera_url('http://127.0.0.1:8081/status.json'), 'http://127.0.0.1:8081/status.json')

    def test_bounded_poll_disconnect_clears_camera_only(self):
        monitor = Monitor('live')
        monitor.camera_status(normalize_camera(status()))
        def oversized(*args, **kwargs): return io.BytesIO(b'x'*65537)
        camera_status_loop('http://localhost/status.json', monitor, Once(), opener=oversized)
        result = monitor.snapshot()
        self.assertIsNone(result['camera']['status'])
        self.assertIn('Oversized', result['camera']['error'])
        self.assertEqual(result['invalid_lines'], 0)
        def valid(*args, **kwargs): return io.BytesIO(json.dumps(status()).encode())
        camera_status_loop('http://localhost/status.json', monitor, Once(), opener=valid)
        self.assertTrue(monitor.snapshot()['camera']['fresh'])


if __name__ == '__main__': unittest.main()
