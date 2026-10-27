"""Offline preview isolation/freshness tests. No camera, model, motor or Pi access."""
import http.client
import importlib.util
import json
from pathlib import Path
import sys
import threading
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from oasis_rt.pose_camera import CameraWorker, Frame, main
from oasis_rt.pose_preview import LatestPreview, PreviewServer, PreviewJob, render_jpeg, PAGE
from test_pose_camera import Clock, Capture, Publisher, Backend, valid_features


def wait_for(predicate, timeout=1.):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.005)
    raise AssertionError('condition timed out')


class TestLatestPreview(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()

    def frame(self, image=b'frame', age=.01):
        return Frame(image, self.clock.ns - int(age * 1e9), self.clock.ns, self.clock.ns)

    def latest(self, encoder=lambda job: job.image):
        latest = LatestPreview(encoder=encoder, clock_ns=self.clock)
        self.addCleanup(latest.close)
        return latest

    def test_latest_replacement_has_no_queue_lag(self):
        entered, release = threading.Event(), threading.Event()
        seen = []
        def encode(job):
            seen.append(job.image)
            if job.image == b'first':
                entered.set()
                release.wait(1.)
            return job.image
        latest = self.latest(encode)
        self.addCleanup(release.set)
        latest.submit(self.frame(b'first'), None, {'reason': 'pose_valid'})
        self.assertTrue(entered.wait(1.))
        for i in range(100):
            latest.submit(self.frame(str(i).encode()), None, {'reason': 'pose_valid'})
        self.assertEqual(latest.summary()['pending_frames'], 1)
        release.set()
        wait_for(lambda: latest.snapshot()[1] == b'99')
        self.assertEqual(seen, [b'first', b'99'])
        self.assertEqual(latest.summary()['encoded_frames'], 2)
        self.assertGreater(latest.summary()['replaced_pending_frames'], 0)

    def test_disconnect_cannot_resurrect_inflight_image(self):
        entered, release, finished = threading.Event(), threading.Event(), threading.Event()
        def encode(job):
            entered.set()
            release.wait(1.)
            finished.set()
            return b'old'
        latest = self.latest(encode)
        self.addCleanup(release.set)
        latest.submit(self.frame(), None, {'reason': 'pose_valid'})
        self.assertTrue(entered.wait(1.))
        latest.unavailable({'reason': 'camera_missing'})
        release.set()
        self.assertTrue(finished.wait(1.))
        self.assertIsNone(latest.snapshot()[1])
        self.assertFalse(latest.summary()['image_fresh'])

    def test_poll_gap_retains_original_timestamp_but_expiry_and_disconnect_clear(self):
        latest = self.latest()
        latest.submit(self.frame(), None, {'reason': 'pose_valid'})
        wait_for(lambda: latest.snapshot()[1] == b'frame')
        for reason in ('camera_ready', 'no_new_frame'):
            latest.unavailable({'reason': reason})
            self.assertEqual(latest.snapshot()[1], b'frame')
            self.assertEqual(latest.snapshot()[2]['reason'], reason)
        self.clock.ns += 300_000_000
        latest.unavailable({'reason': 'camera_ready'})
        self.assertIsNone(latest.snapshot()[1])
        latest.submit(self.frame(b'new'), None, {'reason': 'pose_valid'})
        wait_for(lambda: latest.snapshot()[1] == b'new')
        latest.unavailable({'reason': 'camera_read_timeout'})
        self.assertIsNone(latest.snapshot()[1])

    def test_future_and_old_capture_not_published(self):
        calls = []
        latest = self.latest(lambda job: calls.append(job.capture_ns) or b'bad')
        for age in (-.1, .3, 1.):
            latest.submit(self.frame(age=age), None, {'reason': 'pose_valid'})
            wait_for(lambda: len(calls) > 0)
            self.assertIsNone(latest.snapshot()[1])
            calls.clear()

    def test_encoder_failure_is_reported_and_recovers(self):
        def encode(job):
            if job.image == b'bad':
                raise RuntimeError('mock jpeg failure')
            return job.image
        latest = self.latest(encode)
        latest.submit(self.frame(b'bad'), None, {'reason': 'pose_valid'})
        wait_for(lambda: latest.summary()['encoder_error'] is not None)
        latest.submit(self.frame(b'good'), None, {'reason': 'pose_valid'})
        wait_for(lambda: latest.snapshot()[1] == b'good')
        self.assertIsNone(latest.summary()['encoder_error'])
        latest.close()
        self.assertIsNone(latest.snapshot()[1])

    def test_slow_encoder_not_starved_by_newer_pending_frame(self):
        first_entered, second_entered, first_release, second_release = [threading.Event() for _ in range(4)]
        def encode(job):
            if job.image == b'first':
                first_entered.set()
                first_release.wait(1.)
            else:
                second_entered.set()
                second_release.wait(1.)
            return job.image
        latest = self.latest(encode)
        self.addCleanup(first_release.set)
        self.addCleanup(second_release.set)
        latest.submit(self.frame(b'first'), None, {'reason': 'pose_valid'})
        self.assertTrue(first_entered.wait(1.))
        latest.submit(self.frame(b'next'), None, {'reason': 'pose_valid'})
        first_release.set()
        self.assertTrue(second_entered.wait(1.))
        self.assertEqual(latest.snapshot()[1], b'first')
        second_release.set()
        wait_for(lambda: latest.snapshot()[1] == b'next')


@unittest.skipUnless(importlib.util.find_spec('cv2'), 'OpenCV not installed')
class TestPreviewRendering(unittest.TestCase):
    def test_synthetic_overlay_jpeg_preserves_original_pixels(self):
        import cv2
        import numpy as np
        original = np.zeros((480, 640, 3), dtype=np.uint8)
        xy = [[200 + i * 5, 120 + i * 8] for i in range(17)]
        job = PreviewJob(1, original, 10, dict(xy=xy, scores=[.9] * 17, roi_xyxy=[0, 0, 640, 480]),
                         {'reason': 'neutral_required'})
        jpeg = render_jpeg(job, max_width=320)
        decoded = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        self.assertEqual(decoded.shape, (240, 320, 3))
        self.assertGreater(int(decoded.max()), 0)
        self.assertEqual(int(original.max()), 0)


class TestPreviewHttp(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.server = PreviewServer(0, clock_ns=self.clock, encoder=lambda job: job.image, blank=b'blank')
        self.addCleanup(self.server.close)

    def get(self, path, host=None):
        conn = http.client.HTTPConnection('127.0.0.1', self.server.port, timeout=1.)
        self.addCleanup(conn.close)
        conn.request('GET', path, headers={} if host is None else {'Host': host})
        response = conn.getresponse()
        return response, response.read()

    def test_local_status_and_html_no_recording_no_motor_authority(self):
        self.assertEqual(self.server.server.server_address[0], '127.0.0.1')
        response, body = self.get('/status.json')
        self.assertEqual(response.status, 200)
        status = json.loads(body)['preview']
        self.assertFalse(status['image_fresh'])
        self.assertFalse(status['recording'])
        self.assertFalse(status['motor_access'])
        self.assertIn('no-store', response.getheader('Cache-Control'))
        response, body = self.get('/')
        self.assertEqual(response.status, 200)
        self.assertIn(b'AbortController', body)
        self.assertIn(b'performance.now()>=validUntil', body)
        self.assertIn("'\\nFrame age: '", PAGE)
        self.assertNotIn("'\nFrame age: '", PAGE)

    def test_untrusted_host_and_unknown_endpoint_rejected(self):
        for host in ('attacker.example', '[malformed'):
            self.assertEqual(self.get('/status.json', host)[0].status, 403)
        self.assertEqual(self.get('/record')[0].status, 404)

    def test_stream_uses_same_encoded_frame_and_placeholder_on_missing(self):
        frame = Frame(b'mock-jpeg', self.clock.ns - 10_000_000, 0, 0)
        self.server.submit(frame, None, {'reason': 'neutral_required'})
        wait_for(lambda: self.server.latest.snapshot()[1] == b'mock-jpeg')
        conn = http.client.HTTPConnection('127.0.0.1', self.server.port, timeout=1.)
        self.addCleanup(conn.close)
        conn.request('GET', '/stream.mjpg')
        response = conn.getresponse()
        self.assertEqual(response.status, 200)
        expected = (b'--oasisframe\r\nContent-Type: image/jpeg\r\nContent-Length: 9\r\n'
                    b'X-Frame-Fresh: 1\r\nX-Capture-Ns: ' + str(frame.capture_ns).encode() +
                    b'\r\nX-Frame-Sequence: 1\r\n\r\nmock-jpeg\r\n')
        self.assertEqual(response.read(len(expected)), expected)
        self.server.unavailable({'reason': 'camera_missing'})
        expected = (b'--oasisframe\r\nContent-Type: image/jpeg\r\nContent-Length: 5\r\n'
                    b'X-Frame-Fresh: 0\r\nX-Capture-Ns: 0\r\nX-Frame-Sequence: 2\r\n\r\nblank\r\n')
        self.assertEqual(response.read(len(expected)), expected)


class PreviewSpy:
    def __init__(self):
        self.frames, self.invalid, self.closed = [], [], False
    def submit(self, frame, landmarks, status):
        self.frames.append((frame, landmarks, status))
    def unavailable(self, status):
        self.invalid.append(status)
    def close(self):
        self.closed = True


class TestCameraPreviewIsolation(unittest.TestCase):
    def setUp(self):
        self.clock, self.publisher, self.preview, self.backend = Clock(), Publisher(), PreviewSpy(), Backend()
    def frame(self):
        return Frame(object(), self.clock.ns - 10_000_000, 0, 0)
    def worker(self, frames, **kwargs):
        config = dict(source_id='test-camera', camera_id='camera1', neutral={'camera_id': 'camera1'},
                      clock_ns=self.clock, feature_fn=valid_features, preview=self.preview)
        config.update(kwargs)
        worker = CameraWorker(Capture(frames), self.backend, self.publisher, **config)
        self.addCleanup(worker.close)
        return worker

    def test_same_frame_single_inference_and_no_preview_authority(self):
        frame, calls = self.frame(), []
        def infer(*args):
            calls.append(args[0])
            self.backend.last_preview_landmarks = {'xy': 'current'}
            return valid_features()
        worker = self.worker([frame], feature_fn=infer)
        worker.step()
        self.assertEqual(calls, [frame])
        self.assertIs(self.preview.frames[0][0], frame)
        self.assertEqual(self.preview.frames[0][1], {'xy': 'current'})
        self.assertTrue(self.publisher.packets[-1]['vision_valid'])
        worker.step()
        self.assertEqual(self.preview.invalid[-1]['reason'], 'camera_missing')
        self.assertFalse(self.publisher.packets[-1]['vision_valid'])

    def test_uncalibrated_frame_displayed_invalid_and_old_joints_never_reused(self):
        def infer(frame, backend, *_):
            backend.last_preview_landmarks = {'xy': 'new'}
            raise ValueError('neutral_required')
        worker = self.worker([self.frame()], neutral=None, feature_fn=infer)
        worker.step()
        self.assertFalse(self.publisher.packets[-1]['vision_valid'])
        self.assertEqual(self.preview.frames[-1][1], {'xy': 'new'})
        self.clock.ns += 20_000_000
        worker.capture.frames.append(self.frame())
        def fail(*_):
            raise RuntimeError('decode failed')
        worker.feature_fn = fail
        worker.step()
        self.assertIsNone(self.preview.frames[-1][1])

    def test_preview_failure_does_not_disable_pose_or_backend(self):
        self.preview.submit = lambda *_: (_ for _ in ()).throw(RuntimeError('viewer broken'))
        worker = self.worker([self.frame()])
        worker.step()
        self.assertTrue(self.publisher.packets[-1]['vision_valid'])
        self.assertEqual(worker.errors['preview_error'], 1)

    def test_unavailable_backend_keeps_status_and_never_opens_camera(self):
        worker = self.worker([])
        worker.backend = None
        worker.step()
        self.assertEqual(worker.capture.polls, 0)
        self.assertEqual(self.preview.invalid[-1]['reason'], 'backend_unavailable')
        self.assertFalse(self.publisher.packets[-1]['vision_valid'])

    def test_preview_bind_failure_keeps_producer_running(self):
        with patch('oasis_rt.pose_camera.make_backend', return_value=Backend()), \
             patch('oasis_rt.pose_camera.ProcessCapture', return_value=Capture()), \
             patch('oasis_rt.pose_camera.LoopbackPublisher', return_value=self.publisher), \
             patch('oasis_rt.pose_preview.PreviewServer', side_effect=OSError('port in use')):
            self.assertEqual(main(['--backend', 'onnx-reference', '--source-id', 'test-camera',
                                   '--preview-port', '8080', '--duration', '.001']), 0)
        self.assertGreater(len(self.publisher.packets), 0)
        self.assertTrue(self.publisher.closed)


if __name__ == '__main__':
    unittest.main()
