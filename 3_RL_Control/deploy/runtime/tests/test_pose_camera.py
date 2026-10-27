"""Camera-worker tests: fake frames/backends only; no cameras or motor links."""
import json
from pathlib import Path
import queue
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from oasis_rt.pose_camera import (CameraWorker, Frame, ProcessCapture, _put_latest,
                                  OnnxReference, _status_file, capture_timestamp_ns, make_backend, main)
from oasis_rt.pose_input import UdpPose


class Clock:
    ns = 10_000_000_000
    def __call__(self):
        return self.ns


class Capture:
    reason = 'camera_missing'
    def __init__(self, frames=()):
        self.frames, self.closed, self.polls = list(frames), 0, 0
    def poll(self):
        self.polls += 1
        return self.frames.pop(0) if self.frames else None
    def close(self):
        self.closed += 1


class Publisher:
    def __init__(self):
        self.packets, self.closed = [], 0
    def send(self, packet):
        self.packets.append(packet)
    def close(self):
        self.closed += 1


class Backend:
    name, model_sha256 = 'onnx_cpu_reference', 'a' * 64


class Journal:
    def record(self, *args, **kwargs):
        pass


class Transport:
    def __init__(self):
        self.items = []
    def recvfrom(self, _size):
        if not self.items:
            raise BlockingIOError
        return self.items.pop(0), ('127.0.0.1', 12345)
    def close(self):
        pass


class FakeProcess:
    def __init__(self):
        self.alive, self.terminated, self.closed = True, False, False
        self.waits = []
    def is_alive(self):
        return self.alive
    def join(self, timeout):
        self.waits.append(timeout)
    def terminate(self):
        self.terminated, self.alive = True, False
    def close(self):
        self.closed = True


class FakeQueue(queue.Queue):
    def close(self):
        pass
    def cancel_join_thread(self):
        pass


class FakeEvent:
    def set(self):
        pass


def valid_features(*_args):
    return dict(torso_lean_deg=5., lean_score=.25, pose_confidence=.9,
                vision_valid=True, neutral_valid=True)


class TestCameraWorker(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.publisher = Publisher()

    def frame(self, ago=.01, generation=1):
        return Frame(object(), self.clock.ns - int(ago * 1e9),
                     self.clock.ns - 2_000_000, self.clock.ns - 1_000_000, generation)

    def worker(self, frames=(), **kwargs):
        capture = kwargs.pop('capture', Capture(frames))
        defaults = dict(source_id='test-camera', camera_id='usb:UC762',
                        neutral={'camera_id': 'usb:UC762'}, clock_ns=self.clock,
                        feature_fn=valid_features)
        defaults.update(kwargs)
        return CameraWorker(capture, Backend(), self.publisher, **defaults)

    def test_kernel_timestamp_required_and_exact_clock_age(self):
        self.assertEqual(capture_timestamp_ns(9990., self.clock.ns, .3), 9_990_000_000)
        for value in (0, -1, float('nan'), float('inf'), True, '9990', 10_001, 9700):
            with self.subTest(value=value), self.assertRaises(ValueError):
                capture_timestamp_ns(value, self.clock.ns, .3)

    def test_only_one_latest_queued_frame(self):
        q = queue.Queue(1)
        for value in range(1000):
            _put_latest(q, value)
        self.assertEqual(q.qsize(), 1)
        self.assertEqual(q.get_nowait(), 999)

    def test_valid_wire_contract_then_camera_missing_clears_receiver(self):
        worker = self.worker([self.frame()])
        worker.step()
        packet = self.publisher.packets[-1]
        self.assertTrue(packet['vision_valid'])
        self.assertEqual(packet['capture_ns'], 9_990_000_000)
        transport = Transport()
        receiver = UdpPose(dict(source_id='test-camera', udp_port=0, stale_s=.3, min_confidence=.5),
                           self.clock, Journal(), transport=transport)
        transport.items.append(json.dumps(packet).encode())
        self.assertIsNotNone(receiver.latest()[0])
        worker.step()  # No new frame: never re-publish the previous image/features.
        invalid = self.publisher.packets[-1]
        self.assertFalse(invalid['vision_valid'])
        self.assertEqual(invalid['capture_ns'], 0)
        transport.items.append(json.dumps(invalid).encode())
        self.assertIsNone(receiver.latest()[0])
        self.assertEqual(worker.valid_packets, 1)
        self.assertEqual(len(self.publisher.packets), 2)

    def test_old_future_and_duplicate_frames_never_infer(self):
        for ago in (.3, 1, -.01):
            worker = self.worker([self.frame(ago)])
            worker.step()
            self.assertEqual(worker.inferences, 0)
            self.assertFalse(self.publisher.packets[-1]['vision_valid'])
        fresh = self.frame()
        worker = self.worker([fresh, fresh])
        worker.step()
        worker.step()
        self.assertEqual(worker.inferences, 1)
        self.assertFalse(self.publisher.packets[-1]['vision_valid'])

    def test_inference_result_aged_out_not_restamped(self):
        def slow(*args):
            self.clock.ns += 300_000_000
            return valid_features()
        worker = self.worker([self.frame()], feature_fn=slow)
        worker.step()
        self.assertFalse(self.publisher.packets[-1]['vision_valid'])
        self.assertIn('inference_result_stale', worker.reason)
        self.assertEqual(worker.inference_ms, 300)

    def test_neutral_required_and_camera_match(self):
        for neutral in (None, {'camera_id': 'another-camera'}):
            worker = self.worker([self.frame()], neutral=neutral)
            worker.step()
            self.assertFalse(self.publisher.packets[-1]['neutral_valid'])
            self.assertEqual(worker.inferences, 1)  # CPU reference still exercises the image/model path.

    def test_reconnect_invalidates_previous_neutral(self):
        first = self.frame()
        second = Frame(object(), first.capture_ns + 1, first.read_started_ns, first.read_finished_ns, 2)
        worker = self.worker([first, second])
        worker.step()
        self.assertTrue(self.publisher.packets[-1]['neutral_valid'])
        worker.step()
        self.assertFalse(self.publisher.packets[-1]['neutral_valid'])
        self.assertEqual(worker.reason, 'camera_reconnected_requires_neutral')
        # Even the first received frame cannot reuse calibration after a failed initial open.
        worker = self.worker([self.frame(generation=2)])
        worker.step()
        self.assertFalse(self.publisher.packets[-1]['neutral_valid'])

    def test_inference_error_or_nonfinite_features_unavailable(self):
        def broken(*args):
            raise RuntimeError('backend disconnected')
        for fn in (broken, lambda *a: dict(valid_features(), torso_lean_deg=float('nan')),
                   lambda *a: dict(valid_features(), lean_score=5),
                   lambda *a: dict(valid_features(), pose_confidence=.1),
                   lambda *a: dict(valid_features(), vision_valid=False)):
            worker = self.worker([self.frame()], feature_fn=fn)
            worker.step()
            self.assertFalse(self.publisher.packets[-1]['vision_valid'])

    def test_hailo_never_constructs_onnx_or_acquires_camera(self):
        with patch('oasis_rt.pose_camera.OnnxReference', side_effect=AssertionError('CPU fallback')):
            with self.assertRaisesRegex(RuntimeError, 'no CPU fallback'):
                make_backend('hailo', 'dummy.hef')
        capture = Capture()
        worker = CameraWorker(capture, None, self.publisher, source_id='test-camera', camera_id='usb',
                              clock_ns=self.clock, backend_error='hailo_unavailable', requested_backend='hailo')
        worker.step()
        self.assertEqual(capture.polls, 0)
        self.assertFalse(self.publisher.packets[-1]['vision_valid'])
        self.assertIsNone(worker.summary()['active_backend'])

    def test_hybrid_factory_requires_explicit_manifest_and_no_full_model_fallback(self):
        sentinel = object()
        with patch('oasis_rt.rtmpose_hailo.HailoHybrid', return_value=sentinel) as factory, \
                patch('oasis_rt.pose_camera.OnnxReference', side_effect=AssertionError('full CPU fallback')):
            self.assertIs(make_backend('hailo-hybrid', hybrid_manifest='artifacts.json'), sentinel)
            factory.assert_called_once_with('artifacts.json')
            for model, manifest in ((None, None), ('full.onnx', 'artifacts.json')):
                with self.assertRaises(ValueError):
                    make_backend('hailo-hybrid', model, manifest)
            with self.assertRaises(ValueError):
                make_backend('onnx-reference', 'full.onnx', 'artifacts.json')

    def test_missing_hybrid_manifest_keeps_camera_optional(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(FileNotFoundError):
                make_backend('hailo-hybrid', hybrid_manifest=Path(directory) / 'missing.json')
        capture = Capture()
        worker = CameraWorker(capture, None, self.publisher, source_id='test-camera', camera_id='usb',
                              clock_ns=self.clock, backend_error='missing_manifest', requested_backend='hailo-hybrid')
        worker.step()
        self.assertEqual(capture.polls, 0)
        self.assertFalse(self.publisher.packets[-1]['vision_valid'])
        self.assertIsNone(worker.summary()['active_backend'])

    def test_hybrid_runtime_failure_clears_pose_and_stops_polling(self):
        class Hybrid:
            name, closed, failed, closed_count = 'hailo10h_conv205_onnx_head', False, None, 0
            artifact_identity = {'partial_acceleration': True, 'hef_sha256': 'b' * 64}
            def close(self):
                self.closed, self.closed_count = True, self.closed_count + 1
        worker = self.worker([self.frame()])
        worker.backend = Hybrid()
        def fail(*_args):
            worker.backend.failed = 'hybrid_runtime_failed:TimeoutError'
            worker.backend.closed = True
            raise TimeoutError('hardware did not finish')
        worker.feature_fn = fail
        worker.step()
        worker.step()
        self.assertFalse(any(p['vision_valid'] for p in self.publisher.packets))
        self.assertEqual(worker.capture.polls, 1)
        self.assertEqual(worker.reason, 'hybrid_runtime_failed:TimeoutError')
        self.assertTrue(worker.summary()['backend_artifacts']['partial_acceleration'])
        worker.close()
        self.assertEqual(worker.backend.closed_count, 1)

    def test_backend_close_attempted_even_after_capture_failure(self):
        backend = SimpleNamespace(close=lambda: (_ for _ in ()).throw(RuntimeError('backend close failure')))
        worker = self.worker()
        worker.backend = backend
        worker.capture.close = lambda: (_ for _ in ()).throw(RuntimeError('capture close failure'))
        with self.assertRaisesRegex(RuntimeError, 'backend close failure'):
            worker.close()
        self.assertEqual(self.publisher.closed, 1)

    def test_publisher_initialization_failure_releases_hybrid_without_camera_start(self):
        capture = Capture()
        backend = SimpleNamespace(close=unittest.mock.Mock())
        with patch('oasis_rt.pose_camera.make_backend', return_value=backend), \
                patch('oasis_rt.pose_camera.ProcessCapture', return_value=capture), \
                patch('oasis_rt.pose_camera.LoopbackPublisher', side_effect=OSError('socket unavailable')):
            with self.assertRaisesRegex(OSError, 'socket unavailable'):
                main(['--backend', 'hailo-hybrid', '--hybrid-manifest', 'real-manifest.json',
                      '--source-id', 'test-camera'])
        backend.close.assert_called_once_with()
        self.assertEqual(capture.closed, 1)
        self.assertEqual(capture.polls, 0)

    def test_original_symbolic_metadata_requires_concrete_coco_outputs(self):
        class Session:
            outputs = [SimpleNamespace(shape=(1, 17, 384)), SimpleNamespace(shape=(1, 17, 512))]
            def __init__(self, *args, **kwargs):
                self.providers = kwargs['providers']
            def get_inputs(self):
                return [SimpleNamespace(name='input', shape=['batch', 3, 256, 192])]
            def get_outputs(self):
                return [SimpleNamespace(name='simcc_x', shape=['batch', 'joints', 384]),
                        SimpleNamespace(name='simcc_y', shape=['batch', 'joints', 512])]
            def run(self, names, tensors):
                return self.outputs
        ort = SimpleNamespace(SessionOptions=SimpleNamespace, InferenceSession=Session)
        with patch.dict(sys.modules, onnxruntime=ort), patch('oasis_rt.pose_camera.file_sha256', return_value='a'*64):
            backend = OnnxReference('model.onnx')
        self.assertEqual(backend.session.providers, ['CPUExecutionProvider'])
        self.assertEqual(len(backend.infer(SimpleNamespace(shape=(1, 3, 256, 192)))), 2)
        backend.session.outputs = [SimpleNamespace(shape=(1, 18, 384)), SimpleNamespace(shape=(1, 18, 512))]
        with self.assertRaisesRegex(ValueError, 'concrete output shapes'):
            backend.infer(SimpleNamespace(shape=(1, 3, 256, 192)))

    def test_cleanup_idempotent_and_publisher_closes_on_capture_close_error(self):
        worker = self.worker([self.frame()])
        worker.close()
        worker.close()
        self.assertEqual(worker.capture.closed, 1)
        self.assertEqual(self.publisher.closed, 1)
        self.assertFalse(self.publisher.packets[-1]['vision_valid'])
        worker = self.worker()
        worker.capture.close = lambda: (_ for _ in ()).throw(RuntimeError('close failure'))
        with self.assertRaisesRegex(RuntimeError, 'close failure'):
            worker.close()
        self.assertEqual(self.publisher.closed, 2)

    def test_status_contains_backend_provenance_and_no_pixels(self):
        worker = self.worker([self.frame()])
        worker.step()
        self.clock.ns += 1_000_000_000
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'status.json'
            _status_file(path, worker)
            summary = json.loads(path.read_text())
        self.assertEqual(summary['active_backend'], 'onnx_cpu_reference')
        self.assertEqual(summary['model_sha256'], 'a' * 64)
        self.assertEqual(summary['valid_pose_fps'], 1.)
        self.assertFalse(summary['metric_body_enabled'])
        self.assertNotIn('image', summary)

    def test_capture_retry_wait_is_bounded_without_opening_device(self):
        capture = ProcessCapture(dict(retry_s=2), clock_ns=self.clock)
        capture.next_attempt_ns = self.clock.ns + 2_000_000_000
        with patch.object(capture.context, 'Process', side_effect=AssertionError('must wait')):
            self.assertIsNone(capture.poll())
        capture.close()
        self.assertIsNone(capture.poll())

    def test_hung_capture_reader_is_terminated_and_retry_delayed(self):
        capture = ProcessCapture(dict(retry_s=2), clock_ns=self.clock)
        process = capture.process = FakeProcess()
        capture.frames, capture.status, capture.stop_event = FakeQueue(1), FakeQueue(1), FakeEvent()
        capture.last_frame_ns = self.clock.ns - 2_000_000_000
        capture.reason = 'camera_ready'
        self.assertIsNone(capture.poll())
        self.assertTrue(process.terminated)
        self.assertTrue(process.closed)
        self.assertLessEqual(sum(process.waits), 1.15)
        self.assertIsNone(capture.process)
        self.assertEqual(capture.next_attempt_ns, self.clock.ns + 2_000_000_000)
        self.assertEqual(capture.reason, 'camera_read_timeout')

    def test_disconnected_reader_discards_final_buffered_frame(self):
        capture = ProcessCapture(dict(retry_s=2), clock_ns=self.clock)
        process = capture.process = FakeProcess()
        process.alive = False
        capture.frames, capture.status, capture.stop_event = FakeQueue(1), FakeQueue(1), FakeEvent()
        capture.frames.put_nowait(self.frame())
        self.assertIsNone(capture.poll())
        self.assertEqual(capture.reason, 'camera_reader_exited')


if __name__ == '__main__':
    unittest.main()
