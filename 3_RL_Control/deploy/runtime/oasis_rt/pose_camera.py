"""Optional, standalone RTMPose producer. Never imports a motor/serial controller.

Run separately from oasis_rt.app; absence or failure only makes pose unavailable.
ONNX is an explicitly named CPU reference backend. The optional hailo-hybrid
backend runs only Conv_205's prefix on Hailo10H and the original head on CPU;
it requires real compiled artifacts and an explicit identity manifest.
No image/video is saved, and no neutral pose is learned automatically.

V4L2 CAP_PROP_POS_MSEC is the dequeued kernel buffer timestamp (OpenCV
cap_v4l.cpp), not read-completion time. We require it to be positive, increasing,
and recent in CLOCK_MONOTONIC. Unknown/old timestamps are rejected. This is a
kernel capture timestamp, not a claim about calibrated optical exposure time.
"""
from collections import Counter
from dataclasses import dataclass
import argparse
import hashlib
import json
import math
import multiprocessing as mp
from pathlib import Path
import queue
import signal
import socket
import time
import uuid


SCHEMA = 'oasis.pose.v1'
TIMESTAMP_SOURCE = 'v4l2_buffer_timestamp_via_opencv_pos_msec'


@dataclass(frozen=True)
class Frame:
    image: object
    capture_ns: int
    read_started_ns: int
    read_finished_ns: int
    generation: int = 1


def capture_timestamp_ns(timestamp_ms, now_ns, max_age_s):
    """Never substitute receipt time for a missing or stale kernel timestamp."""
    if isinstance(timestamp_ms, bool) or not isinstance(timestamp_ms, (float, int)):
        raise ValueError('capture_timestamp_unavailable')
    if not math.isfinite(timestamp_ms) or timestamp_ms <= 0:
        raise ValueError('capture_timestamp_unavailable')
    captured = int(timestamp_ms * 1_000_000)
    if not 0 <= now_ns - captured < max_age_s * 1e9:
        raise ValueError('capture_timestamp_wrong_clock_or_stale')
    return captured


def _put_latest(q, item):
    """A full one-element queue is replaced or the new item is dropped; never wait."""
    try:
        q.put_nowait(item)
        return
    except queue.Full:
        pass
    try:
        q.get_nowait()
    except queue.Empty:
        pass  # multiprocessing feeder may not have made its item visible yet.
    try:
        q.put_nowait(item)
    except queue.Full:
        pass


def _capture_process(options, frames, status, stop, generation):
    """Only this disposable child owns V4L2; a blocked native read can be killed."""
    cap = None
    try:
        import cv2
        cap = cv2.VideoCapture(options['device'], cv2.CAP_V4L2)
        if not cap.isOpened() or cap.getBackendName() != 'V4L2':
            raise RuntimeError('camera_missing_busy_or_not_v4l2')
        # Respect a busy owner: failed open/read is unavailable, never stop DAQ.
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, options['width'])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, options['height'])
        cap.set(cv2.CAP_PROP_FPS, options['fps'])
        requested = bool(cap.set(cv2.CAP_PROP_BUFFERSIZE, 1))
        _put_latest(status, dict(event='opened', buffer_request_accepted=requested,
                    actual_buffer_count=cap.get(cv2.CAP_PROP_BUFFERSIZE),
                    width=cap.get(cv2.CAP_PROP_FRAME_WIDTH), height=cap.get(cv2.CAP_PROP_FRAME_HEIGHT),
                    camera_fps=cap.get(cv2.CAP_PROP_FPS), timestamp_source=TIMESTAMP_SOURCE))
        last_ns = 0
        while not stop.is_set():
            started = time.monotonic_ns()
            ok, image = cap.read()
            finished = time.monotonic_ns()
            if not ok or image is None:
                raise RuntimeError('camera_read_failed')
            captured = capture_timestamp_ns(cap.get(cv2.CAP_PROP_POS_MSEC), finished, options['max_age_s'])
            if captured <= last_ns:
                raise ValueError('capture_timestamp_not_increasing')
            last_ns = captured
            if image.ndim == 2:
                image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
            _put_latest(frames, Frame(image, captured, started, finished, generation))
    except Exception as exc:
        _put_latest(status, dict(event='error', reason=f'{type(exc).__name__}:{exc}'))
    finally:
        if cap is not None:
            cap.release()


class ProcessCapture:
    """One reader process, one pending frame, bounded retry and shutdown time."""
    def __init__(self, options, *, clock_ns=time.monotonic_ns, context=None):
        self.options, self.clock_ns = dict(options), clock_ns
        self.context = context or mp.get_context('spawn')
        self.process = self.frames = self.status = self.stop_event = None
        self.generation = self.restarts = 0
        self.next_attempt_ns = self.last_frame_ns = 0
        self.reason, self.metadata, self.closed = 'camera_not_started', {}, False
        self.errors = Counter()

    def _stop(self):
        if self.process is None:
            return
        self.stop_event.set()
        self.process.join(.15)
        if self.process.is_alive():
            self.process.terminate()
            self.process.join(.5)
        if self.process.is_alive():
            self.process.kill()
            self.process.join(.5)
        if self.process.is_alive():
            # Never start another reader while the previous one still exists.
            self.closed = True
            raise RuntimeError('capture_reader_did_not_exit')
        self.process.close()
        for q in (self.frames, self.status):
            q.cancel_join_thread()
            q.close()
        self.process = None

    def _failed(self, reason):
        self.reason = reason
        self.errors[reason] += 1
        self._stop()
        self.next_attempt_ns = self.clock_ns() + int(self.options['retry_s'] * 1e9)

    def poll(self):
        if self.closed:
            return None
        now = self.clock_ns()
        if self.process is None:
            if now < self.next_attempt_ns:
                return None
            self.generation += 1
            self.restarts = self.generation - 1
            self.frames, self.status = self.context.Queue(1), self.context.Queue(1)
            self.stop_event = self.context.Event()
            self.process = self.context.Process(target=_capture_process,
                args=(self.options, self.frames, self.status, self.stop_event, self.generation), daemon=True)
            try:
                self.process.start()
            except Exception:
                for q in (self.frames, self.status):
                    q.close()
                self.process = None
                self.reason = 'camera_process_start_failed'
                self.errors[self.reason] += 1
                self.next_attempt_ns = now + int(self.options['retry_s'] * 1e9)
                return None
            self.last_frame_ns = now
            self.reason = 'camera_starting'
        # At most two reads, even when the producer is continuously writing.
        for _ in range(2):
            try:
                event = self.status.get_nowait()
            except queue.Empty:
                break
            if event['event'] == 'error':
                self._failed(event['reason'])
                return None
            self.metadata = event
        frame = None
        for _ in range(2):
            try:
                frame = self.frames.get_nowait()
            except queue.Empty:
                break
        if not self.process.is_alive():
            self._failed('camera_reader_exited')
            return None  # Do not reuse the final buffered frame after disconnect.
        if frame is not None:
            self.last_frame_ns = now
            self.reason = 'camera_ready'
            return frame
        limit_s = 5.0 if self.reason == 'camera_starting' else 2.0
        if now - self.last_frame_ns >= limit_s * 1e9:
            self._failed('camera_read_timeout')
        return None

    def close(self):
        self.closed = True
        self._stop()


class OnnxReference:
    name = 'onnx_cpu_reference'

    def __init__(self, model_path):
        import onnxruntime as ort
        self.model_sha256 = file_sha256(model_path)
        opts = ort.SessionOptions()
        opts.intra_op_num_threads = 2
        opts.inter_op_num_threads = 1
        self.session = ort.InferenceSession(str(model_path), sess_options=opts, providers=['CPUExecutionProvider'])
        inputs, outputs = self.session.get_inputs(), self.session.get_outputs()
        # Original RTMPose export has symbolic batch/joint metadata. We execute
        # batch one and verify concrete shapes on every call; never infer shapes
        # from symbolic labels alone.
        def dimension(value, expected):
            return value == expected or isinstance(value, str) or value is None
        input_shape = inputs[0].shape if len(inputs) == 1 else []
        if (len(inputs) != 1 or inputs[0].name != 'input' or len(input_shape) != 4 or
                not dimension(input_shape[0], 1) or input_shape[1:] != [3, 256, 192]):
            raise ValueError('RTMPose input must be input:[1,3,256,192]')
        shapes = {o.name: o.shape for o in outputs}
        if (set(shapes) != {'simcc_x', 'simcc_y'} or any(
                len(shapes[name]) != 3 or not dimension(shapes[name][0], 1) or
                not dimension(shapes[name][1], 17) or shapes[name][2] != bins
                for name, bins in (('simcc_x', 384), ('simcc_y', 512)))):
            raise ValueError('RTMPose COCO17 SimCC outputs do not match')
        self.last_body_confidence = None
        self.last_body_core_detected = False

    def infer(self, tensor):
        if tuple(tensor.shape) != (1, 3, 256, 192):
            raise ValueError('RTMPose inference requires batch-one tensor')
        outputs = self.session.run(['simcc_x', 'simcc_y'], {'input': tensor})
        if [tuple(value.shape) for value in outputs] != [(1, 17, 384), (1, 17, 512)]:
            raise ValueError('RTMPose concrete output shapes do not match COCO17')
        return outputs


def file_sha256(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def make_backend(name, model_path=None, hybrid_manifest=None):
    if name == 'hailo-hybrid':
        if model_path is not None or not hybrid_manifest:
            raise ValueError('hailo-hybrid requires --hybrid-manifest and no --model')
        from .rtmpose_hailo import HailoHybrid
        return HailoHybrid(hybrid_manifest)
    if hybrid_manifest is not None:
        raise ValueError('--hybrid-manifest is only valid with hailo-hybrid')
    if name == 'hailo':
        raise RuntimeError('hailo_unavailable: compatible RTMPose HEF and Hailo10H backend required; no CPU fallback')
    if name != 'onnx-reference':
        raise ValueError('unknown_backend')
    if not model_path:
        raise ValueError('onnx-reference requires --model')
    return OnnxReference(model_path)


def infer_features(frame, backend, neutral, roi, min_confidence):
    from .rtmpose_math import prepare_input, decode_simcc, extract_body_geometry, relative_features
    backend.last_body_confidence = None
    backend.last_body_core_detected = False
    backend.last_body_geometry = None
    tensor, transform = prepare_input(frame.image, roi)
    simcc_x, simcc_y = backend.infer(tensor)
    xy, scores = decode_simcc(simcc_x, simcc_y, transform)
    # Display the output of this inference only. It never qualifies control
    # features and remains useful while an explicit neutral is still missing.
    backend.last_preview_landmarks = dict(xy=xy.tolist(), scores=scores.tolist(),
                                         roi_xyxy=list(transform.roi_xyxy))
    confidence = float(min(scores[i] for i in (5, 6, 11, 12)))
    backend.last_body_confidence = confidence if math.isfinite(confidence) else None
    geometry = extract_body_geometry(xy, scores, transform, min_confidence=min_confidence)
    backend.last_body_core_detected = True
    from dataclasses import asdict
    backend.last_body_geometry = asdict(geometry)
    if neutral is None:
        raise ValueError('neutral_required')
    return relative_features(geometry, neutral, transform, backend.model_sha256)


class LoopbackPublisher:
    def __init__(self, port):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setblocking(False)
        self.address = ('127.0.0.1', port)

    def send(self, packet):
        self.socket.sendto(json.dumps(packet, allow_nan=False, separators=(',', ':')).encode(), self.address)

    def close(self):
        self.socket.close()


class CameraWorker:
    """Testable optional producer. Each frame is consumed once; failures clear pose."""
    def __init__(self, capture, backend, publisher, *, source_id, camera_id, neutral=None,
                 roi=None, max_age_s=.3, min_confidence=.5, clock_ns=time.monotonic_ns,
                 feature_fn=infer_features, backend_error=None, requested_backend='onnx-reference',
                 preview=None):
        if not isinstance(source_id, str) or not source_id.strip() or len(source_id) > 128:
            raise ValueError('source_id required, maximum 128 characters')
        if not 0 < max_age_s <= 1 or not 0 <= min_confidence <= 1:
            raise ValueError('invalid age/confidence bounds')
        self.capture, self.backend, self.publisher = capture, backend, publisher
        self.source_id, self.camera_id, self.neutral = source_id, camera_id, neutral
        self.roi, self.max_age_s, self.min_confidence = roi, max_age_s, min_confidence
        self.clock_ns, self.feature_fn = clock_ns, feature_fn
        self.backend_error, self.requested_backend = backend_error, requested_backend
        self.preview = preview
        self.stream_id, self.seq = uuid.uuid4().hex, 0
        self.started_ns, self.last_capture_ns = clock_ns(), 0
        self.frames_seen = self.inferences = self.valid_packets = 0
        self.errors = Counter()
        self.reason, self.closed = 'starting', False
        self.inference_ms = self.capture_age_ms = self.read_ms = None
        self.inference_ms_total = self.inference_ms_max = 0.
        self.last_inference_capture_ns = None
        self._first_generation = None
        self.calibration_observation = None

    def _send(self, features=None, capture_ns=0):
        packet = dict(schema=SCHEMA, source_id=self.source_id, stream_id=self.stream_id,
                      seq=self.seq, capture_clock='monotonic', capture_ns=capture_ns,
                      torso_lean_deg=0., lean_score=0., pose_confidence=0.,
                      vision_valid=False, neutral_valid=False)
        if features is not None:
            packet.update(features)
        self.seq += 1
        try:
            self.publisher.send(packet)
        except OSError as exc:
            self.errors['publish_error'] += 1
            self.reason = f'publish_error:{exc}'

    def _unavailable(self, reason):
        if reason not in ('camera_ready', 'no_new_frame'):
            self.calibration_observation = None
        self.reason = reason
        self.errors[reason] += 1
        self._send()  # capture=0/invalid: never pretend this is a newly captured frame.
        if self.preview is not None:
            try:
                self.preview.unavailable(self.summary())
            except Exception:
                self.errors['preview_error'] += 1  # Preview failure cannot alter motor authority.

    def step(self):
        if self.closed:
            return
        if self.backend is None or getattr(self.backend, 'closed', False):
            self._unavailable(self.backend_error or getattr(self.backend, 'failed', None) or 'backend_unavailable')
            return  # An unsupported backend does not unnecessarily acquire the camera.
        try:
            frame = self.capture.poll()
        except Exception as exc:
            self._unavailable(f'capture_error:{type(exc).__name__}:{exc}')
            return
        if frame is None:
            self._unavailable(getattr(self.capture, 'reason', 'no_new_frame'))
            return
        self.frames_seen += 1
        if self._first_generation is None:
            self._first_generation = frame.generation
        if type(frame.capture_ns) is not int:
            self._unavailable('capture_not_fresh_or_increasing')
            return
        age = (self.clock_ns() - frame.capture_ns) / 1e9
        if frame.capture_ns <= self.last_capture_ns or not 0 <= age < self.max_age_s:
            self._unavailable('capture_not_fresh_or_increasing')
            return
        self.last_capture_ns = frame.capture_ns
        self.read_ms = (frame.read_finished_ns - frame.read_started_ns) / 1e6
        neutral = self.neutral
        neutral_reason = None
        if neutral is not None and neutral.get('camera_id') != self.camera_id:
            neutral, neutral_reason = None, 'neutral_camera_mismatch'
        if frame.generation != 1 or frame.generation != self._first_generation:
            neutral, neutral_reason = None, 'camera_reconnected_requires_neutral'
        start = self.clock_ns()
        self.backend.last_preview_landmarks = None  # Never overlay an earlier frame's joints.
        self.backend.last_body_geometry = None
        self.calibration_observation = None
        try:
            self.inferences += 1
            self.last_inference_capture_ns = frame.capture_ns
            features = self.feature_fn(frame, self.backend, neutral, self.roi, self.min_confidence)
            expected = {'torso_lean_deg', 'lean_score', 'pose_confidence', 'vision_valid', 'neutral_valid'}
            if neutral is None:
                raise ValueError(neutral_reason or 'neutral_required')
            if set(features) != expected or features['vision_valid'] is not True or features['neutral_valid'] is not True:
                raise ValueError('invalid_pose_features')
            for name in ('torso_lean_deg', 'lean_score', 'pose_confidence'):
                if isinstance(features[name], bool) or not math.isfinite(float(features[name])):
                    raise ValueError('nonfinite_pose_features')
                features[name] = float(features[name])
            if not (abs(features['torso_lean_deg']) <= 90 and abs(features['lean_score']) <= 4 and
                    self.min_confidence <= features['pose_confidence'] <= 1):
                raise ValueError('pose_features_out_of_range')
            age = (self.clock_ns() - frame.capture_ns) / 1e9
            if not 0 <= age < self.max_age_s:
                raise ValueError('inference_result_stale')
            self.capture_age_ms = age * 1000
            self.reason = 'pose_valid'
            self.valid_packets += 1
            self._send(features, frame.capture_ns)
        except Exception as exc:
            self._unavailable(neutral_reason or f'inference_unavailable:{type(exc).__name__}:{exc}')
        finally:
            self.inference_ms = (self.clock_ns() - start) / 1e6
            self.inference_ms_total += self.inference_ms
            self.inference_ms_max = max(self.inference_ms_max, self.inference_ms)
            self.capture_age_ms = (self.clock_ns() - frame.capture_ns) / 1e6
            geometry = getattr(self.backend, 'last_body_geometry', None)
            if geometry is not None and 0 <= self.capture_age_ms < self.max_age_s * 1000:
                # Read-only calibration evidence from this exact inference. This
                # neither accepts a neutral nor makes the UDP pose valid.
                self.calibration_observation = dict(
                    schema='oasis.pose.calibration_observation.v1',
                    capture_ns=frame.capture_ns, generation=frame.generation,
                    stream_id=self.stream_id, camera_id=self.camera_id,
                    model_sha256=getattr(self.backend, 'model_sha256', None),
                    geometry=geometry)
            if self.preview is not None:
                try:
                    self.preview.submit(frame, self.backend.last_preview_landmarks, self.summary())
                except Exception:
                    self.errors['preview_error'] += 1

    def summary(self):
        elapsed = max((self.clock_ns() - self.started_ns) / 1e9, 1e-9)
        return dict(schema='oasis.pose.camera.status.v1', source_id=self.source_id, stream_id=self.stream_id,
                    requested_backend=self.requested_backend, active_backend=getattr(self.backend, 'name', None),
                    model_sha256=getattr(self.backend, 'model_sha256', None), camera_id=self.camera_id,
                    backend_artifacts=getattr(self.backend, 'artifact_identity', None),
                    backend_completed_inferences=getattr(self.backend, 'completed_inferences', None),
                    reason=self.reason, closed=self.closed, elapsed_s=elapsed, frames=self.frames_seen,
                    inference_fps=self.inferences / elapsed, valid_pose_fps=self.valid_packets / elapsed,
                    inference_ms=self.inference_ms, capture_age_ms=self.capture_age_ms, read_ms=self.read_ms,
                    inference_ms_mean=self.inference_ms_total / self.inferences if self.inferences else None,
                    inference_ms_max=self.inference_ms_max if self.inferences else None,
                    last_inference_capture_ns=self.last_inference_capture_ns,
                    last_body_confidence=getattr(self.backend, 'last_body_confidence', None),
                    last_body_core_detected=getattr(self.backend, 'last_body_core_detected', False),
                    valid_packets=self.valid_packets, errors=dict(self.errors),
                    camera_errors=dict(getattr(self.capture, 'errors', {})),
                    camera_restarts=getattr(self.capture, 'restarts', 0),
                    capture_metadata=getattr(self.capture, 'metadata', {}),
                    calibration_observation=self.calibration_observation,
                    neutral_provided=self.neutral is not None, metric_body_enabled=False)

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.reason = 'closed'
        try:
            self._send()
        finally:
            try:
                self.capture.close()
            finally:
                try:
                    close_backend = getattr(self.backend, 'close', None)
                    if close_backend is not None:
                        close_backend()
                finally:
                    try:
                        self.publisher.close()
                    finally:
                        if self.preview is not None:
                            self.preview.close()


def _status_file(path, worker):
    if path is not None:
        path = Path(path)
        temporary = path.with_name(path.name + '.tmp')
        temporary.write_text(json.dumps(worker.summary(), indent=2, allow_nan=False) + '\n', encoding='utf-8')
        temporary.replace(path)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend', required=True, choices=('onnx-reference', 'hailo-hybrid', 'hailo'))
    parser.add_argument('--model', type=Path)
    parser.add_argument('--hybrid-manifest', type=Path,
                        help='Pinned real Hailo10H HEF + original CPU head; partial acceleration')
    parser.add_argument('--device', default='/dev/video0')
    parser.add_argument('--camera-id', help='Stable camera identity; defaults to the device path')
    parser.add_argument('--source-id', required=True)
    parser.add_argument('--udp-port', type=int, default=47802)
    parser.add_argument('--neutral', type=Path, help='Explicit operator-confirmed neutral JSON with matching camera_id')
    parser.add_argument('--roi', type=int, nargs=4, metavar=('X0', 'Y0', 'X1', 'Y1'))
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--camera-fps', type=float, default=30.)
    parser.add_argument('--publish-hz', type=float, default=10.)
    parser.add_argument('--max-age-s', type=float, default=.3)
    parser.add_argument('--min-confidence', type=float, default=.5)
    parser.add_argument('--retry-s', type=float, default=2.)
    parser.add_argument('--duration', type=float, default=0., help='Seconds; 0 runs until SIGINT/SIGTERM')
    parser.add_argument('--status', type=Path)
    parser.add_argument('--preview-port', type=int, default=0,
                        help='0 disables preview; otherwise serve MJPEG/status on 127.0.0.1 only')
    args = parser.parse_args(argv)
    if not args.source_id.strip() or len(args.source_id) > 128:
        parser.error('source-id required, maximum 128 characters')
    numeric = (args.camera_fps, args.publish_hz, args.max_age_s, args.min_confidence, args.retry_s, args.duration)
    if not all(math.isfinite(v) for v in numeric) or not (0 <= args.preview_port <= 65535 and
            1 <= args.udp_port <= 65535 and
            2 <= args.width <= 4096 and 2 <= args.height <= 4096 and 0 < args.camera_fps <= 120 and
            0 < args.publish_hz <= 30 and .5 <= args.retry_s <= 60 and args.duration >= 0 and
            0 < args.max_age_s <= 1 and 0 <= args.min_confidence <= 1):
        parser.error('invalid port, image, timing or confidence bounds')
    neutral, error, backend = None, None, None
    if args.neutral:
        neutral = json.loads(args.neutral.read_text(encoding='utf-8'))
        if not isinstance(neutral, dict):
            parser.error('neutral must be a JSON object')
    try:
        backend = make_backend(args.backend, args.model, args.hybrid_manifest)
    except Exception as exc:
        error = f'{type(exc).__name__}:{exc}'
    capture = ProcessCapture(dict(device=args.device, width=args.width, height=args.height,
                             fps=args.camera_fps, max_age_s=args.max_age_s, retry_s=args.retry_s))
    publisher = preview = None
    try:
        if args.preview_port:
            try:
                from .pose_preview import PreviewServer
                preview = PreviewServer(args.preview_port, max_age_s=args.max_age_s,
                                        min_confidence=args.min_confidence)
                print('Pose preview (loopback only): ' + preview.url, flush=True)
            except Exception as exc:
                print(f'Pose preview unavailable: {type(exc).__name__}:{exc}', flush=True)
                # An optional web viewer cannot take down the pose publisher.
                preview = None
        publisher = LoopbackPublisher(args.udp_port)
        worker = CameraWorker(capture, backend, publisher, source_id=args.source_id,
                              camera_id=args.camera_id or args.device, neutral=neutral, roi=args.roi,
                              max_age_s=args.max_age_s, min_confidence=args.min_confidence,
                              requested_backend=args.backend, backend_error=error, preview=preview)
    except Exception:
        try:
            capture.close()
        finally:
            try:
                close_backend = getattr(backend, 'close', None)
                if close_backend is not None:
                    close_backend()
            finally:
                if publisher is not None:
                    publisher.close()
                if preview is not None:
                    preview.close()
        raise
    stopped = False
    def stop(_signum, _frame):
        nonlocal stopped
        stopped = True
    old_signals = {sig: signal.signal(sig, stop) for sig in (signal.SIGINT, signal.SIGTERM)}
    start, last_status = time.monotonic(), 0.
    try:
        while not stopped and (not args.duration or time.monotonic() - start < args.duration):
            tick = time.monotonic()
            worker.step()
            if tick - last_status >= 1.:
                _status_file(args.status, worker)
                last_status = tick
            time.sleep(max(0., 1. / args.publish_hz - (time.monotonic() - tick)))
    finally:
        try:
            worker.close()
        finally:
            for sig, previous in old_signals.items():
                signal.signal(sig, previous)
            _status_file(args.status, worker)
    print(json.dumps(worker.summary(), allow_nan=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
