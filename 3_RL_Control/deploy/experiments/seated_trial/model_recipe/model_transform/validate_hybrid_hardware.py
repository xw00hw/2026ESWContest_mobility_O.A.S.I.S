"""Offline-corpus Hailo parity harness; never captures cameras or controls motors.

Default: hash/shape/qualification preflight ONLY (no accelerator access).
--run-device explicitly runs the existing HailoHybrid FLOAT32 backend on all32
heldout inputs. It writes a NEW report, never modifies artifacts/services/config.
The report is research evidence, never camera/control deployment approval.
"""
from pathlib import Path
import argparse
import hashlib
import io
import json
import math
import time
import numpy as np

from oasis_rt import rtmpose_hailo as runtime

CORE = np.array([5, 6, 11, 12])
ARRAY_FILES = {
    'input': 'benchmark_input_normalized_bgr_nhwc.npy',
    'float': 'benchmark_expected_float_nhwc.npy',
    'quantized': 'benchmark_expected_quantized_nhwc.npy',
}


def sha(data):
    return hashlib.sha256(data).hexdigest()


def strict_json(raw):
    return json.loads(raw.decode('utf-8'), object_pairs_hook=runtime._unique_object,
                      parse_constant=lambda value: (_ for _ in ()).throw(ValueError('nonfinite_json')))


def preflight(manifest_path, qualification_path):
    manifest_path, qualification_path = Path(manifest_path).resolve(), Path(qualification_path).resolve()
    manifest, identity, hef, head = runtime.load_manifest(manifest_path)  # file-only, no Hailo import
    contract = runtime.contract_for_boundary(manifest['boundary'])
    raw = qualification_path.read_bytes()
    qa = strict_json(raw)
    exact = dict(schema='oasis.rtmpose.research-qualification.v1', target='HAILO10H',
                 hef_sha256=manifest['hef']['sha256'], source_model_sha256=runtime.SOURCE_SHA256,
                 prefix_onnx_sha256=contract['prefix_sha256'], head_onnx_sha256=contract['head_sha256'],
                 boundary=manifest['boundary'], host_io_dtype='float32',
                 input_shape_nhwc=[1,256,192,3], output_shape_nhwc=[1,*contract['output_shape']],
                 preprocessing='BGR affine1.25 normalized(mean123.675,116.28,103.53;std58.395,57.12,57.375)',
                 model_script_normalization='none; host normalization already applied')
    if any(qa.get(key) != value for key, value in exact.items()):
        raise ValueError('qualification_model_identity_mismatch')
    for key in ('float_precheck_pass', 'research_quality_pass', 'all_reference_confident_core_joints_retained'):
        if qa.get(key) is not True:
            raise ValueError('successful_research_qualification_required:' + key)
    if (qa.get('holdout_indices_for_benchmark') != list(range(32)) or
            any(type(v) is not int for v in qa['holdout_indices_for_benchmark']) or qa.get('heldout_frames') != 32):
        raise ValueError('all32_ordered_heldout_samples_required')
    criteria = qa.get('research_quality_criteria', {})
    if criteria.get('prefix_snr_min_db') != 10.0 or criteria.get('confident_core_max_shift_input_pixels') != 5.0:
        raise ValueError('unexpected_research_quality_criteria_requires_review')
    artifacts = qa.get('artifacts')
    if not isinstance(artifacts, dict):
        raise ValueError('hash_linked_corpus_artifacts_required')
    tensors, hashes = {}, {}
    for kind, filename in ARRAY_FILES.items():
        entry = artifacts.get(filename)
        if not isinstance(entry, dict) or set(entry) != {'sha256', 'bytes'}:
            raise ValueError('missing_corpus_artifact_receipt:' + kind)
        data = (qualification_path.parent / filename).read_bytes()
        if type(entry['bytes']) is not int or len(data) != entry['bytes'] or sha(data) != entry['sha256']:
            raise ValueError('corpus_artifact_hash_mismatch:' + kind)
        arr = np.load(io.BytesIO(data), allow_pickle=False)
        shape = (32, 256, 192, 3) if kind == 'input' else (32, *contract['output_shape'])
        if arr.shape != shape or arr.dtype != np.float32 or not np.isfinite(arr).all():
            raise ValueError('invalid_corpus_tensor:' + kind)
        tensors[kind], hashes[kind] = arr, entry['sha256']
    return dict(manifest=manifest, manifest_sha256=identity, qualification=qa,
                qualification_sha256=sha(raw), corpus_sha256=hashes, head_bytes=head, tensors=tensors,
                manifest_path=str(manifest_path), qualification_path=str(qualification_path), contract=contract)


def tensor_metrics(reference, actual):
    ref, pred = np.asarray(reference, dtype=np.float64), np.asarray(actual, dtype=np.float64)
    if ref.shape != pred.shape or not np.isfinite(ref).all() or not np.isfinite(pred).all():
        raise ValueError('invalid_comparison_tensors')
    error = pred - ref
    power, mse = float(np.mean(ref * ref)), float(np.mean(error * error))
    return dict(snr_db=float(10 * math.log10(max(power, 1e-30) / max(mse, 1e-30))),
                max_abs=float(abs(error).max()), mean_abs=float(abs(error).mean()))


def simcc_geometry(outputs):
    if not isinstance(outputs, (list, tuple)) or len(outputs) != 2:
        raise ValueError('two_simcc_outputs_required')
    for value, bins in zip(outputs, (384, 512)):
        if not isinstance(value, np.ndarray) or value.dtype != np.float32 or value.shape != (1, 17, bins) or not np.isfinite(value).all():
            raise ValueError('invalid_simcc_output')
    x, y = outputs
    xy = np.stack((x.argmax(axis=2), y.argmax(axis=2)), axis=-1)[0].astype(np.float64) / 2.0
    confidence = ((x.max(axis=2) + y.max(axis=2)) * 0.5)[0]
    return xy, confidence


def compare_simcc(reference, actual):
    rxy, rconf = simcc_geometry(reference)
    axy, aconf = simcc_geometry(actual)
    good = rconf[CORE] >= 0.5
    delta = abs(axy[CORE] - rxy[CORE])
    shifts = np.max(delta, axis=1)[good]
    return dict(reference_confident_core_count=int(good.sum()),
                retained_confident_core_count=int(np.sum(good & (aconf[CORE] >= 0.5))),
                core_joint_indices=CORE.tolist(),
                reference_core_xy_input_pixels=rxy[CORE].tolist(), actual_core_xy_input_pixels=axy[CORE].tolist(),
                confident_core_axis_max_shifts_input_pixels=shifts.tolist(),
                confident_core_euclidean_shifts_input_pixels=np.linalg.norm(delta[good], axis=1).tolist(),
                reference_core_confidence=rconf[CORE].tolist(), actual_core_confidence=aconf[CORE].tolist(),
                simcc_x=tensor_metrics(reference[0], actual[0]), simcc_y=tensor_metrics(reference[1], actual[1]))


def summarize_comparisons(rows):
    shifts = [v for row in rows for v in row['confident_core_axis_max_shifts_input_pixels']]
    count = sum(r['reference_confident_core_count'] for r in rows)
    retained = sum(r['retained_confident_core_count'] for r in rows)
    return dict(reference_confident_core_count=count, retained_confident_core_count=retained,
                all_reference_confident_core_joints_retained=bool(count and retained == count),
                core_max_axis_shift_input_pixels=max(shifts) if shifts else None,
                core_axis_shift_p50_input_pixels=float(np.percentile(shifts, 50)) if shifts else None,
                core_axis_shift_p95_input_pixels=float(np.percentile(shifts, 95)) if shifts else None,
                simcc_max_abs=max(max(r['simcc_x']['max_abs'], r['simcc_y']['max_abs']) for r in rows))


class CaptureHead:
    """Observe existing CPU-head feed, without replacing its numerical computation."""
    def __init__(self, session, clock=time.perf_counter_ns, *, boundary='Conv_205:625'):
        self.session, self.clock = session, clock
        self.contract = runtime.contract_for_boundary(boundary)
        self.prefix_nhwc, self.last_latency_ns = None, None

    def run(self, names, feed):
        name = self.contract['head_input_name']
        if names != ['simcc_x', 'simcc_y'] or set(feed) != {name}:
            raise ValueError('unexpected_cpu_head_contract')
        value = feed[name]
        if value.shape != tuple(self.contract['head_input_shape']) or value.dtype != np.float32 or not np.isfinite(value).all():
            raise ValueError('invalid_actual_head_input')
        self.prefix_nhwc = np.ascontiguousarray(value.transpose(0, 2, 3, 1))
        started = self.clock()
        result = self.session.run(names, feed)
        self.last_latency_ns = self.clock() - started
        return result


def acquire(backend_factory, inputs, warmups=2, repeats=1, clock=time.perf_counter_ns, state=None):
    """Only function that invokes a supplied device backend; always closes it."""
    if type(warmups) is not int or not 0 <= warmups <= 10 or type(repeats) is not int or not 1 <= repeats <= 10:
        raise ValueError('invalid_bounded_inference_count')
    state = {} if state is None else state
    state['device_access_attempted'] = True
    backend = backend_factory()
    try:
        capture = CaptureHead(backend.session, clock, boundary=backend.manifest['boundary'])
        backend.session = capture
        first = np.ascontiguousarray(inputs[0:1].transpose(0, 3, 1, 2))
        for _ in range(warmups):
            state['hardware_inference_attempted'] = True
            backend.infer(first)
            state['completed_inferences'] = state.get('completed_inferences', 0) + 1
        rows = []
        for repeat in range(repeats):
            for index, item in enumerate(inputs):
                tensor = np.ascontiguousarray(item[None].transpose(0, 3, 1, 2))
                started = clock()
                state['hardware_inference_attempted'] = True
                outputs = backend.infer(tensor)
                state['completed_inferences'] = state.get('completed_inferences', 0) + 1
                elapsed = clock() - started
                simcc_geometry(outputs)
                if capture.prefix_nhwc is None or capture.last_latency_ns is None:
                    raise RuntimeError('runtime_did_not_use_verified_cpu_head')
                rows.append(dict(index=index, repeat=repeat, prefix=capture.prefix_nhwc.copy(), outputs=outputs,
                                 hybrid_latency_ms=elapsed / 1e6, head_latency_ms=capture.last_latency_ns / 1e6))
        return rows
    finally:
        state['backend_close_attempted'] = True
        backend.close()
        state['backend_closed'] = True


def run_validation(bundle, warmups, repeats, state=None):
    import onnxruntime as ort  # CPU reference; Hailo import stays inside backend.
    opts = ort.SessionOptions()
    opts.intra_op_num_threads, opts.inter_op_num_threads = 2, 1
    head = ort.InferenceSession(bundle['head_bytes'], sess_options=opts, providers=['CPUExecutionProvider'])
    references = {}
    for kind in ('float', 'quantized'):
        references[kind] = [head.run(['simcc_x', 'simcc_y'], {bundle['contract']['head_input_name']: np.ascontiguousarray(x[None].transpose(0, 3, 1, 2))})
                            for x in bundle['tensors'][kind]]
    # Check reference confidence before opening any device.
    if any(np.any(simcc_geometry(o)[1][CORE] < 0.5) for o in references['float']):
        raise ValueError('packaged_float_reference_missing_required_core_confidence')
    actual = acquire(lambda: runtime.HailoHybrid(bundle['manifest_path']), bundle['tensors']['input'], warmups, repeats, state=state)
    result = {}
    for kind in ('float', 'quantized'):
        comparisons = [compare_simcc(references[kind][r['index']], r['outputs']) for r in actual]
        expected = np.concatenate([bundle['tensors'][kind][r['index']:r['index']+1] for r in actual])
        observed = np.concatenate([r['prefix'] for r in actual])
        result[kind] = dict(prefix=tensor_metrics(expected, observed), pose=summarize_comparisons(comparisons),
                            per_frame=[dict(index=r['index'], repeat=r['repeat'], **c) for r, c in zip(actual, comparisons)])
    native = result['float']
    quality = bool(native['prefix']['snr_db'] >= 10 and native['pose']['all_reference_confident_core_joints_retained'] and
                   native['pose']['core_max_axis_shift_input_pixels'] is not None and native['pose']['core_max_axis_shift_input_pixels'] <= 5)
    # Report hardware-vs-SDK error separately; no arbitrary equality tolerance is
    # promoted into deployment approval. Both are real dequantized float values.
    latency = {}
    for key in ('hybrid_latency_ms', 'head_latency_ms'):
        values = [r[key] for r in actual]
        latency[key] = dict(min=min(values), p50=float(np.percentile(values, 50)), p95=float(np.percentile(values, 95)), max=max(values))
    return dict(hardware_inference_executed=True, measured_frames=len(actual), unique_heldout_frames=32,
                warmup_frames=warmups, repeats=repeats, comparisons=result, latency=latency,
                native_research_quality_pass=quality, hardware_sdk_parity_requires_review=True,
                latency_includes_head_observation_copy=True)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest', required=True, type=Path)
    p.add_argument('--qualification', required=True, type=Path)
    p.add_argument('--output', required=True, type=Path)
    p.add_argument('--run-device', action='store_true')
    p.add_argument('--warmups', type=int, default=2)
    p.add_argument('--repeats', type=int, default=1)
    args = p.parse_args(argv)
    if args.output.exists() or not args.output.parent.exists():
        p.error('output must be a new file in an existing directory')
    if not 0 <= args.warmups <= 10 or not 1 <= args.repeats <= 10:
        p.error('warmups0..10, repeats1..10 required')
    report = dict(schema='oasis.rtmpose.hardware-parity-report.v1', camera_opened=False,
                  motors_or_control_accessed=False, services_or_configuration_changed=False,
                  deployment_approved=False, pose_control_usable=False,
                  current_ov9281_representativeness_verified=False,
                  hardware_inference_executed=False, hardware_sdk_parity_requires_review=True,
                  harness_sha256=sha(Path(__file__).read_bytes()),
                  runtime_backend_sha256=sha(Path(runtime.__file__).read_bytes()))
    code = 0
    state = {}
    try:
        bundle = preflight(args.manifest, args.qualification)
        report.update(preflight_pass=True, manifest_sha256=bundle['manifest_sha256'],
                      qualification_sha256=bundle['qualification_sha256'], corpus_sha256=bundle['corpus_sha256'],
                      source_model_sha256=runtime.SOURCE_SHA256, prefix_onnx_sha256=bundle['contract']['prefix_sha256'],
                      head_sha256=bundle['contract']['head_sha256'], hef_sha256=bundle['manifest']['hef']['sha256'],
                      boundary=bundle['manifest']['boundary'],
                      device_id=bundle['manifest']['device_id'], target='HAILO10H',
                      host_io='normalized_BGR_FLOAT32_NHWC; runtime_transposes_head_to_NCHW; no_manual_quantization')
        if args.run_device:
            report.update(run_validation(bundle, args.warmups, args.repeats, state))
            code = 0 if report['native_research_quality_pass'] else 2
    except Exception as exc:
        report['failure'] = type(exc).__name__ + ':' + str(exc)
        code = 1
    report['device_execution_state'] = state
    report['hardware_inference_executed'] = state.get('completed_inferences', 0) > 0
    with args.output.open('x', encoding='utf-8') as f:
        json.dump(report, f, indent=2, allow_nan=False)
        f.write('\n')
    print(json.dumps(dict(report=str(args.output), hardware_inference_executed=report['hardware_inference_executed'],
                          native_research_quality_pass=report.get('native_research_quality_pass'),
                          deployment_approved=False, exit_code=code)))
    return code


if __name__ == '__main__':
    raise SystemExit(main())
