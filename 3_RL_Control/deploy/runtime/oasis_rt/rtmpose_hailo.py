"""Allowlisted RTMPose Hailo10H prefixes + their exact original ONNX CPU heads.

This is PARTIAL acceleration, not a native full-model HEF. API calls follow the
official HailoRT v5.4.0 InferModel Python bindings. Unit tests use stubs; they do
not establish quantized accuracy, device throughput, or deployment readiness.

The JSON manifest is deliberately required (no default/fabricated HEF). Schema
``oasis.rtmpose.hailo-hybrid.v1`` contains source_model_sha256,
prefix_onnx_sha256, boundary='Conv_205:625' or 'Mul_204:624', target='HAILO10H',
hailort_version='5.4.0', preprocessing=PREPROCESSING, device_id, timeout_ms,
hef={path,sha256}, head={path,sha256}, and input/output={name,shape,layout,
dtype,quantization}. Stream shapes EXCLUDE batch, layout='NHWC',
dtype='float32'; quantization is the HEF's list of {qp_scale,qp_zp} pairs.
Input is [256,192,3]; output is [8,6,17] for Conv_205 or [8,6,768] for
Mul_204. Each boundary pins its own prefix/head hashes and CPU input contract.
Paths resolve against the manifest.
device_id is a complete local PCI BDF, optionally prefixed by ``pci/``.

Normalized BGR float input is quantized by HailoRT; FLOAT32 output is already
dequantized by HailoRT. No uint8 cast, manual scale, or second normalization is
performed. The HEF must be compiled with this same normalized input contract.
The manifest hash identifies the execution pipeline for neutral calibration;
the original source model hash remains separately available for provenance.
"""
from contextlib import ExitStack
import hashlib
import json
import math
from pathlib import Path
import re
from types import MappingProxyType
from typing import NamedTuple

import numpy as np


SOURCE_SHA256 = '5c0a4bf67953e6d2ac43ce15e77dc9d5d354ae18430a47d2c5963a7bc5683e3c'
PREFIX_SHA256 = 'e8466a8002311f883a8716ff85005eaac19055b02d9bc63910dabe7625313b98'
HEAD_SHA256 = 'ecf100b549adc0b7ca19fade010a8be27a95057d695153d9db716a10c8aa895b'
PREPROCESSING = 'rtmlib_bgr_affine1.25_mean123.675,116.28,103.53_std58.395,57.12,57.375'


class _BoundaryContract(NamedTuple):
    prefix_sha256: str
    head_sha256: str
    head_input_name: str
    head_input_shape: tuple
    output_shape: tuple


# Original constants above remain the Conv_205 aliases for existing callers.
# Only these exact graph splits are accepted; a manifest cannot define a head.
_BOUNDARY_CONTRACTS = MappingProxyType({
    'Conv_205:625': _BoundaryContract(PREFIX_SHA256, HEAD_SHA256, '625',
                                    (1, 17, 8, 6), (8, 6, 17)),
    'Mul_204:624': _BoundaryContract(
        'feb57245237d06b24450b59843beb8e7faf977c599b18c39c4ac0fc80b63e028',
        '62815b915d90d26607282a86aa403652225aed8ee1447d22c0e48bcd5d493251',
        '624', (1, 768, 8, 6), (8, 6, 768)),
})


def contract_for_boundary(boundary):
    """Return a fresh fixed contract; unknown boundaries cannot supply shapes/hashes."""
    if not isinstance(boundary, str) or boundary not in _BOUNDARY_CONTRACTS:
        raise ValueError('unsupported_hybrid_boundary')
    contract = _BOUNDARY_CONTRACTS[boundary]
    return dict(prefix_sha256=contract.prefix_sha256, head_sha256=contract.head_sha256,
                head_input_name=contract.head_input_name,
                head_input_shape=list(contract.head_input_shape), output_shape=list(contract.output_shape))


def _sha256(data):
    return hashlib.sha256(data).hexdigest()


def _pci_bdf(value):
    """Match only a full local PCI address, never USB, TCP or domain wildcards."""
    if not isinstance(value, str):
        raise ValueError('explicit_pcie_device_id_required')
    match = re.fullmatch(r'(?:pci/)?([0-9a-fA-F]{4}:[0-9a-fA-F]{2}:[01][0-9a-fA-F]\.[0-7])', value)
    if match is None:
        raise ValueError('explicit_pcie_device_id_required')
    return match.group(1).lower()


def _discovered_pci_id(configured, discovered):
    expected = _pci_bdf(configured)
    matches = []
    for candidate in discovered:
        try:
            canonical = _pci_bdf(candidate)
        except ValueError:
            continue
        if canonical == expected:
            matches.append(candidate)
    if not matches:
        raise RuntimeError('configured_hailo_device_missing')
    if len(matches) != 1:
        raise RuntimeError('configured_hailo_device_ambiguous')
    return matches[0]  # Preserve the actual HailoRT identifier, including pci/.


def _unique_object(pairs):
    obj = {}
    for key, value in pairs:
        if key in obj:
            raise ValueError('duplicate_manifest_key')
        obj[key] = value
    return obj


def _quantization(value):
    if not isinstance(value, list) or not value:
        raise ValueError('missing_quantization_metadata')
    result = []
    for pair in value:
        if not isinstance(pair, dict) or set(pair) != {'qp_scale', 'qp_zp'}:
            raise ValueError('invalid_quantization_metadata')
        scale, zero = pair['qp_scale'], pair['qp_zp']
        if (type(scale) not in (int, float) or type(zero) not in (int, float) or
                not math.isfinite(scale) or not math.isfinite(zero) or scale <= 0):
            raise ValueError('invalid_quantization_metadata')
        result.append((float(scale), float(zero)))
    return result


def _artifact_bytes(root, entry, pinned=None):
    if not isinstance(entry, dict) or set(entry) != {'path', 'sha256'}:
        raise ValueError('artifact_path_and_sha256_required')
    digest = entry['sha256']
    if not isinstance(digest, str) or not re.fullmatch('[0-9a-f]{64}', digest):
        raise ValueError('invalid_artifact_sha256')
    if pinned is not None and digest != pinned:
        raise ValueError('unsupported_cpu_head_identity')
    if not isinstance(entry['path'], str) or not entry['path']:
        raise ValueError('artifact_path_required')
    path = (root / entry['path']).resolve()
    data = path.read_bytes()  # Load verified bytes, avoiding a hash/open path race.
    if not data or _sha256(data) != digest:
        raise ValueError('artifact_sha256_mismatch')
    return data


def load_manifest(path):
    path = Path(path).resolve()
    if path.stat().st_size > 64 * 1024:
        raise ValueError('manifest_too_large')
    raw = path.read_bytes()
    manifest = json.loads(raw.decode('utf-8'), object_pairs_hook=_unique_object,
                          parse_constant=lambda value: (_ for _ in ()).throw(ValueError('nonfinite_manifest')))
    required = {'schema', 'source_model_sha256', 'prefix_onnx_sha256', 'boundary', 'target',
                'hailort_version', 'preprocessing', 'device_id', 'timeout_ms', 'hef', 'head', 'input', 'output'}
    if not isinstance(manifest, dict) or set(manifest) != required:
        raise ValueError('invalid_hybrid_manifest_fields')
    contract = contract_for_boundary(manifest['boundary'])
    expected = dict(schema='oasis.rtmpose.hailo-hybrid.v1', source_model_sha256=SOURCE_SHA256,
                    prefix_onnx_sha256=contract['prefix_sha256'], target='HAILO10H',
                    hailort_version='5.4.0', preprocessing=PREPROCESSING)
    if any(manifest[key] != value for key, value in expected.items()):
        raise ValueError('unsupported_hybrid_model_contract')
    _pci_bdf(manifest['device_id'])
    if type(manifest['timeout_ms']) is not int or not 1 <= manifest['timeout_ms'] <= 1000:
        raise ValueError('invalid_inference_timeout')
    for key, shape in (('input', [256, 192, 3]), ('output', contract['output_shape'])):
        stream = manifest[key]
        if (not isinstance(stream, dict) or set(stream) != {'name', 'shape', 'layout', 'dtype', 'quantization'} or
                not isinstance(stream['name'], str) or not stream['name'] or
                stream['shape'] != shape or any(type(v) is not int for v in stream['shape']) or
                stream['layout'] != 'NHWC' or stream['dtype'] != 'float32'):
            raise ValueError('invalid_hybrid_stream_contract')
        _quantization(stream['quantization'])
    hef = _artifact_bytes(path.parent, manifest['hef'])
    head = _artifact_bytes(path.parent, manifest['head'], contract['head_sha256'])
    return manifest, _sha256(raw), hef, head


def _tensor(value, shape, name):
    if (not isinstance(value, np.ndarray) or value.shape != tuple(shape) or
            value.dtype != np.float32 or not np.isfinite(value).all()):
        raise ValueError(f'invalid_{name}_tensor')
    return np.ascontiguousarray(value)


class HailoHybrid:
    name = 'hailo10h_conv205_onnx_head'

    def __init__(self, manifest_path, *, hailo_module=None, ort_module=None):
        self.closed, self.failed, self.completed_inferences = False, None, 0
        self._resources = ExitStack()
        self.configured = self.session = None
        self.last_body_confidence, self.last_body_core_detected = None, False
        manifest, identity, hef_bytes, head_bytes = load_manifest(manifest_path)
        self.manifest = manifest
        self._contract = contract_for_boundary(manifest['boundary'])
        self.name = 'hailo10h_' + manifest['boundary'].split(':')[0].replace('_', '').lower() + '_onnx_head'
        self.model_sha256 = identity
        self.source_model_sha256 = SOURCE_SHA256
        self.artifact_identity = dict(manifest_sha256=identity, hef_sha256=manifest['hef']['sha256'],
                                      head_sha256=self._contract['head_sha256'],
                                      prefix_onnx_sha256=self._contract['prefix_sha256'],
                                      boundary=manifest['boundary'],
                                      source_model_sha256=SOURCE_SHA256, partial_acceleration=True)
        # Dependencies import only after all artifacts and contracts are checked.
        if ort_module is None:
            import onnxruntime as ort_module
        if hailo_module is None:
            import hailo_platform as hailo_module
        hp, ort = hailo_module, ort_module
        if hp.__version__ != manifest['hailort_version']:
            raise RuntimeError('hailort_version_mismatch')
        try:
            opts = ort.SessionOptions()
            opts.intra_op_num_threads, opts.inter_op_num_threads = 2, 1
            self.session = ort.InferenceSession(head_bytes, sess_options=opts, providers=['CPUExecutionProvider'])
            self._validate_head()
            # Identify exactly the selected device; never take the first available accelerator.
            self.discovered_device_id = _discovered_pci_id(manifest['device_id'], hp.Device.scan())
            with hp.Device(self.discovered_device_id) as device:
                if str(device.control.identify().device_architecture) != 'HAILO10H':
                    raise RuntimeError('hailo_device_architecture_mismatch')
            # HailoRT5.4 defaults are ROUND_ROBIN, single process, no shared service.
            self.device = self._resources.enter_context(hp.VDevice(device_ids=[self.discovered_device_id]))
            self.model = self.device.create_infer_model(hef_bytes)
            if (list(self.model.input_names) != [manifest['input']['name']] or
                    list(self.model.output_names) != [manifest['output']['name']]):
                raise ValueError('hef_stream_names_mismatch')
            self.model.set_batch_size(1)
            for key in ('input', 'output'):
                spec = manifest[key]
                stream = getattr(self.model, key)(spec['name'])
                if list(stream.shape) != spec['shape'] or stream.is_nms:
                    raise ValueError('hef_stream_shape_mismatch')
                actual = _quantization([dict(qp_scale=float(q.qp_scale), qp_zp=float(q.qp_zp))
                                        for q in stream.quant_infos])
                expected = _quantization(spec['quantization'])
                if len(actual) != len(expected) or not np.allclose(actual, expected, rtol=1e-6, atol=1e-12):
                    raise ValueError('hef_quantization_mismatch')
                stream.set_format_order(hp.FormatOrder.NHWC)
                stream.set_format_type(hp.FormatType.FLOAT32)
                if stream.format.order != hp.FormatOrder.NHWC or stream.format.type != hp.FormatType.FLOAT32:
                    raise ValueError('hailort_host_format_mismatch')
            self.configured = self._resources.enter_context(self.model.configure())
        except Exception:
            self.close()
            raise

    def _validate_head(self):
        inputs, outputs = self.session.get_inputs(), self.session.get_outputs()
        if (len(inputs) != 1 or inputs[0].name != self._contract['head_input_name'] or
                inputs[0].shape != self._contract['head_input_shape'] or
                inputs[0].type != 'tensor(float)' or len(outputs) != 2 or
                {o.name: (o.shape, o.type) for o in outputs} != {
                    'simcc_x': ([1, 17, 384], 'tensor(float)'),
                    'simcc_y': ([1, 17, 512], 'tensor(float)')}):
            raise ValueError('cpu_head_contract_mismatch')

    def infer(self, tensor):
        if self.closed:
            raise RuntimeError(self.failed or 'hybrid_backend_closed')
        tensor = _tensor(tensor, (1, 3, 256, 192), 'input')
        # InferModel Bindings represent ONE frame, not a batched tensor.
        image = np.ascontiguousarray(tensor[0].transpose(1, 2, 0))
        features = np.full(self._contract['output_shape'], np.nan, dtype=np.float32)
        try:
            bindings = self.configured.create_bindings(
                input_buffers={self.manifest['input']['name']: image},
                output_buffers={self.manifest['output']['name']: features})
            timeout = self.manifest['timeout_ms']
            self.configured.wait_for_async_ready(timeout_ms=timeout, frames_count=1)
            self.configured.run([bindings], timeout=timeout)
            features = _tensor(features, self._contract['output_shape'], 'dequantized_prefix_output')
            head_input = np.ascontiguousarray(features.transpose(2, 0, 1)[None])
            result = self.session.run(['simcc_x', 'simcc_y'], {self._contract['head_input_name']: head_input})
            if not isinstance(result, (list, tuple)) or len(result) != 2:
                raise ValueError('invalid_cpu_head_outputs')
            outputs = [_tensor(value, (1, 17, bins), name)
                       for value, bins, name in zip(result, (384, 512), ('simcc_x', 'simcc_y'))]
            self.completed_inferences += 1
            return outputs
        except Exception as exc:
            # A timeout may leave DMA in flight. Latch unavailable, shut down and
            # never reuse buffers/device until an explicit worker restart.
            self.failed = f'hybrid_runtime_failed:{type(exc).__name__}:{exc}'
            try:
                self.close()
            except Exception:
                pass
            raise

    def close(self):
        if self.closed:
            return
        self.closed = True
        try:
            if self.configured is not None:
                self.configured.shutdown()
        finally:
            self._resources.close()
            self.session = None
