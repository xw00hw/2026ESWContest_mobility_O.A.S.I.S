"""Stub HailoRT API tests ONLY: no accelerator, camera, socket or motor access.

These check identity, layout, float I/O and failure lifecycle. They do not prove
quantized accuracy or hardware inference; those require the real compiled HEF.
"""
import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from oasis_rt import rtmpose_hailo as hybrid

REAL_HEAD_SHA256 = hybrid.HEAD_SHA256
REAL_BOUNDARY_CONTRACTS = hybrid._BOUNDARY_CONTRACTS


class FakeOrt:
    SessionOptions = NS
    def __init__(self, head_input_name='625', head_input_shape=(1, 17, 8, 6)):
        self.sessions = []
        self.bad_meta = False
        self.bad_result = None
        self.head_input_name, self.head_input_shape = head_input_name, list(head_input_shape)
    def InferenceSession(self, data, **kwargs):
        owner = self
        class Session:
            def get_inputs(self):
                return [NS(name=owner.head_input_name, shape=owner.head_input_shape if not owner.bad_meta else [1, 8, 6, 17],
                           type='tensor(float)')]
            def get_outputs(self):
                return [NS(name='simcc_x', shape=[1, 17, 384], type='tensor(float)'),
                        NS(name='simcc_y', shape=[1, 17, 512], type='tensor(float)')]
            def run(self, names, inputs):
                self.names, self.inputs = names, inputs
                if owner.bad_result is not None:
                    return owner.bad_result
                return [np.zeros((1, 17, bins), np.float32) for bins in (384, 512)]
        session = Session()
        session.model_bytes, session.options = data, kwargs
        self.sessions.append(session)
        return session


class FakeHailo:
    __version__ = '5.4.0'
    FormatType = NS(FLOAT32='float32')
    FormatOrder = NS(NHWC='nhwc')
    def __init__(self, output_shape=(8, 6, 17), output_name='prefix/conv50'):
        owner = self
        self.events, self.models, self.vdevices = [], [], []
        self.devices = ['pci/0000:01:00.0']
        self.architecture = 'HAILO10H'
        self.input_name, self.output_name = 'prefix/input_layer1', output_name
        self.input_shape, self.output_shape = [256, 192, 3], list(output_shape)
        self.quant = [NS(qp_scale=.125, qp_zp=111.)]
        self.ignored_format = False
        self.run_error = self.configure_error = self.shutdown_error = None
        self.bad_features = False
        self.features = (np.arange(np.prod(output_shape)).reshape(output_shape).astype(np.float32) - 55) / 8
        class Device:
            @staticmethod
            def scan():
                owner.events.append('scan')
                return owner.devices
            def __init__(self, device_id):
                owner.events.append(('identify', device_id))
                self.control = NS(identify=lambda: NS(device_architecture=owner.architecture))
            def __enter__(self):
                return self
            def __exit__(self, *_):
                owner.events.append('identity_device_closed')
        class Stream:
            def __init__(self, shape):
                self.shape, self.is_nms = shape, False
                self.quant_infos = owner.quant
                self.format = NS(order='native', type='uint8')
            def set_format_order(self, value):
                if not owner.ignored_format:
                    self.format.order = value
            def set_format_type(self, value):
                if not owner.ignored_format:
                    self.format.type = value
        class Configured:
            def __enter__(self):
                return self
            def __exit__(self, *_):
                owner.events.append('configured_closed')
            def create_bindings(self, input_buffers, output_buffers):
                self.bindings = NS(inputs=input_buffers, outputs=output_buffers)
                return self.bindings
            def wait_for_async_ready(self, **kwargs):
                owner.events.append(('ready', kwargs))
            def run(self, bindings, timeout):
                owner.events.append(('run', timeout))
                if owner.run_error:
                    raise owner.run_error
                assert len(bindings) == 1
                if not owner.bad_features:
                    bindings[0].outputs[owner.output_name][:] = owner.features
            def shutdown(self):
                owner.events.append('shutdown')
                if owner.shutdown_error:
                    raise owner.shutdown_error
        class Model:
            def __init__(self):
                self.input_names, self.output_names = [owner.input_name], [owner.output_name]
                self.streams = dict(input=Stream(owner.input_shape), output=Stream(owner.output_shape))
            def input(self, _name):
                return self.streams['input']
            def output(self, _name):
                return self.streams['output']
            def set_batch_size(self, batch):
                self.batch_size = batch
            def configure(self):
                if owner.configure_error:
                    raise owner.configure_error
                self.configured = Configured()
                return self.configured
        class VDevice:
            def __init__(self, *, device_ids):
                owner.vdevices.append(self)
                self.device_ids = device_ids
            def __enter__(self):
                return self
            def __exit__(self, *_):
                owner.events.append('vdevice_closed')
            def create_infer_model(self, data):
                self.hef_bytes = data
                model = Model()
                owner.models.append(model)
                return model
        self.Device, self.VDevice = Device, VDevice


class TestHybridBackend(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.hef_data, self.head_data = b'STUB ONLY - NOT A HEF', b'STUB ONLY - NOT ONNX'
        (self.root / 'test.hef').write_bytes(self.hef_data)
        (self.root / 'head.onnx').write_bytes(self.head_data)
        self.head_sha = hashlib.sha256(self.head_data).hexdigest()
        self.mul_head_data = b'STUB MUL204 CPU HEAD - NOT ONNX'
        (self.root / 'mul_head.onnx').write_bytes(self.mul_head_data)
        self.mul_head_sha = hashlib.sha256(self.mul_head_data).hexdigest()
        stub_contracts = {boundary: contract._replace(head_sha256=(
            self.head_sha if boundary == 'Conv_205:625' else self.mul_head_sha))
            for boundary, contract in REAL_BOUNDARY_CONTRACTS.items()}
        patcher = patch.object(hybrid, '_BOUNDARY_CONTRACTS', stub_contracts)
        patcher.start()
        self.addCleanup(patcher.stop)
        self.manifest = dict(schema='oasis.rtmpose.hailo-hybrid.v1',
            source_model_sha256=hybrid.SOURCE_SHA256, prefix_onnx_sha256=hybrid.PREFIX_SHA256,
            boundary='Conv_205:625', target='HAILO10H', hailort_version='5.4.0',
            preprocessing=hybrid.PREPROCESSING, device_id='0000:01:00.0', timeout_ms=150,
            hef=dict(path='test.hef', sha256=hashlib.sha256(self.hef_data).hexdigest()),
            head=dict(path='head.onnx', sha256=self.head_sha),
            input=dict(name='prefix/input_layer1', shape=[256, 192, 3], layout='NHWC',
                       dtype='float32', quantization=[dict(qp_scale=.125, qp_zp=111.)]),
            output=dict(name='prefix/conv50', shape=[8, 6, 17], layout='NHWC',
                        dtype='float32', quantization=[dict(qp_scale=.125, qp_zp=111.)]))
        self.path = self.root / 'hybrid.json'
        self.hp, self.ort = FakeHailo(), FakeOrt()

    def write_manifest(self):
        self.path.write_text(json.dumps(self.manifest), encoding='utf-8')

    def backend(self):
        self.write_manifest()
        return hybrid.HailoHybrid(self.path, hailo_module=self.hp, ort_module=self.ort)

    def select_boundary(self, boundary):
        contract = hybrid.contract_for_boundary(boundary)
        self.manifest['boundary'] = boundary
        self.manifest['prefix_onnx_sha256'] = contract['prefix_sha256']
        self.manifest['head'] = dict(path='head.onnx' if boundary == 'Conv_205:625' else 'mul_head.onnx',
                                     sha256=contract['head_sha256'])
        self.manifest['output']['shape'] = contract['output_shape']
        self.hp = FakeHailo(output_shape=contract['output_shape'])
        self.ort = FakeOrt(contract['head_input_name'], contract['head_input_shape'])

    def test_only_two_exact_contracts_and_helper_returns_independent_values(self):
        with patch.object(hybrid, '_BOUNDARY_CONTRACTS', REAL_BOUNDARY_CONTRACTS):
            old = hybrid.contract_for_boundary('Conv_205:625')
            self.assertEqual(old, dict(prefix_sha256=hybrid.PREFIX_SHA256,
                head_sha256=REAL_HEAD_SHA256, head_input_name='625',
                head_input_shape=[1, 17, 8, 6], output_shape=[8, 6, 17]))
            new = hybrid.contract_for_boundary('Mul_204:624')
            self.assertEqual(new, dict(
                prefix_sha256='feb57245237d06b24450b59843beb8e7faf977c599b18c39c4ac0fc80b63e028',
                head_sha256='62815b915d90d26607282a86aa403652225aed8ee1447d22c0e48bcd5d493251',
                head_input_name='624', head_input_shape=[1, 768, 8, 6], output_shape=[8, 6, 768]))
            new['output_shape'][2] = 17
            new['head_sha256'] = '0' * 64
            self.assertEqual(hybrid.contract_for_boundary('Mul_204:624')['output_shape'], [8, 6, 768])
            self.assertNotEqual(hybrid.contract_for_boundary('Mul_204:624')['head_sha256'], '0' * 64)
            for boundary in ('Mul_204:625', 'Conv_205:624', 'Conv_203:622', '', None, [], {}):
                with self.subTest(boundary=boundary), self.assertRaisesRegex(ValueError, 'unsupported_hybrid_boundary'):
                    hybrid.contract_for_boundary(boundary)

    def test_mul204_float_layout_head_and_selected_artifact_identity(self):
        old = self.backend()
        old_identity = old.model_sha256
        old.close()
        self.select_boundary('Mul_204:624')
        backend = self.backend()
        result = backend.infer(np.zeros((1, 3, 256, 192), np.float32))
        np.testing.assert_array_equal(self.ort.sessions[0].inputs['624'], self.hp.features.transpose(2, 0, 1)[None])
        self.assertEqual(self.ort.sessions[0].inputs['624'].shape, (1, 768, 8, 6))
        self.assertEqual(self.hp.models[0].configured.bindings.outputs[self.hp.output_name].shape, (8, 6, 768))
        self.assertEqual(self.ort.sessions[0].model_bytes, self.mul_head_data)
        self.assertEqual([x.shape for x in result], [(1, 17, 384), (1, 17, 512)])
        self.assertEqual(backend.name, 'hailo10h_mul204_onnx_head')
        self.assertEqual(backend.artifact_identity['prefix_onnx_sha256'], self.manifest['prefix_onnx_sha256'])
        self.assertEqual(backend.artifact_identity['head_sha256'], self.mul_head_sha)
        self.assertEqual(backend.artifact_identity['boundary'], 'Mul_204:624')
        self.assertEqual(backend.model_sha256, hashlib.sha256(self.path.read_bytes()).hexdigest())
        self.assertNotEqual(backend.model_sha256, old_identity)
        backend.close()

    def test_mixed_boundary_prefix_head_or_feature_shape_rejected_before_hardware(self):
        for boundary, other in [('Conv_205:625', 'Mul_204:624'), ('Mul_204:624', 'Conv_205:625')]:
            for mismatch in ('prefix', 'head', 'shape', 'unknown_boundary'):
                self.select_boundary(boundary)
                other_contract = hybrid.contract_for_boundary(other)
                if mismatch == 'prefix':
                    self.manifest['prefix_onnx_sha256'] = other_contract['prefix_sha256']
                elif mismatch == 'head':
                    self.manifest['head'] = dict(path='head.onnx' if other == 'Conv_205:625' else 'mul_head.onnx',
                                                sha256=other_contract['head_sha256'])
                elif mismatch == 'shape':
                    self.manifest['output']['shape'] = other_contract['output_shape']
                else:
                    self.manifest['boundary'] = 'Mul_204:625'
                with self.subTest(boundary=boundary, mismatch=mismatch), self.assertRaises(ValueError):
                    self.backend()
                self.assertFalse(self.hp.events)
                self.assertFalse(self.ort.sessions)

    def test_mul204_wrong_head_name_or_shape_never_opens_hardware(self):
        for field, value in [('head_input_name', '625'), ('head_input_shape', [1, 17, 8, 6]),
                             ('head_input_shape', [1, 8, 6, 768])]:
            self.select_boundary('Mul_204:624')
            setattr(self.ort, field, value)
            with self.subTest(field=field, value=value), self.assertRaisesRegex(ValueError, 'cpu_head_contract'):
                self.backend()
            self.assertFalse(self.hp.events)

    def test_mul204_wrong_hef_shape_or_partial_output_fails_closed(self):
        self.select_boundary('Mul_204:624')
        self.hp.output_shape = [8, 6, 17]
        with self.assertRaisesRegex(ValueError, 'hef_stream_shape'):
            self.backend()
        self.assertEqual(self.hp.events[-1], 'vdevice_closed')
        self.select_boundary('Mul_204:624')
        backend = self.backend()
        self.hp.features[7, 5, 767] = np.nan
        with self.assertRaisesRegex(ValueError, 'dequantized_prefix_output'):
            backend.infer(np.zeros((1, 3, 256, 192), np.float32))
        self.assertTrue(backend.closed)
        self.assertEqual(backend.completed_inferences, 0)
        self.assertEqual(self.hp.events[-3:], ['shutdown', 'configured_closed', 'vdevice_closed'])

    def test_explicit_float_layout_and_original_cpu_head(self):
        backend = self.backend()
        self.assertEqual(backend.name, 'hailo10h_conv205_onnx_head')
        tensor = (np.arange(3 * 256 * 192).reshape(1, 3, 256, 192) / 10000 - 2).astype(np.float32)
        result = backend.infer(tensor)
        model = self.hp.models[0]
        self.assertEqual(model.batch_size, 1)
        self.assertEqual(self.hp.vdevices[0].device_ids, ['pci/0000:01:00.0'])
        self.assertIn(('identify', 'pci/0000:01:00.0'), self.hp.events)
        np.testing.assert_array_equal(model.configured.bindings.inputs[self.hp.input_name], tensor[0].transpose(1, 2, 0))
        np.testing.assert_array_equal(self.ort.sessions[0].inputs['625'], self.hp.features.transpose(2, 0, 1)[None])
        self.assertEqual(self.ort.sessions[0].options['providers'], ['CPUExecutionProvider'])
        self.assertEqual(self.ort.sessions[0].model_bytes, self.head_data)
        self.assertEqual(self.hp.vdevices[0].hef_bytes, self.hef_data)
        self.assertEqual([x.shape for x in result], [(1, 17, 384), (1, 17, 512)])
        self.assertEqual(backend.completed_inferences, 1)
        for stream in model.streams.values():
            self.assertEqual(stream.format.type, 'float32')
            self.assertEqual(stream.format.order, 'nhwc')
        self.assertIn(('ready', dict(timeout_ms=150, frames_count=1)), self.hp.events)
        self.assertIn(('run', 150), self.hp.events)
        backend.close()
        backend.close()
        self.assertEqual(self.hp.events[-3:], ['shutdown', 'configured_closed', 'vdevice_closed'])
        self.assertEqual(self.hp.events.count('shutdown'), 1)

    def test_pipeline_identity_prevents_reusing_other_artifact_neutral(self):
        backend = self.backend()
        self.assertEqual(backend.model_sha256, hashlib.sha256(self.path.read_bytes()).hexdigest())
        self.assertNotEqual(backend.model_sha256, hybrid.SOURCE_SHA256)
        self.assertEqual(backend.source_model_sha256, hybrid.SOURCE_SHA256)
        self.assertTrue(backend.artifact_identity['partial_acceleration'])
        backend.close()

    def test_missing_tampered_or_different_head_fails_before_hardware(self):
        self.write_manifest()
        for path, data in ((self.root / 'test.hef', b'changed'), (self.root / 'head.onnx', b'changed')):
            original = path.read_bytes()
            path.write_bytes(data)
            with self.assertRaisesRegex(ValueError, 'sha256_mismatch'):
                hybrid.HailoHybrid(self.path, hailo_module=self.hp, ort_module=self.ort)
            path.write_bytes(original)
        self.manifest['head']['sha256'] = 'b' * 64
        with self.assertRaisesRegex(ValueError, 'cpu_head_identity'):
            self.backend()
        (self.root / 'test.hef').unlink()
        with self.assertRaises(FileNotFoundError):
            self.backend()
        self.assertEqual(self.hp.events, [])

    def test_wrong_identity_preprocessing_batch_or_quantization_is_rejected(self):
        original = json.dumps(self.manifest)
        changes = [('target', 'HAILO8'), ('source_model_sha256', '0' * 64),
                   ('preprocessing', 'rgb_uint8'), ('timeout_ms', True), ('timeout_ms', 1001)]
        for key, value in changes:
            self.manifest = json.loads(original)
            self.manifest[key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                self.backend()
        for spec, key, value in [('input', 'shape', [1, 256, 192, 3]), ('output', 'layout', 'NCHW'),
                                 ('output', 'dtype', 'uint8'), ('input', 'quantization', []),
                                 ('output', 'quantization', [dict(qp_scale=0, qp_zp=0)])]:
            self.manifest = json.loads(original)
            self.manifest[spec][key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                self.backend()
        self.assertFalse(self.hp.events)

    def test_duplicate_and_nonfinite_manifest_json_rejected(self):
        self.path.write_text('{"schema": "a", "schema": "b"}')
        with self.assertRaisesRegex(ValueError, 'duplicate_manifest_key'):
            hybrid.load_manifest(self.path)
        self.path.write_text('{"x": NaN}')
        with self.assertRaisesRegex(ValueError, 'nonfinite_manifest'):
            hybrid.load_manifest(self.path)

    def test_missing_device_wrong_arch_and_version_never_configure(self):
        for key, value, reason in [('devices', [], 'device_missing'), ('architecture', 'HAILO8', 'architecture'),
                                    ('__version__', '4.23.0', 'version')]:
            self.hp = FakeHailo()
            setattr(self.hp, key, value)
            with self.subTest(key=key), self.assertRaisesRegex(RuntimeError, reason):
                self.backend()
            self.assertFalse(self.hp.vdevices)

    def test_pci_prefix_matching_preserves_actual_discovered_identifier(self):
        for configured, discovered in [('0000:01:00.0', 'pci/0000:01:00.0'),
                                       ('pci/0000:01:00.0', '0000:01:00.0'),
                                       ('ABCD:0A:1F.7', 'pci/abcd:0a:1f.7')]:
            self.hp = FakeHailo()
            self.hp.devices = ['usb/0000:01:00.0', '192.168.0.145', 'pci/0000:02:00.0', discovered]
            self.manifest['device_id'] = configured
            backend = self.backend()
            self.assertEqual(backend.discovered_device_id, discovered)
            self.assertEqual(self.hp.vdevices[0].device_ids, [discovered])
            self.assertIn(('identify', discovered), self.hp.events)
            backend.close()

    def test_nonlocal_or_ambiguous_device_ids_are_never_opened(self):
        for value in ('192.168.0.145', 'tcp://0000:01:00.0', 'usb/0000:01:00.0',
                      'pci/01:00.0', '0000:01:20.0', 'pci/0000:01:00.8',
                      'pci/0000:01:00.0/extra', 'pci/0000:01:00.0\n', None):
            self.manifest['device_id'] = value
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, 'pcie_device_id'):
                self.backend()
        self.assertFalse(self.hp.events)
        self.manifest['device_id'] = '0000:01:00.0'
        self.hp.devices = ['tcp://0000:01:00.0', 'usb/0000:01:00.0', 'pci/01:00.0']
        with self.assertRaisesRegex(RuntimeError, 'device_missing'):
            self.backend()
        self.assertFalse(self.hp.vdevices)
        self.assertFalse(any(isinstance(e, tuple) and e[0] == 'identify' for e in self.hp.events))
        self.hp.devices = ['pci/0000:01:00.0', '0000:01:00.0']
        with self.assertRaisesRegex(RuntimeError, 'device_ambiguous'):
            self.backend()
        self.assertFalse(self.hp.vdevices)

    def test_cpu_head_metadata_mismatch_never_opens_hardware(self):
        self.ort.bad_meta = True
        with self.assertRaisesRegex(ValueError, 'cpu_head_contract'):
            self.backend()
        self.assertFalse(self.hp.events)

    def test_hef_metadata_failure_releases_device(self):
        for key, value, reason in [('output_name', 'another_node', 'names'),
                                  ('output_shape', [8, 6, 18], 'shape'),
                                  ('quant', [NS(qp_scale=.25, qp_zp=111)], 'quantization'),
                                  ('ignored_format', True, 'host_format')]:
            self.hp = FakeHailo()
            setattr(self.hp, key, value)
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, reason):
                self.backend()
            self.assertEqual(self.hp.events[-1], 'vdevice_closed')

    def test_configure_failure_releases_device(self):
        self.hp.configure_error = RuntimeError('configuration failed')
        with self.assertRaisesRegex(RuntimeError, 'configuration failed'):
            self.backend()
        self.assertEqual(self.hp.events[-1], 'vdevice_closed')

    def test_invalid_input_never_launches_inference(self):
        backend = self.backend()
        for value in (np.zeros((1, 3, 256, 192), np.uint8), np.zeros((3, 256, 192), np.float32),
                      np.full((1, 3, 256, 192), np.nan, np.float32)):
            with self.assertRaisesRegex(ValueError, 'input_tensor'):
                backend.infer(value)
        self.assertFalse(any(isinstance(e, tuple) and e[0] == 'run' for e in self.hp.events))
        backend.close()

    def test_timeout_latches_unavailable_and_never_reuses_inflight_buffer(self):
        backend = self.backend()
        self.hp.run_error = TimeoutError('not completed')
        tensor = np.zeros((1, 3, 256, 192), np.float32)
        with self.assertRaises(TimeoutError):
            backend.infer(tensor)
        self.assertTrue(backend.closed)
        with self.assertRaisesRegex(RuntimeError, 'hybrid_runtime_failed'):
            backend.infer(tensor)
        self.assertEqual(self.hp.events.count(('run', 150)), 1)
        self.assertEqual(backend.completed_inferences, 0)
        self.assertEqual(self.hp.events[-3:], ['shutdown', 'configured_closed', 'vdevice_closed'])

    def test_partial_output_and_bad_cpu_head_result_clear_backend(self):
        for bad in ('prefix', 'head_shape', 'head_nan', 'head_dtype'):
            self.hp, self.ort = FakeHailo(), FakeOrt()
            if bad == 'prefix':
                self.hp.bad_features = True
            else:
                outputs = [np.zeros((1, 17, bins), np.float32) for bins in (384, 512)]
                if bad == 'head_shape':
                    outputs[0] = outputs[0][:, :16]
                if bad == 'head_nan':
                    outputs[0].fill(np.nan)
                if bad == 'head_dtype':
                    outputs[0] = outputs[0].astype(np.uint8)
                self.ort.bad_result = outputs
            backend = self.backend()
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                backend.infer(np.zeros((1, 3, 256, 192), np.float32))
            self.assertTrue(backend.closed)
            self.assertEqual(backend.completed_inferences, 0)

    def test_shutdown_error_still_releases_configured_model_and_device(self):
        backend = self.backend()
        self.hp.shutdown_error = RuntimeError('shutdown failed')
        with self.assertRaisesRegex(RuntimeError, 'shutdown failed'):
            backend.close()
        self.assertEqual(self.hp.events[-3:], ['shutdown', 'configured_closed', 'vdevice_closed'])
        backend.close()

    @unittest.skipUnless(os.environ.get('OASIS_RTMPOSE_PARITY_ROOT') and os.environ.get('OASIS_RTMPOSE_FULL'),
                         'optional pinned ONNX artifacts unavailable; no hardware inference in this test')
    def test_real_cpu_head_parity_through_stub_hailo_layout(self):
        self.check_real_cpu_head_parity('Conv_205:625', 'derived_20261027T133306_261860Z',
                                        'rtmpose_m_conv205_prefix.onnx', 'rtmpose_m_simcc_head.onnx')

    @unittest.skipUnless(os.environ.get('OASIS_RTMPOSE_PARITY_ROOT') and os.environ.get('OASIS_RTMPOSE_FULL'),
                         'optional pinned ONNX artifacts unavailable; no hardware inference in this test')
    def test_real_mul204_cpu_head_parity_through_stub_hailo_layout(self):
        self.check_real_cpu_head_parity('Mul_204:624', 'derived_mul204_20261027T151036_163064Z',
                                        'rtmpose_m_mul204_prefix.onnx', 'rtmpose_m_conv205_simcc_head.onnx')

    def check_real_cpu_head_parity(self, boundary, artifact_dir, prefix_name, head_name):
        import onnxruntime as ort
        root = Path(os.environ['OASIS_RTMPOSE_PARITY_ROOT'])
        artifacts = root / artifact_dir
        full = Path(os.environ['OASIS_RTMPOSE_FULL'])
        prefix, head = artifacts / prefix_name, artifacts / head_name
        contract = REAL_BOUNDARY_CONTRACTS[boundary]
        for path, digest in ((full, hybrid.SOURCE_SHA256), (prefix, contract.prefix_sha256), (head, contract.head_sha256)):
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), digest)
        inputs_path = root / 'parity_inputs.npz'
        self.assertEqual(hashlib.sha256(inputs_path.read_bytes()).hexdigest(),
                         '786ccb5ef251ec4128d5100d0e83547643239aec0c8a5a4df7fb05bfc4487ead')
        options = ort.SessionOptions()
        options.intra_op_num_threads, options.inter_op_num_threads = 2, 1
        reference = ort.InferenceSession(str(full), sess_options=options, providers=['CPUExecutionProvider'])
        prefix_session = ort.InferenceSession(str(prefix), sess_options=options, providers=['CPUExecutionProvider'])
        self.select_boundary(boundary)
        self.manifest['head'] = dict(path=str(head), sha256=contract.head_sha256)
        self.write_manifest()
        with patch.object(hybrid, '_BOUNDARY_CONTRACTS', REAL_BOUNDARY_CONTRACTS):
            backend = hybrid.HailoHybrid(self.path, hailo_module=self.hp, ort_module=ort)
        try:
            with np.load(inputs_path, allow_pickle=False) as archive:
                tensors = archive['input']
            self.assertEqual(len(tensors), 18)
            for index, item in enumerate(tensors):
                tensor = item[None]
                # ONLY the Hailo prefix is stubbed, with its exact original CPU result.
                self.hp.features = prefix_session.run([contract.head_input_name], {'input': tensor})[0][0].transpose(1, 2, 0)
                actual = backend.infer(tensor)
                expected = reference.run(['simcc_x', 'simcc_y'], {'input': tensor})
                for a, b in zip(actual, expected):
                    with self.subTest(input=index):
                        np.testing.assert_allclose(a, b, rtol=1e-5, atol=1e-5)
            self.assertEqual(backend.completed_inferences, 18)
        finally:
            backend.close()


if __name__ == '__main__':
    unittest.main()
