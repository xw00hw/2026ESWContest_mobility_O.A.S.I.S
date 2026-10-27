"""CPU/stub tests only; no accelerator, camera, socket or motor access."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
import validate_hybrid_hardware as h


def outputs(x_offset=0, score=0.8):
    x = np.zeros((1,17,384), np.float32)
    y = np.zeros((1,17,512), np.float32)
    x[:,:,100+x_offset], y[:,:,200] = score, score
    return [x,y]


class Head:
    def __init__(self,name='625'):
        self.name=name
    def run(self, names, feed):
        self.last_feed = feed[self.name].copy()
        return outputs()


class Backend:
    def __init__(self, fail_at=None, boundary='Conv_205:625'):
        self.manifest={'boundary':boundary}
        self.contract=h.runtime.contract_for_boundary(boundary)
        self.session, self.closed, self.calls, self.fail_at = Head(self.contract['head_input_name']),False,0,fail_at
    def infer(self, tensor):
        assert tensor.shape == (1,3,256,192) and tensor.dtype == np.float32 and tensor.flags.c_contiguous
        self.calls += 1
        if self.calls == self.fail_at:
            raise RuntimeError('stub_inference_failure')
        shape=self.contract['head_input_shape']
        features = np.arange(np.prod(shape),dtype=np.float32).reshape(shape)
        return self.session.run(['simcc_x','simcc_y'], {self.contract['head_input_name']:features})
    def close(self):
        self.closed = True


class HardwareHarnessTests(unittest.TestCase):
    def test_simcc_half_pixel_resolution_and_confidence_loss(self):
        reference, actual = outputs(), outputs(x_offset=3)
        actual[0][0,11,103] = actual[1][0,11,200] = 0.4
        row = h.compare_simcc(reference,actual)
        self.assertEqual(row['confident_core_axis_max_shifts_input_pixels'], [1.5]*4)
        self.assertEqual(row['core_joint_indices'],[5,6,11,12])
        self.assertEqual(row['reference_core_xy_input_pixels'],[[50.0,100.0]]*4)
        self.assertEqual(row['actual_core_xy_input_pixels'],[[51.5,100.0]]*4)
        self.assertEqual(row['reference_confident_core_count'],4)
        self.assertEqual(row['retained_confident_core_count'],3)
        self.assertFalse(h.summarize_comparisons([row])['all_reference_confident_core_joints_retained'])

    def test_wrong_shape_and_nonfinite_are_rejected(self):
        bad=outputs();bad[0][0,0,0]=np.nan
        with self.assertRaises(ValueError):h.compare_simcc(outputs(),bad)
        with self.assertRaises(ValueError):h.tensor_metrics(np.zeros((2,)),np.zeros((3,)))

    def test_capture_exact_nchw_to_nhwc_without_mutation(self):
        head=Head();capture=h.CaptureHead(head)
        x=np.arange(17*8*6,dtype=np.float32).reshape(1,17,8,6)
        before=x.copy()
        capture.run(['simcc_x','simcc_y'],{'625':x})
        self.assertTrue(np.array_equal(x,before))
        self.assertEqual(capture.prefix_nhwc.shape,(1,8,6,17))
        self.assertEqual(capture.prefix_nhwc[0,2,3,4],x[0,4,2,3])
        self.assertTrue(capture.prefix_nhwc.flags.c_contiguous)

    def test_repeated_acquisition_closes_backend_and_preserves_indices(self):
        backend=Backend();state={}
        inputs=np.zeros((2,256,192,3),np.float32)
        rows=h.acquire(lambda:backend,inputs,warmups=1,repeats=2,state=state)
        self.assertEqual([(r['repeat'],r['index']) for r in rows],[(0,0),(0,1),(1,0),(1,1)])
        self.assertEqual(state['completed_inferences'],5)
        self.assertTrue(backend.closed and state['backend_closed'])
        self.assertEqual(rows[0]['prefix'].shape,(1,8,6,17))

    def test_runtime_failure_closes_and_does_not_claim_zero_execution(self):
        backend=Backend(fail_at=2);state={}
        with self.assertRaisesRegex(RuntimeError,'stub_inference_failure'):
            h.acquire(lambda:backend,np.zeros((2,256,192,3),np.float32),warmups=0,state=state)
        self.assertTrue(backend.closed and state['hardware_inference_attempted'])
        self.assertEqual(state['completed_inferences'],1)

    def test_earlier_boundary_acquisition_has_768_features(self):
        backend=Backend(boundary='Mul_204:624')
        rows=h.acquire(lambda:backend,np.zeros((1,256,192,3),np.float32),warmups=0)
        self.assertEqual(rows[0]['prefix'].shape,(1,8,6,768))
        self.assertEqual(backend.session.prefix_nhwc[0,2,3,700],700*48+2*6+3)
        self.assertTrue(backend.closed)

    def test_failed_qualification_blocks_before_loading_arrays(self):
        qa=dict(schema='oasis.rtmpose.research-qualification.v1',target='HAILO10H',
                hef_sha256='0'*64,source_model_sha256=h.runtime.SOURCE_SHA256,
                prefix_onnx_sha256=h.runtime.PREFIX_SHA256,head_onnx_sha256=h.runtime.HEAD_SHA256,
                boundary='Conv_205:625',host_io_dtype='float32',input_shape_nhwc=[1,256,192,3],output_shape_nhwc=[1,8,6,17],
                preprocessing='BGR affine1.25 normalized(mean123.675,116.28,103.53;std58.395,57.12,57.375)',
                model_script_normalization='none; host normalization already applied',float_precheck_pass=True,research_quality_pass=False)
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'failed_research_qualification.json';path.write_text(json.dumps(qa))
            with patch.object(h.runtime,'load_manifest',return_value=({'hef':{'sha256':'0'*64},'boundary':'Conv_205:625'},'id',b'',b'')):
                with self.assertRaisesRegex(ValueError,'research_quality_pass'):
                    h.preflight(Path(d)/'nonexistent_manifest.json',path)

    def test_default_cli_preflight_does_not_invoke_device_path(self):
        bundle=dict(manifest_sha256='m',qualification_sha256='q',corpus_sha256={},
                    manifest={'hef':{'sha256':'h'},'device_id':'0000:01:00.0','boundary':'Conv_205:625'},
                    contract=h.runtime.contract_for_boundary('Conv_205:625'))
        with tempfile.TemporaryDirectory() as d,patch.object(h,'preflight',return_value=bundle),patch.object(h,'run_validation') as run:
            path=Path(d)/'report.json'
            code=h.main(['--manifest','not_opened','--qualification','not_opened','--output',str(path)])
            self.assertEqual(code,0);run.assert_not_called()
            report=json.loads(path.read_text())
            self.assertFalse(report['hardware_inference_executed'])
            self.assertFalse(report['deployment_approved'])


if __name__=='__main__':unittest.main()
