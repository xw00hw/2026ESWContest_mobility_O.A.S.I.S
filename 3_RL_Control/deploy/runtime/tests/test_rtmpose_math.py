"""Geometry, model contract, and optional RTMLib golden parity; no live devices."""
from dataclasses import replace
import importlib.util
import math
import os
from pathlib import Path
import sys
import unittest

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from oasis_rt import rtmpose_math as M

HASH = 'a'*64
HAVE_CV = importlib.util.find_spec('cv2') is not None
HAVE_RTM = importlib.util.find_spec('rtmlib') is not None


def body(shift_shoulder=0, shift_hip=0):
    xy, scores = np.zeros((17, 2)), np.zeros(17)
    xy[5], xy[6] = [220+shift_shoulder, 160], [420+shift_shoulder, 160]
    xy[11], xy[12] = [240+shift_hip, 360], [400+shift_hip, 360]
    scores[list(M.CORE)] = .9
    return xy, scores


class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.t = M.image_transform((480, 640))
        self.g = M.extract_body_geometry(*body(), self.t)
        self.neutral = M.make_neutral(self.g, self.t, HASH, operator_confirmed=True)

    def test_neutral_zero_and_image_right_positive(self):
        zero = M.relative_features(self.g, self.neutral, self.t, HASH)
        self.assertEqual((zero['torso_lean_deg'], zero['lean_score']), (0, 0))
        moved = M.extract_body_geometry(*body(20), self.t)
        f = M.relative_features(moved, self.neutral, self.t, HASH)
        self.assertAlmostEqual(f['torso_lean_deg'], math.degrees(math.atan2(20, 200)))
        self.assertAlmostEqual(f['lean_score'], .1)
        self.assertTrue(f['vision_valid'] and f['neutral_valid'])

    def test_whole_body_translation_preserves_angle_but_changes_shoulder_shift(self):
        moved = M.extract_body_geometry(*body(20, 20), self.t)
        f = M.relative_features(moved, self.neutral, self.t, HASH)
        self.assertEqual(f['torso_lean_deg'], 0)
        self.assertAlmostEqual(f['lean_score'], .1)  # not a new hip-relative metric

    def test_angle_is_relative_to_nonzero_neutral_and_uses_pixel_aspect(self):
        tilted = M.extract_body_geometry(*body(20), self.t)
        neutral = M.make_neutral(tilted, self.t, HASH, operator_confirmed=True)
        f = M.relative_features(self.g, neutral, self.t, HASH)
        self.assertAlmostEqual(f['torso_lean_deg'], -math.degrees(math.atan2(20, 200)))
        self.assertAlmostEqual(f['lean_score'], -.1)

    def test_scale_invariant_with_separate_matching_reference(self):
        t2 = M.image_transform((960, 1280))
        xy, scores = body()
        g2 = M.extract_body_geometry(xy*2, scores, t2)
        n2 = M.make_neutral(g2, t2, HASH, operator_confirmed=True)
        xy, scores = body(20)
        moved2 = M.extract_body_geometry(xy*2, scores, t2)
        f = M.relative_features(moved2, n2, t2, HASH)
        self.assertAlmostEqual(f['lean_score'], .1)
        self.assertAlmostEqual(f['torso_lean_deg'], math.degrees(math.atan2(20, 200)))

    def test_unconfirmed_missing_or_wrong_identity_neutral_is_rejected(self):
        with self.assertRaises(ValueError): M.make_neutral(self.g, self.t, HASH)
        with self.assertRaises(ValueError): M.relative_features(self.g, None, self.t, HASH)
        for key, value in [('model_sha256', 'b'*64), ('frame_hw', [481,640]),
                           ('roi_xyxy', [0,1,640,480]), ('operator_confirmed', False)]:
            with self.subTest(key=key), self.assertRaises(ValueError):
                M.relative_features(self.g, dict(self.neutral, **{key:value}), self.t, HASH)

    def test_malformed_neutral_is_rejected(self):
        for key, value in [('shoulder_width_px', 0), ('shoulder_width_px', float('nan')),
                           ('shoulder_center_px', [float('inf'),100]), ('shoulder_center_px', [-1,100]),
                           ('torso_angle_deg', 50), ('hip_center_px', [320,160])]:
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                M.relative_features(self.g, dict(self.neutral, **{key:value}), self.t, HASH)

    def test_missing_nonfinite_outside_or_degenerate_core_is_rejected(self):
        for index, value in [(5, float('nan')), (11, float('inf')), (12, 640)]:
            xy, scores = body(); xy[index,0] = value
            with self.subTest(index=index, value=value), self.assertRaises(ValueError):
                M.extract_body_geometry(xy, scores, self.t)
        for index in M.CORE:
            xy, scores = body(); scores[index] = .49
            with self.subTest(index=index), self.assertRaises(ValueError):
                M.extract_body_geometry(xy, scores, self.t)
        xy, scores = body(); xy[6] = xy[5]
        with self.assertRaises(ValueError): M.extract_body_geometry(xy, scores, self.t)
        xy, scores = body(); xy[[11,12],1] = 160
        with self.assertRaises(ValueError): M.extract_body_geometry(xy, scores, self.t)

    def test_noncore_landmarks_do_not_fake_or_invalidate_observable_torso(self):
        xy, scores = body(); xy[16] = [float('nan'), float('nan')]; scores[16] = 0
        self.assertEqual(M.extract_body_geometry(xy, scores, self.t), self.g)
        scores[list(M.CORE)] = 0
        with self.assertRaises(ValueError): M.extract_body_geometry(xy, scores, self.t)

    def test_shape_roi_and_transform_validation(self):
        with self.assertRaises(ValueError): M.extract_body_geometry(np.zeros((16,2)), np.ones(17), self.t)
        for roi in [(0,0,641,480), (0,0,0,480), (0,0,1.5,480), (4,4,2,2)]:
            with self.subTest(roi=roi), self.assertRaises(ValueError): M.image_transform((480,640),roi)
        with self.assertRaises(ValueError):
            M.extract_body_geometry(*body(), replace(self.t, center=(0,0)))
        with self.assertRaises(ValueError):
            M.make_neutral(replace(self.g, pose_confidence=float('nan')), self.t, HASH, operator_confirmed=True)

    def test_simcc_peak_decodes_to_affine_pixel_location_and_validates_outputs(self):
        x, y = np.zeros((1,17,384), np.float32), np.zeros((1,17,512), np.float32)
        x[:,:,192], y[:,:,256] = .6, .8
        xy, scores = M.decode_simcc(x,y,self.t)
        np.testing.assert_allclose(xy, np.tile(self.t.center,(17,1)))
        np.testing.assert_allclose(scores,.7)
        for bad in (x[:, :, :383], np.full_like(x, np.nan), np.full_like(x, np.inf)):
            with self.assertRaises(ValueError): M.decode_simcc(bad,y,self.t)
        with self.assertRaises(ValueError):
            M.decode_simcc(np.full_like(x,np.finfo(np.float32).max),
                           np.full_like(y,np.finfo(np.float32).max),self.t)


@unittest.skipUnless(HAVE_CV, 'OpenCV is optional until camera processing is installed')
class ImageTests(unittest.TestCase):
    def test_input_layout_channel_order_and_invalid_images(self):
        image = np.empty((256,192,3),np.uint8); image[:] = [10,20,30]
        tensor,t = M.prepare_input(image)
        self.assertEqual(tensor.shape,(1,3,256,192)); self.assertEqual(tensor.dtype,np.float32)
        np.testing.assert_allclose(tensor[0,:,128,96],(np.array([10,20,30])-M.MEAN)/M.STD,rtol=1e-6)
        for bad in (np.zeros((20,30),np.uint8),np.zeros((20,30,3)),np.zeros((20,30,4),np.uint8)):
            with self.assertRaises(ValueError): M.prepare_input(bad)

    @unittest.skipUnless(HAVE_RTM, 'RTMLib reference package required for golden parity')
    def test_rtmlib_pre_and_postprocessing_parity(self):
        from rtmlib import RTMPose
        model = RTMPose.__new__(RTMPose)
        model.model_input_size, model.mean, model.std = M.INPUT_WH, tuple(M.MEAN), tuple(M.STD)
        image = np.random.default_rng(26).integers(0,256,(480,640,3),dtype=np.uint8)
        roi = (70,30,590,470)
        actual,t = M.prepare_input(image,roi)
        crop = image[30:470,70:590]
        normalized,center,scale = model.preprocess(crop,[0,0,crop.shape[1],crop.shape[0]])
        expected = np.ascontiguousarray(normalized.transpose(2,0,1)[None],dtype=np.float32)
        np.testing.assert_array_equal(actual,expected)
        rng=np.random.default_rng(7)
        outputs=[rng.random((1,17,384),dtype=np.float32),rng.random((1,17,512),dtype=np.float32)]
        want_xy,want_score=model.postprocess(outputs,center,scale)
        xy,score=M.decode_simcc(*outputs,t)
        np.testing.assert_allclose(xy,want_xy[0]+[70,30],atol=1e-5,rtol=0)
        np.testing.assert_array_equal(score,want_score[0])

    @unittest.skipUnless(HAVE_RTM and os.environ.get('OASIS_RTMPOSE_PARITY_FRAME') and os.environ.get('OASIS_RTMPOSE_MODEL'),
                         'optional local real-frame + exported-model parity fixture')
    def test_real_frame_model_parity(self):
        import cv2
        import onnxruntime as ort
        from rtmlib import RTMPose
        image=cv2.imread(os.environ['OASIS_RTMPOSE_PARITY_FRAME'])
        self.assertIsNotNone(image)
        model=RTMPose.__new__(RTMPose)
        model.model_input_size,model.mean,model.std=M.INPUT_WH,tuple(M.MEAN),tuple(M.STD)
        actual,t=M.prepare_input(image)
        normal,center,scale=model.preprocess(image,[0,0,image.shape[1],image.shape[0]])
        expected=np.ascontiguousarray(normal.transpose(2,0,1)[None],dtype=np.float32)
        np.testing.assert_array_equal(actual,expected)
        options=ort.SessionOptions(); options.intra_op_num_threads=2; options.inter_op_num_threads=1
        session=ort.InferenceSession(os.environ['OASIS_RTMPOSE_MODEL'],sess_options=options,providers=['CPUExecutionProvider'])
        outputs=session.run(['simcc_x','simcc_y'],{'input':actual})
        xy,score=M.decode_simcc(*outputs,t)
        want_xy,want_score=model.postprocess(outputs,center,scale)
        np.testing.assert_allclose(xy,want_xy[0],atol=1e-5,rtol=0)
        np.testing.assert_array_equal(score,want_score[0])


if __name__=='__main__': unittest.main()
