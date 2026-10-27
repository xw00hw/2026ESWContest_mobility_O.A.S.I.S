"""Clock-boundary and observability regressions; no connected hardware needed."""
import unittest

import numpy as np
import pandas as pd

from organize_bench import annotate
from pose_core import estimate, features


class ProcessingTest(unittest.TestCase):
    def test_dwell_is_half_open_and_transition_is_unlabelled(self):
        base = 2_802_217_896_525_400
        points = pd.DataFrame(dict(point_index=[1,2], dwell_start_ns=[base+100,base+400],
                                   dwell_end_ns=[base+300,base+600]))
        query = np.array([-10,100,299,300,399,400,599,600])+base
        np.testing.assert_array_equal(annotate(query, points), [-1,1,1,-1,-1,2,2,-1])

    def test_crop_maps_back_to_source_and_unseen_legs_are_invalid(self):
        class Stub:
            def __call__(self, image):
                assert image.shape == (600,620,3)
                xy = np.full((1,17,2), [310,300], dtype=np.float32)
                xy[0,0] = [310,100]
                xy[0,5] = [180,250]
                xy[0,6] = [440,250]
                return xy,np.ones((1,17),dtype=np.float32)
        xy,score,valid,target = estimate(Stub(),np.zeros((720,1280,3),np.uint8))
        np.testing.assert_array_equal(xy[0],[610,220])
        self.assertTrue(target)
        self.assertTrue(valid[:13].all())
        self.assertFalse(valid[13:].any())

    def test_background_head_cannot_validate_a_body_without_shoulders(self):
        class Stub:
            def __call__(self, image):
                xy=np.full((1,17,2), [310,250],dtype=np.float32)
                score=np.ones((1,17),dtype=np.float32)
                score[0,5]=.1
                return xy,score
        _,_,valid,target=estimate(Stub(),np.zeros((720,1280,3),np.uint8))
        self.assertFalse(target)
        self.assertFalse(valid.any())

    def test_invalid_features_remain_missing_and_sign_is_image_right(self):
        xy=np.zeros((17,2),np.float32)
        valid=np.zeros(17,bool)
        self.assertTrue(all(np.isnan(v) for v in features(xy,valid).values()))
        xy[5]=[100,100];xy[6]=[300,100]
        xy[11]=[50,300];xy[12]=[250,300]
        xy[0]=[225,30];valid[[0,5,6,11,12]]=True
        result=features(xy,valid)
        self.assertAlmostEqual(result['shoulder_tilt_deg'],0)
        self.assertGreater(result['torso_lean_deg'],0)
        self.assertAlmostEqual(result['head_offset_shoulder_ratio'],.125)

    def test_asof_uses_host_capture_clock_not_video_display_time(self):
        base=2_802_217_896_525_400
        # Repeated video frames can share exactly one source capture timestamp.
        video=pd.DataFrame(dict(video_frame_index=[0,1,2],host_ns=[base+60,base+60,base+95]))
        pose=pd.DataFrame(dict(pose_host_ns=[base+60,base+160],pose_id=[7,8]))
        aligned=pd.merge_asof(video,pose,left_on='host_ns',right_on='pose_host_ns',
                               direction='nearest',tolerance=40)
        self.assertEqual(aligned.pose_id.tolist(),[7,7,7])
        self.assertEqual(aligned.pose_host_ns.iloc[0],aligned.pose_host_ns.iloc[1])


if __name__=='__main__':
    unittest.main()
