import copy
import importlib.util
from pathlib import Path
import unittest
from types import SimpleNamespace

from oasis_rt.pose_camera import CameraWorker,Frame

spec=importlib.util.spec_from_file_location('collect_pose_reference',Path(__file__).with_name('collect_pose_reference.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


def status(stamp=10_000_000_000):
    return {'calibration_observation':dict(schema='oasis.pose.calibration_observation.v1',capture_ns=stamp,
        generation=1,stream_id='stream1',camera_id='camera',model_sha256='a'*64,
        geometry=dict(frame_hw=[480,640],roi_xyxy=[0,0,640,480],shoulder_center_px=[320.,160.],
                      hip_center_px=[320.,350.],shoulder_width_px=200.,torso_angle_deg=0.,pose_confidence=.8))}


class ReferenceTests(unittest.TestCase):
    def sample(self,stamp=10_000_000_000):
        return m.observation(status(stamp),stamp+100_000_000,'camera','a'*64)

    def test_reference_geometry_and_identity(self):
        samples=[self.sample(10_000_000_000+i*150_000_000) for i in range(25)]
        neutral,g,t=m.summarize(samples)
        self.assertEqual(neutral['camera_id'],'camera')
        self.assertEqual(neutral['collection']['samples'],25)
        self.assertEqual(m.relative_features(g,neutral,t,'a'*64)['lean_score'],0.)

    def test_stale_future_mismatch_refused(self):
        for now,camera,sha in [(10_300_000_000,'camera','a'*64),(9_999_999_999,'camera','a'*64),
                              (10_100_000_000,'other','a'*64),(10_100_000_000,'camera','b'*64)]:
            with self.subTest(now=now,camera=camera,sha=sha),self.assertRaises(ValueError):
                m.observation(status(),now,camera,sha)

    def test_generation_change_and_duplicate_rejected(self):
        samples=[self.sample(10_000_000_000+i*150_000_000) for i in range(25)]
        changed=copy.deepcopy(samples);changed[-1]['generation']=2
        with self.assertRaises(ValueError):m.summarize(changed)
        with self.assertRaises(ValueError):m.summarize(samples[:20]+[samples[-2],samples[-2]])

    def test_movement_rejected(self):
        samples=[]
        for i in range(25):
            raw=status(10_000_000_000+i*150_000_000)
            shift=30 if i%2 else -30
            raw['calibration_observation']['geometry']['shoulder_center_px'][0]+=shift
            raw['calibration_observation']['geometry']['hip_center_px'][0]+=shift
            samples.append(m.observation(raw,raw['calibration_observation']['capture_ns']+1,'camera','a'*64))
        with self.assertRaises(ValueError):m.summarize(samples)

    def test_export_never_enables_pose(self):
        geometry=status()['calibration_observation']['geometry']
        class Capture:
            reason='camera_missing'
            def __init__(self):self.frames=[Frame(object(),9_990_000_000,9_980_000_000,9_985_000_000)]
            def poll(self):return self.frames.pop(0) if self.frames else None
        class Publisher:
            def __init__(self):self.packets=[]
            def send(self,p):self.packets.append(p)
        def infer(frame,backend,*_):
            backend.last_body_geometry=geometry
            raise ValueError('neutral_required')
        publisher=Publisher();backend=SimpleNamespace(model_sha256='a'*64)
        worker=CameraWorker(Capture(),backend,publisher,source_id='source',camera_id='camera',
                            feature_fn=infer,clock_ns=lambda:10_000_000_000)
        worker.step()
        self.assertEqual(worker.summary()['calibration_observation']['capture_ns'],9_990_000_000)
        self.assertFalse(publisher.packets[-1]['vision_valid'])
        worker.step()
        self.assertIsNone(worker.summary()['calibration_observation'])


if __name__=='__main__':unittest.main()
