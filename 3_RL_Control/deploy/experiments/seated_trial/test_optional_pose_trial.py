"""Offline optional-camera qualification; injected datagrams, no camera/socket/UART."""
import copy
import hashlib
import json
import os
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import yaml
import occupied_trial as T
from prepare_candidate import candidate


def config():
    return yaml.safe_load(Path(__file__).with_name('candidate.yaml').read_text(encoding='utf-8'))


class OptionalPoseCandidateTests(unittest.TestCase):
    def test_exact_udp_and_disabled_both_allowed_without_approval(self):
        cfg=config()
        self.assertEqual(cfg['pose'],T.OPTIONAL_POSE_CONFIG)
        T.validate_candidate(cfg,require_approval=False)
        cfg['pose']['source']='none'
        T.validate_candidate(cfg,require_approval=False)
        with self.assertRaises(ValueError):T.validate_candidate(cfg)

    def test_other_source_provider_and_changed_gates_rejected(self):
        cases=(('source','camera'),('source','replay'),('source_id','other_camera'),
               ('min_confidence',.49),('min_confidence',True),('min_confidence',float('nan')),
               ('stale_s',1.1),('stale_s',.3),('lean_sign',0),('lean_sign',2),('lean_sign',float('nan')),('lean_sign',True),
               ('udp_port',47803),('udp_port',47802.),('bind_host','0.0.0.0'))
        for key,value in cases:
            cfg=config();cfg['pose'][key]=value
            with self.subTest(key=key,value=value):
                with self.assertRaises(ValueError):T.validate_candidate(cfg,require_approval=False)
        for source in ('none','udp'):
            cfg=config();cfg['pose']['source']=source;cfg['profile']['from_camera']=True
            with self.assertRaises(ValueError):T.validate_candidate(cfg,require_approval=False)

    def test_either_measured_sign_allowed_without_claiming_alignment(self):
        for sign in (-1,1,-1.,1.):
            cfg=config();cfg['pose']['lean_sign']=sign
            T.validate_candidate(cfg,require_approval=False)
            self.assertIs(cfg['deployment']['camera_pose_physical_alignment_verified'],False)

    def test_candidate_clears_camera_calibration_and_prior_run_claims(self):
        src=config();src['deployment'].update(camera_pose_neutral_available=True,
            camera_pose_physical_alignment_verified=True,physical_motion_authorized=True,
            occupied_trial_accepted=True,operator_stop_ready=True,approved_trial=T.APPROVAL)
        original=copy.deepcopy(src)
        out=candidate(src,2)
        self.assertEqual(src,original)
        self.assertEqual(out['pose'],T.OPTIONAL_POSE_CONFIG)
        self.assertFalse(out['profile']['from_camera'])
        self.assertTrue(out['deployment']['camera_input_optional'])
        for name in ('camera_pose_neutral_available','camera_pose_physical_alignment_verified',
                     'physical_motion_authorized','occupied_trial_accepted','operator_stop_ready'):
            self.assertIs(out['deployment'][name],False)
        self.assertIsNone(out['deployment']['approved_trial'])

    def test_prior_no_camera_approval_cannot_authorize_changed_input(self):
        cfg=config();cfg['confirmations']={key:True for key in T.CONFIRM}
        cfg['deployment'].update(occupied_trial_accepted=True,operator_stop_ready=True,
            approved_trial='occupied_g_hybrid1g20_preview10_res1p5_pitch0_rate10_300s_raw2500_neutral18181212_v1')
        with self.assertRaises(ValueError):T.validate_candidate(cfg)
        cfg['deployment']['approved_trial']=T.APPROVAL
        T.validate_candidate(cfg)

    def test_raw_guard_byte_identical_and_threshold_not_relaxed(self):
        guard=Path(__file__).with_name('raw_count_guard.py').read_bytes()
        self.assertEqual(hashlib.sha256(guard).hexdigest(),
            '6a65e2193b9fb8b70e089c44d23d5b7fd0d4a0abbbda2ccfec90582746667ac3')
        for section,key,value in (('safety','pad_saturation_counts',2501),
                                  ('deployment','raw_release_counts',2501),
                                  ('deployment','raw_release_latched',False)):
            cfg=config();cfg[section][key]=value
            with self.assertRaises(ValueError):T.validate_candidate(cfg,require_approval=False)


class Transport:
    def __init__(self):self.items=[];self.closed=False
    def recvfrom(self,_):
        if not self.items:raise BlockingIOError
        return self.items.pop(0)
    def close(self):self.closed=True
    def add(self,packet,host='127.0.0.1'):
        self.items.append((json.dumps(packet).encode(),(host,47802)))


class Journal:
    def __init__(self):self.events=[]
    def record(self,event,**fields):self.events.append((event,fields))


def packet(now,**changes):
    value=dict(schema='oasis.pose.v1',source_id='front_seat_pose',stream_id='offline-test',
        seq=1,capture_clock='monotonic',capture_ns=now-10_000_000,
        torso_lean_deg=4.,lean_score=.1,pose_confidence=.9,vision_valid=True,neutral_valid=True)
    value.update(changes)
    return value


@unittest.skipUnless(os.environ.get('OASIS_TRIAL_TEST_RUNTIME'),'Explicit reviewed local runtime required')
class OptionalPoseReceiverTests(unittest.TestCase):
    def setUp(self):
        sys.path.insert(0,os.environ['OASIS_TRIAL_TEST_RUNTIME'])
        from oasis_rt import pose_input
        self.api=pose_input;self.now=10_000_000_000
        self.transport=Transport();self.journal=Journal()
        # Opening a real socket is a test failure even if injection regresses.
        self.socket_patch=patch.object(pose_input.socket,'socket',side_effect=AssertionError('No live socket permitted'))
        self.socket_patch.start();self.addCleanup(self.socket_patch.stop)
        self.receiver=pose_input.make_pose_input(config(),lambda:self.now,self.journal,
            allow_udp=False,transport=self.transport,clocks={'monotonic':lambda:self.now})
        self.addCleanup(self.receiver.close)

    def test_valid_pose_accepted_and_capture_age_preserved(self):
        self.transport.add(packet(self.now))
        pose,age=self.receiver.latest()
        self.assertEqual(pose['torso_lean_deg'],4.)
        self.assertAlmostEqual(age,.01)
        self.now+=500_000_000
        self.assertAlmostEqual(self.receiver.latest()[1],.51)
        self.assertEqual(self.receiver.summary()['listen_host'],'127.0.0.1')

    def test_missing_no_neutral_or_low_confidence_is_unavailable_without_exception(self):
        self.assertEqual(self.receiver.latest(),(None,1e9))
        for changes,reason in (({'neutral_valid':False},'invalid_vision_or_neutral'),
                               ({'vision_valid':False},'invalid_vision_or_neutral'),
                               ({'pose_confidence':.499},'low_confidence'),
                               ({'source_id':'wrong_camera'},'source_mismatch')):
            self.transport.add(packet(self.now,**changes))
            self.assertEqual(self.receiver.latest(),(None,1e9))
            self.assertEqual(self.receiver.last_reason,reason)

    def test_timestamp_and_loopback_gates_unchanged(self):
        for changes,reason in (({'capture_ns':self.now-1_000_000_000},'stale_capture'),
                               ({'capture_ns':self.now+1},'future_capture')):
            self.transport.add(packet(self.now,**changes))
            self.assertEqual(self.receiver.latest(),(None,1e9))
            self.assertEqual(self.receiver.last_reason,reason)
        self.transport.add(packet(self.now),host='192.168.0.20')
        self.assertEqual(self.receiver.latest(),(None,1e9))
        self.assertEqual(self.receiver.last_reason,'non_loopback_sender')
        self.transport.add(packet(self.now));self.assertIsNotNone(self.receiver.latest()[0])
        self.transport.add(packet(self.now))
        self.assertEqual(self.receiver.latest(),(None,1e9))
        self.assertEqual(self.receiver.last_reason,'sequence_not_increasing')

    def test_producer_disappearance_expires_using_capture_time(self):
        self.transport.add(packet(self.now));self.assertIsNotNone(self.receiver.latest()[0])
        self.now+=990_000_000
        self.assertEqual(self.receiver.latest(),(None,1e9))
        self.assertEqual(self.receiver.last_reason,'stale_capture')
        cfg=config();cfg['pose']['source']='none'
        disabled=self.api.make_pose_input(cfg,lambda:self.now,self.journal,allow_udp=False)
        self.assertEqual(disabled.latest(),(None,1e9))

    @unittest.skipUnless(os.environ.get('OASIS_TEST_MODEL_DIR'),'Actual local v8 model required')
    def test_actual_hybrid_controller_keeps_running_without_valid_camera(self):
        runtime=Path(os.environ['OASIS_TRIAL_TEST_RUNTIME'])
        sys.path.insert(0,str(runtime/'tests'))
        import test_g_control_integration as helper
        helper.TestGControlIntegration.setUpClass()
        case=helper.TestGControlIntegration()
        ctl,link,vehicle,clock=case.build()
        ctl.cfg['pose']=copy.deepcopy(T.OPTIONAL_POSE_CONFIG)
        ctl.pose=self.receiver
        # Missing producer, invalid neutral, then producer disappearance: no abort.
        for changes in (None,{'neutral_valid':False},{'vision_valid':False}):
            if changes:self.transport.add(packet(self.now,**changes))
            case.tick(ctl,clock)
            self.assertEqual(ctl.state,'RUN')
            self.assertIsNone(link.fault)
            self.assertEqual(ctl.g_snapshot['requested_yaw_lr_deg'],[0.,20.])
            self.assertEqual(len(ctl.row[1]),57)
        self.assertGreater(link.authorizations,0)


if __name__=='__main__':unittest.main()
