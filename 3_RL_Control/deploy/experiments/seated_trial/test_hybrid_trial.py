"""Candidate/runtime qualification only. No hardware, sockets, or ARM execution."""
import copy
import os
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
import yaml

import occupied_trial as T
from prepare_candidate import candidate


class HybridCandidateTests(unittest.TestCase):
    def source(self):
        return yaml.safe_load(Path(__file__).with_name('candidate.yaml').read_text(encoding='utf-8'))

    def test_saved_candidate_all_confirmations_and_approvals_false(self):
        cfg=self.source()
        T.validate_candidate(cfg,require_approval=False)
        self.assertTrue(all(v is False for v in cfg['confirmations'].values()))
        for field in ('occupied_trial_accepted','operator_stop_ready','physical_motion_authorized',
                      'hardware_arm_performed','rl_after_new_neutral_authorized'):
            self.assertIs(cfg['deployment'][field],False)
        self.assertIsNone(cfg['deployment']['approved_trial'])
        self.assertIsNone(cfg['deployment']['approval_recorded_wall_time_ns'])
        with self.assertRaises(ValueError):T.validate_candidate(cfg)

    def test_preparation_clears_old_run_approval_without_modifying_source(self):
        source=self.source()
        source['confirmations']={k:True for k in T.CONFIRM}
        source['deployment'].update(approved_trial='occupied_yaw20_pitch0_rate10_300s_raw2500_neutral18181212_settle2_v1',
            occupied_trial_accepted=True,operator_stop_ready=True,physical_motion_authorized=True,
            hardware_arm_performed=True,rl_after_new_neutral_authorized=True)
        before=copy.deepcopy(source)
        out=candidate(source,2)
        self.assertEqual(source,before)
        self.assertIsNone(out['deployment']['approved_trial'])
        self.assertTrue(all(v is False for v in out['confirmations'].values()))
        self.assertFalse(out['deployment']['physical_motion_authorized'])

    def test_exact_enabled_hybrid_settings_and_filter_required(self):
        for field, value in (('enabled',False),('enabled',1),('full_g',.8),
                             ('full_yaw_deg',19),('preview_max_yaw_deg',11),
                             ('residual_max_deg',2),('full_g',True),('deadband_g',float('nan'))):
            cfg=self.source();cfg['g_residual_control'][field]=value
            with self.subTest(field=field,value=value):
                with self.assertRaises(ValueError):T.validate_candidate(cfg,require_approval=False)
        for field,value in (('dt_s',.1),('filter_tau_s',0)):
            cfg=self.source();cfg[field]=value
            with self.assertRaises(ValueError):T.validate_candidate(cfg,require_approval=False)
        cfg=self.source();del cfg['g_residual_control']
        with self.assertRaises(ValueError):T.validate_candidate(cfg,require_approval=False)

    def test_old_approval_tag_cannot_authorize_new_hybrid(self):
        cfg=self.source();cfg['confirmations']={k:True for k in T.CONFIRM}
        cfg['deployment'].update(occupied_trial_accepted=True,operator_stop_ready=True,
            approved_trial='occupied_yaw20_pitch0_rate10_300s_raw2500_neutral18181212_settle2_v1')
        with self.assertRaises(ValueError):T.validate_candidate(cfg)
        cfg['deployment']['approved_trial']=T.APPROVAL
        T.validate_candidate(cfg)

    def test_exact_revision_and_trial_identity_required(self):
        for field,value in (('expected_tuning_revision',0),('expected_tuning_revision',3),
                            ('arm_time_budget_s',301),('policy','rl')):
            cfg=self.source();cfg['deployment'][field]=value
            with self.assertRaises(ValueError):T.validate_candidate(cfg,require_approval=False)

    def test_old_runtime_ignoring_hybrid_config_is_rejected(self):
        with self.assertRaisesRegex(RuntimeError,'governor missing'):
            T.validate_hybrid_policy(SimpleNamespace())

    @unittest.skipUnless(os.environ.get('OASIS_TRIAL_TEST_RUNTIME'),'Explicit local runtime required')
    def test_actual_governor_contract_and_training_prior_required(self):
        sys.path.insert(0,os.environ['OASIS_TRIAL_TEST_RUNTIME'])
        from oasis_rt.g_residual_control import GResidualGovernor
        ctl=SimpleNamespace(g_governor=GResidualGovernor(),g_filter=object(),
                            cfg={'_training_prior':lambda x:x})
        T.validate_hybrid_policy(ctl)
        ctl.g_governor=GResidualGovernor({'residual_max_deg':1.})
        with self.assertRaisesRegex(RuntimeError,'defaults/signed filter mismatch'):
            T.validate_hybrid_policy(ctl)
        ctl.g_governor=GResidualGovernor();ctl.cfg={}
        with self.assertRaisesRegex(RuntimeError,'training prior missing'):
            T.validate_hybrid_policy(ctl)


if __name__=='__main__':unittest.main()
