"""Wrapper/config contract tests only; runtime variance/freshness tests live upstream."""
import copy
import hashlib
from pathlib import Path
import unittest
import yaml

import occupied_trial as T
from prepare_candidate import candidate


def config():
    return yaml.safe_load(Path(__file__).with_name('candidate.yaml').read_text(encoding='utf-8'))


class BaselineWarnTrialTests(unittest.TestCase):
    def test_exact_warn_mode_and_unchanged_diagnostic_window_required(self):
        cfg=config()
        T.validate_candidate(cfg,require_approval=False)
        self.assertEqual(cfg['pressure']['zero_max_std_counts'],40)
        self.assertEqual(cfg['pressure']['zero_window_s'],2.)
        self.assertEqual(cfg['pressure']['baseline_variability_mode'],'warn')
        self.assertEqual(cfg['deployment']['baseline_variability_mode'],'warn')
        for section,key,value in (('pressure','baseline_variability_mode',None),
                ('pressure','baseline_variability_mode','reject'),('pressure','baseline_variability_mode','ignore'),
                ('deployment','baseline_variability_mode','reject'),
                ('pressure','zero_max_std_counts',60),('pressure','zero_window_s',3)):
            bad=copy.deepcopy(cfg);bad[section][key]=value
            with self.subTest(section=section,key=key,value=value),self.assertRaises(ValueError):
                T.validate_candidate(bad,require_approval=False)

    def test_prior_optionalpose_approval_cannot_authorize_warn_trial(self):
        cfg=config();cfg['confirmations']={key:True for key in T.CONFIRM}
        cfg['deployment'].update(occupied_trial_accepted=True,operator_stop_ready=True,
            approved_trial='occupied_g_hybrid1g20_preview10_res1p5_pitch0_rate10_300s_raw2500_neutral18181212_optionalpose_v2')
        with self.assertRaises(ValueError):T.validate_candidate(cfg)
        cfg['deployment']['approved_trial']=T.APPROVAL
        T.validate_candidate(cfg)
        self.assertTrue(T.APPROVAL.endswith('baselinewarn_v3'))

    def test_preparer_changes_only_reviewed_variance_mode_and_clears_approval(self):
        source=config();source['pressure'].pop('baseline_variability_mode')
        source['deployment'].update(approved_trial=T.APPROVAL,occupied_trial_accepted=True,
                                    baseline_variability_mode='reject')
        before=copy.deepcopy(source)
        result=candidate(source,2)
        self.assertEqual(source,before)
        self.assertEqual(result['pressure']['baseline_variability_mode'],'warn')
        self.assertEqual(result['pressure']['zero_max_std_counts'],40)
        self.assertEqual(result['pressure']['stale_s'],source['pressure']['stale_s'])
        self.assertIsNone(result['deployment']['approved_trial'])
        self.assertFalse(result['deployment']['occupied_trial_accepted'])

    def test_absolute_guard_and_motion_time_limits_remain_unchanged(self):
        cfg=config()
        self.assertEqual(hashlib.sha256(Path(__file__).with_name('raw_count_guard.py').read_bytes()).hexdigest(),
                         '6a65e2193b9fb8b70e089c44d23d5b7fd0d4a0abbbda2ccfec90582746667ac3')
        self.assertEqual(cfg['safety']['pad_saturation_counts'],2500)
        self.assertEqual(cfg['deployment']['raw_release_counts'],2500)
        self.assertTrue(cfg['deployment']['raw_release_latched'])
        self.assertEqual(cfg['deployment']['neutral_settle_s'],2.)
        self.assertEqual(cfg['deployment']['arm_time_budget_s'],300)
        self.assertEqual(T.TRIAL_SECONDS,300)
        self.assertEqual(cfg['theta_max_deg'],20)
        self.assertEqual(cfg['pitch_deg'],{'left':0.,'right':0.})


if __name__=='__main__':unittest.main()
