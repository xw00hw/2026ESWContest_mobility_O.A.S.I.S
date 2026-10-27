"""Prepare an UNAPPROVED occupied limited candidate. No hardware imports/access."""
from pathlib import Path
import argparse
import copy
import hashlib
import json

from occupied_trial import (CONFIRM,HYBRID_CONFIG,OPTIONAL_POSE_CONFIG,TRIAL_SECONDS,
                            BASELINE_STD_COUNTS,BASELINE_VARIABILITY_MODE,validate_candidate)


def candidate(source, expected_revision):
    cfg=copy.deepcopy(source)
    if float(cfg.get('theta_max_deg',0))!=20:
        raise ValueError('Source model theta_max_deg is not 20; review rather than silently changing it')
    cfg['confirmations']={k:False for k in CONFIRM}
    cfg['dt_s']=.05;cfg['filter_tau_s']=.3
    cfg['g_residual_control']=copy.deepcopy(HYBRID_CONFIG)
    cfg['control'].update(mode='rl',cold_fallback=None)
    cfg['pitch_deg']={'left':0.,'right':0.}
    cfg['pose']=copy.deepcopy(OPTIONAL_POSE_CONFIG);cfg['profile']['from_camera']=False
    cfg['rig']='static_bench'
    cfg['safety']['arm_deg']=0;cfg['safety']['mode']='raw'
    cfg['safety']['pad_saturation_counts']=2500
    cfg['pressure']['no_pressure_yaw_cap_deg']=20.  # Missing/stale is independently latched OPEN; valid zero counts are no-contact, not invalid.
    if float(cfg['pressure']['zero_max_std_counts'])!=BASELINE_STD_COUNTS:
        raise ValueError('Existing baseline diagnostic threshold must remain 40 counts')
    cfg['pressure']['baseline_variability_mode']=BASELINE_VARIABILITY_MODE
    cfg.setdefault('deployment',{}).update(approved_trial=None,occupied_trial_accepted=False,
        operator_stop_ready=False,pressure_masking=False,physical_trial_yaw_cap_deg=20,raw_release_counts=2500,raw_release_latched=True,expected_tuning_revision=expected_revision,
        requested_trial='UNAPPROVED G-hybrid: measured1G prefilter20deg, preview10, residual1.5; raw2500latched release, pitch0,300s including2s settle+2s baseline; variability40count warning only',
        baseline_window_s=2.,baseline_max_std_counts=BASELINE_STD_COUNTS,
        baseline_variability_mode=BASELINE_VARIABILITY_MODE,
        policy='g_residual_rl',arm_time_budget_s=TRIAL_SECONDS,
        physical_motion_authorized=False,hardware_arm_performed=False,
        rl_after_new_neutral_authorized=False,approval_recorded_wall_time_ns=None,
        release='reconnect_20261028_optionalpose_candidate',
        commissioning='Local software candidate only; no hardware opened or current approval inherited',
        source_config_sha256=None,
        inherited_rl_wrapper_use='Only this isolated G-hybrid wrapper and a newly reviewed runtime release; old approval tags are invalid',
        physical_pressure_calibrated=False,neutral_pulses_us=[1800,1800,1200,1200],neutral_settle_s=2.,
        camera_excluded_by_operator=False,camera_input_optional=True,
        camera_pose_neutral_available=False,camera_pose_physical_alignment_verified=False)
    # Keep the explicitly reviewed destination from the input template.
    cfg['log_dir']=source.get('log_dir','/REPLACE_WITH_NEW_TRIAL_LOG_DIRECTORY')
    validate_candidate(cfg,require_approval=False)
    return cfg


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',required=True,type=Path);p.add_argument('--output',required=True,type=Path)
    p.add_argument('--expected-tuning-revision',required=True,type=int)
    args=p.parse_args(argv)
    import yaml
    cfg=candidate(yaml.safe_load(args.source.read_text(encoding='utf-8')),args.expected_tuning_revision)
    cfg['deployment'].update(source_config=str(args.source.resolve()),
        source_config_sha256=hashlib.sha256(args.source.read_bytes()).hexdigest())
    with args.output.open('x',encoding='utf-8') as h:yaml.safe_dump(cfg,h,sort_keys=False)
    print(json.dumps({'output':str(args.output),'source_sha256':hashlib.sha256(args.source.read_bytes()).hexdigest(),
        'approved':False,'all_confirmations':cfg['confirmations'],'pressure_saturation_counts':cfg['safety'].get('pad_saturation_counts')}))


if __name__=='__main__':main()
