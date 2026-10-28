"""Local packaging sanitizer, not a runtime/approval or device launcher."""
import hashlib
import json
from pathlib import Path
import yaml


def main():
    root = Path(__file__).parent
    cfg = yaml.safe_load((root / 'candidate.yaml').read_text(encoding='utf-8'))
    cfg['fb_root'] = '/REPLACE_WITH_ESW_REPOSITORY/4_Seat_Actuation'
    cfg['model_dir'] = '/REPLACE_WITH_MODEL_V8_DIRECTORY'
    cfg['actor']['c_lib'] = '/REPLACE_WITH_BUILT_LIBOASIS_ACTOR_SO'
    cfg['actor']['hef'] = None
    cfg['actor']['check_set'] = '/REPLACE_WITH_MODEL_V8_DIRECTORY/check_set.npy'
    cfg['link']['port'] = '/dev/serial/by-id/REPLACE_WITH_VERIFIED_STM32'
    cfg['pressure']['pad_calib'] = '/REPLACE_WITH_EXISTING_PAD_CALIBRATION_JSON'
    cfg['vehicle']['replay'] = None
    cfg['beamng']['allow_from'] = ['REPLACE_WITH_VERIFIED_PC_IP']
    cfg['wheel']['allow_from'] = ['REPLACE_WITH_VERIFIED_PC_IP']
    cfg['wheel']['device_key'] = 'REPLACE_WITH_VERIFIED_MOZA_DEVICE'
    cfg['wheel']['calibration']['confirmed'] = False
    cfg['wheel']['calibration']['beamng_vehicle_id'] = 'REPLACE_WITH_CALIBRATED_VEHICLE_ID'
    cfg['log_dir'] = '/REPLACE_WITH_NEW_TRIAL_LOG_DIRECTORY'
    cfg['deployment']['source_config'] = 'candidate.yaml (repository template)'
    cfg['deployment']['source_config_sha256'] = None
    cfg['deployment']['commissioning'] = 'Template only; paths and local calibration require review; readiness is not inherited'
    (root / 'candidate.yaml').write_text(yaml.safe_dump(cfg,sort_keys=False,allow_unicode=True),encoding='utf-8')
    path = root / 'prepare_candidate.py'
    text = path.read_text(encoding='utf-8')
    text = text.replace("    cfg['log_dir']='/home/esw/ESW_RL_FB/reconnect/logs'", "    # Keep the explicitly reviewed destination from the input template.\n    cfg['log_dir']=source.get('log_dir','/REPLACE_WITH_NEW_TRIAL_LOG_DIRECTORY')")
    path.write_text(text,encoding='utf-8')
    paths = sorted(p for p in root.rglob('*.py') if p.name != Path(__file__).name)
    provenance = {'scope':'seated commissioning source, no run authorization',
        'unchanged_from_physically_used_core':['occupied_trial.py','raw_count_guard.py','startup_alignment.py','native/rtmpose_viewer.py','camera/collect_pose_reference.py','pc/start_pc_displays.py'],
        'template_changes':['candidate.yaml: local identifiers/paths removed; all readiness remains false; wheel calibration confirmation false',
            'prepare_candidate.py: preserve explicitly supplied log_dir instead of historical machine path'],
        'sha256':{p.relative_to(root).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}}
    (root/'source_provenance.json').write_text(json.dumps(provenance,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')


if __name__ == '__main__':
    main()
