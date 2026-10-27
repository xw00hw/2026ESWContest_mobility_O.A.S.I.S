"""Install/update the same local shortcut target, preserving trial settings."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import uuid

from .bootstrap import application_lock
from .releases import publish_release


def install(source_root, install_root, collector_root, output_root, *, reset_settings=False):
    source=Path(source_root).resolve();destination=Path(install_root).resolve()
    destination.mkdir(parents=True,exist_ok=True)
    profile_path=destination/'desktop_profile.json'
    with application_lock(profile_path.with_suffix('.lock')):
        if profile_path.exists():
            profile=json.loads(profile_path.read_text(encoding='utf-8-sig'))
        else:
            profile=dict(schema='oasis-experiment-a-desktop-v2',
                collector_root=str(Path(collector_root).resolve()),
                settings_path=str(destination/'trial_settings.json'),output_root=str(Path(output_root).resolve()),
                port='auto',labels=dict(experiment='A',controller='A_BeamNG_only',
                    participant='unpersonalized',scenario='use_collector_scenario_label',
                    replicate='unique_session_id'))
        profile.update(schema='oasis-experiment-a-desktop-v2',source_root=str(source),install_root=str(destination))
        settings_path=Path(profile['settings_path'])
        if not settings_path.is_absolute():settings_path=destination/settings_path
        settings_backup=None
        old_settings=settings_path.read_bytes() if settings_path.exists() else None
        if reset_settings:
            suffix=datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8]
            if old_settings is not None:
                settings_backup=settings_path.with_name(settings_path.stem+'.before_reset_'+suffix+'.json')
                with settings_backup.open('xb') as handle:handle.write(old_settings)
            if profile_path.exists():
                profile_backup=profile_path.with_name('desktop_profile.before_reset_'+suffix+'.json')
                with profile_backup.open('xb') as handle:handle.write(profile_path.read_bytes())
            # Reset is explicit and intended for a new control/geometry contract.
            # Normal updates below continue to preserve operator settings exactly.
            baseline=(source/'experiment_a/config.candidate.json').read_bytes()
            parsed=json.loads(baseline)
            if not isinstance(parsed,dict) or any(parsed.get('confirmations',{}).values()):
                raise ValueError('Reset candidate must not include positive hardware confirmations')
            temporary=settings_path.with_suffix('.reset.tmp')
            temporary.write_bytes(baseline);temporary.replace(settings_path)
            profile.setdefault('labels',{})['participant']='unpersonalized'
        elif not settings_path.exists():
            with settings_path.open('xb') as handle:handle.write((source/'experiment_a/config.candidate.json').read_bytes())
        try:
            result=publish_release(source,destination)
        except BaseException:
            if reset_settings:
                if old_settings is None:settings_path.unlink(missing_ok=True)
                else:
                    temporary=settings_path.with_suffix('.restore.tmp')
                    temporary.write_bytes(old_settings);temporary.replace(settings_path)
            raise
        bootstrap=destination/'bootstrap.py';temporary=destination/'bootstrap.tmp'
        temporary.write_bytes((Path(result['release_root'])/'experiment_a/bootstrap.py').read_bytes());temporary.replace(bootstrap)
        temporary=destination/'desktop_profile.tmp'
        temporary.write_text(json.dumps(profile,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
        temporary.replace(profile_path)
        (Path(profile['output_root'])/result['version']).mkdir(parents=True,exist_ok=True)
        return dict(version=result['version'],changed=result['changed'],fingerprint=result['fingerprint'],
                    bootstrap=str(bootstrap),profile=str(profile_path),settings=str(settings_path),
                    output=str(Path(profile['output_root'])/result['version']),
                    settings_reset=reset_settings,settings_backup=str(settings_backup) if settings_backup else None)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('source-root','install-root','collector-root','output-root'):
        parser.add_argument('--'+name,required=True)
    parser.add_argument('--reset-settings',action='store_true',help='Back up old settings and replace with unconfirmed candidate')
    args=parser.parse_args()
    print(json.dumps(install(args.source_root,args.install_root,args.collector_root,args.output_root,
                            reset_settings=args.reset_settings),
                     indent=2,ensure_ascii=False))


if __name__=='__main__':main()
