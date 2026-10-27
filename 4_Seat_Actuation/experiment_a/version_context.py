"""Identity of the running code, independent of the newest installed pointer."""
from pathlib import Path
import re


def running_release():
    root=Path(__file__).resolve().parent.parent
    if (root/'release.json').exists():
        from .releases import inspect_release
        result=inspect_release(root)
        return {key:result[key] for key in ('version','fingerprint','release_root')}
    return dict(version='DEV',fingerprint=None,release_root=str(root))


def session_release(profile):
    info=dict(profile.get('release') or running_release())
    version=info.get('version')
    if version != 'DEV' and (not isinstance(version,str) or not re.fullmatch(r'V[1-9][0-9]*',version)):
        raise ValueError('Invalid runtime code version')
    if version != 'DEV' and not re.fullmatch(r'[0-9a-f]{64}',str(info.get('fingerprint',''))):
        raise ValueError('Missing runtime release fingerprint')
    return info


def versioned_output_root(profile):
    return Path(profile['output_root'])/session_release(profile)['version']
