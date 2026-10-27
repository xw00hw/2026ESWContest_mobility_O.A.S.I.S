"""Stable installed entry point; resolve and verify a snapshot before imports."""
from contextlib import contextmanager
import argparse
import hashlib
import json
from pathlib import Path
import re
import sys


@contextmanager
def application_lock(path):
    import msvcrt
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('a+b') as handle:
        if handle.tell()==0:handle.write(b'0');handle.flush()
        handle.seek(0)
        try:msvcrt.locking(handle.fileno(),msvcrt.LK_NBLCK,1)
        except OSError:raise RuntimeError('BeamNG Bolster A가 이미 실행 중입니다.')
        try:yield
        finally:
            handle.seek(0);msvcrt.locking(handle.fileno(),msvcrt.LK_UNLCK,1)


def resolve_release(install_root):
    root=Path(install_root).resolve()
    current=json.loads((root/'current.json').read_text(encoding='utf-8'))
    version=current.get('version','')
    if not isinstance(version,str) or not re.fullmatch(r'V[1-9][0-9]*',version):
        raise ValueError('Invalid installed version')
    release=(root/'releases'/version).resolve()
    if release.parent != (root/'releases').resolve():raise ValueError('Invalid release location')
    manifest=json.loads((release/'release.json').read_text(encoding='utf-8'))
    if manifest.get('version')!=version or manifest.get('fingerprint')!=current.get('fingerprint'):
        raise ValueError('Release pointer/manifest mismatch')
    hashes=manifest.get('files',{})
    if not isinstance(hashes,dict) or 'experiment_a/desktop.py' not in hashes:
        raise ValueError('Incomplete installed release')
    actual={f'experiment_a/{p.name}' for p in (release/'experiment_a').glob('*.py')}
    actual.update(('experiment_a/config.candidate.json','fallback/__init__.py',
                   'fallback/beamng.py','fallback/beamng_bridge.py','fallback/serial_link.py'))
    if set(hashes)!=actual:raise ValueError('Unlisted or missing runtime files')
    digest=hashlib.sha256()
    for relative,expected in sorted(hashes.items()):
        if not isinstance(relative,str) or '\\' in relative or ':' in relative:
            raise ValueError('Invalid release file')
        original=release/relative
        if original.is_symlink() or original.parent.is_symlink():raise ValueError('Linked release file')
        path=original.resolve()
        if not path.is_relative_to(release) or not path.is_file():raise ValueError('Missing release file')
        if hashlib.sha256(path.read_bytes()).hexdigest()!=expected:
            raise ValueError('Installed code changed: '+relative)
        digest.update(relative.encode('utf-8')+b'\0'+expected.encode('ascii')+b'\n')
    if digest.hexdigest()!=manifest['fingerprint']:raise ValueError('Release fingerprint mismatch')
    return release


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile',required=True)
    parser.add_argument('--check-ui',action='store_true')
    parser.add_argument('--preview',action='store_true',help='Visible read-only status UI without hardware or network')
    args=parser.parse_args(argv)
    profile_path=Path(args.profile).resolve()
    try:
        with application_lock(profile_path.with_suffix('.lock')):
            profile=json.loads(profile_path.read_text(encoding='utf-8-sig'))
            release=resolve_release(profile.get('install_root',profile_path.parent))
            sys.path.insert(0,str(release))
            sys.dont_write_bytecode=True
            from experiment_a.desktop import main as desktop_main
            forwarded=['--profile',str(profile_path)]+(['--check-ui'] if args.check_ui else [])
            if args.preview:forwarded.append('--preview')
            return desktop_main(forwarded,lock_held=True)
    except Exception as exc:
        if args.check_ui:raise
        import tkinter as tk
        from tkinter import messagebox
        window=tk.Tk();window.withdraw()
        messagebox.showerror('BeamNG Bolster A',str(exc),parent=window);window.destroy()
        return 1


if __name__=='__main__':raise SystemExit(main())
