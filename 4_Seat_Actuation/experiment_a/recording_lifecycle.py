"""Temporary acquisition records, explicitly saved or discarded after stopping.

Importing performs no I/O. Only owned, non-reparse pending trees can be moved
or removed. Raw records are never rewritten when their directory is promoted;
recording_relocation.json supplies the old-root/new-root mapping for references.
"""
from __future__ import annotations

from contextlib import contextmanager
import errno
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import stat
import threading
import time
import uuid

SCHEMA='oasis-pending-recording-v1'
MARKER='recording.json'
RELOCATION='recording_relocation.json'
VERSION=re.compile(r'(?:DEV|V[1-9][0-9]*)\Z')
IDENTIFIER=re.compile(r'A_[0-9]{8}_[0-9]{6}_[0-9a-f]{8}\Z')


class RecordingError(RuntimeError):
    pass


def pending_root(profile):
    if profile.get('pending_root'):
        root=Path(profile['pending_root'])
    elif profile.get('install_root'):
        root=Path(profile['install_root'])/'pending_sessions'
    else:
        # Standalone/offline profiles have no installed application directory.
        root=Path(profile['output_root']).resolve().parent/'.oasis_experiment_a_pending'
    root=_canonical_root(root)
    output=_canonical_root(Path(profile['output_root']))
    if root==output or root.is_relative_to(output) or output.is_relative_to(root):
        raise RecordingError('Temporary and saved recording roots must be separate')
    return root


def _reparse(path):
    info=path.lstat()
    return stat.S_ISLNK(info.st_mode) or bool(getattr(info,'st_file_attributes',0)&0x400)


def _canonical_root(path):
    path=path.expanduser().absolute()
    for ancestor in (path,*path.parents):
        if ancestor.exists() and _reparse(ancestor):
            raise RecordingError('Reparse/symlink recording ancestors are forbidden')
    return path.resolve()


def _no_reparse_tree(path):
    if _reparse(path):raise RecordingError('Reparse/symlink recording paths are forbidden')
    for base, dirs, files in os.walk(path,followlinks=False):
        for name in dirs+files:
            if _reparse(Path(base)/name):
                raise RecordingError('Reparse/symlink inside recording tree is forbidden')


def _atomic_json(path,value):
    temporary=path.with_name(path.name+'.'+uuid.uuid4().hex+'.tmp')
    with temporary.open('x',encoding='utf-8') as handle:
        json.dump(value,handle,indent=2,ensure_ascii=False,allow_nan=False)
        handle.write('\n');handle.flush();os.fsync(handle.fileno())
    temporary.replace(path)


class _Lease:
    """A crash releases the OS lock; lease files live outside movable trees."""
    def __init__(self,path):
        path.parent.mkdir(parents=True,exist_ok=True)
        if _reparse(path.parent) or (path.exists() and _reparse(path)):
            raise RecordingError('Unsafe pending-recording lock path')
        self.handle=path.open('a+b')
        if self.handle.tell()==0:self.handle.write(b'0');self.handle.flush()
        self.handle.seek(0)
        try:
            if os.name=='nt':
                import msvcrt
                msvcrt.locking(self.handle.fileno(),msvcrt.LK_NBLCK,1)
            else:
                import fcntl
                fcntl.flock(self.handle.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
        except OSError as exc:
            self.handle.close();self.handle=None
            raise RecordingError('Recording is still active or being handled') from exc

    def close(self):
        if self.handle is not None:
            self.handle.seek(0)
            try:
                if os.name=='nt':
                    import msvcrt
                    msvcrt.locking(self.handle.fileno(),msvcrt.LK_UNLCK,1)
                else:
                    import fcntl
                    fcntl.flock(self.handle.fileno(),fcntl.LOCK_UN)
            finally:self.handle.close();self.handle=None

    def __del__(self):
        if getattr(self,'handle',None) is not None:self.close()


class PendingRecording:
    def __init__(self,profile,version,recording_id,*,create=False):
        if not VERSION.fullmatch(version) or not IDENTIFIER.fullmatch(recording_id):
            raise RecordingError('Invalid recording version/identifier')
        self.root=pending_root(profile)
        self.output_root=_canonical_root(Path(profile['output_root']))
        self.version,self.recording_id=version,recording_id
        self.staging_path=self.root/version/recording_id
        self.intended_output=self.output_root/version/recording_id
        self.persisted_path=None;self.retention='pending';self.interrupted=False
        self.finished=False;self.error=None;self.cleanup_error=None
        self._lock=threading.RLock();self._active_lease=None
        if create:
            self.root.mkdir(parents=True,exist_ok=True)
            if _reparse(self.root):raise RecordingError('Unsafe pending root')
            parent=self.staging_path.parent
            parent.mkdir(exist_ok=True)
            if _reparse(parent):raise RecordingError('Unsafe pending version directory')
            self._active_lease=_Lease(self._lease_path())
            try:
                self.staging_path.mkdir(exist_ok=False)
                self._metadata=dict(schema=SCHEMA,recording_id=recording_id,version=version,
                    staging_path=str(self.staging_path),intended_output=str(self.intended_output),
                    output_root=str(self.output_root),created_wall_time_ns=time.time_ns(),
                    finished=False,retention='pending',error=None)
                _atomic_json(self.staging_path/MARKER,self._metadata)
            except Exception:
                self._active_lease.close();self._active_lease=None
                raise
        else:
            with self._operation():
                self._metadata=self._validate()
                self.finished=True
                self.interrupted=not bool(self._metadata.get('finished'))
                self.error=self._metadata.get('error')

    def _lease_path(self):
        return self.root/'.locks'/(self.version+'_'+self.recording_id+'.lock')

    def _validate(self):
        # Resolve and verify before *every* recursive move/delete, not just load.
        path=self.staging_path
        if (not path.is_dir() or path.resolve()!=path or path.parent.resolve()!=self.root/self.version
                or path.resolve().parent.parent!=self.root):
            raise RecordingError('Pending recording escapes its owned root')
        if _reparse(self.root) or _reparse(path.parent):raise RecordingError('Unsafe pending ancestors')
        _no_reparse_tree(path)
        try:metadata=json.loads((path/MARKER).read_text(encoding='utf-8'))
        except (OSError,ValueError) as exc:raise RecordingError('Missing/invalid pending ownership marker') from exc
        expected=dict(schema=SCHEMA,recording_id=self.recording_id,version=self.version,
            staging_path=str(path),intended_output=str(self.intended_output),output_root=str(self.output_root))
        if any(metadata.get(key)!=value for key,value in expected.items()):
            raise RecordingError('Pending ownership marker does not match this profile/path')
        return metadata

    @contextmanager
    def _operation(self):
        with self._lock:
            if self._active_lease is not None:raise RecordingError('Stop recording before saving or discarding')
            if _canonical_root(self.root)!=self.root:
                raise RecordingError('Pending root moved or redirected')
            lease=_Lease(self._lease_path())
            try:yield
            finally:lease.close()

    def mark_finished(self,error=None):
        with self._lock:
            try:
                self._metadata=self._validate()
                self.finished=True;self.error=error
                self._metadata.update(finished=True,error=error,ended_wall_time_ns=time.time_ns())
                _atomic_json(self.staging_path/MARKER,self._metadata)
            finally:
                if self._active_lease is not None:
                    self._active_lease.close();self._active_lease=None

    def snapshot(self):
        with self._lock:
            current=self.persisted_path if self.retention=='saved' else self.staging_path if self.retention=='pending' else None
            return dict(retention=self.retention,recording_id=self.recording_id,finished=self.finished,
                staging_path=str(self.staging_path),persisted_path=str(self.persisted_path) if self.persisted_path else None,
                intended_output=str(self.intended_output),output=str(current) if current else None,
                interrupted=self.interrupted,error=self.error,cleanup_error=self.cleanup_error,
                phase='STOPPED',message={
                    'pending':'임시 기록 — 저장 또는 버리기를 선택하세요.',
                    'saved':'기록 저장 완료 — 제어는 중지 상태이며 다시 연결로 새 시험을 시작할 수 있습니다.',
                    'discarded':'임시 기록을 버렸습니다 — 다시 연결로 새 시험을 시작할 수 있습니다.',
                }[self.retention])

    def _saved_metadata(self):
        return dict(self._metadata,retention='saved',saved_wall_time_ns=time.time_ns(),
                    persisted_path=str(self.intended_output),finished=True)

    def _write_relocation(self,root):
        _atomic_json(root/RELOCATION,dict(schema='oasis-recording-relocation-v1',
            recording_id=self.recording_id,original_root=str(self.staging_path),
            saved_root=str(self.intended_output),raw_files_rewritten=False,
            resolution='Replace original_root prefix with the directory containing this sidecar'))

    def save_recording(self):
        with self._operation():
            if self.retention!='pending' or not self.finished:
                raise RecordingError('Only a stopped pending recording can be saved')
            self._metadata=self._validate()
            target=self.intended_output
            if target.exists():raise RecordingError('Saved destination already exists; no overwrite performed')
            if _canonical_root(target.parent)!=self.output_root/self.version:
                raise RecordingError('Saved-recording root moved or redirected')
            target.parent.mkdir(parents=True,exist_ok=True)
            if target.parent.resolve()!=self.output_root/self.version or _reparse(target.parent) or _reparse(self.output_root):
                raise RecordingError('Unsafe saved-recording directory')
            self._write_relocation(self.staging_path)
            _atomic_json(self.staging_path/MARKER,self._saved_metadata())
            try:
                self.staging_path.rename(target)
            except OSError as exc:
                # Windows reports cross-drive rename as WinError17/EXDEV.
                if exc.errno!=errno.EXDEV and getattr(exc,'winerror',None)!=17:
                    _atomic_json(self.staging_path/MARKER,self._metadata)
                    raise
                self._copy_across_volumes(target)
            self.retention='saved';self.persisted_path=target
            return target

    def _copy_across_volumes(self,target):
        temporary=target.parent/('.pending_save_'+self.recording_id+'_'+uuid.uuid4().hex)
        try:
            source_hashes=_hash_tree(self.staging_path)
            shutil.copytree(self.staging_path,temporary)
            if _hash_tree(temporary)!=source_hashes:
                raise RecordingError('Cross-volume copy verification failed; original pending record retained')
            temporary.rename(target)
        except Exception:
            if temporary.exists():_remove_owned_tree(temporary,target.parent,prefix='.pending_save_'+self.recording_id+'_')
            _atomic_json(self.staging_path/MARKER,self._metadata)
            raise
        # The destination is fully promoted. A cleanup problem must not hide it.
        try:
            self._validate()
            _remove_owned_tree(self.staging_path,self.root/self.version,exact=self.recording_id)
        except Exception as exc:self.cleanup_error=str(exc)

    def discard_recording(self):
        with self._operation():
            if self.retention!='pending' or not self.finished:
                raise RecordingError('Only a stopped pending recording can be discarded')
            self._validate()
            _remove_owned_tree(self.staging_path,self.root/self.version,exact=self.recording_id)
            self.retention='discarded'


def _remove_owned_tree(path,parent,*,exact=None,prefix=None):
    parent=parent.resolve();resolved=path.resolve()
    if resolved.parent!=parent or resolved==parent or (exact is not None and path.name!=exact) or (prefix is not None and not path.name.startswith(prefix)):
        raise RecordingError('Refusing deletion outside the explicitly owned recording directory')
    if resolved!=path:raise RecordingError('Refusing deletion through a redirected path')
    _no_reparse_tree(path)
    shutil.rmtree(path)


def _hash_tree(root):
    _no_reparse_tree(root)
    result={}
    for path in sorted(root.rglob('*')):
        if path.is_file():
            digest=hashlib.sha256()
            with path.open('rb') as handle:
                for chunk in iter(lambda:handle.read(1024*1024),b''):digest.update(chunk)
            result[path.relative_to(root).as_posix()]=digest.hexdigest()
    return result


def discover_pending(profile):
    root=pending_root(profile)
    if not root.exists():return []
    if _reparse(root):raise RecordingError('Unsafe pending root')
    records=[]
    for version in sorted(root.iterdir()):
        if not VERSION.fullmatch(version.name):continue
        if _reparse(version):raise RecordingError('Unsafe pending version directory')
        if not version.is_dir():continue
        for folder in sorted(version.iterdir()):
            if not IDENTIFIER.fullmatch(folder.name):continue
            try:records.append(PendingRecording(profile,version.name,folder.name))
            except RecordingError as exc:
                if str(exc)=='Recording is still active or being handled':continue
                raise
    return records


def resolve_recording_path(recording_root,reference):
    """Resolve historical absolute paths after save without editing raw files."""
    root=Path(recording_root).resolve();value=Path(reference)
    if not value.is_absolute():
        candidate=(root/value).resolve()
    elif (root/RELOCATION).exists():
        info=json.loads((root/RELOCATION).read_text(encoding='utf-8'))
        original=Path(info['original_root'])
        if not value.is_relative_to(original):raise RecordingError('Reference is outside original recording')
        candidate=(root/value.relative_to(original)).resolve()
    else:candidate=value.resolve()
    if not candidate.is_relative_to(root):raise RecordingError('Reference escapes recording')
    return candidate
