"""Immutable, local-only desktop releases; never opens hardware or downloads code.

The source tree is an editable checkout. A release contains only the Python
runtime and candidate settings, and current.json selects a complete snapshot.
Callers must separately prevent updates while a hardware session is running.
"""
from __future__ import annotations

import argparse
from contextlib import contextmanager
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import time
import uuid


VERSION_RE = re.compile(r"V([1-9][0-9]*)\Z")
SHA_RE = re.compile(r"[0-9a-f]{64}\Z")
SHARED_FILES = ("fallback/__init__.py", "fallback/beamng.py",
                "fallback/beamng_bridge.py", "fallback/serial_link.py")
BASELINE_PATH = "experiment_a/config.candidate.json"


class ReleaseError(RuntimeError):
    """A local release is incomplete, modified or cannot be published safely."""


def _strict_json(path):
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ReleaseError(f"Duplicate JSON key: {key}")
            result[key] = value
        return result
    try:
        with Path(path).open("r", encoding="utf-8") as stream:
            value = json.load(stream, object_pairs_hook=pairs,
                              parse_constant=lambda x: (_ for _ in ()).throw(
                                  ReleaseError(f"Non-finite JSON number: {x}")))
    except (OSError, ValueError) as exc:
        raise ReleaseError(f"Cannot read release metadata: {path}") from exc
    if not isinstance(value, dict):
        raise ReleaseError(f"Expected JSON object: {path}")
    return value


def _version(value):
    if not isinstance(value, str) or not VERSION_RE.fullmatch(value):
        raise ReleaseError(f"Invalid release version: {value!r}")
    return value


def _relative_path(value):
    if not isinstance(value, str) or "\\" in value or ":" in value:
        raise ReleaseError(f"Invalid release path: {value!r}")
    path = PurePosixPath(value)
    if (path.is_absolute() or len(path.parts) != 2 or
            any(part in ("", ".", "..") for part in path.parts) or
            str(path) != value):
        raise ReleaseError(f"Invalid release path: {value!r}")
    if value not in SHARED_FILES and value != BASELINE_PATH:
        if path.parts[0] != "experiment_a" or path.suffix != ".py":
            raise ReleaseError(f"Unexpected release file: {value!r}")
    return path


def _safe_file(root, relative):
    parts = _relative_path(relative).parts
    candidate = root.joinpath(*parts)
    if candidate.parent.is_symlink() or candidate.is_symlink():
        raise ReleaseError(f"Symlink in release: {relative}")
    if candidate.resolve().parent != (root / parts[0]).absolute():
        raise ReleaseError(f"Release path escapes root: {relative}")
    return candidate


def _inventory(root):
    root = Path(root).resolve()
    package = root / "experiment_a"
    if not package.is_dir() or package.is_symlink():
        raise ReleaseError(f"Missing or linked runtime package: {package}")
    relative = {f"experiment_a/{path.name}" for path in package.glob("*.py")}
    relative.update(SHARED_FILES)
    relative.add(BASELINE_PATH)
    relative.add("experiment_a/__init__.py")
    files = {}
    for name in sorted(relative):
        path = _safe_file(root, name)
        try:
            files[name] = path.read_bytes()
        except OSError as exc:
            raise ReleaseError(f"Missing or unreadable runtime file: {path}") from exc
    # Reject malformed baseline before it becomes the installed reference.
    _strict_json(root / BASELINE_PATH)
    return files


def _hashes(files):
    return {name: hashlib.sha256(data).hexdigest() for name, data in files.items()}


def _fingerprint(hashes):
    digest = hashlib.sha256()
    for name, sha in sorted(hashes.items()):
        _relative_path(name)
        if not isinstance(sha, str) or not SHA_RE.fullmatch(sha):
            raise ReleaseError(f"Invalid SHA256 for {name}")
        digest.update(name.encode("utf-8") + b"\0" + sha.encode("ascii") + b"\n")
    return digest.hexdigest()


def _write_json(path, value):
    with Path(path).open("x", encoding="utf-8", newline="\n") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())


def _verify_release(release_root, version, fingerprint=None):
    release_root = Path(release_root)
    manifest = _strict_json(release_root / "release.json")
    if _version(manifest.get("version")) != version:
        raise ReleaseError("Release version does not match selected directory")
    hashes = manifest.get("files")
    if not isinstance(hashes, dict) or not hashes:
        raise ReleaseError("Release file manifest is missing")
    calculated = _fingerprint(hashes)
    if manifest.get("fingerprint") != calculated:
        raise ReleaseError("Release manifest fingerprint mismatch")
    if fingerprint is not None and fingerprint != calculated:
        raise ReleaseError("Current pointer fingerprint mismatch")
    actual = _hashes(_inventory(release_root))
    if actual != hashes:
        raise ReleaseError("Release contents changed or are incomplete")
    if manifest.get("baseline_config_sha256") != hashes.get(BASELINE_PATH):
        raise ReleaseError("Baseline configuration hash mismatch")
    return manifest


def inspect_release(release_root):
    """Verify this exact Vn snapshot independently of the current selection.

    An already running V1 keeps reporting V1 if an idle publisher selects V2.
    """
    original = Path(release_root).absolute()
    version = _version(original.name)
    if original.is_symlink() or original.parent.is_symlink():
        raise ReleaseError("Linked release directory is not permitted")
    root = original.resolve()
    manifest = _verify_release(root, version)
    return {"version": version, "fingerprint": manifest["fingerprint"],
            "release_root": str(root), "manifest": manifest,
            "baseline_config_sha256": manifest["baseline_config_sha256"]}


def get_current(install_root, verify=True):
    """Return selected release metadata, or None before first publication."""
    root = Path(install_root).resolve()
    pointer = root / "current.json"
    if not pointer.exists():
        return None
    selected = _strict_json(pointer)
    version = _version(selected.get("version"))
    fingerprint = selected.get("fingerprint")
    if not isinstance(fingerprint, str) or not SHA_RE.fullmatch(fingerprint):
        raise ReleaseError("Invalid current release fingerprint")
    releases = root / "releases"
    release_root = releases / version
    if (releases.is_symlink() or release_root.is_symlink() or
            release_root.resolve().parent != releases.absolute()):
        raise ReleaseError("Release directory escapes install root")
    manifest = (_verify_release(release_root, version, fingerprint) if verify else
                _strict_json(release_root / "release.json"))
    return {"version": version, "fingerprint": fingerprint,
            "release_root": str(release_root), "manifest": manifest,
            "baseline_config_sha256": manifest.get("baseline_config_sha256")}


def check_update(source_root, install_root):
    """Compare checkout with installation without creating or modifying files."""
    hashes = _hashes(_inventory(source_root))
    fingerprint = _fingerprint(hashes)
    current = get_current(install_root)
    return {"changed": current is None or current["fingerprint"] != fingerprint,
            "installed_version": current["version"] if current else None,
            "fingerprint": fingerprint, "current": current,
            "baseline_config_changed": current is not None and
                current["baseline_config_sha256"] != hashes[BASELINE_PATH]}


@contextmanager
def _update_lock(root):
    """OS lock survives crashes without a stale-lock deletion race."""
    root.mkdir(parents=True, exist_ok=True)
    with (root / "update.lock").open("a+b") as stream:
        stream.seek(0, os.SEEK_END)
        if stream.tell() == 0:
            stream.write(b"\0")
            stream.flush()
        stream.seek(0)
        try:
            if os.name == "nt":
                import msvcrt
                msvcrt.locking(stream.fileno(), msvcrt.LK_NBLCK, 1)
            else:
                import fcntl
                fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            raise ReleaseError("Another release update is in progress") from exc
        try:
            yield
        finally:
            stream.seek(0)
            if os.name == "nt":
                msvcrt.locking(stream.fileno(), msvcrt.LK_UNLCK, 1)
            else:
                fcntl.flock(stream.fileno(), fcntl.LOCK_UN)


def publish_release(source_root, install_root):
    """Publish a verified immutable Vn snapshot and atomically select it.

    Identical current content reuses its Vn. Old directories are never rewritten
    or removed. Failure before pointer replacement preserves the selected Vn.
    """
    source_root, root = Path(source_root).resolve(), Path(install_root).resolve()
    with _update_lock(root):
        current = get_current(root)
        files = _inventory(source_root)
        for name, data in files.items():
            if name.endswith(".py"):
                try:
                    compile(data, name, "exec", dont_inherit=True)
                except (SyntaxError, ValueError) as exc:
                    raise ReleaseError(f"Python syntax error in {name}: {exc}") from exc
        hashes = _hashes(files)
        fingerprint = _fingerprint(hashes)
        if current is not None and current["fingerprint"] == fingerprint:
            return {**current, "changed": False}
        releases = root / "releases"
        if releases.is_symlink():
            raise ReleaseError("Linked releases directory is not permitted")
        releases.mkdir(exist_ok=True)
        numbers = [int(match.group(1)) for path in releases.iterdir()
                   if (match := VERSION_RE.fullmatch(path.name))]
        version = f"V{max(numbers, default=0) + 1}"
        destination = releases / version
        stage = releases / f".pending-{version}-{uuid.uuid4().hex}"
        pointer_tmp = root / f".current-{uuid.uuid4().hex}.json"
        manifest = {"schema_version": 1, "version": version,
                    "fingerprint": fingerprint, "files": hashes,
                    "created_wall_time_ns": time.time_ns(),
                    "source_root": str(source_root),
                    "baseline_config_path": BASELINE_PATH,
                    "baseline_config_sha256": hashes[BASELINE_PATH]}
        try:
            stage.mkdir()
            for name, data in files.items():
                target = stage.joinpath(*PurePosixPath(name).parts)
                target.parent.mkdir(exist_ok=True)
                with target.open("xb") as stream:
                    stream.write(data)
                    stream.flush()
                    os.fsync(stream.fileno())
            _write_json(stage / "release.json", manifest)
            _verify_release(stage, version, fingerprint)
            if _hashes(_inventory(source_root)) != hashes:
                raise ReleaseError("Source changed during update; retry after editing finishes")
            # rename (not replace) refuses an existing release on Windows.
            # The update lock serializes publishers on all supported systems.
            if destination.exists():
                raise ReleaseError("Release destination already exists")
            stage.rename(destination)
            _write_json(pointer_tmp, {"version": version, "fingerprint": fingerprint})
            os.replace(pointer_tmp, root / "current.json")
        finally:
            if pointer_tmp.exists():
                pointer_tmp.unlink()
            if stage.exists():
                # Only this operation's private staging directory may be removed.
                if (stage.resolve().parent != releases.resolve() or
                        not stage.name.startswith(".pending-") or stage.is_symlink()):
                    raise ReleaseError("Refusing unsafe staging cleanup")
                shutil.rmtree(stage)
        return {"version": version, "fingerprint": fingerprint,
                "release_root": str(destination), "changed": True,
                "manifest": manifest, "baseline_config_sha256": hashes[BASELINE_PATH]}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("publish", "check", "current"))
    parser.add_argument("--source-root", type=Path)
    parser.add_argument("--install-root", type=Path, required=True)
    args = parser.parse_args(argv)
    if args.command != "current" and args.source_root is None:
        parser.error("--source-root is required for publish/check")
    try:
        result = (publish_release(args.source_root, args.install_root)
                  if args.command == "publish" else
                  check_update(args.source_root, args.install_root)
                  if args.command == "check" else get_current(args.install_root))
    except (ReleaseError, OSError) as exc:
        parser.exit(1, f"Release update failed: {exc}\n")
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
