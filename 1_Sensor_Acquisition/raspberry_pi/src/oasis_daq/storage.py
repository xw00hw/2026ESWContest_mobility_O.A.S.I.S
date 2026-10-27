"""Session directory layout, buffered writers and free-space policy.

Nothing in this module ever deletes, formats or truncates existing data.  New
sessions always land in a freshly created directory; if that directory somehow
exists the session id is suffixed rather than reused.
"""

from __future__ import annotations

import csv
import json
import os
import shutil
import threading
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Iterable, Sequence

from .clock import monotonic_ns

__all__ = [
    "StorageRoot",
    "resolve_storage_root",
    "SessionPaths",
    "CsvWriter",
    "JsonlWriter",
    "RawChunkWriter",
    "free_gb",
    "DiskStatus",
]


def free_gb(path: str | os.PathLike[str]) -> float:
    """Free gibibytes on the filesystem holding *path* (0.0 if unavailable)."""
    try:
        return shutil.disk_usage(str(path)).free / (1024 ** 3)
    except OSError:
        return 0.0


@dataclass
class DiskStatus:
    path: str
    mounted: bool
    writable: bool
    free_gb: float
    total_gb: float

    def to_dict(self) -> dict:
        return {
            "path": self.path,
            "mounted": self.mounted,
            "writable": self.writable,
            "free_gb": round(self.free_gb, 2),
            "total_gb": round(self.total_gb, 2),
        }


@dataclass
class StorageRoot:
    """The directory that holds ``sessions/``, plus how we got there."""

    path: Path
    is_fallback: bool
    reason: str

    def status(self) -> DiskStatus:
        try:
            usage = shutil.disk_usage(str(self.path))
            total, free = usage.total / (1024 ** 3), usage.free / (1024 ** 3)
            mounted = True
        except OSError:
            total = free = 0.0
            mounted = False
        return DiskStatus(
            path=str(self.path),
            mounted=mounted,
            writable=os.access(self.path, os.W_OK),
            free_gb=free,
            total_gb=total,
        )

    def to_dict(self) -> dict:
        d = self.status().to_dict()
        d.update({"is_fallback": self.is_fallback, "reason": self.reason})
        return d


def _probe_writable(directory: Path) -> bool:
    """Create *directory* if needed and confirm we can actually write in it."""
    try:
        directory.mkdir(parents=True, exist_ok=True)
    except OSError:
        return False
    probe = directory / ".oasis_write_probe"
    try:
        with open(probe, "wb") as fh:
            fh.write(b"ok")
            fh.flush()
            os.fsync(fh.fileno())
        probe.unlink()
        return True
    except OSError:
        try:
            probe.unlink()
        except OSError:
            pass
        return False


def resolve_storage_root(cfg: dict) -> StorageRoot:
    """Pick the first configured root that exists and is writable.

    The external SSD is expected to be mounted already (udisks2 mounts the
    Samsung T7 at ``/media/esw/T7``).  We never mount, unmount or modify the
    device here -- only the ``OASIS_DAQ`` subtree is created.
    """
    # A configured root must sit on a real mount by default.  is_dir() is not
    # that test: a leftover directory from a previous mount would pass it, and
    # the DAQ would then silently record onto the microSD while reporting the
    # SSD path in metadata.json.  Set require_mountpoint: false for a
    # deliberately non-removable root.
    require_mount = bool(cfg.get("require_mountpoint", True))
    for candidate in cfg.get("roots", []) or []:
        path = Path(os.path.expanduser(str(candidate)))
        if require_mount and not os.path.ismount(path.parent):
            continue
        if not require_mount and not path.parent.is_dir():
            continue
        if _probe_writable(path):
            return StorageRoot(path=path, is_fallback=False,
                               reason=f"configured root {path} is mounted and writable")

    fallback = Path(os.path.expanduser(str(cfg.get("fallback_root",
                                                   "~/oasis-daq-fallback"))))
    if _probe_writable(fallback):
        return StorageRoot(
            path=fallback, is_fallback=True,
            reason="no configured storage root was available; using local fallback",
        )
    raise RuntimeError(
        f"no writable storage root: tried {cfg.get('roots')} and fallback {fallback}"
    )


@dataclass
class SessionPaths:
    """Every path inside one session directory."""

    root: Path            # <storage_root>/sessions/<date>/<session_dir>
    session_id: str

    @property
    def camera(self) -> Path: return self.root / "camera"
    @property
    def pose(self) -> Path: return self.root / "pose"
    @property
    def imu(self) -> Path: return self.root / "imu"
    @property
    def gps(self) -> Path: return self.root / "gps"
    @property
    def health(self) -> Path: return self.root / "health"

    @property
    def events(self) -> Path: return self.root / "events.jsonl"
    @property
    def metadata(self) -> Path: return self.root / "metadata.json"
    @property
    def manifest(self) -> Path: return self.root / "manifest.json"
    @property
    def summary(self) -> Path: return self.root / "session_summary.json"
    @property
    def route_geojson(self) -> Path: return self.root / "route.geojson"
    @property
    def route_map(self) -> Path: return self.root / "route_map.html"
    @property
    def route_summary(self) -> Path: return self.root / "route_summary.json"
    @property
    def incomplete_marker(self) -> Path: return self.root / "INCOMPLETE"

    def create(self) -> None:
        for directory in (self.root, self.camera, self.pose,
                          self.imu, self.gps, self.health):
            directory.mkdir(parents=True, exist_ok=True)

    @classmethod
    def allocate(cls, storage_root: Path, when: datetime | None = None,
                 suffix: str = "") -> "SessionPaths":
        """Create a brand-new, never-reused session directory."""
        when = when or datetime.now()
        date_dir = storage_root / "sessions" / when.strftime("%Y-%m-%d")
        stamp = when.strftime("%Y%m%d_%H%M%S")
        base_id = f"session_{stamp}"
        if suffix:
            base_id = f"{base_id}_{suffix}"

        candidate = date_dir / base_id
        session_id = base_id
        # Never overwrite: if the directory somehow exists, take the next free
        # name instead of reusing it.
        n = 1
        while candidate.exists():
            session_id = f"{base_id}_{n:02d}"
            candidate = date_dir / session_id
            n += 1

        paths = cls(root=candidate, session_id=session_id)
        paths.create()
        return paths


class _BufferedFile:
    """Common flush/fsync bookkeeping for the writers below.

    ``flush()`` only moves bytes into the kernel page cache.  On the external
    USB/exFAT volume that is *not* durable: an abrupt disconnect during bench
    testing lost ~10 minutes of acquisition because the page cache was never
    written back (``Synchronize Cache(10) failed`` in dmesg, and the volume
    remounted read-only).  So the writers also fsync on an interval, which
    bounds worst-case loss to ``fsync_interval_s`` instead of the whole session.
    """

    def __init__(self, path: Path, mode: str, flush_interval_s: float = 5.0,
                 fsync_interval_s: float = 15.0):
        self._path = path
        self._flush_interval_ns = int(flush_interval_s * 1e9)
        self._fsync_interval_ns = (int(fsync_interval_s * 1e9)
                                   if fsync_interval_s > 0 else 0)
        self._last_flush = monotonic_ns()
        self._last_fsync = monotonic_ns()
        self.fsync_count = 0
        self._lock = threading.Lock()
        path.parent.mkdir(parents=True, exist_ok=True)
        self._fh = open(path, mode, buffering=1 << 16,
                        **({"newline": ""} if "b" not in mode else {}))
        self.rows_written = 0
        self.bytes_written = 0

    @property
    def path(self) -> Path:
        return self._path

    def _maybe_flush(self, force: bool = False) -> None:
        now = monotonic_ns()
        if force or (now - self._last_flush) >= self._flush_interval_ns:
            self._fh.flush()
            self._last_flush = now
            # Push the page cache to the device on a slower cadence so a power
            # cut or an unplugged SSD costs seconds, not the whole session.
            if (self._fsync_interval_ns
                    and now - self._last_fsync >= self._fsync_interval_ns):
                try:
                    os.fsync(self._fh.fileno())
                    self.fsync_count += 1
                except OSError:
                    pass          # device gone; the writer stays alive
                self._last_fsync = monotonic_ns()

    def flush(self) -> None:
        with self._lock:
            if not self._fh.closed:
                self._fh.flush()
                self._last_flush = monotonic_ns()

    def close(self, fsync: bool = True) -> None:
        """Flush, optionally fsync, then close.  Safe to call twice."""
        with self._lock:
            if self._fh.closed:
                return
            try:
                self._fh.flush()
                if fsync:
                    os.fsync(self._fh.fileno())
            except OSError:
                pass
            finally:
                self._fh.close()

    def __enter__(self): return self
    def __exit__(self, *exc): self.close()


class CsvWriter(_BufferedFile):
    """Append-only CSV with a fixed header, buffered and periodically flushed."""

    def __init__(self, path: Path, fieldnames: Sequence[str],
                 flush_interval_s: float = 5.0, fsync_interval_s: float = 15.0):
        super().__init__(path, "w", flush_interval_s, fsync_interval_s)
        self.fieldnames = list(fieldnames)
        self._writer = csv.DictWriter(self._fh, fieldnames=self.fieldnames,
                                      extrasaction="ignore", restval="")
        self._writer.writeheader()
        self._fh.flush()

    def write(self, row: dict[str, Any]) -> None:
        with self._lock:
            if self._fh.closed:
                return
            self._writer.writerow(row)
            self.rows_written += 1
            self._maybe_flush()

    def write_many(self, rows: Iterable[dict[str, Any]]) -> None:
        with self._lock:
            if self._fh.closed:
                return
            n = 0
            for row in rows:
                self._writer.writerow(row)
                n += 1
            self.rows_written += n
            self._maybe_flush()


class JsonlWriter(_BufferedFile):
    """Append-only newline-delimited JSON."""

    def __init__(self, path: Path, flush_interval_s: float = 5.0,
                 fsync_interval_s: float = 15.0):
        super().__init__(path, "w", flush_interval_s, fsync_interval_s)

    @staticmethod
    def _json_safe(obj: Any) -> Any:
        """Replace non-finite floats with null.

        ``json.dumps`` emits bare ``Infinity``/``NaN`` for these, which the JSON
        spec does not allow -- strict parsers reject the whole file.
        """
        import math
        if isinstance(obj, float):
            return obj if math.isfinite(obj) else None
        if isinstance(obj, dict):
            return {k: JsonlWriter._json_safe(v) for k, v in obj.items()}
        if isinstance(obj, (list, tuple)):
            return [JsonlWriter._json_safe(v) for v in obj]
        return obj

    def write(self, obj: Any) -> None:
        with self._lock:
            if self._fh.closed:
                return
            line = json.dumps(self._json_safe(obj), ensure_ascii=False,
                              separators=(",", ":"), allow_nan=False)
            self._fh.write(line + "\n")
            self.rows_written += 1
            self.bytes_written += len(line) + 1
            self._maybe_flush()


class RawChunkWriter:
    """Archive an unmodified byte stream as size-bounded numbered chunks.

    Used for the IMU serial stream so that a parser bug can never destroy the
    evidence: the raw bytes are always recoverable.

    ``chunk_bytes`` is a rollover threshold, not a hard split: a single write is
    never divided across two files, so a chunk may exceed it by up to the size
    of one write.  Concatenating the chunks in name order always reproduces the
    stream byte-for-byte.
    """

    def __init__(self, directory: Path, prefix: str, chunk_bytes: int = 1 << 20,
                 suffix: str = ".bin", flush_interval_s: float = 5.0):
        self._dir = directory
        self._prefix = prefix
        self._suffix = suffix
        self._chunk_bytes = max(int(chunk_bytes), 1 << 12)
        self._index = 0
        self._fh = None
        self._written = 0
        self._lock = threading.Lock()
        # Without a periodic flush an abrupt power loss would discard every byte
        # buffered since the last chunk boundary -- at a low baud rate that can
        # be the whole session.
        self._flush_interval_ns = int(flush_interval_s * 1e9)
        self._last_flush = monotonic_ns()
        self.total_bytes = 0
        self.chunks: list[str] = []
        directory.mkdir(parents=True, exist_ok=True)

    def _open_next(self) -> None:
        name = f"{self._prefix}_{self._index:03d}{self._suffix}"
        self._fh = open(self._dir / name, "wb", buffering=1 << 16)
        self.chunks.append(name)
        self._written = 0
        self._index += 1

    def write(self, data: bytes) -> None:
        if not data:
            return
        with self._lock:
            if self._fh is None:
                self._open_next()
            self._fh.write(data)
            self._written += len(data)
            self.total_bytes += len(data)
            if self._written >= self._chunk_bytes:
                # Guarded like the interval branch below: an SSD disconnect
                # landing on a rollover would otherwise raise out of write(),
                # out of ImuWorker.work(), and kill the IMU worker -- and leave
                # self._fh set to a handle that was never closed.
                try:
                    self._fh.flush()
                    os.fsync(self._fh.fileno())
                except OSError:
                    pass
                try:
                    self._fh.close()
                except OSError:
                    pass
                self._fh = None
                self._last_flush = monotonic_ns()
            elif monotonic_ns() - self._last_flush >= self._flush_interval_ns:
                self._fh.flush()
                try:
                    os.fsync(self._fh.fileno())
                except OSError:
                    pass
                self._last_flush = monotonic_ns()

    def close(self) -> None:
        with self._lock:
            if self._fh is not None and not self._fh.closed:
                try:
                    self._fh.flush()
                    os.fsync(self._fh.fileno())
                except OSError:
                    pass
                finally:
                    self._fh.close()
            self._fh = None
