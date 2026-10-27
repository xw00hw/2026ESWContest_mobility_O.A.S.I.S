"""Explicit launcher for the unchanged BeamNG collector plus a row observer.

Example: python -m fallback.beamng_bridge --collector-root PATH
The original collector remains the sole 127.0.0.1:64256 receiver and archive
writer. A nonblocking sender copies each new returned vehicle row to 64257.
This module does not change the installed source, shortcut, mod or game config.
Running this launcher intentionally opens the collector GUI; import does not.
"""
from __future__ import annotations

import argparse
import ast
import hashlib
import importlib
import inspect
import json
from pathlib import Path
import socket
import sys
import time

from .beamng import (BRIDGE_SCHEMA, BRIDGE_HOST, BRIDGE_PORT, CLOCK_ID,
                     MAX_DATAGRAM_BYTES)

COLLECTOR_FILES = ("collect.py", "desktop_app.py", "build_mod.py", "live_view.py", "dashboard.py")


def verify_collector(root):
    """Read-only source/API check before importing any collector code."""
    root = Path(root).expanduser().resolve()
    sources = {}
    for name in COLLECTOR_FILES:
        path = root/name
        if not path.is_file():
            raise ValueError("Missing collector source: " + str(path))
        sources[name] = path.read_bytes()
    tree = ast.parse(sources["collect.py"].decode("utf-8-sig"))
    metadata_version = False
    for node in ast.walk(tree):
        if isinstance(node, ast.Dict):
            literals = {k.value: v.value for k, v in zip(node.keys, node.values)
                        if isinstance(k, ast.Constant) and isinstance(v, ast.Constant)}
            if literals.get("collector_version") == "0.3" and literals.get("csv_revision") == 2 and literals.get("schema_version") == "imso-beamng/0.1":
                metadata_version = True
    session = next((n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "Session"), None)
    accept = next((n for n in session.body if isinstance(n, ast.FunctionDef) and n.name == "accept"), None) if session else None
    if not metadata_version or accept is None or [a.arg for a in accept.args.args] != ["self", "payload", "mono", "wall"]:
        raise ValueError("Expected collector v0.3/revision2 Session.accept API")
    desktop = ast.parse(sources["desktop_app.py"].decode("utf-8-sig"))
    names = {n.name for n in desktop.body if isinstance(n, ast.FunctionDef)}
    if not {"capture", "main"} <= names:
        raise ValueError("Expected collector desktop capture/main API")
    return {"collector_root": str(root), "collector_version": "0.3", "csv_revision": 2,
            "source_sha256": {name: hashlib.sha256(data).hexdigest() for name, data in sources.items()}}


def configuration_sha256(config):
    payload = json.dumps(config, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def make_packet(row, provenance, *, forwarded_ns=None):
    """Serialize one source row without editing its time or physical fields."""
    if not isinstance(row, dict) or any(str(k).startswith("_bridge_") for k in row):
        raise ValueError("Expected an unmodified collector row")
    forwarded_ns = time.perf_counter_ns() if forwarded_ns is None else forwarded_ns
    if type(forwarded_ns) is not int or forwarded_ns < 0:
        raise ValueError("Invalid forwarding timestamp")
    packet = {"schema": BRIDGE_SCHEMA, "collector_version": "0.3", "csv_revision": 2,
        "clock": CLOCK_ID, "forwarded_monotonic_ns": forwarded_ns,
        "provenance": provenance, "row": row}
    payload = json.dumps(packet, ensure_ascii=False, allow_nan=False, separators=(",", ":")).encode("utf-8")
    if len(payload) > MAX_DATAGRAM_BYTES:
        raise ValueError("Bridge datagram exceeds bounded size")
    return payload


class UDPForwarder:
    """Best-effort nonblocking observer. Errors never escape publish()."""
    def __init__(self, *, socket_factory=socket.socket, clock=time.perf_counter_ns):
        self.socket = socket_factory(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.socket.setblocking(False)
        except Exception:
            self.socket.close()
            raise
        self.clock = clock
        self.sent, self.dropped, self.last_error = 0, 0, None

    def publish(self, row, provenance):
        try:
            payload = make_packet(row, provenance, forwarded_ns=self.clock())
            sent = self.socket.sendto(payload, (BRIDGE_HOST, BRIDGE_PORT))
            if sent != len(payload):
                raise OSError("Incomplete UDP datagram send")
            self.sent += 1
            return True
        except Exception as exc:
            self.dropped += 1
            self.last_error = f"{type(exc).__name__}: {exc}"
            return False

    def close(self):
        self.socket.close()


class DisabledForwarder:
    """Archive remains usable if the optional observer socket cannot start."""
    def __init__(self, error):
        self.sent, self.dropped, self.last_error = 0, 0, str(error)

    def publish(self, row, provenance):
        self.dropped += 1
        return False

    def close(self):
        pass


def observed_session_class(base_session, forwarder, source_provenance):
    """Decorate only Session.accept; original archive/conversion stays first."""
    class ObservedSession(base_session):
        def accept(self, payload, mono=None, wall=None):
            row = super().accept(payload, mono, wall)
            if row is not None:
                try:
                    if not hasattr(self, "_fallback_provenance"):
                        self._fallback_provenance = dict(source_provenance,
                            session_id=str(Path(self.path).resolve()),
                            config_sha256=configuration_sha256(self.c))
                    forwarder.publish(row, self._fallback_provenance)
                except Exception as exc:
                    # An observer bug must not turn successful archival into a
                    # collector error or erase its original row return value.
                    self._fallback_observer_error = f"{type(exc).__name__}: {exc}"
            return row
    ObservedSession.__name__ = "FallbackObservedSession"
    return ObservedSession


def run_collector(root, *, publisher_factory=UDPForwarder, importer=importlib.import_module,
                  verifier=verify_collector):
    """Explicit GUI launcher; dependencies injectable for entirely offline tests."""
    provenance = verifier(root)
    root = Path(provenance["collector_root"]).resolve()
    names = tuple(Path(name).stem for name in COLLECTOR_FILES)
    for name in names:
        existing = sys.modules.get(name)
        if existing is not None and Path(getattr(existing, "__file__", "")).resolve().parent != root:
            raise RuntimeError("Conflicting imported module: " + name)
    old_path, old_bytecode = list(sys.path), sys.dont_write_bytecode
    module, original, forwarder = None, None, None
    try:
        sys.path.insert(0, str(root))
        # Importing/launching through this wrapper does not add/update pyc files
        # in the installed collector directory.
        sys.dont_write_bytecode = True
        module = importer("desktop_app")
        if Path(getattr(module, "__file__", "")).resolve() != root/"desktop_app.py":
            raise RuntimeError("Imported collector came from a different directory")
        original = module.Session
        if list(inspect.signature(original.accept).parameters) != ["self", "payload", "mono", "wall"] or not callable(module.main):
            raise RuntimeError("Collector runtime API differs from inspected v0.3 source")
        try:
            forwarder = publisher_factory()
        except Exception as exc:
            forwarder = DisabledForwarder(exc)
            print("Optional BeamNG observer unavailable; raw collector still starts: " + str(exc), file=sys.stderr)
        module.Session = observed_session_class(original, forwarder, provenance)
        module.main()
        return {"sent": forwarder.sent, "dropped": forwarder.dropped,
                "last_error": forwarder.last_error, "provenance": provenance}
    finally:
        if module is not None and original is not None:
            module.Session = original
        if forwarder is not None:
            try:
                forwarder.close()
            except Exception as exc:
                print("Optional observer close failed: " + str(exc), file=sys.stderr)
        sys.path[:] = old_path
        sys.dont_write_bytecode = old_bytecode


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--collector-root", required=True,
                        help="Existing BeamNG collector v0.3 directory; installed files are preserved")
    parser.add_argument("--check-only", action="store_true", help="Read source/API hashes only; no GUI or sockets")
    args = parser.parse_args(argv)
    report = verify_collector(args.collector_root) if args.check_only else run_collector(args.collector_root)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
