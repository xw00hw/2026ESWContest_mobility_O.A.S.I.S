"""Supervised, archive-first adapter for the installed BeamNG v0.3 collector.

Importing this module opens no device, socket or GUI. ``run_capture`` owns one
collector worker until its stop event is set. The caller owns the motor worker
and must stop it when this worker ends or reports an error. Installed collector
sources, configuration, shortcuts and game mods are never written here.
"""
from __future__ import annotations

from contextlib import contextmanager
import copy
import importlib
import inspect
from pathlib import Path
import sys
import threading

from fallback.beamng_bridge import (COLLECTOR_FILES, UDPForwarder,
    configuration_sha256, observed_session_class, verify_collector)


class CollectorRuntimeError(RuntimeError):
    """The collector could not start, archive, forward or close normally."""


_IMPORT_LOCK = threading.Lock()


@contextmanager
def _installed_module(root, *, importer, verifier):
    """Verify paths/APIs before temporarily importing the installed collector.

    The imported legacy modules have generic names. Reject a different module
    with any such name, and remove only imports owned by this context on exit.
    A process can supervise one collector at a time. This lock does not claim
    ownership of UDP ports; the collector itself binds them exclusively.
    """
    if not _IMPORT_LOCK.acquire(blocking=False):
        raise CollectorRuntimeError("A collector adapter is already active")
    names = tuple(Path(name).stem for name in COLLECTOR_FILES)
    saved = {name: sys.modules.get(name) for name in names}
    imported = {}
    old_path, old_bytecode = list(sys.path), sys.dont_write_bytecode
    try:
        provenance = verifier(root)
        root = Path(provenance["collector_root"]).resolve()
        for name, existing in saved.items():
            if existing is not None and Path(getattr(existing, "__file__", "")).resolve().parent != root:
                raise CollectorRuntimeError("Conflicting imported module: " + name)
        try:
            sys.path.insert(0, str(root))
            sys.dont_write_bytecode = True
            module = importer("desktop_app")
            for name in names:
                candidate = sys.modules.get(name)
                if candidate is not None:
                    imported[name] = candidate
                    if Path(getattr(candidate, "__file__", "")).resolve() != root/(name+".py"):
                        raise CollectorRuntimeError("Collector import came from another path: " + name)
        finally:
            # Also account for helpers loaded before a partially failed import.
            for name in names:
                candidate = sys.modules.get(name)
                if (candidate is not None and saved[name] is None
                        and Path(getattr(candidate, "__file__", "")).resolve().parent == root):
                    imported[name] = candidate
            # Do not leave the installed directory on other workers' import path.
            sys.path[:] = old_path
            sys.dont_write_bytecode = old_bytecode
        if Path(getattr(module, "__file__", "")).resolve() != root/"desktop_app.py":
            raise CollectorRuntimeError("Imported desktop collector has an unexpected path")
        if (list(inspect.signature(module.capture).parameters) != ["config", "stop", "notify", "output"]
                or list(inspect.signature(module.Session.accept).parameters) != ["self", "payload", "mono", "wall"]
                or not callable(module.load_config)):
            raise CollectorRuntimeError("Collector runtime API differs from verified v0.3 source")
        yield module, root, provenance
    finally:
        sys.path[:] = old_path
        sys.dont_write_bytecode = old_bytecode
        for name, candidate in imported.items():
            if saved[name] is None and sys.modules.get(name) is candidate:
                del sys.modules[name]
        _IMPORT_LOCK.release()


def run_capture(collector_root, output_root, stop_event, notify, *,
                publisher_factory=UDPForwarder, importer=importlib.import_module,
                verifier=verify_collector):
    """Capture all original vehicle logs and forward rows to the A controller.

    ``output_root`` must not exist; the original collector creates its session
    below ``output_root/sessions/<UTC date>/<unique session>``. ``notify`` is a
    nonblocking callback, normally a queue writer, receiving ``(kind, dict)``:

    * ``collector_ready``: verified source/config, before the socket is bound.
    * ``collector_session_open``: socket bound and archive open; absolute path.
    * ``collector_status``: original live snapshot plus observer counters.
    * ``collector_done``: files closed, error/path/observer summary. This event
      also occurs for preparation errors. An error is raised after the event.

    The motor receiver may bind before this worker starts. Neither the ready
    nor open event is permission to move: the motor worker still qualifies
    actual fresh vehicle samples. This function never launches the legacy GUI,
    monitor, game or mod installer and never opens a serial port.
    """
    report = {"path": None, "error": None, "monitor_error": None,
              "sent": 0, "dropped": 0, "last_error": None}
    publisher = None
    seen_open = seen_done = False
    error = None
    try:
        if stop_event.is_set():
            report["stopped_before_start"] = True
            return report
        with _installed_module(collector_root, importer=importer, verifier=verifier) as (module, root, provenance):
            config = copy.deepcopy(module.load_config(root/"config.json"))
            output = Path(output_root).expanduser().resolve()
            output.mkdir(parents=True, exist_ok=False)
            publisher = publisher_factory()
            provenance = dict(provenance, config_sha256=configuration_sha256(config))
            report["provenance"] = provenance
            report["config"] = copy.deepcopy(config)
            notify("collector_ready", {"provenance": provenance, "config": copy.deepcopy(config),
                                        "output_root": str(output)})

            def observe(kind, payload):
                nonlocal seen_open, seen_done
                counters = {name: getattr(publisher, name, None)
                            for name in ("sent", "dropped", "last_error")}
                report.update(counters)
                if kind == "opened":
                    path = Path(payload).resolve()
                    if seen_open or not path.is_relative_to(output):
                        raise CollectorRuntimeError("Unexpected collector session path")
                    seen_open = True
                    report["path"] = str(path)
                    notify("collector_session_open", {"path": str(path), "provenance": provenance,
                                                       "config": copy.deepcopy(config)})
                elif kind == "status":
                    notify("collector_status", dict(payload, observer=counters))
                elif kind == "done":
                    seen_done = True
                    report.update({k: payload.get(k) for k in ("error", "monitor_error")})
                    if payload.get("path") is not None:
                        path = Path(payload["path"]).resolve()
                        if not path.is_relative_to(output):
                            raise CollectorRuntimeError("Unexpected collector completion path")
                        report["path"] = str(path)
                else:
                    raise CollectorRuntimeError("Unknown collector event: " + str(kind))

            original = module.Session
            try:
                module.Session = observed_session_class(original, publisher, provenance)
                module.capture(copy.deepcopy(config), stop_event, observe, output=str(output))
            finally:
                module.Session = original
            if report["error"]:
                raise CollectorRuntimeError(str(report["error"]))
            if not seen_done:
                raise CollectorRuntimeError("Collector returned without a completion event")
            if not stop_event.is_set():
                raise CollectorRuntimeError("Collector stopped unexpectedly")
    except Exception as exc:
        error = exc
        report["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        if publisher is not None:
            report.update({name: getattr(publisher, name, None)
                           for name in ("sent", "dropped", "last_error")})
            try:
                publisher.close()
            except Exception as exc:
                if error is None:
                    error = exc
                    report["error"] = f"Observer close failed: {type(exc).__name__}: {exc}"
        notify("collector_done", copy.deepcopy(report))
    if error is not None:
        raise CollectorRuntimeError(report["error"]) from error
    return report
