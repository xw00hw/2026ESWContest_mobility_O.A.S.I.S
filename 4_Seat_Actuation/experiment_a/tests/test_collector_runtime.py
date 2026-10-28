"""Offline collector supervision tests; no UDP, serial port or GUI is opened."""
import json
from pathlib import Path
import sys
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from experiment_a.collector_runtime import CollectorRuntimeError, run_capture


class Publisher:
    def __init__(self, trace, *, close_error=False):
        self.trace = trace
        self.sent = self.dropped = 0
        self.last_error = None
        self.closed = False
        self.close_error = close_error

    def publish(self, row, provenance):
        self.trace.append(("forward", dict(row), provenance))
        self.sent += 1

    def close(self):
        self.closed = True
        if self.close_error:
            raise OSError("close test")


class CollectorRuntimeTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)/"collector"
        self.root.mkdir()
        self.output = Path(self.temp.name)/"run"/"beamng"
        self.config = {"host": "127.0.0.1", "port": 64256,
                       "groups": ["motion"], "vehicle_label": "car_A"}
        (self.root/"config.json").write_text(json.dumps(self.config), encoding="utf-8")
        self.stop = threading.Event()
        self.events = []
        self.trace = []
        self.publisher = Publisher(self.trace)
        root = self.root
        trace = self.trace

        class Session:
            def __init__(self, config, output=None):
                self.c = config
                self.path = Path(output)/"sessions"/"2026-01-01"/"session_unique"
                self.path.mkdir(parents=True)

            def accept(self, payload, mono=None, wall=None):
                row = {"host_monotonic_ns": mono, "host_wall_time_ns": wall,
                       "sequence_id": payload}
                trace.append(("archive", row))
                return row

        def capture(config, stop, notify, output=None):
            session = self.module.Session(config, output)
            notify("opened", str(session.path))
            session.accept(1, 123456789, 987654321)
            notify("status", {"live": True, "rows": 1, "path": str(session.path)})
            stop.set()
            trace.append(("archive_closed",))
            notify("done", {"error": None, "path": str(session.path), "monitor_error": None})

        def load_config(path):
            return json.loads(Path(path).read_text(encoding="utf-8"))

        self.module = SimpleNamespace(__file__=str(root/"desktop_app.py"),
                                      Session=Session, capture=capture, load_config=load_config)
        self.original_session = Session

    def run_adapter(self, **overrides):
        kw = dict(publisher_factory=lambda: self.publisher,
                  importer=lambda name: self.module,
                  verifier=lambda root: {"collector_root": str(Path(root).resolve()),
                                          "collector_version": "0.3", "csv_revision": 2,
                                          "source_sha256": {"collect.py": "test-source-hash"}})
        kw.update(overrides)
        return run_capture(self.root, self.output, self.stop,
                           lambda kind, value: self.events.append((kind, value)), **kw)

    def test_archive_first_forward_preserves_clocks_and_session_provenance(self):
        before = (self.root/"config.json").read_bytes()
        report = self.run_adapter()
        self.assertEqual([item[0] for item in self.trace], ["archive", "forward", "archive_closed"])
        self.assertEqual(self.trace[0][1], self.trace[1][1])
        self.assertEqual(self.trace[1][2]["session_id"], report["path"])
        self.assertEqual(self.trace[1][2]["source_sha256"], {"collect.py": "test-source-hash"})
        self.assertEqual(len(self.trace[1][2]["config_sha256"]), 64)
        self.assertEqual([kind for kind, _ in self.events], ["collector_ready", "collector_session_open",
                                                          "collector_status", "collector_done"])
        self.assertEqual(self.events[2][1]["observer"]["sent"], 1)
        self.assertTrue(self.publisher.closed)
        self.assertIs(self.module.Session, self.original_session)
        self.assertEqual((self.root/"config.json").read_bytes(), before)
        self.assertFalse((self.root/"__pycache__").exists())

    def test_capture_bind_error_reports_done_and_raises(self):
        def capture(config, stop, notify, output=None):
            notify("done", {"error": "OSError: address already in use", "path": None})
        self.module.capture = capture
        with self.assertRaisesRegex(CollectorRuntimeError, "address already in use"):
            self.run_adapter()
        self.assertEqual(self.events[-1][0], "collector_done")
        self.assertIsNone(self.events[-1][1]["path"])
        self.assertTrue(self.publisher.closed)
        self.assertIs(self.module.Session, self.original_session)

    def test_unexpected_capture_completion_is_not_a_success(self):
        def capture(config, stop, notify, output=None):
            notify("done", {"error": None, "path": None})
        self.module.capture = capture
        with self.assertRaisesRegex(CollectorRuntimeError, "stopped unexpectedly"):
            self.run_adapter()

    def test_missing_completion_event_is_a_failure(self):
        def capture(config, stop, notify, output=None):
            stop.set()
        self.module.capture = capture
        with self.assertRaisesRegex(CollectorRuntimeError, "without a completion"):
            self.run_adapter()

    def test_output_directory_must_be_new(self):
        self.output.mkdir(parents=True)
        marker = self.output/"preserve.txt"
        marker.write_text("original", encoding="utf-8")
        with self.assertRaises(CollectorRuntimeError):
            self.run_adapter()
        self.assertEqual(marker.read_text(encoding="utf-8"), "original")
        self.assertFalse(self.publisher.closed)  # Publisher factory was not called.
        self.assertEqual([kind for kind, _ in self.events], ["collector_done"])

    def test_unexpected_archive_path_restores_patch_and_closes_observer(self):
        def capture(config, stop, notify, output=None):
            notify("opened", str(self.root))
        self.module.capture = capture
        with self.assertRaisesRegex(CollectorRuntimeError, "Unexpected collector session path"):
            self.run_adapter()
        self.assertTrue(self.publisher.closed)
        self.assertIs(self.module.Session, self.original_session)

    def test_verified_import_collision_is_rejected_before_publisher_or_output(self):
        collision = SimpleNamespace(__file__=str(Path(self.temp.name)/"other"/"collect.py"))
        with patch.dict(sys.modules, {"collect": collision}):
            with self.assertRaisesRegex(CollectorRuntimeError, "Conflicting imported module: collect"):
                self.run_adapter()
            self.assertIs(sys.modules["collect"], collision)
        self.assertFalse(self.output.exists())

    def test_new_imports_and_globals_restored_after_success(self):
        old_path, old_bytecode = list(sys.path), sys.dont_write_bytecode
        fake_helper = SimpleNamespace(__file__=str(self.root/"collect.py"))
        def importer(name):
            self.assertEqual(sys.path[0], str(self.root))
            self.assertTrue(sys.dont_write_bytecode)
            sys.modules["desktop_app"] = self.module
            sys.modules["collect"] = fake_helper
            return self.module
        with patch.dict(sys.modules):
            sys.modules.pop("desktop_app", None)
            sys.modules.pop("collect", None)
            self.run_adapter(importer=importer)
            self.assertNotIn("desktop_app", sys.modules)
            self.assertNotIn("collect", sys.modules)
        self.assertEqual(sys.path, old_path)
        self.assertEqual(sys.dont_write_bytecode, old_bytecode)

    def test_partial_import_is_cleaned_and_existing_same_path_module_preserved(self):
        old_path, old_bytecode = list(sys.path), sys.dont_write_bytecode
        helper = SimpleNamespace(__file__=str(self.root/"collect.py"))
        def importer(name):
            sys.modules["collect"] = helper
            raise ImportError("partial-import-test")
        with patch.dict(sys.modules):
            sys.modules.pop("collect", None)
            sys.modules["desktop_app"] = self.module
            with self.assertRaisesRegex(CollectorRuntimeError, "partial-import-test"):
                self.run_adapter(importer=importer)
            self.assertNotIn("collect", sys.modules)
            self.assertIs(sys.modules["desktop_app"], self.module)
        self.assertEqual(sys.path, old_path)
        self.assertEqual(sys.dont_write_bytecode, old_bytecode)

    def test_close_failure_is_reported_and_not_silently_accepted(self):
        self.publisher.close_error = True
        with self.assertRaisesRegex(CollectorRuntimeError, "Observer close failed"):
            self.run_adapter()
        self.assertIn("Observer close failed", self.events[-1][1]["error"])

    def test_stop_before_start_does_not_import_or_create_resources(self):
        self.stop.set()
        def forbidden(*args):
            raise AssertionError("Unexpected resource preparation")
        report = self.run_adapter(verifier=forbidden, importer=forbidden, publisher_factory=forbidden)
        self.assertTrue(report["stopped_before_start"])
        self.assertFalse(self.output.exists())
        self.assertEqual([kind for kind, _ in self.events], ["collector_done"])


if __name__ == "__main__":
    unittest.main()
