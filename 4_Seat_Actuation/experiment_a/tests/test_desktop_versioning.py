"""Offline release-to-recording integration; no device, socket or installed app.

Approved settings and release identities below exist only in temporary fixtures.
The fake workers report paths/metadata and never open UART or a BeamNG socket.
"""
import copy
from datetime import datetime
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from experiment_a.desktop_session import DesktopSession
from experiment_a.settings import Settings
from experiment_a.recording_lifecycle import pending_root


class DesktopVersioningTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.output_root = Path(self.temp.name)/"records"
        self.settings = Settings()
        self.settings.confirmations = {key: True for key in self.settings.confirmations}
        self.settings.metadata["parameter_selection"] = "offline_versioning_fixture_only"
        self.sessions = []
        self.addCleanup(self.stop_sessions)

    def stop_sessions(self):
        for session in self.sessions:
            session.request_stop("offline_test_cleanup")
        for session in self.sessions:
            if session.thread is not None:
                session.thread.join(3)
                self.assertFalse(session.is_running(), "Offline session did not stop")

    def profile(self, version="V1", fingerprint="a"*64):
        return {
            "output_root": str(self.output_root),
            "collector_root": "OFFLINE_UNUSED_COLLECTOR",
            "port": "OFFLINE_UNUSED_PORT",
            "release": {"version": version, "fingerprint": fingerprint,
                        "release_root": "offline fixture"},
            "labels": {"scenario": "offline_fixture_only"},
        }

    def run_session(self, version="V1", fingerprint="a"*64):
        observed = {}

        def capture(root, output, stop_event, notify):
            observed["capture_output"] = output
            notify("collector_session_open", {
                "path": str(output/"sessions"/"offline_fixture"),
                "config": {"scenario_label": "offline_fixture_only"},
            })
            stop_event.wait(3)
            notify("collector_done", {"error": None})

        def control(port, output, settings, **kwargs):
            observed["control_output"] = output
            observed["control_metadata"] = copy.deepcopy(kwargs["session_metadata"])
            observed["pressure_enabled"] = kwargs["record_pressure"]
            kwargs["stop_event"].set()
            return {"result": "offline_fixture_complete"}

        session = DesktopSession(self.profile(version, fingerprint), self.settings,
                                 capture=capture, control=control,
                                 port_probe=lambda requested: "OFFLINE_PORT")
        self.sessions.append(session)
        output = session.start()
        session.thread.join(3)
        self.assertFalse(session.is_running())
        self.assertEqual(session.snapshot()["phase"], "STOPPED")
        self.assertEqual(session.snapshot()['retention'],'pending')
        manifest = json.loads((output/"experiment.json").read_text(encoding="utf-8"))
        return output, manifest, observed

    def test_same_version_and_same_second_create_distinct_session_directories(self):
        with patch("experiment_a.desktop_session.datetime") as clock:
            clock.now.return_value = datetime(2026, 10, 26, 12, 34, 56)
            first, _, _ = self.run_session()
            second, _, _ = self.run_session()
        self.assertNotEqual(first, second)
        self.assertEqual(first.parent, pending_root(self.profile())/"V1")
        self.assertFalse(self.output_root.exists())
        self.assertEqual(second.parent, first.parent)
        for output in (first, second):
            self.assertRegex(output.name, r"^A_20261026_123456_[0-9a-f]{8}$")
            self.assertTrue((output/"experiment.json").is_file())

    def test_new_version_uses_separate_parent_and_preserves_previous_manifest(self):
        first, _, _ = self.run_session("V1", "a"*64)
        first=self.sessions[-1].save_recording()
        first_manifest = (first/"experiment.json").read_bytes()
        second, _, _ = self.run_session("V2", "b"*64)
        second=self.sessions[-1].save_recording()
        self.assertEqual(first.parent, self.output_root/"V1")
        self.assertEqual(second.parent, self.output_root/"V2")
        self.assertEqual((first/"experiment.json").read_bytes(), first_manifest)

    def test_version_and_fingerprint_reach_both_experiment_and_control_metadata(self):
        output, manifest, observed = self.run_session("V12", "c"*64)
        expected = self.profile("V12", "c"*64)["release"]
        self.assertEqual(manifest["code_version"], "V12")
        self.assertEqual(manifest["release"], expected)
        metadata = observed["control_metadata"]
        self.assertEqual(metadata["code_version"], "V12")
        self.assertEqual(metadata["release"], expected)
        self.assertEqual(metadata["integrated_session"], str(output))
        self.assertEqual(observed["capture_output"], output/"beamng")
        self.assertEqual(observed["control_output"], output/"control")
        self.assertIs(observed["pressure_enabled"], True)

    def test_existing_unversioned_and_versioned_raw_files_remain_byte_identical(self):
        legacy = self.output_root/"A_20261025_010203_legacy"/"raw"/"telemetry.jsonl"
        previous = self.output_root/"V1"/"A_20261026_000001_previous"/"control"/"pressure_raw.csv"
        before = {}
        for path, payload in ((legacy, b'{"fixture":"legacy raw"}\r\n'),
                              (previous, b"fixture,raw\r\nold,4095\r\n")):
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(payload)
            before[path] = (payload, path.stat().st_mtime_ns)
        self.run_session("V2", "b"*64)
        self.sessions[-1].save_recording()
        for path, (payload, timestamp) in before.items():
            self.assertEqual(path.read_bytes(), payload)
            self.assertEqual(path.stat().st_mtime_ns, timestamp)
        self.assertEqual(sorted(p.name for p in self.output_root.iterdir()),
                         ["A_20261025_010203_legacy", "V1", "V2"])

    def test_invalid_version_is_rejected_before_threads_paths_or_workers(self):
        worker = Mock(side_effect=AssertionError("An invalid release opened a worker"))
        for version in ("../V2", "V1/../../outside", "V1\\..\\outside", "V0", "V01", "v1", "", True):
            with self.subTest(version=version), patch("experiment_a.desktop_session.threading.Thread") as thread:
                with self.assertRaisesRegex(ValueError, "Invalid runtime code version"):
                    DesktopSession(self.profile(version), self.settings,
                                   capture=worker, control=worker, port_probe=worker)
                thread.assert_not_called()
        worker.assert_not_called()
        self.assertFalse(self.output_root.exists())

    def test_invalid_release_fingerprint_is_rejected_without_creating_output(self):
        worker = Mock(side_effect=AssertionError("An invalid release opened a worker"))
        for fingerprint in (None, "", "a"*63, "g"*64, "A"*64):
            with self.subTest(fingerprint=fingerprint):
                with self.assertRaisesRegex(ValueError, "Missing runtime release fingerprint"):
                    DesktopSession(self.profile("V1", fingerprint), self.settings,
                                   capture=worker, control=worker, port_probe=worker)
        worker.assert_not_called()
        self.assertFalse(self.output_root.exists())


if __name__ == "__main__":
    unittest.main()
