"""Filesystem-only release tests. No installed app, network or UART is touched."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from experiment_a import releases


class ReleaseTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.source = self.root / "source"
        self.install = self.root / "install"
        self.files = {"experiment_a/__init__.py": b"# fixture\n",
                      "experiment_a/desktop.py": b"# desktop fixture\n",
                      releases.BASELINE_PATH: b'{"max_yaw_deg":25}\n'}
        self.files.update({name: b"# shared fixture\n" for name in releases.SHARED_FILES})
        for name, data in self.files.items():
            path = self.source / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(data)

    def publish(self):
        return releases.publish_release(self.source, self.install)

    def test_readonly_check_does_not_create_installation(self):
        result = releases.check_update(self.source, self.install)
        self.assertTrue(result["changed"])
        self.assertIsNone(result["installed_version"])
        self.assertIsNone(releases.get_current(self.install))
        self.assertFalse(self.install.exists())

    def test_publish_initial_release_and_reuse_identical_code(self):
        result = self.publish()
        self.assertEqual(result["version"], "V1")
        self.assertTrue(result["changed"])
        selected = releases.get_current(self.install)
        self.assertEqual(selected["version"], "V1")
        for name, data in self.files.items():
            self.assertEqual((Path(selected["release_root"]) / name).read_bytes(), data)
        pointer_before = (self.install / "current.json").read_bytes()
        manifest_before = (Path(selected["release_root"]) / "release.json").read_bytes()
        again = self.publish()
        self.assertFalse(again["changed"])
        self.assertEqual(again["version"], "V1")
        self.assertEqual((self.install / "current.json").read_bytes(), pointer_before)
        self.assertEqual((Path(selected["release_root"]) / "release.json").read_bytes(), manifest_before)

    def test_code_change_creates_v2_without_mutating_v1(self):
        first = self.publish()
        path = self.source / "experiment_a/desktop.py"
        path.write_text("# changed runtime\n", encoding="utf-8")
        before = releases.check_update(self.source, self.install)
        self.assertTrue(before["changed"])
        self.assertFalse(before["baseline_config_changed"])
        second = self.publish()
        self.assertEqual(second["version"], "V2")
        self.assertEqual((Path(first["release_root"]) / "experiment_a/desktop.py").read_bytes(),
                         self.files["experiment_a/desktop.py"])
        self.assertNotEqual(first["fingerprint"], second["fingerprint"])

    def test_settings_change_is_versioned_and_detected_separately(self):
        self.publish()
        (self.source / releases.BASELINE_PATH).write_text('{"max_yaw_deg":24}', encoding="utf-8")
        self.assertTrue(releases.check_update(self.source, self.install)["baseline_config_changed"])
        self.assertEqual(self.publish()["version"], "V2")

    def test_running_release_inspection_stays_on_original_after_update(self):
        first = self.publish()
        (self.source / "experiment_a/desktop.py").write_bytes(b"# changed")
        self.publish()
        self.assertEqual(releases.get_current(self.install)["version"], "V2")
        original = releases.inspect_release(first["release_root"])
        self.assertEqual(original["version"], "V1")
        self.assertEqual(original["fingerprint"], first["fingerprint"])

    def test_syntax_error_rejects_publish_without_executing_source(self):
        self.publish()
        (self.source / "experiment_a/desktop.py").write_bytes(b"def incomplete(:\n")
        with self.assertRaisesRegex(releases.ReleaseError, "Python syntax error"):
            self.publish()
        self.assertEqual(releases.get_current(self.install)["version"], "V1")
        (self.source / "experiment_a/desktop.py").write_bytes(
            b"raise RuntimeError('must never import release code while publishing')\n")
        self.assertEqual(self.publish()["version"], "V2")

    def test_docs_tests_and_bytecode_do_not_change_release(self):
        first = self.publish()
        for name in ("experiment_a/README.md", "experiment_a/tests/test_example.py",
                     "experiment_a/__pycache__/desktop.pyc", "fallback/unrelated.py"):
            path = self.source / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text("ignored fixture", encoding="utf-8")
        self.assertFalse(releases.check_update(self.source, self.install)["changed"])
        self.assertEqual(self.publish()["fingerprint"], first["fingerprint"])

    def test_added_and_removed_runtime_module_changes_version(self):
        self.publish()
        extra = self.source / "experiment_a/new_module.py"
        extra.write_text("# additional runtime\n", encoding="utf-8")
        self.assertEqual(self.publish()["version"], "V2")
        extra.unlink()
        self.assertEqual(self.publish()["version"], "V3")

    def test_modified_release_is_rejected_before_publishing(self):
        first = self.publish()
        (Path(first["release_root"]) / "experiment_a/desktop.py").write_bytes(b"modified")
        with self.assertRaisesRegex(releases.ReleaseError, "changed or are incomplete"):
            releases.get_current(self.install)
        with self.assertRaises(releases.ReleaseError):
            self.publish()
        self.assertEqual(json.loads((self.install / "current.json").read_text())["version"], "V1")

    def test_extra_runtime_python_in_snapshot_is_rejected(self):
        first = self.publish()
        (Path(first["release_root"]) / "experiment_a/unlisted.py").write_bytes(b"# unexpected")
        with self.assertRaisesRegex(releases.ReleaseError, "changed or are incomplete"):
            releases.get_current(self.install)

    def test_missing_source_file_does_not_modify_current(self):
        self.publish()
        before = (self.install / "current.json").read_bytes()
        (self.source / releases.SHARED_FILES[-1]).unlink()
        with self.assertRaises(releases.ReleaseError):
            self.publish()
        self.assertEqual((self.install / "current.json").read_bytes(), before)

    def test_failure_before_pointer_swap_preserves_current_and_old_files(self):
        first = self.publish()
        before = (self.install / "current.json").read_bytes()
        (self.source / "experiment_a/desktop.py").write_bytes(b"changed")
        with patch.object(releases.os, "replace", side_effect=OSError("offline injected disk error")):
            with self.assertRaisesRegex(OSError, "injected"):
                self.publish()
        self.assertEqual((self.install / "current.json").read_bytes(), before)
        self.assertEqual(releases.get_current(self.install)["version"], first["version"])
        self.assertEqual(list(self.install.glob(".current-*.json")), [])
        # Complete but unselected V2 is immutable; the next publication uses V3.
        self.assertEqual(self.publish()["version"], "V3")

    def test_source_edit_during_copy_is_rejected_and_stage_cleaned(self):
        self.publish()
        (self.source / "experiment_a/desktop.py").write_bytes(b"changed")
        original = releases._verify_release
        def edit_source(*args, **kwargs):
            result = original(*args, **kwargs)
            if Path(args[0]).name.startswith(".pending-"):
                (self.source / "experiment_a/desktop.py").write_bytes(b"changed again")
            return result
        with patch.object(releases, "_verify_release", side_effect=edit_source):
            with self.assertRaisesRegex(releases.ReleaseError, "Source changed"):
                self.publish()
        self.assertEqual(releases.get_current(self.install)["version"], "V1")
        self.assertEqual(list((self.install / "releases").glob(".pending-*")), [])

    def test_existing_numeric_version_is_never_overwritten(self):
        self.publish()
        occupied = self.install / "releases/V7"
        occupied.mkdir()
        (occupied / "do_not_touch.txt").write_text("existing data", encoding="utf-8")
        (self.source / "experiment_a/desktop.py").write_bytes(b"changed")
        self.assertEqual(self.publish()["version"], "V8")
        self.assertEqual((occupied / "do_not_touch.txt").read_text(), "existing data")

    def test_invalid_pointer_version_cannot_escape_release_directory(self):
        self.publish()
        pointer = self.install / "current.json"
        original = json.loads(pointer.read_text())
        for version in ("../outside", "V0", "V01", "V1/../V2", "v1", 1):
            pointer.write_text(json.dumps({**original, "version": version}), encoding="utf-8")
            with self.subTest(version=version), self.assertRaises(releases.ReleaseError):
                releases.get_current(self.install)

    def test_manifest_path_traversal_is_rejected(self):
        first = self.publish()
        path = Path(first["release_root"]) / "release.json"
        manifest = json.loads(path.read_text())
        manifest["files"]["../outside.py"] = "0" * 64
        path.write_text(json.dumps(manifest), encoding="utf-8")
        with self.assertRaisesRegex(releases.ReleaseError, "Invalid release path"):
            releases.get_current(self.install)

    def test_install_lock_rejects_parallel_publisher_and_releases_afterward(self):
        with releases._update_lock(self.install):
            with self.assertRaisesRegex(releases.ReleaseError, "in progress"):
                self.publish()
        self.assertEqual(self.publish()["version"], "V1")

    def test_invalid_baseline_and_duplicate_json_keys_are_rejected(self):
        (self.source / releases.BASELINE_PATH).write_text('{"x":1,"x":2}', encoding="utf-8")
        with self.assertRaisesRegex(releases.ReleaseError, "Duplicate JSON"):
            self.publish()
        self.assertFalse((self.install / "current.json").exists())


if __name__ == "__main__":
    unittest.main()
