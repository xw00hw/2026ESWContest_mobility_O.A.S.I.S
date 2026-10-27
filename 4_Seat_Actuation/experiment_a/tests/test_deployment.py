"""Temporary-directory installer/bootstrap tests; no real desktop or hardware."""
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from experiment_a.bootstrap import application_lock, resolve_release
from experiment_a.deployment import install
from experiment_a.releases import BASELINE_PATH, SHARED_FILES, get_current


@unittest.skipUnless(os.name == "nt", "Desktop application lock uses Windows msvcrt")
class DeploymentTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.source = self.root / "source"
        self.destination = self.root / "installed"
        self.output = self.root / "data"
        self.collector = self.root / "collector"
        self.files = {
            "experiment_a/__init__.py": b"# fixture package\n",
            "experiment_a/desktop.py": b"# fixture desktop\n",
            "experiment_a/bootstrap.py": b"# fixture stable launcher\n",
            BASELINE_PATH: b'{"max_yaw_deg":25,"confirmations":{"trial_settings_reviewed":false}}\n',
            **{name: b"# fixture shared dependency\n" for name in SHARED_FILES}}
        for name, data in self.files.items():
            path = self.source / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(data)

    def deploy(self, **overrides):
        options = dict(source_root=self.source, install_root=self.destination,
                       collector_root=self.collector, output_root=self.output)
        options.update(overrides)
        return install(**options)

    def test_first_install_creates_v1_and_same_content_reuses_v1(self):
        first = self.deploy()
        self.assertEqual(first["version"], "V1")
        self.assertTrue(first["changed"])
        self.assertTrue((self.output / "V1").is_dir())
        self.assertEqual((self.destination / "trial_settings.json").read_bytes(),
                         self.files[BASELINE_PATH])
        pointer = (self.destination / "current.json").read_bytes()
        manifest = (self.destination / "releases/V1/release.json").read_bytes()
        again = self.deploy()
        self.assertEqual(again["version"], "V1")
        self.assertFalse(again["changed"])
        self.assertEqual(again["bootstrap"], first["bootstrap"])
        self.assertEqual((self.destination / "current.json").read_bytes(), pointer)
        self.assertEqual((self.destination / "releases/V1/release.json").read_bytes(), manifest)
        self.assertFalse((self.destination / "releases/V2").exists())

    def test_update_preserves_exact_trial_settings_and_custom_profile_values(self):
        self.deploy()
        settings = b'{\n  "max_yaw_deg": 12, "metadata": {"note": "custom fixture"}\n}\n'
        (self.destination / "trial_settings.json").write_bytes(settings)
        profile_path = self.destination / "desktop_profile.json"
        profile = json.loads(profile_path.read_text(encoding="utf-8"))
        custom_output = self.root / "custom-data"
        custom_collector = self.root / "custom-collector"
        custom_labels = {"participant": "fixture-person", "scenario": "S23", "extra": "retain"}
        profile.update(output_root=str(custom_output), collector_root=str(custom_collector),
                       port="COM123", labels=custom_labels, user_note="preserve unknown field")
        profile_path.write_text(json.dumps(profile, ensure_ascii=False), encoding="utf-8")
        (self.source / "experiment_a/desktop.py").write_bytes(b"# changed runtime\n")
        # Changed candidate defaults must not silently replace operator settings.
        (self.source / BASELINE_PATH).write_bytes(b'{"max_yaw_deg":24}\n')
        result = self.deploy(output_root=self.root / "ignored-new-default",
                             collector_root=self.root / "ignored-collector-default")
        actual = json.loads(profile_path.read_text(encoding="utf-8"))
        self.assertEqual(result["version"], "V2")
        self.assertEqual((self.destination / "trial_settings.json").read_bytes(), settings)
        for key in ("port", "labels", "output_root", "collector_root", "user_note"):
            self.assertEqual(actual[key], profile[key])
        self.assertEqual(result["output"], str(custom_output / "V2"))
        self.assertTrue((custom_output / "V2").is_dir())
        self.assertFalse((self.output / "V2").exists())

    def test_code_update_keeps_old_release_and_old_data_unchanged(self):
        first = self.deploy()
        old_code = self.destination / "releases/V1/experiment_a/desktop.py"
        old_manifest = self.destination / "releases/V1/release.json"
        before = old_manifest.read_bytes()
        historical_data = self.output / "V1/previous_session.csv"
        historical_data.write_bytes(b"row,unchanged\n")
        (self.source / "experiment_a/desktop.py").write_bytes(b"# version two fixture\n")
        second = self.deploy()
        self.assertEqual(second["version"], "V2")
        self.assertEqual(second["bootstrap"], first["bootstrap"])
        self.assertEqual(old_code.read_bytes(), self.files["experiment_a/desktop.py"])
        self.assertEqual(old_manifest.read_bytes(), before)
        self.assertEqual(historical_data.read_bytes(), b"row,unchanged\n")
        self.assertEqual(get_current(self.destination)["version"], "V2")

    def test_explicit_settings_reset_backs_up_previous_values_and_removes_body_label(self):
        self.deploy()
        settings_path=self.destination/'trial_settings.json'
        old=b'{"max_yaw_deg":25,"custom":"operator old values"}\n'
        settings_path.write_bytes(old)
        candidate=b'{"max_yaw_deg":12,"confirmations":{"trial_settings_reviewed":false}}\n'
        (self.source/BASELINE_PATH).write_bytes(candidate)
        result=self.deploy(reset_settings=True)
        self.assertEqual(settings_path.read_bytes(),candidate)
        self.assertEqual(Path(result['settings_backup']).read_bytes(),old)
        profile=json.loads((self.destination/'desktop_profile.json').read_text(encoding='utf-8'))
        self.assertEqual(profile['labels']['participant'],'unpersonalized')
        self.assertEqual(result['version'],'V2')

    def test_failed_release_reset_restores_operator_settings_and_pointer(self):
        self.deploy()
        settings_path=self.destination/'trial_settings.json'
        old=b'{"operator":"preserve byte-for-byte"}\n'
        settings_path.write_bytes(old)
        pointer=(self.destination/'current.json').read_bytes()
        with patch('experiment_a.deployment.publish_release',side_effect=RuntimeError('offline injected failure')):
            with self.assertRaisesRegex(RuntimeError,'injected failure'):
                self.deploy(reset_settings=True)
        self.assertEqual(settings_path.read_bytes(),old)
        self.assertEqual((self.destination/'current.json').read_bytes(),pointer)

    def test_reset_rejects_candidate_with_positive_hardware_confirmations(self):
        self.deploy()
        settings_path=self.destination/'trial_settings.json'
        before=settings_path.read_bytes()
        (self.source/BASELINE_PATH).write_bytes(b'{"confirmations":{"trial_settings_reviewed":true}}')
        with self.assertRaisesRegex(ValueError,'positive hardware confirmations'):
            self.deploy(reset_settings=True)
        self.assertEqual(settings_path.read_bytes(),before)

    def test_held_app_lock_blocks_first_install_before_settings_profile_and_current(self):
        lock = self.destination / "desktop_profile.lock"
        with application_lock(lock):
            with self.assertRaisesRegex(RuntimeError, "이미 실행"):
                self.deploy()
        for name in ("trial_settings.json", "desktop_profile.json", "current.json", "bootstrap.py"):
            self.assertFalse((self.destination / name).exists(), name)
        self.assertFalse(self.output.exists())
        self.assertEqual(self.deploy()["version"], "V1")

    def test_held_app_lock_blocks_update_without_mutating_existing_install(self):
        self.deploy()
        paths = [self.destination / name for name in
                 ("trial_settings.json", "desktop_profile.json", "current.json", "bootstrap.py")]
        snapshots = {path: path.read_bytes() for path in paths}
        (self.source / "experiment_a/desktop.py").write_bytes(b"# pending edit\n")
        with application_lock(self.destination / "desktop_profile.lock"):
            with self.assertRaisesRegex(RuntimeError, "이미 실행"):
                self.deploy()
            for path, data in snapshots.items():
                self.assertEqual(path.read_bytes(), data)
        self.assertFalse((self.destination / "releases/V2").exists())

    def test_bootstrap_resolves_verified_current_release_and_rejects_tampering(self):
        self.deploy()
        self.assertEqual(resolve_release(self.destination),
                         (self.destination / "releases/V1").resolve())
        runtime = self.destination / "releases/V1/experiment_a/desktop.py"
        runtime.write_bytes(b"# unexpected installed edit\n")
        with self.assertRaisesRegex(ValueError, "Installed code changed"):
            resolve_release(self.destination)

    def test_bootstrap_rejects_missing_file_and_unsafe_pointer(self):
        self.deploy()
        runtime = self.destination / "releases/V1/experiment_a/desktop.py"
        runtime.unlink()
        with self.assertRaises(ValueError):
            resolve_release(self.destination)
        runtime.write_bytes(self.files["experiment_a/desktop.py"])
        pointer = self.destination / "current.json"
        metadata = json.loads(pointer.read_text(encoding="utf-8"))
        metadata["version"] = "../outside"
        pointer.write_text(json.dumps(metadata), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "Invalid installed version"):
            resolve_release(self.destination)


if __name__ == "__main__":
    unittest.main()
