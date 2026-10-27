"""Temporary record ownership, explicit retention, crash recovery and relocation."""
import errno
import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch

from experiment_a import recording_lifecycle as lifecycle
from experiment_a.recording_lifecycle import PendingRecording, RecordingError, discover_pending, resolve_recording_path


class RecordingLifecycleTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name)
        self.profile=dict(output_root=str(self.root/'saved'),install_root=str(self.root/'installed'))
        self.records=[];self.addCleanup(self.close_leases)

    def close_leases(self):
        for item in self.records:
            if item._active_lease is not None:item._active_lease.close();item._active_lease=None

    def record(self,finished=True):
        identifier=f'A_20261027_120000_{len(self.records)+1:08x}'
        item=PendingRecording(self.profile,'V6',identifier,create=True);self.records.append(item)
        (item.staging_path/'control').mkdir()
        raw=item.staging_path/'control/raw.jsonl'
        raw.write_bytes(b'{"raw":"original","nested_path":"untouched"}\r\n')
        if finished:item.mark_finished()
        return item,raw

    def test_stop_leaves_pending_outside_saved_tree_and_discovery_does_not_save(self):
        item,raw=self.record()
        self.assertFalse(Path(self.profile['output_root']).exists())
        found=discover_pending(self.profile)
        self.assertEqual(len(found),1)
        self.assertEqual(found[0].snapshot()['retention'],'pending')
        self.assertTrue(found[0].snapshot()['finished'])
        self.assertFalse(found[0].snapshot()['interrupted'])
        self.assertTrue(raw.exists())
        self.assertFalse(Path(self.profile['output_root']).exists())

    def test_active_records_cannot_save_discard_or_be_recovered(self):
        item,raw=self.record(finished=False)
        for action in (item.save_recording,item.discard_recording):
            with self.assertRaisesRegex(RecordingError,'Stop recording'):action()
        self.assertEqual(discover_pending(self.profile),[])
        self.assertTrue(raw.exists())
        self.assertFalse(Path(self.profile['output_root']).exists())

    def test_finish_validation_failure_still_releases_active_lease_for_recovery(self):
        item,raw=self.record(finished=False)
        with patch.object(item,'_validate',side_effect=RecordingError('offline finish validation failure')):
            with self.assertRaisesRegex(RecordingError,'finish validation'):item.mark_finished()
        self.assertIsNone(item._active_lease)
        found=discover_pending(self.profile)
        self.assertEqual(len(found),1)
        self.assertTrue(found[0].interrupted)
        self.assertTrue(raw.exists())

    def test_explicit_save_promotes_without_changing_raw_bytes_and_resolves_references(self):
        item,raw=self.record();before=raw.read_bytes();original=str(raw)
        saved=item.save_recording()
        self.assertEqual(saved,item.intended_output)
        self.assertEqual(item.snapshot()['retention'],'saved')
        self.assertIn('저장 완료',item.snapshot()['message'])
        self.assertNotIn('선택하세요',item.snapshot()['message'])
        self.assertFalse(item.staging_path.exists())
        resolved=resolve_recording_path(saved,original)
        self.assertEqual(resolved,saved/'control/raw.jsonl')
        self.assertEqual(resolved.read_bytes(),before)
        self.assertEqual(discover_pending(self.profile),[])
        marker=json.loads((saved/lifecycle.MARKER).read_text(encoding='utf-8'))
        self.assertEqual(marker['retention'],'saved')
        with self.assertRaises(RecordingError):resolve_recording_path(saved,self.root/'outside')
        with self.assertRaises(RecordingError):resolve_recording_path(saved,'../outside')

    def test_discard_removes_only_selected_pending_and_preserves_other_and_old_data(self):
        item,raw=self.record();other,other_raw=self.record()
        legacy=self.root/'saved/V5/original.bin';legacy.parent.mkdir(parents=True);legacy.write_bytes(b'old')
        item.discard_recording()
        self.assertFalse(item.staging_path.exists());self.assertTrue(other_raw.exists())
        self.assertEqual(legacy.read_bytes(),b'old')
        self.assertEqual(item.snapshot()['retention'],'discarded')
        self.assertIn('버렸습니다',item.snapshot()['message'])
        self.assertIsNone(item.snapshot()['output'])

    def test_save_collision_preserves_existing_target_and_pending(self):
        item,raw=self.record();before=raw.read_bytes()
        item.intended_output.mkdir(parents=True)
        existing=item.intended_output/'important.bin';existing.write_bytes(b'do not overwrite')
        with self.assertRaisesRegex(RecordingError,'already exists'):item.save_recording()
        self.assertEqual(existing.read_bytes(),b'do not overwrite')
        self.assertEqual(raw.read_bytes(),before)
        self.assertEqual(item.retention,'pending')

    def test_failed_save_keeps_pending_recoverable(self):
        item,raw=self.record();before=raw.read_bytes()
        with patch.object(Path,'rename',side_effect=PermissionError('offline failure')):
            with self.assertRaises(PermissionError):item.save_recording()
        self.assertEqual(raw.read_bytes(),before)
        self.assertEqual(item.retention,'pending')
        self.assertEqual(len(discover_pending(self.profile)),1)
        self.assertFalse(item.intended_output.exists())

    def test_cross_volume_copy_is_verified_before_promotion(self):
        item,raw=self.record();before=raw.read_bytes();rename=Path.rename
        def across(path,target):
            if path==item.staging_path:raise OSError(errno.EXDEV,'offline cross-volume fixture')
            return rename(path,target)
        with patch.object(Path,'rename',across):saved=item.save_recording()
        self.assertEqual((saved/'control/raw.jsonl').read_bytes(),before)
        self.assertFalse(item.staging_path.exists())
        self.assertEqual(list(saved.parent.glob('.pending_save_*')),[])

    def test_failed_cross_volume_verification_never_promotes_or_deletes_source(self):
        item,raw=self.record();before=raw.read_bytes();rename=Path.rename;copytree=lifecycle.shutil.copytree
        def across(path,target):
            if path==item.staging_path:raise OSError(errno.EXDEV,'offline fixture')
            return rename(path,target)
        def corrupt(source,target,*args,**kwargs):
            result=copytree(source,target,*args,**kwargs)
            if Path(source)==item.staging_path:(Path(target)/'control/raw.jsonl').write_bytes(b'corrupted')
            return result
        with patch.object(Path,'rename',across),patch.object(lifecycle.shutil,'copytree',corrupt):
            with self.assertRaisesRegex(RecordingError,'verification failed'):item.save_recording()
        self.assertEqual(raw.read_bytes(),before)
        self.assertFalse(item.intended_output.exists())
        self.assertEqual(list(item.intended_output.parent.glob('.pending_save_*')),[])

    def test_abandoned_unfinished_record_is_recoverable_without_hardware_or_auto_save(self):
        item,raw=self.record(finished=False)
        item._active_lease.close();item._active_lease=None  # Simulate process death, OS releases lease.
        recovered=discover_pending(self.profile)[0]
        self.assertTrue(recovered.snapshot()['interrupted'])
        self.assertTrue(recovered.snapshot()['finished'])
        self.assertFalse(Path(self.profile['output_root']).exists())
        saved=recovered.save_recording()
        self.assertTrue((saved/'control/raw.jsonl').exists())

    def test_ownership_marker_tamper_never_deletes_or_moves(self):
        item,raw=self.record()
        marker=item.staging_path/lifecycle.MARKER
        metadata=json.loads(marker.read_text());metadata['intended_output']=str(self.root/'unrelated')
        marker.write_text(json.dumps(metadata))
        for action in (item.save_recording,item.discard_recording):
            with self.assertRaisesRegex(RecordingError,'ownership marker'):action()
        self.assertTrue(raw.exists())
        self.assertFalse(Path(self.profile['output_root']).exists())

    def test_reparse_child_is_rejected_before_any_recursive_delete_or_move(self):
        item,raw=self.record();real=lifecycle._reparse
        # Exercise Windows junction detection without requiring privileged creation.
        with patch.object(lifecycle,'_reparse',side_effect=lambda path:path==raw or real(path)):
            for action in (item.save_recording,item.discard_recording):
                with self.assertRaisesRegex(RecordingError,'Reparse/symlink'):action()
        self.assertTrue(raw.exists())
        self.assertFalse(Path(self.profile['output_root']).exists())

    def test_save_discard_race_has_one_winner_and_no_partial_record(self):
        item,raw=self.record();barrier=threading.Barrier(2);outcomes=[]
        def perform(name,action):
            barrier.wait()
            try:action();outcomes.append(name)
            except RecordingError:outcomes.append('rejected')
        threads=[threading.Thread(target=perform,args=('saved',item.save_recording)),
                 threading.Thread(target=perform,args=('discarded',item.discard_recording))]
        for thread in threads:thread.start()
        for thread in threads:thread.join(3);self.assertFalse(thread.is_alive())
        self.assertEqual(outcomes.count('rejected'),1)
        self.assertFalse(item.staging_path.exists())
        self.assertEqual(item.intended_output.exists(),item.retention=='saved')


if __name__=='__main__':unittest.main()
