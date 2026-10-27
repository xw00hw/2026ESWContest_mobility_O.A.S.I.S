"""Filesystem-mocked monitor publication; no real symlink or hardware."""
import unittest
from unittest.mock import MagicMock,patch
import occupied_trial as T


class MonitorLinkTests(unittest.TestCase):
    def test_only_exact_fixed_path_or_none_accepted(self):
        self.assertIsNone(T.validate_monitor_link(None))
        self.assertEqual(str(T.validate_monitor_link(T.MONITOR_LINK)),str(T.Path(T.MONITOR_LINK)))
        for value in ('live_control.jsonl','/tmp/control.jsonl',T.MONITOR_LINK+'/../x'):
            with self.assertRaises(ValueError):T.validate_monitor_link(value)

    def setup_paths(self):
        link=MagicMock(name='link');workspace=link.parent
        workspace.resolve.return_value=workspace
        link.exists.return_value=False
        out=MagicMock(name='out');target=(out/'control.jsonl').resolve.return_value
        target.is_file.return_value=True
        temporary=link.with_name.return_value;temporary.is_symlink.return_value=False
        return link,out,target,temporary

    def test_atomic_publish_only_after_existing_journal_validation(self):
        link,out,target,tmp=self.setup_paths()
        calls=MagicMock();calls.attach_mock(tmp.symlink_to,'symlink');calls.attach_mock(tmp.replace,'replace')
        with patch.object(T,'Path',return_value=out):T.publish_monitor_link('session_out',link)
        target.relative_to.assert_called_once_with(link.parent/'logs')
        target.is_file.assert_called_once()
        self.assertEqual([call[0] for call in calls.mock_calls],['symlink','replace'])
        tmp.symlink_to.assert_called_once_with(target);tmp.replace.assert_called_once_with(link)

    def test_other_directory_regular_file_and_failed_target_never_published(self):
        for mode in ('outside','regular_file','nonfile','symlink_parent'):
            link,out,target,tmp=self.setup_paths()
            if mode=='outside':target.relative_to.side_effect=ValueError('outside logs')
            elif mode=='regular_file':link.exists.return_value=True;link.is_symlink.return_value=False
            elif mode=='nonfile':target.is_file.return_value=False
            elif mode=='symlink_parent':link.parent.resolve.return_value=MagicMock()
            with self.subTest(mode=mode),patch.object(T,'Path',return_value=out):
                with self.assertRaises((ValueError,FileExistsError)):T.publish_monitor_link('out',link)
                tmp.symlink_to.assert_not_called();tmp.replace.assert_not_called()

    def test_disabled_monitor_does_no_filesystem_work(self):
        with patch.object(T,'Path',side_effect=AssertionError('no filesystem')):
            T.publish_monitor_link('unused',None)


if __name__=='__main__':unittest.main()
