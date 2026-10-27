"""Read-only desktop telemetry; Live Expressions is the tuning authority."""
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from experiment_a.desktop import App, command_rows
from experiment_a.settings import Settings
from experiment_a.tests.test_tuning import SNAPSHOT


class DesktopTuningTests(unittest.TestCase):
    def test_command_rows_do_not_invent_telemetry(self):
        self.assertEqual(command_rows(None)[0],('좌측','—','—','—','—'))
        board=dict(pose_mdeg=(1000,250,-3000,0),target_mdeg=(2000,-100,0,0))
        self.assertEqual(command_rows(board)[0],('좌측','1.000°','0.250°','2.000°','-0.100°'))
        self.assertEqual(command_rows({'pose_mdeg':(0,)}),command_rows(None))

    def test_preview_never_starts_or_creates_configuration(self):
        import tkinter as tk
        root=tk.Tk();root.withdraw();self.addCleanup(root.destroy)
        settings=Settings();settings.confirmations={key:True for key in settings.confirmations}
        with tempfile.TemporaryDirectory() as folder:
            profile=dict(output_root=folder,settings_path=str(Path(folder)/'unused.json'),release=dict(version='DEV'))
            with patch('experiment_a.desktop.DesktopSession') as session:
                app=App(root,profile,settings,preview=True)
                app.start();root.update()
                session.assert_not_called();self.assertIsNone(app.session)
                self.assertFalse(hasattr(app,'tuning_vars'))
                self.assertEqual(str(app.setup_button['state']),'disabled')
                self.assertFalse(Path(profile['settings_path']).exists())
                values=[app.tuning_table.item(row,'values')[-1] for row in app.tuning_table.get_children()]
                self.assertEqual(values,['—']*6)

    def test_committed_firmware_values_are_shown_without_local_override(self):
        import tkinter as tk
        root=tk.Tk();root.withdraw();self.addCleanup(root.destroy)
        profile=dict(output_root='unused',release=dict(version='DEV'))
        app=App(root,profile,Settings(),preview=True)
        app.session=Mock()
        app.session.snapshot.return_value=dict(phase='WAIT_VEHICLE',message='fixture',finished=False,
            motor=dict(firmware_tuning=dict(SNAPSHOT,yaw_max_mdeg=12000,revision=9)))
        app.poll();root.update()
        values=[app.tuning_table.item(row,'values')[-1] for row in app.tuning_table.get_children()]
        self.assertEqual(values[0],'12 °')
        self.assertIn('9',app.tuning_status.get())
        self.assertEqual(app.settings.max_yaw_deg,15)


if __name__=='__main__':unittest.main()
