"""Hidden Tk layout checks; no device, worker, visible window, or shell launch."""
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from experiment_a.desktop import App, preview_policy_text, window_geometry
from experiment_a.settings import Settings


class DesktopLayoutTests(unittest.TestCase):
    def test_preview_description_uses_settings_without_claiming_vehicle_detection(self):
        settings=Settings()
        settings.steering_preview_max_yaw_deg=1.25
        text=preview_policy_text(settings)
        self.assertIn('기준 차량: Vivace 160 DCT',text)
        self.assertIn('최대 1.25°',text)
        self.assertIn('Pitch 0°',text)
        self.assertIn('실제 횡가속도 우선',text)
        self.assertIn('가속페달은 기록만',text)
        self.assertIn('목표각도는 설정 상한까지',text)
        self.assertIn('충돌 의심 시 모터 OFF, 차량·압력 기록 계속',text)
        self.assertNotIn('인식 완료',text)
        settings.steering_preview_enabled=False
        settings.metadata={}
        text=preview_policy_text(settings)
        self.assertIn('기준 차량: 미지정',text)
        self.assertIn('선제 움직임 꺼짐',text)
        self.assertNotIn('Vivace',text)

    def test_window_fits_logical_desktop_at_125_percent_scaling(self):
        for screen in ((1536,864),(1536,960),(1280,720),(1024,600)):
            with self.subTest(screen=screen):
                width,height,x,y=window_geometry(*screen)
                self.assertLessEqual(x+width,screen[0])
                self.assertLessEqual(y+height+60,screen[1])
                self.assertLessEqual(height,780)

    def test_stop_and_retention_buttons_stay_outside_scroll_area_at_large_font_scale(self):
        import tkinter as tk
        root=tk.Tk();root.withdraw();self.addCleanup(root.destroy)
        old_scale=root.tk.call('tk','scaling')
        self.addCleanup(lambda:root.tk.call('tk','scaling',old_scale))
        root.tk.call('tk','scaling',2.0)
        with tempfile.TemporaryDirectory() as folder:
            app=App(root,dict(output_root=folder,release=dict(version='DEV')),Settings(),preview=True)
            root.update_idletasks()
            self.assertEqual(app.footer.pack_info()['side'],'bottom')
            for name in ('stop_button','save_button','discard_button','reconnect_button'):
                button=getattr(app,name)
                self.assertEqual(button.master.master,app.footer)
                self.assertNotEqual(button.master.master,app.content_frame)
            self.assertTrue(app.content_canvas['yscrollcommand'])
            self.assertTrue(app.content_canvas['scrollregion'])
            self.assertLess(app.footer.winfo_reqheight(),200)
            self.assertLess(app.footer.winfo_reqwidth(),800)

    def test_open_folder_does_not_create_saved_version_directory(self):
        import tkinter as tk
        root=tk.Tk();root.withdraw();self.addCleanup(root.destroy)
        with tempfile.TemporaryDirectory() as folder:
            output=Path(folder)/'saved_records'
            app=App(root,dict(output_root=str(output),release=dict(version='V6',fingerprint='a'*64)),Settings(),preview=True)
            with patch('experiment_a.desktop.os.startfile') as startfile:app.open_output()
            self.assertFalse(output.exists())
            startfile.assert_called_once_with(str(Path(folder)))


if __name__=='__main__':unittest.main()
