"""Explicit reconnection and truthful ended-state UI, with offline workers only."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from experiment_a.desktop import App, vehicle_status_text
from experiment_a.desktop_session import DesktopSession
from experiment_a.settings import Settings


class DesktopReconnectTests(unittest.TestCase):
    def setUp(self):
        import tkinter as tk
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=tk.Tk();self.root.withdraw();self.addCleanup(self.close_root)
        self.profile=dict(output_root=str(Path(self.temp.name)/'records'),
                          settings_path=str(Path(self.temp.name)/'settings.json'),
                          collector_root='OFFLINE_UNUSED',port='OFFLINE_ONLY',
                          release=dict(version='DEV'))
        settings=Settings()
        settings.confirmations={key:True for key in settings.confirmations}
        Path(self.profile['settings_path']).write_text(json.dumps(settings.to_dict()),encoding='utf-8')
        self.sessions=[];self.addCleanup(self.stop_sessions)

    def close_root(self):
        for pending in self.root.tk.call('after','info'):
            self.root.after_cancel(pending)
        self.root.destroy()

    def stop_sessions(self):
        for session in self.sessions:
            session.request_stop('offline_cleanup')
            if session.thread:session.thread.join(3)

    def app(self,preview=False):
        app=App(self.root,self.profile,Settings(),preview=preview)
        app.session=Mock()
        app.session.snapshot.return_value=dict(phase='ERROR',message='fixture ended',error='vehicle_stale_or_invalid',
            finished=True,collector=dict(live=False,rows=403),motor={})
        return app

    def factory(self,profile,settings):
        def capture(root,output,stop,notify):
            notify('collector_session_open',dict(path=str(output/'offline_session')))
            stop.wait(3)
            notify('collector_done',dict(error=None))
        def control(port,output,settings,**kwargs):
            kwargs['stop_event'].set()
            return {'result':'offline_complete'}
        session=DesktopSession(profile,settings,capture=capture,control=control,
                               port_probe=lambda port:'OFFLINE_PORT')
        self.sessions.append(session)
        return session

    def test_finished_ui_does_not_describe_active_wait_or_old_on_state(self):
        app=self.app()
        app.session.snapshot.return_value['motor']=dict(port='OFFLINE_PORT',off_verified=True,
            board=dict(received_ns=1,armed=False,pose_mdeg=[0]*4,target_mdeg=[0]*4,pulses_us=[0]*4),
            pressure=dict(samples=5440,complete_sweeps=1360,last_reason='ok'))
        app.poll()
        self.assertIn('수신 중단',app.vehicle.get())
        self.assertNotIn('연결 대기',app.vehicle.get())
        self.assertIn('출력 OFF 확인',app.board.get())
        self.assertNotIn('출력 ON',app.pwm.get())
        self.assertIn('0 / 0 / 0 / 0',app.pwm.get())
        self.assertIn('5,440',app.pressure.get())
        self.assertEqual(str(app.reconnect_button['state']),'normal')

    def test_missing_off_confirmation_is_not_presented_as_verified(self):
        app=self.app()
        app.session.snapshot.return_value['motor']=dict(off_verified=False,
            board=dict(received_ns=1,armed=True,pose_mdeg=[0]*4,target_mdeg=[0]*4,
                       pulses_us=[1800,1800,1200,1200]))
        app.poll()
        self.assertIn('OFF 미확인',app.board.get())
        self.assertIn('OFF 미확인',app.pwm.get())
        self.assertNotIn('출력 ON',app.pwm.get())

    def test_reconnect_requires_explicit_click_and_creates_distinct_records(self):
        app=self.app();old=app.session
        with patch('experiment_a.desktop.DesktopSession',side_effect=self.factory) as factory:
            app.poll();app.poll()
            factory.assert_not_called()
            app.reconnect()
            first=app.session;first.thread.join(3)
            self.assertIsNot(first,old)
            first_bytes=(first.output/'experiment.json').read_bytes()
            self.assertTrue(first.snapshot()['finished'])
            self.assertEqual(first.snapshot()['retention'],'pending')
            app.reconnect()
            self.assertIs(app.session,first)
            self.assertEqual(factory.call_count,1)
            app.save_recording()
            self.assertEqual(first.snapshot()['retention'],'saved')
            app.reconnect()
            second=app.session;second.thread.join(3)
            self.assertIsNot(second,first)
            self.assertNotEqual(first.output,second.output)
            self.assertEqual((first.output/'experiment.json').read_bytes(),first_bytes)
            self.assertEqual(factory.call_count,2)

    def test_preview_closing_update_and_unfinished_session_cannot_reconnect(self):
        app=self.app()
        with patch('experiment_a.desktop.DesktopSession') as factory:
            for flag in ('preview','closing','restart_required'):
                with self.subTest(flag=flag):
                    setattr(app,flag,True);app.reconnect()
                    self.assertFalse(app.can_reconnect())
                    setattr(app,flag,False)
            app.session.snapshot.return_value['finished']=False
            app.reconnect();self.assertFalse(app.can_reconnect())
            factory.assert_not_called()

    def test_reconnect_reloads_settings_and_rechecks_confirmations(self):
        app=self.app();old=app.session
        Path(self.profile['settings_path']).write_text(json.dumps(Settings().to_dict()),encoding='utf-8')
        with patch('experiment_a.desktop.DesktopSession',side_effect=self.factory):app.reconnect()
        self.assertIs(app.session,old)
        self.assertIn('before opening any device',app.update_status.get())
        self.assertIsNone(self.sessions[0].thread)
        self.assertFalse(Path(self.profile['output_root']).exists())

    def test_finished_preview_keeps_reconnect_and_update_disabled(self):
        app=self.app(preview=True);app.poll()
        self.assertEqual(str(app.reconnect_button['state']),'disabled')
        self.assertEqual(str(app.update_button['state']),'disabled')

    def test_wait_and_saved_are_distinct_even_if_last_collector_flag_is_live(self):
        self.assertIn('연결 대기',vehicle_status_text(dict(finished=False,collector={})))
        self.assertIn('기록 종료',vehicle_status_text(dict(finished=True,collector=dict(live=True))))


if __name__=='__main__':unittest.main()
