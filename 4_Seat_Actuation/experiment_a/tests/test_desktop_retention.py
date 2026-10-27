"""No live devices: stopping and record retention are independent UI actions."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from experiment_a.desktop import App, recording_status_text, vehicle_status_text
from experiment_a.settings import Settings


class DesktopRetentionTests(unittest.TestCase):
    def setUp(self):
        import tkinter as tk
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=tk.Tk();self.root.withdraw();self.addCleanup(self.close_root)
        self.profile=dict(output_root=str(Path(self.temp.name)/'records'),
                          settings_path=str(Path(self.temp.name)/'settings.json'),release=dict(version='DEV'))
        self.settings=Settings()
        self.settings.confirmations={key:True for key in self.settings.confirmations}
        Path(self.profile['settings_path']).write_text(json.dumps(self.settings.to_dict()),encoding='utf-8')

    def close_root(self):
        try:
            for pending in self.root.tk.call('after','info'):self.root.after_cancel(pending)
            self.root.destroy()
        except Exception:pass

    def record(self, *, finished=True, interrupted=False):
        record=Mock()
        status=dict(phase='STOPPED' if finished else 'CONTROL',message='fixture',finished=finished,
                    retention='pending',recording_id='A_fixture',staging_path='/temporary/A_fixture',
                    output='/temporary/A_fixture',persisted_path=None,interrupted=interrupted,
                    collector=dict(live=True,rows=123),motor=dict(off_verified=True))
        record.snapshot.side_effect=lambda:dict(status)
        record.is_running.side_effect=lambda:not status['finished']
        def save():
            status.update(retention='saved',output='/permanent/A_fixture',persisted_path='/permanent/A_fixture')
            return Path('/permanent/A_fixture')
        def discard():status.update(retention='discarded',output=None)
        record.save_recording.side_effect=save;record.discard_recording.side_effect=discard
        return record,status

    def app(self, recovered=()):
        with patch('experiment_a.desktop.discover_pending',return_value=list(recovered)):
            return App(self.root,self.profile,self.settings)

    def test_stop_only_requests_off_and_never_resolves_retention(self):
        app=self.app();record,status=self.record(finished=False);app.session=record
        app.stop()
        record.request_stop.assert_called_once_with()
        record.save_recording.assert_not_called();record.discard_recording.assert_not_called()
        self.assertIn('자동 저장하지 않습니다',app.message.get())
        self.assertEqual(str(app.save_button['state']),'disabled')
        self.assertEqual(str(app.discard_button['state']),'disabled')
        status.update(finished=True,phase='ERROR',error='vehicle:lateral_acceleration_outlier')
        app.poll()
        self.assertIn('미저장',app.recording.get())
        self.assertEqual(str(app.save_button['state']),'normal')
        self.assertEqual(str(app.discard_button['state']),'normal')
        self.assertEqual(str(app.reconnect_button['state']),'disabled')

    def test_current_pending_blocks_reconnect_until_explicit_save(self):
        app=self.app();record,status=self.record();app.session=record
        with patch('experiment_a.desktop.DesktopSession') as factory:
            app.reconnect();factory.assert_not_called()
            self.assertFalse(app.can_reconnect())
            app.save_recording()
            record.save_recording.assert_called_once_with()
            record.discard_recording.assert_not_called()
            self.assertTrue(app.can_reconnect())
            factory.assert_not_called()  # Saving does not restart anything.
        app.poll()
        self.assertEqual(app.recording.get(),'저장 완료')
        self.assertTrue(app.path.get().startswith('저장 위치: '))

    def test_discard_is_explicit_and_confirmation_cancel_keeps_record(self):
        app=self.app();record,status=self.record();app.session=record
        with patch('tkinter.messagebox.askyesno',return_value=False):app.discard_recording()
        record.discard_recording.assert_not_called()
        self.assertFalse(app.can_reconnect())
        with patch('tkinter.messagebox.askyesno',return_value=True):app.discard_recording()
        record.discard_recording.assert_called_once_with()
        record.save_recording.assert_not_called()
        self.assertTrue(app.can_reconnect())

    def test_close_keeps_finished_record_pending_without_implicit_save_or_discard(self):
        app=self.app();record,status=self.record();app.session=record
        app.root=Mock();app.close()
        app.root.destroy.assert_called_once_with()
        record.save_recording.assert_not_called();record.discard_recording.assert_not_called()
        self.assertEqual(status['retention'],'pending')

    def test_close_requests_stop_before_exit_but_never_saves(self):
        app=self.app();record,status=self.record(finished=False);app.session=record
        app.root=Mock();app.close()
        record.request_stop.assert_called_once_with()
        app.root.destroy.assert_not_called()
        record.save_recording.assert_not_called();record.discard_recording.assert_not_called()
        status.update(finished=True,phase='STOPPED')
        app.poll()
        app.root.destroy.assert_called_once_with()
        self.assertEqual(status['retention'],'pending')

    def test_restart_recovery_blocks_auto_start_and_requires_explicit_new_session(self):
        record,status=self.record(interrupted=True)
        with patch('experiment_a.desktop.discover_pending',side_effect=lambda profile:
                   [record] if status['retention']=='pending' else []), \
             patch('experiment_a.desktop.DesktopSession') as factory:
            app=App(self.root,self.profile,self.settings)
            app.start();self.root.update();factory.assert_not_called()
            self.assertIn('미저장 기록이 있어 자동 시작하지 않습니다',app.message.get())
            self.assertFalse(app.can_reconnect())
            self.assertEqual(str(app.pending_button['state']),'normal')
            app.show_pending()
            self.assertIsNotNone(app._pending_dialog)
            factory.assert_not_called()
            app._resolve_recording(record)
            self.assertTrue(app.can_reconnect())
            factory.assert_not_called()

    def test_inventory_error_blocks_start_instead_of_ignoring_unrecoverable_records(self):
        with patch('experiment_a.desktop.discover_pending',side_effect=OSError('inventory unavailable')), \
             patch('experiment_a.desktop.DesktopSession') as factory:
            app=App(self.root,self.profile,self.settings);app.start();app.reconnect()
            factory.assert_not_called()
            self.assertIn('목록을 확인하지 못해 자동 시작을 막았습니다',app.message.get())
            self.assertFalse(app.can_reconnect())

    def test_all_recovered_records_must_be_resolved_before_new_session(self):
        first,state1=self.record();second,state2=self.record()
        state2['recording_id']='A_second'
        with patch('experiment_a.desktop.discover_pending',side_effect=lambda profile:
                   [record for record in (first,second) if record.snapshot()['retention']=='pending']):
            app=App(self.root,self.profile,self.settings)
            app._resolve_recording(first)
            self.assertFalse(app.can_reconnect())
            self.assertEqual(len(app._pending_items()),1)
            app._resolve_recording(second)
            self.assertTrue(app.can_reconnect())

    def test_failed_save_stays_pending_and_does_not_allow_reconnect(self):
        app=self.app();record,status=self.record();app.session=record
        record.save_recording.side_effect=OSError('disk full')
        app.save_recording()
        self.assertEqual(status['retention'],'pending')
        self.assertIn('저장 실패',app.update_status.get())
        self.assertFalse(app.can_reconnect())

    def test_stopping_label_never_claims_saved_and_pending_active_is_explicit(self):
        status=dict(phase='STOPPING',finished=False,retention='pending',collector=dict(rows=123))
        self.assertNotIn('저장 중',vehicle_status_text(status))
        self.assertIn('수집 123행',vehicle_status_text(status))
        self.assertIn('자동 저장하지 않습니다',recording_status_text(status))

    def test_impact_record_only_keeps_stop_enabled_and_retention_choices_disabled(self):
        app=self.app();record,status=self.record(finished=False);app.session=record
        status.update(phase='IMPACT_OFF_PENDING',message='충돌 의심 → 모터 OFF 확인 중')
        status['motor']=dict(off_verified=False,board=dict(armed=True,received_ns=1,
            pose_mdeg=[1000,250,0,0],target_mdeg=[1500,400,0,0],pulses_us=[1800,1800,1200,1200]))
        app.poll()
        self.assertIn('OFF 확인 중',app.board.get())
        self.assertNotIn('출력 OFF 확인 · 기록만',app.board.get())
        self.assertEqual(str(app.stop_button['state']),'normal')
        self.assertEqual(str(app.save_button['state']),'disabled')
        status.update(phase='RECORD_ONLY',message='모터 OFF · 차량·압력 기록 계속 · 자동 재구동 없음')
        status['motor'].update(off_verified=True)
        status['motor']['board'].update(armed=False,pulses_us=[0]*4)
        app.poll()
        self.assertIn('기록 계속',app.message.get())
        self.assertIn('임시 기록 계속',app.recording.get())
        self.assertIn('수신 중',app.vehicle.get())
        self.assertIn('OFF 확인 · 기록만 계속',app.board.get())
        self.assertIn('0 / 0 / 0 / 0',app.pwm.get())
        self.assertIn('각도 표시는 마지막 명령값',app.pwm.get())
        left=app.command_table.item(app.command_table.get_children()[0],'values')
        self.assertEqual(left[1:3],('1.000°','0.250°'))
        for button in (app.save_button,app.discard_button,app.reconnect_button):
            self.assertEqual(str(button['state']),'disabled')
        self.assertEqual(str(app.stop_button['state']),'normal')
        self.assertFalse(app.can_reconnect())
        record.request_stop.assert_not_called()
        app.stop();app.poll()
        self.assertEqual(str(app.stop_button['state']),'disabled')
        record.request_stop.assert_called_once_with()

    def test_record_only_phase_without_off_confirmation_never_claims_verified_output(self):
        app=self.app();record,status=self.record(finished=False);app.session=record
        status.update(phase='RECORD_ONLY')
        status['motor'].update(off_verified=False)
        app.poll()
        self.assertIn('출력 OFF 미확인',app.board.get())
        self.assertIn('출력 OFF 미확인',app.pwm.get())


if __name__=='__main__':unittest.main()
