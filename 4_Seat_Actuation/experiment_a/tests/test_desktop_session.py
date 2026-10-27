"""Real-thread supervisor tests with fake archive, motor and USB enumeration.

No UART, UDP socket, GUI, installed collector or physical approval is used.
All positive confirmations below belong to temporary offline fixtures only.
"""
import json
from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace
import unittest

from experiment_a.desktop_session import DesktopSession, choose_port
from experiment_a.settings import Settings
from experiment_a.recording_lifecycle import RecordingError, discover_pending


def port(device, vid=None, pid=None, description="fixture"):
    return SimpleNamespace(device=device, vid=vid, pid=pid, description=description)


class PortSelectionTests(unittest.TestCase):
    def test_auto_ignores_bluetooth_unrelated_usb_and_misleading_description(self):
        ports = [port("COM1", description="Bluetooth Serial Port"),
                 port("COM2", vid=0x1234, pid=0x374B, description="ST-LINK fixture"),
                 port("COM3", vid=0x0483, pid=0x0001, description="Other STM USB device")]
        self.assertIsNone(choose_port("auto", ports))

    def test_auto_selects_one_stlink_among_unrelated_ports(self):
        ports = [port("COM1", description="Bluetooth Serial Port"),
                 port("COM11", 0x0483, 0x374E, "STM virtual serial fixture")]
        self.assertEqual(choose_port("auto", ports), "COM11")
        self.assertEqual(choose_port("auto", [port("COM5", 0x0483, 0x1234, "ST-LINK VCP")]), "COM5")

    def test_auto_never_guesses_when_two_stlink_boards_exist(self):
        with self.assertRaisesRegex(RuntimeError, "여러 개"):
            choose_port("auto", [port("COM11", 0x0483, 0x374E), port("COM12", 0x0483, 0x374B)])

    def test_explicit_port_must_be_present_and_is_not_replaced_by_another(self):
        ports = [port("COM11", 0x0483, 0x374E)]
        self.assertEqual(choose_port("COM11", ports), "COM11")
        self.assertIsNone(choose_port("COM12", ports))


class DesktopSessionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.profile = {"output_root": str(Path(self.temp.name)/"records"),
                        "collector_root": "OFFLINE_UNUSED_COLLECTOR",
                        "port": "auto", "labels": {"scenario_id": "offline_fixture_only"}}
        self.settings = Settings()
        self.settings.confirmations = {name: True for name in self.settings.confirmations}
        self.settings.metadata["parameter_selection"] = "offline_supervisor_fixture_only"
        self.capture_entered = threading.Event()
        self.capture_finished = threading.Event()
        self.control_entered = threading.Event()
        self.probe_entered = threading.Event()
        self.board_present = threading.Event()
        self.capture_calls = []
        self.control_calls = []
        self.probe_calls = []
        self.sessions = []
        self.addCleanup(self.stop_sessions)

    def stop_sessions(self):
        for session in self.sessions:
            session.request_stop("offline_test_cleanup")
        for session in self.sessions:
            if session.thread is not None:
                session.thread.join(3)
                self.assertFalse(session.thread.is_alive(), "Offline supervisor thread did not stop")

    def capture(self, root, output, stop_event, notify):
        self.capture_calls.append((root, output, stop_event))
        self.capture_entered.set()
        path = Path(output)/"sessions"/"fixture_session"
        notify("collector_session_open", {"path": str(path)})
        notify("collector_status", {"live": False, "rows": 0})
        stop_event.wait(3)
        notify("collector_done", {"error": None, "path": str(path)})
        self.capture_finished.set()
        return {"error": None, "path": str(path)}

    def control(self, selected_port, output, settings, **kwargs):
        self.control_calls.append((selected_port, output, settings, kwargs))
        self.control_entered.set()
        kwargs["status_callback"]("WAIT_VEHICLE", {"port": selected_port})
        kwargs["stop_event"].wait(3)
        kwargs["status_callback"]("ended", {"result": "stop_requested"})
        return {"result": "stop_requested", "pwm_off_telemetry_verified": True,
                "fixture_only": True}

    def probe(self, requested):
        self.probe_calls.append(requested)
        self.probe_entered.set()
        return "OFFLINE_COM_FIXTURE" if self.board_present.is_set() else None

    def session(self, **overrides):
        kwargs = dict(capture=self.capture, control=self.control, port_probe=self.probe)
        kwargs.update(overrides)
        session = DesktopSession(self.profile, self.settings, **kwargs)
        self.sessions.append(session)
        return session

    def wait_finished(self, session):
        session.thread.join(3)
        self.assertFalse(session.is_running())
        snapshot = session.snapshot()
        self.assertTrue(snapshot["finished"])
        return snapshot

    def manifest(self, session):
        return json.loads((session.output/"experiment.json").read_text(encoding="utf-8"))

    def test_waits_for_usb_then_starts_once_with_pressure_logging_and_unlimited_game_wait(self):
        session = self.session()
        output = session.start()
        self.assertTrue(self.capture_entered.wait(2))
        self.assertTrue(self.probe_entered.wait(2))
        self.assertEqual(self.control_calls, [])
        self.board_present.set()
        self.assertTrue(self.control_entered.wait(2))
        self.assertEqual(len(self.control_calls), 1)
        selected_port, control_output, settings, kwargs = self.control_calls[0]
        self.assertEqual(selected_port, "OFFLINE_COM_FIXTURE")
        self.assertEqual(control_output, output/"control")
        self.assertIs(settings, self.settings)
        self.assertIs(kwargs["record_pressure"], True)
        self.assertIsNone(kwargs["startup_timeout_s"])
        self.assertIs(kwargs["stop_event"], session.stop_event)
        self.assertEqual(kwargs["session_metadata"]["integrated_session"], str(output))
        labels=kwargs["session_metadata"]["labels"]
        for key,value in self.profile["labels"].items():
            self.assertEqual(labels[key],value)
        self.assertEqual(labels["replicate"],output.name)
        session.request_stop("offline_operator_stop")
        snapshot = self.wait_finished(session)
        self.assertEqual(snapshot["phase"], "STOPPED")
        self.assertEqual(snapshot['retention'],'pending')
        self.assertFalse(Path(self.profile['output_root']).exists())
        self.assertIsNone(snapshot["error"])
        self.assertTrue(self.capture_finished.is_set())
        manifest = self.manifest(session)
        self.assertEqual(manifest["pressure_role"], "evaluation_only")
        self.assertEqual(manifest["policy"], "A_BeamNG_only")
        self.assertFalse(manifest["video_enabled"])
        self.assertEqual(manifest["collector_path"], str(output/"beamng"/"sessions"/"fixture_session"))
        self.assertTrue(manifest["motor_result"]["fixture_only"])
        with self.assertRaisesRegex(RuntimeError, "재ARM"):
            session.start()
        self.assertEqual(len(self.control_calls), 1)

    def test_collector_failure_before_archive_open_never_probes_or_opens_motor(self):
        def failed_capture(root, output, stop_event, notify):
            self.capture_entered.set()
            notify("collector_done", {"error": "fixture UDP bind failed", "path": None})
        self.board_present.set()
        session = self.session(capture=failed_capture)
        session.start()
        snapshot = self.wait_finished(session)
        self.assertEqual(self.control_calls, [])
        self.assertEqual(self.probe_calls, [])
        self.assertEqual(snapshot["phase"], "ERROR")
        self.assertIn("fixture UDP bind failed", snapshot["error"])
        self.assertEqual(self.manifest(session)["status"], "error")

    def test_motor_exception_stops_collector_and_never_retries(self):
        def failed_control(selected_port, output, settings, **kwargs):
            self.control_calls.append((selected_port, output, settings, kwargs))
            raise OSError("fixture UART disconnected")
        self.board_present.set()
        session = self.session(control=failed_control)
        session.start()
        snapshot = self.wait_finished(session)
        self.assertEqual(len(self.control_calls), 1)
        self.assertTrue(self.capture_finished.is_set())
        self.assertTrue(session.stop_event.is_set())
        self.assertIn("fixture UART disconnected", snapshot["error"])
        with self.assertRaisesRegex(RuntimeError, "재ARM"):
            session.start()
        self.assertEqual(len(self.control_calls), 1)

    def test_stale_ending_keeps_reason_and_final_off_telemetry_for_ui(self):
        final_board=dict(armed=False,pulses_us=[0,0,0,0],received_ns=42)
        final_pressure=dict(samples=5440,complete_sweeps=1360,scan_active=False)
        def failed_control(selected_port, output, settings, **kwargs):
            notify=kwargs['status_callback']
            notify('telemetry',dict(board=dict(armed=True,pulses_us=[1800,1800,1200,1200])))
            notify('ended',dict(result='vehicle_stale_or_invalid',off_verified=True,
                                board=final_board,pressure=final_pressure))
            raise RuntimeError('vehicle_stale_or_invalid')
        self.board_present.set()
        session=self.session(control=failed_control);session.start()
        snapshot=self.wait_finished(session)
        self.assertEqual(snapshot['error'],'vehicle_stale_or_invalid')
        self.assertIn('200ms',snapshot['message'])
        self.assertIn('멈췄을 수',snapshot['message'])
        self.assertIn('다시 연결',snapshot['message'])
        self.assertEqual(snapshot['motor']['board'],final_board)
        self.assertEqual(snapshot['motor']['pressure'],final_pressure)
        self.assertTrue(snapshot['motor']['off_verified'])

    def test_stop_while_waiting_for_board_closes_archive_without_motor(self):
        session = self.session()
        session.start()
        self.assertTrue(self.probe_entered.wait(2))
        session.request_stop("offline_stop_before_board")
        snapshot = self.wait_finished(session)
        self.assertEqual(self.control_calls, [])
        self.assertTrue(self.capture_finished.is_set())
        self.assertEqual(snapshot["phase"], "STOPPED")
        self.assertEqual(snapshot['retention'],'pending')
        self.assertFalse(Path(self.profile['output_root']).exists())
        self.assertIsNone(snapshot["error"])

    def test_active_workers_prevent_save_and_discard_then_stop_allows_explicit_save(self):
        session=self.session();session.start()
        self.assertTrue(self.capture_entered.wait(2))
        for action in (session.save_recording,session.discard_recording):
            with self.assertRaisesRegex(RecordingError,'Stop all recording workers'):action()
        self.assertFalse(Path(self.profile['output_root']).exists())
        session.request_stop();self.wait_finished(session)
        raw_manifest=(session.output/'experiment.json').read_bytes()
        saved=session.save_recording()
        self.assertEqual(session.output,saved)
        self.assertEqual(session.snapshot()['retention'],'saved')
        self.assertIn('저장 완료',session.snapshot()['message'])
        self.assertEqual((saved/'experiment.json').read_bytes(),raw_manifest)
        self.assertEqual(discover_pending(self.profile),[])

    def test_fault_only_leaves_pending_until_explicit_discard(self):
        def failed(*args,**kwargs):raise RuntimeError('vehicle:lateral_acceleration_outlier')
        self.board_present.set()
        session=self.session(control=failed);session.start()
        snapshot=self.wait_finished(session)
        self.assertEqual(snapshot['retention'],'pending')
        self.assertEqual(snapshot['phase'],'ERROR')
        self.assertIn('데이터 유효 범위',snapshot['message'])
        self.assertIn('충돌을 확정하지',snapshot['message'])
        self.assertNotIn('±12',snapshot['message'])
        self.assertFalse(Path(self.profile['output_root']).exists())
        self.assertEqual(len(discover_pending(self.profile)),1)
        session.discard_recording()
        self.assertEqual(session.snapshot()['retention'],'discarded')
        self.assertIn('버렸습니다',session.snapshot()['message'])
        self.assertEqual(session.snapshot()['error'],'vehicle:lateral_acceleration_outlier')
        self.assertIsNone(session.output)
        self.assertEqual(discover_pending(self.profile),[])

    def test_impact_turns_motor_off_but_both_workers_record_until_operator_stops(self):
        off_pending=threading.Event();allow_off=threading.Event();record_only=threading.Event()
        impact=dict(planar_accel_mps2=31.,window_impulse_mps=1.7)
        final_board=dict(armed=False,pulses_us=[0]*4,pose_mdeg=[1000,250,0,0],
                         target_mdeg=[1000,250,0,0],received_ns=42)
        def impact_control(selected_port, output, settings, **kwargs):
            self.control_calls.append((selected_port,output,settings,kwargs))
            notify=kwargs['status_callback'];stop=kwargs['stop_event']
            notify('IMPACT_OFF_PENDING',dict(reason='impact_suspected',impact=impact,off_verified=False))
            off_pending.set();allow_off.wait(2)
            notify('RECORD_ONLY',dict(reason='impact_suspected',impact=impact,off_verified=True,board=final_board))
            notify('telemetry',dict(pressure=dict(samples=80,complete_sweeps=20)))
            record_only.set();stop.wait(3)
            notify('ended',dict(result='stop_requested',off_verified=True,board=final_board))
            return dict(result='stop_requested',pwm_off_telemetry_verified=True,impact=impact)
        self.board_present.set();session=self.session(control=impact_control);session.start()
        self.assertTrue(off_pending.wait(2))
        pending=session.snapshot()
        self.assertEqual(pending['phase'],'IMPACT_OFF_PENDING')
        self.assertIn('OFF 확인 중',pending['message'])
        self.assertFalse(pending['impact_event']['off_verified'])
        self.assertFalse(session.stop_event.is_set());self.assertFalse(self.capture_finished.is_set())
        allow_off.set();self.assertTrue(record_only.wait(2))
        active=session.snapshot()
        self.assertEqual(active['phase'],'RECORD_ONLY')
        self.assertIn('차량·압력 기록 계속',active['message'])
        self.assertIn('자동 재구동 없음',active['message'])
        self.assertFalse(active['finished']);self.assertIsNone(active['error'])
        self.assertTrue(active['motor']['off_verified'])
        self.assertEqual(active['motor']['board'],final_board)
        self.assertEqual(active['motor']['pressure']['complete_sweeps'],20)
        self.assertFalse(session.stop_event.is_set());self.assertFalse(self.capture_finished.is_set())
        for action in (session.save_recording,session.discard_recording):
            with self.assertRaisesRegex(RecordingError,'Stop all recording workers'):action()
        session.request_stop();stopped=self.wait_finished(session)
        self.assertEqual(stopped['phase'],'STOPPED');self.assertIsNone(stopped['error'])
        self.assertIn('충돌 의심 감지 이력',stopped['message'])
        self.assertEqual(stopped['impact_event']['reason'],'impact_suspected')
        self.assertEqual(stopped['impact_event']['impact'],impact)
        self.assertTrue(stopped['impact_event']['off_verified'])
        manifest=self.manifest(session)
        self.assertEqual(manifest['impact_event'],stopped['impact_event'])
        self.assertEqual(manifest['motor_result']['impact'],impact)
        self.assertEqual(len(self.control_calls),1)
        session.save_recording()
        self.assertIn('충돌 의심 감지 이력',session.snapshot()['message'])
        self.assertEqual(self.manifest(session)['impact_event'],manifest['impact_event'])

    def test_recording_error_after_impact_preserves_both_reasons(self):
        def impact_then_error(selected_port, output, settings, **kwargs):
            kwargs['status_callback']('RECORD_ONLY',dict(reason='impact_suspected',
                impact=dict(planar_accel_mps2=61.),off_verified=True))
            raise OSError('fixture recording failure after impact')
        self.board_present.set();session=self.session(control=impact_then_error);session.start()
        final=self.wait_finished(session)
        self.assertEqual(final['phase'],'ERROR')
        self.assertIn('fixture recording failure',final['error'])
        self.assertEqual(final['impact_event']['reason'],'impact_suspected')
        self.assertIn('충돌 의심 감지 이력',final['message'])
        self.assertEqual(self.manifest(session)['impact_event'],final['impact_event'])

    def test_unconfirmed_settings_create_no_threads_directories_or_resources(self):
        session = self.session()
        session.settings = Settings()  # Real defaults remain unconfirmed.
        with self.assertRaisesRegex(ValueError, "before opening any device"):
            session.start()
        self.assertIsNone(session.thread)
        self.assertIsNone(session.output)
        self.assertFalse(Path(self.profile["output_root"]).exists())
        self.assertEqual(self.capture_calls, [])
        self.assertEqual(self.control_calls, [])
        self.assertEqual(self.probe_calls, [])
        self.assertTrue(all(not value for value in session.settings.confirmations.values()))

    def test_motor_return_without_stop_is_an_error_and_not_auto_restarted(self):
        def returned_control(selected_port, output, settings, **kwargs):
            self.control_calls.append((selected_port, output, settings, kwargs))
            return {"result": "fixture unexpected return"}
        self.board_present.set()
        session = self.session(control=returned_control)
        session.start()
        snapshot = self.wait_finished(session)
        self.assertEqual(len(self.control_calls), 1)
        self.assertTrue(self.capture_finished.is_set())
        self.assertEqual(snapshot["phase"], "ERROR")
        self.assertIn("자동 재시작하지 않음", snapshot["error"])

    def test_collector_silent_return_also_prevents_motor_start(self):
        def returned_capture(root, output, stop_event, notify):
            return None
        self.board_present.set()
        session = self.session(capture=returned_capture)
        session.start()
        snapshot = self.wait_finished(session)
        self.assertEqual(self.control_calls, [])
        self.assertEqual(self.probe_calls, [])
        self.assertEqual(snapshot["phase"], "ERROR")
        self.assertIn("예상 밖 종료", snapshot["error"])


if __name__ == "__main__":
    unittest.main()
