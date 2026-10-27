import json
import os
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
from types import SimpleNamespace
import tkinter as tk

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import app


class Device:
    connected = True
    port_name = 'SYNTHETIC_TEST'
    capabilities = {}
    last_state = dict(armed=0,moving=0,left_yaw_mdeg=0,left_pitch_mdeg=0,right_yaw_mdeg=0,right_pitch_mdeg=0)
    def __init__(self):
        self.last_state_at = time.monotonic()
        self.commands = []
        self.hold_acknowledged = threading.Event()
    def send(self,command):
        self.commands.append(command)
        if command=='HOLD':
            self.hold_acknowledged.set()
    def attach_session(self,session):
        pass
    def close(self):
        self.connected = False


@unittest.skipUnless(sys.platform=='win32' or os.environ.get('DISPLAY'),'Tk display unavailable')
class AutoUiTests(unittest.TestCase):
    def test_yaw_auto_prepares_pressure_and_camera_before_start_and_can_cancel(self):
        for cancel in (False,True):
            with self.subTest(cancel=cancel),patch.object(app,'serial_devices',return_value=[]),patch.object(app,'camera_devices',return_value=[]):
                root=tk.Tk();root.withdraw()
                gui=app.ControlApp(root)
                gui.scan_target.set('왼쪽')
                device=Device();device.pressure_active=False
                def send(command):
                    Device.send(device,command)
                    if command=='START_12,15':
                        device.pressure_active=True
                device.send=send
                device.snapshot=lambda:({(b,c):dict(status=0,received_at=time.monotonic()) for b in range(3) for c in range(4)}, {})
                gui.serial=device
                camera=SimpleNamespace(ready=False,last_frame_at=time.monotonic(),close=lambda:None)
                with (patch.object(gui,'toggle_camera',side_effect=lambda:setattr(gui,'camera',camera)) as opened,
                      patch.object(gui,'_start_scan') as started):
                    try:
                        gui.toggle_scan('yaw')
                        self.assertEqual(gui.scan_target.get(),'양쪽 동시')
                        opened.assert_called_once()
                        self.assertIn('START_12,15',device.commands)
                        self.assertNotIn('ARM',device.commands)
                        started.assert_not_called()
                        if cancel:
                            gui.hold()
                        camera.ready=True
                        camera.last_frame_at=time.monotonic()
                        gui._prepare_scan()
                        if cancel:
                            started.assert_not_called()
                        else:
                            started.assert_called_once_with('yaw')
                        self.assertIsNone(gui.scan_preparing)
                    finally:
                        gui.close()

    def test_independent_manual_targets_and_single_side_zero(self):
        with patch.object(app,'serial_devices',return_value=[]),patch.object(app,'camera_devices',return_value=[]):
            root=tk.Tk();root.withdraw()
            gui=app.ControlApp(root)
            gui.serial=Device();gui.armed=True
            try:
                gui._set_pose((5,2,7,-3))
                gui.left_yaw.set(8)
                gui.send_target()
                self.assertEqual(gui.serial.commands[-1],'SET,1,8000,2000,7000,-3000')
                gui.neutral(0)
                self.assertEqual(gui.serial.commands[-1],'SET,2,0,0,7000,-3000')
                gui.right_pitch.set(4)
                gui.send_target()
                self.assertEqual(gui.serial.commands[-1],'SET,3,0,0,7000,4000')
                gui.neutral(1)
                self.assertEqual(gui.serial.commands[-1],'SET,4,0,0,0,0')
            finally:
                gui.close()

    def test_auto_button_starts_motion_only_logging_and_stop_saves_cancelled(self):
        with tempfile.TemporaryDirectory() as tmp,patch.object(app,'serial_devices',return_value=[]),\
                patch.object(app,'camera_devices',return_value=[]):
            root = tk.Tk(); root.withdraw()
            gui = app.ControlApp(root)
            gui.serial = Device()
            gui.destination.set(tmp)
            gui.use_pressure.set(False)
            gui.use_video.set(False)
            try:
                gui.scan_button.invoke()
                self.assertTrue(gui.scan.active)
                self.assertEqual(str(gui.arm_button['state']),'disabled')
                with self.assertRaises(RuntimeError):
                    gui.send_target()
                directory = gui.session.directory
                gui.scan_button.invoke()
                self.assertIsNone(gui.scan)
                self.assertIsNone(gui.session)
                deadline = time.monotonic()+3
                while not (directory/'auto_summary.json').exists() and time.monotonic()<deadline:
                    time.sleep(.01)
                summary = json.loads((directory/'auto_summary.json').read_text())
                self.assertEqual(summary['scan_status'],'cancelled')
                self.assertEqual(summary['not_attempted'],357)
                self.assertTrue((directory/'INCOMPLETE').exists())
            finally:
                gui.close()
