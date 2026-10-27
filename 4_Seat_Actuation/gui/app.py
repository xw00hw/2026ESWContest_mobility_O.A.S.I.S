"""Windows desktop control station for one H723 + 12 FSRs + USB Arducam."""
from __future__ import annotations

import json
import os
from pathlib import Path
import queue
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import cv2
from PIL import Image, ImageTk

from devices import CameraWorker, SerialWorker, camera_devices, serial_devices
from protocol import set_command
from kinematics import inverse,BOOT_PULSES,YAW_LIMIT_DEG,PITCH_LIMIT_DEG,POSE_FIELDS
from session import Session, VideoSink
from auto_scan import AutoScan,grid
from auto_summary import summarize

HERE = Path(__file__).resolve().parent
AUTO_TOTAL = len(grid())


class ControlApp:
    def __init__(self, root, config=None):
        self.root = root
        self.config = config or json.loads((HERE/'station_config.json').read_text(encoding='utf-8'))
        self.events = queue.Queue()
        self.serial = self.camera = self.session = self.video = None
        self.scan = None
        self.scan_preparing = None
        self.armed = False
        self.command_id = 0
        self.last_sent = None
        self.send_after = None
        self.closing = False
        self.port_choices, self.camera_choices = [],[]
        self.root.title('O.A.S.I.S. · Yaw / Pitch & Acquisition')
        self.root.geometry('1220x850')
        self.root.minsize(1000,800)
        self.root.protocol('WM_DELETE_WINDOW',self.close)
        self.root.bind('<Escape>',lambda event:self.hold())
        self.port = tk.StringVar()
        self.camera_name = tk.StringVar()
        self.pose_vars = [tk.DoubleVar(value=0) for _ in range(4)]
        self.left_yaw,self.left_pitch,self.right_yaw,self.right_pitch = self.pose_vars
        self.scan_target = tk.StringVar(value='양쪽 동시')
        self.auto = tk.BooleanVar(value=True)
        self.use_pressure = tk.BooleanVar(value=True)
        self.use_video = tk.BooleanVar(value=True)
        self.destination = tk.StringVar(value=str(HERE/'sessions'))
        self.serial_text = tk.StringVar(value='STM32 연결 안 됨')
        self.motion_text = tk.StringVar(value='구동 비활성 · 1200 µs 기준 중립')
        self.pwm_text = tk.StringVar(value='  '.join(f'D{i}={us}' for i,us in zip((3,4,5,6),BOOT_PULSES))+' µs')
        self.crank_text = tk.StringVar(value='θ1=0.000° · θ2=0.000°')
        self.record_text = tk.StringVar(value='기록 대기')
        self.pressure_text = tk.StringVar(value='압력 측정 정지 · 표시 단위: 추정 전압 V')
        self.notice = tk.StringVar(value='MUX 주소선 S0~S3 → D8·D9·D10·D11 / EN D12 / SIG A0·A1·A2')
        self._build()
        self.refresh_devices()
        self.root.after(100,self.poll)

    def _build(self):
        style = ttk.Style()
        style.theme_use('clam')
        style.configure('Title.TLabel',font=('Segoe UI',17,'bold'))
        outer = ttk.Frame(self.root,padding=12)
        outer.pack(fill='both',expand=True)
        ttk.Label(outer,text='O.A.S.I.S.  CONTROL STATION',style='Title.TLabel').pack(anchor='w')
        ttk.Label(outer,text='Yaw · Pitch 명령 / 압력매트 12채널 / Arducam 영상').pack(anchor='w',pady=(0,8))
        bar = ttk.Frame(outer)
        bar.pack(fill='x')
        ttk.Label(bar,text='STM32').pack(side='left')
        self.port_box = ttk.Combobox(bar,textvariable=self.port,width=31,state='readonly')
        self.port_box.pack(side='left',padx=6)
        self.connect_button = ttk.Button(bar,text='연결',command=lambda:self.action(self.toggle_serial))
        self.connect_button.pack(side='left')
        ttk.Button(bar,text='장치 새로고침',command=lambda:self.action(self.refresh_devices)).pack(side='left',padx=6)
        ttk.Label(bar,textvariable=self.serial_text).pack(side='left',padx=8)
        footer = ttk.Frame(outer)
        footer.pack(side='bottom',fill='x')
        middle = ttk.Frame(outer)
        middle.pack(fill='both',expand=True,pady=10)
        controls = ttk.LabelFrame(middle,text='구조물 자세각 · C4_R8 역기구학',padding=12)
        controls.pack(side='left',fill='y',padx=(0,10))
        self.arm_button = ttk.Button(controls,text='구동 활성화',command=lambda:self.action(self.arm))
        self.arm_button.pack(fill='x',pady=(0,4))
        self.manual_widgets = [self.arm_button]
        joints = ttk.Frame(controls)
        joints.pack(fill='x')
        for side,title in enumerate(('왼쪽 · D4 / D6','오른쪽 · D3 / D5')):
            panel = ttk.LabelFrame(joints,text=title,padding=6)
            panel.pack(side='left',fill='both',expand=True,padx=2)
            for axis,(label,limit) in enumerate((('Yaw',YAW_LIMIT_DEG),('Pitch',PITCH_LIMIT_DEG))):
                var = self.pose_vars[side*2+axis]
                minimum = 0 if axis==0 else -limit
                row = ttk.Frame(panel)
                row.pack(fill='x',pady=2)
                ttk.Label(row,text=label).pack(side='left')
                spin = ttk.Spinbox(row,from_=minimum,to=limit,increment=.1,width=6,textvariable=var,
                                   command=self.changed)
                spin.pack(side='right')
                scale = ttk.Scale(panel,from_=minimum,to=limit,variable=var,
                                  command=lambda value:self.changed(),length=150)
                scale.pack(fill='x',pady=(0,4))
                self.manual_widgets.extend([spin,scale])
                spin.bind('<Return>',lambda event:self.changed())
                spin.bind('<FocusOut>',lambda event:self.changed())
            zero = ttk.Button(panel,text='영점',command=lambda s=side:self.action(lambda:self.neutral(s)))
            zero.pack(fill='x')
            self.manual_widgets.append(zero)
        send_row = ttk.Frame(controls)
        send_row.pack(fill='x',pady=3)
        ttk.Checkbutton(send_row,text='조절 시 자동 전송',variable=self.auto).pack(side='left')
        send_button = ttk.Button(send_row,text='전송',width=6,command=lambda:self.action(self.send_target))
        send_button.pack(side='right')
        neutral_button = ttk.Button(controls,text='양쪽 영점으로 이동',command=lambda:self.action(self.neutral))
        neutral_button.pack(fill='x')
        self.manual_widgets.extend([send_button,neutral_button])
        tk.Button(controls,text='정지 · 현재 PWM 유지  [Esc]',background='#b42318',foreground='white',
                  font=('Segoe UI',11,'bold'),command=self.hold).pack(fill='x',pady=6)
        ttk.Label(controls,textvariable=self.motion_text,wraplength=290).pack(anchor='w')
        ttk.Label(controls,textvariable=self.pwm_text,wraplength=290).pack(anchor='w',pady=3)
        ttk.Label(controls,textvariable=self.crank_text,wraplength=290).pack(anchor='w')
        ttk.Label(controls,text='DS · 1200 µs 기준 · 실제 자세 피드백 없음\n왼쪽 D4=θ1 / D6=−θ2\n오른쪽 D3=−θ1 / D5=θ2',
                  wraplength=290,foreground='#475467').pack(anchor='w')
        camera = ttk.LabelFrame(middle,text='Arducam 미리보기',padding=8)
        camera.pack(side='left',fill='both',expand=True)
        camera_bar = ttk.Frame(camera)
        camera_bar.pack(fill='x')
        self.camera_box = ttk.Combobox(camera_bar,textvariable=self.camera_name,width=34,state='readonly')
        self.camera_box.pack(side='left',fill='x',expand=True)
        self.camera_button = ttk.Button(camera_bar,text='카메라 열기',command=lambda:self.action(self.toggle_camera))
        self.camera_button.pack(side='left',padx=(8,0))
        self.preview = tk.Label(camera,text='Arducam을 선택하고 카메라 열기를 누르세요.',
                                background='#101828',foreground='white',width=1,height=1)
        self.preview.pack(fill='both',expand=True,pady=(8,0))
        pressure = ttk.LabelFrame(footer,text='압력매트 · ADC 전압 (하중·압력 보정 전)',padding=8)
        pressure.pack(fill='x')
        pbar = ttk.Frame(pressure)
        pbar.pack(fill='x',pady=(0,5))
        ttk.Button(pbar,text='압력 측정 시작',command=lambda:self.action(self.start_pressure)).pack(side='left')
        ttk.Button(pbar,text='측정 정지',command=lambda:self.action(self.stop_pressure)).pack(side='left',padx=5)
        ttk.Label(pbar,textvariable=self.pressure_text).pack(side='left',padx=8)
        self.cells = {}
        for bank,label in enumerate(['A0 · 좌판','A1 · Side','A2 · Back']):
            row = ttk.Frame(pressure)
            row.pack(fill='x',pady=2)
            ttk.Label(row,text=label,width=13).pack(side='left')
            for channel in range(4):
                canvas = tk.Canvas(row,height=30,width=210,background='#eef2f6',highlightthickness=0)
                canvas.pack(side='left',fill='x',expand=True,padx=3)
                self.cells[bank,channel] = canvas
        recording = ttk.LabelFrame(footer,text='공통 시간축 기록',padding=8)
        recording.pack(fill='x',pady=(10,0))
        top = ttk.Frame(recording)
        top.pack(fill='x')
        pressure_check = ttk.Checkbutton(top,text='압력 데이터',variable=self.use_pressure)
        pressure_check.pack(side='left')
        video_check = ttk.Checkbutton(top,text='영상',variable=self.use_video)
        video_check.pack(side='left',padx=8)
        self.manual_widgets.extend([pressure_check,video_check])
        self.record_button = ttk.Button(top,text='기록 시작',command=lambda:self.action(self.toggle_recording))
        self.record_button.pack(side='left',padx=8)
        self.scan_button = ttk.Button(top,text=f'AUTO {AUTO_TOTAL}',command=lambda:self.action(self.toggle_scan))
        self.scan_button.pack(side='left',padx=4)
        self.yaw_scan_button = ttk.Button(top,text='양쪽 Yaw AUTO 784',width=20,command=lambda:self.action(lambda:self.toggle_scan('yaw')))
        self.yaw_scan_button.pack(side='left',padx=4)
        scan_target = ttk.Combobox(top,textvariable=self.scan_target,values=('양쪽 동시','왼쪽','오른쪽'),
                                  width=10,state='readonly')
        scan_target.pack(side='left',padx=4)
        self.manual_widgets.append(scan_target)
        ttk.Label(top,textvariable=self.record_text).pack(side='left',padx=8)
        bottom = ttk.Frame(recording)
        bottom.pack(fill='x',pady=(6,0))
        ttk.Entry(bottom,textvariable=self.destination).pack(side='left',fill='x',expand=True)
        ttk.Button(bottom,text='저장 폴더',command=self.choose_destination).pack(side='left',padx=6)
        ttk.Button(bottom,text='폴더 열기',command=self.open_destination).pack(side='left')
        ttk.Label(footer,textvariable=self.notice,wraplength=1150,foreground='#344054').pack(anchor='w',pady=(8,0))

    def action(self, callback):
        try:
            callback()
        except Exception as error:
            self.notice.set(str(error))
            messagebox.showerror('O.A.S.I.S.',str(error),parent=self.root)

    def refresh_devices(self):
        if self.serial and self.serial.connected:
            self.serial.touch()
        self.port_choices = serial_devices()
        self.port_box['values'] = [p['label'] for p in self.port_choices]
        if self.port.get() not in self.port_box['values']:
            chosen_port = next((p['label'] for p in self.port_choices if p['stlink']), '')
            self.port.set(chosen_port)
        self.camera_choices = camera_devices()
        self.camera_box['values'] = [c['label'] for c in self.camera_choices]
        if self.camera_name.get() not in self.camera_box['values']:
            chosen = next((c for c in self.camera_choices if 'arducam' in c['name'].lower()),
                          self.camera_choices[0] if self.camera_choices else None)
            self.camera_name.set(chosen['label'] if chosen else '')

    def toggle_serial(self):
        if self.session:
            raise RuntimeError('기록을 종료한 뒤 연결을 변경하세요.')
        if self.serial:
            self.serial.close()
            self.serial = None
            self.armed = False
            self.connect_button.config(text='연결')
            return
        chosen = next((p for p in self.port_choices if p['label']==self.port.get()),None)
        if not chosen:
            raise ValueError('STM32 포트를 선택하세요.')
        self.serial = SerialWorker(chosen['port'],self.events,self.config['vref_v'])
        self.serial.start()
        self.serial_text.set('펌웨어 확인 중…')
        self.connect_button.config(text='연결 해제')

    def arm(self):
        if self.scan and self.scan.active:
            raise RuntimeError('AUTO 중에는 수동 명령을 사용할 수 없습니다.')
        if not self.serial or not self.serial.connected:
            raise RuntimeError('STM32 연결을 먼저 완료하세요.')
        if self.armed:
            self.hold()
            return
        self.serial.send('ARM')
        self.serial.send('STATE')
        # Activation never transmits a stale slider target automatically.
        self.notice.set('구동 활성화 요청. 슬라이더를 조절하거나 목표각 전송을 누르세요.')

    def hold(self):
        if self.scan_preparing:
            self.scan_preparing = None
            self._scan_controls(False)
        if self.scan and self.scan.active:
            self.scan.abort('user_stop','cancelled')
        self.armed = False
        self.last_sent = None
        self.arm_button.config(text='구동 활성화')
        if self.send_after:
            self.root.after_cancel(self.send_after)
            self.send_after = None
        if self.serial and self.serial.connected:
            self.serial.send('HOLD')
        self.notice.set('정지 요청: 현재 PWM 유지. 외부 모터 전원은 유지됩니다.')

    def changed(self):
        try:
            self._cranks(tuple(var.get() for var in self.pose_vars))
        except (ValueError,tk.TclError) as error:
            self.notice.set(str(error))
        if self.send_after is None:
            self.send_after = self.root.after(120,self._auto_send)

    def _auto_send(self):
        self.send_after = None
        if self.auto.get() and self.armed and not (self.scan and self.scan.active):
            try:
                self.send_target()
            except (ValueError,tk.TclError,RuntimeError) as error:
                self.notice.set(str(error))

    def send_target(self):
        if self.scan and self.scan.active:
            raise RuntimeError('AUTO 중에는 수동 명령을 사용할 수 없습니다.')
        if not self.serial or not self.serial.connected or not self.armed:
            raise RuntimeError('STM32 연결 후 구동 활성화를 누르세요.')
        pose = tuple(var.get() for var in self.pose_vars)
        self._send_pose(*pose)
        self.notice.set(f'목표 전송: 왼쪽 {pose[:2]}° / 오른쪽 {pose[2:]}° · ACK 대기')

    def _cranks(self,pose):
        text = []
        for side,label in enumerate(('왼쪽','오른쪽')):
            motors = inverse(*pose[side*2:side*2+2])['motors']
            text.append(f"{label} θ1={motors[0]['phi_deg']:.2f}° · θ2={motors[1]['phi_deg']:.2f}°")
        self.crank_text.set('\n'.join(text))

    def _set_pose(self,pose):
        for var,value in zip(self.pose_vars,pose):
            var.set(value)
        self._cranks(pose)

    def _send_pose(self,*pose):
        self.command_id = (self.command_id+1)&0x7fffffff
        command = set_command(self.command_id,*pose)
        self.serial.send(command)
        self.last_sent = pose
        self._set_pose(pose)
        return self.command_id

    def neutral(self,side=None):
        if self.scan and self.scan.active:
            raise RuntimeError('AUTO 중에는 수동 명령을 사용할 수 없습니다.')
        for i,var in enumerate(self.pose_vars):
            if side is None or i//2==side:
                var.set(0)
        self.send_target()

    def start_pressure(self):
        if self.scan and self.scan.active:
            raise RuntimeError('AUTO 중에는 압력 설정을 변경할 수 없습니다.')
        if not self.serial or not self.serial.connected:
            raise RuntimeError('STM32 연결을 먼저 완료하세요.')
        self.serial.send('START_12,15')
        self.use_pressure.set(True)

    def stop_pressure(self):
        if self.session and self.session.metadata['context']['pressure']:
            self.stop_recording()
        if self.serial and self.serial.connected:
            self.serial.send('DISABLE')

    def toggle_camera(self):
        if self.session:
            raise RuntimeError('기록을 종료한 뒤 카메라를 변경하세요.')
        if self.camera:
            self.camera.close()
            self.camera = None
            self.camera_button.config(text='카메라 열기')
            self.preview.config(image='',text='카메라 닫힘')
            return
        chosen = next((c for c in self.camera_choices if c['label']==self.camera_name.get()),None)
        if not chosen:
            raise ValueError('카메라를 선택하세요.')
        self.camera = CameraWorker(chosen,self.events,**self.config['camera'])
        self.camera.start()
        self.camera_button.config(text='카메라 닫기')

    def choose_destination(self):
        path = filedialog.askdirectory(initialdir=self.destination.get(),parent=self.root)
        if path:
            self.destination.set(path)

    def open_destination(self):
        path = self.session.directory if self.session else Path(self.destination.get())
        path.mkdir(parents=True,exist_ok=True)
        os.startfile(path)

    def toggle_recording(self):
        if self.session:
            self.stop_recording()
        else:
            self.start_recording()

    def start_recording(self,automatic=False):
        if self.session:
            raise RuntimeError('이미 기록 중입니다.')
        now = time.monotonic()
        if not automatic and not self.use_pressure.get() and not self.use_video.get():
            raise ValueError('압력 또는 영상을 하나 이상 선택하세요.')
        if self.use_pressure.get() and (not self.serial or not self.serial.connected or
            not self.serial.pressure_active or now-self.serial.last_pressure_at>.5):
            raise RuntimeError('압력 측정을 먼저 시작하고 데이터 수신을 확인하세요.')
        if self.use_video.get() and (not self.camera or not self.camera.ready or
                                     now-self.camera.last_frame_at>.5):
            raise RuntimeError('카메라를 열고 미리보기를 확인하세요.')
        session = Session(self.destination.get(),self.config,dict(automatic=automatic,pressure=self.use_pressure.get(),
            video=self.use_video.get(), port=self.serial.port_name if self.serial else None,
            camera=self.camera.device if self.camera else None,
            firmware_capabilities=self.serial.capabilities if self.serial else None,
            initial_motion_state=self.serial.last_state if self.serial else None))
        try:
            if self.use_video.get():
                self.video = VideoSink(session,(self.camera.width,self.camera.height),self.camera.fps)
                self.camera.start_recording(self.video)
            if self.serial:
                self.serial.attach_session(session)
            self.session = session
            self.record_button.config(text='기록 종료')
            self.notice.set(str(session.directory))
        except Exception as error:
            session.error(str(error))
            if self.video:
                self.video.close()
                self.video = None
            session.close()
            raise

    def stop_recording(self):
        if not self.session:
            return
        self.hold()
        session = self.session
        if self.serial and self.serial.connected:
            if not self.serial.hold_acknowledged.wait(timeout=.6):
                session.error('hold_ack_timeout_at_recording_stop')
        if self.camera:
            self.camera.stop_recording()
        if self.serial:
            self.serial.attach_session(None)
        if self.video:
            try:
                self.video.close()
            except Exception as error:
                session.error(str(error))
            self.video = None
        try:
            session.close()
        finally:
            self.session = None
            self.record_button.config(text='기록 시작')
        if self.scan:
            self.scan = None
            self._scan_controls(False)
        if 'auto_scan' in session.metadata:
            (session.directory/'AUTO_SUMMARY_INCOMPLETE').write_text('processing\n',encoding='utf-8')
            def process():
                try:
                    result = summarize(session.directory)
                    self.events.put(dict(type='auto_saved',directory=str(session.directory),result=result))
                except Exception as error:
                    self.events.put(dict(type='auto_save_error',message=str(error),directory=str(session.directory)))
            threading.Thread(target=process,name='AUTO data summary',daemon=False).start()
        self.record_text.set('오류 기록 보존' if session.errors else '저장 완료')
        self.notice.set(str(session.directory))

    def _scan_source_error(self):
        if not self.serial or not self.serial.connected:
            return 'STM32 연결이 끊겼습니다.'
        context = self.session.metadata['context'] if self.session else dict(
            pressure=self.use_pressure.get(),video=self.use_video.get())
        now = time.monotonic()
        if context['pressure']:
            values,_ = self.serial.snapshot()
            if not self.serial.pressure_active or any((b,c) not in values or
                values[b,c]['status']!=0 or now-values[b,c]['received_at']>.5
                for b in range(3) for c in range(4)):
                return '압력 12채널 수신/유효성 확인이 필요합니다.'
        if context['video'] and (not self.camera or not self.camera.ready or now-self.camera.last_frame_at>.5):
            return '카메라 미리보기를 먼저 확인하세요.'

    def _scan_controls(self,busy):
        for widget in self.manual_widgets:
            widget.config(state='disabled' if busy else ('readonly' if isinstance(widget,ttk.Combobox) else 'normal'))
        self.scan_button.config(text='AUTO 중지' if busy else f'AUTO {AUTO_TOTAL}')
        self.yaw_scan_button.config(text='AUTO 중지' if busy else '양쪽 Yaw AUTO 784')

    def toggle_scan(self,mode='grid'):
        if self.scan_preparing:
            self.hold()
            return
        if self.scan:
            if self.scan.active:
                self.scan.abort('auto_button_stop','cancelled')
            self.stop_recording()
            return
        if self.session:
            raise RuntimeError('현재 기록을 종료한 뒤 AUTO를 시작하세요.')
        if mode=='yaw':
            self.scan_target.set('양쪽 동시')
        self.hold()
        self.scan_preparing = mode
        self.scan_prepare_deadline = time.monotonic()+15
        self._prepare_pressure_sent = False
        self._scan_controls(True)
        self._prepare_scan()

    def _prepare_scan(self):
        if not self.scan_preparing or self.closing:
            return
        try:
            if time.monotonic()>self.scan_prepare_deadline:
                raise RuntimeError('AUTO 장치 준비 시간 초과. 연결과 카메라/압력 수신을 확인하세요.')
            if self.serial is None:
                self.toggle_serial()
            if self.use_video.get() and self.camera is None:
                self.toggle_camera()
            if self.serial.connected:
                if self.use_pressure.get() and not self.serial.pressure_active and not self._prepare_pressure_sent:
                    self.start_pressure()
                    self._prepare_pressure_sent = True
                if not self._scan_source_error() and self.serial.last_state is not None and (
                    time.monotonic()-self.serial.last_state_at<=.5 and not self.serial.last_state['moving']):
                    self._start_scan(self.scan_preparing)
                    self.scan_preparing = None
                    return
            self.notice.set('AUTO 준비: STM32 · 압력 · 선택 카메라 수신 대기')
            self.root.after(100,self._prepare_scan)
        except Exception as error:
            self.hold()
            self.notice.set(str(error))

    def _start_scan(self,mode):
        error = self._scan_source_error()
        if error:
            raise RuntimeError(error)
        if self.serial.last_state is None or time.monotonic()-self.serial.last_state_at>.5:
            raise RuntimeError('STM32 상태 응답을 기다려 주세요.')
        if self.serial.last_state['moving']:
            raise RuntimeError('이동이 끝난 뒤 AUTO를 시작하세요.')
        self.start_recording(automatic=True)
        try:
            self.scan = AutoScan(self.session,self.serial.send,self._send_pose,
                                 mode=mode,
                                 side={'양쪽 동시':'both','왼쪽':'left','오른쪽':'right'}[self.scan_target.get()],
                                 initial_pose=tuple(self.serial.last_state[name]/1000 for name in POSE_FIELDS),
                                 **self.config.get('auto_scan',{}))
        except Exception as error:
            self.session.error('auto_start: '+str(error))
            self.stop_recording()
            raise
        self._scan_controls(True)

    def _event(self,event):
        if self.scan and self.scan.active:
            if event['type'] in ('serial_error','camera_error','device_error','disconnected'):
                self.scan.abort(event.get('message',event['type']))
            else:
                self.scan.event(event)
        kind = event['type']
        if kind=='connected':
            self.serial_text.set(event['port']+' · 좌우 독립 / DS 펌웨어 v6')
        elif kind=='disconnected':
            if self.scan_preparing:
                self.hold()
            self.serial_text.set('연결 해제됨')
            self.armed = False
            self.arm_button.config(text='구동 활성화')
        elif kind=='state':
            s = event['state']
            if s['armed'] and not self.armed and not (self.scan and self.scan.active):
                self._set_pose(tuple(s[name]/1000 for name in POSE_FIELDS))
            self.armed = bool(s['armed'])
            self.arm_button.config(text='구동 비활성화' if self.armed else '구동 활성화')
            self.motion_text.set(f"왼쪽 Y {s['left_yaw_mdeg']/1000:.2f}° / P {s['left_pitch_mdeg']/1000:.2f}°\n"
                + f"오른쪽 Y {s['right_yaw_mdeg']/1000:.2f}° / P {s['right_pitch_mdeg']/1000:.2f}°\n"
                + ('이동 중' if s['moving'] else '유지 중')+f" · {s['fault']}")
            self.pwm_text.set('  '.join(f"D{i}={s[f'D{i}_us']}" for i in (3,4,5,6))+' µs')
        elif kind=='reply':
            self.notice.set(event['text'])
        elif kind in ('serial_error','camera_error','device_error'):
            self.hold()
            if self.session:
                self.session.error(event['message'])
                self.stop_recording()
            self.notice.set(event['message'])
        elif kind=='camera_ready':
            self.notice.set(f"카메라 준비 · {event['size'][0]}×{event['size'][1]} · 요청 {event['requested_fps']}fps")
        elif kind=='auto_saved' and not self.session:
            self.notice.set(f"AUTO 저장: {event['result']['completed']}/{event['result']['planned']} 완료 · {event['directory']}")
        elif kind=='auto_save_error':
            self.notice.set('AUTO 요약 오류: '+event['message']+' · '+event['directory'])

    def poll(self):
        if self.closing:
            return
        if self.serial:
            self.serial.touch()
        for _ in range(200):
            try:
                self._event(self.events.get_nowait())
            except queue.Empty:
                break
        now = time.monotonic()
        values, histories = self.serial.snapshot() if self.serial else ({},{})
        for key,canvas in self.cells.items():
            value = values.get(key)
            stale = value is None or now-value['received_at']>.5
            volts = value['voltage_v'] if value else None
            canvas.delete('all')
            width = max(100,canvas.winfo_width())
            if stale:
                text,color = f'C{key[1]}  수신 대기 / 오래된 값','#eef2f6'
            elif volts is None:
                text,color = f"C{key[1]}  ADC 오류 {value['status']}",'#fee4e2'
            else:
                strength = min(1,max(0,volts/self.config['vref_v']))
                text = f"C{key[1]}  {volts:.3f} V  · raw {value['raw']}"
                color = f'#{int(230-100*strength):02x}{int(240-65*strength):02x}ff'
            canvas.config(background=color)
            points = histories.get(key,[])[-150:]
            points = [(i,v) for i,(_,v) in enumerate(points) if v is not None]
            if len(points)>1:
                coords = [coordinate for i,v in points for coordinate in
                          (i/149*width,29-v/self.config['vref_v']*12)]
                canvas.create_line(coords,fill='#528bff',width=1)
            canvas.create_text(8,12,text=text,anchor='w',font=('Segoe UI',9))
        if self.serial and self.serial.pressure_active:
            age = now-self.serial.last_pressure_at
            self.pressure_text.set(f'수신 지연 {age*1000:.0f} ms · 누락 {self.serial.tracker.missing} · 추정 전압 V')
        else:
            self.pressure_text.set('압력 측정 정지 · 표시 단위: 추정 전압 V')
        if self.camera and self.camera.ready:
            frame = self.camera.latest()
            if frame is not None:
                image = Image.fromarray(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB))
                image.thumbnail((max(200,self.preview.winfo_width()),max(150,self.preview.winfo_height())))
                self._photo = ImageTk.PhotoImage(image)
                self.preview.config(image=self._photo,text='')
        if self.session:
            if self.scan:
                error = self.session.errors[-1] if self.session.errors else self._scan_source_error()
                if error and self.scan.active:
                    self.scan.abort(error)
                if self.scan.active:
                    self.scan.tick()
                if not self.scan.active:
                    self.stop_recording()
                    self.root.after(100,self.poll)
                    return
            age = (time.perf_counter_ns()-self.session.origin_ns)/1e9
            if self.scan:
                phase = '중립 복귀' if self.scan.returning else {
                    'holding':'준비','arming':'활성화','armed':'준비','sending':'전송',
                    'ack':'응답 대기','moving':'이동','settle':'안정 대기','dwell':'측정',
                    'finish_hold':'종료'}.get(self.scan.stage,self.scan.stage)
                self.record_text.set(f'AUTO {self.scan.info["completed"]}/{self.scan.info["total"]} 완료 · {phase} · {age:.1f}s')
            else:
                self.record_text.set(f"기록 중 {age:.1f}s · 압력 {self.session.counts['pressure_rows']}행 · 영상 {self.session.counts['video_frames']}프레임")
            if self.session.errors:
                if self.scan and self.scan.active:
                    self.scan.abort(self.session.errors[-1])
                self.notice.set('기록 오류: '+self.session.errors[-1])
                self.stop_recording()
            elif self.session.metadata['context']['video'] and self.camera and now-self.camera.last_frame_at>1:
                self.session.error('camera_stale')
                self.stop_recording()
        self.root.after(100,self.poll)

    def close(self):
        self.closing = True
        self.hold()
        errors = []
        for operation in (self.stop_recording,
                          lambda:self.serial.close() if self.serial else None,
                          lambda:self.camera.close() if self.camera else None):
            try:
                operation()
            except Exception as error:
                errors.append(str(error))
        if errors:
            messagebox.showerror('종료 확인','\n'.join(errors),parent=self.root)
        self.root.destroy()


def main():
    root = tk.Tk()
    ControlApp(root)
    root.mainloop()


if __name__=='__main__':
    main()
