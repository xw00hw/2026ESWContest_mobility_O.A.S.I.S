"""Desktop entry point. Launch once, wait for BeamNG, run A and record pressure."""
from __future__ import annotations

import argparse
from contextlib import nullcontext
from dataclasses import replace
import json
import os
from pathlib import Path
import time

from .settings import Settings
from .desktop_session import DesktopSession
from .recording_lifecycle import discover_pending
from .version_context import running_release, versioned_output_root
from .cad_limits import CAD_YAW_MAX_MDEG, CAD_PITCH_MAX_MDEG, cad_pitch_cap_deg
from .tuning import tuning_display_rows


def save_trial_settings(path, settings):
    settings.validate()
    path = Path(path)
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(settings.to_dict(), indent=2, ensure_ascii=False,
                                   allow_nan=False)+'\n', encoding='utf-8')
    temporary.replace(path)


def command_rows(board):
    """Only display actual received command telemetry, never assumed positions."""
    if not board:
        return [('좌측', '—', '—', '—', '—'), ('우측', '—', '—', '—', '—')]
    pose, target = board.get('pose_mdeg'), board.get('target_mdeg')
    if not (isinstance(pose, (list, tuple)) and len(pose) == 4 and
            isinstance(target, (list, tuple)) and len(target) == 4):
        return command_rows(None)
    return [(side, *(f'{pose[i+j]/1000:.3f}°' for j in (0, 1)),
             *(f'{target[i+j]/1000:.3f}°' for j in (0, 1)))
            for side, i in (('좌측', 0), ('우측', 2))]


def vehicle_status_text(status):
    vehicle=status.get('collector',{})
    if status.get('finished'):
        state='수신 중단 · 다시 연결 필요' if status.get('error') else '기록 종료'
    elif status.get('phase') == 'STOPPING':
        state='중지 중'
    else:
        state='수신 중' if vehicle.get('live') else '연결 대기'
    return f"BeamNG: {state} · 수집 {vehicle.get('rows',0):,}행"


def recording_status_text(status):
    retention=status.get('retention','none')
    if retention == 'pending':
        if not status.get('finished') and status.get('phase') == 'RECORD_ONLY':
            return '임시 기록 계속 — 모터 OFF 후에도 차량·압력을 수집합니다. 중지 후 저장 또는 버리기를 선택하세요.'
        return ('임시 보관 (미저장) — 저장 또는 버리기를 선택하세요.' if status.get('finished') else
                '임시 기록 중 — 중지해도 자동 저장하지 않습니다.')
    if retention == 'saved':return '저장 완료'
    if retention == 'discarded':return '버리기 완료'
    return '기록 시작 전'


def window_geometry(screen_width, screen_height):
    """Leave desktop/titlebar room, including Windows display scaling."""
    width=max(320,min(1000,screen_width-80))
    height=max(360,min(780,screen_height-100))
    x=min(100,max(0,(screen_width-width)//2))
    y=min(35,max(0,(screen_height-height)//2))
    return width,height,x,y


def preview_policy_text(settings):
    """Describe configured policy; never imply live vehicle identification."""
    vehicle=settings.metadata.get('declared_vehicle_profile') or {}
    config=vehicle.get('config','')
    if config == 'vehicles/vivace/vivace_160_DCT.pc':vehicle_name='Vivace 160 DCT'
    else:vehicle_name=vehicle.get('model') or '미지정'
    preview=(f'선제 Yaw 최대 {settings.steering_preview_max_yaw_deg:g}°, Pitch 0°'
             if settings.steering_preview_enabled else '선제 움직임 꺼짐')
    return (f'기준 차량: {vehicle_name} · {preview} · 실제 횡가속도 우선 · 가속페달은 기록만\n'
            '큰 횡가속도에서도 목표각도는 설정 상한까지 · 충돌 의심 시 모터 OFF, 차량·압력 기록 계속')


def load_profile(path):
    path=Path(path).resolve()
    profile=json.loads(path.read_text(encoding='utf-8-sig'))
    for key in ('collector_root','settings_path','output_root','source_root','install_root'):
        if key not in profile:continue
        value=Path(profile[key]).expanduser()
        profile[key]=str(value.resolve() if value.is_absolute() else (path.parent/value).resolve())
    profile['release']=running_release()
    return profile,Settings.load(profile['settings_path'])


class InstanceLock:
    """Windows process lock releases even after a crash; no stale PID guesswork."""
    def __init__(self,path): self.path=Path(path);self.handle=None
    def __enter__(self):
        import msvcrt
        self.path.parent.mkdir(parents=True,exist_ok=True)
        self.handle=self.path.open('a+b')
        if self.handle.tell()==0:
            self.handle.write(b'0');self.handle.flush()
        self.handle.seek(0)
        try: msvcrt.locking(self.handle.fileno(),msvcrt.LK_NBLCK,1)
        except OSError:
            self.handle.close();self.handle=None
            raise RuntimeError('BeamNG Bolster A가 이미 실행 중입니다.')
        return self
    def __exit__(self,*unused):
        if self.handle is not None:
            import msvcrt
            self.handle.seek(0)
            msvcrt.locking(self.handle.fileno(),msvcrt.LK_UNLCK,1)
            self.handle.close()


class App:
    def __init__(self,root,profile,settings,*,preview=False):
        import tkinter as tk
        from tkinter import ttk
        self.root,self.profile,self.settings=root,profile,settings
        self.session=None;self.closing=False;self.preview=preview;self._sounded=False
        self._stop_requested=False
        self.restart_required=False
        self.pending_records=[];self.pending_error=None;self._startup_blocked=False
        self._pending_dialog=None
        if not preview:self._refresh_pending_records()
        version=profile.get('release',{}).get('version','DEV')
        root.title(f'BeamNG Bolster A {version} — 자동 제어 · 데이터 수집')
        width,height,x,y=window_geometry(root.winfo_screenwidth(),root.winfo_screenheight())
        root.geometry(f'{width}x{height}+{x}+{y}');root.minsize(min(800,width),min(550,height))
        # Keep stop and record decisions reachable while the detail area scrolls.
        self.footer=ttk.Frame(root,padding=(18,8,18,12));self.footer.pack(side='bottom',fill='x')
        content=ttk.Frame(root);content.pack(side='top',fill='both',expand=True)
        self.content_canvas=tk.Canvas(content,highlightthickness=0)
        scrollbar=ttk.Scrollbar(content,orient='vertical',command=self.content_canvas.yview)
        scrollbar.pack(side='right',fill='y')
        self.content_canvas.configure(yscrollcommand=scrollbar.set)
        self.content_canvas.pack(side='left',fill='both',expand=True)
        frame=ttk.Frame(self.content_canvas,padding=18);self.content_frame=frame
        content_window=self.content_canvas.create_window((0,0),window=frame,anchor='nw')
        frame.bind('<Configure>',lambda event:self.content_canvas.configure(scrollregion=self.content_canvas.bbox('all')))
        self.content_canvas.bind('<Configure>',lambda event:self.content_canvas.itemconfigure(content_window,width=event.width))
        def scroll_wheel(event):
            if event.delta and self.content_canvas.yview() != (0.,1.):
                self.content_canvas.yview_scroll(-int(event.delta/120),'units')
        root.bind('<MouseWheel>',scroll_wheel)
        ttk.Label(frame,text=f'BeamNG Bolster A {version}',font=('맑은 고딕',21,'bold')).pack(anchor='w')
        ttk.Label(frame,text='BeamNG 연결 → 자동 모터 제어 + 차량·압력 기록'+
                  (' · 오른쪽 UR/DR 시험, 왼쪽 목표 0°' if settings.available_bolsters=='right_only' else ''),
                  font=('맑은 고딕',11)).pack(anchor='w',pady=(3,10))
        self.preview_policy=tk.StringVar(value=preview_policy_text(settings))
        ttk.Label(frame,textvariable=self.preview_policy,wraplength=900).pack(anchor='w',pady=(0,8))
        self.message=tk.StringVar(value='시작 준비 중')
        self.vehicle=tk.StringVar(value='BeamNG: 연결 대기')
        self.board=tk.StringVar(value='STM: USB 연결 대기')
        self.pressure=tk.StringVar(value='압력 12채널: 기록 대기 (제어에는 사용하지 않음)')
        self.path=tk.StringVar(value='저장 위치: '+str(versioned_output_root(profile)))
        self.recording=tk.StringVar(value='기록 시작 전')
        self.update_status=tk.StringVar(value='코드 변경 후 업데이트 → 같은 바로가기로 재실행')
        for variable in (self.message,self.vehicle,self.board,self.pressure,self.recording,self.path):
            ttk.Label(frame,textvariable=variable,wraplength=900,
                      font=('맑은 고딕',10)).pack(anchor='w',pady=3)
        status_frame=ttk.LabelFrame(frame,text='STM 명령 상태 — 실제 축 위치·부하 측정값이 아닙니다',padding=8)
        status_frame.pack(fill='x',pady=(6,8))
        self.command_table=ttk.Treeview(status_frame,columns=('side','yaw','pitch','target_yaw','target_pitch'),
                                        show='headings',height=2)
        for key,label in (('side','볼스터'),('yaw','최근 명령 Yaw'),('pitch','최근 명령 Pitch'),
                          ('target_yaw','목표 Yaw'),('target_pitch','목표 Pitch')):
            self.command_table.heading(key,text=label)
            self.command_table.column(key,width=145,anchor='center')
        for row in command_rows(None):self.command_table.insert('', 'end', values=row)
        self.command_table.pack(fill='x')
        self.pwm=tk.StringVar(value='UR / UL / DR / DL PWM: 응답 대기')
        ttk.Label(status_frame,textvariable=self.pwm).pack(anchor='w',pady=(5,0))
        tuning=ttk.LabelFrame(frame,text='STM Live Expressions 적용값 — 이 창에서는 읽기 전용',padding=8)
        tuning.pack(fill='x',pady=(0,10))
        self.tuning_table=ttk.Treeview(tuning,columns=('parameter','variable','value'),show='headings',height=6)
        for key,label,width in (('parameter','항목',200),('variable','g_experiment_a_tune 변수',340),('value','적용값',180)):
            self.tuning_table.heading(key,text=label);self.tuning_table.column(key,width=width,anchor='w')
        for row in tuning_display_rows(None):self.tuning_table.insert('','end',values=row)
        self.tuning_table.pack(fill='x')
        self.tuning_status=tk.StringVar(value='STM 적용 리비전: 연결 전 — 로컬 설정을 실제 적용값으로 표시하지 않습니다.')
        ttk.Label(tuning,textvariable=self.tuning_status,wraplength=870).pack(anchor='w',pady=(5,0))
        ttk.Label(tuning,text='중지 → 저장/버리기 선택 → CubeIDE Live Expressions에서 6개 값 수정 → request_seq를 마지막에 증가 →\n'
                  'applied_seq/결과 확인 → 앱 재실행. 구동 중에는 변경하지 않습니다.',wraplength=870).pack(anchor='w',pady=(5,0))
        self.bound_summary=tk.StringVar()
        ttk.Label(tuning,textvariable=self.bound_summary,wraplength=870).pack(anchor='w',pady=(5,0))
        self._refresh_bounds()
        ttk.Label(frame,text='연결마다 한 번 자동 시작합니다. 통신 오류나 중지 후에는 자동 재시작하지 않습니다.\n'
                  '중지 후 저장 또는 버리기를 선택하세요. 다시 연결은 새 기록으로 유효 데이터 2초 후 자동 시작합니다.',
                  wraplength=900).pack(anchor='w',pady=(0,10))
        buttons=ttk.Frame(self.footer);buttons.pack(fill='x')
        self.stop_button=ttk.Button(buttons,text='중지',command=self.stop,state='disabled')
        self.stop_button.pack(side='left',padx=(0,8))
        self.save_button=ttk.Button(buttons,text='저장',command=self.save_recording,state='disabled')
        self.save_button.pack(side='left',padx=(0,8))
        self.discard_button=ttk.Button(buttons,text='버리기',command=self.discard_recording,state='disabled')
        self.discard_button.pack(side='left',padx=(0,8))
        self.reconnect_button=ttk.Button(buttons,text='다시 연결',command=self.reconnect,state='disabled')
        self.reconnect_button.pack(side='left',padx=(0,8))
        self.pending_button=ttk.Button(buttons,text='미저장 기록',command=self.show_pending,state='disabled')
        self.pending_button.pack(side='left',padx=(0,8))
        utility_buttons=ttk.Frame(self.footer);utility_buttons.pack(fill='x',pady=(8,0))
        ttk.Button(utility_buttons,text='기록 폴더 열기',command=self.open_output).pack(side='left',padx=(0,8))
        self.setup_button=ttk.Button(utility_buttons,text='장치·설정 확인',command=self.setup)
        self.setup_button.pack(side='right')
        self.update_button=ttk.Button(utility_buttons,text='업데이트',command=self.update_release)
        self.update_button.pack(side='left',padx=(0,8))
        ttk.Label(self.footer,textvariable=self.update_status,wraplength=720).pack(anchor='w',pady=(8,0))
        root.protocol('WM_DELETE_WINDOW',self.close)
        if preview:
            self.message.set('미리보기 — 장치 연결·구동 없음. 조정은 STM32CubeIDE Live Expressions에서 진행합니다.')
            self.stop_button.configure(state='disabled');self.setup_button.configure(state='disabled')
            self.update_button.configure(state='disabled')
        elif self._has_pending():
            self._startup_blocked=True
            self._show_pending_notice()
            self.stop_button.configure(state='disabled')
        elif settings.missing_confirmations():
            self.message.set('최초 장착 확인 대기 — 첫 하드웨어 디버깅 때 장치 설정을 완료하세요.')
            self.board.set('모터·압력 통합 펌웨어 업로드와 장착 확인 후 자동 시작 가능')
            self.stop_button.configure(state='disabled')
        else:
            self.setup_button.configure(state='disabled')
            root.after(300,self.start)
        self._refresh_recording_controls()
        root.after(100,self.poll)

    def start(self):
        if self.preview or self.closing or self.session is not None or self.restart_required:return
        self._refresh_pending_records()
        if self._has_pending():
            self._startup_blocked=True;self._show_pending_notice();self._refresh_recording_controls();return
        try:
            self.session=DesktopSession(self.profile,self.settings)
            self.session.start()
            self._stop_requested=False
            self.stop_button.configure(state='normal');self.setup_button.configure(state='disabled')
            self.update_button.configure(state='disabled')
        except Exception as exc:
            self.message.set('시작 실패: '+str(exc))
            self.stop_button.configure(state='disabled')

    def can_reconnect(self, status=None):
        if self.preview or self.closing or self.restart_required or self._has_pending(status):
            return False
        if self.session is None:
            return self._startup_blocked and not self.settings.missing_confirmations()
        return bool((status if status is not None else self.session.snapshot()).get('finished'))

    def reconnect(self):
        """An explicit click creates a new owner/session; never revive a faulted one."""
        self._refresh_pending_records()
        if self._has_pending():self._show_pending_notice();return
        if not self.can_reconnect():return
        self.reconnect_button.configure(state='disabled')
        try:
            settings=Settings.load(self.profile['settings_path'])
            candidate=DesktopSession(self.profile,settings)
            candidate.start()  # Re-check physical confirmations before any worker.
            self.settings=settings;self.session=candidate;self._sounded=False
            self._stop_requested=False
            self.preview_policy.set(preview_policy_text(settings))
            self.stop_button.configure(state='normal');self.setup_button.configure(state='disabled')
            self.update_button.configure(state='disabled')
            self.update_status.set('새 기록으로 연결 중 — 유효 데이터 2초 후 자동 시작합니다.')
        except Exception as exc:
            self.update_status.set('다시 연결 실패: '+str(exc))

    def _refresh_pending_records(self):
        """Discover without resolving records; an unreadable inventory blocks ARM."""
        try:
            self.pending_records=discover_pending(self.profile)
            self.pending_error=None
        except Exception as exc:
            self.pending_error=str(exc)

    def _pending_items(self, status=None):
        current=status if status is not None else (self.session.snapshot() if self.session else {})
        items=[]
        current_id=current.get('recording_id')
        if self.session and current.get('finished') and current.get('retention') == 'pending':
            items.append(self.session)
        for record in self.pending_records:
            recovered=record.snapshot()
            if recovered.get('retention') != 'pending':continue
            if current_id is not None and recovered.get('recording_id') == current_id:continue
            items.append(record)
        return items

    def _has_pending(self, status=None):
        current=status if status is not None else (self.session.snapshot() if self.session else {})
        return bool(self.pending_error or current.get('retention') == 'pending' or self._pending_items(current))

    def _show_pending_notice(self):
        if self.pending_error:
            self.message.set('미저장 기록 목록을 확인하지 못해 자동 시작을 막았습니다: '+self.pending_error)
        else:
            self.message.set('미저장 기록이 있어 자동 시작하지 않습니다. 저장 또는 버리기를 선택한 뒤 다시 연결하세요.')
        self.recording.set('미저장 기록: 임시 보관 중 · 창을 닫아도 다음 실행에서 복구할 수 있습니다.')

    def _refresh_recording_controls(self, status=None):
        current=status if status is not None else (self.session.snapshot() if self.session else {})
        eligible=bool(not self.preview and not self.closing and current.get('finished') and
                      current.get('retention') == 'pending')
        self.save_button.configure(state='normal' if eligible else 'disabled')
        self.discard_button.configure(state='normal' if eligible else 'disabled')
        count=len(self._pending_items(current))
        self.pending_button.configure(text=f'미저장 기록 ({count})',
            state='normal' if not self.preview and not self.closing and (count or self.pending_error) else 'disabled')
        self.reconnect_button.configure(state='normal' if self.can_reconnect(current) else 'disabled')

    def _resolve_recording(self, record, *, discard=False):
        from tkinter import messagebox
        if self.preview or self.closing:return False
        status=record.snapshot()
        if not status.get('finished') or status.get('retention') != 'pending':return False
        if discard and not messagebox.askyesno('임시 기록 버리기',
                f"이 임시 기록을 영구히 버릴까요?\n{status.get('recording_id','')}\n버린 기록은 복구할 수 없습니다.",
                parent=self.root):return False
        try:
            if discard:
                record.discard_recording()
                self.update_status.set('선택한 임시 기록을 버렸습니다. 다시 연결은 직접 눌러야 시작합니다.')
            else:
                saved=record.save_recording()
                self.update_status.set('저장 완료: '+str(saved))
            self._refresh_pending_records()
            if self.session is None:
                self.recording.set('남은 미저장 기록: '+str(len(self._pending_items())))
                if not self._has_pending():
                    self.message.set('미저장 기록 처리가 끝났습니다. 다시 연결을 누르면 새 실험을 시작합니다.')
            self._refresh_recording_controls()
            return True
        except Exception as exc:
            self.update_status.set(('버리기' if discard else '저장')+' 실패 — 임시 기록을 유지합니다: '+str(exc))
            self._refresh_recording_controls()
            return False

    def save_recording(self):
        if self.session is not None:self._resolve_recording(self.session)

    def discard_recording(self):
        if self.session is not None:self._resolve_recording(self.session,discard=True)

    def show_pending(self):
        """Recover stopped/abandoned records without starting any device worker."""
        import tkinter as tk
        from tkinter import ttk
        if self.preview or self.closing:return
        if self._pending_dialog is not None and self._pending_dialog.winfo_exists():
            self._pending_dialog.lift();return
        self._refresh_pending_records()
        dialog=tk.Toplevel(self.root);self._pending_dialog=dialog
        dialog.title('미저장 기록 — 저장 또는 버리기');dialog.geometry('900x380')
        dialog.transient(self.root)
        frame=ttk.Frame(dialog,padding=16);frame.pack(fill='both',expand=True)
        ttk.Label(frame,text='임시 기록을 선택하여 저장 또는 버리기를 결정하세요. 이 창을 닫으면 임시 보관을 유지합니다.',
                  wraplength=850).pack(anchor='w',pady=(0,10))
        table=ttk.Treeview(frame,columns=('record','state','path'),show='headings',selectmode='browse',height=8)
        for key,label,width in (('record','기록 ID',260),('state','상태',150),('path','임시 위치',450)):
            table.heading(key,text=label);table.column(key,width=width,anchor='w')
        table.pack(fill='both',expand=True)
        result=tk.StringVar();ttk.Label(frame,textvariable=result,wraplength=850).pack(anchor='w',pady=8)
        records=[]
        def refresh():
            self._refresh_pending_records();records[:]=self._pending_items()
            for item in table.get_children():table.delete(item)
            for index,record in enumerate(records):
                status=record.snapshot()
                table.insert('','end',iid=str(index),values=(status.get('recording_id',''),
                    '비정상 종료 · 불완전' if status.get('interrupted') else '미저장',status.get('staging_path','')))
            if records:table.selection_set('0')
            result.set('목록 확인 실패: '+self.pending_error if self.pending_error else
                       f'미저장 기록 {len(records)}개 · 처리 후 메인 창에서 다시 연결을 누르세요.')
            self._refresh_recording_controls()
        def resolve(discard):
            selected=table.selection()
            if selected and self._resolve_recording(records[int(selected[0])],discard=discard):refresh()
            else:result.set(self.update_status.get())
        buttons=ttk.Frame(frame);buttons.pack(fill='x')
        ttk.Button(buttons,text='선택 기록 저장',command=lambda:resolve(False)).pack(side='left',padx=(0,8))
        ttk.Button(buttons,text='선택 기록 버리기',command=lambda:resolve(True)).pack(side='left',padx=(0,8))
        ttk.Button(buttons,text='목록 새로고침',command=refresh).pack(side='left',padx=(0,8))
        ttk.Button(buttons,text='닫기 · 임시 보관 유지',command=dialog.destroy).pack(side='right')
        refresh()

    def _refresh_bounds(self,tuning=None):
        text=f'CAD 형상 제한: Yaw ≤ {CAD_YAW_MAX_MDEG/1000:g}°, Pitch ≤ ±{CAD_PITCH_MAX_MDEG/1000:g}°.'
        if tuning is not None:
            yaw=tuning['yaw_max_mdeg']/1000
            cap=min(tuning['pitch_max_mdeg']/1000,cad_pitch_cap_deg(yaw))
            text+=f' STM Yaw 상한 {yaw:g}°에서 적용 Pitch는 최대 ±{cap:g}°입니다.'
        self.bound_summary.set(text)

    def setup(self):
        import tkinter as tk
        from tkinter import ttk,messagebox
        if self.session is not None or self.preview or self.restart_required:return
        dialog=tk.Toplevel(self.root);dialog.title('장치·시험 설정 확인');dialog.transient(self.root);dialog.grab_set()
        frame=ttk.Frame(dialog,padding=18);frame.pack(fill='both',expand=True)
        ttk.Label(frame,text='첫 하드웨어 디버깅에서 확인한 항목만 선택하세요.\n'
                  '한 번 저장하면 이후에는 프로그램 실행 후 자동으로 연결을 기다립니다.\n'
                  '압력 전용 펌웨어는 모터 제어를 지원하지 않습니다.',wraplength=620).pack(anchor='w',pady=(0,12))
        labels={
            'hardware_mapping_checked':('실험 A 펌웨어·CAD 운동축/링크·'+
                ('UR=D3 / DR=D5 장착, UL/DL 미장착' if self.settings.available_bolsters=='right_only'
                 else 'UR=D3 / UL=D4 / DR=D5 / DL=D6')+
                '\n270° 모터의 PWM-축각 보정과 혼 영점 확인'),
            'pitch_direction_checked':'중립 위치와 Pitch 방향·등받이 기준축 확인',
            'right_bolster_fastened':'우측 볼스터 나사를 포함한 링크 체결 확인',
            'neutral_start_clear':'최초 중립 신호를 출력해도 기구 간섭이 없는 상태 확인',
            'trial_settings_reviewed':'STM Live Expressions에서 적용한 각도·속도·응답 설정 및 실행 후 자동 시작 확인',
        }
        variables={key:tk.BooleanVar(value=bool(self.settings.confirmations.get(key))) for key in labels}
        for key,label in labels.items():
            ttk.Checkbutton(frame,text=label,variable=variables[key]).pack(anchor='w',pady=5)
        def save():
            flags={key:var.get() for key,var in variables.items()}
            if not all(flags.values()):
                messagebox.showinfo('장착 확인','확인되지 않은 항목은 첫 하드웨어 디버깅에서 확인해주세요.',parent=dialog)
                return
            candidate=replace(self.settings,confirmations=flags,
                metadata=dict(self.settings.metadata,desktop_setup_acknowledged_wall_ns=time.time_ns()))
            candidate.validate()
            save_trial_settings(self.profile['settings_path'],candidate)
            self.settings=candidate;dialog.destroy();self.start()
        ttk.Button(frame,text='확인 저장 후 연결 대기',command=save).pack(anchor='e',pady=(15,0))

    def stop(self):
        self._stop_requested=True
        if self.session is not None and self.session.is_running():
            self.message.set('모터 중지 중… 기록은 임시 보관하며 자동 저장하지 않습니다.')
            self.session.request_stop()
        self.stop_button.configure(state='disabled')
        self.reconnect_button.configure(state='disabled')
        self._refresh_recording_controls()

    def close(self):
        self.closing=True;self.stop()
        self.message.set('종료 중… 미저장 기록은 임시 보관하며 다음 실행에서 복구할 수 있습니다.')
        if self.session is None or not self.session.is_running():self.root.destroy()

    def open_output(self):
        path=self.session.output if self.session and self.session.output else versioned_output_root(self.profile)
        path=Path(path)
        # Opening a folder must not create a permanent session/version folder.
        while not path.exists() and path != path.parent:path=path.parent
        if not path.is_dir():
            self.update_status.set('열 수 있는 기록 폴더가 없습니다.');return
        os.startfile(str(path))

    def update_release(self):
        if self.preview or (self.session is not None and self.session.is_running()):return
        try:
            from .releases import check_update,publish_release
            source=self.profile['source_root'];install=self.profile['install_root']
            report=check_update(source,install)
            result=publish_release(source,install)
            if result['version'] != self.profile['release']['version']:
                self.restart_required=True
                self.setup_button.configure(state='disabled')
                self.reconnect_button.configure(state='disabled')
                suffix=' 기존 시험 설정은 유지됩니다.' if report.get('baseline_config_changed') else ''
                self.update_status.set(f"{result['version']} 설치 완료. 창을 닫고 같은 바로가기로 다시 실행하세요."+suffix)
            else:
                self.update_status.set(f"현재 {result['version']}이 최신입니다. 버전 번호를 추가하지 않았습니다.")
        except Exception as exc:
            self.update_status.set('업데이트 실패 — 기존 버전 유지: '+str(exc))

    def poll(self):
        if self.session:
            status=self.session.snapshot()
            self.message.set(status['message'])
            self.recording.set(recording_status_text(status))
            self.vehicle.set(vehicle_status_text(status))
            motor=status.get('motor',{})
            tuning=motor.get('firmware_tuning')
            for item,row in zip(self.tuning_table.get_children(),tuning_display_rows(tuning)):
                self.tuning_table.item(item,values=row)
            if tuning is not None:
                self.tuning_status.set(f"STM 적용 리비전: {tuning['revision']} — 이번 실행에 고정하여 사용합니다.")
            self._refresh_bounds(tuning)
            record_only=status['phase']=='RECORD_ONLY'
            off_pending=status['phase']=='IMPACT_OFF_PENDING'
            self.board.set('STM: '+str(motor.get('port','USB 연결 대기'))+
                           (' · 충돌 의심 · 출력 OFF 확인 중' if off_pending else
                            ' · 출력 OFF 확인 · 기록만 계속' if record_only and motor.get('off_verified') else
                            ' · 출력 OFF 미확인' if record_only else
                            ' · 출력 OFF 확인' if status.get('finished') and motor.get('off_verified') else
                            ' · 출력 OFF 미확인' if status.get('finished') else
                            ' · 모터 제어 중' if status['phase']=='CONTROL' else ''))
            board=motor.get('board')
            for item,row in zip(self.command_table.get_children(),command_rows(board)):
                self.command_table.item(item,values=row)
            pulses=board.get('pulses_us') if board else None
            age=max(0.,(time.perf_counter_ns()-board.get('received_ns',0))/1e9) if board else None
            self.pwm.set('UR / UL / DR / DL PWM: '+
                         (' / '.join(str(value) for value in pulses)+' µs' if pulses else '응답 대기')+
                         (' · 충돌 의심: OFF 확인 중' if off_pending else
                          ' · 출력 OFF 확인 · 각도 표시는 마지막 명령값' if record_only and motor.get('off_verified') else
                          ' · 출력 OFF 미확인' if record_only else
                          ' · 최종 출력 OFF 미확인' if status.get('finished') and not motor.get('off_verified') else
                          ' · 마지막 응답: 출력 ON' if board and board.get('armed') else ' · 마지막 응답: OFF 또는 미연결')+
                         (f' · {age:.1f}초 전' if age is not None else ''))
            pressure=motor.get('pressure') or {}
            self.pressure.set(f"압력 기록: {pressure.get('samples',0):,}행 / "
                f"완성 스캔 {pressure.get('complete_sweeps',0):,}회 · {pressure.get('last_reason','대기')}")
            if status.get('output'):
                label='저장 위치: ' if status.get('retention') == 'saved' else '임시 기록 위치: '
                self.path.set(label+status['output'])
            elif status.get('retention') == 'discarded':
                self.path.set('다음 저장 위치: '+str(versioned_output_root(self.profile)))
            if status['phase']=='CONTROL' and not self._sounded:
                self._sounded=True
                try:
                    import winsound
                    winsound.MessageBeep(winsound.MB_OK)
                except (ImportError,RuntimeError):pass
            if status['finished']:
                self.stop_button.configure(state='disabled')
                self.update_button.configure(state='disabled' if self.preview or self.closing else 'normal')
                if self.closing:self.root.destroy();return
            elif (off_pending or record_only) and not self.preview and not self.closing and not self._stop_requested:
                self.stop_button.configure(state='normal')
            self._refresh_recording_controls(status)
        self.root.after(100,self.poll)


def main(argv=None,*,lock_held=False):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile',required=True)
    parser.add_argument('--check-ui',action='store_true',help='Hidden UI construction test; no UART/UDP/worker')
    parser.add_argument('--preview',action='store_true',help='Visible read-only status UI; no UART/UDP/workers')
    args=parser.parse_args(argv)
    import tkinter as tk
    from tkinter import messagebox
    root=tk.Tk()
    if args.check_ui:root.withdraw()
    try:
        with (nullcontext() if lock_held else InstanceLock(Path(args.profile).with_suffix('.lock'))):
            profile,settings=load_profile(args.profile)
            if args.check_ui:
                App(root,profile,settings,preview=True);root.update();root.destroy()
                print('UI check passed; no UART, UDP or workers opened; version='+profile['release']['version'])
                return 0
            App(root,profile,settings,preview=args.preview)
            root.mainloop()
    except Exception as exc:
        if args.check_ui:root.destroy();raise
        messagebox.showerror('BeamNG Bolster A',str(exc),parent=root)
        root.destroy()
        return 1
    return 0


if __name__=='__main__':raise SystemExit(main())
