"""Read-only PC monitor: local JSONL, explicit replay, or SSH tail of Pi telemetry.

No serial, motor, control/config write, or recording paths. HTTP binds loopback.
Pi monotonic timestamps are chart coordinates, never compared with the PC clock.
"""
import argparse
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import re
import shlex
import subprocess
import threading
import time
from urllib.parse import urlsplit
from urllib.request import urlopen
import webbrowser

MAX_LINE = 65536
JOURNAL_PHASES = {'session_created': 'INIT', 'neutral_settle_started': 'NEUTRAL_SETTLE',
                  'neutral_settle_completed': 'BASELINE'}
PAIRS = ('base_yaw_deg', 'rl_delta_yaw_deg', 'requested_yaw_deg', 'filtered_yaw_deg',
         'final_sent_yaw_deg', 'board_command_yaw_deg')


def number(value):
    return float(value) if type(value) in (int, float) and math.isfinite(value) else None


def camera_url(value):
    parsed = urlsplit(value)
    if (parsed.scheme != 'http' or parsed.hostname not in ('127.0.0.1', 'localhost')
            or parsed.username or parsed.password or parsed.path != '/status.json'
            or parsed.query or parsed.fragment):
        raise ValueError('Camera status requires a loopback HTTP /status.json URL')
    return value


def normalize_camera(raw):
    if not isinstance(raw, dict) or raw.get('schema') != 'oasis.pose.camera.status.v1':
        raise ValueError('Unexpected camera status schema')
    result = {'reason': str(raw.get('reason', 'unknown'))[:200]}
    for key in ('inference_fps', 'capture_age_ms', 'last_body_confidence', 'camera_restarts', 'valid_packets'):
        value = number(raw.get(key))
        result[key] = value if value is not None and value >= 0 else None
    for key in ('neutral_provided', 'last_body_core_detected'):
        value = raw.get(key)
        result[key] = value if type(value) is bool else None
    preview = raw.get('preview')
    result['image_fresh'] = bool(isinstance(preview, dict) and preview.get('image_fresh') is True)
    return result


def camera_status_loop(url, monitor, stop, *, opener=urlopen):
    """Independent read-only producer health, never evidence of RL consumption."""
    camera_url(url)
    while not stop.is_set():
        try:
            with opener(url, timeout=.5) as response:
                payload = response.read(65537)
                if len(payload) > 65536:
                    raise ValueError('Oversized camera status')
                monitor.camera_status(normalize_camera(json.loads(payload)))
        except Exception as exc:
            monitor.camera_status(None, f'{type(exc).__name__}: {exc}')
        stop.wait(.25)


SESSION_READER = r'''import json,sys,time
from pathlib import Path
link=Path(sys.argv[1])
while True:
 result={'schema':'oasis.monitor.session.v1','state':'waiting','session_id':None}
 try:
  target=link.resolve(strict=True)
  result['session_id']=target.parent.name
  manifest=target.parent/'manifest.json'
  if manifest.is_file():
   with manifest.open('rb') as stream: data=stream.read(2*1024*1024+1)
   if len(data)>2*1024*1024: raise ValueError('manifest too large')
   value=json.loads(data).get('result')
   if not isinstance(value,dict): raise ValueError('manifest result unavailable')
   result.update(state='ended',phase=str(value.get('phase','unknown'))[:80],
       result=str(value.get('result','unknown'))[:500],off_verified=value.get('off_verified') is True)
  else: result['state']='no_final_manifest'
 except Exception as exc: result.update(state='unavailable',error=(type(exc).__name__+': '+str(exc))[:200])
 print(json.dumps(result),flush=True)
 time.sleep(.5)
'''


def session_ssh_command(prefix, remote_path):
    if not remote_path.startswith('/') or '\n' in remote_path or '\x00' in remote_path:
        raise ValueError('Session status requires an absolute log path')
    return list(prefix) + ['python3 -u -c ' + shlex.quote(SESSION_READER) + ' ' + shlex.quote(remote_path)]


def tail_session_ssh(args, monitor, stop):
    """Separate reader for the final manifest; it never opens a serial device."""
    class Sink:
        def ingest(self, line):
            try:
                raw = json.loads(line)
                if not isinstance(raw, dict) or raw.get('schema') != 'oasis.monitor.session.v1':
                    raise ValueError('Unexpected session status schema')
                state = raw.get('state')
                if state not in ('waiting', 'ended', 'no_final_manifest', 'unavailable'):
                    raise ValueError('Unexpected session state')
                monitor.session_status(dict(state=state,
                    session_id=str(raw['session_id'])[:200] if raw.get('session_id') is not None else None,
                    phase=str(raw.get('phase', 'unknown'))[:80],
                    result=str(raw.get('result', ''))[:500], off_verified=raw.get('off_verified') is True,
                    error=str(raw.get('error', ''))[:200]))
            except (ValueError, TypeError):
                monitor.session_status(None, 'Invalid session status')
        def source_state(self, state, error=None):
            if state != 'receiving': monitor.session_status(None, error)
    tail_ssh(args, Sink(), stop)


def normalize(raw):
    if not isinstance(raw, dict) or raw.get('event') != 'g_control_tick':
        return None
    if isinstance(raw.get('snapshot'), dict):
        raw = dict(raw['snapshot'], event='g_control_tick')
    tick = number(raw.get('monotonic_s'))
    if tick is None or tick < 0:
        return None
    result = dict(monotonic_s=tick, measured_g=number(raw.get('measured_g')),
                  predicted_g=number(raw.get('predicted_g')))
    for key in PAIRS:
        value = raw.get(key)
        result[key] = {side: number(value.get(side)) if isinstance(value, dict) else None
                       for side in ('left', 'right')}
    pressure = raw.get('pressure_counts')
    result['pressure_counts'] = {key: number(pressure.get(key)) if isinstance(pressure, dict) else None
                                 for key in ('UR', 'UL', 'DR', 'DL')}
    guards = raw.get('guards')
    result['guards'] = [str(x)[:100] for x in guards[:20]] if isinstance(guards, list) else []
    result['status'] = str(raw.get('status', 'unknown'))[:200]
    result['policy_mode'] = str(raw.get('policy_mode', raw.get('policy', 'unknown')))[:80]
    result['source'] = str(raw.get('source', 'unspecified'))[:40]
    return result


class Monitor:
    def __init__(self, mode='waiting', *, clock=time.monotonic, stale_s=1., max_rows=1200):
        self.mode, self.clock, self.stale_s = mode, clock, stale_s
        self.rows = deque(maxlen=max_rows)
        self.lock = threading.Lock()
        self.last_received = None
        self.state, self.error, self.errors = 'waiting', None, 0
        self.ignored_events, self.journal_at, self.journal_phase = 0, None, None
        self.session, self.session_at, self.session_error = None, None, None
        self.source_epoch = 0
        self.camera, self.camera_at, self.camera_error = None, None, None

    def camera_status(self, status, error=None):
        with self.lock:
            self.camera = dict(status) if status is not None else None
            self.camera_at = self.clock() if status is not None else None
            self.camera_error = str(error)[:200] if error else None

    def session_status(self, status, error=None):
        with self.lock:
            old_id = self.session.get('session_id') if self.session else None
            new_id = status.get('session_id') if status else None
            if old_id and new_id and old_id != new_id:
                self.rows.clear(); self.last_received = None; self.journal_phase = None
                self.source_epoch += 1
            self.session = dict(status) if status is not None else None
            self.session_at = self.clock() if status is not None else None
            self.session_error = str(error)[:200] if error else None

    def source_state(self, state, error=None):
        with self.lock:
            self.state, self.error = state, str(error)[:200] if error else None

    def ingest(self, line):
        if len(line) > MAX_LINE:
            with self.lock:
                self.errors += 1
            return False
        try:
            raw = json.loads(line)
        except (ValueError, TypeError, UnicodeDecodeError):
            raw = None
        if not isinstance(raw, dict) or not isinstance(raw.get('event'), str):
            with self.lock:
                self.errors += 1
            return False
        if raw['event'] != 'g_control_tick':
            with self.lock:
                self.ignored_events += 1
                self.journal_at = self.clock()
                if raw['event'] == 'session_created':
                    self.rows.clear(); self.last_received = None
                if raw['event'] in JOURNAL_PHASES:
                    self.journal_phase = JOURNAL_PHASES[raw['event']]
            return False
        data = normalize(raw)
        if data is None:
            with self.lock:
                self.errors += 1
            return False
        with self.lock:
            # Reboot/new run: never draw a line through unrelated Pi clocks.
            if self.rows and data['monotonic_s'] < self.rows[-1]['monotonic_s']:
                self.rows.clear()
                self.source_epoch += 1
            # Chart at 10 Hz while the latest values retain every received tick.
            # This keeps a full 60 s view even when the controller logs at 50 Hz.
            if self.rows and int(data['monotonic_s'] * 10) == int(self.rows[-1]['monotonic_s'] * 10):
                self.rows[-1] = data
            else:
                self.rows.append(data)
            cutoff = data['monotonic_s'] - 60.
            while self.rows and self.rows[0]['monotonic_s'] < cutoff:
                self.rows.popleft()
            self.last_received = self.clock()
            self.journal_at, self.journal_phase = self.last_received, 'CONTROL'
            self.state, self.error = 'receiving', None
        return True

    def snapshot(self):
        with self.lock:
            age = None if self.last_received is None else max(0., self.clock() - self.last_received)
            fresh = age is not None and age < self.stale_s and self.state == 'receiving'
            camera_age = None if self.camera_at is None else max(0., self.clock() - self.camera_at)
            camera_fresh = camera_age is not None and camera_age < 1.5
            session_age = None if self.session_at is None else max(0., self.clock() - self.session_at)
            session_fresh = session_age is not None and session_age < 1.5
            journal_age = None if self.journal_at is None else max(0., self.clock() - self.journal_at)
            return dict(schema='oasis.rl.monitor.v1', mode=self.mode, source_state=self.state,
                        connection='receiving' if fresh else ('waiting' if age is None else 'stale'),
                        fresh=fresh, age_s=age, stale_s=self.stale_s, error=self.error,
                        invalid_lines=self.errors, ignored_events=self.ignored_events, source_epoch=self.source_epoch,
                        session=dict(status=dict(self.session) if session_fresh and self.session else None,
                            fresh=session_fresh, age_s=session_age, error=self.session_error,
                            journal_phase=self.journal_phase, journal_age_s=journal_age),
                        latest=dict(self.rows[-1]) if fresh and self.rows else None,
                        history=list(self.rows), control_authority=False, recording=False,
                        board_values='reported command state; not measured shaft position',
                        camera=dict(status=dict(self.camera) if camera_fresh and self.camera else None,
                            fresh=camera_fresh, age_s=camera_age, error=self.camera_error,
                            rl_consumption='not_reported_by_control_log'))


def bounded_lines(stream):
    """Drop oversized records without allocating an unbounded line."""
    while True:
        line = stream.readline(MAX_LINE + 1)
        if not line:
            return
        if len(line) > MAX_LINE:
            while line and not line.endswith(b'\n'):
                line = stream.readline(MAX_LINE + 1)
            continue
        yield line


def tail_file(path, monitor, stop):
    """Only appended rows count as live; existing historical rows are skipped."""
    while not stop.is_set():
        try:
            with Path(path).open('rb') as stream:
                stream.seek(0, 2)
                monitor.source_state('waiting')
                pending = b''
                dropping = False
                while not stop.is_set():
                    chunk = stream.readline(MAX_LINE + 1)
                    if not chunk:
                        if Path(path).stat().st_size < stream.tell():
                            break
                        stop.wait(.1)
                        continue
                    if dropping or len(pending) + len(chunk) > MAX_LINE:
                        pending = b''
                        dropping = not chunk.endswith(b'\n')
                        continue
                    pending += chunk
                    if pending.endswith(b'\n'):
                        monitor.ingest(pending)
                        pending = b''
        except OSError as exc:
            monitor.source_state('waiting', f'{type(exc).__name__}: {exc}')
            stop.wait(1.)


def replay_file(path, monitor, stop, speed=1.):
    previous = None
    try:
        with Path(path).open('rb') as stream:
            for line in bounded_lines(stream):
                try:
                    record = normalize(json.loads(line))
                except (ValueError, TypeError):
                    record = None
                if record is None:
                    continue
                stamp = record['monotonic_s']
                if previous is not None and stop.wait(min(5., max(0., stamp - previous) / speed)):
                    return
                if stop.is_set():
                    return
                monitor.ingest(line)
                previous = stamp
        monitor.source_state('replay_complete')
    except OSError as exc:
        monitor.source_state('replay_error', exc)


def ssh_command(host, user, remote_path, identity=None, known_hosts=None):
    if not host or host.startswith('-') or any(c.isspace() for c in host) or not re.fullmatch(r'[a-zA-Z0-9_.-]+', user):
        raise ValueError('invalid SSH host/user')
    if not remote_path.startswith('/') or '\n' in remote_path or '\x00' in remote_path:
        raise ValueError('SSH log requires an absolute remote path')
    args = ['ssh', '-T', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5',
            '-o', 'ServerAliveInterval=5', '-o', 'ServerAliveCountMax=2']
    if identity:
        args += ['-i', str(identity)]
    if known_hosts:
        args += ['-o', 'UserKnownHostsFile=' + str(known_hosts)]
    return args + [user + '@' + host, 'tail -n 0 -F -- ' + shlex.quote(remote_path)]


def tail_ssh(args, monitor, stop):
    """Read-only tail; BatchMode uses preconfigured SSH auth, no password storage."""
    while not stop.is_set():
        process = None
        try:
            process = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                       creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
            monitor.source_state('waiting')
            def interrupt(proc):
                while proc.poll() is None:
                    if stop.wait(.2):
                        if proc.poll() is None:
                            proc.terminate()
                        return
            watcher = threading.Thread(target=interrupt, args=(process,), daemon=True)
            watcher.start()
            for line in bounded_lines(process.stdout):
                if stop.is_set():
                    break
                monitor.ingest(line)
            monitor.source_state('disconnected', 'SSH ended; check host, key, path and Pi availability')
        except OSError as exc:
            monitor.source_state('disconnected', exc)
        finally:
            if process is not None:
                if process.poll() is None:
                    process.terminate()
                try:
                    process.wait(1.)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(1.)
                if process.stdout:
                    process.stdout.close()
        stop.wait(2.)


def demo(monitor, stop):
    """Synthetic visualization only. Never sent to any controller."""
    start = time.monotonic()
    while not stop.is_set():
        t = time.monotonic() - start
        measured, predicted = .35 * math.sin(t * .5), .35 * math.sin((t + .3) * .5)
        base = dict(left=max(0., -measured) * 34, right=max(0., measured) * 34)
        delta = {side: (math.sin(t) * .65 if base[side] > .1 else 0.) for side in base}
        sent = {side: max(0., base[side] + delta[side]) for side in base}
        monitor.ingest(json.dumps(dict(event='g_control_tick', monotonic_s=t,
            measured_g=measured, predicted_g=predicted, base_yaw_deg=base,
            rl_delta_yaw_deg=delta, final_sent_yaw_deg=sent, board_command_yaw_deg=sent,
            requested_yaw_deg=sent, filtered_yaw_deg=sent, source='simulation',
            pressure_counts=dict(UR=900+sent['right']*30, DR=1000+sent['right']*25,
                                 UL=850+sent['left']*30, DL=950+sent['left']*25),
            guards=['합성 예시 · 실제 모터 연결 없음'], status='synthetic_demo', policy_mode='G base + RL residual')))
        stop.wait(.1)


PAGE = r'''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>BeamNG RL Monitor</title><style>
:root{color-scheme:dark}*{box-sizing:border-box}body{margin:0;background:#0d1520;color:#e3edf7;font:15px system-ui,sans-serif}
main{max-width:1440px;margin:auto;padding:24px}header{display:flex;align-items:center;justify-content:space-between;gap:20px}h1{font-size:25px;margin:0 0 7px}p{margin:6px 0;color:#98aec5}.badge{padding:10px 15px;background:#5b3b16;border-radius:8px;font-weight:700;color:#ffcd78}.live{background:#163e34;color:#8ce2bc}
.grid{display:grid;grid-template-columns:1.1fr 1fr;gap:16px;margin-top:18px}.panel{background:#162331;border:1px solid #293d50;border-radius:12px;padding:18px}.metrics{display:flex;gap:30px}.number{font-size:34px;font-variant-numeric:tabular-nums;font-weight:650}.label{color:#a5bdd4;font-size:13px}.cyan{color:#61d6f7}.purple{color:#bd9aff}canvas{width:100%;height:210px;display:block;margin-top:12px}table{width:100%;border-collapse:collapse}td,th{text-align:right;padding:12px 6px;border-bottom:1px solid #2b3e4f}th:first-child,td:first-child{text-align:left}thead{font-size:12px;color:#9eb8ce}.pressure{display:grid;grid-template-columns:repeat(4,1fr);gap:12px;margin-top:15px}.bar{background:#293e4f;height:8px;border-radius:4px;margin-top:12px;overflow:hidden}.fill{background:#61d6f7;height:100%;width:0}.small{font-size:12px;color:#93abc2}.guard{min-height:36px;color:#ffc779;white-space:pre-wrap;overflow-wrap:anywhere}.status{margin-top:15px;display:flex;justify-content:space-between;gap:16px}.unavailable{color:#ffcb83}.legend{font-size:12px;color:#a8bed0}.wide{grid-column:1/-1}@media(max-width:850px){.grid{grid-template-columns:1fr}.wide{grid-column:1}.pressure{grid-template-columns:repeat(2,1fr)}header{align-items:flex-start}.number{font-size:29px}}
</style><main><header><div><h1>BeamNG · RL 제어 모니터</h1><p>관찰 전용 · 차량 G → 기본 각도 → RL 보정 → 전송 명령</p></div><div class="badge" id="mode">연결 대기</div></header>
<div class="status"><span id="connection" class="unavailable">새 제어 기록을 기다리는 중</span><span class="small" id="age">현재 값 없음</span></div>
<section class="panel" style="margin-top:16px"><div class="label">시험 상태</div><p id="session_state">세션 상태 수신 대기</p><p class="small" id="session_detail">—</p></section>
<section class="panel" style="margin-top:18px"><div class="label">준비한 G 기본 곡선 · 1g / 20° 실험 설정 · 필터 전 목표</div>
<svg viewBox="0 0 960 170" role="img" aria-label="0.0625g에서 10도, 0.5g에서 14.7도, 1g에서 20도" style="width:100%;max-height:170px">
<path d="M50 145H930 M50 85H930 M50 25H930" stroke="#2a3d4d" fill="none"/>
<polyline points="50,145 58,145 100,85 450,57 850,25 930,25" stroke="#61d6f7" stroke-width="3" fill="none"/>
<g fill="#a5bdd4" font-size="14"><text x="10" y="149">0°</text><text x="8" y="89">10°</text><text x="8" y="29">20°</text><text x="45" y="165">0g</text><text x="100" y="75">0.0625g → 10°</text><text x="420" y="48">0.5g → 14.7°</text><text x="790" y="17">1g → 20°</text></g></svg>
<p class="small">예측 단독 최대 10° · RL 보정 최대 ±1.5° (1g에서 0°) · Pitch 0°. 실제 실행 값은 아래 로그에 표시하며, 곡선은 필터·속도 제한 전 설정입니다.</p></section>
<section class="panel" style="margin-top:16px"><div class="label">카메라 생산자 상태 · 제어 입력과 별도 관찰</div><p id="camera_state">카메라 상태 수신 대기</p><p class="small" id="camera_detail">—</p><p class="small">RL의 실제 카메라 입력 사용 여부: 현재 제어 로그에 별도 보고되지 않음. 아래 상태는 영상·자세 생산자의 상태이며, RL 사용 확인을 뜻하지 않습니다.</p></section>
<div class="grid"><section class="panel"><div class="metrics"><div><div class="label">측정 횡가속도</div><div id="measured" class="number cyan">—</div></div><div><div class="label">예측 횡가속도</div><div id="predicted" class="number purple">—</div></div></div><canvas id="gchart"></canvas><div class="legend">● <span class="cyan">측정 G</span> &nbsp; ● <span class="purple">예측 G</span> · 최근 60초 · +y 차량 왼쪽</div></section>
<section class="panel"><table><thead><tr><th>Yaw 제어 단계</th><th>왼쪽 °</th><th>오른쪽 °</th></tr></thead><tbody id="angles"></tbody></table><p class="small">보드 보고값은 명령 상태입니다. 실제 축 위치를 측정한 값이 아닙니다.</p></section>
<section class="panel"><div class="label">좌우 최종 전송 대상과 기본각</div><canvas id="anglechart"></canvas><div class="legend">청록: 왼쪽 / 보라: 오른쪽 · 실선: 최종 전송 대상 / 점선: 기본각</div></section>
<section class="panel"><div class="label">측면 압력 원시 ADC · count</div><div class="pressure" id="pressure"></div><p class="small">막대 표시 범위 0–4095 count. 힘 단위 환산이나 보호 임계값 표시가 아닙니다.</p><div class="label" style="margin-top:22px">보호 조건 · 제어 상태</div><div class="guard" id="guards">—</div><p class="small" id="policy">—</p></section></div>
<p class="small" style="margin-top:18px">최종 전송 대상은 제한 적용 후 큐에 넣은 값이며 전송 ACK나 실제 축 위치가 아닙니다. 데이터가 끊기면 현재 수치는 — 로 전환합니다. 그래프는 과거 기록입니다. 이 창은 명령 전송·저장 기능이 없습니다.</p></main><script>
const $=id=>document.getElementById(id), names=[['base_yaw_deg','G 기반 기본각'],['rl_delta_yaw_deg','RL 각도 보정'],['requested_yaw_deg','필터 전 목표'],['filtered_yaw_deg','필터 후 목표'],['final_sent_yaw_deg','최종 전송 대상'],['board_command_yaw_deg','보드 보고 명령 상태']];
$('angles').innerHTML=names.map(([key,name])=>`<tr><td>${name}</td><td id="${key}_left">—</td><td id="${key}_right">—</td></tr>`).join('');
$('pressure').innerHTML=['UR','UL','DR','DL'].map(k=>`<div><div class="label">${k}</div><div id="p_${k}" class="number" style="font-size:25px">—</div><div class="bar"><div id="bar_${k}" class="fill"></div></div></div>`).join('');
let deadline=0, history=[];
function fmt(v,n=2){return Number.isFinite(v)?v.toFixed(n):'—'}
function values(v){$('measured').textContent=v?fmt(v.measured_g,3)+' g':'—';$('predicted').textContent=v?fmt(v.predicted_g,3)+' g':'—';
for(const [key] of names)for(const side of ['left','right'])$(key+'_'+side).textContent=fmt(v?.[key]?.[side]);
for(const k of ['UR','UL','DR','DL']){const p=v?.pressure_counts?.[k];$('p_'+k).textContent=fmt(p,0);$('bar_'+k).style.width=(Number.isFinite(p)?Math.max(0,Math.min(100,p/4095*100)):0)+'%';}
$('guards').textContent=v?(v.guards.length?v.guards.join(' · '):'활성 보호 조건 없음 / 로그 기준'):'현재 상태 없음';$('policy').textContent=v?'모드: '+v.policy_mode+' · '+v.status:'—';}
function cameraValues(camera){const s=camera?.fresh?camera.status:null;if(!s){$('camera_state').textContent='카메라 상태 수신 대기 / 최신 상태 없음';$('camera_detail').textContent=camera?.error||'—';return;}
const yes=v=>v===true?'확인':v===false?'미확인':'—';$('camera_state').textContent=(s.image_fresh?'영상 수신 중':'새 영상 대기')+' · '+s.reason;
$('camera_detail').textContent='추론 '+fmt(s.inference_fps,1)+' FPS · 어깨·골반 검출 '+yes(s.last_body_core_detected)+' · 중립 보정 파일 '+yes(s.neutral_provided)+' · 신뢰도 '+fmt(s.last_body_confidence,2)+' · 카메라 재시작 '+fmt(s.camera_restarts,0)+'회';}
function sessionValues(session,controlFresh=false){const value=session?.fresh?session.status:null,labels={INIT:'세션 준비',HANDSHAKE:'보드 연결 확인',QUALIFY:'입력 확인',ARM:'중립 출력 적용',NEUTRAL_SETTLE:'중립 유지',BASELINE:'압력 기준선 측정',CONTROL:'RL 제어'};
if(value?.state==='ended'){$('session_state').textContent='시험 종료 · '+(labels[value.phase]||value.phase);$('session_detail').textContent=value.result+' · '+(value.off_verified?'PWM OFF 확인됨 (종료 기록)':'PWM OFF 확인 정보 없음');return;}
const phase=session?.journal_phase,recent=Number.isFinite(session?.journal_age_s)&&session.journal_age_s<1.5;
$('session_state').textContent=controlFresh?'RL 제어 실행 중':recent&&phase==='CONTROL'?'제어각 데이터 대기 · 마지막 단계 RL 제어':recent&&phase?(labels[phase]||phase):phase?'새 상태 대기 · 마지막 단계 '+(labels[phase]||phase):'세션 상태 대기';
$('session_detail').textContent=(value?.session_id?'세션 '+value.session_id+' · ':'')+(session?.error||'단계는 수신한 이벤트 기준이며, 종료 결과는 세션 기록으로 확인합니다.');}
function chart(id,series){const canvas=$(id),r=canvas.getBoundingClientRect(),d=window.devicePixelRatio||1;canvas.width=r.width*d;canvas.height=r.height*d;const c=canvas.getContext('2d');c.scale(d,d);const w=r.width,h=r.height,L=38,R=12,T=15,B=26;if(!history.length)return;
const last=history.at(-1).monotonic_s,start=last-60;let vals=[];for(const row of history)for(const s of series){const v=s.get(row);if(Number.isFinite(v))vals.push(v)}let lo=Math.min(0,...vals),hi=Math.max(1,...vals);const pad=(hi-lo)*.1;lo-=pad;hi+=pad;const x=t=>L+(t-start)/60*(w-L-R),y=v=>T+(hi-v)/(hi-lo)*(h-T-B);c.font='11px system-ui';
for(let i=0;i<4;i++){const v=lo+(hi-lo)*i/3,yy=y(v);c.strokeStyle='#2a3d4d';c.beginPath();c.moveTo(L,yy);c.lineTo(w-R,yy);c.stroke();c.fillStyle='#89a3b9';c.fillText(v.toFixed(1),0,yy+4)}
for(const s of series){c.strokeStyle=s.color;c.lineWidth=2;c.setLineDash(s.dash||[]);c.beginPath();let connected=false;for(const row of history){const v=s.get(row);if(!Number.isFinite(v)){connected=false;continue}if(connected)c.lineTo(x(row.monotonic_s),y(v));else c.moveTo(x(row.monotonic_s),y(v));connected=true}c.stroke()}c.setLineDash([]);c.fillStyle='#89a3b9';c.fillText('−60 s',L,h-4);c.fillText('최신 기록',w-65,h-4);}
function charts(){chart('gchart',[{get:r=>r.measured_g,color:'#61d6f7'},{get:r=>r.predicted_g,color:'#bd9aff'}]);chart('anglechart',[{get:r=>r.base_yaw_deg.left,color:'#61d6f7',dash:[4,4]},{get:r=>r.base_yaw_deg.right,color:'#bd9aff',dash:[4,4]},{get:r=>r.final_sent_yaw_deg.left,color:'#61d6f7'},{get:r=>r.final_sent_yaw_deg.right,color:'#bd9aff'}]);}
setInterval(()=>{if(performance.now()>deadline){values(null);$('connection').textContent='연결 대기 / 최신 데이터 없음';$('connection').className='unavailable'}},100);
async function poll(){const ctrl=new AbortController(),timeout=setTimeout(()=>ctrl.abort(),1200),began=performance.now();try{const response=await fetch('/status.json',{cache:'no-store',signal:ctrl.signal});if(!response.ok)throw Error(response.status);const s=await response.json();
const replay=s.mode==='replay'||s.mode==='demo',source=s.history?.at(-1)?.source,live=!replay&&source==='live';$('mode').textContent=s.mode==='demo'?'합성 데모 · 실제 장치 없음':replay?'기록 재생 · LIVE 아님':source==='simulation'?'시뮬레이션 · 실물 LIVE 아님':live&&s.fresh?'LIVE · 읽기 전용':source&&source!=='live'?'입력 출처 미확인':'LIVE 연결 대기';$('mode').className='badge'+(live&&s.fresh?' live':'');
const remaining=s.fresh?Math.max(0,(s.stale_s-s.age_s)*1000-(performance.now()-began)):0;deadline=performance.now()+remaining;values(remaining>0?s.latest:null);cameraValues(s.camera);sessionValues(s.session,remaining>0);history=s.history||[];charts();
$('connection').textContent=s.fresh?(replay?'재생 중 · 모터 명령 없음':'Pi 제어 로그 수신 중'):('대기 / '+s.source_state+(s.error?' · '+s.error:''));$('connection').className=s.fresh?'':'unavailable';$('age').textContent=s.age_s==null?'현재 값 없음':'마지막 수신 '+s.age_s.toFixed(1)+'초 전 · 과거 그래프 유지';
}catch(e){deadline=0;values(null);cameraValues(null);sessionValues(null);$('connection').textContent='모니터 서버 연결 끊김';$('connection').className='unavailable'}finally{clearTimeout(timeout)}setTimeout(poll,200)}poll();window.addEventListener('resize',charts);
</script></html>'''


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_):
        pass
    def do_GET(self):
        try:
            host = urlsplit('http://' + self.headers.get('Host', '')).hostname
        except ValueError:
            host = None
        if host not in ('127.0.0.1', 'localhost'):
            self.send_error(403)
            return
        if self.path == '/':
            data, kind = PAGE.encode('utf-8'), 'text/html; charset=utf-8'
        elif self.path == '/status.json':
            data, kind = json.dumps(self.server.monitor.snapshot(), allow_nan=False).encode(), 'application/json'
        else:
            self.send_error(404)
            return
        try:
            self.send_response(200)
            for key,value in [('Content-Type',kind),('Content-Length',str(len(data))),('Cache-Control','no-store'),
                              ('X-Content-Type-Options','nosniff'),('X-Frame-Options','DENY'),('Cross-Origin-Resource-Policy','same-origin')]:
                self.send_header(key,value)
            self.end_headers()
            self.wfile.write(data)
        except OSError:
            pass


class Server(ThreadingHTTPServer):
    daemon_threads = True
    block_on_close = False
    request_queue_size = 8
    def __init__(self, port, monitor):
        self.monitor = monitor
        self.slots = threading.BoundedSemaphore(8)
        super().__init__(('127.0.0.1', port), Handler)
    def process_request(self, request, address):
        request.settimeout(2.)
        if not self.slots.acquire(False):
            self.shutdown_request(request)
            return
        try:
            super().process_request(request,address)
        except Exception:
            self.slots.release()
            raise
    def process_request_thread(self, request, address):
        try:
            super().process_request_thread(request,address)
        finally:
            self.slots.release()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group()
    source.add_argument('--file', type=Path)
    source.add_argument('--replay', type=Path)
    source.add_argument('--ssh-host')
    source.add_argument('--demo', action='store_true')
    parser.add_argument('--ssh-user', default='esw')
    parser.add_argument('--ssh-log')
    parser.add_argument('--ssh-identity', type=Path)
    parser.add_argument('--ssh-known-hosts', type=Path)
    parser.add_argument('--port', type=int, default=8766)
    parser.add_argument('--speed', type=float, default=1.)
    parser.add_argument('--open-browser', action='store_true')
    parser.add_argument('--camera-status-url', type=camera_url,
                        help='Optional loopback camera producer status, separate from RL input validity')
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535 or not math.isfinite(args.speed) or not .1 <= args.speed <= 20:
        parser.error('invalid port/replay speed')
    if args.ssh_host and not args.ssh_log:
        parser.error('--ssh-host requires --ssh-log absolute path')
    mode = 'demo' if args.demo else 'replay' if args.replay else 'live' if args.file or args.ssh_host else 'waiting'
    monitor, stop = Monitor(mode), threading.Event()
    target = values = None
    if args.demo:
        target, values = demo, (monitor,stop)
    elif args.file:
        target, values = tail_file, (args.file,monitor,stop)
    elif args.replay:
        target, values = replay_file, (args.replay,monitor,stop,args.speed)
    elif args.ssh_host:
        try:
            command = ssh_command(args.ssh_host,args.ssh_user,args.ssh_log,args.ssh_identity,args.ssh_known_hosts)
        except ValueError as exc:
            parser.error(str(exc))
        target, values = tail_ssh, (command,monitor,stop)
    server = Server(args.port, monitor)
    if target:
        threading.Thread(target=target,args=values,name='read-only-log-source',daemon=True).start()
    if args.camera_status_url:
        threading.Thread(target=camera_status_loop,args=(args.camera_status_url,monitor,stop),
                         name='read-only-camera-status',daemon=True).start()
    url = f'http://127.0.0.1:{args.port}/'
    print(f'BeamNG RL Monitor ({mode}, read-only): {url}',flush=True)
    if args.open_browser:
        webbrowser.open(url)
    try:
        server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        server.server_close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
