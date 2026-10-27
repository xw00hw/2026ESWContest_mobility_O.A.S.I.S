"""Software stand-in for the NUCLEO-H723ZG running OASIS_FALLBACK v2 (BeamNG_Bolster_FB stm32, -FallbackControl),
for SIL tests on a PC / the Pi. Follows Core/Src/fallback_control.c + fallback_tuning.c + joint_math.c:
  * CONTROL_HELLO v2 (270 deg), CAD_INFO, TUNE_STATE, STATE (18 fields), FB_OFF, FB_ARM,<sid>,<tune rev>,
    FB_SET,<sid>,<id>,LY,LP,RY,RP, FB_PING,<sid>,<n>; FSR: HELLO, START_12,15, DISABLE, PING -> #PONG,12
  * every reject while armed -> PWM OFF ("PROTOCOL"); CAD / TUNE violations -> CAD_BOUNDS / TUNE_BOUNDS fault
  * 20 ms tracking: per-axis tune rate (common fraction), then per-servo 2 us / frame (Joint_LimitStep bisection)
  * motor lease 500 ms (FB_ARM / FB_SET / FB_PING only) -> LEASE fault; #STATE telemetry 5 Hz
  * pressure: 4 rows (C0..C3, banks A0..A2) per 10 ms sweep while scanning; `pressure_fn(pose)` -> 12 counts
Simulated time: `clock` (MockClockNs); the board advances to clock.now_ns whenever the host reads.
It is a protocol / kinematics twin, not evidence of real servo, pressure or timing behaviour."""
import math, os, sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
from oasis_rt import fbstack  # noqa: E402

FB = fbstack.load()
T, KIN, CAD = FB.transport, FB.kinematics, FB.cad_limits


class MockClockNs:
    def __init__(self, t0_ns=1_000_000_000): self.now_ns = int(t0_ns)
    def __call__(self): return self.now_ns
    def sleep(self, dt): self.now_ns += max(1, int(dt * 1e9))


def default_pressure(pose):
    """rest 1000 counts; a closing wing raises its two side pads (A1: C0 DR, C1 UR, C2 DL, C3 UL)."""
    p = {f'A{b}_C{c}': 1000 for b in range(3) for c in range(4)}
    for (lo, hi), yaw in ((('A1_C2', 'A1_C3'), pose[0]), (('A1_C0', 'A1_C1'), pose[2])):
        extra = int(40 * max(0.0, yaw / 1000 - 3.0)); p[lo] += extra; p[hi] += extra // 2
    return p


def _pulses(pose):
    try:
        f = KIN.pose_to_pulses_us([v / 1000 for v in pose])
    except ValueError:
        return None
    return tuple(int(math.floor(x + 0.5)) for x in f)


class MockFbBoard:
    timeout = write_timeout = 0

    def __init__(self, clock, tune=None, pressure_fn=default_pressure, revision=0):
        self.clock = clock
        self.tune = dict(yaw_max_mdeg=15000, pitch_max_mdeg=6000, yaw_rate_mdeg_s=2000, pitch_rate_mdeg_s=2000,
                         full_scale_ay_mmps2=2942, pitch_scale_permille=1000)        # fallback_tuning.c DEFAULT_VALUES
        self.tune.update(tune or {})
        self.revision = revision
        self.buf = bytearray(); self.commands = []; self.closed = False
        self.armed = self.moving = False; self.fault = 'BOOT'
        self.pose = [0, 0, 0, 0]; self.target = [0, 0, 0, 0]; self.pulses = [0, 0, 0, 0]
        self.floor = self.session = self.command_id = self.ping = 0
        self.t = clock.now_ns; self.last_alive = self.last_update = self.last_status = self.t
        self.scan = False; self.p_seq = 0; self.p_next = self.t; self.pressure_fn = pressure_fn
        self.max_pulse_step = 0; self.lease_faults = 0

    # ---------------------------------------------------------------- transport
    def emit(self, line): self.buf.extend((line + '\r\n').encode('ascii'))

    def write(self, data):
        self._advance()
        for cmd in data.decode('ascii').split('\n'):
            cmd = cmd.strip()
            if cmd: self.commands.append((self.clock.now_ns, cmd)); self._command(cmd)
        return len(data)

    @property
    def in_waiting(self):
        self._advance(); return len(self.buf)

    def read(self, n):
        v = bytes(self.buf[:n]); del self.buf[:n]; return v

    def reset_input_buffer(self): self.buf.clear()
    def close(self): self.closed = True

    # ---------------------------------------------------------------- firmware
    def _off(self, reason):
        self.armed = self.moving = False; self.session = 0; self.pulses = [0, 0, 0, 0]
        self.target = list(self.pose); self.fault = reason

    def _state(self):
        f = (self.t // 1_000_000, int(self.armed), int(self.moving), *self.pose, *self.target, *self.pulses,
             self.command_id, self.fault)
        return '#STATE,' + ','.join(map(str, f))

    def _reject(self, cid, reason):
        if self.armed: self._off('PROTOCOL')
        self.emit(f'#ERR,{cid},{reason}')

    def _tune_ok(self, pose):
        return all(0 <= pose[i] <= self.tune['yaw_max_mdeg'] and abs(pose[i + 1]) <= self.tune['pitch_max_mdeg'] for i in (0, 2))

    def _command(self, cmd):
        f = cmd.split(',')
        if self.armed and self.t - self.last_alive >= 500_000_000: self._off('LEASE'); self.lease_faults += 1
        if cmd == 'CONTROL_HELLO':
            self.emit('#HELLO,OASIS_FALLBACK,v2,' + ','.join(f'{k}={v}' for k, v in T.HELLO_FIELDS.items())
                      + f',session_floor={self.floor}')
        elif cmd == 'CAD_INFO': self.emit(CAD.CAD_INFO_LINE)
        elif cmd == 'TUNE_STATE':
            self.emit(f'#TUNE,OASIS_A,v1,revision={self.revision},' + ','.join(f'{k}={v}' for k, v in self.tune.items()))
        elif cmd == 'STATE': self.emit(self._state())
        elif cmd == 'FB_OFF':
            self._off('OFF' if self.fault in ('NONE', 'BOOT') else self.fault); self.emit('#ACK,FB_OFF')
        elif cmd == 'HELLO': self.emit(FB.serial_link.FSR_HELLO)
        elif cmd == 'START_12,15': self.scan = True; self.p_next = self.t; self.emit('#CONTROL,12,MASK=15,EN=0')
        elif cmd == 'DISABLE': self.scan = False; self.emit('#CONTROL,12,MASK=0,EN=1')
        elif cmd == 'PING': self.emit('#PONG,12')
        elif f[0] == 'FB_ARM':
            sid, rev = int(f[1]), int(f[2])
            if self.armed: return self._reject(0, 'ALREADY_ARMED')
            if rev != self.revision: return self._reject(0, 'TUNE_REVISION')
            if sid <= self.floor: return self._reject(0, 'SESSION_REPLAY')
            self.floor = self.session = sid; self.pose = [0, 0, 0, 0]; self.target = [0, 0, 0, 0]
            self.pulses = list(T.NEUTRAL_US); self.command_id = self.ping = 0; self.armed = True; self.moving = False
            self.fault = 'NONE'; self.last_alive = self.last_update = self.t
            self.emit(f'#ACK,FB_ARM,{sid},{rev}')
        elif f[0] == 'FB_PING':
            sid, n = int(f[1]), int(f[2])
            if not self.armed: return self._reject(0, 'DISARMED')
            if sid != self.session: return self._reject(0, 'SESSION')
            if n <= self.ping: return self._reject(0, 'PING_REPLAY')
            self.ping = n; self.last_alive = self.t; self.emit(f'#ACK,FB_PING,{sid},{n}')
        elif f[0] == 'FB_SET':
            sid, cid = int(f[1]), int(f[2]); req = [int(v) for v in f[3:7]]
            if not self.armed: return self._reject(cid, 'DISARMED')
            if sid != self.session: return self._reject(cid, 'SESSION')
            if cid <= self.command_id: return self._reject(cid, 'SET_REPLAY')
            if not CAD.pose_within_cad(tuple(req)): self._off('CAD_BOUNDS'); return self.emit(f'#ERR,{cid},CAD_BOUNDS')
            if not self._tune_ok(req): self._off('TUNE_BOUNDS'); return self.emit(f'#ERR,{cid},TUNE_BOUNDS')
            if _pulses(req) is None: return self._reject(cid, 'MOTOR_BOUNDS')
            self.target = req; self.command_id = cid; self.last_alive = self.t; self.moving = self.pose != self.target
            self.emit('#ACK,FB_SET,' + ','.join(map(str, (sid, cid, *req))))
        elif f[0].startswith('FB_') or cmd in ('ARM', 'HOLD', 'SET') or cmd.startswith('SET,'):
            self._reject(0, 'UNSUPPORTED')
        else:
            self.emit('#CONTROL_ERROR,INVALID_COMMAND')

    def _step(self):
        """One 20 ms frame of fallback_control.c Control_Poll."""
        nxt = list(self.target); num, den = 1, 1
        for i in range(4):                                         # Fallback_TuneLimitStep (common fraction)
            d = abs(nxt[i] - self.pose[i]); lim = (self.tune['pitch_rate_mdeg_s'] if i & 1 else self.tune['yaw_rate_mdeg_s']) * 20 // 1000
            if d and lim * den < d * num: num, den = lim, d
        trunc = lambda x: int(x)                                   # C int64 division truncates toward zero
        nxt = [self.pose[i] + trunc((nxt[i] - self.pose[i]) * num / den) for i in range(4)]
        cur = _pulses(self.pose)
        for s in (0, 2):                                           # Joint_LimitStep: bisection, 2 us per servo
            cand = list(self.pose); cand[s:s + 2] = nxt[s:s + 2]; p = _pulses(cand)
            if p is not None and max(abs(a - b) for a, b in zip(p, cur)) <= 2: continue
            dy, dp = nxt[s] - self.pose[s], nxt[s + 1] - self.pose[s + 1]; lo, hi = 0.0, 1.0
            best = self.pose[s:s + 2]
            for _ in range(20):
                fr = (lo + hi) / 2; y, pp = self.pose[s] + trunc(dy * fr), self.pose[s + 1] + trunc(dp * fr)
                c2 = list(self.pose); c2[s:s + 2] = [y, pp]; q = _pulses(c2)
                if q is not None and max(abs(a - b) for a, b in zip(q, cur)) <= 2: lo = fr; best = [y, pp]
                else: hi = fr
            nxt[s:s + 2] = best
        p = _pulses(nxt)
        if p is None or not CAD.pose_within_cad(tuple(nxt)): return self._off('PATH_BOUNDS')
        self.max_pulse_step = max(self.max_pulse_step, max(abs(a - b) for a, b in zip(p, self.pulses)))
        self.pose, self.pulses = nxt, list(p); self.moving = self.pose != self.target

    def _advance(self):
        while self.t < self.clock.now_ns:
            self.t = min(self.clock.now_ns, self.t + 10_000_000)       # 10 ms resolution
            if self.armed and self.t - self.last_alive >= 500_000_000: self._off('LEASE'); self.lease_faults += 1
            if self.armed and self.moving and self.t - self.last_update >= 20_000_000:
                self._step(); self.last_update = self.t
            if self.scan and self.t >= self.p_next:
                vals = self.pressure_fn(self.pose)
                for c in range(4):
                    self.p_seq += 1
                    self.emit(f'{self.p_seq},{self.t // 1_000_000 + c},{c},' + ','.join(f'{vals[f"A{b}_C{c}"]},0' for b in range(3)))
                self.p_next += 10_000_000
            if self.t - self.last_status >= 200_000_000:
                self.emit(self._state()); self.last_status = self.t
