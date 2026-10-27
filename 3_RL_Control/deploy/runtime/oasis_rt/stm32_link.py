"""Pi <-> NUCLEO-H723ZG link, OASIS_CONTROL firmware v6 (ESW repo, 4_Seat_Actuation).

UART 460800 8N1, ASCII lines (LF out, CRLF in). Host commands used here:
  CONTROL_HELLO -> #HELLO,OASIS_CONTROL,v6,<capabilities>
  HOLD / DISABLE / STATE / ARM / PING / START_12,15
  SET,<id>,<L yaw mdeg>,<L pitch mdeg>,<R yaw mdeg>,<R pitch mdeg>   -> #ACK,SET,... | #ERR,<id>,<reason>
  TRK,<id>,...  (PROPOSED firmware tracking mode, see deploy/firmware_proposal.md)
Device lines: #STATE,..., #ACK,..., #ERR,..., pressure packets "seq,tick,ch,raw,st,raw,st,raw,st".

Heartbeat: the firmware's 1.5 s control timeout is refreshed by ARM, SET and PING
(fsr_single.c: PING -> Control_Heartbeat(), reply #PONG,12). PING is sent every 0.25 s in all
modes, so no keep-alive SET is needed -- and a keep-alive SET would hurt: every SET restarts
the firmware's quintic path from the current pose (joint_control.c), stalling a moving bolster.

Sender policies (config `send_mode`), from the sim/v2 actuator study (actuator_speed_study.csv):
  event : FW v6 as is. SET only when a target moved >= deadband_deg and not more often than
          min_interval_s (keepalive_s = 0: none; > 0 re-sends the last target that often).
  track : TRK every control tick (PROPOSED firmware mode, deploy/firmware_proposal.md).
"""
import time
from collections import deque
from . import kinematics as K

HELLO = '#HELLO,OASIS_CONTROL,v6,'
REQUIRED = dict(yaw_min_mdeg=0, yaw_limit_mdeg=27000, pitch_limit_mdeg=8000, timeout_ms=1500,
                ds_outputs=4, ik_revision=8, crank_zero_us=1200, joints=2)
STATE_FIELDS = ('mcu_ms', 'armed', 'moving', 'left_yaw_mdeg', 'left_pitch_mdeg', 'right_yaw_mdeg', 'right_pitch_mdeg',
                'target_left_yaw_mdeg', 'target_left_pitch_mdeg', 'target_right_yaw_mdeg', 'target_right_pitch_mdeg',
                'D3_us', 'D4_us', 'D5_us', 'D6_us', 'command_id', 'fault')


def parse_hello(line):
    vals = dict(p.split('=', 1) for p in line[len(HELLO):].strip().split(','))
    vals = {k: int(v) for k, v in vals.items()}
    bad = {k: (vals.get(k), v) for k, v in REQUIRED.items() if vals.get(k) != v}
    if bad:
        raise RuntimeError(f'firmware capabilities differ from the runtime contract: {bad}')
    return vals


def parse_state(line):
    f = line.strip().split(',')
    if f[0] != '#STATE' or len(f) != len(STATE_FIELDS) + 1:
        raise ValueError('bad #STATE')
    s = dict(zip(STATE_FIELDS, f[1:]))
    for k in STATE_FIELDS[:-1]: s[k] = int(s[k])
    return s


def parse_pressure(line):
    f = line.strip().split(',')
    if len(f) != 9: raise ValueError('bad pressure packet')
    seq, tick, ch = map(int, f[:3])
    vals = []
    for b in range(3):
        raw, st = f[3 + 2 * b], int(f[4 + 2 * b])
        vals.append(int(raw) if (raw and st == 0) else None)
    return seq, tick, ch, vals


class Stm32Link:
    def __init__(self, transport, cfg, clock=time.monotonic):
        self.tx, self.cfg, self.clock = transport, cfg, clock
        self.caps = None; self.state = None; self.state_at = 0.0; self.fault = None
        self.cmd_id = 0; self.last_tx = -1e9; self.last_sent = None; self.last_ping = -1e9
        self.pressure = {}; self.pressure_at = 0.0; self.errors = deque(maxlen=50); self.log = deque(maxlen=2000)
        self.sent = 0; self.rejected = 0; self.projected = 0

    # -------------------------------------------------- lines
    def _write(self, text):
        self.tx.write_line(text); self.log.append(('TX', self.clock(), text))

    def poll(self):
        for line in self.tx.read_lines():
            self.log.append(('RX', self.clock(), line)); line = line.strip()
            if not line: continue
            if line.startswith(HELLO): self.caps = parse_hello(line)
            elif line.startswith('#STATE,'):
                self.state = parse_state(line); self.state_at = self.clock()
                if self.state['fault'] not in ('NONE', 'HOLD', 'BOOT') and not self.fault:
                    self.fault = self.state['fault']
            elif line.startswith('#ERR,'):
                self.errors.append(line); self.rejected += 1
                if 'PATH_UNREACHABLE' in line or 'DISARMED' in line: self.fault = line
            elif line.startswith('#BOOT'):
                self.fault = 'MCU_RESET'
            elif line[0].isdigit():
                try:
                    seq, tick, ch, vals = parse_pressure(line)
                except ValueError:
                    continue
                for bank, v in enumerate(vals):
                    if v is not None: self.pressure[f'A{bank}_C{ch}'] = v
                self.pressure_at = self.clock()

    # -------------------------------------------------- session
    def connect(self, timeout=3.0):
        self._write('CONTROL_HELLO'); t0 = self.clock()
        while self.caps is None and self.clock() - t0 < timeout:
            self.tx.wait(0.02); self.poll()
        if self.caps is None: raise RuntimeError('no #HELLO,OASIS_CONTROL,v6 reply')
        for c in ('HOLD', 'DISABLE', 'STATE', 'START_12,15'): self._write(c)
        return self.caps

    def arm(self):
        self.fault = None; self._write('ARM'); self._write('STATE')
        self.last_tx = self.clock()

    def hold(self):
        self._write('HOLD')

    def close(self):
        try: self._write('HOLD'); self._write('DISABLE')
        except Exception: pass

    # -------------------------------------------------- commands
    def send_targets(self, yaw_L, pitch_L, yaw_R, pitch_R):
        """Project into the reachable set, then send according to the sender policy.
        Returns the (projected) targets actually requested, in degrees."""
        now = self.clock(); c = self.cfg
        if now - self.last_ping >= 0.25: self._write('PING'); self.last_ping = now
        yL, pL, chL = K.project(yaw_L, pitch_L, c['yaw_max_deg'])
        yR, pR, chR = K.project(yaw_R, pitch_R, c['yaw_max_deg'])
        self.projected += int(chL or chR)
        tgt = (round(yL * 1000), round(pL * 1000), round(yR * 1000), round(pR * 1000))
        mode = c['send_mode']
        if mode == 'track':
            self._send('TRK', tgt, now)
        else:
            moved = self.last_sent is None or max(abs(a - b) for a, b in zip(tgt, self.last_sent)) >= 1000 * c['deadband_deg']
            due = now - self.last_tx >= c['min_interval_s']
            alive = c.get('keepalive_s', 0) > 0 and now - self.last_tx >= c['keepalive_s']
            if (moved and due) or (alive and self.last_sent is not None):
                self._send('SET', tgt if moved else self.last_sent, now)
        return yL, pL, yR, pR

    def _send(self, verb, tgt, now):
        self.cmd_id = (self.cmd_id + 1) & 0x7fffffff
        self._write(f'{verb},{self.cmd_id},' + ','.join(str(v) for v in tgt))
        self.last_tx, self.last_sent = now, tgt; self.sent += 1

    def reported(self):
        """Firmware pose (deg) from the last #STATE, or None."""
        if not self.state: return None
        s = self.state
        return (s['left_yaw_mdeg'] / 1000, s['left_pitch_mdeg'] / 1000, s['right_yaw_mdeg'] / 1000, s['right_pitch_mdeg'] / 1000)


class SerialTransport:
    """pyserial transport for the Pi (/dev/ttyACM*)."""
    def __init__(self, port, baud=460800):
        import serial
        self.s = serial.Serial(port, baud, timeout=0, write_timeout=0.3); self.buf = b''
    def write_line(self, text): self.s.write((text + '\n').encode('ascii'))
    def read_lines(self):
        self.buf += self.s.read(self.s.in_waiting or 0)
        *lines, self.buf = self.buf.split(b'\n')
        return [l.decode('ascii', 'replace') for l in lines]
    def wait(self, dt): time.sleep(dt)
