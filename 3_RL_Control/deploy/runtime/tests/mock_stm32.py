"""Software stand-in for the NUCLEO-H723ZG running OASIS_CONTROL v6 (for SIL tests on a PC).
Behaviour follows sim/v2/firmware_model.py: SET validation, quintic path restart per SET,
20 ms update, 1.5 s heartbeat timeout (refreshed by ARM / SET / PING), latched faults,
#STATE every 200 ms, pressure packets.
`trk=True` adds the PROPOSED TRK tracking command; `crank_rate` the proposed faster rate."""
import os, sys
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..', 'sim', 'v2'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from firmware_model import Side, duration_s  # noqa: E402
from oasis_rt import kinematics as K  # noqa: E402

HELLO = ('#HELLO,OASIS_CONTROL,v6,yaw_min_mdeg=0,yaw_limit_mdeg=27000,pitch_limit_mdeg=8000,timeout_ms=1500,'
         'motor_rate_mdeg_s={rate},ds_outputs=4,ik_revision=8,crank_zero_us=1200,joints=2')


class MockClock:
    def __init__(self): self.t = 0.0
    def __call__(self): return self.t


class MockStm32:
    def __init__(self, clock, crank_rate=15.0, trk=False, pressure_hz=25.0, pressure_fn=None):
        self.clock, self.rate, self.trk = clock, crank_rate, trk
        self.side = [Side(), Side()]; self.armed = False; self.fault = 'BOOT'; self.cmd_id = 0
        self.out = []; self.last_alive = 0.0; self.last_status = -1.0; self.last_update = 0.0
        self.pressure_on = False; self.p_period = 1 / pressure_hz; self.p_next = 0.0; self.p_seq = 0
        self.pressure_fn = pressure_fn or (lambda t: {f'A{b}_C{c}': 1000 for b in range(3) for c in range(4)})
        self.track_mode = False

    # transport side ------------------------------------------------------
    def write_line(self, text): self._command(text.strip())
    def read_lines(self):
        out, self.out = self.out, []; return out
    def wait(self, dt): self.advance(dt)

    def _tx(self, s): self.out.append(s + '\r\n')

    def _state(self):
        p = [round(v * 1000) for v in (*self.side[0].pose, *self.side[1].pose)]
        g = [round(v * 1000) for v in (*self.side[0].tgt, *self.side[1].tgt)]
        mv = int(any(s.moving for s in self.side))
        return '#STATE,' + ','.join(map(str, [int(self.clock() * 1000), int(self.armed), mv, *p, *g, 1500, 1200, 1200, 1500, self.cmd_id])) + f',{self.fault}'

    def _command(self, c):
        now = self.clock()
        if c == 'CONTROL_HELLO': self._tx(HELLO.format(rate=int(self.rate * 1000)))
        elif c == 'ARM': self.armed, self.fault, self.last_alive = True, 'NONE', now; self._tx('#ACK,ARM')
        elif c == 'HOLD': self._fault('HOLD'); self._tx('#ACK,HOLD')
        elif c == 'STATE': self._tx(self._state())
        elif c.startswith('START_12'): self.pressure_on = True
        elif c == 'DISABLE': self.pressure_on = False
        elif c == 'PING': self.last_alive = now; self._tx('#PONG,12')          # fsr_single.c: PING -> Control_Heartbeat()
        elif c.startswith('SET,') or (self.trk and c.startswith('TRK,')):
            f = c.split(',')
            try:
                cid, v = int(f[1]), [int(x) for x in f[2:6]]
            except (ValueError, IndexError):
                self._tx('#ERR,0,INVALID'); return
            poses = [(v[0] / 1000, v[1] / 1000), (v[2] / 1000, v[3] / 1000)]
            if not all(K.pulses_ok(y, p) for y, p in poses): self._tx(f'#ERR,{cid},MOTOR_BOUNDS'); return
            if not self.armed: self._tx(f'#ERR,{cid},DISARMED'); return
            self.cmd_id, self.last_alive = cid, now
            if c.startswith('TRK,'):
                self.track_mode = True
                for s, pz in zip(self.side, poses): s.tgt = np.array(pz, float)
                return
            self.track_mode = False
            dur = max(duration_s(*s.pose, *pz, rate=self.rate) for s, pz in zip(self.side, poses))
            for s, pz in zip(self.side, poses):
                if not s.set(now, list(pz), self.rate):
                    self._fault('PATH_UNREACHABLE'); self._tx('#ERR,0,PATH_UNREACHABLE'); return
            self._tx(f'#ACK,SET,{cid},' + ','.join(map(str, v)) + f',{int(round(dur * 1000))}')
        else:
            self._tx(f'#ERR,0,UNKNOWN')

    def _fault(self, reason):
        self.armed = False; self.fault = reason
        for s in self.side: s.moving = False; s.tgt = s.pose.copy()

    # firmware main loop -------------------------------------------------
    def advance(self, dt):
        end = self.clock() + dt
        while self.clock() < end - 1e-12:
            step = min(0.005, end - self.clock()); self.clock.t += step; now = self.clock()
            if self.armed and now - self.last_alive >= 1.5: self._fault('TIMEOUT')
            if now - self.last_update >= 0.02:
                self.last_update = now
                for s in self.side:
                    if self.track_mode and self.armed: s.track(s.tgt, 0.02, vmax=0.8 * self.rate / 2.3, amax=8 * 0.8 * self.rate / 2.3)
                    else: s.update(now)
            if now - self.last_status >= 0.2: self.last_status = now; self._tx(self._state())
            if self.pressure_on and now >= self.p_next:
                self.p_next = now + self.p_period; vals = self.pressure_fn(now)
                for ch in range(4):
                    self.p_seq += 1
                    raws = [vals[f'A{b}_C{ch}'] for b in range(3)]
                    self._tx(f'{self.p_seq},{int(now * 1000)},{ch},' + ','.join(f'{int(r)},0' for r in raws))
