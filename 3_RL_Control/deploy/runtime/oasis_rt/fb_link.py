"""Pi <-> NUCLEO-H723ZG running OASIS_FALLBACK v2 (BeamNG_Bolster_FB stm32, built with -FallbackControl).

Thin adapter: the protocol itself is BeamNG_Bolster_FB's BoardLink (experiment_a/transport.py, unchanged), so the
Pi follows exactly the contract the desktop Experiment A was validated with:
  start   CONTROL_HELLO (v2, 270 deg, 2 us / 20 ms) -> FB_OFF -> CAD_INFO (profile sha256) -> TUNE_STATE -> DISABLE
          -> HELLO (FSR v10) -> START_12,15 -> STATE (PWM OFF verified)        [ready]
  arm     FB_ARM,<session_floor + 1>,<tune revision>  (once; never re-armed after a fault)
  run     one request outstanding (200 ms ACK timeout), FB runner priorities:
            FB_PING   every 100 ms, ONLY while the control loop has authorised a healthy tick in the last 120 ms
            STATE     every 100 ms (+ the board's own 5 Hz telemetry)
            FB_SET    new target, >= 40 ms apart, only while authorised
            PING      pressure-scanner watchdog, 1 s
  stop    FB_OFF -> DISABLE -> STATE (fresh OFF evidence, 300 ms)
Firmware lease 500 ms: a stalled Pi loop or an unhealthy input stops the PINGs and the board switches PWM OFF itself.
Pressure: 12-channel sweeps from the same UART (BoardLink's PressureRecorder); the latest VALID sweep feeds the
policy observation and the interlock as raw 12-bit counts (A<bank>_C<channel>), exactly like Stm32Link.pressure.
The interface (poll, send_targets, reported, pressure, pressure_at, fault) matches stm32_link.Stm32Link, so
controller.Controller runs unchanged on either firmware.
"""
from collections import deque
from .envelope import Envelope

PING_NS, STATE_NS, SET_NS, AUTH_NS = 100_000_000, 100_000_000, 40_000_000, 120_000_000


class FbLink:
    def __init__(self, transport, fb, clock_ns, log=print, record_pressure=True, pressure_log=None):
        self.fb, self.clock_ns, self.log = fb, clock_ns, log
        self.b = fb.transport.BoardLink(transport, record_pressure=record_pressure)
        self.pressure_log, self.logging_error = pressure_log, None
        self.tuning = self.hello = self.env = None
        self.pressure, self.pressure_at = {}, -1e9
        self.sweeps = self.bad_sweeps = 0
        self.target = self.last_sent = (0, 0, 0, 0)
        self.last_ping = self.last_state = self.last_set = 0; self.auth_ns = None
        self.abort = None; self.sent = self.projected = self.rejected = 0
        self.events = deque(maxlen=1000)

    def clock(self): return self.clock_ns() / 1e9

    @property
    def fault(self):
        return self.abort or (f'board:{self.b.fault}' if self.b.fault else None)

    def stop(self, reason):
        """Session-level stop (input fault, policy fault): no more lease renewal; the owner then calls close()."""
        if not self.abort: self.abort = reason; self.log(f'STOP: {reason}')

    # -------------------------------------------------- handshake
    def start(self): return self.b.start(self.clock_ns())

    @property
    def ready(self): return self.b.ready

    @property
    def armed(self): return self.b.armed_session is not None

    def freeze(self, yaw_cap_deg=None):
        """Adopt the committed STM tuning snapshot for this session (BoardLink re-checks it never changes)."""
        self.tuning, self.hello = dict(self.b.tuning), dict(self.b.hello)
        self.env = Envelope(self.fb, self.tuning, yaw_cap_deg)
        return self.tuning

    def state_fresh(self, max_age_ns=200_000_000, since_ns=None):
        s, now = self.b.state, self.clock_ns()
        return s is not None and 0 <= now - s.received_ns <= max_age_ns and (since_ns is None or s.received_ns > since_ns)

    def board_off(self):
        s = self.b.state
        return self.state_fresh() and not s.armed and not any(s.pulses_us)

    def armed_neutral(self, since_ns):
        s = self.b.state
        return (self.armed and self.state_fresh(since_ns=since_ns) and s.armed and not s.moving
                and s.pose_mdeg == (0, 0, 0, 0) and s.target_mdeg == (0, 0, 0, 0))

    def arm(self):
        if not self.check_logging(): return False
        if self.b.pending is not None: return False
        return self.b.arm(self.clock_ns())

    # -------------------------------------------------- loop
    def check_logging(self):
        """A failed journal revokes motor authority before ARM / lease / targets."""
        if self.logging_error: return False
        try:
            if self.pressure_log is not None: self.pressure_log.check()
        except Exception as e:
            self.logging_error = f'{type(e).__name__}: {e}'
            self.stop('pressure_logging:' + self.logging_error)
        return self.logging_error is None

    def _pressure_records(self):
        records = self.b.drain_pressure_records()
        if self.pressure_log is not None and not self.logging_error:
            try:
                self.pressure_log.pressure(records)
            except Exception as e:
                self.logging_error = f'{type(e).__name__}: {e}'
                self.stop('pressure_logging:' + self.logging_error)
        self.check_logging()
        for r in records:
            if r['type'] == 'sweep':
                if r['valid']:
                    self.pressure = {k: v['raw'] for k, v in r['values'].items() if v['raw'] is not None and v['status'] == 0}
                    self.pressure_at = r['completed_host_monotonic_ns'] / 1e9; self.sweeps += 1
                else:
                    self.bad_sweeps += 1
            elif r['type'] != 'sample':
                self.events.append(r)

    def poll(self):
        if self.b._closed: return
        now = self.clock_ns(); self.b.poll(now)
        if self.tuning is not None and self.b.tuning is not None and dict(self.b.tuning) != self.tuning:
            self.stop('firmware_tuning_changed_during_session')
        self._pressure_records()

    def authorize(self):
        """Called by the control loop after a healthy tick; lease renewal and new targets depend on it."""
        if self.check_logging() and not self.abort: self.auth_ns = self.clock_ns()

    def service(self):
        """Issue at most one request (BoardLink allows one outstanding)."""
        b = self.b
        if not self.check_logging(): return
        if not b.ready or b.pending is not None or self.fault: return
        now = self.clock_ns()
        ok = self.armed and self.auth_ns is not None and now - self.auth_ns <= AUTH_NS
        try:
            if ok and now - self.last_ping >= PING_NS:
                if b.ping(now): self.last_ping = now
            elif now - self.last_state >= STATE_NS:
                if b.request_state(now): self.last_state = now
            elif ok and self.target != self.last_sent and now - self.last_set >= SET_NS:
                if b.set_pose(self.target, now): self.last_sent, self.last_set = self.target, now; self.sent += 1
            else:
                b.pressure_ping(now)
        except self.fb.serial_link.LinkError as e:
            self.stop(f'link:{e}')

    # -------------------------------------------------- Controller interface (same as Stm32Link)
    def send_targets(self, yaw_L, pitch_L, yaw_R, pitch_R):
        """Project into CAD / STM tuning / IK (envelope.py), keep as the latest target. Returns degrees."""
        pose, changed = self.env.project(yaw_L, pitch_L, yaw_R, pitch_R)
        self.projected += int(changed); self.target = pose
        return tuple(v / 1000 for v in pose)

    def set_pose_mdeg(self, pose):
        if not self.env.ok(tuple(pose)): raise ValueError(f'pose {pose} outside the board envelope')
        self.target = tuple(int(v) for v in pose)

    def reported(self):
        s = self.b.state
        return None if s is None else tuple(v / 1000 for v in s.pose_mdeg)

    def hold(self):
        self.target = (0, 0, 0, 0)

    # -------------------------------------------------- shutdown
    def close(self, wait, timeout_s=0.3):
        """FB_OFF -> DISABLE -> STATE; True if fresh PWM-OFF telemetry was seen. `wait(dt)` advances time / sleeps.
        PWM OFF is a signal state: it does not prove a loaded mechanism released or stays put."""
        b = self.b; ok = False
        if b._closed: return False
        try:
            t0 = self.clock_ns(); b.off()
            while self.clock_ns() - t0 < timeout_s * 1e9:
                self.poll()  # Preserve samples and scanner events through verified DISABLE.
                s = b.state
                if s and s.received_ns > t0 and not s.armed and not any(s.pulses_us):
                    ok = True
                    if b.pressure_status(self.clock_ns()).get('scan_disabled_verified') and b.pending is None: break
                wait(0.002)
        except Exception as e:
            self.log(f'off error: {type(e).__name__}: {e}')
        finally:
            b.close()
        return ok
