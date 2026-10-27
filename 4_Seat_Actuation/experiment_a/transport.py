"""Experiment A UART transport with optional observation-only FSR recording.

The existing motor firmware contract, bounded parser and exact ACK/session
rules are preserved. Pressure acquisition is opt-in, uses this same UART
owner, and never supplies the controller or ARM gates with pressure values.
Board STATE is commanded actuator and transport-health evidence, never
measured human posture or pressure feedback. Importing opens no device.
"""
from __future__ import annotations

from fallback.serial_link import (
    ACK_TIMEOUT_NS, BAUD, HELLO_FIELDS as LEGACY_HELLO_FIELDS, MAX_LINE_BYTES, MAX_LINES_PER_POLL,
    MAX_POLL_GAP_NS, MAX_READ_BYTES, NEUTRAL_US, UINT32_MAX,
    BoardState, FSR_HELLO, LinkError, PendingCommand, open_serial, parse_state,
    _now, _pose, _uint,
)
from .pressure_logging import PressureRecorder
from .cad_limits import CAD_INFO_LINE, pose_within_cad

HELLO_FIELDS = dict(LEGACY_HELLO_FIELDS, servo_range_deg='270')
TUNING_RANGES = {
    'revision': (0, UINT32_MAX), 'yaw_max_mdeg': (100, 22000),
    'pitch_max_mdeg': (0, 6000), 'yaw_rate_mdeg_s': (100, 20000),
    'pitch_rate_mdeg_s': (100, 20000), 'full_scale_ay_mmps2': (500, 8000),
    'pitch_scale_permille': (10, 1000),
}


def _key_values(line, prefix):
    fields=line.split(',')
    if fields[:len(prefix)] != list(prefix):
        raise ValueError('Firmware identity/version mismatch')
    values={}
    for item in fields[len(prefix):]:
        if item.count('=') != 1:
            raise ValueError('Malformed firmware descriptor')
        key,value=item.split('=')
        if key in values:
            raise ValueError('Duplicate firmware descriptor field')
        values[key]=value
    return values


def parse_hello(line):
    values=_key_values(line, ('#HELLO','OASIS_FALLBACK','v2'))
    if set(values) != set(HELLO_FIELDS)|{'session_floor'}:
        raise ValueError('Unexpected Experiment A HELLO fields')
    if any(values[key] != value for key,value in HELLO_FIELDS.items()):
        raise ValueError('Firmware 270-degree calibration/bounds mismatch')
    values['session_floor']=_uint(values['session_floor'])
    values.update(firmware='OASIS_FALLBACK',version='v2')
    return values


def parse_tuning(line):
    values=_key_values(line, ('#TUNE','OASIS_A','v1'))
    if set(values) != set(TUNING_RANGES):
        raise ValueError('Unexpected STM tuning fields')
    for key,(low,high) in TUNING_RANGES.items():
        values[key]=_uint(values[key], high)
        if values[key] < low:
            raise ValueError('STM tuning value outside permitted range')
    return values


class BoardLink:
    """One outstanding request; no worker, reconnect, queued motion or auto ARM.

    Startup explicitly switches PWM and the scanner OFF and verifies STATE;
    optional pressure recording starts only the scanner during this handshake.
    ARM is separate and is called by the owner after BeamNG qualification.
    Every motor lease refresh must originate from that owner's healthy tick.
    """
    def __init__(self, transport, *, record_pressure=False, pressure_queue_limit=2048):
        if type(record_pressure) is not bool:
            raise ValueError("record_pressure must be a bool")
        for name in ("timeout", "write_timeout"):
            if getattr(transport, name, 0) != 0:
                raise ValueError("Transport must have nonblocking timeouts")
        self.transport = transport
        self.hello = self.state = self.pending = self.last_ack = None
        self.cad_verified = False
        self.tuning = None
        self.fault = self.armed_session = None
        self.motor_inhibited = False
        self.motor_off_verified = False
        self.ready = False
        self._started = self._closed = self._off_requested = False
        self._buffer = bytearray()
        self._last_poll_ns = self._last_now_ns = None
        self._command_id = self._ping_id = 0
        self._phase = self._ignored_ack = None
        self.record_pressure = record_pressure
        self._pressure = PressureRecorder(pressure_queue_limit) if record_pressure else None
        self._scan = False
        self._scan_disabled_verified = False
        self._pressure_ping_ns = 0

    def _time(self, now_ns):
        now_ns = _now(now_ns)
        if self._last_now_ns is not None and now_ns < self._last_now_ns:
            self._latch("host_clock_regression")
            raise LinkError(self.fault)
        self._last_now_ns = now_ns
        return now_ns

    def _request(self, command, expected, now_ns, *, phase=None):
        if self.pending:
            return False
        if len(command) > 63 or not command.isascii() or "\n" in command or "\r" in command:
            raise LinkError("Command exceeds firmware line bounds")
        self.pending = PendingCommand(command, tuple(expected), now_ns)
        if phase is not None:
            self._phase = phase
        payload = (command+"\n").encode("ascii")
        try:
            written = self.transport.write(payload)
            if type(written) is not int or written != len(payload):
                raise OSError("Partial/nonblocking UART write")
        except Exception as exc:
            self._latch(f"write_failed:{type(exc).__name__}:{exc}")
            raise LinkError(self.fault) from exc
        return True

    def _emergency(self):
        # Terminate any partial request before the explicit motor OFF command.
        try:
            self.transport.write(b"\nFB_OFF\nDISABLE\n")
        except Exception:
            pass

    def _latch(self, reason):
        if self.fault is None:
            self.fault = reason
            if self._pressure is not None:
                self._pressure.event(self._last_now_ns or 0, "transport_fault", detail=reason)
            if self.hello is not None:
                self._emergency()
        self.ready = False
        self._scan = False
        self._scan_disabled_verified = False
        self.armed_session = self.pending = self._phase = None

    def start(self, now_ns):
        now_ns = self._time(now_ns)
        if self._started or self._closed or self.fault:
            raise LinkError("Create a new link for an explicit new connection")
        self._started = True
        if hasattr(self.transport, "reset_input_buffer"):
            self.transport.reset_input_buffer()
        return self._request("CONTROL_HELLO", ("#HELLO", "OASIS_FALLBACK"),
                             now_ns, phase="control_hello")

    def _healthy(self, now_ns, *, require_armed=False):
        now_ns = self._time(now_ns)
        if self._closed or self.fault or not self.ready or self._off_requested:
            raise LinkError(self.fault or "Motor-only handshake is not ready")
        if self.state is None or not 0 <= now_ns-self.state.received_ns <= 400_000_000:
            self._latch("state_stale")
            raise LinkError(self.fault)
        if require_armed and self.armed_session is None:
            raise LinkError("Explicit ARM has not been acknowledged")
        return now_ns

    def arm(self, now_ns):
        now_ns = self._healthy(now_ns)
        if self.motor_inhibited:
            raise LinkError("Motor output inhibited for this recording session")
        if self.pending:
            return False
        if self.armed_session is not None or self.state.armed:
            raise LinkError("Already armed; ARM is never replayed")
        sid = self.hello["session_floor"]+1
        if sid > UINT32_MAX:
            raise LinkError("Session ID exhausted")
        self.hello["session_floor"] = sid
        if self.tuning is None:
            raise LinkError('STM tuning not verified')
        revision = self.tuning['revision']
        return self._request(f"FB_ARM,{sid},{revision}",
                             ("#ACK", "FB_ARM", str(sid), str(revision)), now_ns)

    def set_pose(self, pose_mdeg, now_ns):
        now_ns = self._healthy(now_ns, require_armed=True)
        if self.motor_inhibited:
            raise LinkError("Motor output inhibited for this recording session")
        pose = _pose(pose_mdeg)
        if not pose_within_cad(pose):
            raise LinkError('Pose outside CAD commissioning envelope')
        if self.pending:
            return False
        self._command_id += 1
        if self._command_id > UINT32_MAX:
            self._latch("command_id_exhausted")
            raise LinkError(self.fault)
        fields = ("FB_SET", str(self.armed_session), str(self._command_id), *(str(x) for x in pose))
        return self._request(",".join(fields), ("#ACK", *fields), now_ns)

    def ping(self, now_ns):
        now_ns = self._healthy(now_ns, require_armed=True)
        if self.motor_inhibited:
            raise LinkError("Motor output inhibited for this recording session")
        if self.pending:
            return False
        self._ping_id += 1
        if self._ping_id > UINT32_MAX:
            self._latch("ping_id_exhausted")
            raise LinkError(self.fault)
        fields = ("FB_PING", str(self.armed_session), str(self._ping_id))
        return self._request(",".join(fields), ("#ACK", *fields), now_ns)

    def request_state(self, now_ns):
        return self._request("STATE", ("#STATE",), self._healthy(now_ns))

    def pressure_ping(self, now_ns):
        """Refresh the independent scanner watchdog, at most once per second.

        The owner calls this after motor/state work. Uses the same one-request
        ACK slot, never a worker or autonomous motor lease refresh.
        """
        if not self.record_pressure:
            return False
        now_ns = self._healthy(now_ns)
        if not self._scan or now_ns-self._pressure_ping_ns < 1_000_000_000:
            return False
        return self._request("PING", ("#PONG", "12"), now_ns)

    def drain_pressure_records(self):
        """Return <=queue_limit+1 records, including any explicit drop report."""
        return self._pressure.drain() if self._pressure is not None else []

    def pressure_status(self, now_ns):
        if self._pressure is None:
            return {"enabled": False, "scan_active": False,
                    "scan_disabled_verified": self._scan_disabled_verified}
        return dict(self._pressure.status(now_ns), enabled=True, scan_active=self._scan,
                    scan_disabled_verified=self._scan_disabled_verified)

    def off(self):
        if self._closed:
            return False
        self._off_requested = True
        self.armed_session = None
        self.ready = False
        if self.hello is None:
            return False
        if self.fault:
            self._emergency()
            return False
        self._ignored_ack = (tuple(FSR_HELLO.split(",")) if self._phase == "pressure_hello"
                             else self.pending.expected if self.pending else None)
        self.pending = None
        return self._request("FB_OFF", ("#ACK", "FB_OFF"), self._last_now_ns or 0, phase="off")

    def motor_off_keep_recording(self, now_ns):
        """Latch motor authority OFF while retaining the independent scanner.

        This preempts a pending motor/state request. An exact outstanding ACK
        may be discarded once; fresh zero-PWM STATE after FB_OFF is required
        before state/scanner requests become ready again. Only a new link can
        ever ARM afterwards. Ordinary off()/close() still stop both devices.
        """
        now_ns = self._healthy(now_ns)
        if self.motor_inhibited:
            return False
        self.motor_inhibited = True
        self.motor_off_verified = False
        self.armed_session = None
        self.ready = False
        self._ignored_ack = self.pending.expected if self.pending else None
        self.pending = None
        if self._pressure is not None:
            self._pressure.event(now_ns, "motor_off_scanner_retained")
        return self._request("FB_OFF", ("#ACK", "FB_OFF"), now_ns, phase="motor_off")

    def _accepted(self, fields, now_ns):
        pending, phase = self.pending, self._phase
        self.pending = self._phase = None
        self.last_ack = {"command": pending.command, "op": pending.command.split(",")[0],
                         "received_ns": now_ns, "fields": tuple(fields)}
        if phase == "control_hello":
            self._request("FB_OFF", ("#ACK", "FB_OFF"), now_ns, phase="initial_off")
        elif phase == "initial_off":
            self._request("CAD_INFO", tuple(CAD_INFO_LINE.split(',')), now_ns, phase="cad_info")
        elif phase == "cad_info":
            self.cad_verified = True
            self._request('TUNE_STATE', ('#TUNE','OASIS_A','v1'), now_ns, phase='tuning')
        elif phase == 'tuning':
            self._request("DISABLE", ("#CONTROL", "12", "MASK=0", "EN=1"), now_ns, phase="initial_disable")
        elif phase == "off":
            self._request("DISABLE", ("#CONTROL", "12", "MASK=0", "EN=1"), now_ns,
                          phase="off_disable")
        elif phase == "motor_off":
            self._request("STATE", ("#STATE",), now_ns, phase="motor_off_state")
        elif phase in ("initial_disable", "off_disable"):
            self._scan = False
            self._scan_disabled_verified = True
            if phase == "off_disable" and self._pressure is not None:
                self._pressure.event(now_ns, "scan_disabled")
            if phase == "initial_disable" and self.record_pressure:
                self._request("HELLO", ("#HELLO", "FSR_12_SCAN"), now_ns, phase="pressure_hello")
            else:
                self._request("STATE", ("#STATE",), now_ns,
                              phase="initial_state" if phase == "initial_disable" else "off_state")
        elif phase == "pressure_hello":
            self._scan_disabled_verified = False
            self._request("START_12,15", ("#CONTROL", "12", "MASK=15", "EN=0"),
                          now_ns, phase="pressure_scan")
        elif phase == "pressure_scan":
            self._scan = True
            self._pressure_ping_ns = now_ns
            self._pressure.event(now_ns, "scan_started", mask=15, nominal_sweep_hz=100)
            self._request("STATE", ("#STATE",), now_ns, phase="initial_state")
        elif phase in ("initial_state", "off_state", "motor_off_state"):
            if self.state.armed or any(self.state.pulses_us):
                self._latch("off_not_verified")
            elif phase in ("initial_state", "motor_off_state"):
                self.ready = True
                if phase == "motor_off_state":
                    self.motor_off_verified = True
        elif fields[:2] == ("#ACK", "FB_ARM"):
            self.armed_session = _uint(fields[2])
            self._command_id = self._ping_id = 0
        elif fields == ("#PONG", "12"):
            self._pressure_ping_ns = now_ns

    def _line(self, line, now_ns):
        fields = tuple(line.split(","))
        if self._ignored_ack and fields == self._ignored_ack:
            self._ignored_ack = None
            return
        if fields[0] == "#STATE":
            state = parse_state(line, now_ns)
            if self.state and ((state.tick_ms-self.state.tick_ms) & UINT32_MAX) > 0x7fffffff:
                raise ValueError("MCU state clock reset")
            self.state = state
            if self.motor_inhibited and self.motor_off_verified and (state.armed or any(state.pulses_us)):
                raise ValueError("Motor output resumed after latched OFF")
            if self.armed_session is not None and (not state.armed or state.fault != "NONE"):
                raise ValueError("Motor session lost: "+state.fault)
            if state.fault not in ("BOOT", "OFF", "NONE"):
                raise ValueError("Board fault: "+state.fault)
        elif fields[0] == "#CAD":
            if self._phase != 'cad_info' or line != CAD_INFO_LINE:
                raise ValueError('CAD profile mismatch: matching Experiment A firmware required')
        elif fields[0] == '#TUNE':
            if self._phase != 'tuning':
                raise ValueError('Unsolicited STM tuning state')
            self.tuning = parse_tuning(line)
        elif fields[0] == "#HELLO":
            if self._phase == "control_hello":
                self.hello = parse_hello(line)
            elif self._phase == "pressure_hello":
                if line != FSR_HELLO:
                    raise ValueError("FSR firmware contract mismatch")
            else:
                raise ValueError("Unsolicited HELLO / potential MCU restart")
        elif fields[0] in ("#ERR", "#CONTROL_ERROR"):
            raise ValueError("Board error: "+line)
        elif fields[0] == "#BOOT":
            if self.hello is not None:
                raise ValueError("MCU reboot")
            return
        elif fields[0] == "#STATS":
            if self._pressure is not None:
                stats = self._pressure.statistics(line, now_ns)
                if any(stats.get(name, 0) for name in ("uart_errors", "rx_errors")):
                    raise ValueError("FSR UART error counter")
            return
        elif fields[0] in ("#COMMANDS", "#ADC_BANKS", "seq"):
            if self._pressure is not None:
                self._pressure.event(now_ns, "scanner_metadata", raw_line=line)
            return
        elif fields[0].isdigit() or (self.record_pressure and not line.startswith("#") and "," in line):
            if self._pressure is not None:
                self._pressure.sample(line, now_ns)
            return
        elif fields[0] not in ("#ACK", "#PONG", "#CONTROL"):
            raise ValueError("Unknown UART line")

        pending = self.pending
        matches = pending and (fields == pending.expected or
            (pending.expected == ("#STATE",) and fields[0] == "#STATE") or
            (self._phase in ("control_hello", "pressure_hello") and fields[:2] == pending.expected) or
            (self._phase == 'tuning' and fields[:3] == pending.expected))
        if matches:
            self._accepted(fields, now_ns)
        elif fields[0] != "#STATE":
            raise ValueError("Unexpected or mismatched command response")

    def poll(self, now_ns):
        now_ns = self._time(now_ns)
        if self._closed:
            return
        if (self._last_poll_ns is not None and now_ns-self._last_poll_ns > MAX_POLL_GAP_NS
                and self.armed_session is not None):
            self._latch("host_poll_gap")
        self._last_poll_ns = now_ns
        if self.pending and now_ns-self.pending.sent_ns >= ACK_TIMEOUT_NS:
            self._latch("ack_timeout:"+self.pending.command.split(",")[0])
        try:
            waiting = self.transport.in_waiting
            if type(waiting) is not int or not 0 <= waiting <= MAX_READ_BYTES:
                raise ValueError("UART receive backlog exceeded")
            if waiting:
                data = self.transport.read(waiting)
                if not isinstance(data, bytes) or len(data) > waiting:
                    raise ValueError("Invalid transport read")
                self._buffer.extend(data)
            count = 0
            while b"\n" in self._buffer and count < MAX_LINES_PER_POLL:
                newline = self._buffer.index(10)
                if newline > MAX_LINE_BYTES:
                    if self._pressure is not None:
                        self._pressure.event(now_ns, "uart_line_too_long",
                            prefix_hex=bytes(self._buffer[:MAX_LINE_BYTES]).hex(), line_length=newline)
                    raise ValueError("UART line too long")
                raw = bytes(self._buffer[:newline]).rstrip(b"\r")
                del self._buffer[:newline+1]
                count += 1
                if not raw:
                    continue
                if any(byte < 32 or byte > 126 for byte in raw):
                    if self._pressure is not None:
                        self._pressure.event(now_ns, "invalid_uart_bytes", raw_hex=raw.hex())
                    raise ValueError("Non-ASCII/control byte in UART response")
                if not self.fault:
                    self._line(raw.decode("ascii"), now_ns)
                elif raw.startswith(b"#STATE,"):
                    self.state = parse_state(raw.decode("ascii"), now_ns)
                elif self._pressure is not None:
                    self._pressure.event(now_ns, "uart_after_fault", raw_line=raw.decode("ascii"))
            if len(self._buffer) > MAX_LINE_BYTES or b"\n" in self._buffer:
                raise ValueError("UART parser backlog exceeded")
        except Exception as exc:
            self._buffer.clear()
            self._latch(f"protocol:{type(exc).__name__}:{exc}")

    def close(self):
        if self._closed:
            return
        try:
            if self.hello is not None:
                self._emergency()
        finally:
            self._closed = True
            self.ready = False
            self._scan = False
            self.armed_session = None
            self.transport.close()
