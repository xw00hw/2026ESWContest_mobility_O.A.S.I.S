"""Explicit-session fallback UART transport; importing never opens a device.

One outstanding request, exact ACK matching, bounded nonblocking reads/writes.
There is deliberately no heartbeat thread. The policy owner calls ping only
while its vehicle/pressure gates are healthy. Angles are commanded, not measured.
Pressure voltage uses the recorded nominal 3.3 V ADC reference, not force units.
"""
from __future__ import annotations

from dataclasses import dataclass, replace
import re

BAUD = 460800
ACK_TIMEOUT_NS = 200_000_000
PRESSURE_MAX_AGE_NS = 100_000_000
MAX_POLL_GAP_NS = 100_000_000
MAX_READ_BYTES = 8192
MAX_LINE_BYTES = 512
MAX_LINES_PER_POLL = 128
UINT32_MAX = 0xffffffff
# Current fallback installation (operator-corrected).
# Pin order: UR/D3, UL/D4, DR/D5, DL/D6. Historical logs retain their own map.
NEUTRAL_US = (1800, 1800, 1200, 1200)
SENSORS = tuple(f"A{bank}_C{channel}" for bank in range(3) for channel in range(4))


class LinkError(RuntimeError):
    pass


@dataclass(frozen=True, slots=True)
class BoardState:
    received_ns: int
    armed: bool
    moving: bool
    pose_mdeg: tuple[int, int, int, int]
    target_mdeg: tuple[int, int, int, int]
    pulses_us: tuple[int, int, int, int]
    command_id: int
    fault: str
    tick_ms: int = 0


@dataclass(frozen=True, slots=True)
class PressureFrame:
    received_ns: int
    voltages: dict[str, float]
    valid: bool
    reason: str
    sequence_id: int
    tick_ms: int

    @property
    def sequence(self):
        return self.sequence_id


@dataclass(frozen=True, slots=True)
class PendingCommand:
    command: str
    expected: tuple[str, ...]
    sent_ns: int


def _uint(text, maximum=UINT32_MAX):
    if not isinstance(text, str) or not re.fullmatch(r"[0-9]{1,10}", text):
        raise ValueError("Expected unsigned decimal integer")
    value = int(text)
    if value > maximum:
        raise ValueError("Unsigned integer outside range")
    return value


def _now(value):
    if type(value) is not int or value < 0:
        raise ValueError("now_ns must be a nonnegative integer")
    return value


def _pose(value):
    if not isinstance(value, (tuple, list)) or len(value) != 4:
        raise ValueError("Expected LY, LP, RY, RP in integer mdeg")
    if any(type(item) is not int for item in value):
        raise ValueError("Pose values must be integer mdeg")
    if not all((0 <= item <= 27000) if index % 2 == 0 else (-8000 <= item <= 8000)
               for index, item in enumerate(value)):
        raise ValueError("Pose outside fallback firmware envelope")
    return tuple(value)


def parse_state(line: str, now_ns: int) -> BoardState:
    fields = line.split(",")
    if len(fields) != 18 or fields[0] != "#STATE":
        raise ValueError("Expected 18-field fallback STATE")
    tick, armed, moving = (_uint(fields[1]), _uint(fields[2], 1), _uint(fields[3], 1))
    angles = []
    for text in fields[4:12]:
        if not re.fullmatch(r"-?[0-9]{1,5}", text):
            raise ValueError("Invalid STATE pose")
        angles.append(int(text))
    pose, target = _pose(angles[:4]), _pose(angles[4:])
    pulses = tuple(_uint(text, 2500) for text in fields[12:16])
    command_id, fault = _uint(fields[16]), fields[17]
    if not re.fullmatch(r"[A-Z][A-Z0-9_]{0,31}", fault):
        raise ValueError("Invalid STATE fault")
    if armed:
        if fault != "NONE" or any(pulse < 500 for pulse in pulses):
            raise ValueError("Inconsistent armed STATE")
        if bool(moving) != (pose != target):
            raise ValueError("Inconsistent moving STATE")
    elif moving or any(pulses) or target != pose or fault == "NONE":
        raise ValueError("Inconsistent OFF STATE")
    return BoardState(_now(now_ns), bool(armed), bool(moving), pose, target, pulses, command_id, fault, tick)


HELLO_FIELDS = {
    "yaw_min_mdeg": "0", "yaw_limit_mdeg": "27000", "pitch_limit_mdeg": "8000",
    "servo_range_deg": "270", "neutral_us": ":".join(map(str, NEUTRAL_US)), "boot_pwm": "OFF",
    "lease_ms": "500", "tick_ms": "20", "max_step_us": "2", "ds_outputs": "4", "ik_revision": "8",
}
FSR_HELLO = "#HELLO,FSR_12_SCAN,v10,bits=12,hz=100,banks=3,max_channels=16"


def parse_hello(line: str) -> dict:
    fields = line.split(",")
    if fields[:3] != ["#HELLO", "OASIS_FALLBACK", "v1"]:
        raise ValueError("Only OASIS_FALLBACK v1 is accepted")
    result = {}
    for item in fields[3:]:
        if item.count("=") != 1:
            raise ValueError("Malformed fallback HELLO")
        key, value = item.split("=")
        if key in result:
            raise ValueError("Duplicate fallback HELLO key")
        result[key] = value
    if set(result) != set(HELLO_FIELDS) | {"session_floor"}:
        raise ValueError("Unexpected fallback HELLO fields")
    if any(result[key] != value for key, value in HELLO_FIELDS.items()):
        raise ValueError("Firmware bounds/calibration contract mismatch")
    result["session_floor"] = _uint(result["session_floor"])
    result.update(firmware="OASIS_FALLBACK", version="v1")
    return result


class PressureTracker:
    """Publish only consecutive C0/C1/C2/C3 sweeps (three banks each).

    received_ns is the oldest row's host receipt, never a new timestamp for
    retained channels. MCU sequence/tick continuity and host/MCU drift reject
    gaps, reboots and delayed backlogs; wraparound remains supported.
    """
    def __init__(self):
        self.frame = None
        self._last = None
        self._parts = []
        self._anchor = None
        self._elapsed_ms = 0

    def invalidate(self, reason):
        self._parts.clear()
        if self.frame is not None:
            self.frame = replace(self.frame, valid=False, reason=reason)

    def update(self, line: str, now_ns: int):
        now_ns = _now(now_ns)
        fields = line.split(",")
        if len(fields) != 9:
            raise ValueError("Expected nine-field FSR sample")
        seq, tick, channel = _uint(fields[0]), _uint(fields[1]), _uint(fields[2], 3)
        readings = []
        for bank in range(3):
            raw, status = fields[3+bank*2:5+bank*2]
            if _uint(status, 5) != 0:
                self.invalidate("adc_status")
                raise ValueError("FSR ADC status is not zero")
            readings.append(_uint(raw, 4095)*3.3/4095.)
        issue = None
        if self._last is not None:
            previous_seq, previous_tick, previous_ns = self._last
            delta = (tick-previous_tick) & UINT32_MAX
            if (seq-previous_seq) & UINT32_MAX != 1:
                issue = "pressure_sequence_gap"
            elif delta == 0 or delta > 50:
                issue = "pressure_tick_gap_or_reset"
            elif now_ns < previous_ns or now_ns-previous_ns > PRESSURE_MAX_AGE_NS:
                issue = "pressure_host_gap"
            self._elapsed_ms += delta
        else:
            self._anchor = now_ns
        if now_ns-self._anchor-self._elapsed_ms*1_000_000 > PRESSURE_MAX_AGE_NS:
            issue = "pressure_transport_delay"
        self._last = (seq, tick, now_ns)
        if issue:
            self.invalidate(issue)
            self.frame = PressureFrame(now_ns, {}, False, issue, seq, tick)
            self._anchor, self._elapsed_ms = now_ns, 0
            if channel != 0:
                return self.frame
        if channel == 0:
            self._parts = [(seq, tick, now_ns, readings)]
        elif len(self._parts) == channel:
            self._parts.append((seq, tick, now_ns, readings))
        else:
            self.invalidate("incomplete_sweep")
        if len(self._parts) == 4:
            first = self._parts[0]
            voltage = {f"A{bank}_C{index}": part[3][bank]
                       for index, part in enumerate(self._parts) for bank in range(3)}
            self.frame = PressureFrame(first[2], voltage, True, "ok", seq, first[1])
            self._parts.clear()
        return self.frame

    def at(self, now_ns):
        if self.frame and (_now(now_ns)-self.frame.received_ns > PRESSURE_MAX_AGE_NS):
            self.invalidate("pressure_stale")
        return self.frame


def open_serial(port, *, serial_factory=None):
    """Explicit device-opening helper. Windows Serial uses exclusive CreateFile.

    timeout=write_timeout=0 prevents blocking. No reset, DTR pulse, auto-probe,
    device selection or background worker is requested by this module.
    """
    if not isinstance(port, str) or not port:
        raise ValueError("An explicit serial port is required")
    if serial_factory is None:
        import serial
        serial_factory = serial.Serial
    transport = serial_factory(port=None, baudrate=BAUD, timeout=0, write_timeout=0, exclusive=True)
    try:
        transport.dtr = False
        transport.rts = False
        transport.port = port
        transport.open()
        return transport
    except Exception:
        transport.close()
        raise


class BoardLink:
    """Injected transport supplies read(n), write(bytes), close(), in_waiting.

    All public action methods return True only when sent, False when a request
    is already pending. Invalid state raises LinkError. No action is queued for
    later movement. start() performs only the OFF/pressure handshake; arm() is
    a separate explicit call. last_ack is metadata, not physical completion.
    """
    def __init__(self, transport):
        for name in ("timeout", "write_timeout"):
            if getattr(transport, name, 0) != 0:
                raise ValueError("Transport must have nonblocking read/write timeouts")
        self.transport = transport
        self.hello = self.state = self.pressure = self.pending = self.last_ack = None
        self.fault = None
        self.armed_session = None
        self.ready = False
        self._started = self._closed = self._scan = self._off_requested = False
        self._buffer = bytearray()
        self._tracker = PressureTracker()
        self._last_poll_ns = self._last_now_ns = None
        self._command_id = self._ping_id = 0
        self._pressure_ping_ns = 0
        self._phase = None
        self._ignored_ack = None
        self._stats = {}

    def _time(self, now_ns):
        now_ns = _now(now_ns)
        if self._last_now_ns is not None and now_ns < self._last_now_ns:
            self._latch("host_clock_regression")
            raise LinkError(self.fault)
        self._last_now_ns = now_ns
        return now_ns

    def _write(self, payload):
        try:
            written = self.transport.write(payload)
            if type(written) is not int or written != len(payload):
                raise OSError("Partial/nonblocking UART write")
        except Exception as exc:
            raise LinkError(f"write_failed:{type(exc).__name__}:{exc}") from exc

    def _request(self, command, expected, now_ns, *, phase=None):
        if self.pending:
            return False
        if len(command) > 63 or not command.isascii() or "\n" in command or "\r" in command:
            raise LinkError("Command exceeds firmware line bounds")
        self.pending = PendingCommand(command, tuple(expected), now_ns)
        if phase is not None:
            self._phase = phase
        try:
            self._write((command+"\n").encode("ascii"))
        except LinkError as exc:
            self._latch(str(exc))
            raise
        return True

    def _emergency(self):
        # A partial old command is terminated first. Firmware rejects it and
        # forces OFF if armed, before the explicit OFF/DISABLE requests.
        try:
            self.transport.write(b"\nFB_OFF\nDISABLE\n")
        except Exception:
            pass

    def _latch(self, reason):
        if self.fault is None:
            self.fault = reason
            if self.hello is not None:
                self._emergency()
        self.ready = False
        self.armed_session = None
        self.pending = None
        self._phase = None
        self._tracker.invalidate(reason)
        self.pressure = self._tracker.frame

    def start(self, now_ns):
        now_ns = self._time(now_ns)
        if self._started or self._closed or self.fault:
            raise LinkError("Create a new link for an explicit new connection")
        self._started = True
        if hasattr(self.transport, "reset_input_buffer"):
            self.transport.reset_input_buffer()
        return self._request("CONTROL_HELLO", ("#HELLO", "OASIS_FALLBACK"), now_ns, phase="control_hello")

    def _healthy(self, now_ns, *, require_armed=False):
        now_ns = self._time(now_ns)
        if self._closed or self.fault or not self.ready or self._off_requested:
            raise LinkError(self.fault or "Fallback handshake is not ready")
        if self.state is None or now_ns-self.state.received_ns > 400_000_000:
            self._latch("state_stale")
            raise LinkError(self.fault)
        if require_armed and self.armed_session is None:
            raise LinkError("Explicit ARM has not been acknowledged")
        return now_ns

    def arm(self, now_ns):
        now_ns = self._healthy(now_ns)
        if self.pending:
            return False
        if self.armed_session is not None or self.state.armed:
            raise LinkError("Already armed; ARM is never replayed")
        self.pressure = self._tracker.at(now_ns)
        if not self.pressure or not self.pressure.valid:
            raise LinkError("Fresh complete pressure sweep required before ARM")
        sid = self.hello["session_floor"]+1
        if sid > UINT32_MAX:
            raise LinkError("Session ID exhausted")
        self.hello["session_floor"] = sid
        return self._request(f"FB_ARM,{sid}", ("#ACK", "FB_ARM", str(sid)), now_ns)

    def set_pose(self, pose_mdeg, now_ns):
        now_ns = self._healthy(now_ns, require_armed=True)
        target = _pose(pose_mdeg)
        if self.pending:
            return False
        self._command_id += 1
        if self._command_id > UINT32_MAX:
            self._latch("command_id_exhausted")
            raise LinkError(self.fault)
        fields = ("FB_SET", str(self.armed_session), str(self._command_id), *(str(x) for x in target))
        return self._request(",".join(fields), ("#ACK", *fields), now_ns)

    def ping(self, now_ns):
        now_ns = self._healthy(now_ns, require_armed=True)
        if self.pending:
            return False
        self._ping_id += 1
        if self._ping_id > UINT32_MAX:
            self._latch("ping_id_exhausted")
            raise LinkError(self.fault)
        fields = ("FB_PING", str(self.armed_session), str(self._ping_id))
        return self._request(",".join(fields), ("#ACK", *fields), now_ns)

    def pressure_ping(self, now_ns):
        now_ns = self._healthy(now_ns)
        return self._request("PING", ("#PONG", "12"), now_ns)

    def request_state(self, now_ns):
        now_ns = self._healthy(now_ns)
        return self._request("STATE", ("#STATE",), now_ns)

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
        self._ignored_ack = self.pending.expected if self.pending else None
        self.pending = None
        return self._request("FB_OFF", ("#ACK", "FB_OFF"), self._last_now_ns or 0, phase="off")

    def _accepted(self, fields, now_ns):
        pending, phase = self.pending, self._phase
        self.pending = None
        self._phase = None
        self.last_ack = {"command": pending.command, "op": pending.command.split(",")[0],
                         "received_ns": now_ns, "fields": tuple(fields)}
        if phase == "control_hello":
            self._request("HELLO", ("#HELLO", "FSR_12_SCAN"), now_ns, phase="pressure_hello")
        elif phase == "pressure_hello":
            self._request("DISABLE", ("#CONTROL", "12", "MASK=0", "EN=1"), now_ns, phase="disable")
        elif phase == "disable":
            self._request("START_12,15", ("#CONTROL", "12", "MASK=15", "EN=0"), now_ns, phase="scan")
        elif phase == "scan":
            self._scan = True
            self._pressure_ping_ns = now_ns
            self._request("STATE", ("#STATE",), now_ns, phase="initial_state")
        elif phase == "initial_state":
            if self.state.armed:
                self._latch("unexpected_armed_initial_state")
            else:
                self.ready = True
        elif phase == "off":
            self._request("DISABLE", ("#CONTROL", "12", "MASK=0", "EN=1"), now_ns, phase="off_disable")
        elif phase == "off_disable":
            self._scan = False
            self._tracker.invalidate("scan_disabled")
            self._request("STATE", ("#STATE",), now_ns, phase="off_state")
        elif phase == "off_state":
            if self.state.armed or any(self.state.pulses_us):
                self._latch("off_not_verified")
        elif fields[:2] == ("#ACK", "FB_ARM"):
            self.armed_session = _uint(fields[2])
            self._command_id = self._ping_id = 0
        elif fields[:2] == ("#ACK", "FB_PING"):
            if now_ns-self._pressure_ping_ns >= 1_000_000_000:
                self._request("PING", ("#PONG", "12"), now_ns)
        elif fields == ("#PONG", "12"):
            self._pressure_ping_ns = now_ns

    def _line(self, line, now_ns):
        fields = tuple(line.split(","))
        if fields[0] == "#STATE":
            state = parse_state(line, now_ns)
            if self.state and ((state.tick_ms-self.state.tick_ms) & UINT32_MAX) > 0x7fffffff:
                raise ValueError("MCU state clock reset")
            self.state = state
            if self.armed_session is not None and (not state.armed or state.fault != "NONE"):
                raise ValueError("Motor session lost: "+state.fault)
            if state.fault not in ("BOOT", "OFF", "NONE"):
                raise ValueError("Board fault: "+state.fault)
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
            for item in fields[1:]:
                key, separator, value = item.partition("=")
                if not separator:
                    raise ValueError("Malformed FSR statistics")
                number = _uint(value)
                previous = self._stats.get(key)
                self._stats[key] = number
                if key in ("adc_errors", "uart_errors", "rx_errors", "aborted") and number:
                    raise ValueError("FSR error counter: "+key)
                if key in ("overruns", "skipped_cycles") and previous is not None and number != previous:
                    raise ValueError("Pressure acquisition counter changed: "+key)
                if self._scan and ((key == "scan_mask" and number != 15) or
                                   (key == "scan_enabled" and number != 1) or
                                   (key == "dwt_ready" and number != 1)):
                    raise ValueError("FSR scan configuration lost")
            return
        elif fields[0].isdigit():
            if self._scan:
                self.pressure = self._tracker.update(line, now_ns)
                if self.pressure is not None and not self.pressure.valid:
                    raise ValueError(self.pressure.reason)
            return
        elif fields[0] in ("#COMMANDS", "seq"):
            return
        elif fields[0] not in ("#ACK", "#PONG", "#CONTROL"):
            raise ValueError("Unknown UART line")

        if self._ignored_ack and fields == self._ignored_ack:
            self._ignored_ack = None
            return
        pending = self.pending
        matched = pending and (fields == pending.expected or
            (pending.expected == ("#STATE",) and fields[0] == "#STATE") or
            (self._phase in ("control_hello", "pressure_hello") and fields[:2] == pending.expected))
        if matched:
            self._accepted(fields, now_ns)
        elif fields[0] != "#STATE":
            raise ValueError("Unexpected or mismatched command response")

    def poll(self, now_ns):
        now_ns = self._time(now_ns)
        if self._closed:
            return
        if self._last_poll_ns is not None and now_ns-self._last_poll_ns > MAX_POLL_GAP_NS and self.armed_session is not None:
            self._latch("host_poll_gap")
        self._last_poll_ns = now_ns
        if self.pending and now_ns-self.pending.sent_ns >= ACK_TIMEOUT_NS:
            self._latch("ack_timeout:"+self.pending.command.split(",")[0])
        try:
            waiting = self.transport.in_waiting
            if type(waiting) is not int or waiting < 0 or waiting > MAX_READ_BYTES:
                raise ValueError("UART receive backlog exceeded")
            if waiting:
                data = self.transport.read(min(waiting, MAX_READ_BYTES))
                if not isinstance(data, bytes) or len(data) > waiting:
                    raise ValueError("Invalid transport read")
                self._buffer.extend(data)
            lines = 0
            while b"\n" in self._buffer and lines < MAX_LINES_PER_POLL:
                newline = self._buffer.index(10)
                if newline > MAX_LINE_BYTES:
                    raise ValueError("UART line too long")
                raw = bytes(self._buffer[:newline]).rstrip(b"\r")
                del self._buffer[:newline+1]
                lines += 1
                if not raw:
                    continue
                if any(byte < 32 or byte > 126 for byte in raw):
                    raise ValueError("Non-ASCII/control byte in UART response")
                if not self.fault:
                    self._line(raw.decode("ascii"), now_ns)
                elif raw.startswith(b"#STATE,"):
                    # Preserve observed OFF/fault evidence even after latching.
                    self.state = parse_state(raw.decode("ascii"), now_ns)
            if len(self._buffer) > MAX_LINE_BYTES or b"\n" in self._buffer:
                raise ValueError("UART parser backlog exceeded")
        except Exception as exc:
            self._buffer.clear()
            self._latch(f"protocol:{type(exc).__name__}:{exc}")
        self.pressure = self._tracker.at(now_ns)

    def close(self):
        if self._closed:
            return
        try:
            if self.hello is not None:
                self._emergency()
        finally:
            self._closed = True
            self.ready = False
            self.armed_session = None
            self.transport.close()
