"""Invented UART/vehicle fixtures: never evidence of actual motor behavior."""
from __future__ import annotations

from dataclasses import dataclass, field
from fallback.tests.test_runner import FakeClock, FakeSocket
from experiment_a import transport
from experiment_a.settings import Settings


class MotorUART:
    timeout = write_timeout = 0

    def __init__(self, clock, *, noise=False, suppress_final_off=False, nonneutral_arm=False):
        self.clock = clock
        self.closed = self.armed = self.silent = False
        self.buffer = bytearray()
        self.commands = []
        self.sid = self.command_id = self.off_count = 0
        self.pose = (0, 0, 0, 0)
        self.next_state_ns = clock.now_ns
        self.noise, self.suppress_final_off = noise, suppress_final_off
        self.nonneutral_arm = nonneutral_arm

    def emit(self, line):
        self.buffer.extend((line+"\r\n").encode("ascii"))

    def state_line(self):
        pulses = transport.NEUTRAL_US if self.armed else (0, 0, 0, 0)
        fields = (self.clock.now_ns//1_000_000, int(self.armed), 0, *self.pose, *self.pose,
                  *pulses, self.command_id, "NONE" if self.armed else "OFF")
        return "#STATE,"+",".join(map(str, fields))

    def write(self, data):
        for command in data.decode("ascii").splitlines():
            if not command:
                continue
            self.commands.append((self.clock.now_ns, command))
            if self.silent:
                continue
            fields = command.split(",")
            if command == "CONTROL_HELLO":
                self.emit("#HELLO,OASIS_FALLBACK,v2,"+",".join(
                    f"{key}={value}" for key, value in transport.HELLO_FIELDS.items())+
                    ",session_floor="+str(self.sid))
            elif command == "CAD_INFO":
                self.emit(transport.CAD_INFO_LINE)
            elif command == 'TUNE_STATE':
                self.emit('#TUNE,OASIS_A,v1,revision=0,yaw_max_mdeg=15000,pitch_max_mdeg=6000,'
                    'yaw_rate_mdeg_s=2000,pitch_rate_mdeg_s=2000,full_scale_ay_mmps2=2942,pitch_scale_permille=1000')
            elif command == "DISABLE":
                self.emit("#CONTROL,12,MASK=0,EN=1")
            elif command == "STATE":
                self.emit(self.state_line())
            elif fields[0] == "FB_ARM":
                self.sid = int(fields[1])
                self.armed = True
                self.pose = (1, 0, 0, 0) if self.nonneutral_arm else (0, 0, 0, 0)
                self.emit("#ACK,"+command)
            elif fields[0] == "FB_SET":
                assert self.armed and int(fields[1]) == self.sid
                self.command_id = int(fields[2])
                self.pose = tuple(map(int, fields[3:]))
                self.emit("#ACK,"+command)
            elif fields[0] == "FB_PING":
                assert self.armed and int(fields[1]) == self.sid
                self.emit("#ACK,"+command)
            elif command == "FB_OFF":
                self.off_count += 1
                if self.suppress_final_off and self.off_count > 1:
                    self.silent = True
                    self.buffer.clear()
                    continue
                self.armed = False
                self.emit("#ACK,FB_OFF")
            else:
                raise AssertionError("Motor-only transport sent unsupported command: "+command)
        return len(data)

    @property
    def in_waiting(self):
        if not self.silent and self.clock.now_ns >= self.next_state_ns:
            self.emit(self.state_line())
            self.next_state_ns = self.clock.now_ns+200_000_000
            if self.noise:
                # These are deliberately unusable as pressure measurements.
                # The A path must not decode, baseline, or gate on them.
                self.emit("44,garbage,stale,adc,error")
                self.emit("#STATS,adc_errors=999,scan_enabled=0")
        return len(self.buffer)

    def read(self, count):
        value = bytes(self.buffer[:count])
        del self.buffer[:count]
        return value

    def reset_input_buffer(self): self.buffer.clear()
    def close(self): self.closed = True


@dataclass
class RuntimeSettings(Settings):
    """Launcher-only fixture. These flags are not actual hardware approvals."""
    fresh_ms: float = 200.
    startup_seconds: float = 2.
    acceleration_source: str = "linear_acceleration"
    max_model_sample_dt_s: float = .1
    confirmations: dict = field(default_factory=lambda: {"fixture_only": True})

    def validate(self): pass
    def missing_confirmations(self):
        return [name for name, enabled in self.confirmations.items() if not enabled]


@dataclass
class RuntimeDecision:
    mode: str
    reason: str
    pose_mdeg: tuple[int, int, int, int]
    permit_lease: bool = True
    emergency_off: bool = False
    diag: dict = field(default_factory=dict)


class RuntimeController:
    """Invented deterministic target for launcher tests, not the human model."""
    def __init__(self, settings): self.settings = settings
    def tick(self, vehicle, state, now_ns):
        if now_ns-state.received_ns > self.settings.fresh_ms*1e6:
            return RuntimeDecision("FAULT", "board_state_stale", (0, 0, 0, 0), False, True)
        pose = (0, 0, 1000, 500) if vehicle.ay_mps2 > 0 else (1000, -500, 0, 0)
        return RuntimeDecision("RUN", "ok", pose)
