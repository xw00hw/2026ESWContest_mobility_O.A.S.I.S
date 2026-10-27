"""Offline UART protocol tests. Every transport is memory-only; no devices."""
from __future__ import annotations

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from fallback.serial_link import (BoardLink, LinkError, PressureTracker, parse_hello,
    parse_state, open_serial, HELLO_FIELDS, FSR_HELLO, NEUTRAL_US, UINT32_MAX,
    ACK_TIMEOUT_NS, MAX_READ_BYTES)


def hello(floor=0):
    return "#HELLO,OASIS_FALLBACK,v1,"+",".join(f"{k}={v}" for k,v in HELLO_FIELDS.items())+f",session_floor={floor}"


def state(tick=100, armed=False, pose=(0,0,0,0), target=None, command_id=0, fault=None):
    target = pose if target is None else target
    fields = ("#STATE", tick, int(armed), int(pose != target), *pose, *target,
              *(NEUTRAL_US if armed else (0,0,0,0)), command_id, fault or ("NONE" if armed else "BOOT"))
    return ",".join(str(x) for x in fields)


def sample(seq, tick, channel, raw=(1000,2000,3000)):
    return f"{seq},{tick},{channel},{raw[0]},0,{raw[1]},0,{raw[2]},0"


class MemorySerial:
    timeout = write_timeout = 0
    def __init__(self):
        self.rx = bytearray()
        self.writes = []
        self.closed = self.opened = False
        self.partial_write = False
        self.write_error = False
        self.kwargs = None

    @property
    def in_waiting(self):
        return len(self.rx)

    def read(self, count):
        value = bytes(self.rx[:count])
        del self.rx[:count]
        return value

    def write(self, value):
        self.writes.append(value)
        if self.write_error:
            raise OSError("fixture disconnected")
        return len(value)-1 if self.partial_write else len(value)

    def feed(self, *lines):
        self.rx.extend(("\r\n".join(lines)+"\r\n").encode("ascii"))

    def reset_input_buffer(self):
        self.rx.clear()

    def open(self):
        self.opened = True

    def close(self):
        self.closed = True


class ParserTests(unittest.TestCase):
    def test_strict_hello_rejects_old_image_bounds_duplicates_and_numeric_aliases(self):
        self.assertEqual(parse_hello(hello(123))["session_floor"], 123)
        for value in (hello().replace("OASIS_FALLBACK", "OASIS_JOINT"), hello().replace("v1", "v6"),
                      hello().replace("neutral_us=1800:1800:1200:1200", "neutral_us=1800:1200:1200:1800"),
                      hello().replace("lease_ms=500", "lease_ms=0500"),
                      hello()+",tick_ms=20", hello().replace("session_floor=0", "session_floor=4294967296")):
            with self.subTest(value=value), self.assertRaises(ValueError):
                parse_hello(value)

    def test_state_off_preserves_command_pose_but_zero_pulses(self):
        value = parse_state(state(pose=(1000,-300,2000,400), fault="OFF"), 99)
        self.assertEqual(value.pose_mdeg, (1000,-300,2000,400))
        self.assertEqual(value.pulses_us, (0,0,0,0))
        self.assertFalse(value.armed)
        self.assertEqual(value.received_ns, 99)

    def test_state_rejects_impossible_arm_or_fault_or_numeric_fields(self):
        for value in (state().replace(",BOOT", ",NONE"), state(armed=True, fault="LEASE"),
                      state(armed=True).replace("1800,1800,1200,1200", "0,0,0,0"),
                      state().replace("#STATE,100", "#STATE,+100"), state()+",extra"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                parse_state(value, 1)

    def test_pressure_full_sweep_oldest_clock_and_voltage_scale(self):
        tracker = PressureTracker()
        for i in range(4):
            frame = tracker.update(sample(i, 100+i*3, i), 10_000_000+i*3_000_000)
            if i < 3:
                self.assertIsNone(frame)
        self.assertTrue(frame.valid)
        self.assertEqual(len(frame.voltages), 12)
        self.assertEqual((frame.received_ns, frame.sequence_id, frame.tick_ms), (10_000_000,3,100))
        self.assertAlmostEqual(frame.voltages["A2_C3"], 3000*3.3/4095)
        tracker.update(sample(4,112,0), 22_000_000)
        self.assertEqual(tracker.frame.received_ns, 10_000_000)
        self.assertEqual(tracker.at(110_000_001).reason, "pressure_stale")

    def test_pressure_gap_reset_status_and_missing_not_filled(self):
        tracker = PressureTracker()
        for i in range(4):
            tracker.update(sample(i,100+i*3,i), i*3_000_000)
        self.assertEqual(tracker.update(sample(5,112,0),12_000_000).reason, "pressure_sequence_gap")
        self.assertFalse(tracker.frame.valid)
        self.assertEqual(tracker.frame.voltages, {})
        tracker.update(sample(6,1,1),15_000_000)
        self.assertFalse(tracker.frame.valid)
        with self.assertRaises(ValueError):
            tracker.update("7,4,2,,2,100,0,200,0",18_000_000)
        with self.assertRaises(ValueError):
            tracker.update("7,4,2,4096,0,100,0,200,0",18_000_000)

    def test_pressure_sequence_and_tick_wrap_are_valid(self):
        tracker = PressureTracker()
        for i in range(4):
            frame = tracker.update(sample((UINT32_MAX-1+i)&UINT32_MAX,
                (UINT32_MAX-4+i*3)&UINT32_MAX,i),i*3_000_000)
        self.assertTrue(frame.valid)

    def test_delayed_transport_cannot_turn_old_pressure_fresh(self):
        tracker = PressureTracker()
        for i in range(4):
            tracker.update(sample(i,100+i*3,i),i*3_000_000)
        # Several small host delays accumulate to >100 ms while MCU advances slowly.
        for i in range(4,8):
            frame = tracker.update(sample(i,100+i*3,i%4),i*25_000_000)
            if frame and not frame.valid:
                break
        self.assertFalse(frame.valid)
        self.assertEqual(frame.reason,"pressure_transport_delay")


class LinkTests(unittest.TestCase):
    def setUp(self):
        self.serial = MemorySerial()
        self.link = BoardLink(self.serial)
        self.now = 1_000_000_000

    def reply(self, *lines, elapsed=1_000_000):
        self.now += elapsed
        self.serial.feed(*lines)
        self.link.poll(self.now)

    def ready(self, floor=7):
        self.link.start(self.now)
        for value in (hello(floor), FSR_HELLO, "#CONTROL,12,MASK=0,EN=1",
                      "#CONTROL,12,MASK=15,EN=0",state()):
            self.reply(value)
        for i in range(4):
            self.reply(sample(i,100+i*3,i),elapsed=3_000_000)
        self.assertIsNone(self.link.fault)
        self.assertTrue(self.link.ready)
        return self.link

    def arm(self):
        self.ready()
        self.assertTrue(self.link.arm(self.now))
        self.reply("#ACK,FB_ARM,8",state(120,armed=True))
        self.assertEqual(self.link.armed_session,8)

    def test_start_handshake_only_and_pressure_scan_exact_order_no_autoarm(self):
        self.ready()
        self.assertEqual(self.serial.writes, [b"CONTROL_HELLO\n",b"HELLO\n",b"DISABLE\n",b"START_12,15\n",b"STATE\n"])
        self.assertIsNone(self.link.pending)
        self.assertFalse(self.link.state.armed)

    def test_wrong_firmware_is_rejected_before_any_pressure_or_motor_command(self):
        self.link.start(self.now)
        self.reply(hello().replace("OASIS_FALLBACK","OASIS_JOINT"))
        self.assertIsNotNone(self.link.fault)
        self.assertEqual(self.serial.writes,[b"CONTROL_HELLO\n"])
        with self.assertRaises(LinkError):
            self.link.arm(self.now)

    def test_arm_uses_floor_plus_one_and_never_replays(self):
        self.arm()
        self.assertIn(b"FB_ARM,8\n",self.serial.writes)
        with self.assertRaises(LinkError):
            self.link.arm(self.now)
        self.assertEqual(self.serial.writes.count(b"FB_ARM,8\n"),1)

    def test_exact_set_ack_and_monotonic_id_no_queued_target(self):
        self.arm()
        self.assertTrue(self.link.set_pose((1000,-200,0,0),self.now))
        self.assertFalse(self.link.set_pose((2000,0,0,0),self.now))
        self.reply("#ACK,FB_SET,8,1,1000,-200,0,0")
        self.assertEqual(self.link.last_ack["op"],"FB_SET")
        self.assertEqual(self.link.last_ack["received_ns"],self.now)
        self.assertTrue(self.link.set_pose((2000,0,0,0),self.now))
        self.assertEqual(self.serial.writes[-1],b"FB_SET,8,2,2000,0,0,0\n")
        self.assertIsNotNone(self.link.pending)

    def test_wrong_ack_session_target_or_id_latches_and_best_effort_off(self):
        self.arm()
        self.link.set_pose((1000,0,0,0),self.now)
        self.reply("#ACK,FB_SET,8,1,1001,0,0,0")
        self.assertIn("mismatched",self.link.fault)
        self.assertEqual(self.serial.writes[-1],b"\nFB_OFF\nDISABLE\n")
        self.assertTrue(self.link.state.armed)  # Never fabricate observed OFF.
        self.assertIsNone(self.link.armed_session)
        with self.assertRaises(LinkError):
            self.link.ping(self.now)

    def test_ack_timeout_is_200ms_no_retries_or_autoheartbeat(self):
        self.arm()
        self.link.ping(self.now)
        command=self.serial.writes[-1]
        self.link.poll(self.now+90_000_000)
        self.link.poll(self.now+180_000_000)
        self.link.poll(self.now+ACK_TIMEOUT_NS)
        self.assertEqual(self.link.fault,"ack_timeout:FB_PING")
        self.assertEqual(self.serial.writes.count(command),1)

    def test_explicit_ping_counters_and_pressure_ping_cannot_arm(self):
        self.ready()
        self.link.pressure_ping(self.now)
        self.reply("#PONG,12")
        self.assertIsNone(self.link.armed_session)
        self.link.arm(self.now)
        self.reply("#ACK,FB_ARM,8",state(120,armed=True))
        self.link.ping(self.now)
        self.reply("#ACK,FB_PING,8,1")
        self.link.ping(self.now)
        self.assertEqual(self.serial.writes[-1],b"FB_PING,8,2\n")

    def test_pressure_ping_scheduled_only_after_caller_motor_ping_not_poll(self):
        self.arm()
        self.link._pressure_ping_ns=self.now-1_000_000_000
        before=len(self.serial.writes)
        self.link.poll(self.now+20_000_000)
        self.assertEqual(len(self.serial.writes),before)
        self.now+=20_000_000
        self.link.ping(self.now)
        self.reply("#ACK,FB_PING,8,1")
        self.assertEqual(self.serial.writes[-1],b"PING\n")

    def test_state_request_and_fault_or_reset_never_auto_rearm(self):
        self.arm()
        self.link.request_state(self.now)
        self.reply(state(140,armed=True))
        self.assertIsNone(self.link.pending)
        self.reply(state(1,armed=False))
        self.assertIn("reset",self.link.fault)
        self.assertEqual(sum(x.startswith(b"FB_ARM") for x in self.serial.writes),1)

    def test_stale_poll_and_pressure_gap_latch_instead_of_freshening_buffer(self):
        self.arm()
        self.link.poll(self.now+100_000_001)
        self.assertEqual(self.link.fault,"host_poll_gap")
        other=LinkTests(); other.setUp(); other.ready()
        other.reply(sample(5,112,0),sample(6,115,1),sample(7,118,2),sample(8,121,3))
        self.assertIn("pressure_sequence_gap",other.link.fault)
        self.assertFalse(other.link.pressure.valid)

    def test_off_preempts_pending_but_requires_actual_off_state(self):
        self.arm()
        self.link.set_pose((1000,0,0,0),self.now)
        self.assertTrue(self.link.off())
        self.reply("#ACK,FB_SET,8,1,1000,0,0,0","#ACK,FB_OFF")
        self.assertTrue(self.link.state.armed)
        self.assertEqual(self.serial.writes[-1],b"DISABLE\n")
        self.reply("#CONTROL,12,MASK=0,EN=1")
        self.assertEqual(self.serial.writes[-1],b"STATE\n")
        self.reply(state(140,fault="OFF"))
        self.assertFalse(self.link.state.armed)
        self.assertIsNone(self.link.pending)
        self.assertIsNone(self.link.fault)

    def test_partial_nonblocking_write_latches_no_command_retries(self):
        self.arm()
        self.serial.partial_write=True
        with self.assertRaises(LinkError):
            self.link.set_pose((1000,0,0,0),self.now)
        self.assertIn("write_failed",self.link.fault)
        self.assertIsNone(self.link.pending)

    def test_bounded_receive_and_partial_lines(self):
        self.ready()
        self.serial.rx.extend(b"#STATE,140,0,0,0,")
        self.link.poll(self.now)
        self.assertIsNone(self.link.fault)
        self.serial.rx.extend(b"0,0,0,0,0,0,0,0,0,0,0,0,BOOT\n")
        self.link.poll(self.now)
        self.assertIsNone(self.link.fault)
        self.serial.rx.extend(b"x"*(MAX_READ_BYTES+1))
        self.link.poll(self.now)
        self.assertIn("backlog",self.link.fault)

    def test_stats_error_adc_error_reboot_and_bounds_reject(self):
        for line in ("#STATS,adc_errors=1", "#BOOT,FSR_12_SCAN,v10", "4,112,0,,2,2,0,3,0"):
            self.setUp(); self.ready(); self.reply(line)
            self.assertIsNotNone(self.link.fault,line)
        self.setUp(); self.arm()
        before=list(self.serial.writes)
        for pose in ((-1,0,0,0),(0,8001,0,0),(True,0,0,0),(0.,0,0,0)):
            with self.assertRaises(ValueError):
                self.link.set_pose(pose,self.now)
        self.assertEqual(before,self.serial.writes)

    def test_close_is_best_effort_off_no_false_state_and_idempotent(self):
        self.arm()
        self.link.close()
        self.assertTrue(self.serial.closed)
        self.assertTrue(self.link.state.armed)
        count=len(self.serial.writes)
        self.link.close()
        self.assertEqual(len(self.serial.writes),count)

    def test_injected_serial_constructor_exclusive_bounded_no_real_port(self):
        fake=MemorySerial()
        def factory(**kwargs):
            fake.kwargs=kwargs
            return fake
        self.assertIs(open_serial("FAKE_ONLY",serial_factory=factory),fake)
        self.assertEqual(fake.kwargs,dict(port=None,baudrate=460800,timeout=0,write_timeout=0,exclusive=True))
        self.assertEqual(fake.port,"FAKE_ONLY")
        self.assertFalse(fake.dtr)
        self.assertFalse(fake.rts)
        self.assertTrue(fake.opened)
        fake.write_timeout=1
        with self.assertRaises(ValueError):
            BoardLink(fake)


if __name__ == "__main__":
    unittest.main(verbosity=2)
