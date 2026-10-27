"""Motor-only protocol verification against injected UART; no hardware calls."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from experiment_a import transport
from experiment_a.tests.runtime_fixtures import FakeClock, MotorUART


class TransportTests(unittest.TestCase):
    def test_exact_270_degree_firmware_and_tuning_contract(self):
        hello = '#HELLO,OASIS_FALLBACK,v2,'+','.join(
            f'{k}={v}' for k,v in transport.HELLO_FIELDS.items())+',session_floor=0'
        self.assertEqual(transport.parse_hello(hello)['version'],'v2')
        self.assertEqual(transport.parse_hello(hello)['neutral_us'],'1800:1800:1200:1200')
        for wrong in (hello.replace('v2','v1'), hello.replace('servo_range_deg=270','servo_range_deg=180'),
                      hello.replace('neutral_us=1800:1800:1200:1200','neutral_us=1800:1200:1200:1800')):
            with self.assertRaises(ValueError):transport.parse_hello(wrong)
        tune = '#TUNE,OASIS_A,v1,revision=7,yaw_max_mdeg=15000,pitch_max_mdeg=6000,yaw_rate_mdeg_s=2000,pitch_rate_mdeg_s=2000,full_scale_ay_mmps2=2942,pitch_scale_permille=1000'
        self.assertEqual(transport.parse_tuning(tune)['revision'],7)
        for wrong in (tune.replace('15000','22001'),tune+',revision=8',
                      tune.replace('6000','6001'),tune.replace('2000','0')):
            with self.assertRaises(ValueError):transport.parse_tuning(wrong)

    def setup_link(self, **kwargs):
        clock = FakeClock(100.)
        uart = MotorUART(clock, **kwargs)
        link = transport.BoardLink(uart)
        link.start(clock.now_ns)
        for _ in range(8):
            clock.now_ns += 2_000_000
            link.poll(clock.now_ns)
        self.assertTrue(link.ready)
        self.assertIsNone(link.fault)
        return clock, uart, link

    def test_old_left_neutral_calibration_never_reaches_arm(self):
        class PreviousNeutralUART(MotorUART):
            def emit(self, line):
                if line.startswith('#HELLO,OASIS_FALLBACK,'):
                    line=line.replace('neutral_us=1800:1800:1200:1200',
                                      'neutral_us=1800:1200:1200:1800')
                return super().emit(line)
        clock=FakeClock(100.)
        uart=PreviousNeutralUART(clock)
        link=transport.BoardLink(uart)
        link.start(clock.now_ns)
        for _ in range(8):
            clock.now_ns+=2_000_000
            link.poll(clock.now_ns)
        self.assertIn('calibration', link.fault)
        self.assertFalse(link.ready)
        with self.assertRaises(transport.LinkError):
            link.arm(clock.now_ns)
        self.assertFalse(any(c.startswith('FB_ARM') for _,c in uart.commands))

    def test_verified_off_then_arm_needs_no_pressure_frame(self):
        clock, uart, link = self.setup_link()
        self.assertEqual([c for _, c in uart.commands], ["CONTROL_HELLO", "FB_OFF", "CAD_INFO", "TUNE_STATE", "DISABLE", "STATE"])
        self.assertFalse(hasattr(link, "pressure"))
        self.assertTrue(link.arm(clock.now_ns))
        clock.now_ns += 2_000_000
        link.poll(clock.now_ns)
        self.assertEqual(link.armed_session, 1)
        self.assertTrue(link.set_pose((10000, -3000, 10000, 3000), clock.now_ns))
        clock.now_ns += 2_000_000
        link.poll(clock.now_ns)
        self.assertIsNone(link.fault)
        self.assertIsNone(link.pending)

    def test_unusable_residual_pressure_cannot_change_motor_health(self):
        clock, uart, link = self.setup_link(noise=True)
        uart.emit("99,nan,broken,pressure")
        uart.emit("#STATS,adc_errors=999,skipped_cycles=999")
        link.poll(clock.now_ns)
        self.assertIsNone(link.fault)
        self.assertTrue(link.ready)
        self.assertTrue(link.arm(clock.now_ns))
        clock.now_ns += 2_000_000
        link.poll(clock.now_ns)
        self.assertEqual(link.armed_session, 1)

    def test_legacy_or_wrong_cad_firmware_cannot_arm(self):
        for reply in ('#ERR,UNKNOWN', transport.CAD_INFO_LINE.replace('yaw_max_mdeg=22000','yaw_max_mdeg=27000')):
            class WrongCAD(MotorUART):
                def write(self, data):
                    if data == b'CAD_INFO\n':
                        self.emit(reply)
                        return len(data)
                    return super().write(data)
            clock=FakeClock(100.)
            uart=WrongCAD(clock)
            link=transport.BoardLink(uart)
            link.start(clock.now_ns)
            for _ in range(8):
                clock.now_ns+=2_000_000
                link.poll(clock.now_ns)
            self.assertIsNotNone(link.fault)
            self.assertFalse(link.cad_verified)
            with self.assertRaises(transport.LinkError):link.arm(clock.now_ns)
            self.assertFalse(any(c.startswith('FB_ARM') for _,c in uart.commands))

    def test_transport_cannot_bypass_coupled_cad_cap(self):
        clock,uart,link=self.setup_link()
        link.arm(clock.now_ns)
        clock.now_ns+=2_000_000
        link.poll(clock.now_ns)
        for pose in ((22000,1,0,0),(20000,2001,0,0),(23000,0,0,0),(0,1,0,0)):
            with self.assertRaises(transport.LinkError):link.set_pose(pose,clock.now_ns)

    def test_exact_ack_mismatch_latches_off(self):
        clock, uart, link = self.setup_link()
        link.arm(clock.now_ns)
        uart.buffer.clear()
        uart.emit("#ACK,FB_ARM,2")
        clock.now_ns += 2_000_000
        link.poll(clock.now_ns)
        self.assertIn("mismatched", link.fault)
        self.assertFalse(link.ready)
        self.assertIsNone(link.armed_session)
        self.assertTrue(any(c == "FB_OFF" for _, c in uart.commands[4:]))

    def test_arm_ack_timeout_and_no_rearm(self):
        clock, uart, link = self.setup_link()
        link.arm(clock.now_ns)
        uart.buffer.clear()
        uart.silent = True
        clock.now_ns += transport.ACK_TIMEOUT_NS
        link.poll(clock.now_ns)
        self.assertEqual(link.fault, "ack_timeout:FB_ARM")
        with self.assertRaises(transport.LinkError): link.arm(clock.now_ns)

    def test_armed_host_loop_gap_withdraws_authority(self):
        clock, uart, link = self.setup_link()
        link.arm(clock.now_ns)
        clock.now_ns += 2_000_000
        link.poll(clock.now_ns)
        clock.now_ns += transport.MAX_POLL_GAP_NS+1
        link.poll(clock.now_ns)
        self.assertEqual(link.fault, "host_poll_gap")

    def test_clock_regression_withdraws_authority(self):
        clock, uart, link = self.setup_link()
        with self.assertRaises(transport.LinkError): link.poll(clock.now_ns-1)
        self.assertEqual(link.fault, "host_clock_regression")

    def test_board_reboot_and_stale_state_are_faults(self):
        clock, uart, link = self.setup_link()
        uart.emit("#BOOT,reset")
        link.poll(clock.now_ns)
        self.assertIn("MCU reboot", link.fault)
        clock, uart, link = self.setup_link()
        clock.now_ns += 400_000_001
        with self.assertRaises(transport.LinkError): link.arm(clock.now_ns)
        self.assertEqual(link.fault, "state_stale")

    def test_bounds_and_single_pending_request(self):
        clock, uart, link = self.setup_link()
        link.arm(clock.now_ns)
        clock.now_ns += 2_000_000
        link.poll(clock.now_ns)
        with self.assertRaises(ValueError): link.set_pose((27001, 0, 0, 0), clock.now_ns)
        with self.assertRaises(ValueError): link.set_pose((0, 8001, 0, 0), clock.now_ns)
        self.assertTrue(link.ping(clock.now_ns))
        count = len(uart.commands)
        self.assertFalse(link.set_pose((1, 0, 0, 0), clock.now_ns))
        self.assertEqual(len(uart.commands), count)

    def test_motor_clock_wrap_is_valid_but_regression_latches(self):
        from dataclasses import replace
        clock, uart, link = self.setup_link()
        link.state = replace(link.state, tick_ms=0xffffffff-5)
        fields = uart.state_line().split(",")
        fields[1] = "4"
        uart.emit(",".join(fields))
        link.poll(clock.now_ns)
        self.assertIsNone(link.fault)
        self.assertEqual(link.state.tick_ms, 4)
        fields[1] = "3"
        uart.emit(",".join(fields))
        link.poll(clock.now_ns)
        self.assertIn("MCU state clock reset", link.fault)

    def test_off_preempts_pending_target_and_ignores_only_its_exact_ack(self):
        clock, uart, link = self.setup_link()
        link.arm(clock.now_ns)
        clock.now_ns += 2_000_000
        link.poll(clock.now_ns)
        link.set_pose((1000, 0, 0, 0), clock.now_ns)
        self.assertTrue(link.pending)
        self.assertTrue(link.off())
        for _ in range(4):
            clock.now_ns += 2_000_000
            link.poll(clock.now_ns)
        self.assertIsNone(link.fault)
        self.assertFalse(link.state.armed)
        self.assertFalse(any(link.state.pulses_us))
        with self.assertRaises(transport.LinkError): link.arm(clock.now_ns)

    def test_motor_only_off_preempts_target_and_permanently_blocks_motor_commands(self):
        clock, uart, link = self.setup_link()
        link.arm(clock.now_ns)
        clock.now_ns += 2_000_000
        link.poll(clock.now_ns)
        link.set_pose((1000, 0, 0, 0), clock.now_ns)
        before = len(uart.commands)
        self.assertTrue(link.motor_off_keep_recording(clock.now_ns))
        self.assertFalse(link.ready)
        self.assertFalse(link.motor_off_verified)
        for _ in range(4):
            clock.now_ns += 2_000_000
            link.poll(clock.now_ns)
        self.assertIsNone(link.fault)
        self.assertTrue(link.ready and link.motor_inhibited and link.motor_off_verified)
        self.assertFalse(link.state.armed)
        self.assertFalse(any(link.state.pulses_us))
        self.assertEqual([c for _, c in uart.commands[before:]], ['FB_OFF', 'STATE'])
        for operation in (lambda: link.arm(clock.now_ns),
                          lambda: link.set_pose((1000, 0, 0, 0), clock.now_ns),
                          lambda: link.ping(clock.now_ns)):
            with self.assertRaises(transport.LinkError): operation()
        self.assertTrue(link.request_state(clock.now_ns))

    def test_motor_inhibit_before_first_arm_never_allows_arm(self):
        clock, uart, link = self.setup_link()
        link.motor_off_keep_recording(clock.now_ns)
        for _ in range(4):
            clock.now_ns += 2_000_000
            link.poll(clock.now_ns)
        self.assertTrue(link.motor_off_verified)
        with self.assertRaises(transport.LinkError): link.arm(clock.now_ns)
        self.assertFalse(any(c.startswith('FB_ARM,') for _, c in uart.commands))

    def test_motor_resuming_after_recording_only_off_is_fatal(self):
        clock, uart, link = self.setup_link()
        link.motor_off_keep_recording(clock.now_ns)
        for _ in range(4):
            clock.now_ns += 2_000_000
            link.poll(clock.now_ns)
        self.assertTrue(link.motor_off_verified)
        uart.armed = True
        uart.emit(uart.state_line())
        link.poll(clock.now_ns)
        self.assertIn('Motor output resumed after latched OFF', link.fault)
        self.assertFalse(link.ready)

    def test_motor_only_off_must_get_new_ack_and_zero_state(self):
        clock, uart, link = self.setup_link(suppress_final_off=True)
        link.motor_off_keep_recording(clock.now_ns)
        clock.now_ns += transport.ACK_TIMEOUT_NS
        link.poll(clock.now_ns)
        self.assertEqual(link.fault, 'ack_timeout:FB_OFF')
        self.assertFalse(link.motor_off_verified)


if __name__ == "__main__": unittest.main()
