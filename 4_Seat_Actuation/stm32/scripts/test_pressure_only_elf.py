"""Offline instruction/MMIO tests of the exact pressure-only ARM ELF.

Requires unicorn (or OASIS_EMULATION_DEPS pointing to an existing install).
UART transmission is captured in memory;
no serial port, probe, flash, GPIO device or other real hardware is opened.
"""
from pathlib import Path
import os
import struct
import sys
import unittest

if os.environ.get("OASIS_EMULATION_DEPS"):
    sys.path.insert(0, os.environ["OASIS_EMULATION_DEPS"])
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE, UC_HOOK_MEM_WRITE
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_PC, UC_ARM_REG_R0, UC_CPU_ARM_CORTEX_M7

ROOT = Path(__file__).resolve().parents[1]
ELF = ROOT / "build_pressure_only/pressure_only_h723.elf"
GPIOE, TIM1 = 0x58021000, 0x40010000
PINS = sum(1 << pin for pin in (9, 11, 13, 14))
MODE_MASK = sum(3 << (2 * pin) for pin in (9, 11, 13, 14))
OUTPUT_MODE = sum(1 << (2 * pin) for pin in (9, 11, 13, 14))
CCR = (0x34, 0x38, 0x3C, 0x40)


class Machine:
    def __init__(self):
        self.uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        self.uc.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M7)
        for base, size in ((0x08000000, 0x100000), (0x20000000, 0x20000),
                           (0x24000000, 0x80000), (0x30000000, 0x80000),
                           (0x40010000, 0x1000), (0x58020000, 0x10000),
                           (0xE000E000, 0x3000)):
            self.uc.mem_map(base, size)
        data = ELF.read_bytes()
        header = struct.unpack_from("<16sHHIIIIIHHHHHH", data)
        if data[:6] != b"\x7fELF\x01\x01":
            raise ValueError("Expected little-endian ARM ELF32")
        for index in range(header[10]):
            p = struct.unpack_from("<IIIIIIII", data, header[5] + index * header[9])
            if p[0] == 1 and p[4]:
                self.uc.mem_write(p[2], data[p[1]:p[1] + p[4]])
        sections = [struct.unpack_from("<IIIIIIIIII", data, header[6] + i * header[11])
                    for i in range(header[12])]
        self.symbols = {}
        for section in sections:
            if section[1] != 2:
                continue
            strings = sections[section[6]]
            names = data[strings[4]:strings[4] + strings[5]]
            for offset in range(section[4], section[4] + section[5], section[9]):
                name, value, _, _, _, _ = struct.unpack_from("<IIIBBH", data, offset)
                self.symbols[names[name:names.find(b"\0", name)].decode("ascii")] = value
        self.tx = []
        self.writes = []
        self.uc.hook_add(UC_HOOK_CODE, self.code)
        self.uc.hook_add(UC_HOOK_MEM_WRITE, self.write)

    def word(self, address, value=None):
        if value is None:
            return struct.unpack("<I", self.uc.mem_read(address, 4))[0]
        self.uc.mem_write(address, struct.pack("<I", value))

    def cstring(self, address):
        data = bytearray()
        while True:
            byte = self.uc.mem_read(address + len(data), 1)[0]
            if not byte:
                return data.decode("ascii")
            data.append(byte)

    def code(self, uc, address, size, user):
        if address == (self.symbols["FSR_Transmit"] & ~1):
            self.tx.append(self.cstring(uc.reg_read(UC_ARM_REG_R0)))
            uc.reg_write(UC_ARM_REG_PC, uc.reg_read(UC_ARM_REG_LR))

    def write(self, uc, access, address, size, value, user):
        self.writes.append((address, value))
        if address == GPIOE + 0x18:  # BSRR, model GPIO output latch only.
            odr = (self.word(GPIOE + 0x14) | (value & 0xFFFF)) & ~(value >> 16)
            self.word(GPIOE + 0x14, odr & 0xFFFF)
            self.word(GPIOE + 0x10, odr & 0xFFFF)

    def call(self, name, text=None):
        self.uc.reg_write(UC_ARM_REG_SP, 0x2404F000)
        self.uc.reg_write(UC_ARM_REG_LR, 0x080FF001)
        if text is not None:
            self.uc.mem_write(0x24048000, text.encode("ascii") + b"\0")
            self.uc.reg_write(UC_ARM_REG_R0, 0x24048000)
        self.uc.emu_start(self.symbols[name] | 1, 0x080FF000, count=300000)
        return self.uc.reg_read(UC_ARM_REG_R0)

    def seed_active(self):
        self.word(GPIOE, 0xAAAAAAAA)
        self.word(GPIOE + 0x14, 0xFFFF)
        self.word(GPIOE + 0x10, 0xFFFF)
        self.word(GPIOE + 0x24, 0xFFFFFFFF)
        self.word(TIM1, 0x81)
        self.word(TIM1 + 0x20, 0x1111)
        self.word(TIM1 + 0x44, 0x8000)
        for register, pulse in zip(CCR, (1800, 1200, 1200, 1800)):
            self.word(TIM1 + register, pulse)

    def assert_off(self, test):
        test.assertEqual(self.word(GPIOE) & MODE_MASK, OUTPUT_MODE)
        test.assertEqual(self.word(GPIOE + 0x14) & PINS, 0)
        test.assertEqual(self.word(TIM1) & 1, 0)
        test.assertEqual(self.word(TIM1 + 0x20), 0)
        test.assertEqual(self.word(TIM1 + 0x44) & 0x8000, 0)
        test.assertEqual([self.word(TIM1 + r) for r in CCR], [0] * 4)
        for address, value in self.writes:
            if address == TIM1:
                test.assertEqual(value & 1, 0)
            if address == TIM1 + 0x20:
                test.assertEqual(value, 0)
            if address == TIM1 + 0x44:
                test.assertEqual(value & 0x8000, 0)


class PressureOnlyTests(unittest.TestCase):
    def test_old_active_outputs_are_removed_by_first_boot_operation(self):
        m = Machine(); m.seed_active(); m.call("PressureOnly_MotorOff"); m.assert_off(self)

    def test_no_motion_implementation_in_linked_image(self):
        names = Machine().symbols
        self.assertFalse(any(name.startswith("Joint_") for name in names))
        self.assertNotIn("HAL_TIM_PWM_Start", names)
        self.assertNotIn("HAL_TIM_PWM_Init", names)

    def test_arm_set_malformed_unknown_hold_and_state_never_enable_pwm(self):
        for command in ("ARM", "SET", "SET,1,27000,0,27000,0", "SET,999999999999,x", "ARM,1", "HOLD", "STATE", "garbage"):
            with self.subTest(command=command):
                m = Machine(); m.seed_active(); m.call("Control_Init")
                m.call("Control_Command", command); m.call("Control_Poll"); m.assert_off(self)
                if command in ("ARM", "SET") or command.startswith("SET,"):
                    self.assertEqual(m.tx, ["#ERR,0,PRESSURE_ONLY\r\n"])

    def test_unique_pressure_identity(self):
        m = Machine(); m.call("Control_Init"); m.call("Control_Command", "CONTROL_HELLO")
        self.assertEqual(m.tx, ["#HELLO,OASIS_PRESSURE_ONLY,v1,bits=12,hz=100,banks=3,max_channels=16,motor_outputs=0,pwm=OFF\r\n"])
        m.assert_off(self)

    def test_poll_and_fault_restore_off_without_mux_gpio_writes(self):
        for function in ("Control_Poll", "Control_Fault"):
            m = Machine(); m.seed_active(); m.call(function, "TEST"); m.assert_off(self)
            self.assertFalse(any(0x58020000 <= address < 0x58021000 or
                                 0x58021400 <= address < 0x58022000 for address, _ in m.writes))


if __name__ == "__main__":
    unittest.main(verbosity=2)
