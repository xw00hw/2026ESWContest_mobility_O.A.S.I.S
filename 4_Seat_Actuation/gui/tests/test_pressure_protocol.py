import unittest
from pathlib import Path
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))

from pressure_protocol import SequenceTracker, parse_packet


class PressureProtocolTests(unittest.TestCase):
    def test_three_banks_and_error_status_preserve_missing_raw(self):
        self.assertEqual(parse_packet('1,20,3,0,0,,1,4095,0'),
                         (1,20,3,[(0,0),(None,1),(4095,0)]))

    def test_invalid_packet_is_rejected(self):
        for packet in ['1,2', '-1,20,0,0,0,0,0,0,0', '1,20,16,0,0,0,0,0,0',
                       '1,20,0,4096,0,0,0,0,0', '1,20,0,1,1,0,0,0,0',
                       '1,20,0,,6,0,0,0,0']:
            with self.subTest(packet=packet),self.assertRaises(ValueError):
                parse_packet(packet)

    def test_uint32_wrap_missing_duplicate_and_reset(self):
        tracker=SequenceTracker()
        tracker.accept(0xfffffffe,0xfffffff0)
        extended,elapsed=tracker.accept(1,5)
        self.assertEqual(extended,0xfffffff0+21)
        self.assertAlmostEqual(elapsed,.021)
        self.assertEqual(tracker.missing,2)
        self.assertIsNone(tracker.accept(1,5))
        self.assertEqual(tracker.duplicates,1)
        with self.assertRaises(RuntimeError):
            tracker.accept(0,4)


if __name__=='__main__':
    unittest.main()
