"""Offline force-interlock regressions; no model weights or devices required.

    python -B deploy/runtime/tests/test_nonfinite_force.py
"""
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

RT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RT))
from oasis_rt import controller as C, sources as SRC  # noqa: E402
from oasis_rt.safety import Interlock  # noqa: E402


def interlock(mode='raw', hold_s=0.3, console_N_per_g=221.9):
    return Interlock(dict(fsr_force_soft_N=60, fsr_force_hard_N=120,
                          arm_deg=2, hold_s=hold_s, mode=mode), console_N_per_g)


class NonfiniteForceTests(unittest.TestCase):
    def test_invalid_force_releases_only_affected_wing_at_any_angle(self):
        for invalid in (float('nan'), -float('inf')):
            for side in (0, 1):
                for angle in (0, 2, 10):
                    for mode in ('raw', 'model'):
                        with self.subTest(invalid=invalid, side=side, angle=angle, mode=mode):
                            il = interlock(mode)
                            force = [0.0, 0.0]; force[side] = invalid
                            out = il(1.0, [20.0, 20.0], [angle, angle], force, 0.2)
                            expected = [20.0, 20.0]; expected[side] = 0.0
                            self.assertEqual(out, expected)
                            self.assertEqual(il.flag, 1.0)
                            # Preserve the invalid evidence; never disguise unknown force as zero N.
                            self.assertFalse(math.isfinite(il.force[side]))

    def test_invalid_force_releases_even_with_zero_hold_duration(self):
        for invalid in (float('nan'), -float('inf')):
            with self.subTest(invalid=invalid):
                il = interlock(hold_s=0)
                self.assertEqual(il(0, [20, 20], [0, 0], [invalid, 0], 0), [0, 20])
                self.assertEqual(il.flag, 1.0)

    def test_invalid_release_hold_survives_missing_and_finite_samples_then_expires(self):
        il = interlock()
        self.assertEqual(il(1, [20, 20], [0, 0], [float('nan'), 0], 0), [0, 20])
        self.assertEqual(il(1.1, [20, 20], [0, 0], [None, None], 0), [0, 20])
        self.assertEqual(il(1.29, [20, 20], [0, 0], [0, 0], 0), [0, 20])
        # Neither missing nor finite recovery samples extend the original hold.
        self.assertEqual(il(1.31, [20, 20], [0, 0], [None, None], 0), [20, 20])

    def test_repeated_invalid_force_renews_release_hold(self):
        il = interlock()
        il(1, [20, 20], [0, 0], [float('nan'), 0], 0)
        il(1.2, [20, 20], [0, 0], [-float('inf'), 0], 0)
        self.assertEqual(il(1.4, [20, 20], [0, 0], [0, 0], 0), [0, 20])
        self.assertEqual(il(1.51, [20, 20], [0, 0], [0, 0], 0), [20, 20])

    def test_fresh_missing_force_does_not_create_hold(self):
        il = interlock()
        self.assertEqual(il(0, [20, 20], [10, 10], [None, None], 0), [20, 20])
        self.assertEqual(il.flag, 0)

    def test_positive_infinity_preserves_saturation_arm_boundary(self):
        for mode in ('raw', 'model'):
            for angle in (0, 2, 2.001, 10):
                with self.subTest(mode=mode, angle=angle):
                    il = interlock(mode)
                    out = il(0, [20, 20], [angle, angle], [float('inf'), float('inf')], 0.2)
                    self.assertEqual(out, [0, 0] if angle > 2 else [20, 20])
                    self.assertEqual(il.flag, 1 if angle > 2 else 0)

    def test_finite_thresholds_and_inertial_compensation_unchanged(self):
        for force, expected, flag in ((-10, 20, 0), (0, 20, 0), (60, 20, 0),
                                      (61, 10, 0.5), (120, 10, 0.5), (121, 0, 1)):
            with self.subTest(force=force):
                il = interlock()
                self.assertEqual(il(0, [20, 20], [10, 10], [force, 0], 0), [expected, 20])
                self.assertEqual(il.flag, flag)
        il = interlock()
        self.assertEqual(il(0, [20, 20], [2, 2], [121, 121], 0), [20, 20])
        il = interlock('model')
        self.assertEqual(il(0, [20, 20], [10, 10], [0, 80], 0.2), [20, 20])
        self.assertAlmostEqual(il.force[1], 80 - 221.9 * 0.2)
        il = interlock('raw')
        self.assertEqual(il(0, [20, 20], [10, 10], [0, 80], 0.2), [20, 10])

    def test_nonfinite_result_after_inertial_subtraction_releases(self):
        # Both supplied forces are finite; the adjusted force must be checked too.
        il = interlock('model', console_N_per_g=1e308)
        self.assertEqual(il(0, [20, 20], [0, 0], [10, 10], 2), [20, 0])
        self.assertEqual(il.force[1], -float('inf'))
        self.assertEqual(il.flag, 1)

    def test_saturated_baseline_through_real_pressure_force_path_releases(self):
        cfg = C.load_config(str(RT / 'config.yaml'))
        cfg['_K'] = 0
        cfg['_fsr_curve'] = str(RT.parent / 'tools' / 'fsr_curve_20261026.json')
        cfg['rig'] = 'static_bench'
        cfg['pressure']['counts_per_N'] = {
            channel: 86.94837255456748
            for channel in cfg['pressure']['channels'].values() if isinstance(channel, str)
        }
        names = {channel for value in cfg['pressure']['channels'].values()
                 for channel in (value if isinstance(value, list) else [value])}
        values = dict.fromkeys(names, 1000.0)
        # Actual recorded A1 readings can exceed this unremapped curve's 1918.6 ceiling.
        values.update(A1_C2=2316.0, A1_C3=2063.0)

        class Sensor:
            def __init__(self):
                self.t = 0.0; self.pressure = {}; self.pressure_at = -1e9; self.fault = None
            def clock(self): return self.t
            def sleep(self, dt): self.t += dt
            def poll(self):
                self.pressure = dict(values); self.pressure_at = self.t

        sensor = Sensor()
        ctl = C.Controller(cfg, sensor, SimpleNamespace(OD=1, A=2),
                           SimpleNamespace(OBS_DIM=1, action_to_targets=True),
                           None, SRC.NoPose(), SRC.NoPose(), sensor.clock, log=lambda *args: None)
        self.assertEqual(ctl.zero_pressure(wait=sensor.sleep), values)
        pressure, valid = ctl._pressure(sensor.clock())
        self.assertTrue(valid)
        force = ctl._wing_force(pressure)
        self.assertTrue(math.isnan(force[0]))  # real FsrInverse: inf - inf
        self.assertEqual(force[1], 0.0)
        self.assertEqual(ctl.ilock(sensor.clock(), [20, 20], [0, 10], force, 0), [0, 20])
        self.assertEqual(ctl.ilock.flag, 1)


if __name__ == '__main__':
    unittest.main()
