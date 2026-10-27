"""Experiment A vehicle adapter: shared quality contract plus optional fields."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from experiment_a.vehicle import VehicleValidator


def row(sequence=1, **changes):
    value = dict(host_monotonic_ns=1_000_000_000+sequence*20_000_000,
        sim_time_s=sequence*.02, stream_id="fixture", vehicle_id=1, sequence_id=sequence,
        source_kind="beamng", linear_acc_y_mps2=1., linear_acc_x_mps2=-.2,
        speed_mps=10., steering_angle_deg=10., throttle_input=.2, brake_input=0.,
        derivative_valid=1, derivative_reason="ok", config_match=1, sample_dt_s=.02)
    value.update(changes)
    return value


class VehicleTests(unittest.TestCase):
    def validator(self):
        value = VehicleValidator()
        first = row()
        self.assertEqual(value.parse(first, first["host_monotonic_ns"]).reason, "first_sample")
        return value

    def test_optional_specific_force_retains_units_without_acceleration_substitution(self):
        validator = self.validator()
        current = row(2, specific_force_x_mps2=-.3, specific_force_y_mps2=1.2,
                      specific_force_z_mps2=9.81, roll_rad=.1, pitch_rad=-.1)
        frame = validator.parse(current, current["host_monotonic_ns"])
        self.assertTrue(frame.valid)
        self.assertEqual((frame.ax_mps2, frame.ay_mps2), (-.2, 1.))
        self.assertEqual((frame.specific_force_x_mps2, frame.specific_force_y_mps2), (-.3, 1.2))
        self.assertEqual(frame.specific_force_z_mps2, 9.81)
        self.assertEqual((frame.roll_rad, frame.pitch_rad), (.1, -.1))

    def test_absent_optional_fields_remain_none(self):
        validator = self.validator()
        current = row(2)
        frame = validator.parse(current, current["host_monotonic_ns"])
        self.assertTrue(frame.valid)
        self.assertIsNone(frame.specific_force_y_mps2)
        self.assertIsNone(frame.roll_rad)
        self.assertIsNone(frame.gear_index)

    def test_gear_index_is_retained_without_imagining_a_forward_gear(self):
        for gear in (None, -1, 0, 1, 7):
            validator = self.validator()
            current = row(2, gear_index=gear)
            frame = validator.parse(current, current['host_monotonic_ns'])
            self.assertTrue(frame.valid)
            self.assertEqual(frame.gear_index, gear)

    def test_nonfinite_optional_field_is_invalid(self):
        for name in ("specific_force_x_mps2", "specific_force_y_mps2", "specific_force_z_mps2",
                     "roll_rad", "pitch_rad", "gear_index"):
            with self.subTest(field=name):
                validator = self.validator()
                current = row(2, **{name: "nan"})
                frame = validator.parse(current, current["host_monotonic_ns"])
                self.assertFalse(frame.valid)
                self.assertEqual(frame.reason, "invalid_number:"+name)

    def test_stream_switch_duplicate_clock_and_quality_still_rejected(self):
        cases = [({"stream_id": "new-session"}, "stream_changed"),
                 ({"sequence_id": 1}, "duplicate_or_reordered"),
                 ({"host_monotonic_ns": 1_000_000_000}, "nonincreasing_host_time"),
                 ({"sim_time_s": .001}, "nonincreasing_sim_time"),
                 ({"derivative_valid": 0}, "invalid_derivative"),
                 ({"linear_acc_y_mps2": 201.}, "lateral_acceleration_outlier")]
        for changes, reason in cases:
            with self.subTest(reason=reason):
                validator = self.validator()
                current = row(2, **changes)
                frame = validator.parse(current, 1_040_000_000)
                self.assertFalse(frame.valid)
                self.assertEqual(frame.reason, reason)

    def test_high_acceleration_is_preserved_for_separate_impact_detection(self):
        for ax, ay in ((7.251468709664833, 12.871359949150792), (0., 30.), (-30., 0.), (200., 0.)):
            validator = self.validator()
            current = row(2, linear_acc_x_mps2=ax, linear_acc_y_mps2=ay)
            frame = validator.parse(current, current['host_monotonic_ns'])
            self.assertTrue(frame.valid)
            self.assertEqual((frame.ax_mps2, frame.ay_mps2), (ax, ay))


if __name__ == "__main__": unittest.main()
