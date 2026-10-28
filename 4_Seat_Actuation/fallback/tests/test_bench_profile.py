import json
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from build_bench_profile import pava


class FitTests(unittest.TestCase):
    def test_pools_violations_with_sample_weights(self):
        self.assertEqual(pava([3,2,4]),[2.5,2.5,4])
        self.assertEqual(pava([3,2,4],[3,1,2]),[2.75,2.75,4])
        self.assertEqual(pava([1,2,0],increasing=False),[1.5,1.5,0])

    def test_rejects_invalid_fit_inputs(self):
        for values,weights in [([],None),([1],[0]),([float('nan')],None),([1,2],[1])]:
            with self.assertRaises(ValueError):pava(values,weights)


class ProfileTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.profile=json.loads((ROOT/'profiles'/'bench_profile.json').read_text(encoding='utf-8'))

    def test_only_measured_complete_independent_branches(self):
        p=self.profile
        self.assertEqual(p['selected_unique_points'],55)
        self.assertTrue(p['quality_all_points_ok'])
        self.assertIsNone(p['physical_channel_mapping'])
        self.assertFalse(p['force_calibrated'])
        self.assertFalse(p['actual_position_feedback'])
        for side,branch in p['branches'].items():
            self.assertEqual(branch['yaw_knots_deg'],list(range(28)))
            for i,point in enumerate(branch['points']):
                self.assertEqual(point['yaw_deg'],i)
                other='right' if side=='left' else 'left'
                self.assertEqual(point['command_deg'][other+'_yaw_deg'],0)
                self.assertEqual(point['command_deg']['left_pitch_deg'],0)
                self.assertEqual(point['command_deg']['right_pitch_deg'],0)
                self.assertTrue(point['motion_check']['passed'])
                self.assertGreater(point['motion_check']['state_samples'],0)
                self.assertEqual(point['interval']['duration_s'],2)
                for sensor in point['channels'].values():
                    self.assertGreaterEqual(sensor['samples'],180)
                    self.assertEqual(sensor['adc_errors'],0)
                    self.assertLessEqual(sensor['min_v'],sensor['p05_v'])
                    self.assertLessEqual(sensor['p05_v'],sensor['p50_v'])
                    self.assertLessEqual(sensor['p50_v'],sensor['p95_v'])
                    self.assertLessEqual(sensor['p95_v'],sensor['max_v'])

    def test_fit_monotonicity_and_limits_are_not_safety_claims(self):
        for branch in self.profile['branches'].values():
            self.assertTrue(branch['no_extrapolation'])
            for fit in branch['per_channel_monotonic_fits'].values():
                inc,dec=fit['increasing_mean_v'],fit['decreasing_mean_v']
                self.assertTrue(all(a<=b+1e-12 for a,b in zip(inc,inc[1:])))
                self.assertTrue(all(a>=b-1e-12 for a,b in zip(dec,dec[1:])))
                self.assertTrue(fit['candidate_is_not_human_pressure_limit'])
                self.assertAlmostEqual(fit['candidate_twenty_percent_positive_delta_v'],
                    .2*fit['positive_p95_span_from_baseline_v'])


if __name__=='__main__':unittest.main()
