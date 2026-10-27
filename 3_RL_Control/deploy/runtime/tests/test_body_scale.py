"""v9 body scale: camera neutral ratio -> one frozen factor on the G base / preview angle. No device access."""
from pathlib import Path
import json
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from oasis_rt import body_scale as B
from oasis_rt.g_residual_control import GResidualGovernor

REF = B.REFERENCE


def neutral(width_ratio=1.0, torso_ratio=1.0, frame_hw=(800, 1280)):
    h, w = frame_hw
    sw = REF['shoulder_width_frac'] * width_ratio * w
    th = REF['torso_height_frac'] * torso_ratio * h
    return dict(schema=B.NEUTRAL_SCHEMA, operator_confirmed=True, frame_hw=[h, w], roi_xyxy=[0, 0, w, h],
                model_sha256='0' * 64, shoulder_center_px=[w * .3, h * .3], hip_center_px=[w * .32, h * .3 + th],
                shoulder_width_px=sw, torso_angle_deg=0.0)


class BodyScaleTests(unittest.TestCase):
    def test_reference_user_keeps_exactly_one(self):
        for hw in ((800, 1280), (480, 640), (400, 640)):            # pixel fractions: resolution independent
            rw, rt = B.ratios(neutral(frame_hw=hw))
            self.assertAlmostEqual(rw, 1.0, places=9); self.assertAlmostEqual(rt, 1.0, places=9)
            self.assertAlmostEqual(B.scale_from_ratios(rw, rt), 1.0, places=9)

    def test_scale_is_small_bounded_and_follows_the_tuned_law(self):
        for rw in (0.8, 0.9, 1.0, 1.1, 1.2):
            for rt in (0.85, 1.0, 1.15):
                s = B.scale_from_ratios(rw, rt)
                raw = 1 + B.K_W * (rw - 1) + B.K_T * (rt - 1)
                self.assertAlmostEqual(s, min(max(raw, B.LAW_MIN), B.LAW_MAX))
                self.assertTrue(B.SCALE_MIN <= B.LAW_MIN <= s <= B.LAW_MAX <= B.SCALE_MAX)

    def test_governor_scale_one_is_the_reviewed_controller(self):
        a, b = GResidualGovernor(), GResidualGovernor(body_scale=1.0)
        for i in range(400):
            g = 1.2 * ((i * 37) % 200 - 100) / 100; p = 1.2 * ((i * 53) % 200 - 100) / 100
            raw = ((i * 7) % 21, (i * 11) % 21); prior = ((i * 3) % 21, (i * 5) % 21)
            self.assertEqual(a.step(g, p, 10., raw, prior)[0], b.step(g, p, 10., raw, prior)[0])

    def test_governor_scales_base_and_preview_but_keeps_caps_and_residual(self):
        for k in (0.9, 1.1):
            gov = GResidualGovernor(body_scale=k)
            out, snap = gov.step(0.5, 0.5, 10., (0., 15.), (0., 15.))
            ref = GResidualGovernor().base_yaw(0.5)
            self.assertAlmostEqual(snap['base_yaw_deg'], min(k * ref, 20.)); self.assertEqual(snap['body_scale'], k)
            out, snap = GResidualGovernor(body_scale=k).step(1.0, 1.0, 10., (0., 0.), (0., 20.))
            self.assertAlmostEqual(out[1], min(20. * k, 20.))        # 1 g endpoint scaled, never above 20
            pv = GResidualGovernor(body_scale=k)                       # preview alone stays <= 10 deg
            for _ in range(4): out, snap = pv.step(0.0, 0.5, 10., (0., 20.), (0., 0.))
            self.assertLessEqual(out[1], 10.)
            gov = GResidualGovernor(body_scale=k)
            _, snap = gov.step(0.3, 0.3, 10., (0., 19.), (0., 9.))     # +10 deg actor delta -> residual still 1.5 max
            self.assertLessEqual(abs(snap['residual_applied_deg']), 1.5 + 1e-9)

    def test_invalid_or_foreign_neutral_refuses(self):
        bad = neutral(); bad['operator_confirmed'] = False
        with self.assertRaises(ValueError): B.ratios(bad)
        with self.assertRaises(ValueError): B.ratios(neutral(width_ratio=1.6))   # other camera / seat composition
        with self.assertRaises(ValueError): GResidualGovernor(body_scale=1.5)
        with self.assertRaises(ValueError): GResidualGovernor(body_scale=float('nan'))

    def test_session_scale_from_config(self):
        self.assertEqual(B.session_scale({})[0], 1.0)
        self.assertEqual(B.session_scale({'body_scale': {'enabled': False, 'neutral': 'x'}})[0], 1.0)
        with self.assertRaises(ValueError): B.session_scale({'body_scale': {'enabled': True}})
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / 'neutral.json'; p.write_text(json.dumps(neutral(width_ratio=1.1)), encoding='utf-8')
            s, rec = B.session_scale({'body_scale': {'enabled': True, 'neutral': str(p)}})
            self.assertAlmostEqual(rec['width_ratio'], 1.1); self.assertAlmostEqual(s, B.scale_from_ratios(1.1, 1.0))


if __name__ == '__main__':
    unittest.main()
