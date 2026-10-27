"""Direct wheel priority, dropout recovery, and unchanged measured-input fallback."""
from pathlib import Path
import sys, unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from oasis_rt.predictor import BeamNGPredictor, MozaPriorityPredictor, WheelPreviewPredictor

def sample(**kw):
    s=dict(t=1., ay_mps2=0., ax_mps2=.2, speed_mps=15., steer_rad=0., steer_input_rad=0.,
           yaw_rate_radps=0., _moza_valid=False)
    s.update(kw)
    return s

class MozaPredictorTests(unittest.TestCase):
    def test_direct_wheel_leads_zero_measured_steer_in_synthetic_case(self):
        p=MozaPriorityPredictor(BeamNGPredictor())
        s=sample(_moza_valid=True,_moza_query_ns=1_000_000_000,_moza_stream_id='one',steer_input_rad=.3)
        ay,ax=p(s)
        self.assertTrue(np.all(ay>0)); self.assertTrue(np.all(np.isfinite(ay)))
        np.testing.assert_allclose(ax,.2/9.81)
        self.assertEqual(s['steer_rad'],0.)
        self.assertEqual(p.selected,'moza_wheel')

    def test_absent_invalid_or_unconfirmed_wheel_uses_original_predictor(self):
        for invalid in ({},dict(_moza_valid=False,steer_input_rad=2.),
                        dict(_moza_valid=True,_moza_query_ns=1,_moza_stream_id='s',steer_input_rad=float('nan'))):
            p=MozaPriorityPredictor(BeamNGPredictor())
            s=sample(steer_rad=.1); s.update(invalid)
            expected=BeamNGPredictor()(dict(s,steer_input_rad=s['steer_rad']))
            actual=p(s)
            np.testing.assert_allclose(actual,expected)
            self.assertEqual(p.selected,'beamng_predictor')

    def test_dropout_keeps_measured_fallback_history_warm(self):
        p=MozaPriorityPredictor(BeamNGPredictor())
        ref=BeamNGPredictor()
        for i in range(12):
            s=sample(t=i*.05,ay_mps2=i*.1,steer_rad=.02*i,steer_input_rad=-.4,
                     _moza_valid=i<10,_moza_query_ns=1_000_000_000+i*50_000_000,_moza_stream_id='s')
            expected=ref(dict(s,steer_input_rad=s['steer_rad']))
            got=p(s)
            if i>=10:np.testing.assert_allclose(got,expected)
        np.testing.assert_allclose(p.fallback.hist,ref.hist)
        self.assertEqual(p.counts,dict(moza_wheel=10,beamng_predictor=2))

    def test_reconnect_resets_winding_rate(self):
        p=MozaPriorityPredictor(WheelPreviewPredictor(steer_tau_s=.3))
        p(sample(_moza_valid=True,_moza_query_ns=1_000_000_000,_moza_stream_id='a',steer_input_rad=.1))
        p(sample(_moza_valid=True,_moza_query_ns=1_050_000_000,_moza_stream_id='a',steer_input_rad=.2))
        self.assertGreater(p.wheel.rate,0)
        p(sample())
        p(sample(_moza_valid=True,_moza_query_ns=2_000_000_000,_moza_stream_id='b',steer_input_rad=-.2))
        self.assertEqual(p.wheel.rate,0)

    def test_stationary_vehicle_is_not_given_acceleration_from_wheel(self):
        p=MozaPriorityPredictor(BeamNGPredictor())
        ay,_=p(sample(speed_mps=0.,_moza_valid=True,_moza_query_ns=1,_moza_stream_id='s',steer_input_rad=2.))
        np.testing.assert_allclose(ay,0.)

if __name__=='__main__':unittest.main()
