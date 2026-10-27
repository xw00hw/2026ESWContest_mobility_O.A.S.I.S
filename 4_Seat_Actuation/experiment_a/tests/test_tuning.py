"""STM committed configuration boundary tests; no board or debugger opens."""
from copy import deepcopy
import unittest
from experiment_a.settings import Settings
from experiment_a.tuning import apply_firmware_tuning,tuning_display_rows

SNAPSHOT=dict(revision=0,yaw_max_mdeg=15000,pitch_max_mdeg=6000,
    yaw_rate_mdeg_s=2000,pitch_rate_mdeg_s=2000,
    full_scale_ay_mmps2=2942,pitch_scale_permille=1000)


class FirmwareTuningTests(unittest.TestCase):
    def test_snapshot_overrides_six_parameters_without_mutating_local_draft(self):
        base=Settings();before=deepcopy(base.to_dict())
        raw=dict(SNAPSHOT,revision=23,yaw_max_mdeg=12000,pitch_max_mdeg=4000,
                 yaw_rate_mdeg_s=1500,pitch_rate_mdeg_s=1200,full_scale_ay_mmps2=4000,pitch_scale_permille=250)
        candidate,frozen=apply_firmware_tuning(base,raw)
        self.assertEqual((candidate.max_yaw_deg,candidate.max_pitch_deg,candidate.max_yaw_rate_deg_s,
            candidate.max_pitch_rate_deg_s,candidate.full_scale_ay_mps2,candidate.pitch_per_predicted_roll),
            (12.,4.,1.5,1.2,4.,.25))
        self.assertEqual(base.to_dict(),before)
        self.assertEqual(candidate.confirmations,base.confirmations)
        raw['revision']=24
        self.assertEqual(frozen['revision'],23)
        self.assertEqual(candidate.metadata['firmware_tuning_revision'],23)

    def test_bad_keys_integer_types_revision_and_geometry_are_rejected(self):
        examples=[None,{},dict(SNAPSHOT,pressure=1),dict(SNAPSHOT,revision=True),
                  dict(SNAPSHOT,revision=-1),dict(SNAPSHOT,revision=0x100000000),
                  dict(SNAPSHOT,yaw_max_mdeg=22001),dict(SNAPSHOT,pitch_max_mdeg=6001),
                  dict(SNAPSHOT,yaw_rate_mdeg_s=-1),dict(SNAPSHOT,pitch_scale_permille=1001),
                  dict(SNAPSHOT,full_scale_ay_mmps2=499),dict(SNAPSHOT,yaw_max_mdeg=15000.0)]
        for snapshot in examples:
            with self.subTest(snapshot=snapshot),self.assertRaises(ValueError):
                apply_firmware_tuning(Settings(),snapshot)

    def test_absent_snapshot_is_not_displayed_as_default_values(self):
        self.assertEqual([row[-1] for row in tuning_display_rows(None)],['—']*6)


if __name__=='__main__':unittest.main()
