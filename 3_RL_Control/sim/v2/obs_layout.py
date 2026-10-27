"""THE observation contract of the bolster policy (v2). Single source of truth, imported by
the simulator (env_v2.BolsterEnvV2._obs) AND by the Raspberry Pi runtime
(deploy/runtime/oasis_rt). Never re-implement it elsewhere (review doc 4-C).

All inputs are physical units as measured on the rig; scaling happens only here.

  index  name                          source on the Pi                     unit in -> scale
  0-9    ay_pred[0.1..1.0 s]           predictor                            g
  10-19  ax_pred[0.1..1.0 s]           predictor                            g
  20     ay_now                        IMU, vehicle frame (+y left)         g
  21     ax_now                        IMU, vehicle frame (+x forward)      g
  22     yaw_rate                      IMU gyro z (+ left turn)             rad/s
  23     speed                         OBD/GPS                              m/s / 20
  24     steering_wheel                OBD/CAN                              rad / 3
  25-30  pressure deltas               mat vs session-start baseline        counts / 1000
         side_R side_L back_R back_L cush_R cush_L
  31-33  bilateral R-L                 side, back, cushion                  counts / 1000
  34     torso_lean                    frontal camera, vs neutral           deg / 5
  35     shoulder_shift                frontal camera, vs neutral           cm / 5
  36-37  yaw_reported L, R             STM32 #STATE pose                    deg / theta_max
  38-39  previous action (c, d)        policy                               [-1, 1]
  40-42  profile mass, shoulder w, sitting torso   calibration / user entry kg/75-1, m/0.42-1, m/0.58-1
  43     pose_age                      now - capture time of last pose      s (clip 1)
  44     pressure_valid                both side channels alive             0/1
  45     safety_flag                   1 = trip (release), 0.5 = hold       0/0.5/1
"""
import numpy as np

OBS_DIM = 46
VERSION = 'oasis-obs-v2.0'


def build_obs(d, theta_max):
    """d: dict with the fields listed above (physical units). Returns float32[46]."""
    o = np.zeros(OBS_DIM, np.float32)
    o[0:10] = d['ay_pred_g']
    o[10:20] = d['ax_pred_g']
    o[20:25] = [d['ay_g'], d['ax_g'], d['yaw_rate_radps'], d['speed_mps'] / 20, d['steer_rad'] / 3]
    sR, sL = d['side_R_counts'] / 1000, d['side_L_counts'] / 1000
    bR, bL = d['back_R_counts'] / 1000, d['back_L_counts'] / 1000
    cR, cL = d['cush_R_counts'] / 1000, d['cush_L_counts'] / 1000
    o[25:31] = [sR, sL, bR, bL, cR, cL]
    o[31:34] = [sR - sL, bR - bL, cR - cL]
    o[34:36] = [d['torso_lean_deg'] / 5.0, d['shoulder_shift_cm'] / 5.0]
    o[36:38] = [d['yaw_reported_L_deg'] / theta_max, d['yaw_reported_R_deg'] / theta_max]
    o[38:40] = d['prev_action']
    o[40:43] = [d['mass_kg'] / 75 - 1, d['shoulder_width_m'] / 0.42 - 1, d['sitting_torso_m'] / 0.58 - 1]
    o[43:46] = [min(d['pose_age_s'], 1.0), float(d['pressure_valid']), d['safety_flag']]
    return o
