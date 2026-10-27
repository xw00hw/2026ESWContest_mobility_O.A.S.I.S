"""Observation contract v3 (bolster yaw + PITCH, two pad zones per side, body-shape cues).
Single source of truth for sim/v3 and the Pi runtime (copied into the model dir, sha-checked).

  index  name                                source on the Pi                         unit in -> scale
  0-9    ay_pred[0.1..1.0 s]                 predictor                                g
  10-19  ax_pred[0.1..1.0 s]                 predictor                                g
  20-24  ay, ax, yaw_rate, speed, steering   IMU (+y left) / OBD                      g, g, rad/s, m/s /20, rad /3
  25-32  side pad deltas vs session start    mat: lo_R lo_L hi_R hi_L                 counts / 1000
         back_R back_L cush_R cush_L
  33-36  bilateral R-L: side_lo side_hi back cush                                     counts / 1000
  37-38  torso_lean, shoulder_shift          frontal camera vs neutral                deg / 5, cm / 5
  39-42  reported yaw L, R; pitch L, R       STM32 #STATE                             deg / theta_max, deg / pitch_max
  43-46  previous action (c, d, cp, dp)      policy                                   [-1, 1]
  47-49  body build: mass, shoulder width, sitting torso    scale/pressure + calibrated camera   kg/75-1, m/0.42-1, m/0.58-1
  50-53  rest pressure of the side pads (session-start baseline, ABSOLUTE): lo_R lo_L hi_R hi_L
                                             -> preload / torso taper (body-shape cue)  (counts - 1000) / 1000
  54     pose_age                            now - capture time of last pose          s (clip 1)
  55     pressure_valid                      all four side channels alive             0/1
  56     safety_flag                         1 = trip (release), 0.5 = hold            0/0.5/1
"""
import numpy as np

OBS_DIM = 57
VERSION = 'oasis-obs-v3.0'
ACTION_DIM = 4                  # c, d (yaw common / differential), cp, dp (pitch common / differential)
PITCH_MAX_DEG = 8.0


def build_obs(d, theta_max, pitch_max=PITCH_MAX_DEG):
    o = np.zeros(OBS_DIM, np.float32)
    o[0:10] = d['ay_pred_g']; o[10:20] = d['ax_pred_g']
    o[20:25] = [d['ay_g'], d['ax_g'], d['yaw_rate_radps'], d['speed_mps'] / 20, d['steer_rad'] / 3]
    sl = [d['side_lo_R_counts'], d['side_lo_L_counts'], d['side_hi_R_counts'], d['side_hi_L_counts']]
    bk = [d['back_R_counts'], d['back_L_counts']]; cu = [d['cush_R_counts'], d['cush_L_counts']]
    o[25:33] = np.array(sl + bk + cu) / 1000
    o[33:37] = np.array([sl[0] - sl[1], sl[2] - sl[3], bk[0] - bk[1], cu[0] - cu[1]]) / 1000
    o[37:39] = [d['torso_lean_deg'] / 5.0, d['shoulder_shift_cm'] / 5.0]
    o[39:43] = [d['yaw_reported_L_deg'] / theta_max, d['yaw_reported_R_deg'] / theta_max,
                d['pitch_reported_L_deg'] / pitch_max, d['pitch_reported_R_deg'] / pitch_max]
    pa = np.zeros(4, np.float32); pa[:len(d['prev_action'])] = d['prev_action']; o[43:47] = pa
    o[47:50] = [d['mass_kg'] / 75 - 1, d['shoulder_width_m'] / 0.42 - 1, d['sitting_torso_m'] / 0.58 - 1]
    o[50:54] = (np.asarray(d['side_rest_counts'], float) - 1000) / 1000
    o[54:57] = [min(d['pose_age_s'], 1.0), float(d['pressure_valid']), d['safety_flag']]
    return o


def action_to_targets(a, theta_max, pitch_max=PITCH_MAX_DEG):
    """[c, d(, cp, dp)] in [-1, 1] -> (yaw_L, yaw_R, pitch_L, pitch_R) in deg, same in sim and on the Pi.
    yaw as v2: c' = (c+1)/2 * th/2, d' = d * th/2, yaw_L = c'+d', yaw_R = c'-d' in [0, th].
    pitch: pitch_L = pmax * clip(cp + dp), pitch_R = pmax * clip(cp - dp); +pitch = lower part inward
    (C4_R8: R = Rz(-yaw) Rx(pitch), crank 1 = upper)."""
    a = np.clip(np.asarray(a, float), -1, 1)
    c = (a[0] + 1) / 2 * theta_max / 2; dd = a[1] * theta_max / 2
    yL, yR = np.clip([c + dd, c - dd], 0, theta_max)
    if len(a) < 4: return float(yL), float(yR), 0.0, 0.0
    pL, pR = pitch_max * np.clip([a[2] + a[3], a[2] - a[3]], -1, 1)
    return float(yL), float(yR), float(pL), float(pR)
