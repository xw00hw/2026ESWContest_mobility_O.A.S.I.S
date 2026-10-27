"""v9 body scale: camera neutral of the seated occupant vs the reference user -> one frozen G-curve factor.

The G base curve + bounded RL residual (g_residual_control) was tuned on the seated bench with one person, the
reference user (10/28 final session, screencast sha256 93f7a0b2...). Another occupant gets the same controller,
only its base / preview angles multiplied by a small factor from the body ratio the camera sees:

  r_w = (shoulder_width_px / frame_width)  / REFERENCE shoulder_width_frac      shoulder joint span
  r_t = ((hip_y - shoulder_y) / frame_height) / REFERENCE torso_height_frac     shoulder-to-hip height
  s   = clip(1 + K_W (r_w - 1) + K_T (r_t - 1), LAW_MIN, LAW_MAX)               s = 1 for the reference user
                                                                  (K_W -1.2, K_T 0, clip 0.9 .. 1.1)

Pixel fractions of the full frame are used so the ratio does not depend on the capture resolution. They assume
the reviewed camera mode, lens, mount and seat position (the 10/28 fixed composition): a moved camera or seat
changes r without a body change. The ratio is taken from the operator-confirmed neutral JSON of the seated
person (oasis.rtmpose.neutral.v1, the same file pose_camera --neutral uses), once per session, never from live
frames. K_W, K_T and the clip come from sim/v3/tune_body_scale.py (results/v9/body_scale.json).
"""
import json
import math

NEUTRAL_SCHEMA = 'oasis.rtmpose.neutral.v1'
SCALE_MIN, SCALE_MAX = 0.8, 1.2            # hard bound of any factor the governor accepts
# reference user, measured from the RTMPose overlay of the 10/28 final-session screencast (results/v9)
REFERENCE = dict(shoulder_width_frac=0.2665, torso_height_frac=0.424,      # 294 frames, cv 2.8 % / 3.6 %
                 source='OASIS 2026-10-28 03:31 final seated session, screencast sha256 93f7a0b265ca0f36...')
# ratio -> scale law (sim/v3/tune_body_scale.py, results/v9/body_scale*.json): the support-optimal angle scale of the
# simulated occupant falls with shoulder width (narrow: clearance to close, wide: preload); the reference body's
# optimum is s = 1.0 (0.997). Sitting height: the tuning and the held-out episodes disagree on the sign -> K_T = 0
# (the torso ratio is still computed and logged). Clipped to +-10 %: a slight correction only.
K_W, K_T = -1.2, 0.0
LAW_MIN, LAW_MAX = 0.9, 1.1
RATIO_RANGE = (0.7, 1.4)                   # outside: not the same camera/seat composition, refuse


def _num(name, v):
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
        raise ValueError(f'{name} must be a finite number')
    return float(v)


def neutral_fractions(neutral):
    """(shoulder width / frame width, shoulder-to-hip height / frame height) of a confirmed neutral."""
    if not isinstance(neutral, dict) or neutral.get('schema') != NEUTRAL_SCHEMA or neutral.get('operator_confirmed') is not True:
        raise ValueError('body scale needs an operator-confirmed oasis.rtmpose.neutral.v1')
    try:
        h, w = (int(v) for v in neutral['frame_hw'])
        sx, sy = (_num('shoulder_center_px', v) for v in neutral['shoulder_center_px'])
        hx, hy = (_num('hip_center_px', v) for v in neutral['hip_center_px'])
        width = _num('shoulder_width_px', neutral['shoulder_width_px'])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f'invalid neutral geometry: {exc}') from None
    if w < 2 or h < 2 or width < 2 or hy - sy < 2:
        raise ValueError('degenerate neutral geometry')
    return width / w, (hy - sy) / h


def ratios(neutral, reference=None):
    ref = dict(REFERENCE, **(reference or {}))
    fw, ft = neutral_fractions(neutral)
    rw, rt = fw / _num('reference shoulder_width_frac', ref['shoulder_width_frac']), \
        ft / _num('reference torso_height_frac', ref['torso_height_frac'])
    lo, hi = RATIO_RANGE
    if not (lo <= rw <= hi and lo <= rt <= hi):
        raise ValueError(f'body ratio outside {RATIO_RANGE} (width {rw:.3f}, torso {rt:.3f}): camera/seat changed?')
    return rw, rt


def scale_from_ratios(rw, rt, k_w=None, k_t=None, lo=None, hi=None):
    k_w = K_W if k_w is None else k_w; k_t = K_T if k_t is None else k_t
    lo = LAW_MIN if lo is None else lo; hi = LAW_MAX if hi is None else hi
    s = 1.0 + k_w * (_num('rw', rw) - 1.0) + k_t * (_num('rt', rt) - 1.0)
    return float(min(max(s, lo, SCALE_MIN), hi, SCALE_MAX))


def session_scale(cfg):
    """(scale, record) from config `body_scale`; disabled -> exactly 1.0. Invalid input when enabled -> ValueError."""
    bs = dict(cfg.get('body_scale') or {})
    enabled = bs.get('enabled', False)
    if type(enabled) is not bool:
        raise ValueError('body_scale.enabled must be boolean')
    if not enabled:
        return 1.0, dict(enabled=False, scale=1.0)
    path = bs.get('neutral')
    if not isinstance(path, str) or not path:
        raise ValueError('body_scale.neutral must name the seated person\'s confirmed neutral JSON')
    try:
        with open(path, encoding='utf-8') as f:
            neutral = json.load(f)
    except (OSError, ValueError) as exc:
        raise ValueError(f'body_scale.neutral unreadable: {exc}') from None
    rw, rt = ratios(neutral, bs.get('reference'))
    s = scale_from_ratios(rw, rt)
    return s, dict(enabled=True, scale=s, width_ratio=rw, torso_ratio=rt, neutral=path,
                   law=dict(k_w=K_W, k_t=K_T, clip=[LAW_MIN, LAW_MAX]), reference=dict(REFERENCE, **(bs.get('reference') or {})))
