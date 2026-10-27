"""RTMPose-m COCO17 image/SimCC math; no camera, inference or device access.

The preprocessing contract matches the existing RTMLib 0.0.16 bench pipeline:
crop first, 1.25 bbox padding, 192x256 affine image, BGR channels unchanged,
mean/std normalization, float32 NCHW. A Hailo artifact must establish parity
with this contract; this module does not claim an accelerator is available.

Features use full-frame pixels (+x image right, +y down). The pose consumer
applies the separately verified image-to-vehicle sign. A neutral reference
requires an explicit operator confirmation and matches frame/ROI/model.
"""
from dataclasses import dataclass
import math
import re

import numpy as np

INPUT_WH = (192, 256)
MEAN = np.array((123.675, 116.28, 103.53))
STD = np.array((58.395, 57.12, 57.375))
CORE = (5, 6, 11, 12)  # COCO shoulders and hips; other joints do not drive features.
NEUTRAL_SCHEMA = 'oasis.rtmpose.neutral.v1'


@dataclass(frozen=True)
class Transform:
    frame_hw: tuple
    roi_xyxy: tuple
    center: tuple  # full-frame pixels
    scale: tuple  # padded width/height, corrected to the model aspect ratio


@dataclass(frozen=True)
class BodyGeometry:
    frame_hw: tuple
    roi_xyxy: tuple
    shoulder_center_px: tuple
    hip_center_px: tuple
    shoulder_width_px: float
    torso_angle_deg: float
    pose_confidence: float


def image_transform(frame_hw, roi_xyxy=None):
    if len(frame_hw) != 2 or any(type(v) is not int or v < 2 for v in frame_hw):
        raise ValueError('frame_hw must contain positive integer height,width')
    height, width = frame_hw
    roi = (0, 0, width, height) if roi_xyxy is None else tuple(roi_xyxy)
    if len(roi) != 4 or any(type(v) is not int for v in roi):
        raise ValueError('ROI must contain four integer pixel bounds')
    x0, y0, x1, y1 = roi
    if not 0 <= x0 < x1 <= width or not 0 <= y0 < y1 <= height or min(x1-x0, y1-y0) < 2:
        raise ValueError('ROI must lie inside the image and have nonzero area')
    sw, sh = (x1-x0)*1.25, (y1-y0)*1.25
    aspect = INPUT_WH[0]/INPUT_WH[1]
    if sw > sh*aspect:
        sh = sw/aspect
    else:
        sw = sh*aspect
    return Transform(tuple(frame_hw), roi, ((x0+x1)/2., (y0+y1)/2.), (sw, sh))


def prepare_input(frame_bgr, roi_xyxy=None):
    """Return normalized float32 [1,3,256,192] and affine metadata.

    uint8 three-channel BGR is required. Monochrome camera conversion is the
    capture adapter's explicit responsibility. No channel reversal is hidden.
    """
    import cv2
    frame = np.asarray(frame_bgr)
    if frame.dtype != np.uint8 or frame.ndim != 3 or frame.shape[2] != 3:
        raise ValueError('frame must be uint8 HxWx3 BGR')
    t = image_transform(tuple(frame.shape[:2]), roi_xyxy)
    x0, y0, x1, y1 = t.roi_xyxy
    crop = frame[y0:y1, x0:x1]
    center = np.asarray(t.center)-[x0, y0]
    # The three points are the zero-rotation RTMLib affine construction.
    direction = np.array([0., -t.scale[0]*.5])
    src0, src1 = center, center+direction
    # RTMLib's third point rotates (src0-src1).
    src2 = src1+np.array([direction[1], -direction[0]])
    dst0 = np.array(INPUT_WH, dtype=float)*.5
    dst1 = dst0+[0., -INPUT_WH[0]*.5]
    delta = dst0-dst1
    dst2 = dst1+[-delta[1], delta[0]]
    matrix = cv2.getAffineTransform(np.float32([src0, src1, src2]),
                                    np.float32([dst0, dst1, dst2]))
    warped = cv2.warpAffine(crop, matrix, INPUT_WH, flags=cv2.INTER_LINEAR)
    normalized = (warped-MEAN)/STD
    tensor = np.ascontiguousarray(normalized.transpose(2, 0, 1)[None], dtype=np.float32)
    return tensor, t


def _check_transform(transform):
    if not isinstance(transform, Transform) or transform != image_transform(transform.frame_hw, transform.roi_xyxy):
        raise ValueError('invalid affine transform')


def decode_simcc(simcc_x, simcc_y, transform):
    """Decode exact batch-one COCO17 outputs into full-frame coordinates."""
    _check_transform(transform)
    x, y = np.asarray(simcc_x), np.asarray(simcc_y)
    if x.shape != (1, 17, 384) or y.shape != (1, 17, 512):
        raise ValueError('expected SimCC shapes (1,17,384) and (1,17,512)')
    if not np.issubdtype(x.dtype, np.number) or not np.issubdtype(y.dtype, np.number):
        raise ValueError('SimCC outputs must be numeric')
    if np.iscomplexobj(x) or np.iscomplexobj(y) or not np.isfinite(x).all() or not np.isfinite(y).all():
        raise ValueError('SimCC outputs must be finite real values')
    xy = np.stack((x.argmax(axis=2), y.argmax(axis=2)), axis=-1).astype(np.float32)[0]
    with np.errstate(over='ignore', invalid='ignore'):
        scores = ((x.max(axis=2)+y.max(axis=2))*.5)[0]
    if not np.isfinite(scores).all():
        raise ValueError('nonfinite SimCC confidence after decoding')
    xy[scores <= 0] = -1
    xy = xy/2./np.asarray(INPUT_WH)*np.asarray(transform.scale)
    xy += np.asarray(transform.center)-np.asarray(transform.scale)/2.
    return xy, scores


def extract_body_geometry(xy, scores, transform, min_confidence=.5):
    """Validate the four required joints; unseen legs/arms never become a body.

    Other joints are not used. Required points must be finite, inside both the
    image and selected ROI, have score >= threshold, and form a nondegenerate
    shoulder/hip geometry. SimCC responses are confidence proxies, not
    calibrated probabilities; their conservative minimum is clipped to [0,1].
    """
    _check_transform(transform)
    xy, scores = np.asarray(xy), np.asarray(scores)
    if xy.shape != (17, 2) or scores.shape != (17,):
        raise ValueError('expected COCO17 xy[17,2] and scores[17]')
    if isinstance(min_confidence, bool) or not math.isfinite(min_confidence) or not 0 < min_confidence <= 1:
        raise ValueError('min_confidence must be in (0,1]')
    if not np.issubdtype(xy.dtype, np.number) or not np.issubdtype(scores.dtype, np.number):
        raise ValueError('keypoints and scores must be numeric')
    if np.iscomplexobj(xy) or np.iscomplexobj(scores):
        raise ValueError('keypoints and scores must be real')
    points, confidence = xy[list(CORE)], scores[list(CORE)]
    if not np.isfinite(points).all() or not np.isfinite(confidence).all() or np.any(confidence < min_confidence):
        raise ValueError('required shoulder/hip landmarks missing or low confidence')
    x0, y0, x1, y1 = transform.roi_xyxy
    if np.any(points[:, 0] < x0) or np.any(points[:, 0] >= x1) or np.any(points[:, 1] < y0) or np.any(points[:, 1] >= y1):
        raise ValueError('required landmark outside selected image/ROI')
    shoulder, hip = points[:2].mean(axis=0), points[2:].mean(axis=0)
    width = float(np.linalg.norm(points[1]-points[0]))
    height = float(hip[1]-shoulder[1])
    # Tiny or upside-down bodies are not an observable seated torso.
    if width < 2 or height < 2:
        raise ValueError('degenerate shoulder width or torso height')
    angle = math.degrees(math.atan2(float(shoulder[0]-hip[0]), height))
    return BodyGeometry(transform.frame_hw, transform.roi_xyxy, tuple(map(float, shoulder)),
                        tuple(map(float, hip)), width, angle, float(np.clip(confidence.min(), 0., 1.)))


def _identity(geometry, transform, model_sha256):
    _check_transform(transform)
    if not isinstance(geometry, BodyGeometry):
        raise ValueError('validated body geometry required')
    if geometry.frame_hw != transform.frame_hw or geometry.roi_xyxy != transform.roi_xyxy:
        raise ValueError('geometry/frame identity mismatch')
    values = (*geometry.shoulder_center_px, *geometry.hip_center_px, geometry.shoulder_width_px,
              geometry.torso_angle_deg, geometry.pose_confidence)
    if len(geometry.shoulder_center_px) != 2 or len(geometry.hip_center_px) != 2 or not all(math.isfinite(v) for v in values) or geometry.shoulder_width_px < 2 or not 0 <= geometry.pose_confidence <= 1:
        raise ValueError('invalid body geometry')
    if not isinstance(model_sha256, str) or not re.fullmatch('[0-9a-f]{64}', model_sha256):
        raise ValueError('model_sha256 must be an exact lowercase SHA256')
    return {'frame_hw': list(transform.frame_hw), 'roi_xyxy': list(transform.roi_xyxy),
            'model_sha256': model_sha256}


def make_neutral(geometry, transform, model_sha256, *, operator_confirmed=False):
    """Serialize an explicitly accepted neutral. Does not establish one automatically."""
    if operator_confirmed is not True:
        raise ValueError('neutral requires explicit operator confirmation')
    identity = _identity(geometry, transform, model_sha256)
    return dict(schema=NEUTRAL_SCHEMA, operator_confirmed=True, **identity,
                shoulder_center_px=list(geometry.shoulder_center_px),
                hip_center_px=list(geometry.hip_center_px), shoulder_width_px=geometry.shoulder_width_px,
                torso_angle_deg=geometry.torso_angle_deg)


def relative_features(geometry, neutral, transform, model_sha256):
    """Return neutral-relative features or raise ValueError (caller marks unavailable).

    lean_score matches the established DAQ/RL contract: shoulder midpoint X
    minus neutral shoulder midpoint X, divided by NEUTRAL shoulder width.
    It is not a metric length or hip-relative translation. torso_lean_deg is
    the change of the pixel shoulder-to-hip angle from the accepted neutral.
    """
    identity = _identity(geometry, transform, model_sha256)
    if not isinstance(neutral, dict) or neutral.get('schema') != NEUTRAL_SCHEMA or neutral.get('operator_confirmed') is not True:
        raise ValueError('neutral reference unavailable or unconfirmed')
    if any(neutral.get(k) != v for k, v in identity.items()):
        raise ValueError('neutral model/frame/ROI mismatch; recalibration required')
    try:
        center = np.asarray(neutral['shoulder_center_px'], dtype=float)
        hip = np.asarray(neutral['hip_center_px'], dtype=float)
        width = float(neutral['shoulder_width_px'])
        angle = float(neutral['torso_angle_deg'])
    except (KeyError, ValueError, TypeError, OverflowError):
        raise ValueError('invalid neutral geometry') from None
    if center.shape != (2,) or hip.shape != (2,) or not np.isfinite(center).all() or not np.isfinite(hip).all() or not math.isfinite(width) or width < 2 or not math.isfinite(angle) or abs(angle) >= 90:
        raise ValueError('invalid neutral geometry')
    x0, y0, x1, y1 = transform.roi_xyxy
    if any(not x0 <= p[0] < x1 or not y0 <= p[1] < y1 for p in (center, hip)) or hip[1]-center[1] < 2 or width > math.hypot(x1-x0,y1-y0):
        raise ValueError('neutral geometry outside selected image/ROI')
    measured_angle = math.degrees(math.atan2(float(center[0]-hip[0]), float(hip[1]-center[1])))
    if abs(measured_angle-angle) > 1e-5:
        raise ValueError('neutral torso angle inconsistent with its landmarks')
    angle_delta = geometry.torso_angle_deg-angle
    lean = (geometry.shoulder_center_px[0]-float(center[0]))/width
    if not math.isfinite(angle_delta) or abs(angle_delta) > 90 or not math.isfinite(lean) or abs(lean) > 4:
        raise ValueError('pose change outside input contract')
    return dict(torso_lean_deg=angle_delta, lean_score=lean, pose_confidence=geometry.pose_confidence,
                vision_valid=True, neutral_valid=True)
