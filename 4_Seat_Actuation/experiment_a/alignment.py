"""Closest pad-side face orientation in an assumed common local frame.

R8 uses Rz(-yaw)*Rx(pitch). Positive local torso tilt is outward at the
upper contact region, so conforming Pitch is positive. The caller converts
right-positive body roll to outward-positive roll for the selected bolster.

The scalar torso prediction is treated as a backrest-local tilt proxy here.
The installed CAD-to-seat transform has not been measured. This function is
not a 3D observation or a guarantee of physical pad-to-body face alignment.
"""
import math


def _angle(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(name + ' must be a finite angle')
    if not -89.0 <= value <= 89.0:
        raise ValueError(name + ' must be within [-89, 89] degrees')
    return math.radians(value)


def closest_face_pitch_deg(yaw_deg: float, torso_roll_deg: float) -> float:
    """Return the signed Pitch maximizing the local face-normal dot product.

    At fixed yaw psi, pad normal n=(sin(psi)*cos(beta),
    cos(psi)*cos(beta), sin(beta)); target n_t=(0,cos(theta),sin(theta)).
    The maximum of n dot n_t occurs at beta=atan2(sin(theta),
    cos(psi)*cos(theta)). This also aligns the upper-axis Y/Z projection.
    Full 3D face equality is generally impossible at nonzero yaw.

    Mechanical Pitch limits and combined reach limits belong to the caller;
    the returned ideal is not silently clipped to a particular mechanism.
    Input angles are limited to a nondegenerate frontal hemisphere.
    """
    yaw = _angle(yaw_deg, 'yaw_deg')
    theta = _angle(torso_roll_deg, 'torso_roll_deg')
    return math.degrees(math.atan2(math.sin(theta), math.cos(yaw)*math.cos(theta)))


def alignment_error_deg(yaw_deg: float, pitch_deg: float, torso_roll_deg: float) -> float:
    """Unsigned angle between assumed pad and target side-face unit normals.

    This is a model geometric residual, not a measured contact angle. Nonzero
    yaw leaves a residual even at the optimal Pitch because yaw also sets
    closure. A reach-clipped Pitch may increase this residual further.
    """
    yaw = _angle(yaw_deg, 'yaw_deg')
    pitch = _angle(pitch_deg, 'pitch_deg')
    theta = _angle(torso_roll_deg, 'torso_roll_deg')
    dot = math.cos(yaw)*math.cos(pitch)*math.cos(theta) + math.sin(pitch)*math.sin(theta)
    return math.degrees(math.acos(max(-1.0, min(1.0, dot))))
