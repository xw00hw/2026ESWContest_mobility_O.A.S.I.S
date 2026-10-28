"""C4_R8 geometry matching the existing STM32 joint_math.c command model.

All public angles are degrees. The firmware pose order is (left yaw, left
pitch, right yaw, right pitch); PWM order is (UR/D3, UL/D4, DR/D5, DL/D6).
Both local positive yaw commands close toward the body by the user's reported
installation convention. Positive local pitch reduces upper link-point Y and
increases lower link-point Y. Those link-point projections are geometry, not
measured pad contact displacement, body angles, force, or mechanical clearance.

Source: Universal_Joint_Control_R8/02_Control_Python/kinematics.py and
stm32/Core/Src/joint_math.c. Exact nonlinear neutral-branch inverse kinematics;
the corrected 270-degree / 2000-us gain and the fallback horn centers
are used. The legacy acquisition image and historical data keep their own zeros.
No serial, pressure data, camera, or runtime calibration file is accessed here.
"""
from __future__ import annotations

import math
from typing import Sequence
from .cad_limits import cad_pitch_cap_deg

HARD_YAW_LIMIT_DEG = 27.0
HARD_PITCH_LIMIT_DEG = 8.0
NEUTRAL_PULSES_US = (1800.0, 1800.0, 1200.0, 1200.0)
PULSE_US_PER_CRANK_DEG = 2000.0 / 270.0
FIRMWARE_PULSE_RANGE_US = (500.0, 2500.0)
FIRMWARE_MAX_PULSE_STEP_US = 2.0
FIRMWARE_FRAME_S = 0.020
FIRMWARE_PULSE_RATE_US_S = FIRMWARE_MAX_PULSE_STEP_US / FIRMWARE_FRAME_S


def _finite(value: float, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} must be a finite number")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{label} must be a finite number")
    return value


def _angles(yaw_deg: float, pitch_deg: float) -> tuple[float, float]:
    yaw = _finite(yaw_deg, "yaw_deg")
    pitch = _finite(pitch_deg, "pitch_deg")
    if not 0.0 <= yaw <= HARD_YAW_LIMIT_DEG:
        raise ValueError("yaw outside firmware 0..27 degree bounds")
    if not -HARD_PITCH_LIMIT_DEG <= pitch <= HARD_PITCH_LIMIT_DEG:
        raise ValueError("pitch outside firmware -8..8 degree bounds")
    return yaw, pitch


def inverse_side(yaw_deg: float, pitch_deg: float) -> tuple[float, float]:
    """Return (upper phi, lower phi) in degrees on the firmware's neutral branch.

    Global CAD crank rx is (-upper phi, +lower phi). No additional left/right
    sign belongs in this function; the calibrated PWM map applies it below.
    """
    yaw, pitch = _angles(yaw_deg, pitch_deg)
    cy, sy = math.cos(math.radians(-yaw)), math.sin(math.radians(-yaw))
    cp, sp = math.cos(math.radians(pitch)), math.sin(math.radians(pitch))
    phi = []
    for sign in (1.0, -1.0):
        az = sign * 48.0
        x = -55.0 * cy - 12.0 * sy * cp + az * sy * sp
        y = -55.0 * sy + 12.0 * cy * cp - az * cy * sp
        z = 12.0 * sp + az * cp
        dx, dy, dz = x + 65.0, y + 36.0, z - az
        rho = math.hypot(dy, dz)
        if rho < 1e-10:
            raise ValueError("singular R8 linkage")
        q = (dx * dx + dy * dy + dz * dz + 576.0 - 2980.0) / (48.0 * rho)
        if abs(q) > 1.0 + 1e-9:
            raise ValueError("unreachable R8 linkage pose")
        q = max(-1.0, min(1.0, q))
        phi.append(math.degrees(math.asin(q) - math.atan2(sign * dz, dy)))
    return phi[0], phi[1]


def pose_to_pulses_us(pose_deg: Sequence[float]) -> tuple[float, float, float, float]:
    """Return unrounded UR/UL/DR/DL pulse commands in microseconds.

    The MCU rounds each signed offset to nearest microsecond, ties away from
    zero, then independently limits every committed frame to 2 us per output.
    Floating-point results here support a conservative host trajectory budget;
    they do not bypass firmware quantization or constitute measured PWM.
    """
    if len(pose_deg) != 4:
        raise ValueError("pose must contain left yaw/pitch and right yaw/pitch")
    left_upper, left_lower = inverse_side(pose_deg[0], pose_deg[1])
    right_upper, right_lower = inverse_side(pose_deg[2], pose_deg[3])
    offsets = (-right_upper, left_upper, right_lower, -left_lower)
    pulse = tuple(c + phi * PULSE_US_PER_CRANK_DEG for c, phi in zip(NEUTRAL_PULSES_US, offsets))
    if not all(FIRMWARE_PULSE_RANGE_US[0] <= p <= FIRMWARE_PULSE_RANGE_US[1] for p in pulse):
        raise ValueError("pose exceeds firmware pulse bounds")
    return pulse


def inward_offsets_mm(yaw_deg: float, pitch_deg: float) -> tuple[float, float]:
    """Return (upper, lower) link-anchor CAD Y displacements from neutral.

    Positive is the inward *projection proxy* under the established +yaw
    closing convention. Real pad contact displacement requires the installed
    CAD-to-seat transform and contact-point geometry, which are not measured.
    """
    yaw, pitch = _angles(yaw_deg, pitch_deg)
    psi, beta = math.radians(yaw), math.radians(pitch)
    common = 55.0 * math.sin(psi) + 12.0 * math.cos(beta) * math.cos(psi) - 12.0
    differential = 48.0 * math.sin(beta) * math.cos(psi)
    return common - differential, common + differential


def max_pitch_for_yaw(yaw_deg: float, pitch_limit_deg: float = 8.0) -> float:
    """Largest symmetric |pitch| keeping both anchor Y offsets nonnegative.

    This optional coupling restriction prevents one link anchor retracting
    behind its neutral Y projection while the other closes. It is an analytical
    command constraint, not a measured human-contact or collision limit. The
    returned bound is symmetric on both sides because both use local R8 axes.
    """
    yaw, _ = _angles(yaw_deg, 0.0)
    limit = _finite(pitch_limit_deg, "pitch_limit_deg")
    if not 0.0 <= limit <= HARD_PITCH_LIMIT_DEG:
        raise ValueError("pitch_limit_deg outside 0..8 degree bounds")
    if yaw == 0.0 or limit == 0.0:
        return 0.0
    psi = math.radians(yaw)
    radius = math.hypot(12.0, 48.0)
    threshold = (12.0 - 55.0 * math.sin(psi)) / (radius * math.cos(psi))
    geometric_limit = math.degrees(math.acos(max(-1.0, min(1.0, threshold)))) - math.degrees(math.atan2(48.0, 12.0))
    return min(limit, max(0.0, geometric_limit))


def allowable_pitch_for_yaw(yaw_deg: float, pitch_limit_deg: float = 8.0) -> float:
    """Intersect user cap, CAD clearance cap and inward-anchor constraint.

    The immutable CAD profile is shared with the MCU and identified before ARM.
    Both curves are concave, so straight interpolation remains in their common
    region. These are geometric limits, not torque or comfort guarantees.
    """
    low_yaw_cap = max_pitch_for_yaw(yaw_deg, pitch_limit_deg)
    return min(low_yaw_cap, cad_pitch_cap_deg(yaw_deg))
