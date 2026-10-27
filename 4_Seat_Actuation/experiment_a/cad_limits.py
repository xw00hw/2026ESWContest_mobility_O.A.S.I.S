"""Commissioning bounds from the versioned, directly analysed R8 STEP solids.

This is a geometric clearance/kinematic guard, not proof of servo torque,
printed-part strength or occupant-contact safety. Both host and firmware use
the same canonical profile digest and piecewise-linear, concave cap.
"""
import hashlib
import json
import math

from .cad_profile_data import CAD_PROFILE

CAD_PROFILE_SHA256 = hashlib.sha256(json.dumps(CAD_PROFILE, sort_keys=True,
    separators=(',', ':'), ensure_ascii=True).encode('utf-8')).hexdigest()
CAD_YAW_MAX_MDEG = CAD_PROFILE['yaw_max_mdeg']
CAD_PITCH_MAX_MDEG = CAD_PROFILE['pitch_max_mdeg']
CAD_CAP_POINTS = tuple(tuple(p) for p in CAD_PROFILE['yaw_pitch_cap_mdeg'])
CAD_INFO_LINE = (f'#CAD,OASIS_A,v2,profile_sha256={CAD_PROFILE_SHA256},'
    f'yaw_max_mdeg={CAD_YAW_MAX_MDEG},pitch_max_mdeg={CAD_PITCH_MAX_MDEG},guard=CAD_KINEMATIC')


def _validate_profile():
    if (CAD_CAP_POINTS[0] != (0, 0) or CAD_CAP_POINTS[-1][0] != CAD_YAW_MAX_MDEG
            or len(CAD_CAP_POINTS) < 2 or CAD_PROFILE['guard'] != 'CAD_KINEMATIC'):
        raise ValueError('Invalid CAD profile endpoints')
    last_slope = math.inf
    for index, (yaw, pitch) in enumerate(CAD_CAP_POINTS):
        if type(yaw) is not int or type(pitch) is not int or not 0 <= pitch <= CAD_PITCH_MAX_MDEG:
            raise ValueError('Invalid CAD integer cap')
        if index:
            y0, p0 = CAD_CAP_POINTS[index-1]
            if yaw <= y0:
                raise ValueError('CAD yaw must strictly increase')
            slope = (pitch-p0)/(yaw-y0)
            if slope > last_slope + 1e-12:
                raise ValueError('CAD cap must be concave for straight-line trajectories')
            last_slope = slope


_validate_profile()


def cad_pitch_cap_deg(yaw_deg):
    if (isinstance(yaw_deg, bool) or not isinstance(yaw_deg, (int, float))
            or not math.isfinite(yaw_deg) or not 0 <= yaw_deg*1000 <= CAD_YAW_MAX_MDEG):
        raise ValueError('Yaw outside CAD commissioning envelope')
    yaw = yaw_deg * 1000
    for (y0, p0), (y1, p1) in zip(CAD_CAP_POINTS, CAD_CAP_POINTS[1:]):
        if yaw <= y1:
            return (p0 + (p1-p0)*(yaw-y0)/(y1-y0))/1000
    raise ValueError('Invalid CAD table')


def pose_within_cad(pose_mdeg):
    if len(pose_mdeg) != 4 or any(type(v) is not int for v in pose_mdeg):
        return False
    for i in (0, 2):
        yaw, pitch = pose_mdeg[i:i+2]
        if not 0 <= yaw <= CAD_YAW_MAX_MDEG:
            return False
        # Integer floor matches the MCU contract and cannot round outward.
        for (y0, p0), (y1, p1) in zip(CAD_CAP_POINTS, CAD_CAP_POINTS[1:]):
            if yaw <= y1:
                cap = (p0*(y1-y0)+(p1-p0)*(yaw-y0)) // (y1-y0)
                if abs(pitch) > cap:
                    return False
                break
    return True
