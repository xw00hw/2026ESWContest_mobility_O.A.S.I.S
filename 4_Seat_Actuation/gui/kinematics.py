"""C4_R8 structure IK, port of reference/c4_r8_kinematics.js.

mm and degrees. R=Rz(-yaw)*Rx(pitch), same asin branch as the original viewer.
phi is local crank rotation; global CAD rx is -phi1,+phi2. User confirmed
D3=-phi1, D4=phi1, D5=phi2, D6=-phi2. All DS, mirrored 1200us crank zero.
"""
from math import asin, atan2, cos, degrees, dist, hypot, isfinite, radians, sin

YAW_LIMIT_DEG, PITCH_LIMIT_DEG = 20, 8
YAW_SCAN_LIMIT_DEG = 27
CRANK_ZERO_US = 1200
BOOT_PULSES = (3000-CRANK_ZERO_US, CRANK_ZERO_US, CRANK_ZERO_US, 3000-CRANK_ZERO_US)
POSE_FIELDS = ('left_yaw_mdeg','left_pitch_mdeg','right_yaw_mdeg','right_pitch_mdeg')

GEOMETRY = dict(revision='C4_R8', units='mm', radius_mm=24,
                centers_mm=[[-65,-36,48],[-65,-36,-48]],
                anchors_mm=[[-55,12,48],[-55,12,-48]], signs=[1,-1],
                yaw_limits_deg=[0,YAW_LIMIT_DEG],pitch_limits_deg=[-PITCH_LIMIT_DEG,PITCH_LIMIT_DEG])


def inverse(yaw_deg,pitch_deg):
    if not all(isfinite(x) for x in (yaw_deg,pitch_deg)):
        raise ValueError('구조물 각도는 유한한 수여야 합니다.')
    if not 0<=yaw_deg<=YAW_SCAN_LIMIT_DEG or not -PITCH_LIMIT_DEG<=pitch_deg<=PITCH_LIMIT_DEG:
        raise ValueError(f'C4_R8 명령 범위: Yaw 0~{YAW_SCAN_LIMIT_DEG}°, Pitch ±{PITCH_LIMIT_DEG}°; 기구학 도달 검사 적용')
    cy,sy = cos(radians(-yaw_deg)),sin(radians(-yaw_deg))
    cp,sp = cos(radians(pitch_deg)),sin(radians(pitch_deg))
    rotation = [[cy,-sy*cp,sy*sp],[sy,cy*cp,-cy*sp],[0,sp,cp]]
    result = []
    for i,(center,anchor,sign) in enumerate(zip(GEOMETRY['centers_mm'],
                                                GEOMETRY['anchors_mm'],GEOMETRY['signs'])):
        radius = GEOMETRY['radius_mm']
        length = dist(anchor,[center[0],center[1],center[2]+sign*radius])
        position = [sum(row[j]*anchor[j] for j in range(3)) for row in rotation]
        delta = [position[j]-center[j] for j in range(3)]
        rho = hypot(delta[1],delta[2])
        if rho<1e-10:
            raise ValueError('기구학 특이점')
        q = (sum(x*x for x in delta)+radius**2-length**2)/(2*radius*rho)
        if abs(q)>1+1e-9:
            raise ValueError('구조물이 도달할 수 없는 자세입니다.')
        phi = asin(max(-1,min(1,q)))-atan2(sign*delta[2],delta[1])
        ball = [center[0],center[1]+radius*sin(phi),center[2]+sign*radius*cos(phi)]
        result.append(dict(id=f'M{i+1}',phi_deg=degrees(phi),rx_deg=-sign*degrees(phi),
            anchor_mm=position,ball_mm=ball,link_length_mm=length,
            length_error_mm=abs(dist(position,ball)-length)))
    return dict(yaw_deg=yaw_deg,pitch_deg=pitch_deg,R=rotation,motors=result)


def pulses(yaw_mdeg,pitch_mdeg):
    if type(yaw_mdeg) is not int or type(pitch_mdeg) is not int:
        raise ValueError('각도 명령은 정수 mdeg여야 합니다.')
    motors = inverse(yaw_mdeg/1000,pitch_mdeg/1000)['motors']
    upper = []
    for m in motors:
        offset = m['phi_deg']*2000/270
        delta = int(offset+.5) if offset>=0 else -int(-offset+.5)
        if not 500<=CRANK_ZERO_US+delta<=2500:
            raise ValueError('DS 모터 각도/PWM 한계 초과')
        upper.append(CRANK_ZERO_US+delta)
    return [3000-upper[0],upper[0],upper[1],3000-upper[1]]


def pair_pulses(left_yaw,left_pitch,right_yaw,right_pitch):
    left,right = pulses(left_yaw,left_pitch),pulses(right_yaw,right_pitch)
    return [right[0],left[1],right[2],left[3]]
