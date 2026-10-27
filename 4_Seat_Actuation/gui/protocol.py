"""Integrated STM32 protocol. Structure angles, exact C4_R8 IK; no measured pose feedback."""
from __future__ import annotations

import math
from kinematics import pulses as motor_pulses
from kinematics import pair_pulses,POSE_FIELDS

# Local copy of the tested v9 parser; no dependency on sibling projects.
from pressure_protocol import parse_packet, SequenceTracker

BAUD = 460800
HELLO_PREFIX = '#HELLO,OASIS_CONTROL,v6,'
REQUIRED_CAPABILITIES = dict(yaw_min_mdeg=0, yaw_limit_mdeg=27000, pitch_limit_mdeg=8000,
                             timeout_ms=1500, motor_rate_mdeg_s=15000, ds_outputs=4, ik_revision=8,
                             crank_zero_us=1200,joints=2)
STATE_FIELDS = ('mcu_ms', 'armed', 'moving', *POSE_FIELDS,
                *('target_'+name for name in POSE_FIELDS), 'D3_us', 'D4_us',
                'D5_us', 'D6_us', 'command_id', 'fault')


def capabilities(line):
    if not line.startswith(HELLO_PREFIX):
        raise ValueError('C4_R8 / DS 통합 펌웨어 v6 응답이 아닙니다.')
    values = dict(part.split('=', 1) for part in line[len(HELLO_PREFIX):].split(','))
    values = {key: int(value) for key, value in values.items()}
    if any(values.get(key) != value for key, value in REQUIRED_CAPABILITIES.items()):
        raise ValueError('GUI와 펌웨어의 각도·속도·시간 제한이 다릅니다.')
    return values


def set_command(command_id, left_yaw, left_pitch, right_yaw, right_pitch):
    if type(command_id) is not int or not 0 <= command_id <= 0x7fffffff:
        raise ValueError('명령 ID가 올바르지 않습니다.')
    pose = (left_yaw,left_pitch,right_yaw,right_pitch)
    if not all(math.isfinite(x) for x in pose):
        raise ValueError('유한한 각도를 입력하세요.')
    pose = tuple(round(x*1000) for x in pose)
    pair_pulses(*pose)
    return f'SET,{command_id},'+','.join(map(str,pose))


def parse_state(line):
    fields = line.split(',')
    if fields[0] != '#STATE' or len(fields) != len(STATE_FIELDS) + 1:
        raise ValueError('잘못된 STM32 상태 패킷')
    result = dict(zip(STATE_FIELDS, fields[1:]))
    for key in STATE_FIELDS[:-1]:
        result[key] = int(result[key])
    if result['armed'] not in (0, 1) or result['moving'] not in (0, 1):
        raise ValueError('잘못된 구동 상태')
    expected = pair_pulses(*(result[name] for name in POSE_FIELDS))
    if expected != [result[f'D{i}_us'] for i in (3, 4, 5, 6)]:
        raise ValueError('각도와 PWM 상태가 일치하지 않습니다.')
    return result
