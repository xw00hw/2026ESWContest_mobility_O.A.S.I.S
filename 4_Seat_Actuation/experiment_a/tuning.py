"""Adopt one validated STM Live Expressions snapshot before the ARM request.

These six values are operator configuration, not pressure/body feedback. The
MCU publishes only committed values; transport binds ARM to their revision.
An active run never changes its copy or substitutes uncommitted mailbox data.
"""
from dataclasses import replace


TUNING_FIELDS = (
    ('yaw_max_mdeg', 'max_yaw_deg', 'Yaw 상한', '°'),
    ('pitch_max_mdeg', 'max_pitch_deg', 'Pitch 상한', '±°'),
    ('yaw_rate_mdeg_s', 'max_yaw_rate_deg_s', 'Yaw 속도', '°/s'),
    ('pitch_rate_mdeg_s', 'max_pitch_rate_deg_s', 'Pitch 속도', '°/s'),
    ('full_scale_ay_mmps2', 'full_scale_ay_mps2', '최대 반응 횡가속도', 'm/s²'),
    ('pitch_scale_permille', 'pitch_per_predicted_roll', 'Pitch 응답 배율', '배'),
)
TUNING_KEYS = frozenset(['revision']+[key for key, *_ in TUNING_FIELDS])


def apply_firmware_tuning(settings, snapshot):
    if not isinstance(snapshot, dict) or set(snapshot) != TUNING_KEYS:
        raise ValueError('STM tuning snapshot fields do not match the experiment contract')
    if any(type(value) is not int for value in snapshot.values()):
        raise ValueError('STM tuning values must be exact integers')
    if not 0 <= snapshot['revision'] <= 0xffffffff:
        raise ValueError('Invalid STM tuning revision')
    changes = {setting: snapshot[key]/1000 for key,setting,*_ in TUNING_FIELDS}
    candidate = replace(settings, **changes, metadata=dict(settings.metadata,
        tuning_authority='STM32_Live_Expressions_committed_snapshot',
        firmware_tuning_revision=snapshot['revision']))
    candidate.validate()
    return candidate, dict(snapshot)


def tuning_display_rows(snapshot):
    return [(label, 'values.'+key, '—' if snapshot is None else f'{snapshot[key]/1000:g} {unit}')
            for key,_,label,unit in TUNING_FIELDS]
