"""Body-independent causal BeamNG response, with no measured occupant input.

atan2(a_y, g) is a vehicle acceleration-vector angle used only to shape the pad
command. It is NOT a prediction/measurement of trunk lean or a cancellation
force. No height, mass, seat pressure, camera or body-type calibration enters.
"""
from dataclasses import asdict, dataclass
import math
from .impact import MAX_ABS_ACCEL_MPS2

GRAVITY_MPS2 = 9.80665


@dataclass(frozen=True)
class VehicleResponseState:
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    roll_rate_deg_s: float = 0.0
    pitch_rate_deg_s: float = 0.0
    preview_roll_deg: float = 0.0
    preview_pitch_deg: float = 0.0
    filtered_ax_mps2: float = 0.0
    filtered_ay_mps2: float = 0.0
    limited: bool = False

    def to_dict(self):
        return asdict(self)


class VehicleResponse:
    def __init__(self, filter_tau_s: float):
        if isinstance(filter_tau_s, bool) or not math.isfinite(filter_tau_s) or not .02 <= filter_tau_s <= 2:
            raise ValueError('Invalid vehicle response filter time constant')
        self.filter_tau_s = filter_tau_s
        self.reset()

    def reset(self):
        self.state = VehicleResponseState()
        return self.state

    def step(self, ax_mps2, ay_mps2, dt_s):
        for x, lo, hi in ((ax_mps2, -MAX_ABS_ACCEL_MPS2, MAX_ABS_ACCEL_MPS2),
                          (ay_mps2, -MAX_ABS_ACCEL_MPS2, MAX_ABS_ACCEL_MPS2), (dt_s, 1e-6, .05)):
            if isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) or not lo <= x <= hi:
                raise ValueError('Invalid vehicle response input')
        old = self.state
        alpha = -math.expm1(-dt_s / self.filter_tau_s)
        ax = old.filtered_ax_mps2 + alpha * (ax_mps2 - old.filtered_ax_mps2)
        ay = old.filtered_ay_mps2 + alpha * (ay_mps2 - old.filtered_ay_mps2)
        roll = math.degrees(math.atan2(ay, GRAVITY_MPS2))
        pitch = -math.degrees(math.atan2(ax, GRAVITY_MPS2))
        self.state = VehicleResponseState(roll, pitch, (roll-old.roll_deg)/dt_s,
            (pitch-old.pitch_deg)/dt_s, roll, pitch, ax, ay, False)
        return self.state
