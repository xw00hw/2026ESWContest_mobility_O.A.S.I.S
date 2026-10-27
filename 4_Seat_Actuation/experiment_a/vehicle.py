"""Experiment A's sole measured control input: BeamNG collector vehicle data.

The shared adapter has no pressure, camera, posture or hardware imports. It
preserves the original same-PC monotonic clock, validates every stream/quality
boundary, and defines vehicle +x forward, +y left, +z up. Steering and brake
can supply a bounded advance cue; they are not road-wheel angles or forces.
Throttle remains observational. No yaw-rate field is inferred from an unknown
schema. Missing gear/orientation disables the advance cue, not core ay control.
"""
from dataclasses import asdict, dataclass, replace
from .impact import MAX_ABS_ACCEL_MPS2

from fallback.beamng import (
    BRIDGE_HOST, BRIDGE_PORT, BRIDGE_SCHEMA, CLOCK_ID, MAX_DATAGRAM_BYTES,
    BridgeReceiver, VehicleFrame as _CoreFrame, VehicleValidator as _CoreValidator,
    _number, read_bridge_packet, read_vehicle_csv,
)


@dataclass(frozen=True, slots=True)
class VehicleFrame(_CoreFrame):
    """Optional collector fields, never silently substituted for linear accel.

    specific_force follows the collector's vehicle axes and includes its
    gravity contribution. roll/pitch are vehicle orientation, not occupant
    angles. A controller selecting these fields must reject missing values.
    """
    specific_force_x_mps2: float | None = None
    specific_force_y_mps2: float | None = None
    specific_force_z_mps2: float | None = None
    roll_rad: float | None = None
    pitch_rad: float | None = None
    gear_index: float | None = None


class VehicleValidator(_CoreValidator):
    def __init__(self, **kwargs):
        # Legacy pressure-feedback fallback retains its own 12/15 defaults.
        # Experiment A separates broad data validity from impact detection.
        kwargs.setdefault('max_abs_ay', MAX_ABS_ACCEL_MPS2)
        kwargs.setdefault('max_abs_ax', MAX_ABS_ACCEL_MPS2)
        super().__init__(**kwargs)

    def parse(self, row, now_ns):
        core = super().parse(row, now_ns)
        optional, malformed = {}, []
        for name in ("specific_force_x_mps2", "specific_force_y_mps2", "specific_force_z_mps2",
                     "roll_rad", "pitch_rad", "gear_index"):
            try:
                optional[name] = _number(row.get(name))
            except ValueError:
                optional[name] = None
                malformed.append(name)
        frame = VehicleFrame(**asdict(core), **optional)
        if malformed:
            return replace(frame, valid=False, reason="invalid_number:"+malformed[0])
        return frame

__all__ = ["BridgeReceiver", "VehicleFrame", "VehicleValidator",
           "read_bridge_packet", "read_vehicle_csv"]
