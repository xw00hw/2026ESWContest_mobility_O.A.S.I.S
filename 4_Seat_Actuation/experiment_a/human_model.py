"""Vehicle-only, uncontrolled torso-motion surrogate for experiment A.

This is an explicitly parameterized *prediction*, never measured occupant pose
and never the response after bolster assistance. It has no pressure, camera,
actuator, network, or serial input. See HUMAN_MODEL.md for the primary sources,
anthropometric reduction, coordinate assumptions, and unobserved parameters.

The linear equations are I*q'' + C*q' + K_net*q = m*h_vertical*a.
K_net includes the net effect of gravity, seat contact, and postural support;
adding a separate gravitational stiffness would double count that effect.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import math


def _finite(value: float, name: str, minimum: float, maximum: float) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    if not math.isfinite(value) or not minimum <= value <= maximum:
        raise ValueError(f"{name} must be within [{minimum}, {maximum}]")
    return float(value)


@dataclass(frozen=True, slots=True)
class HumanModelConfig:
    """Anthropometry and unidentified effective dynamics, all in SI units.

    The first seven fields require an explicit choice. Frequencies and damping
    are design parameters until identified for the participant and seat. Height
    and mass alone cannot identify them. Reference fractions are an engineering
    reduction of Desai et al. (2023), Table 2, not population-average dynamics.
    """

    body_mass_kg: float
    body_height_m: float
    seat_recline_deg: float
    roll_natural_frequency_hz: float
    pitch_natural_frequency_hz: float
    roll_damping_ratio: float
    pitch_damping_ratio: float
    upper_body_mass_fraction: float = 0.46693227091633466
    com_height_fraction: float = 0.16323504563553629
    roll_inertia_height_factor: float = 0.050020267241007624
    pitch_inertia_height_factor: float = 0.04875873886806713
    acceleration_filter_tau_s: float = 0.10
    preview_horizon_s: float = 0.0
    max_angle_deg: float = 20.0
    max_rate_deg_s: float = 90.0

    def __post_init__(self):
        ranges = {
            "body_mass_kg": (20., 250.), "body_height_m": (1.2, 2.3),
            "seat_recline_deg": (0., 60.),
            "roll_natural_frequency_hz": (.1, 5.),
            "pitch_natural_frequency_hz": (.1, 5.),
            "roll_damping_ratio": (.1, 2.), "pitch_damping_ratio": (.1, 2.),
            "upper_body_mass_fraction": (.1, .8), "com_height_fraction": (.02, .4),
            "roll_inertia_height_factor": (.001, .2),
            "pitch_inertia_height_factor": (.001, .2),
            "acceleration_filter_tau_s": (.001, 2.), "preview_horizon_s": (0., .25),
            "max_angle_deg": (.1, 30.), "max_rate_deg_s": (1., 180.),
        }
        for name, (lo, hi) in ranges.items():
            _finite(getattr(self, name), name, lo, hi)

    def effective_parameters(self) -> dict[str, float]:
        mass = self.body_mass_kg * self.upper_body_mass_fraction
        com = self.body_height_m * self.com_height_fraction
        lever = com * math.cos(math.radians(self.seat_recline_deg))
        roll_i = mass * self.body_height_m ** 2 * self.roll_inertia_height_factor
        pitch_i = mass * self.body_height_m ** 2 * self.pitch_inertia_height_factor
        roll_w = 2 * math.pi * self.roll_natural_frequency_hz
        pitch_w = 2 * math.pi * self.pitch_natural_frequency_hz
        return {
            "upper_body_mass_kg": mass, "com_distance_m": com,
            "vertical_com_lever_m": lever,
            "roll_inertia_kg_m2": roll_i, "pitch_inertia_kg_m2": pitch_i,
            "roll_net_stiffness_nm_per_rad": roll_i * roll_w ** 2,
            "pitch_net_stiffness_nm_per_rad": pitch_i * pitch_w ** 2,
            "roll_damping_nms_per_rad": 2 * self.roll_damping_ratio * roll_i * roll_w,
            "pitch_damping_nms_per_rad": 2 * self.pitch_damping_ratio * pitch_i * pitch_w,
            "roll_static_gain_deg_per_mps2": math.degrees(mass * lever / (roll_i * roll_w ** 2)),
            "pitch_static_gain_deg_per_mps2": math.degrees(mass * lever / (pitch_i * pitch_w ** 2)),
        }


@dataclass(frozen=True, slots=True)
class PredictedHumanState:
    """Angles relative to an assumed neutral torso, not observed angles.

    +roll = leaning to the occupant's right; +pitch = leaning forward.
    Vehicle +x is forward and +y left. Thus +ay drives +roll, and +ax
    drives -pitch. Pad Pitch is a different mechanical coordinate.
    """

    roll_deg: float
    pitch_deg: float
    roll_rate_deg_s: float
    pitch_rate_deg_s: float
    preview_roll_deg: float
    preview_pitch_deg: float
    filtered_ax_mps2: float
    filtered_ay_mps2: float
    limited: bool

    def to_dict(self):
        return asdict(self)


class HumanModel:
    """Causal two-axis reduced model; callers own timestamps and freshness.

    Only fresh, validated vehicle samples may be passed to ``step``. The input
    acceleration is held over this interval; its low-pass response is evaluated
    at RK4 stages, with substeps <= 5 ms. Gaps above 50 ms are rejected, never
    extrapolated as valid input. Reset after stream changes or invalid data.
    """

    def __init__(self, config: HumanModelConfig):
        if not isinstance(config, HumanModelConfig):
            raise TypeError("An explicit HumanModelConfig is required")
        self.config = config
        parameters = config.effective_parameters()
        mass_lever = parameters["upper_body_mass_kg"] * parameters["vertical_com_lever_m"]
        self._roll_b = mass_lever / parameters["roll_inertia_kg_m2"]
        self._pitch_b = mass_lever / parameters["pitch_inertia_kg_m2"]
        self._roll_w = 2 * math.pi * config.roll_natural_frequency_hz
        self._pitch_w = 2 * math.pi * config.pitch_natural_frequency_hz
        self.reset()

    def reset(self) -> PredictedHumanState:
        self._roll = self._pitch = 0.
        self._roll_rate = self._pitch_rate = 0.
        self._ax = self._ay = 0.
        return self._state(False)

    def _state(self, limited: bool) -> PredictedHumanState:
        cfg = self.config
        roll, pitch = math.degrees(self._roll), math.degrees(self._pitch)
        roll_rate, pitch_rate = math.degrees(self._roll_rate), math.degrees(self._pitch_rate)
        raw_preview_roll = roll + roll_rate * cfg.preview_horizon_s
        raw_preview_pitch = pitch + pitch_rate * cfg.preview_horizon_s
        preview_roll = max(-cfg.max_angle_deg, min(cfg.max_angle_deg, raw_preview_roll))
        preview_pitch = max(-cfg.max_angle_deg, min(cfg.max_angle_deg, raw_preview_pitch))
        return PredictedHumanState(roll, pitch, roll_rate, pitch_rate,
                                  preview_roll, preview_pitch, self._ax, self._ay,
                                  limited or preview_roll != raw_preview_roll or
                                  preview_pitch != raw_preview_pitch)

    @staticmethod
    def _rk4(q, rate, acceleration, step, omega, damping, drive):
        def derivative(pos, vel, at):
            return vel, drive * acceleration(at) - 2 * damping * omega * vel - omega ** 2 * pos
        a = derivative(q, rate, 0.)
        b = derivative(q + step * a[0] / 2, rate + step * a[1] / 2, step / 2)
        c = derivative(q + step * b[0] / 2, rate + step * b[1] / 2, step / 2)
        d = derivative(q + step * c[0], rate + step * c[1], step)
        return (q + step * (a[0] + 2*b[0] + 2*c[0] + d[0]) / 6,
                rate + step * (a[1] + 2*b[1] + 2*c[1] + d[1]) / 6)

    def _bound(self, q, rate):
        angle_limit = math.radians(self.config.max_angle_deg)
        rate_limit = math.radians(self.config.max_rate_deg_s)
        limited = abs(q) > angle_limit or abs(rate) > rate_limit
        q = max(-angle_limit, min(angle_limit, q))
        rate = max(-rate_limit, min(rate_limit, rate))
        # Remove only velocity farther outside the model's valid angle domain.
        if abs(q) >= angle_limit and q * rate > 0:
            rate = 0.
        return q, rate, limited

    def step(self, ax_mps2: float, ay_mps2: float, dt_s: float) -> PredictedHumanState:
        # Validate the entire input before mutating state.
        ax = _finite(ax_mps2, "ax_mps2", -15., 15.)
        ay = _finite(ay_mps2, "ay_mps2", -12., 12.)
        dt = _finite(dt_s, "dt_s", .000001, .05)
        cfg = self.config
        substeps = max(1, math.ceil(dt / .005))
        step = dt / substeps
        limited = False
        for _ in range(substeps):
            old_ax, old_ay = self._ax, self._ay
            # +ax accelerates the seat forward: relative torso lean is backward.
            a_x = lambda t: -(ax + (old_ax - ax) * math.exp(-t / cfg.acceleration_filter_tau_s))
            a_y = lambda t: ay + (old_ay - ay) * math.exp(-t / cfg.acceleration_filter_tau_s)
            roll, roll_rate = self._rk4(self._roll, self._roll_rate, a_y, step,
                                       self._roll_w, cfg.roll_damping_ratio, self._roll_b)
            pitch, pitch_rate = self._rk4(self._pitch, self._pitch_rate, a_x, step,
                                         self._pitch_w, cfg.pitch_damping_ratio, self._pitch_b)
            self._roll, self._roll_rate, roll_limited = self._bound(roll, roll_rate)
            self._pitch, self._pitch_rate, pitch_limited = self._bound(pitch, pitch_rate)
            limited |= roll_limited or pitch_limited
            self._ax = ax + (old_ax - ax) * math.exp(-step / cfg.acceleration_filter_tau_s)
            self._ay = ay + (old_ay - ay) * math.exp(-step / cfg.acceleration_filter_tau_s)
        return self._state(limited)


def nominal_design_config() -> HumanModelConfig:
    """An OFFLINE example, not identified occupant or approved motion settings.

    1.76 m / 75.3 kg is the cited model's reference size. Frequencies 1.2 / 1 Hz
    and damping 0.85 are engineering design choices, not values extracted from
    a human dataset. Use an explicit participant configuration for experiments.
    """
    return HumanModelConfig(75.3, 1.76, 0., 1.2, 1., .85, .85)
