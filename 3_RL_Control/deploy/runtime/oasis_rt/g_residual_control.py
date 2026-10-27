"""Measured-G yaw request with bounded learned correction and optional preview.

Pure high-level calculation, nominally called at 20 Hz. Inputs are fresh vehicle
G (9.81 m/s^2), vehicle speed in m/s, and SAME-TICK unfiltered actor/prior yaw
vectors in LEFT, RIGHT degrees. Positive lateral G supports RIGHT. The prior
must be the deployed model's own training prior, not a configurable fallback.
The raw actor already includes its clipped prior: their angle difference is an
effective residual after the original action saturation, not the bare NN head.

The returned angle is BEFORE the caller's one output filter and all existing
pressure, stale-input, CAD, communication and STM rate/bounds checks. This module
does not send commands, filter yaw, verify input freshness, or estimate force.
Disabled operation is the caller's unchanged legacy path; do not call this module
and then claim the learned policy alone produced its output.

body_scale (v9, oasis_rt.body_scale) multiplies the G base curve and the preview
angle for an occupant whose camera neutral differs from the reference user. It is
frozen per session; 1.0 is the reference user's reviewed behaviour. The 20-degree
measured cap, the 10-degree preview-only cap and the residual bound are unchanged.
"""
from dataclasses import asdict, dataclass
import math
from collections.abc import Mapping


def _finite(name, value):
    if isinstance(value, (bool, str, bytes)):
        raise ValueError(f'{name} must be a finite number')
    try:
        value = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f'{name} must be a finite number') from exc
    if not math.isfinite(value):
        raise ValueError(f'{name} must be a finite number')
    return value


def _clip(value, lower, upper):
    return min(upper, max(lower, value))


def _side(value):
    return 1 if value > 0 else -1 if value < 0 else 0


def _label(side):
    return 'right' if side > 0 else 'left' if side < 0 else 'none'


@dataclass(frozen=True)
class GResidualConfig:
    # These are provisional settings informed by one recorded run, not a force
    # calibration. The 1G/20-degree endpoint is an explicit experiment request.
    deadband_g: float = 0.01
    measured_on_g: float = 0.02
    measured_off_g: float = 0.01
    knee_g: float = 0.0625
    knee_yaw_deg: float = 10.0
    full_g: float = 1.0
    full_yaw_deg: float = 20.0
    preview_on_g: float = 0.03
    preview_off_g: float = 0.015
    preview_full_g: float = 0.0625
    preview_max_yaw_deg: float = 10.0
    preview_confirm_s: float = 0.10
    residual_max_deg: float = 1.5
    speed_min_mps: float = 1.0

    def __post_init__(self):
        for name, value in asdict(self).items():
            object.__setattr__(self, name, _finite(name, value))
        if not (0 <= self.measured_off_g <= self.deadband_g < self.measured_on_g
                <= self.knee_g < self.full_g):
            raise ValueError('require 0 <= measured_off <= deadband < measured_on <= knee < full G')
        if not (0 <= self.preview_off_g < self.preview_on_g <= self.preview_full_g):
            raise ValueError('require 0 <= preview_off < preview_on <= preview_full G')
        if not (0 < self.knee_yaw_deg < self.full_yaw_deg <= 20):
            raise ValueError('require 0 < knee yaw < full yaw <=20 degrees')
        if not (0 <= self.preview_max_yaw_deg <= self.knee_yaw_deg
                and 0 <= self.residual_max_deg <= self.knee_yaw_deg):
            raise ValueError('preview/residual bounds must lie between zero and knee yaw')
        if not (0 < self.preview_confirm_s <= 1 and 0 < self.speed_min_mps <= 10):
            raise ValueError('preview confirmation or speed gate outside supported range')

    def to_dict(self):
        return asdict(self)


class GResidualGovernor:
    """State contains direction hysteresis only, never an actuator command lease."""
    version = 'measured_g_bounded_residual_v1'

    def __init__(self, config=None, body_scale=1.0):
        if config is None:
            config = GResidualConfig()
        elif isinstance(config, Mapping):
            try:
                config = GResidualConfig(**config)
            except TypeError as exc:
                raise ValueError(f'unknown G residual configuration field: {exc}') from exc
        if not isinstance(config, GResidualConfig):
            raise ValueError('config must be GResidualConfig or a mapping')
        from .body_scale import SCALE_MIN, SCALE_MAX
        body_scale = _finite('body_scale', body_scale)
        if not SCALE_MIN <= body_scale <= SCALE_MAX:
            raise ValueError(f'body_scale must be in [{SCALE_MIN}, {SCALE_MAX}]')
        self.config, self.body_scale = config, body_scale
        self.reset()

    def reset(self):
        self.measured_side = 0
        self.preview_side = 0
        self.preview_pending_side = 0
        self.preview_pending_s = 0.0

    def base_yaw(self, abs_g):
        """Monotone magnitude curve; direction gate/preview/residual are separate."""
        a = _finite('abs_g', abs_g)
        if a < 0:
            raise ValueError('abs_g must be nonnegative')
        c = self.config
        if a <= c.deadband_g:
            return 0.0
        if a <= c.knee_g:
            return c.knee_yaw_deg * (a-c.deadband_g) / (c.knee_g-c.deadband_g)
        return c.knee_yaw_deg + (c.full_yaw_deg-c.knee_yaw_deg) * _clip(
            (a-c.knee_g)/(c.full_g-c.knee_g), 0.0, 1.0)

    @staticmethod
    def _yaw_pair(name, values):
        try:
            values = tuple(values)
        except TypeError as exc:
            raise ValueError(f'{name} must have LEFT, RIGHT yaw') from exc
        if len(values) != 2:
            raise ValueError(f'{name} must have LEFT, RIGHT yaw')
        pair = tuple(_finite(f'{name}[{i}]', v) for i, v in enumerate(values))
        if not all(0 <= v <= 20 for v in pair):
            raise ValueError(f'{name} must be mapped yaw in [0,20] degrees')
        return pair

    def _directions(self, measured_g, predicted_g, dt_s):
        c = self.config
        m, p = _side(measured_g), _side(predicted_g)
        if abs(measured_g) >= c.measured_on_g:
            self.measured_side = m
        elif (abs(measured_g) <= c.measured_off_g or
              (self.measured_side and m != self.measured_side)):
            self.measured_side = 0
        # Never retain the old predicted direction across a zero crossing. A new
        # direction must remain above ON for the confirmation interval.
        if abs(predicted_g) <= c.preview_off_g or p != self.preview_side:
            self.preview_side = 0
        if abs(predicted_g) >= c.preview_on_g:
            if p != self.preview_pending_side:
                self.preview_pending_side = p
                self.preview_pending_s = 0.0
            self.preview_pending_s = min(c.preview_confirm_s,
                                         self.preview_pending_s + dt_s)
            if self.preview_pending_s + 1e-12 >= c.preview_confirm_s:
                self.preview_side = p
        else:
            self.preview_pending_side = 0
            self.preview_pending_s = 0.0

    def step(self, measured_g, predicted_g, speed_mps, raw_rl_yaw_lr,
             prior_yaw_lr, dt_s=0.05):
        """Return ((left,right) degrees, JSON-safe diagnostic snapshot).

        Nonfinite/invalid inputs raise ValueError for the caller's existing fault
        handling. Input age/pressure protection remain the caller's responsibility.
        Speed below threshold resets direction history and requests exact zero.
        """
        g = _finite('measured_g', measured_g)
        pred = _finite('predicted_g', predicted_g)
        speed = _finite('speed_mps', speed_mps)
        dt = _finite('dt_s', dt_s)
        if not (0 < dt <= 0.5):
            raise ValueError('dt_s must be in (0,0.5] seconds')
        raw = self._yaw_pair('raw_rl_yaw_lr', raw_rl_yaw_lr)
        prior = self._yaw_pair('prior_yaw_lr', prior_yaw_lr)
        c = self.config
        delta = tuple(a-b for a, b in zip(raw, prior))
        stopped = speed < c.speed_min_mps
        if stopped:
            self.reset()
        else:
            self._directions(g, pred, dt)
        selected = self.measured_side or self.preview_side
        source = 'stopped' if stopped else 'measured' if self.measured_side else 'preview' if selected else 'none'
        k = self.body_scale
        base = min(k * self.base_yaw(abs(g)), c.full_yaw_deg) if self.measured_side else 0.0
        preview_agrees = bool(selected and self.preview_side == selected)
        preview = min(k * c.preview_max_yaw_deg * _clip(
            (abs(pred)-c.preview_off_g)/(c.preview_full_g-c.preview_off_g), 0., 1.),
            c.preview_max_yaw_deg) if preview_agrees else 0.0
        reference = max(base, preview)
        taper = _clip((c.full_g-abs(g))/(c.full_g-c.knee_g), 0., 1.)
        residual_limit = c.residual_max_deg * taper
        requested = [0.0, 0.0]
        correction = 0.0
        if selected:
            j = 1 if selected > 0 else 0
            correction = _clip(delta[j], -residual_limit, residual_limit)
            # Preview alone can never exceed 10 degrees by borrowing a positive
            # residual. Once measured direction exists, bounded fine adjustment
            # is allowed; it fades to exact zero residual at the full-G endpoint.
            cap = c.full_yaw_deg if self.measured_side else c.preview_max_yaw_deg
            requested[j] = _clip(reference + correction, 0., cap)
            correction = requested[j] - reference
        snapshot = dict(version=self.version, body_scale=k, measured_g=g, predicted_g=pred,
            speed_mps=speed, speed_gate=stopped, measured_side=_label(self.measured_side),
            preview_side=_label(self.preview_side), selected_side=_label(selected),
            direction_source=source, preview_pending_s=self.preview_pending_s,
            preview_opposed_to_measured=bool(self.measured_side and self.preview_side
                                            and self.measured_side != self.preview_side),
            base_yaw_deg=base, preview_yaw_deg=preview, effective_base_yaw_deg=reference,
            residual_raw_lr_deg=list(delta), residual_limit_deg=residual_limit,
            residual_applied_deg=correction, requested_yaw_lr_deg=requested)
        return tuple(requested), snapshot
