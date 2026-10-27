"""Explicit V7 vehicle-only settings; no participant-dependent calibration."""
from dataclasses import asdict, dataclass, field, fields
import json
import math
from pathlib import Path
from .cad_limits import CAD_YAW_MAX_MDEG, CAD_PITCH_MAX_MDEG, CAD_PROFILE_SHA256

@dataclass
class Settings:
    available_bolsters: str = 'both'
    response_filter_tau_s: float = .15
    max_yaw_deg: float = 15.0
    max_pitch_deg: float = 6.0
    max_yaw_rate_deg_s: float = 2.0
    max_pitch_rate_deg_s: float = 2.0
    max_pulse_rate_us_s: float = 100.0
    full_scale_ay_mps2: float = 2.942
    on_ay_mps2: float = 0.2941995
    off_ay_mps2: float = 0.196133
    pitch_per_predicted_roll: float = 1.0
    pitch_mode: str = 'conform_lean'
    acceleration_source: str = 'linear_acceleration'
    startup_seconds: float = 2.0
    fresh_ms: float = 200.0
    max_model_sample_dt_s: float = 0.1
    neutral_tolerance_mdeg: int = 100
    steering_preview_enabled: bool = True
    steering_to_ay_sign: int = 1
    steering_preview_max_yaw_deg: float = 2.0
    steering_preview_deadband_deg: float = 5.0
    steering_preview_reference_deg: float = 45.0
    steering_preview_max_abs_deg: float = 560.0
    steering_preview_min_speed_mps: float = 5.0
    steering_preview_full_speed_mps: float = 15.0
    steering_preview_dwell_s: float = .10
    steering_preview_timeout_s: float = 1.5
    steering_preview_speed_lookahead_s: float = .25
    impact_planar_threshold_mps2: float = 25.0
    impact_jerk_threshold_mps3: float = 150.0
    impact_min_duration_s: float = .06
    impact_min_delta_v_mps: float = 1.5
    impact_extreme_planar_mps2: float = 60.0
    metadata: dict = field(default_factory=lambda: {
        'controller_id': 'A_vehicle_only_cad_v7_impact_recording',
        'impact_model': 'provisional_planar_shock_heuristic_not_collision_ground_truth',
        'impact_action': 'latched_motor_OFF_continue_vehicle_and_pressure_recording',
        'model_status': 'vehicle_acceleration_vector_response_not_human_prediction',
        'left_right_symmetry': 'user_assumption_after_reported_right_fastener_issue',
        'parameter_selection': 'CAD_geometry_with_commissioning_derating',
        'participant_source': 'none_no_body_compensation',
        'alignment_frame': 'assumed_neutral_pad_axes_aligned_with_local_backrest',
        'pressure_inverse_used': False,
        'cad_profile_sha256': CAD_PROFILE_SHA256,
        'physical_sway_reduction_verified': False,
        'steering_preview_model': 'bounded_steering_cue_not_roadwheel_or_lateral_acceleration_prediction',
        'declared_vehicle_profile': {
            'model': 'vivace', 'config': 'vehicles/vivace/vivace_160_DCT.pc',
            'powertrain': '1.6L_turbo_gasoline_FWD_7DCT',
            'pc_sha256': '4eb3d9c52cb02b1a8885779090215f6b631671a3fee03296a0d32303d367ce85',
            'identity_source': 'operator_selected_profile_not_live_vehicle_verification',
            'steering_gain_calibrated': False,
        },
    })
    confirmations: dict = field(default_factory=lambda: {
        'hardware_mapping_checked': False,
        'pitch_direction_checked': False,
        'right_bolster_fastened': False,
        'neutral_start_clear': False,
        'trial_settings_reviewed': False,
    })

    def validate(self):
        if self.available_bolsters not in ('both', 'right_only'):
            raise ValueError('available_bolsters must be both or right_only')
        if type(self.steering_preview_enabled) is not bool:
            raise ValueError('steering_preview_enabled must be boolean')
        if type(self.steering_to_ay_sign) is not int or self.steering_to_ay_sign not in (-1, 1):
            raise ValueError('steering_to_ay_sign must be -1 or 1')
        for key, lo, hi in (
            ('response_filter_tau_s', .02, 2),
            ('max_yaw_deg', .1, CAD_YAW_MAX_MDEG/1000), ('max_pitch_deg', 0, CAD_PITCH_MAX_MDEG/1000),
            ('max_yaw_rate_deg_s', .1, 20), ('max_pitch_rate_deg_s', .1, 20),
            ('max_pulse_rate_us_s', 1, 100),
            ('full_scale_ay_mps2', .5, 8), ('on_ay_mps2', .05, 2),
            ('off_ay_mps2', .01, 1.5), ('pitch_per_predicted_roll', .01, 1),
            ('startup_seconds', 2, 30), ('fresh_ms', 50, 200),
            ('max_model_sample_dt_s', .02, .1),
            ('steering_preview_max_yaw_deg', 0, 2),
            ('steering_preview_deadband_deg', 1, 15),
            ('steering_preview_reference_deg', 10, 90),
            ('steering_preview_max_abs_deg', 90, 560),
            ('steering_preview_min_speed_mps', 5, 20),
            ('steering_preview_full_speed_mps', 10, 40),
            ('steering_preview_dwell_s', .1, .5),
            ('steering_preview_timeout_s', .2, 1.5),
            ('steering_preview_speed_lookahead_s', 0, .25),
            ('impact_planar_threshold_mps2', 20, 50),
            ('impact_jerk_threshold_mps3', 50, 1000),
            ('impact_min_duration_s', .04, .2),
            ('impact_min_delta_v_mps', .5, 10),
            ('impact_extreme_planar_mps2', 50, 150),
        ):
            v = getattr(self, key)
            if isinstance(v, bool) or not isinstance(v, (float, int)) or not math.isfinite(v) or not lo <= v <= hi:
                raise ValueError('Invalid setting: '+key)
        if not self.off_ay_mps2 < self.on_ay_mps2 < self.full_scale_ay_mps2:
            raise ValueError('Require off < on < full_scale')
        if not self.steering_preview_deadband_deg < self.steering_preview_reference_deg <= self.steering_preview_max_abs_deg:
            raise ValueError('Require steering deadband < reference <= supported angle')
        if not self.steering_preview_min_speed_mps < self.steering_preview_full_speed_mps:
            raise ValueError('Require preview min speed < full speed')
        if self.steering_preview_dwell_s >= self.steering_preview_timeout_s:
            raise ValueError('Preview dwell must be shorter than timeout')
        if self.impact_extreme_planar_mps2 <= self.impact_planar_threshold_mps2:
            raise ValueError('Extreme shock threshold must exceed sustained threshold')
        if self.pitch_mode not in ('counter_lean', 'conform_lean'):
            raise ValueError('Unknown pitch_mode')
        if self.acceleration_source not in ('linear_acceleration', 'specific_force'):
            raise ValueError('Unknown acceleration_source')
        if type(self.neutral_tolerance_mdeg) is not int or not 0 <= self.neutral_tolerance_mdeg <= 100:
            raise ValueError('Invalid neutral tolerance')
        expected = {'hardware_mapping_checked','pitch_direction_checked','right_bolster_fastened',
                    'neutral_start_clear','trial_settings_reviewed'}
        if not isinstance(self.confirmations, dict) or set(self.confirmations) != expected or any(type(v) is not bool for v in self.confirmations.values()):
            raise ValueError('Explicit boolean hardware confirmations required')
        if not isinstance(self.metadata, dict):
            raise ValueError('Metadata must be an object')
        json.dumps(self.metadata, allow_nan=False)
        return self

    def missing_confirmations(self):
        self.validate()
        return [k for k,v in self.confirmations.items() if not v]

    @classmethod
    def load(cls, path):
        def unique(pairs):
            result = {}
            for k,v in pairs:
                if k in result:
                    raise ValueError('Duplicate config key: '+k)
                result[k] = v
            return result
        obj = json.loads(Path(path).read_text(encoding='utf-8-sig'),object_pairs_hook=unique)
        if not isinstance(obj,dict) or set(obj)-{f.name for f in fields(cls)}:
            raise ValueError('Unknown experiment A configuration field')
        return cls(**obj).validate()

    def to_dict(self):
        return asdict(self)
