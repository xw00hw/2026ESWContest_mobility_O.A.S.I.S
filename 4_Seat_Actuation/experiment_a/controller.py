"""Vehicle-only model feedforward with coupled R8 command constraints.

No pressure, observed body pose, contact estimate, or learned policy is accepted.
Board state is used only to verify command authority/bounds and neutral passage;
it is commanded geometry telemetry, not a measurement of the person's body.
"""
from dataclasses import asdict, dataclass, field
import math

from .vehicle_response import VehicleResponse, GRAVITY_MPS2
from .alignment import closest_face_pitch_deg
from .kinematics import allowable_pitch_for_yaw, pose_to_pulses_us
from .settings import Settings
from .steering_preview import SteeringPreview
from .cad_limits import pose_within_cad, CAD_YAW_MAX_MDEG
from .impact import ImpactDetector, MAX_ABS_ACCEL_MPS2


@dataclass(frozen=True)
class Decision:
    mode: str
    reason: str
    pose_mdeg: tuple[int,int,int,int] = (0,0,0,0)
    permit_lease: bool = False
    emergency_off: bool = False
    diag: dict = field(default_factory=dict)


class ExperimentAController:
    def __init__(self, settings: Settings):
        self.settings = settings.validate()
        self.model = VehicleResponse(settings.response_filter_tau_s)
        self.predicted = self.model.reset()
        self.steering_preview = SteeringPreview(settings)
        self.preview_state = self.steering_preview.state
        self.impact_detector = ImpactDetector(settings)
        self.target_source = 'actual_acceleration'
        self.pose = [0.0]*4
        self.last_tick_ns = None
        self.last_source = None
        self.fault = None
        self.active_side = 'none'
        self.trigger_side = 'none'
        self.reversing = False

    def fail(self, reason):
        self.fault = reason
        return Decision('FAULT',reason,emergency_off=True,
                        diag={'impact': self.impact_detector.state.to_dict()})

    def _fresh(self, stamp, now):
        return type(stamp) is int and 0 <= now-stamp <= self.settings.fresh_ms*1e6

    def _board_neutral(self, state):
        return (not state.moving and all(abs(x)<=self.settings.neutral_tolerance_mdeg for x in state.pose_mdeg)
                and all(x==0 for x in state.target_mdeg))

    def _advance_model(self, vehicle):
        key = (vehicle.stream_id,vehicle.vehicle_id,vehicle.sequence_id,
               vehicle.received_monotonic_ns,vehicle.sim_time_s)
        old = self.last_source
        if old == key:
            return  # Held latest input is never integrated twice.
        if old is not None:
            if key[:2] != old[:2]:
                raise ValueError('model_stream_changed')
            if key[2] <= old[2] or key[3] <= old[3]:
                raise ValueError('model_source_reordered')
            dt = key[4]-old[4]
            if not 0 < dt <= self.settings.max_model_sample_dt_s+1e-10:
                raise ValueError('model_sim_time_gap')
        else:
            dt = 0.0
        if self.settings.acceleration_source == 'specific_force':
            ax = getattr(vehicle,'specific_force_x_mps2',None)
            ay = getattr(vehicle,'specific_force_y_mps2',None)
        else:
            ax,ay = vehicle.ax_mps2,vehicle.ay_mps2
        if any(isinstance(x,bool) or not isinstance(x,(int,float)) or not math.isfinite(x) for x in (ax,ay)):
            raise ValueError('selected_acceleration_missing_or_invalid')
        if max(abs(ax),abs(ay))>MAX_ABS_ACCEL_MPS2:
            raise ValueError('selected_acceleration_outlier')
        impact = self.impact_detector.observe(vehicle)
        if impact.triggered:
            raise ValueError(impact.reason)
        if vehicle.reason == 'stopped':
            self.predicted = self.model.reset()
        elif dt:
            # Simulation time drives the vehicle response filter. Host time drives
            # actuator rates/leases. Current sample is held over this bounded
            # observed interval; there is no future-row interpolation.
            count = max(1,math.ceil(dt/.05))
            for _ in range(count):
                self.predicted = self.model.step(ax,ay,dt/count)
        source_elapsed = 0.0 if old is None else (key[3]-old[3])/1e9
        self.preview_state = self.steering_preview.step(vehicle, source_elapsed,
            raw_ay_mps2=ay, filtered_ay_mps2=self.predicted.filtered_ay_mps2,
            primary_active=self.trigger_side != 'none')
        self.last_source = key

    def _target(self, stopped):
        s,p = self.settings,self.predicted
        ay = p.filtered_ay_mps2
        direction = 'right' if ay>0 else 'left'
        if stopped or abs(ay)<=s.off_ay_mps2:
            self.trigger_side = 'none'
        elif abs(ay)>=s.on_ay_mps2:
            self.trigger_side = direction
        elif direction != self.trigger_side:
            self.trigger_side = 'none'
        roll = p.preview_roll_deg
        expected_direction = 'right' if roll>0 else 'left'
        desired_side = self.trigger_side
        if desired_side != expected_direction:
            desired_side = 'none'  # No old-direction support across a reversal.
        if s.available_bolsters == 'right_only' and desired_side == 'left':
            desired_side = 'none'  # Do not command or await a physically absent side.
        fraction = max(0.0,min(1.0,(abs(ay)-s.off_ay_mps2)/
                            (s.full_scale_ay_mps2-s.off_ay_mps2)))
        if desired_side == 'none':
            fraction = 0.0
        yaw = fraction*s.max_yaw_deg
        pitch_sign = -1 if s.pitch_mode=='counter_lean' else 1
        # At fixed Yaw, maximize the pad-normal alignment to the predicted
        # reference side plane derived from atan2(filtered_ay, g). This is a
        # vehicle cue, not a measured/predicted human surface or force command.
        alignment_pitch = closest_face_pitch_deg(yaw,abs(roll)*s.pitch_per_predicted_roll)
        pitch = pitch_sign*min(abs(alignment_pitch),allowable_pitch_for_yaw(yaw,s.max_pitch_deg))
        requested = (yaw,pitch,0.,0.) if desired_side=='left' else (
                    (0.,0.,yaw,pitch) if desired_side=='right' else (0.,0.,0.,0.))
        self.target_source = 'actual_acceleration'
        if desired_side == 'none' and not stopped and self.preview_state.active:
            desired_side = self.preview_state.side
            yaw = self.preview_state.yaw_deg
            requested = (yaw,0.,0.,0.) if desired_side == 'left' else (0.,0.,yaw,0.)
            fraction = yaw/s.max_yaw_deg
            self.target_source = 'steering_advance'
        return desired_side,fraction,requested

    def _slew(self, target, dt):
        before = pose_to_pulses_us(self.pose)
        delta = [b-a for a,b in zip(self.pose,target)]
        rates = [self.settings.max_yaw_rate_deg_s,self.settings.max_pitch_rate_deg_s]*2
        alpha = min([1.0]+[rate*dt/abs(d) for d,rate in zip(delta,rates) if d])
        pulse_budget = self.settings.max_pulse_rate_us_s*dt
        def candidate(fraction):
            return [a+fraction*d for a,d in zip(self.pose,delta)]
        def within(fraction):
            pose = candidate(fraction)
            # Do not assume a straight line inside an arbitrary coupled set.
            if any(abs(pose[i+1])>allowable_pitch_for_yaw(pose[i],self.settings.max_pitch_deg)+1e-9 for i in (0,2)):
                return False
            return max(abs(a-b) for a,b in zip(pose_to_pulses_us(pose),before)) <= pulse_budget+1e-10
        limited = not within(alpha)
        if limited:
            low,high = 0.0,alpha
            for _ in range(40):
                mid = (low+high)/2
                if within(mid): low = mid
                else: high = mid
            alpha = low
        self.pose = candidate(alpha)
        # Avoid perpetually tiny tails at neutral.
        for i in range(4):
            if abs(self.pose[i])<1e-10:
                self.pose[i]=0.0
        output = [round(x*1000) for x in self.pose]
        for i in (0,2):
            cap = math.floor(allowable_pitch_for_yaw(output[i]/1000,self.settings.max_pitch_deg)*1000+1e-8)
            output[i+1] = max(-cap,min(cap,output[i+1]))
        return tuple(output),limited

    def tick(self, vehicle, state, now_ns):
        if self.fault:
            return self.fail(self.fault)
        if type(now_ns) is not int or now_ns<0:
            return self.fail('invalid_host_clock')
        dt = 0.0 if self.last_tick_ns is None else (now_ns-self.last_tick_ns)/1e9
        if self.last_tick_ns is not None and not 0<dt<=.1:
            return self.fail('host_loop_gap_or_reversal')
        if state is None or not self._fresh(state.received_ns,now_ns):
            return self.fail('board_state_stale')
        if not state.armed or state.fault!='NONE':
            return self.fail('board_not_armed_or_fault')
        for angles in (state.pose_mdeg,state.target_mdeg):
            if len(angles)!=4 or any(type(x) is not int for x in angles):
                return self.fail('invalid_board_pose')
            if self.settings.available_bolsters == 'right_only' and any(angles[:2]):
                return self.fail('unavailable_left_bolster_nonzero_command_state')
            if not pose_within_cad(angles):
                return self.fail('board_pose_outside_CAD_envelope')
            if any(not 0<=angles[i]<=self.settings.max_yaw_deg*1000+100 or
                   abs(angles[i+1])>self.settings.max_pitch_deg*1000+100 for i in (0,2)):
                return self.fail('board_pose_outside_experiment_bounds')
            if any(abs(angles[i+1]) > allowable_pitch_for_yaw(
                    min(CAD_YAW_MAX_MDEG/1000,angles[i]/1000),self.settings.max_pitch_deg)*1000+100 for i in (0,2)):
                return self.fail('board_pose_outside_coupled_pitch_envelope')
        if self.last_tick_ns is None and not self._board_neutral(state):
            return self.fail('nonneutral_initial_state')
        if vehicle is None or not self._fresh(vehicle.received_monotonic_ns,now_ns):
            return self.fail('vehicle_stale')
        if not(vehicle.valid or vehicle.reason=='stopped'):
            return self.fail('vehicle:'+vehicle.reason)
        try:
            self._advance_model(vehicle)
            wanted_side,fraction,requested = self._target(vehicle.reason=='stopped')
            if self.active_side not in ('none',wanted_side) and wanted_side!='none':
                self.reversing = True
            if self.reversing:
                if all(abs(x)<1e-9 for x in self.pose) and self._board_neutral(state):
                    self.reversing = False
                    self.active_side = 'none'
                else:
                    requested = (0.,0.,0.,0.)
            if not self.reversing:
                if wanted_side!='none':
                    self.active_side = wanted_side
                elif all(abs(x)<1e-9 for x in self.pose) and self._board_neutral(state):
                    self.active_side='none'
            output,pulse_limited = self._slew(requested,min(dt,.05))
        except (ValueError,OverflowError) as exc:
            return self.fail(str(exc))
        self.last_tick_ns = now_ns
        reason = 'returning_neutral_before_side_change' if self.reversing else (
                 'stopped_neutral' if vehicle.reason=='stopped' else (
                 'steering_advance' if self.target_source == 'steering_advance' else 'vehicle_only_feedforward'))
        if (vehicle.reason != 'stopped' and self.settings.available_bolsters == 'right_only'
                and self.trigger_side == 'left'):
            reason = 'unavailable_left_bolster_return_neutral'
        return Decision('RUN',reason,output,True,False,dict(
            vehicle_response=asdict(self.predicted),
            impact=self.impact_detector.state.to_dict(),
            steering_preview=self.preview_state.to_dict(), target_source=self.target_source,
            active_side=self.active_side,demand_fraction=fraction,
            available_bolsters=self.settings.available_bolsters,
            requested_support_side=(self.preview_state.side if self.target_source == 'steering_advance' else self.trigger_side),
            support_side_available=(self.settings.available_bolsters == 'both' or
                (self.preview_state.side if self.target_source == 'steering_advance' else self.trigger_side) != 'left'),
            requested_pose_deg=list(requested),pulse_slew_limited=pulse_limited,
            pulse_command_float_us=list(pose_to_pulses_us(self.pose)),
            acceleration_vector_reference_deg=math.degrees(math.atan2(self.settings.full_scale_ay_mps2,GRAVITY_MPS2)),
            pitch_limits_at_requested_yaw_deg=[allowable_pitch_for_yaw(requested[i],self.settings.max_pitch_deg) for i in (0,2)],
            alignment_frame=self.settings.metadata.get('alignment_frame'),
            acceleration_source=self.settings.acceleration_source,
            runtime_input='BeamNG_only',pressure_used=False,observed_body_pose_used=False,
            body_compensation_used=False,human_angle_predicted=False))
