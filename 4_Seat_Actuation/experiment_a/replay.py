"""Offline command replay with ideal acknowledgement, never hardware/body validation."""
from collections import Counter
from dataclasses import asdict
import csv
import hashlib
import json
from pathlib import Path

from fallback.serial_link import BoardState
from .controller import ExperimentAController
from .kinematics import pose_to_pulses_us
from .settings import Settings
from .vehicle import VehicleValidator, read_vehicle_csv
from .impact import ImpactDetector


def replay_vehicle(vehicle_csv, output_dir, settings=None, plot=False):
    settings = (settings or Settings()).validate()
    output = Path(output_dir)
    output.mkdir(parents=True,exist_ok=True)
    csv_path, report_path = output/'commands.csv',output/'report.json'
    if csv_path.exists() or report_path.exists():
        raise FileExistsError('Choose a new replay output directory')
    validator = VehicleValidator(max_age_ms=settings.fresh_ms,max_host_gap_ms=settings.fresh_ms,
                                 max_sim_dt_s=settings.max_model_sample_dt_s)
    controller = None
    impact_detector = ImpactDetector(settings)
    first_impact = None
    pose = (0,0,0,0)
    ready_since = first = None
    reasons,policy_reasons = Counter(),Counter()
    total=windows=0
    peaks=[0.0]*4
    model_peak=0.0
    plot_rows=[]
    columns=['host_monotonic_ns','elapsed_s','sim_time_s','stream_id','vehicle_id','sequence_id',
             'input_valid','input_reason','analysis_window','policy_reason','ay_mps2',
             'vehicle_lateral_vector_angle_deg','vehicle_longitudinal_vector_angle_deg',
             'desired_LY_deg','desired_LP_deg','desired_RY_deg','desired_RP_deg',
             'command_LY_deg','command_LP_deg','command_RY_deg','command_RP_deg',
             'pulse_slew_limited','impact_suspected','impact_reason',
             'planar_acceleration_mps2','impact_high_duration_s','impact_vector_delta_v_mps',
             'hardware_output_enabled']
    with csv_path.open('x',encoding='utf-8',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=columns);writer.writeheader()
        for raw in read_vehicle_csv(vehicle_csv):
            now=int(raw['host_monotonic_ns'])
            frame=validator.parse(raw,now)
            impact=impact_detector.observe(frame)
            if impact.triggered and first_impact is None:
                first_impact=dict(host_monotonic_ns=now,**impact.to_dict())
            if first is None:first=now
            total+=1;reasons[frame.reason]+=1
            good=frame.valid or frame.reason=='stopped'
            if settings.acceleration_source=='specific_force':
                good=good and frame.specific_force_x_mps2 is not None and frame.specific_force_y_mps2 is not None
            if not good or impact.triggered:
                controller=None;ready_since=None;pose=(0,0,0,0)
            elif ready_since is None:
                ready_since=now
            decision=None
            if good and not impact.triggered and now-ready_since>=settings.startup_seconds*1e9:
                if controller is None:
                    controller=ExperimentAController(settings);windows+=1
                board=BoardState(now,True,False,pose,pose,
                                 tuple(round(x) for x in pose_to_pulses_us([x/1000 for x in pose])),
                                 total,'NONE')
                decision=controller.tick(frame,board,now)
                if decision.emergency_off:
                    controller=None;ready_since=None;pose=(0,0,0,0)
                else:pose=decision.pose_mdeg
            diag=decision.diag if decision else {}
            prediction=diag.get('vehicle_response',{})
            requested=diag.get('requested_pose_deg',[0.]*4)
            reason=('impact_record_only' if impact.triggered else decision.reason if decision else
                    ('qualifying_input' if good else 'invalid_input_reset'))
            policy_reasons[reason]+=1
            for i,v in enumerate(pose):peaks[i]=max(peaks[i],abs(v)/1000)
            model_peak=max(model_peak,abs(prediction.get('roll_deg',0.)))
            row=dict(host_monotonic_ns=now,elapsed_s=(now-first)/1e9,sim_time_s=frame.sim_time_s,
                     stream_id=frame.stream_id,vehicle_id=frame.vehicle_id,sequence_id=frame.sequence_id,
                     input_valid=int(good),input_reason=frame.reason,analysis_window=windows,
                     policy_reason=reason,ay_mps2=frame.ay_mps2,
                     vehicle_lateral_vector_angle_deg=prediction.get('roll_deg'),
                     vehicle_longitudinal_vector_angle_deg=prediction.get('pitch_deg'),
                     pulse_slew_limited=int(diag.get('pulse_slew_limited',False)),
                     impact_suspected=int(impact.triggered),impact_reason=impact.reason,
                     planar_acceleration_mps2=impact.planar_mps2,
                     impact_high_duration_s=impact.high_duration_s,
                     impact_vector_delta_v_mps=impact.vector_delta_v_mps,
                     hardware_output_enabled=0)
            for i,k in enumerate(('LY','LP','RY','RP')):
                row['desired_'+k+'_deg']=requested[i];row['command_'+k+'_deg']=pose[i]/1000
            writer.writerow(row)
            if plot:plot_rows.append(row)
    report=dict(schema='oasis-experiment-a-offline-replay-v1',rows=total,input_reasons=dict(reasons),
                policy_reasons=dict(policy_reasons),independent_analysis_windows=windows,
                peak_absolute_command_deg=dict(zip(('LY','LP','RY','RP'),peaks)),
                peak_vehicle_lateral_vector_angle_deg=model_peak,
                source=str(Path(vehicle_csv).resolve()),source_sha256=hashlib.sha256(Path(vehicle_csv).read_bytes()).hexdigest(),
                settings=asdict(settings),settings_sha256=hashlib.sha256(json.dumps(asdict(settings),sort_keys=True).encode()).hexdigest(),
                code_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in Path(__file__).parent.glob('*.py')},
                hardware_commands_sent=0,pressure_or_pose_input_used=False,
                impact_event=first_impact,
                interpretation='Ideal immediate command acknowledgement only; no actuator dynamics, contact force or achieved human pose simulated. Other fault-separated replay windows restart only for analysis; impact stays record-only for the rest of the input. Zero commands after impact denote OFF, not physical return to neutral.')
    report_path.write_text(json.dumps(report,indent=2,ensure_ascii=False,allow_nan=False)+'\n',encoding='utf-8')
    if plot and plot_rows:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,axes=plt.subplots(4,1,figsize=(12,9),sharex=True,constrained_layout=True)
        ts=[r['elapsed_s'] for r in plot_rows]
        for ax,keys,label in zip(axes,
            [('ay_mps2',),('vehicle_lateral_vector_angle_deg',),('command_LY_deg','command_RY_deg'),('command_LP_deg','command_RP_deg')],
            ['Vehicle ay (m/s²)','Vehicle vector angle (deg)','Yaw command (deg)','Pad Pitch command (deg)']):
            for k in keys:ax.plot(ts,[float('nan') if r[k] is None else r[k] for r in plot_rows],lw=.8,label=k)
            ax.set_ylabel(label);ax.legend(loc='upper right',fontsize=7);ax.grid(alpha=.2)
        axes[-1].set_xlabel('Recorded host elapsed time (s)')
        fig.suptitle('Experiment A: OFFLINE vehicle-only replay / ideal command acknowledgement')
        fig.savefig(output/'replay.png',dpi=150);plt.close(fig)
    return report
