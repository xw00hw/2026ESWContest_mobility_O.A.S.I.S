"""Inspect and replay by default; hardware requires an explicit run subcommand."""
import argparse
import json

from .settings import Settings


def main():
    parser=argparse.ArgumentParser(description='Experiment A: BeamNG-only model feedforward')
    parser.add_argument('--config',help='Reviewed experiment configuration JSON')
    sub=parser.add_subparsers(dest='command')
    sub.add_parser('inspect')
    replay=sub.add_parser('replay',help='Offline CSV, no device or network')
    replay.add_argument('--vehicle-csv',required=True);replay.add_argument('--out',required=True)
    replay.add_argument('--plot',action='store_true')
    run=sub.add_parser('run',help='Explicit hardware session after physical debugging')
    run.add_argument('--port',required=True);run.add_argument('--out',required=True)
    run.add_argument('--arm',action='store_true')
    run.add_argument('--record-pressure',action='store_true',help='Log 12 FSR channels for evaluation only')
    args=parser.parse_args()
    settings=Settings.load(args.config) if args.config else Settings().validate()
    if args.command in (None,'inspect'):
        print(json.dumps(dict(settings=settings.to_dict(),response_model='body_independent_vehicle_vector',
            missing_hardware_confirmations=settings.missing_confirmations(),runtime_input='BeamNG vehicle data only',
            pressure_feedback=False,observed_pose_feedback=False,
            physical_effect_verified=False),indent=2,ensure_ascii=False))
    elif args.command=='replay':
        from .replay import replay_vehicle
        print(json.dumps(replay_vehicle(args.vehicle_csv,args.out,settings,args.plot),indent=2,ensure_ascii=False))
    elif args.command=='run':
        if not args.config or not args.arm:
            parser.error('run requires explicit --config and --arm; no device opened')
        from .runner import run_live
        run_live(args.port,args.out,settings,record_pressure=args.record_pressure)


if __name__=='__main__':main()
