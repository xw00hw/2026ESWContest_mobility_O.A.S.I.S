"""Explicit subcommands: inspecting/replaying never connects to a device."""
import argparse
import json

from .profile import BenchProfile, DEFAULT_PROFILE, Settings


def main():
    parser = argparse.ArgumentParser(description="BeamNG pressure-limited fallback; no camera/RL")
    parser.add_argument("--profile", default=str(DEFAULT_PROFILE))
    parser.add_argument("--config", help="Explicit physical mapping/confirmation JSON")
    sub = parser.add_subparsers(dest="command")
    sub.add_parser("inspect", help="Print evidence, candidate limits and missing calibration")
    replay = sub.add_parser("replay", help="Offline vehicle-only demand analysis; no serial/socket")
    replay.add_argument("--vehicle-csv", required=True)
    replay.add_argument("--out", required=True)
    replay.add_argument("--plot", action="store_true")
    run = sub.add_parser("run", help="Explicit hardware session, after separate firmware upload/calibration")
    run.add_argument("--port", required=True)
    run.add_argument("--arm", action="store_true", help="Deliberate consent to neutral PWM and subsequent support")
    run.add_argument("--out", required=True)
    args = parser.parse_args()
    settings = Settings.load(args.config) if args.config else Settings()
    profile = BenchProfile(args.profile)
    if args.command in (None, "inspect"):
        print(json.dumps(profile.derive(settings).report(), indent=2, ensure_ascii=False))
    elif args.command == "replay":
        from .replay import replay_vehicle
        report = replay_vehicle(args.vehicle_csv, args.out, settings, profile, args.plot)
        print(json.dumps(report, indent=2, ensure_ascii=False))
    elif args.command == "run":
        if not args.arm or not args.config:
            parser.error("run requires --arm and an explicitly reviewed --config; no device was opened")
        from .runner import run_live
        run_live(args.port, args.out, settings, profile)


if __name__ == "__main__":
    main()
