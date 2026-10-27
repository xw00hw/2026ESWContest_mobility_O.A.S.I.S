"""Verify the integrated seat-control sources with mocks; never open hardware.

    python tools/verify_seat_control.py
    python tools/verify_seat_control.py --model-dir /path/to/exported/model_v8

Needs numpy, pyyaml and pyserial, as deploy/runtime/requirements.txt specifies.
Model weights remain outside Git. A model-free run verifies cold fallback and
reports model-dependent checks as SKIP, not as successful RL inference.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
RL = ROOT / '3_RL_Control'
FB = ROOT / '4_Seat_Actuation'
REQUIRED = (
    '4_Seat_Actuation/experiment_a/transport.py',
    '4_Seat_Actuation/experiment_a/vehicle.py',
    '4_Seat_Actuation/experiment_a/impact.py',
    '4_Seat_Actuation/experiment_a/cad_limits.py',
    '4_Seat_Actuation/fallback/beamng.py',
    '4_Seat_Actuation/stm32/Core/Src/fallback_control.c',
    '3_RL_Control/beamng/pi_relay.py',
    '3_RL_Control/deploy/runtime/oasis_rt/app.py',
)
# These are hardware-independent control-path tests. The separate desktop/UI
# acquisition suite is documented in SEAT_CONTROL_INTEGRATION.md.
EXPERIMENT_TESTS = (
    'test_controller_faults', 'test_controller_geometry', 'test_kinematics',
    'test_tuning', 'test_transport', 'test_vehicle', 'test_impact',
    'test_right_only', 'test_steering_preview', 'test_pressure_logging',
)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--model-dir', type=Path, help='exported model; missing or invalid models fail the model-backed run')
    ap.add_argument('--report', type=Path, help='optional JSON result outside source/data directories')
    args = ap.parse_args(argv)
    missing = [p for p in REQUIRED if not (ROOT / p).is_file()]
    if missing:
        ap.error('missing control sources: ' + ', '.join(missing))
    env = os.environ.copy()
    env['PYTHONDONTWRITEBYTECODE'] = '1'
    env['OASIS_FB_ROOT'] = str(FB)
    # Existing impact fixtures import a sibling test by its top-level name.
    env['PYTHONPATH'] = os.pathsep.join((str(FB), str(FB / 'experiment_a/tests')))
    # Do not accidentally use a model or a package from another checkout.
    if args.model_dir:
        model = args.model_dir.resolve()
        for name in ('manifest.json', 'actor.npz', 'check_set.npy', 'obs_layout.py'):
            if not (model / name).is_file():
                ap.error(f'model-backed check requires {model / name}')
    else:
        model = Path(tempfile.gettempdir()) / 'oasis-verification-no-model'
        if model.exists():
            ap.error(f'model-free sentinel already exists: {model}; use --model-dir explicitly')
    env['OASIS_TEST_MODEL_V3'] = str(model)
    # Both modes validate provenance before executing an exported actor.
    if args.model_dir:
        runtime = RL / 'deploy/runtime'
        sys.path.insert(0, str(runtime))
        from oasis_rt import controller
        cfg = controller.load_config(str(runtime / 'config.yaml'))
        cfg['model_dir'] = str(model)
        cfg['actor']['check_set'] = str(model / 'check_set.npy')
        cfg['actor']['backends'] = ['numpy']
        controller.prepare_policy(cfg, log=lambda *a: None)
    jobs = [
        ('fallback_protocol_and_controller', FB, ['-m', 'unittest', 'discover', '-s', 'fallback/tests', '-q']),
        ('experiment_control_path', FB, ['-m', 'unittest', '-q',
            *[f'experiment_a.tests.{name}' for name in EXPERIMENT_TESTS]]),
        ('pi_collector_to_mock_board', ROOT, [str(RL / 'deploy/runtime/tests/test_app_fb.py')]),
    ]
    try:
        revision = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT,
                                           text=True, stderr=subprocess.DEVNULL).strip()
    except (OSError, subprocess.CalledProcessError):
        revision = None
    result = dict(checked_at=datetime.now(timezone.utc).isoformat(),
                  coverage='model_backed_sil' if args.model_dir else 'cold_fallback_sil',
                  hardware_used=False, source_revision=revision,
                  source_sha256={p: hashlib.sha256((ROOT / p).read_bytes()).hexdigest() for p in REQUIRED},
                  model_dir=str(model) if args.model_dir else None,
                  model_manifest_sha256=hashlib.sha256((model / 'manifest.json').read_bytes()).hexdigest() if args.model_dir else None,
                  suites=[])
    for name, cwd, tail in jobs:
        print(f'Running {name}', flush=True)
        run = subprocess.run([sys.executable, '-B', *tail], cwd=cwd, env=env,
                             capture_output=True, text=True, encoding='utf-8', errors='replace', timeout=180)
        output = run.stdout + run.stderr
        print(output, end='' if output.endswith('\n') else '\n', flush=True)
        result['suites'].append(dict(name=name, returncode=run.returncode, output=output))
    result['ok'] = all(s['returncode'] == 0 for s in result['suites'])
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print('PASS' if result['ok'] else 'FAIL', result['coverage'])
    return 0 if result['ok'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
