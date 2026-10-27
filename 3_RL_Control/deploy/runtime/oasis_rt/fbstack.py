"""Import the BeamNG_Bolster_FB host modules the Pi runtime reuses unchanged.

ESW repo: 4_Seat_Actuation/{experiment_a, fallback} (branch BeamNG_Bolster_FB; main has A V7, 28be7eb,
on top of V6 6bcf848). Used here:
  experiment_a.transport  BoardLink: OASIS_FALLBACK v2 UART (one outstanding request, exact ACKs, explicit session ARM)
  experiment_a.vehicle    VehicleValidator / VehicleFrame: collector v0.3 rows, quality and stream gates
  experiment_a.cad_limits CAD envelope shared with the firmware (profile sha256 564128a1...)
  experiment_a.kinematics C4_R8 IK, 270 deg / 2000 us pulse map
  experiment_a.controller / settings / tuning   vehicle-only controller (control.mode fb_v6, cold fallback; V6 law)
  experiment_a.impact     V7 planar shock heuristic, run on every row in all modes (absent in V6: None, and the V6
                          VehicleValidator stops on a single |a_y| > 12 / |a_x| > 15 m/s^2 sample instead)
  fallback.beamng         bridge envelope (schema oasis.beamng.fallback.v1) parser

Search order: $OASIS_FB_ROOT, config `fb_root`, then a `4_Seat_Actuation` folder in any parent of this
file (repo: <ESW>/4_Seat_Actuation, 3 levels above deploy/runtime).
"""
import os, sys
from types import SimpleNamespace

NAME = '4_Seat_Actuation'


def root(hint=None):
    cands = [os.environ.get('OASIS_FB_ROOT'), hint]
    d = os.path.dirname(os.path.realpath(__file__))          # realpath: works through a symlinked checkout
    for _ in range(8):
        d = os.path.dirname(d); cands.append(os.path.join(d, NAME))
    for c in cands:
        if c and os.path.isfile(os.path.join(c, 'experiment_a', 'transport.py')):
            return os.path.abspath(c)
    raise RuntimeError(f'{NAME} (BeamNG_Bolster_FB host modules) not found; set OASIS_FB_ROOT or fb_root')


def load(hint=None):
    r = root(hint)
    if r not in sys.path: sys.path.insert(0, r)
    from experiment_a import cad_limits, controller, kinematics, session_logging, settings, transport, tuning, vehicle
    from fallback import beamng, serial_link
    try:
        from experiment_a import impact
    except ImportError:
        impact = None
    return SimpleNamespace(root=r, cad_limits=cad_limits, controller=controller, kinematics=kinematics,
                           settings=settings, transport=transport, tuning=tuning, vehicle=vehicle,
                           beamng=beamng, serial_link=serial_link, impact=impact, session_logging=session_logging)
