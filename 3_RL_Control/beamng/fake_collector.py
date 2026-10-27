"""Stand-in for BeamNG + Collector v0.3 + fallback.beamng_bridge when the game is not available: synthetic vehicle
rows (deploy/runtime/oasis_rt/synthetic.py route: straight / 60 deg left / right / 45 deg slalom at 14 m/s) wrapped
in the FB bridge envelope (fallback.beamng_bridge.make_packet) and sent to 127.0.0.1:64257, i.e. into pi_relay.py.

    python beamng/fake_collector.py --seconds 120          (PC: then pi_relay.py --pi <PI_IP>; Pi: beamng.allow_synthetic true)

Rows carry source_kind = 'synthetic_test': the Pi refuses them unless beamng.allow_synthetic is true, so they cannot
be mistaken for a BeamNG drive. For network / Pi / board commissioning only (no vehicle dynamics claim).
"""
import argparse, os, socket, sys, time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'deploy', 'runtime'))
from oasis_rt import fbstack  # noqa: E402
from oasis_rt.synthetic import SyntheticCar, drive, row  # noqa: E402


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--seconds', type=float, default=120.0); ap.add_argument('--rate', type=float, default=50.0)
    ap.add_argument('--to', default='127.0.0.1:64257'); ap.add_argument('--steer_sign', type=int, default=1)
    a = ap.parse_args(argv)
    fb = fbstack.load()
    from fallback.beamng_bridge import make_packet
    prov = dict(collector_version='0.3', csv_revision=2, session_id='fake_collector', config_sha256='0' * 64,
                source_sha256={'fake_collector.py': '0' * 64})
    host, port = a.to.rsplit(':', 1); tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    car, dt = SyntheticCar(), 1.0 / a.rate; t0 = time.perf_counter(); k = 0
    while k * dt < a.seconds:
        due = t0 + k * dt
        while time.perf_counter() < due: time.sleep(0.0005)
        t = k * dt; sw = drive(t); ay = car.step(sw, dt); k += 1
        r = row(k, t, dt, ay, a.steer_sign * sw); r['host_monotonic_ns'] = time.perf_counter_ns()
        tx.sendto(make_packet(r, prov), (host, int(port)))
        if k % int(5 * a.rate) == 0: print(f't {t:6.1f} s  rows {k}  a_y {ay:+.2f} m/s^2  wheel {sw:+.0f} deg', flush=True)
    print('sent', k, 'rows')


if __name__ == '__main__':
    main()
