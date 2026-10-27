"""PC side of the BeamNG link: forward BeamNG_Bolster_FB's collector bridge to the Raspberry Pi.

    BeamNG.drive -> BeamNG Collector v0.3 (UDP 64256, raw archive; external, unchanged)
      python -m fallback.beamng_bridge --collector-root <BeamNG_Collector_v0_3>     (4_Seat_Actuation/)
        -> 127.0.0.1:64257, one FB bridge datagram per collector row (schema oasis.beamng.fallback.v1)
      python beamng/pi_relay.py --pi <PI_IP>                                         (this file)
        -> <PI_IP>:64257, b'OASIS-RELAY1 <pc_age_ns> <relay_seq>\\n' + the datagram, byte-identical

pc_age_ns = relay send time - collector receipt time (host_monotonic_ns), both perf_counter_ns on THIS PC, so the
Pi (oasis_rt.beamng_rx) can keep BeamNG_Bolster_FB's 200 ms freshness gate on its own clock. Datagrams that fail
fallback.beamng.read_bridge_packet are counted and not forwarded (the Pi validates every row again).
The relay owns 127.0.0.1:64257 exclusively: do not run the Experiment A desktop app at the same time.
Ctrl+C stops it; it never touches the collector, the game or the board.
"""
import argparse, os, socket, sys, time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'deploy', 'runtime'))
from oasis_rt import fbstack  # noqa: E402
from oasis_rt.beamng_rx import peer_ip, relay_wrap, udp_endpoint  # noqa: E402


def serve(pi, port=64257, listen=('127.0.0.1', 64257), stats_s=5.0, clock_ns=time.perf_counter_ns, fb=None, stop=None, log=print):
    fb = fb or fbstack.load()
    rx_family, rx_address = udp_endpoint(*listen)
    tx_family, destination = udp_endpoint(pi, port)
    loopback = '::1' if rx_family == socket.AF_INET6 else '127.0.0.1'
    rx = socket.socket(rx_family, socket.SOCK_DGRAM); tx = None
    n = bad = drop = seq = 0; ages = []; t_next = time.monotonic() + stats_s; t0 = time.monotonic()
    try:
        if rx_family == socket.AF_INET6: rx.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 1)
        if hasattr(socket, 'SO_EXCLUSIVEADDRUSE'): rx.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        rx.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 16); rx.bind(rx_address); rx.settimeout(0.2)
        tx = socket.socket(tx_family, socket.SOCK_DGRAM); tx.setblocking(False)
        log(f'relay {rx.getsockname()} -> {destination}')
        while stop is None or not stop():
            try:
                data, peer = rx.recvfrom(fb.beamng.MAX_DATAGRAM_BYTES + 1)
            except socket.timeout:
                data = None
            if data is not None:
                now = clock_ns()
                try:
                    if peer_ip(peer[0]) != loopback: raise ValueError('bridge peer is not loopback')
                    row = fb.beamng.read_bridge_packet(data)
                    age = max(0, now - int(row['host_monotonic_ns']))
                except (ValueError, KeyError) as e:
                    bad += 1
                    if bad <= 5: log(f'dropped invalid bridge datagram: {e}')
                    continue
                seq += 1
                try:
                    tx.sendto(relay_wrap(data, age, seq), destination); n += 1; ages.append(age / 1e6)
                except (BlockingIOError, OSError):
                    drop += 1
            if time.monotonic() >= t_next:
                a = sorted(ages[-500:]) or [0.0]
                log(f'forwarded {n} ({n / (time.monotonic() - t0):.0f}/s), invalid {bad}, send drops {drop}, '
                    f'PC age p50 {a[len(a) // 2]:.1f} ms p95 {a[int(0.95 * (len(a) - 1))]:.1f} ms')
                t_next += stats_s
    except KeyboardInterrupt:
        pass
    finally:
        rx.close()
        if tx is not None: tx.close()
    return dict(forwarded=n, invalid=bad, send_drops=drop)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--pi', required=True, help='Raspberry Pi IPv4/IPv6 address (link-local example: fe80::1234%%12)')
    ap.add_argument('--port', type=int, default=64257, help='Pi UDP port (config beamng.listen)')
    ap.add_argument('--listen', default='127.0.0.1:64257', help='FB bridge output; IPv4 default 127.0.0.1:64257, explicit IPv6 [::1]:64257')
    ap.add_argument('--stats', type=float, default=5.0)
    a = ap.parse_args(argv)
    h, p = a.listen.rsplit(':', 1)
    print(serve(a.pi, a.port, (h, int(p)), a.stats))


if __name__ == '__main__':
    main()
