"""UDP relay tests use loopback only; no BeamNG process, serial port or physical interface sends.

    python -B deploy/runtime/tests/test_beamng_ipv6.py
"""
import errno
import importlib.util
from pathlib import Path
import select
import socket
import sys
import threading
import time
import unittest

RT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RT))
from oasis_rt import beamng_rx as BR, fbstack  # noqa: E402
from oasis_rt.synthetic import row  # noqa: E402

spec = importlib.util.spec_from_file_location('pi_relay', RT.parents[1] / 'beamng/pi_relay.py')
RELAY = importlib.util.module_from_spec(spec)
spec.loader.exec_module(RELAY)
FB = fbstack.load()
from fallback.beamng_bridge import make_packet  # noqa: E402


def packet(sequence=1):
    r = row(sequence, sequence * 0.02, 0.02, 1.0, 10.0, source_kind='beamng')
    r['host_monotonic_ns'] = time.perf_counter_ns()
    provenance = dict(collector_version='0.3', csv_revision=2, session_id='ipv6_test',
                      config_sha256='a' * 64, source_sha256={'collect.py': 'b' * 64})
    return make_packet(r, provenance), r['host_monotonic_ns']


class RelayNetworkTests(unittest.TestCase):
    def require_ipv6(self):
        if not socket.has_ipv6: self.skipTest('OS has no IPv6 support')
        try:
            with socket.socket(socket.AF_INET6, socket.SOCK_DGRAM) as probe:
                probe.bind(('::1', 0))
        except OSError as exc:
            if exc.errno in (errno.EAFNOSUPPORT, errno.EPROTONOSUPPORT, errno.EADDRNOTAVAIL,
                             errno.ENETDOWN, errno.ENETUNREACH, 10047, 10049, 10050, 10051):
                self.skipTest(f'OS IPv6 loopback unavailable: {exc}')
            raise

    def wait_datagram(self, sock):
        self.assertTrue(select.select([sock], [], [], 2.0)[0], 'no loopback datagram within 2 seconds')

    def relay_roundtrip(self, input_host, output_host):
        if ':' in input_host or ':' in output_host: self.require_ipv6()
        family, endpoint = BR.udp_endpoint(input_host, 0)
        with socket.socket(family, socket.SOCK_DGRAM) as reserve:
            reserve.bind(endpoint); input_port = reserve.getsockname()[1]
        # Use the same wildcard IPv6 binding the Pi uses; a canonical allow entry must accept ::1.
        receiver = BR.RelayReceiver(FB, ('::' if ':' in output_host else output_host, 0),
                                    allow=['0:0:0:0:0:0:0:1' if ':' in output_host else output_host])
        ready, stop = threading.Event(), threading.Event()
        errors, reports = [], []

        def worker():
            try:
                reports.append(RELAY.serve(output_host, receiver.s.getsockname()[1],
                    listen=(input_host, input_port), fb=FB, stop=stop.is_set,
                    log=lambda message: ready.set() if message.startswith('relay ') else None))
            except BaseException as exc:
                errors.append(exc); ready.set()

        thread = threading.Thread(target=worker, daemon=True)
        thread.start()
        try:
            self.assertTrue(ready.wait(2.0), 'relay did not bind')
            if errors: raise errors[0]
            with socket.socket(family, socket.SOCK_DGRAM) as sender:
                destination = BR.udp_endpoint(input_host, input_port)[1]
                payload, _ = packet()
                sender.sendto(payload, destination)
                self.wait_datagram(receiver.s)
                forwarded, _ = receiver.s.recvfrom(BR.MAX_DATAGRAM)
                age, sequence, actual = BR.relay_unwrap(forwarded)
                self.assertEqual(actual, payload)
                self.assertEqual(sequence, 1)
                self.assertGreaterEqual(age, 0)
                payload, stamp = packet(2)
                sender.sendto(payload, destination)
                self.wait_datagram(receiver.s)
                received = receiver.receive()
                self.assertEqual(received['_pc_host_monotonic_ns'], stamp)
                self.assertEqual(received['_bridge_session_id'], 'ipv6_test')
                self.assertEqual(receiver.foreign, 0)
                self.assertLessEqual(received['host_monotonic_ns'], time.monotonic_ns())
        finally:
            stop.set(); thread.join(2.0); receiver.close()
        self.assertFalse(thread.is_alive(), 'relay failed to stop')
        self.assertFalse(errors, errors)
        self.assertEqual(reports, [dict(forwarded=2, invalid=0, send_drops=0)])

    def test_ipv4_relay_regression(self): self.relay_roundtrip('127.0.0.1', '127.0.0.1')
    def test_ipv4_collector_to_ipv6_pi(self): self.relay_roundtrip('127.0.0.1', '::1')
    def test_explicit_ipv6_collector_listener(self): self.relay_roundtrip('::1', '::1')

    def test_scoped_ipv6_destination_resolution(self):
        self.require_ipv6()
        interfaces = socket.if_nameindex()
        self.assertTrue(interfaces, 'IPv6 is available but no interfaces are enumerated')
        index, name = interfaces[0]
        for zone in (str(index), name):
            with self.subTest(zone=zone):
                family, endpoint = BR.udp_endpoint(f'[fe80::1234%{zone}]', 64257)
                self.assertEqual(family, socket.AF_INET6)
                self.assertEqual(endpoint, ('fe80::1234', 64257, 0, index))

    def test_peer_ip_normalization_is_exact(self):
        self.assertEqual(BR.peer_ip('fe80:0:0:0:88a2:9eff:fef0:bf29%12'), 'fe80::88a2:9eff:fef0:bf29')
        self.assertNotEqual(BR.peer_ip('127.0.0.2'), BR.peer_ip('127.0.0.1'))
        self.assertNotEqual(BR.peer_ip('::ffff:127.0.0.1'), BR.peer_ip('127.0.0.1'))

    def test_ipv6_allowlist_header_and_freshness_gates(self):
        self.require_ipv6()
        receiver = BR.RelayReceiver(FB, ('::1', 0), allow=['::2'])
        try:
            with socket.socket(socket.AF_INET6, socket.SOCK_DGRAM) as sender:
                destination = receiver.s.getsockname()
                payload, _ = packet()
                sender.sendto(BR.relay_wrap(payload, 0, 1), destination)
                self.wait_datagram(receiver.s)
                self.assertIsNone(receiver.receive())
                self.assertEqual(receiver.foreign, 1)
                receiver.allow = {BR.peer_ip('0:0:0:0:0:0:0:1')}
                sender.sendto(b'garbage', destination)
                self.wait_datagram(receiver.s)
                with self.assertRaisesRegex(ValueError, 'OASIS-RELAY1'):
                    receiver.receive()
                sender.sendto(BR.relay_wrap(payload, 60_000_000, 2), destination)
                self.wait_datagram(receiver.s)
                self.assertEqual(receiver.receive()['derivative_reason'], 'pc_forward_delay')
        finally:
            receiver.close()

    def test_ipv4_relay_rejects_other_loopback_address(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
            receiver.bind(('127.0.0.1', 0))
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reserve:
                reserve.bind(('127.0.0.1', 0)); port = reserve.getsockname()[1]
            ready, stop = threading.Event(), threading.Event()
            reports, errors = [], []
            def worker():
                try:
                    reports.append(RELAY.serve('127.0.0.1', receiver.getsockname()[1], listen=('127.0.0.1', port),
                        fb=FB, stop=stop.is_set, log=lambda message: ready.set()))
                except BaseException as exc: errors.append(exc); ready.set()
            thread = threading.Thread(target=worker, daemon=True); thread.start()
            try:
                self.assertTrue(ready.wait(2.0))
                if errors: raise errors[0]
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
                    sender.bind(('127.0.0.2', 0))
                    sender.sendto(packet()[0], ('127.0.0.1', port))
                    self.assertFalse(select.select([receiver], [], [], 0.3)[0])
            finally:
                stop.set(); thread.join(2.0)
            self.assertFalse(thread.is_alive())
            self.assertFalse(errors, errors)
            self.assertEqual(reports, [dict(forwarded=0, invalid=1, send_drops=0)])


if __name__ == '__main__':
    unittest.main()
