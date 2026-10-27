"""MOZA wire/clock/calibration/lifecycle tests; no real socket or HID is opened."""
from collections import deque
import importlib.util
import json
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from oasis_rt import wheel_input as W

PC = 'fe80::52a:153f:fd2b:1a0d'


class Clock:
    def __init__(self, value=1_000_000_000): self.value = value
    def __call__(self): return self.value


class Journal:
    def __init__(self): self.rows = []; self.failure = None
    def record(self, event, **fields):
        if self.failure: raise self.failure
        self.rows.append(dict(event=event, **fields))


class Transport:
    def __init__(self): self.queue = deque(); self.closes = 0; self.error = None
    def push(self, row, peer=(PC, 32123, 0, 12)):
        data = json.dumps(row).encode() if isinstance(row, dict) else row
        self.queue.append((data, peer))
    def recvfrom(self, size):
        if self.error: raise self.error
        if not self.queue: raise BlockingIOError
        data, peer = self.queue.popleft()
        return data[:size], peer
    def close(self): self.closes += 1


def config(confirmed=True, gain=360., offset=0.):
    return dict(wheel=dict(source='udp', listen=['::', 64258], allow_from=[PC],
                source_id='moza-r3', device_key=W.DEVICE_KEY, physical_span_deg=450,
                fresh_ms=100, max_pc_age_ms=50,
                calibration=dict(confirmed=confirmed, game_gain_deg=gain, game_offset_deg=offset)))


def packet(**values):
    row = dict(schema=W.SCHEMA, source_id='moza-r3', stream_id='sender-session-1',
               seq=1, device_key=W.DEVICE_KEY, pc_query_ns=10_000_000_000,
               pc_send_ns=10_005_000_000, valid=True, axis_raw=65535,
               axis_min=0, axis_max=65535, reason='')
    row.update(values)
    return row


class WheelInputTests(unittest.TestCase):
    def test_vehicle_calibration_identity_is_explicit_or_disabled(self):
        cfg=config()
        self.assertIsNone(W.validate_config(cfg)['calibration']['beamng_vehicle_id'])
        cfg['wheel']['calibration']['beamng_vehicle_id']='52076'
        self.assertEqual(W.validate_config(cfg)['calibration']['beamng_vehicle_id'],'52076')
        for invalid in ['', ' ', 52076, True]:
            cfg['wheel']['calibration']['beamng_vehicle_id']=invalid
            with self.subTest(invalid=invalid), self.assertRaises(ValueError): W.validate_config(cfg)

    def receiver(self, cfg=None):
        clock, journal, transport = Clock(), Journal(), Transport()
        source = W.make_wheel(cfg or config(), clock, journal, transport=transport, allow_udp=False)
        self.addCleanup(source.close)
        return source, clock, journal, transport

    def test_disabled_default_opens_nothing(self):
        with patch.object(W.socket, 'socket', side_effect=AssertionError('opened socket')):
            for cfg in ({}, {'wheel': None}, {'wheel': {'source': 'none'}}):
                source = W.make_wheel(cfg, Clock(), Journal())
                source.poll(); self.assertEqual(source.latest(), (None, 1e9))
                self.assertFalse(source.summary()['enabled'])
                source.close(); source.close()
                self.assertTrue(source.summary()['closed'])

    def test_defaults_preserve_uncalibrated_raw_recording(self):
        cfg = dict(wheel=dict(source='udp', allow_from=[PC]))
        source, _, journal, transport = self.receiver(cfg)
        transport.push(packet())
        self.assertIsNone(source.latest()[0])
        sample = next(row for row in journal.rows if row['event'] == 'wheel_sample')
        self.assertEqual(sample['packet']['axis_raw'], 65535)
        self.assertEqual(sample['physical_angle_deg'], 225.)
        self.assertIsNone(sample['angle_rad'])
        self.assertFalse(sample['control_available'])
        self.assertEqual(sample['unavailable_reason'], 'calibration_unconfirmed')
        self.assertEqual(source.summary()['listen'], ['::', 64258])
        self.assertFalse(source.summary()['absolute_network_latency_known'])

    def test_real_pc_packet_serializer_interoperates_without_device_or_udp(self):
        path = Path(__file__).resolve().parents[3]/'beamng'/'moza_sender.py'
        spec = importlib.util.spec_from_file_location('_wheel_test_moza_sender', path)
        sender = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = sender
        spec.loader.exec_module(sender)
        source, _, journal, transport = self.receiver()
        row = sender.make_packet(sender.WheelRead(True, 0, ''), source_id='moza-r3',
                                 stream_id='serialized-pc-session', seq=1, joy_id=0,
                                 pc_query_ns=10_000_000_000, pc_send_ns=10_005_000_000)
        transport.push(json.dumps(row, allow_nan=False, separators=(',', ':')).encode())
        value, age = source.latest()
        self.assertEqual(value['norm'], -1.)
        self.assertAlmostEqual(value['angle_rad'], -2*math.pi)
        self.assertAlmostEqual(age, .005)
        unavailable = sender.make_packet(sender.WheelRead(False, None, 'query_error:167'),
                                         source_id='moza-r3', stream_id='serialized-pc-session',
                                         seq=2, joy_id=0, pc_query_ns=10_010_000_000,
                                         pc_send_ns=10_015_000_000)
        transport.push(json.dumps(unavailable, allow_nan=False).encode())
        self.assertIsNone(source.latest()[0])
        self.assertEqual(source.last_reason, 'device_invalid')
        self.assertTrue(any(r.get('packet') == unavailable for r in journal.rows))

    def test_physical_angle_and_signed_game_calibration_are_distinct(self):
        for raw, norm, physical in ((0, -1., -225.), (65535, 1., 225.)):
            with self.subTest(raw=raw):
                source, _, _, transport = self.receiver(config(gain=-360., offset=5.))
                transport.push(packet(axis_raw=raw))
                value, age = source.latest()
                self.assertEqual(value['norm'], norm)
                self.assertEqual(value['physical_angle_deg'], physical)
                self.assertAlmostEqual(value['angle_rad'], math.radians(norm*-360.+5.))
                self.assertAlmostEqual(age, .005)
                self.assertEqual(value['pc_query_ns'], 10_000_000_000)
                self.assertEqual(value['sequence'], 1)
                self.assertEqual(value['stream_id'], 'sender-session-1')
                value['angle_rad'] = 99  # Caller cannot modify the cached sample.
                self.assertNotEqual(source.latest()[0]['angle_rad'], 99)

    def test_center_quantization_is_not_hidden_by_deadband(self):
        source, _, _, transport = self.receiver()
        transport.push(packet(axis_raw=32768))
        value, _ = source.latest()
        self.assertAlmostEqual(value['norm'], 1/65535)
        self.assertAlmostEqual(value['physical_angle_deg'], 225/65535)

    def test_local_residence_and_pc_processing_age_expire_at_exact_boundary(self):
        source, clock, _, transport = self.receiver()
        transport.push(packet()); self.assertIsNotNone(source.latest()[0])
        clock.value += 94_999_999
        self.assertIsNotNone(source.latest()[0])
        clock.value += 1
        self.assertEqual(source.latest(), (None, 1e9))
        self.assertEqual(source.reasons['stale_sample'], 1)
        source.latest(); self.assertEqual(source.reasons['stale_sample'], 1)

    def test_relative_queue_delay_is_detected_without_comparing_clock_epochs(self):
        source, clock, journal, transport = self.receiver()
        transport.push(packet()); self.assertAlmostEqual(source.latest()[1], .005)
        # PC advances 10ms; arrival advances 30ms: 20ms extra relative delay.
        clock.value += 30_000_000
        transport.push(packet(seq=2, pc_query_ns=10_010_000_000, pc_send_ns=10_015_000_000))
        self.assertAlmostEqual(source.latest()[1], .025)
        clock.value += 100_000_000
        transport.push(packet(seq=3, pc_query_ns=10_020_000_000, pc_send_ns=10_025_000_000))
        self.assertIsNone(source.latest()[0])
        self.assertEqual(source.last_reason, 'relative_delay_stale')
        last_raw = [r for r in journal.rows if r['event'] == 'wheel_sample'][-1]
        self.assertEqual(last_raw['relative_queue_age_ms'], 110.)
        self.assertEqual(last_raw['packet']['seq'], 3)

    def test_first_packet_does_not_claim_absolute_network_delay_and_anchor_improves(self):
        source, clock, journal, transport = self.receiver()
        transport.push(packet()); source.latest()
        first_offset = source.summary()['offset_min_ns']
        # A faster path lowers the unknown offset; estimated age stays nonnegative.
        clock.value += 5_000_000
        transport.push(packet(seq=2, pc_query_ns=10_010_000_000, pc_send_ns=10_015_000_000))
        self.assertAlmostEqual(source.latest()[1], .005)
        self.assertEqual(source.summary()['offset_min_ns'], first_offset-5_000_000)
        self.assertFalse(journal.rows[-1]['absolute_network_latency_known'])

    def test_pc_processing_stale_packet_is_recorded_and_never_controls(self):
        source, _, journal, transport = self.receiver()
        transport.push(packet(pc_send_ns=10_050_000_000))
        self.assertIsNone(source.latest()[0])
        self.assertEqual(source.last_reason, 'pc_query_stale')
        self.assertEqual(source.validated, 1)
        self.assertTrue(any(r['event'] == 'wheel_sample' and r['pc_age_ms'] == 50. for r in journal.rows))

    def test_invalid_device_clears_cache_records_reason_and_consumes_sequence(self):
        source, _, journal, transport = self.receiver()
        transport.push(packet()); source.latest()
        invalid = packet(seq=2, pc_query_ns=10_010_000_000, pc_send_ns=10_015_000_000,
                         valid=False, axis_raw=None, reason='WinMM read failed: unplugged')
        transport.push(invalid); self.assertIsNone(source.latest()[0])
        self.assertEqual(source.last_reason, 'device_invalid')
        self.assertTrue(any(r.get('packet') == invalid for r in journal.rows))
        transport.push(packet(seq=2, pc_query_ns=10_020_000_000, pc_send_ns=10_025_000_000))
        self.assertIsNone(source.latest()[0])
        self.assertEqual(source.last_reason, 'sequence_not_increasing')

    def test_strict_wire_types_ranges_identity_and_timestamp_order(self):
        cases = [dict(schema='other'), dict(source_id='foreign'), dict(device_key='different'),
                 dict(stream_id=''), dict(seq=True), dict(seq=-1), dict(seq=2**63),
                 dict(pc_query_ns=True), dict(pc_query_ns=0), dict(pc_query_ns=1.),
                 dict(pc_send_ns=9_999_999_999), dict(valid=1), dict(axis_min=False),
                 dict(axis_min=1), dict(axis_max=65536), dict(axis_raw=True),
                 dict(axis_raw=-1), dict(axis_raw=65536), dict(axis_raw=1.5),
                 dict(axis_raw=float('nan')), dict(reason='error'), dict(reason=None),
                 dict(valid=False, axis_raw=None, reason=''),
                 dict(valid=False, axis_raw=0, reason='unplugged')]
        for changes in cases:
            with self.subTest(changes=changes):
                source, _, _, transport = self.receiver()
                transport.push(packet()); self.assertIsNotNone(source.latest()[0])
                row = packet(seq=2, pc_query_ns=10_010_000_000, pc_send_ns=10_015_000_000)
                row.update(changes); transport.push(row)
                self.assertIsNone(source.latest()[0])
                self.assertEqual(source.accepted, 1)

    def test_json_duplicate_missing_extra_oversize_and_foreign_peer(self):
        missing = packet(); del missing['reason']
        extra = packet(physical_degrees=225)
        duplicate = json.dumps(packet())[:-1]+',"seq":2}'
        for value in (b'no json', b'[]', missing, extra, duplicate.encode(), b' '*4097,
                      b'['*1500+b']'*1500):
            with self.subTest(value=str(value)[:30]):
                source, _, _, transport = self.receiver()
                transport.push(value); self.assertIsNone(source.latest()[0])
                self.assertEqual(source.validated, 0)
        source, _, _, transport = self.receiver()
        transport.push(packet(), peer=('fe80::9999', 64258, 0, 12))
        self.assertIsNone(source.latest()[0]); self.assertEqual(source.last_reason, 'foreign_sender')

    def test_ipv4_and_canonical_ipv6_allowlist_identity(self):
        for allow, peer in [('192.0.2.2', ('192.0.2.2', 1234)),
                            (PC, ('fe80:0:0:0:52a:153f:fd2b:1a0d%12', 1234, 0, 12))]:
            cfg = config(); cfg['wheel']['allow_from'] = [allow]
            source, _, _, transport = self.receiver(cfg)
            transport.push(packet(), peer); self.assertIsNotNone(source.latest()[0])

    def test_sequence_pc_clock_stream_restart_and_retired_replay(self):
        source, _, _, transport = self.receiver()
        transport.push(packet()); source.latest()
        for changes, reason in [(dict(seq=1), 'sequence_not_increasing'),
                                (dict(pc_query_ns=10_000_000_000), 'pc_clock_not_increasing'),
                                (dict(pc_send_ns=10_005_000_000, pc_query_ns=10_004_000_000), 'pc_clock_not_increasing')]:
            row = packet(seq=2, pc_query_ns=10_010_000_000, pc_send_ns=10_015_000_000)
            row.update(changes); transport.push(row)
            self.assertIsNone(source.latest()[0]); self.assertEqual(source.last_reason, reason)
        # New stream permits a genuinely restarted PC clock and fresh sequence.
        transport.push(packet(stream_id='new-session', pc_query_ns=10_000_000, pc_send_ns=15_000_000))
        self.assertEqual(source.latest()[0]['stream_id'], 'new-session')
        transport.push(packet(seq=3, pc_query_ns=10_020_000_000, pc_send_ns=10_025_000_000))
        self.assertIsNone(source.latest()[0]); self.assertEqual(source.last_reason, 'retired_stream')

    def test_stream_memory_is_bounded_without_forgetting_retired_ids(self):
        source, _, _, transport = self.receiver()
        for n in range(W.MAX_STREAMS):
            transport.push(packet(stream_id=f'session-{n}'))
            self.assertIsNotNone(source.latest()[0])
        transport.push(packet(stream_id='overflow-session'))
        self.assertIsNone(source.latest()[0]); self.assertEqual(source.last_reason, 'stream_capacity_exhausted')
        self.assertEqual(source.summary()['seen_streams'], W.MAX_STREAMS)
        transport.push(packet(stream_id='session-0', seq=2))
        self.assertIsNone(source.latest()[0]); self.assertEqual(source.last_reason, 'retired_stream')

    def test_bounded_receive_backlog_clears_usable_value(self):
        source, _, _, transport = self.receiver()
        for n in range(40):
            transport.push(packet(seq=n, pc_query_ns=10_000_000_000+n*1_000_000,
                                  pc_send_ns=10_005_000_000+n*1_000_000))
        source.poll()
        self.assertEqual(source.received, W.MAX_DATAGRAMS+1)
        self.assertIsNone(source.last); self.assertEqual(source.last_reason, 'datagram_backlog')
        self.assertEqual(len(transport.queue), 7)

    def test_socket_error_and_local_clock_regression_clear_usable_value(self):
        source, clock, _, transport = self.receiver()
        transport.push(packet()); source.latest()
        clock.value -= 1
        self.assertIsNone(source.latest()[0]); self.assertEqual(source.last_reason, 'local_clock_regressed')
        clock.value += 10_000_001
        transport.push(packet(seq=2, pc_query_ns=10_010_000_000, pc_send_ns=10_015_000_000))
        self.assertIsNotNone(source.latest()[0])
        transport.error = OSError('injected receive error')
        self.assertIsNone(source.latest()[0]); self.assertEqual(source.last_reason, 'socket_error')

    def test_ipv6_socket_scope_bind_and_nonblocking_without_opening_real_socket(self):
        sock = Mock()
        address = ('fe80::1', 64258, 0, 7)
        cfg = config(); cfg['wheel']['listen'] = ['fe80::1%eth0', 64258]
        with patch.object(W.socket, 'if_nametoindex', return_value=7), \
             patch.object(W.socket, 'getaddrinfo', return_value=[(W.socket.AF_INET6, W.socket.SOCK_DGRAM, W.socket.IPPROTO_UDP, '', address)]) as resolve, \
             patch.object(W.socket, 'socket', return_value=sock):
            source = W.make_wheel(cfg, Clock(), Journal())
        self.assertEqual(resolve.call_args.args[0], 'fe80::1%7')
        sock.setsockopt.assert_called_once_with(W.socket.IPPROTO_IPV6, W.socket.IPV6_V6ONLY, 1)
        sock.bind.assert_called_once_with(address); sock.setblocking.assert_called_once_with(False)
        source.close(); source.close(); sock.close.assert_called_once()

    def test_bind_failure_is_optional_but_journal_failure_is_not_masked(self):
        sock = Mock(); sock.bind.side_effect = OSError('injected bind failure')
        journal = Journal()
        with patch.object(W.socket, 'getaddrinfo', return_value=[(W.socket.AF_INET6, 2, 17, '', ('::', 64258, 0, 0))]), \
             patch.object(W.socket, 'socket', return_value=sock):
            source = W.make_wheel(config(), Clock(), journal)
        self.assertIsInstance(source, W.NoWheel); self.assertEqual(source.reason, 'bind_error')
        sock.close.assert_called_once()
        self.assertEqual(journal.rows[-1]['event'], 'wheel_unavailable')
        journal.failure = OSError('journal disk failure')
        transport = Transport()
        with self.assertRaisesRegex(OSError, 'journal disk failure'):
            W.make_wheel(config(), Clock(), journal, transport=transport)
        self.assertEqual(transport.closes, 1)

    def test_sample_journal_failure_propagates_and_close_is_idempotent(self):
        source, _, journal, transport = self.receiver()
        transport.push(packet()); source.latest()
        journal.failure = OSError('sample journal disk failure')
        transport.push(packet(seq=2, pc_query_ns=10_010_000_000, pc_send_ns=10_015_000_000))
        with self.assertRaisesRegex(OSError, 'sample journal disk failure'):
            source.latest()
        self.assertIsNone(source.last)
        source.close(); source.close()
        self.assertEqual(transport.closes, 1); self.assertEqual(source.latest(), (None, 1e9))

    def test_config_rejects_implicit_calibration_and_invalid_numeric_values(self):
        changes = [dict(source='unknown'), dict(allow_from=[]), dict(allow_from=['PC-hostname']),
                   dict(allow_from=PC), dict(listen=['::', True]), dict(listen=['::']),
                   dict(physical_span_deg=float('inf')), dict(fresh_ms=0), dict(max_pc_age_ms=101),
                   dict(calibration={'confirmed': 1}), dict(calibration={'confirmed': True}),
                   dict(calibration={'confirmed': True, 'game_gain_deg': 0}),
                   dict(calibration={'game_gain_deg': '360'}),
                   dict(calibration={'game_gain_deg': float('nan')}),
                   dict(calibration={'game_gain_deg': 1081}),
                   dict(calibration={'game_gain_deg': 1080, 'game_offset_deg': 1}),
                   dict(calibration={'game_offset_deg': 11})]
        with patch.object(W.socket, 'socket', side_effect=AssertionError('opened socket')):
            for change in changes:
                with self.subTest(change=change):
                    cfg = config(); cfg['wheel'].update(change)
                    with self.assertRaises(ValueError): W.make_wheel(cfg, Clock(), Journal())
            with self.assertRaisesRegex(ValueError, 'injected transport'):
                W.make_wheel(config(), Clock(), Journal(), allow_udp=False)

    def test_priority_switch_requires_a_real_boolean_even_when_disabled(self):
        with patch.object(W.socket, 'socket', side_effect=AssertionError('opened socket')):
            for source in ('none', 'udp'):
                for value in ('false', 'true', 0, 1, None):
                    with self.subTest(source=source, value=value):
                        cfg = config()
                        cfg['wheel'].update(source=source, priority_prediction=value)
                        with self.assertRaisesRegex(ValueError, 'priority_prediction must be boolean'):
                            W.make_wheel(cfg, Clock(), Journal())


if __name__ == '__main__':
    unittest.main()
