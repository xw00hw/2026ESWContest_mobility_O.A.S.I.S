"""Optional pose contract and lifecycle: injected transports/clocks, no cameras."""
from collections import deque
import csv
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE.parent), str(HERE)]
from oasis_rt import app, controller as C, fbstack, pose_input as P

FB = fbstack.load()


class Clock:
    def __init__(self, value=1_000_000_000): self.value = value
    def __call__(self): return self.value


class Journal:
    def __init__(self): self.rows = []; self.fail = False
    def record(self, event, **fields):
        if self.fail: raise RuntimeError('injected journal failure')
        self.rows.append(dict(event=event, **fields))


class Transport:
    def __init__(self): self.queue = deque(); self.closes = 0
    def push(self, packet, peer=('127.0.0.1', 12345)):
        self.queue.append((json.dumps(packet).encode() if isinstance(packet, dict) else packet, peer))
    def recvfrom(self, size):
        if not self.queue: raise BlockingIOError
        data, peer = self.queue.popleft()
        return data[:size], peer
    def close(self): self.closes += 1


def config():
    return dict(pose=dict(source='udp', source_id='front_seat_pose', udp_port=0,
                          stale_s=.3, min_confidence=.5, lean_sign=1),
                profile=dict(from_camera=False))


def packet(**values):
    row = dict(schema=P.SCHEMA, source_id='front_seat_pose', stream_id='camera_session_1', seq=1,
               capture_clock='monotonic', capture_ns=990_000_000, torso_lean_deg=2.5,
               lean_score=.05, pose_confidence=.9, vision_valid=True, neutral_valid=True)
    row.update(values)
    return row


class PoseInputTests(unittest.TestCase):
    def receiver(self, **kwargs):
        clock, journal, transport = Clock(), Journal(), Transport()
        source = P.make_pose_input(config(), clock, journal, transport=transport, **kwargs)
        self.addCleanup(source.close)
        return source, clock, journal, transport

    def test_none_opens_nothing_and_metric_body_mode_is_rejected(self):
        cfg = config(); cfg['pose']['source'] = 'none'
        with patch.object(P.socket, 'socket', side_effect=AssertionError('socket opened')):
            source = P.make_pose_input(cfg, Clock(), Journal())
            self.assertEqual(source.latest(), (None, 1e9))
            self.assertFalse(P.pose_summary(source)['enabled'])
            P.close_pose(source)
            cfg['profile']['from_camera'] = True
            with self.assertRaisesRegex(ValueError, 'calibrated metric'):
                P.make_pose_input(cfg, Clock(), Journal())

    def test_valid_frame_retains_capture_time_and_freshness_expires(self):
        source, clock, journal, transport = self.receiver()
        transport.push(packet())
        data, age = source.latest()
        self.assertEqual(data['capture_ns'], 990_000_000)
        self.assertAlmostEqual(age, .01)
        self.assertEqual(journal.rows[-1]['packet']['capture_ns'], 990_000_000)
        self.assertEqual(journal.rows[-1]['host_monotonic_ns'], clock())
        clock.value = 1_290_000_000  # Exact stale boundary, without a new datagram.
        self.assertEqual(source.latest(), (None, 1e9))
        self.assertEqual(source.reasons['stale_capture'], 1)
        source.latest()
        self.assertEqual(source.reasons['stale_capture'], 1)

    def test_monotonic_raw_uses_its_own_clock_not_receipt_clock(self):
        raw_clock = Clock(51_000_000_000)
        source, clock, _, transport = self.receiver(clocks={'monotonic_raw': raw_clock})
        transport.push(packet(capture_clock='monotonic_raw', capture_ns=50_980_000_000))
        value, age = source.latest()
        self.assertAlmostEqual(age, .02)
        self.assertEqual(value['capture_ns'], 50_980_000_000)
        clock.value += 10_000_000_000  # Different clock offset must not age the raw-clock frame.
        self.assertAlmostEqual(source.latest()[1], .02)
        raw_clock.value += 300_000_000
        self.assertIsNone(source.latest()[0])

    def test_invalid_packets_invalidate_cached_pose_without_restamping(self):
        cases = [dict(schema='other'), dict(source_id='unexpected'), dict(stream_id=''),
                 dict(seq=True), dict(seq=-1), dict(capture_ns=True), dict(capture_ns=600_000_000),
                 dict(capture_ns=1_000_000_001), dict(capture_clock='unknown'),
                 dict(torso_lean_deg=float('nan')), dict(lean_score=float('inf')),
                 dict(torso_lean_deg='2'), dict(torso_lean_deg=91), dict(lean_score=5),
                 dict(pose_confidence=.1), dict(pose_confidence=1.1),
                 dict(vision_valid=False), dict(neutral_valid=False), dict(vision_valid=1)]
        for changes in cases:
            with self.subTest(changes=changes):
                source, _, _, transport = self.receiver()
                transport.push(packet()); self.assertIsNotNone(source.latest()[0])
                row = packet(seq=2, capture_ns=995_000_000); row.update(changes)
                transport.push(row)
                self.assertEqual(source.latest(), (None, 1e9))
                self.assertEqual(source.accepted, 1)

    def test_bad_json_missing_extra_duplicate_oversize_and_foreign_sender(self):
        missing = packet(); del missing['neutral_valid']
        extra = packet(shoulder_width_m=.42)
        duplicate = json.dumps(packet())[:-1]+', "seq": 2}'
        for value in (b'not-json', b'[]', missing, extra, duplicate.encode(), b' '*4097,
                      b'['*1500+b']'*1500):
            with self.subTest(value=str(value)[:60]):
                source, _, _, transport = self.receiver()
                transport.push(value)
                self.assertIsNone(source.latest()[0])
                self.assertEqual(source.accepted, 0)
        source, _, _, transport = self.receiver()
        transport.push(packet(), peer=('192.0.2.1', 12345))
        self.assertIsNone(source.latest()[0])
        self.assertEqual(source.reasons['non_loopback_sender'], 1)

    def test_stream_sequence_capture_order_and_clock_change_are_checked(self):
        source, clock, _, transport = self.receiver(clocks={'monotonic': Clock(), 'monotonic_raw': Clock()})
        transport.push(packet()); self.assertIsNotNone(source.latest()[0])
        transport.push(packet(capture_ns=995_000_000))
        self.assertIsNone(source.latest()[0])
        self.assertEqual(source.reasons['sequence_not_increasing'], 1)
        transport.push(packet(seq=2, capture_ns=989_000_000))
        self.assertIsNone(source.latest()[0])
        self.assertEqual(source.reasons['capture_not_increasing'], 1)
        transport.push(packet(seq=2, capture_ns=995_000_000, capture_clock='monotonic_raw'))
        self.assertIsNone(source.latest()[0])
        self.assertEqual(source.reasons['capture_clock_changed'], 1)
        transport.push(packet(stream_id='camera_session_2', seq=0, capture_ns=995_000_000))
        self.assertEqual(source.latest()[0]['seq'], 0)
        self.assertEqual(source.summary()['stream_id'], 'camera_session_2')

    def test_receive_work_is_bounded_and_reports_backlog(self):
        source, _, journal, transport = self.receiver()
        for n in range(40):
            transport.push(packet(seq=n, capture_ns=900_000_000+n*1_000_000))
        source.poll()
        self.assertEqual(source.received, P.MAX_DATAGRAMS+1)
        self.assertIsNone(source.last)
        self.assertEqual(len(transport.queue), 7)
        self.assertEqual(journal.rows[-1]['reason'], 'datagram_backlog')
        self.assertEqual(source.latest()[0]['seq'], 39)

    def test_journal_failure_is_not_silenced_and_close_is_idempotent(self):
        source, _, journal, transport = self.receiver()
        journal.fail = True
        transport.push(packet())
        with self.assertRaisesRegex(RuntimeError, 'journal failure'):
            source.latest()
        source.close(); source.close()
        self.assertEqual(transport.closes, 1)
        self.assertEqual(source.latest(), (None, 1e9))
        failed_journal, unopened_transport = Journal(), Transport()
        failed_journal.fail = True
        with self.assertRaisesRegex(RuntimeError, 'journal failure'):
            P.make_pose_input(config(), Clock(), failed_journal, transport=unopened_transport)
        self.assertEqual(unopened_transport.closes, 1)

    def test_socket_error_clears_cached_pose(self):
        source, _, _, transport = self.receiver()
        transport.push(packet()); self.assertIsNotNone(source.latest()[0])
        with patch.object(transport, 'recvfrom', side_effect=OSError('injected socket failure')):
            self.assertIsNone(source.latest()[0])
        self.assertEqual(source.reasons['socket_error'], 1)

    def test_unsupported_configuration_is_refused_before_hardware(self):
        cfg = C.load_config(str(HERE.parent/'config.yaml'))
        cfg['profile']['from_camera'] = True
        with patch.object(app, 'RelayReceiver') as receiver, patch.object(FB.serial_link, 'open_serial') as serial:
            with self.assertRaisesRegex(ValueError, 'calibrated metric'):
                app.assemble(cfg, 'hw', lambda *args: None)
            receiver.assert_not_called(); serial.assert_not_called()
        with patch.object(P.socket, 'socket') as socket_factory:
            with self.assertRaisesRegex(ValueError, 'injected transport'):
                P.make_pose_input(config(), Clock(), Journal(), allow_udp=False)
            socket_factory.assert_not_called()

    def test_app_injected_pose_is_closed_and_journaled(self):
        cfg = C.load_config(str(HERE.parent/'config.yaml'))
        cfg['control']['mode'] = 'fb_v6'
        cfg['pose'].update(config()['pose'])
        made = []
        def inject(clock, journal):
            transport = Transport()
            transport.push(packet(capture_ns=clock()-10_000_000))
            source = P.make_pose_input(cfg, clock, journal, allow_udp=False, transport=transport,
                                       clocks={'monotonic': clock})
            made.append((source, transport))
            return source
        with tempfile.TemporaryDirectory() as directory:
            cfg['log_dir'] = directory
            with patch.object(P.socket, 'socket', side_effect=AssertionError('live socket opened')):
                manifest, session, _ = app.run(cfg, 'sil', .02, log=lambda *args: None, sil_pose=inject)
            source, transport = made[0]
            self.assertEqual(manifest['result']['result'], 'completed')
            self.assertTrue(manifest['pose']['closed'])
            self.assertEqual(manifest['pose']['accepted'], 1)
            self.assertEqual(transport.closes, 1)
            journal = [json.loads(line) for line in (Path(session.out)/'control.jsonl').read_text().splitlines()]
            self.assertTrue(any(row['event'] == 'pose_sample' for row in journal))

    def test_armed_pose_journal_failure_verifies_off_and_closes_resources(self):
        cfg = C.load_config(str(HERE.parent/'config.yaml'))
        cfg['control']['mode'] = 'fb_v6'; cfg['pose'].update(config()['pose'])
        made = []
        def inject(clock, journal):
            transport = Transport()
            source = P.make_pose_input(cfg, clock, journal, transport=transport, clocks={'monotonic': clock})
            made.append((source, transport)); return source
        with tempfile.TemporaryDirectory() as directory:
            cfg['log_dir'] = directory
            _, session, dev = app.assemble(cfg, 'sil', lambda *args: None, sil_pose=inject)
            journal = session.link.pressure_log
            original_record = journal.record
            def fail_pose_record(event, **fields):
                if event == 'pose_sample':
                    journal._latch('injected_pose_journal_failure')
                    journal.check()
                original_record(event, **fields)
            journal.record = fail_pose_record
            def fail_during_control(s, _duration):
                self.assertTrue(s.link.armed)
                made[0][1].push(packet(capture_ns=s.clock_ns()-10_000_000))
                s.pump()
            session.policy.run = fail_during_control
            result = session.run(.1, startup_timeout_s=5)
            self.assertIn('injected_pose_journal_failure', result['result'])
            self.assertTrue(result['off_verified'])
            self.assertTrue(dev.closed)
            self.assertFalse(dev.armed)
            self.assertEqual(made[0][1].closes, 1)
            self.assertEqual(sum(command.startswith('FB_ARM') for _, command in dev.commands), 1)
            self.assertFalse(journal.summary()['writer_alive'])
            self.assertTrue((Path(session.out)/'INCOMPLETE').exists())

    def test_pose_close_error_preserves_off_result_manifest_and_journal_close(self):
        cfg = C.load_config(str(HERE.parent/'config.yaml'))
        cfg['control']['mode'] = 'fb_v6'; cfg['pose'].update(config()['pose'])
        class BadClose(Transport):
            def close(self):
                super().close()
                raise OSError('injected pose close failure')
        def inject(clock, journal):
            return P.make_pose_input(cfg, clock, journal, transport=BadClose(), clocks={'monotonic': clock})
        with tempfile.TemporaryDirectory() as directory:
            cfg['log_dir'] = directory
            manifest, session, dev = app.run(cfg, 'sil', .02, log=lambda *args: None, sil_pose=inject)
            self.assertTrue(manifest['result']['off_verified'])
            self.assertTrue(dev.closed)
            self.assertIn('pose_close:', manifest['result']['result'])
            self.assertIn('injected pose close failure', manifest['pose']['cleanup_error'])
            self.assertTrue(manifest['pressure_logging']['complete'])
            self.assertTrue((Path(session.out)/'manifest.json').exists())

    def test_model_observations_use_valid_pose_and_revert_on_stale(self):
        cfg = C.load_config(str(HERE.parent/'config.yaml'))
        model = os.environ.get('OASIS_TEST_MODEL_V3', cfg['model_dir'])
        if not (Path(model)/'manifest.json').exists():
            self.skipTest('requires exported model for observation integration')
        cfg['model_dir'] = str(Path(model).resolve())
        cfg['actor']['backends'] = ['numpy']
        cfg['actor']['check_set'] = str(Path(model)/'check_set.npy')
        cfg['control']['mode'] = 'rl'; cfg['control']['cold_fallback'] = None
        cfg['pose'].update(config()['pose'])
        cfg['pose']['lean_sign'] = -1
        made = []
        def inject(clock, journal):
            transport = Transport()
            source = P.make_pose_input(cfg, clock, journal, transport=transport, clocks={'monotonic': clock})
            made.append((source, transport))
            return source
        with tempfile.TemporaryDirectory() as directory:
            cfg['log_dir'] = directory
            _, session, _ = app.assemble(cfg, 'sil', lambda *args: None, sil_pose=inject,
                                         sil_tune=dict(yaw_max_mdeg=20000, yaw_rate_mdeg_s=10000))
            original_start = session.policy.start
            def start_and_feed(s):
                original_start(s)
                made[0][1].push(packet(capture_ns=s.clock_ns()-10_000_000))
            session.policy.start = start_and_feed
            result = session.run(.6, startup_timeout_s=5)
            self.assertEqual(result['result'], 'completed')
            self.assertIs(session.policy.ctl.pose, session.pose)
            with (Path(session.out)/'control.csv').open(newline='') as handle:
                rows = list(csv.DictReader(handle))
            self.assertAlmostEqual(float(rows[0]['o37']), -.5, places=5)
            self.assertAlmostEqual(float(rows[0]['o38']), -.42, places=5)
            self.assertAlmostEqual(float(rows[0]['o54']), .01, places=5)
            self.assertEqual(float(rows[-1]['o37']), 0)
            self.assertEqual(float(rows[-1]['o38']), 0)
            self.assertEqual(float(rows[-1]['o54']), 1)
            self.assertEqual(session.policy.ctl.profile, [75, .42, .58])


if __name__ == '__main__':
    unittest.main()
