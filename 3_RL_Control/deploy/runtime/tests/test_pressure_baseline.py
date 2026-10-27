"""Model-free pressure baseline regression checks; no serial port or socket is opened.

    python -B deploy/runtime/tests/test_pressure_baseline.py
"""
import copy
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
import json
import tempfile

RT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RT))
from oasis_rt import controller as C, sources as SRC  # noqa: E402


class Clock:
    def __init__(self): self.t = 0.0
    def __call__(self): return self.t
    def sleep(self, dt): self.t += dt


class PressureStream:
    """Timestamped sensor deliveries. Polling after the final delivery retains the cached sample."""
    def __init__(self, clock, deliveries):
        self.clock, self.deliveries = clock, iter(deliveries)
        self.next = next(self.deliveries, None)
        self.pressure, self.pressure_at, self.fault = {}, -1e9, None

    def poll(self):
        while self.next is not None and self.next[0] <= self.clock() + 1e-9:
            _, self.pressure_at, self.pressure = self.next
            self.next = next(self.deliveries, None)


class PressureBaselineTests(unittest.TestCase):
    def setUp(self):
        self.cfg = copy.deepcopy(C.load_config(str(RT / 'config.yaml')))
        self.cfg['_K'] = 0
        self.cfg['pressure']['counts_per_N'] = {
            ch: 87.0 for ch in self.cfg['pressure']['channels'].values() if isinstance(ch, str)
        }
        names = {ch for value in self.cfg['pressure']['channels'].values()
                 for ch in (value if isinstance(value, list) else [value])}
        self.values = {ch: 500 + i * 10 for i, ch in enumerate(sorted(names))}

    def stream(self, *, step=0.02, until=2.4):
        return [(round(i * step, 9), round(i * step, 9), dict(self.values))
                for i in range(1, int(until / step) + 1)]

    def controller(self, deliveries):
        clock = Clock(); link = PressureStream(clock, deliveries)
        ctl = C.Controller(self.cfg, link, SimpleNamespace(OD=1, A=2), SimpleNamespace(OBS_DIM=1),
                           None, SRC.NoPose(), SRC.NoPose(), clock, log=lambda *args: None)
        return ctl, clock

    def test_continuous_stream_uses_full_acquisition_window(self):
        ctl, clock = self.controller(self.stream())
        self.assertEqual(ctl.zero_pressure(wait=clock.sleep), self.values)
        self.assertGreaterEqual(clock(), 2.02 - 1e-9)

    def test_duplicate_reads_do_not_weight_the_average(self):
        deliveries = self.stream(step=0.1)
        # Same physical stream delivered slightly off the polling grid: each sample may be read
        # four, five or six times. The baseline must average acquisitions, not reads.
        deliveries = [(t + (0.01 if i % 2 else 0.0), t, {ch: v + (10 if i % 2 else 0)
                       for ch, v in values.items()}) for i, (t, _, values) in enumerate(deliveries)]
        ctl, clock = self.controller(deliveries)
        baseline = ctl.zero_pressure(wait=clock.sleep)
        for ch, value in self.values.items():
            self.assertAlmostEqual(baseline[ch], value + 100 / 21)

    def test_one_cached_sweep_never_qualifies(self):
        ctl, clock = self.controller(self.stream(until=0.02))
        with self.assertRaisesRegex(RuntimeError, 'stale pressure'):
            ctl.zero_pressure(wait=clock.sleep)
        self.assertIsNone(ctl.base)

    def test_no_pressure_or_preexisting_stale_sample_rejected(self):
        for deliveries in ([], [(0, -10.0, self.values)]):
            with self.subTest(deliveries=deliveries):
                ctl, clock = self.controller(deliveries)
                with self.assertRaisesRegex(RuntimeError, 'no fresh pressure'):
                    ctl.zero_pressure(wait=clock.sleep)
                self.assertLessEqual(clock(), 0.32)

    def test_late_burst_does_not_replace_full_window(self):
        ctl, clock = self.controller([d for d in self.stream() if d[0] >= 1.8])
        with self.assertRaisesRegex(RuntimeError, 'no fresh pressure'):
            ctl.zero_pressure(wait=clock.sleep)

    def test_gap_then_fresh_tail_is_rejected(self):
        deliveries = [d for d in self.stream() if d[0] < 0.5 or d[0] >= 1.0]
        ctl, clock = self.controller(deliveries)
        with self.assertRaisesRegex(RuntimeError, 'stale pressure'):
            ctl.zero_pressure(wait=clock.sleep)

    def test_missing_required_channel_rejected(self):
        deliveries = self.stream()
        del deliveries[20][2]['A1_C1']
        ctl, clock = self.controller(deliveries)
        with self.assertRaisesRegex(RuntimeError, 'missing/invalid channels'):
            ctl.zero_pressure(wait=clock.sleep)

    def test_future_or_regressing_timestamp_rejected(self):
        for stamp, reason in ((10.0, 'invalid acquisition timestamp'), (0.01, 'timestamp regressed')):
            with self.subTest(stamp=stamp):
                deliveries = self.stream()
                deliveries[20] = (deliveries[20][0], stamp, self.values)
                ctl, clock = self.controller(deliveries)
                with self.assertRaisesRegex(RuntimeError, reason):
                    ctl.zero_pressure(wait=clock.sleep)

    def test_noisy_fresh_stream_still_rejected(self):
        deliveries = [(t, stamp, {ch: value + (100 if i % 2 else 0) for ch, value in values.items()})
                      for i, (t, stamp, values) in enumerate(self.stream())]
        ctl, clock = self.controller(deliveries)
        with self.assertRaisesRegex(RuntimeError, r'baseline rejected \(n, std\)'):
            ctl.zero_pressure(wait=clock.sleep)

    def test_too_few_distinct_samples_rejected(self):
        ctl, clock = self.controller(self.stream(step=0.25, until=2.5))
        with self.assertRaisesRegex(RuntimeError, r'baseline rejected \(n, std\)'):
            ctl.zero_pressure(wait=clock.sleep)

    def test_explicit_warn_mode_accepts_noise_and_records_actual_statistics(self):
        self.cfg['pressure']['baseline_variability_mode'] = 'warn'
        deliveries = [(t, stamp, {ch: value + (200 if i % 2 else 0) for ch, value in values.items()})
                      for i, (t, stamp, values) in enumerate(self.stream())]
        ctl, clock = self.controller(deliveries)
        events = []
        ctl.link.pressure_log = SimpleNamespace(record=lambda event, **fields: events.append((event, fields)))
        baseline = ctl.zero_pressure(wait=clock.sleep)
        self.assertTrue(all(self.values[ch] < value < self.values[ch]+200 for ch,value in baseline.items()))
        self.assertEqual(events[0][0], 'pressure_baseline_quality')
        self.assertEqual(events[0][1]['variability_mode'], 'warn')
        self.assertTrue(events[0][1]['noisy_channels'])

    def test_warn_mode_still_rejects_missing_stale_and_insufficient_samples(self):
        self.cfg['pressure']['baseline_variability_mode'] = 'warn'
        missing = self.stream(); del missing[20][2]['A1_C1']
        for deliveries in (missing, self.stream(until=.02), self.stream(step=.25,until=2.5)):
            with self.subTest(deliveries=len(deliveries)):
                ctl, clock = self.controller(deliveries)
                with self.assertRaises(RuntimeError):ctl.zero_pressure(wait=clock.sleep)
                self.assertIsNone(ctl.base)

    def test_unknown_variability_mode_rejected(self):
        self.cfg['pressure']['baseline_variability_mode'] = 'ignore_everything'
        ctl,clock = self.controller(self.stream())
        with self.assertRaisesRegex(ValueError,'baseline_variability_mode'):
            ctl.zero_pressure(wait=clock.sleep)

    def test_baseline_quality_event_with_real_session_journal(self):
        from oasis_rt import fbstack
        fbstack.load()
        from oasis_rt.pressure_log import PressureLogger
        self.cfg['pressure']['baseline_variability_mode'] = 'warn'
        ctl,clock = self.controller(self.stream())
        with tempfile.TemporaryDirectory(prefix='baseline_journal_') as folder:
            journal = PressureLogger(folder)
            ctl.link.pressure_log = journal
            try:ctl.zero_pressure(wait=clock.sleep)
            finally:journal.close()
            rows = [json.loads(line) for line in (Path(folder)/'control.jsonl').read_text().splitlines()]
            quality = next(row for row in rows if row['event']=='pressure_baseline_quality')
            self.assertEqual(quality['host_monotonic_ns'],round(clock()*1e9))
            self.assertTrue(quality['acquisition_valid'])
            self.assertEqual(quality['variability_mode'],'warn')

    def test_mock_fb_session_baseline_and_scanner_loss(self):
        """Real session/FB parser path: a frozen scanner cannot reach control and ends with verified OFF."""
        from oasis_rt import app, beamng_rx, fb_link, fbstack
        from oasis_rt.synthetic import SyntheticReceiver
        from mock_fb_board import MockClockNs, MockFbBoard

        cfg = self.cfg

        class BaselineOnlyPolicy:
            name = 'rule'
            control_entered = False

            def bind(self, session):
                self.ctl = C.Controller(cfg, session.link, SimpleNamespace(OD=1, A=2),
                                        SimpleNamespace(OBS_DIM=1), None, SRC.NoPose(), SRC.NoPose(),
                                        session.clock, log=lambda *args: None)

            def start(self, session):
                if freeze_scanner: board.scan = False
                self.baseline = self.ctl.zero_pressure(wait=session.wait_neutral)

            def run(self, session, duration):
                self.control_entered = True
                session.wait_neutral(0.05)

        for freeze_scanner in (False, True):
            with self.subTest(freeze_scanner=freeze_scanner):
                fb = fbstack.load(); clock = MockClockNs(); board = MockFbBoard(clock)
                link = fb_link.FbLink(board, fb, clock, log=lambda *args: None)
                vin = beamng_rx.VehicleInput(fb, SyntheticReceiver(clock), clock, allow_synthetic=True)
                policy = BaselineOnlyPolicy()
                session = app.Session(cfg, fb, link, vin, policy, clock, clock.sleep, '', log=lambda *args: None)
                result = session.run(startup_timeout_s=10)
                self.assertTrue(result['off_verified'], result)
                self.assertFalse(board.armed)
                self.assertEqual(sum(command.startswith('FB_ARM') for _, command in board.commands), 1)
                self.assertEqual(board.lease_faults, 0)
                self.assertEqual(policy.control_entered, not freeze_scanner)
                if freeze_scanner:
                    self.assertIn('pressure baseline rejected', result['result'])
                    self.assertEqual(result['phase'], 'BASELINE')
                else:
                    self.assertEqual(result['result'], 'completed')
                    self.assertTrue(all(value == 1000 for value in policy.baseline.values()))


if __name__ == '__main__':
    unittest.main()
