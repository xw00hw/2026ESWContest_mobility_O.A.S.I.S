"""Integrated runtime tests: BeamNG collector path -> RL / tuned rule / FB V6 -> OASIS_FALLBACK v2 (MockFbBoard).

    python -m pytest deploy/runtime/tests -q        (or: python deploy/runtime/tests/test_app_fb.py)
Needs the BeamNG_Bolster_FB host modules (4_Seat_Actuation/, found automatically or $OASIS_FB_ROOT)
and, for the rl cases, an exported model (config model_dir, else $OASIS_TEST_MODEL_V3); without it the rl cases
check the cold fallback instead.
"""
import copy, os, socket, sys, time, unittest
from unittest.mock import patch
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RT = os.path.abspath(os.path.join(HERE, '..'))
sys.path[:0] = [RT, HERE]
from oasis_rt import app, beamng_rx as BR, controller as C, envelope as E, fbstack  # noqa: E402
from oasis_rt.synthetic import SyntheticReceiver, row  # noqa: E402
from mock_fb_board import MockClockNs, MockFbBoard  # noqa: E402

FB = fbstack.load()
CFG = C.load_config(os.path.join(RT, 'config.yaml'))
if os.environ.get('OASIS_TEST_MODEL_V3'): CFG['model_dir'] = os.path.abspath(os.environ['OASIS_TEST_MODEL_V3'])
HAVE_MODEL = os.path.exists(os.path.join(CFG['model_dir'], 'manifest.json'))
MODEL_FB_A2 = HAVE_MODEL and (__import__('json').load(open(os.path.join(CFG['model_dir'], 'manifest.json'))).get('actuator') or {}).get('mode') == 'fb_a2'
QUIET = lambda *a: None


def cfg_for(mode='rl', **beamng):
    c = copy.deepcopy(CFG); c['control']['mode'] = mode; c['actor']['backends'] = ['numpy']
    c['control']['actuator_mismatch'] = beamng.pop('actuator_mismatch', c['control'].get('actuator_mismatch', 'rule'))
    c['beamng'].update(beamng); c['log_dir'] = os.path.join(os.environ.get('TMP', '/tmp'), 'oasis_test_logs')
    if HAVE_MODEL: c['actor']['check_set'] = os.path.join(c['model_dir'], 'check_set.npy')
    return c


def sil(mode='rl', seconds=30.0, tune=None, rx=None, **beamng):
    man, s, dev = app.run(cfg_for(mode, **beamng), 'sil', seconds, log=QUIET, sil_tune=tune, sil_receiver=rx)
    return man, s, dev


def commands(dev):
    return [c for _, c in dev.commands]


# ------------------------------------------------------------------ envelope / receiver units
def test_envelope_never_leaves_cad_tune_ik():
    rng = np.random.default_rng(0)
    for tune in (dict(yaw_max_mdeg=15000, pitch_max_mdeg=6000), dict(yaw_max_mdeg=22000, pitch_max_mdeg=6000),
                 dict(yaw_max_mdeg=8000, pitch_max_mdeg=2000)):
        env = E.Envelope(FB, tune)
        for yL, pL, yR, pR in rng.uniform([-5, -10, -5, -10], [30, 10, 30, 10], (3000, 4)):
            pose, _ = env.project(yL, pL, yR, pR)
            assert FB.cad_limits.pose_within_cad(pose) and env.ok(pose), (tune, pose)
            assert pose[0] <= tune['yaw_max_mdeg'] and abs(pose[1]) <= tune['pitch_max_mdeg']
    env = E.Envelope(FB, dict(yaw_max_mdeg=15000, pitch_max_mdeg=6000))
    assert env.project(10.0, 0.0, 3.25, 0.0) == ((10000, 0, 3250, 0), False)
    assert env.project(20.0, 8.0, 0.0, 0.0)[0][:2] == (15000, 6000)          # yaw clipped to tune, pitch at CAD cap
    for bad in (float('nan'), float('inf')):
        try: env.project(bad, 0, 0, 0); assert False
        except ValueError: pass


def test_relay_header_roundtrip_and_restamp():
    r = BR.RelayReceiver(FB, ('127.0.0.1', 0), allow=['127.0.0.1'], clock_ns=time.perf_counter_ns)
    port = r.s.getsockname()[1]
    from fallback.beamng_bridge import make_packet
    prov = dict(collector_version='0.3', csv_revision=2, session_id='s', config_sha256='a' * 64,
                source_sha256={'collect.py': 'b' * 64})
    raw = row(1, 0.0, 0.02, 1.0, 10.0, source_kind='beamng'); raw['host_monotonic_ns'] = 123
    payload = make_packet(raw, prov, forwarded_ns=456)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tx.sendto(BR.relay_wrap(payload, 5_000_000, 7), ('127.0.0.1', port)); time.sleep(0.05)
    got = r.receive(); now = time.perf_counter_ns()
    assert got['_pc_host_monotonic_ns'] == 123 and got['_bridge_session_id'] == 's'
    assert 0 <= now - got['host_monotonic_ns'] - 5_000_000 < 1e9                # Pi receipt minus the PC-side age
    tx.sendto(b'garbage', ('127.0.0.1', port)); time.sleep(0.05)
    try: r.receive(); assert False, 'malformed relay datagram accepted'
    except ValueError: pass
    r.close(); tx.close()


def test_steer_sign_monitor():
    m = BR.SteerSignMonitor(n_min=20)
    for _ in range(30): m.update(3.0, 14.0, 40.0)
    assert m.verdict == 'ok'
    m = BR.SteerSignMonitor(n_min=20)
    for _ in range(30): m.update(3.0, 14.0, -40.0)
    assert m.verdict == 'mismatch'


# ------------------------------------------------------------------ session (SIL)
REC_TUNE = dict(yaw_max_mdeg=20000, yaw_rate_mdeg_s=20000)   # deploy/README.md board preparation (mock default: FB's 2 deg/s)


def test_rl_session_clean():
    man, s, dev = sil('rl', 30.0, tune=REC_TUNE)
    r = man['result']
    assert r['result'] == 'completed' and r['off_verified'], r
    cmds = commands(dev); arms = [c for c in cmds if c.startswith('FB_ARM')]
    assert len(arms) == 1 and cmds.index(arms[0]) > cmds.index('START_12,15')       # pressure scan before ARM
    assert dev.fault == 'OFF' and dev.lease_faults == 0 and dev.max_pulse_step <= 2
    assert man['board']['sweeps'] > 1000 and man['board']['bad_sweeps'] == 0
    if HAVE_MODEL:
        ps = man['policy_summary']; assert man['policy'] == 'rl' and ps['actor_fallbacks'] == 0
        assert ps['mode_final'] == ('rl' if MODEL_FB_A2 else 'rule'), ps                 # actuator check (v7 -> rule)
        sets = [tuple(map(int, c.split(',')[3:])) for c in cmds if c.startswith('FB_SET')]
        assert max(max(p[0], p[2]) for p in sets) > 1000                                 # the policy moved a bolster
    else:
        assert man['policy'] == 'fb_v6'                                              # cold fallback without a model


def test_lease_only_from_healthy_ticks_and_set_rate():
    man, s, dev = sil('rule', 20.0)
    t = np.array([ts for ts, c in dev.commands if c.startswith('FB_SET')]) / 1e9
    p = np.array([ts for ts, c in dev.commands if c.startswith('FB_PING')]) / 1e9
    assert len(t) > 50 and np.min(np.diff(t)) >= 0.04 - 1e-9                         # <= 25 Hz
    assert np.max(np.diff(p)) < 0.3 and dev.lease_faults == 0                         # PING ~10 Hz while healthy


def test_fb_v6_session_clean():
    man, s, dev = sil('fb_v6', 30.0, tune=dict(yaw_rate_mdeg_s=4000))
    assert man['result']['result'] == 'completed' and man['result']['off_verified'], man['result']
    rs = man['policy_summary']['reasons']
    assert rs.get('vehicle_only_feedforward', 0) > 100 and dev.lease_faults == 0
    sets = [tuple(map(int, c.split(',')[3:])) for c in commands(dev) if c.startswith('FB_SET')]
    assert max(p[0] for p in sets) > 1000 and max(p[2] for p in sets) > 1000              # both sides used
    assert all(not (p[0] and p[2]) for p in sets)                                         # FB: one side at a time


def test_vehicle_pause_turns_off_without_rearm():
    """BeamNG menu / lost link after ARM: stale -> FB_OFF, session ends, exactly one FB_ARM."""
    man, s, dev = sil('rule', 40.0, rx=lambda clk: SyntheticReceiver(clk, pause_at=(15.0, 16.0)))
    assert man['result']['result'].startswith('vehicle'), man['result']
    assert man['result']['off_verified'] and not dev.armed
    assert sum(c.startswith('FB_ARM') for c in commands(dev)) == 1


def test_invalid_row_after_arm_is_fatal():
    man, s, dev = sil('rule', 40.0, rx=lambda clk: SyntheticReceiver(clk, bad_at=12.0))
    assert 'invalid_derivative' in man['result']['result'] and not dev.armed, man['result']


def test_impact_turns_off_and_ends_session():
    """FB V7 shock heuristic on the Pi path (rule and rl): 0.15 s planar shock -> FB_OFF, session ends, one FB_ARM."""
    if FB.impact is None: return                                           # FB V6: VehicleValidator 12 / 15 m/s^2 gate
    for mode, tune in (('rule', None), ('rl', REC_TUNE)):
        man, s, dev = sil(mode, 30.0, tune=tune, rx=lambda clk: SyntheticReceiver(clk, shock_at=(12.0, 12.15)))
        assert 'impact' in man['result']['result'] and man['result']['off_verified'] and not dev.armed, man['result']
        assert sum(c.startswith('FB_ARM') for c in commands(dev)) == 1


def test_wrong_steer_sign_stops_rl():
    if not HAVE_MODEL: raise unittest.SkipTest('requires an exported model: cold fallback does not use the RL sign monitor')
    man, s, dev = sil('rule', 60.0, rx=lambda clk: SyntheticReceiver(clk, steer_sign=-1))
    assert 'steer_sign_mismatch' in man['result']['result'], man['result']
    assert man['result']['off_verified']


def test_tune_yaw_max_clips_policy_and_board_never_faults():
    """STM yaw_max 6 deg: every target stays inside -> the board never sees CAD / TUNE bounds."""
    if not HAVE_MODEL: raise unittest.SkipTest('requires an exported model: tests the RL/rule envelope projection')
    man, s, dev = sil('rule', 25.0, tune=dict(yaw_max_mdeg=6000, yaw_rate_mdeg_s=20000))
    sets = [tuple(map(int, c.split(',')[3:])) for c in commands(dev) if c.startswith('FB_SET')]
    assert man['result']['result'] == 'completed' and max(max(p[0], p[2]) for p in sets) <= 6000
    assert man['board']['projected'] > 0


def test_actor_nan_hot_fallback_to_rule():
    if not HAVE_MODEL: raise unittest.SkipTest('requires an exported model for actor fault injection')
    cfg = cfg_for('rl', actuator_mismatch='run'); fb, s, dev = app.assemble(cfg, 'sil', QUIET)
    good = s.policy.actor
    class Bad:
        OD = good.OD if hasattr(good, 'OD') else None; A = getattr(good, 'A', 2); name = 'bad'; n = 0
        def __call__(self, x):
            Bad.n += 1; return good(x) if Bad.n < 200 else np.full(self.A, np.nan, np.float32)
    s.policy.actor = Bad()
    res = s.run(20.0, startup_timeout_s=30.0)
    ps = s.policy.summary()
    assert res['result'] == 'completed' and ps['mode_final'] == 'rule' and ps['actor_fallbacks'] == 1, (res, ps)


def test_actuator_check_downgrades_to_rule():
    """policy trained for another actuator, or STM tune slower than trained -> the tuned rule runs the session."""
    if not HAVE_MODEL: raise unittest.SkipTest('requires an exported model for actuator compatibility checks')
    for act, tune, want in ((dict(mode='fw_event_hb'), None, 'rule'),
                            (dict(mode='fb_a2', min_tune_rate_dps=10.0, yaw_caps_deg=[15.0, 20.0]), dict(yaw_rate_mdeg_s=2000), 'rule'),
                            (dict(mode='fb_a2', min_tune_rate_dps=10.0, yaw_caps_deg=[15.0, 20.0]), dict(yaw_rate_mdeg_s=20000), 'rl')):
        fb, s, dev = app.assemble(cfg_for('rl'), 'sil', QUIET, sil_tune=tune)
        s.policy.man = dict(s.policy.man, actuator=act)
        res = s.run(8.0, startup_timeout_s=30.0); ps = s.policy.summary()
        assert res['result'] == 'completed' and ps['mode_final'] == want, (act, tune, ps)


def test_cold_fallback_when_model_missing():
    cfg = cfg_for('rl'); cfg['model_dir'] = os.path.join(RT, 'no_such_model')
    man, s, dev = app.run(cfg, 'sil', 15.0, log=QUIET)
    assert man['policy'] == 'fb_v6' and man['result']['result'] == 'completed'
    cfg['control']['cold_fallback'] = None
    with unittest.TestCase().assertRaises(FileNotFoundError):
        app.run(cfg, 'sil', 5.0, log=QUIET)


def test_lease_expires_when_loop_stalls():
    """the Pi loop hangs 0.7 s: no PING -> the firmware's own lease switches PWM OFF; the host sees it and stops."""
    cfg = cfg_for('rule'); fb, s, dev = app.assemble(cfg, 'sil', QUIET)
    orig = s.policy.run
    def stalled(sess, duration):
        sess.wait(5.0); sess.clock_ns.sleep(0.7)                              # simulated hang, no pumping
        sess.wait(1.0); return None
    s.policy.run = stalled
    res = s.run(None, startup_timeout_s=30.0)
    assert dev.lease_faults == 1 and not dev.armed and res['off_verified'], (res, dev.fault)
    assert any(k in res['result'] for k in ('host_poll_gap', 'LEASE', 'session lost')), res
    s.policy.run = orig


def test_hardware_mode_refuses_without_confirmations():
    cfg = cfg_for()
    cfg['confirmations'] = {name: False for name in app.CONFIRM}
    with patch.object(C, 'load_config', return_value=cfg), patch.object(app, 'run') as run:
        with unittest.TestCase().assertRaisesRegex(SystemExit, 'confirm'):
            app.main(['--config', os.path.join(RT, 'config.yaml')])
        run.assert_not_called()


if __name__ == '__main__':
    fails = passed = skipped = 0
    print('COVERAGE', 'model-backed RL/rule SIL' if HAVE_MODEL else 'cold fallback SIL; model-dependent checks will skip')
    for name, f in sorted(globals().items()):
        if name.startswith('test_') and callable(f):
            try: f(); passed += 1; print('PASS', name)
            except unittest.SkipTest as e: skipped += 1; print('SKIP', name, str(e))
            except Exception as e: fails += 1; print('FAIL', name, type(e).__name__, e)
    print(f'SUMMARY passed={passed} skipped={skipped} failed={fails}')
    sys.exit(1 if fails else 0)
