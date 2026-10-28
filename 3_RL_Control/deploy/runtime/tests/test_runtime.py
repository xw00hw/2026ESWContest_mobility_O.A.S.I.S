"""Runtime tests: protocol, IK projection, interlock, actor backends, closed-loop SIL with MockStm32.

    python -m pytest deploy/runtime/tests -q        (or: python deploy/runtime/tests/test_runtime.py)
"""
import copy, json, os, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RT = os.path.abspath(os.path.join(HERE, '..'))
ROOT = os.path.abspath(os.path.join(RT, '..', '..'))
sys.path[:0] = [RT, HERE, os.path.join(ROOT, 'sim', 'rl')]
from oasis_rt import kinematics as K, stm32_link as SL, controller as C, sources as SRC  # noqa: E402
from oasis_rt.actor import NumpyActor, CActor, make_actor  # noqa: E402
from oasis_rt.safety import Interlock, FsrInverse  # noqa: E402
from mock_stm32 import MockClock, MockStm32  # noqa: E402

CFG = C.load_config(os.path.join(RT, 'config.yaml'))
MODEL = CFG['model_dir']


def cfg_for(**over):
    c = copy.deepcopy(CFG)
    c['predictor'] = {'kind': 'sequence'}; c['actor']['backends'] = ['numpy']
    for k, v in over.items():
        sec, key = k.split('__'); c[sec][key] = v
    return c


# ------------------------------------------------------------------ protocol / IK
def test_hello_and_state_parse():
    clk = MockClock(); dev = MockStm32(clk, crank_rate=60.0)
    dev.write_line('CONTROL_HELLO'); hello = [l for l in dev.read_lines() if l.startswith('#HELLO')][0]
    caps = SL.parse_hello(hello); assert caps['motor_rate_mdeg_s'] == 60000      # faster firmware accepted
    try:
        SL.parse_hello(hello.replace('ik_revision=8', 'ik_revision=7')); assert False, 'IK revision mismatch not caught'
    except RuntimeError:
        pass
    dev.write_line('STATE'); st = SL.parse_state(dev.read_lines()[-1].strip())
    assert st['fault'] == 'BOOT' and st['armed'] == 0
    assert SL.parse_pressure('12,345,2,1000,0,,1,30,0') == (12, 345, 2, [1000, None, 30])


def test_projection_always_reachable():
    rng = np.random.default_rng(0)
    for y, p in rng.uniform([-5, -12], [30, 12], (3000, 2)):
        y2, p2, _ = K.project(y, p, 20.0)
        assert 0 <= y2 <= 20 and K.pulses_ok(y2, p2), (y, p, y2, p2)
    assert K.project(10.0, 0.0, 20.0) == (10.0, 0.0, False)


# ------------------------------------------------------------------ interlock
def test_interlock_trip_hold_and_inertial_load():
    il = Interlock(CFG['safety'], 221.9)
    out = il(0.0, [10.0, 10.0], (10.0, 10.0), (130.0, 0.0), 0.0)               # 130 N on the left wing
    assert out[0] == 0.0 and out[1] == 10.0 and il.flag == 1.0
    out = il(0.1, [10.0, 10.0], (5.0, 10.0), (0.0, 0.0), 0.0)                  # still inside hold_s
    assert out[0] == 0.0
    il2 = Interlock(CFG['safety'], 221.9)
    out = il2(0.0, [15.0, 15.0], (8.0, 8.0), (0.0, 80.0), 0.0)                  # 80 N right: soft hold
    assert out[1] == 8.0 and il2.flag == 0.5
    il3 = Interlock(CFG['safety'], 221.9)                                       # same 80 N while a_y = +0.2 g loads it
    out = il3(0.0, [15.0, 15.0], (8.0, 8.0), (0.0, 80.0), 0.2)
    assert out[1] == 15.0 and il3.flag == 0.0


# ------------------------------------------------------------------ actor backends
def test_backends_match_training_policy():
    from deploy_actor import DeployedActor
    X = np.load(os.path.join(MODEL, 'check_set.npy'))
    ref = DeployedActor(os.path.join(MODEL, 'actor.npz'))
    na = NumpyActor(os.path.join(MODEL, 'actor.npz'))
    assert max(np.abs(na(x) - ref.act(x)).max() for x in X) < 1e-5
    lib = CFG['actor']['c_lib']
    if sys.platform.startswith('linux') and os.path.exists(lib):
        ca = CActor(lib); assert max(np.abs(ca(x) - ref.act(x)).max() for x in X) < 1e-4
    best, _ = make_actor(dict(CFG['actor'], npz=os.path.join(MODEL, 'actor.npz')), log=lambda *a: None)
    assert best.name in ('c', 'numpy', 'hailo')


# ------------------------------------------------------------------ closed loop (SIL)
def run_sil(seconds=40.0, **over):
    cfg = cfg_for(**over)
    ctl, link, dev, sleep = C.build(cfg, mock=True, log=lambda *a: None)
    link.connect(); ctl.zero_pressure(wait=sleep); link.arm()
    err = []
    def w(row, busy, late):
        now, o, a_raw, a, cmd, sent, rep, flag, force = row
        if now > 5: err.append((sent[0] - dev.side[0].pose[0], sent[2] - dev.side[1].pose[0]))
    res = ctl.run(seconds, w, sleep)
    e = np.array(err)
    return dict(res, sent=link.sent, rejected=link.rejected, fault=link.fault, dev_fault=dev.fault,
                track_rms_deg=float(np.sqrt(np.mean(e ** 2))) if len(e) else None), ctl, link, dev


def test_sil_current_firmware_runs_clean():
    r, ctl, link, dev = run_sil(40.0, link__send_mode='event', link__mock_crank_rate_dps=15.0)
    assert r['fault'] is None and r['rejected'] == 0 and r['overruns'] == 0, r
    assert r['sent'] <= 40.0 / CFG['link']['min_interval_s'] + 5, r                # never streams SET
    assert ctl.state == 'RUN'


def test_sil_faster_actuator_tracks_better():
    slow, *_ = run_sil(40.0, link__send_mode='event', link__mock_crank_rate_dps=15.0)
    fast, *_ = run_sil(40.0, link__send_mode='track', link__mock_crank_rate_dps=60.0)
    assert fast['fault'] is None and fast['track_rms_deg'] < slow['track_rms_deg'], (slow, fast)


MODEL_V3 = os.environ.get('OASIS_TEST_MODEL_V3') or CFG['model_dir']        # v3-contract model (config model_dir: the deployed one)


def test_sil_v3_model():
    """v3 contract model: pad zones, rest pressures; a 4-action model commands pitch within +-8 deg,
    a 2-action (yaw-only) model holds pitch at exactly 0. No faults / rejects."""
    if not os.path.exists(os.path.join(MODEL_V3, 'manifest.json')):
        print('  (skipped: no v3 model at', MODEL_V3, ')'); return
    cfg = cfg_for(link__send_mode='event'); cfg['model_dir'] = os.path.abspath(MODEL_V3)
    cfg['actor']['check_set'] = os.path.join(MODEL_V3, 'check_set.npy')
    ctl, link, dev, sleep = C.build(cfg, mock=True, log=lambda *a: None)
    link.connect(); ctl.zero_pressure(wait=sleep); link.arm()
    rows = []; ctl.run(40.0, lambda row, b, l: rows.append(row), sleep)
    cmd = np.array([r[4] for r in rows])
    assert ctl.v3 and link.fault is None and link.rejected == 0
    if ctl.A == 4: assert np.all(np.abs(cmd[:, 2:]) <= 8.0 + 1e-9) and np.ptp(cmd[:, 2:]) > 0
    else: assert np.all(cmd[:, 2:] == 0.0) and np.ptp(cmd[:, :2]) > 0


def test_stale_vehicle_opens_bolsters():
    cfg = cfg_for()
    ctl, link, dev, sleep = C.build(cfg, mock=True, log=lambda *a: None)
    link.connect(); ctl.zero_pressure(wait=sleep); link.arm()
    ctl.run(10.0, None, sleep, hold_on_exit=False)
    ctl.vehicle = SRC.NoPose()                                                  # vehicle stream lost
    ctl.run(3.0, None, sleep)
    assert ctl.state == 'STALE' and link.last_sent[0] == 0 and link.last_sent[2] == 0
    assert max(abs(dev.side[0].pose[0]), abs(dev.side[1].pose[0])) < 0.5 and link.fault is None


def test_ping_alone_keeps_mcu_armed():
    clk = MockClock(); dev = MockStm32(clk); link = SL.Stm32Link(dev, CFG['link'], clk)
    link.connect(); link.arm(); link.send_targets(5.0, 0.0, 5.0, 0.0)
    for _ in range(100):                                                        # 5 s, same target -> no SET
        dev.advance(0.05); link.poll(); link.send_targets(5.0, 0.0, 5.0, 0.0)
    assert dev.armed and link.fault is None and link.sent == 1


def test_fsr_inverse_and_saturation_guard():
    inv = FsrInverse(os.path.join(RT, '..', 'tools', 'fsr_curve.json'))
    F = [inv(c) for c in (500, 1000, 1450, 1700, 1850)]
    assert all(b > a for a, b in zip(F, F[1:])) and abs(inv(1001) - 4.3) < 0.6        # fitted table point
    assert inv(4095) == float('inf') and inv(0) == 0.0
    # SIL: a saturated right pad (v2 wing R = A1_C1) must keep the right bolster (almost) open
    import tempfile
    seq = os.path.join(tempfile.mkdtemp(), 'steady_right_load.csv')          # steady 0.3 g left turn: loads the RIGHT wing
    t = np.arange(0, 60, 0.1)
    pd_ = __import__('pandas'); pd_.DataFrame(dict(t=t, ax=0 * t, ay=np.minimum(t / 3, 1) * 2.94, yaw=0.2 + 0 * t, v=15 + 0 * t, sw=0.4 + 0 * t)).to_csv(seq, index=False)

    def frac_closed(sat):
        cfg = cfg_for(); cfg['safety']['pad_saturation_counts'] = sat; cfg['vehicle']['replay'] = seq
        ctl, link, dev, sleep = C.build(cfg, mock=True, log=lambda *a: None)
        link.connect(); ctl.zero_pressure(wait=sleep); link.arm()
        dev.pressure_fn = lambda t: {f'A{b}_C{c}': (2300 if (b, c) == (1, 1) else 1000) for b in range(3) for c in range(4)}
        rows = []; ctl.run(30.0, lambda row, b_, l: rows.append(row), sleep)
        return float(np.mean([r[6][2] > CFG['safety']['arm_deg'] + 1.0 for r in rows]))
    on, off = frac_closed(2200), frac_closed(None)
    assert on < 0.1 and on < off, (on, off)


def test_actor_nan_falls_back_to_rule():
    """actor output NaN -> latched switch to the tuned rule (= the model's prior); session keeps running."""
    cfg = cfg_for()
    ctl, link, dev, sleep = C.build(cfg, mock=True, log=lambda *a: None)
    link.connect(); ctl.zero_pressure(wait=sleep); link.arm(); ctl.run(5.0, None, sleep, hold_on_exit=False)
    good = ctl.actor; ctl.actor = lambda x: np.full(ctl.A, np.nan, np.float32)
    rows = []; ctl.run(10.0, lambda row, b, l: rows.append(row), sleep, hold_on_exit=False)
    assert ctl.mode == 'rule' and ctl.fallbacks == 1 and link.fault is None and ctl.state == 'RUN'
    a_raw = np.array([r[2] for r in rows]); assert np.all(np.isfinite(a_raw))
    assert np.allclose(a_raw[-1], ctl.rule(rows[-1][1]), atol=1e-6)        # the rule reads the newest frame only
    ctl.actor = good


def test_rule_mode_without_actor_backend():
    cfg = cfg_for(); cfg['actor']['backends'] = ['hailo']; cfg['actor']['hef'] = '/nonexistent.hef'
    ctl, link, dev, sleep = C.build(cfg, mock=True, log=lambda *a: None)
    link.connect(); ctl.zero_pressure(wait=sleep); link.arm(); ctl.run(20.0, None, sleep)
    assert ctl.mode == 'rule' and link.fault is None and link.sent > 0


def test_static_bench_interlock_raw():
    cfg = cfg_for(); cfg['rig'] = 'static_bench'
    ctl, *_ = C.build(cfg, mock=True, log=lambda *a: None)
    assert not ctl.ilock.model                      # no inertial-load subtraction on a bench that does not move
    il = Interlock(dict(CFG['safety'], mode='raw'), CFG['console_N_per_g'])
    assert il(0.0, (10.0, 10.0), (10.0, 10.0), (0.0, 130.0), 0.5)[1] == 0.0   # 130 N at a_y 0.5 g: trips (model: 19 N)


def test_wheel_preview_leads_hold():
    """step of the raw wheel while the car's a_y is still 0: the preview rises with the yaw lag; hold stays 0."""
    from oasis_rt.predictor import WheelPreviewPredictor, HoldPredictor
    w = WheelPreviewPredictor(steer_tau_s=0.3); s = dict(t=0.0, steer_rad=0.0, speed_mps=15.0, ay_mps2=0.0, ax_mps2=0.0)
    w(s); ay, _ = w(dict(s, t=0.05, steer_input_rad=0.5))          # 0.5 rad wheel (left) -> a_ss ~ 0.3 g
    a_ss = w.steady_ay(0.5, 15.0) / 9.81
    assert 0 < ay[0] < ay[2] < ay[-1] and ay[-1] > 0.9 * a_ss and HoldPredictor()(s)[0][-1] == 0.0


def test_beamng_predictor_from_manifest():
    """v7: predictor kind 'model' loads the learned BeamNG preview shipped in the model dir; without weights the
    class returns exactly the wheel prior; a SIL run with it stays clean."""
    from oasis_rt.predictor import BeamNGPredictor, WheelPreviewPredictor
    s = dict(t=0.0, steer_rad=0.0, steer_input_rad=0.3, speed_mps=14.0, ay_mps2=0.5, ax_mps2=0.0, yaw_rate_radps=0.02,
             road_ay_g=[0.05 * k for k in range(10)], road_valid=1.0)
    b, w = BeamNGPredictor(steer_tau_s=0.3), WheelPreviewPredictor(steer_tau_s=0.3)
    assert np.allclose(b(s)[0], w(s)[0]) and len(b.x) == BeamNGPredictor.NF
    man = os.path.join(MODEL_V3, 'manifest.json')
    if not os.path.exists(man) or (json.load(open(man)).get('predictor') or {}).get('kind') != 'beamng':
        print('  (skipped: no v7 model with a beamng predictor at', MODEL_V3, ')'); return
    cfg = cfg_for(link__send_mode='event'); cfg['model_dir'] = os.path.abspath(MODEL_V3); cfg['predictor'] = {'kind': 'model'}
    cfg['actor']['check_set'] = os.path.join(MODEL_V3, 'check_set.npy'); cfg['rig'] = 'static_bench'
    ctl, link, dev, sleep = C.build(cfg, mock=True, log=lambda *a: None)
    assert isinstance(ctl.pred, BeamNGPredictor) and ctl.pred.W is not None
    link.connect(); ctl.zero_pressure(wait=sleep); link.arm(); ctl.run(20.0, None, sleep)
    assert link.fault is None and link.rejected == 0 and ctl.mode == 'rl'


def test_heartbeat_loss_faults_mcu():
    cfg = cfg_for()
    ctl, link, dev, sleep = C.build(cfg, mock=True, log=lambda *a: None)
    link.connect(); ctl.zero_pressure(wait=sleep); link.arm(); ctl.run(3.0, None, sleep, hold_on_exit=False)
    dev.advance(1.6)                                                            # Pi hangs for 1.6 s
    assert dev.fault == 'TIMEOUT' and not dev.armed
    ctl.tick(); assert ctl.state == 'FAULT'                                     # host sees it, does not re-arm


if __name__ == '__main__':
    fails = 0
    for name, f in sorted(globals().items()):
        if name.startswith('test_') and callable(f):
            try: f(); print('PASS', name)
            except Exception as e: fails += 1; print('FAIL', name, type(e).__name__, e)
    r, *_ = run_sil(60.0, link__send_mode='event', link__mock_crank_rate_dps=15.0); print('SIL event@15  :', r)
    r, *_ = run_sil(60.0, link__send_mode='event', link__mock_crank_rate_dps=60.0); print('SIL event@60  :', r)
    r, *_ = run_sil(60.0, link__send_mode='event', link__mock_crank_rate_dps=15.0, link__keepalive_s=1.0); print('SIL event@15 +keepalive SET:', r)
    r, *_ = run_sil(60.0, link__send_mode='track', link__mock_crank_rate_dps=60.0); print('SIL track@60  :', r)
    sys.exit(1 if fails else 0)
