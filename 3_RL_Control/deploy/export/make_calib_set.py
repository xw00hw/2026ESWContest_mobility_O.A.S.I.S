"""Realistic observation histories for (a) the runtime backend equivalence check and (b) the Hailo
DFC quantisation calibration. Rolled out in sim/v2 with the deployed policy itself (filter 0.3 s)
on seeds 5000+ (disjoint from training, validation 3000+ and the eval bank 1000+).

    python deploy/export/make_calib_set.py --model deploy/model --episodes 24

writes into --model:
  check_set.npy        512 x N_IN raw histories (runtime make_actor check)
  calib_raw.npy        N x N_IN raw histories
  calib_norm_fc.npy    N x N_IN normalised + clipped (input of actor_core_fc.onnx)
  calib_norm_nhwc.npy  N x 1 x 1 x N_IN (Hailo DFC calibration layout for actor_core_conv.onnx)
  calib_actions.npy    N x 2 reference actions (unfiltered actor output)
"""
import argparse, os, sys
import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))


def _env_for_model():                    # OD 46 -> sim/v2, OD 57 -> sim/v3 (pitch); decided before common is imported
    import numpy as _np
    m = sys.argv[sys.argv.index('--model') + 1] if '--model' in sys.argv else os.path.join(ROOT, 'deploy', 'model')
    return {46: 'v2', 57: 'v3'}[int(_np.load(os.path.join(m, 'actor.npz'))['OD'])]


os.environ['OASIS_ENV'] = _env_for_model()
sys.path.insert(0, os.path.join(ROOT, 'sim', 'rl'))
from common import BolsterEnv, History, EVAL_KINDS  # noqa: E402
from deploy_actor import DeployedActor  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', default=os.path.join(ROOT, 'deploy', 'model'))
    ap.add_argument('--episodes', type=int, default=24)
    ap.add_argument('--every', type=int, default=3, help='keep every n-th control tick')
    a = ap.parse_args()
    pol = DeployedActor(os.path.join(a.model, 'actor.npz'), filter_tau=0.3)
    X, A = [], []
    for e in range(a.episodes):
        kind = EVAL_KINDS[e % len(EVAL_KINDS)]; level = [0, 1, 2][e % 3]
        env = BolsterEnv(level=level, seed=5000 + e, scenario_kind=kind)
        o, _ = env.reset(); h = History(pol.OD, pol.K); x = h.reset(o); act = pol.reset(o); k = 0
        while True:
            if k % a.every == 0: X.append(x.copy()); A.append(pol.act(x))
            o, r, te, tr, info = env.step(act); k += 1
            x = h.push(o); act = pol.step(o)
            if tr: break
        print(f'episode {e}: {kind} level {level}, {k} ticks, total {len(X)}')
    X = np.asarray(X, np.float32); A = np.asarray(A, np.float32)
    Z = np.clip((X - pol.mean) / pol.std, -5, 5).astype(np.float32)
    rng = np.random.default_rng(0); idx = rng.permutation(len(X))
    np.save(os.path.join(a.model, 'check_set.npy'), X[idx[:512]])
    np.save(os.path.join(a.model, 'calib_raw.npy'), X)
    np.save(os.path.join(a.model, 'calib_norm_fc.npy'), Z)
    np.save(os.path.join(a.model, 'calib_norm_nhwc.npy'), Z[:, None, None, :])
    np.save(os.path.join(a.model, 'calib_actions.npy'), A)
    print(f'{len(X)} histories; clipped at +-5: {100 * np.mean(np.abs(Z) >= 5):.3f} % of inputs')
    rehash(a.model, ('check_set.npy', 'calib_raw.npy', 'calib_norm_fc.npy', 'calib_norm_nhwc.npy', 'calib_actions.npy'))


def rehash(model, names):
    """manifest.json lists the sha256 of every package file (deploy/tools/pack_model.py checks them): add these."""
    import hashlib, json
    mp = os.path.join(model, 'manifest.json'); man = json.load(open(mp))
    for f in names: man['files'][f] = hashlib.sha256(open(os.path.join(model, f), 'rb').read()).hexdigest()
    json.dump(man, open(mp, 'w'), indent=1)


if __name__ == '__main__':
    main()
