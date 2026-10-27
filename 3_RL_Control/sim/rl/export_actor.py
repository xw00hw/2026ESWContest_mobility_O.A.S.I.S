"""Export a trained actor for the Pi.

Primary: <ckpt>.npz with weights + normaliser + residual settings, run by
`deploy_actor.py` using numpy only (no torch / onnx on the Pi; a 2x256 MLP is
~0.1 ms). Optional: ONNX (needs a working `onnx` package; this PC's is broken).

  python export_actor.py runs/resid/actor.pt [--onnx]
"""
import sys, time, os
import numpy as np, torch
from evaluate import ActorPolicy
from deploy_actor import DeployedActor

if __name__ == '__main__':
    ck = sys.argv[1]; pol = ActorPolicy(ck)
    W = {f'W{i}': l.weight.detach().numpy().T.astype(np.float32) for i, l in enumerate(pol.net) if hasattr(l, 'weight')}
    B = {f'b{i}': l.bias.detach().numpy().astype(np.float32) for i, l in enumerate(pol.net) if hasattr(l, 'bias')}
    out = os.path.splitext(ck)[0] + '.npz'
    np.savez(out, K=pol.K, OD=pol.OD, residual=pol.res, res_scale=pol.scale, ff_gain=pol.ff[0], ff_idx=pol.ff[1], pitch_prior=pol.pitch_prior, ff_both=pol.both, ff_deadband=pol.db,
             mean=pol.norm.mean.astype(np.float32), std=pol.norm.std.astype(np.float32), **W, **B)
    dep = DeployedActor(out)
    # equivalence with the evaluation policy on random histories
    rng = np.random.default_rng(0); err = 0.0
    for _ in range(300):
        h = (rng.normal(0, 1, (pol.K + 1, pol.OD)) * pol.norm.std + pol.norm.mean).astype(np.float32)
        pol.hist.q.clear(); [pol.hist.q.append(f) for f in h]
        a_ref = pol(h[-1], None); a_dep = dep.act(h.reshape(-1))
        err = max(err, float(np.abs(a_ref - a_dep).max()))
    t0 = time.perf_counter()
    for _ in range(5000): dep.act(h.reshape(-1))
    us = (time.perf_counter() - t0) / 5000 * 1e6
    print(f'{out}: {os.path.getsize(out) / 1024:.0f} kB, max |deployed - eval| = {err:.1e}, {us:.0f} us/inference (this PC, numpy)')
    if '--onnx' in sys.argv:
        print('ONNX export requires a working onnx package: pip install --upgrade onnx')
