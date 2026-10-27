"""Actor backends for the Pi. All take the RAW stacked history (oldest first) and return [c, d] (v2)
or [c, d, cp, dp] (v3, + pitch).

  numpy : reference (bit-for-bit the simulator's evaluation policy, ~tens of us)
  c     : actor_core.c via ctypes (fp32, auto-vectorised; fastest on the Cortex-A76)
  hailo : MLP core compiled to a HEF (int8) on the AI HAT+ 2; normalisation, tanh and the
          residual prior stay on the CPU. For a 0.17 M-MAC MLP the PCIe/HailoRT call overhead is
          expected to exceed the compute -- make_actor() benchmarks and keeps the fastest
          backend that also passes the equivalence check, so Hailo is used only if it wins.
"""
import ctypes, os, time
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))


class History:
    def __init__(self, OD, K):
        self.OD, self.K = OD, K; self.h = np.zeros((K + 1, OD), np.float32)
    def reset(self, o):
        self.h[:] = 0; self.h[-1] = o; return self.h.reshape(-1)
    def push(self, o):
        self.h[:-1] = self.h[1:]; self.h[-1] = o; return self.h.reshape(-1)


class OutputFilter:
    """First-order low-pass on [c, d] (sim/v2: tau 0.3 s -> -71 % command jitter, same CoM effect)."""
    def __init__(self, tau_s, dt_s):
        self.a = 1.0 if tau_s <= 0 else min(1.0, dt_s / tau_s); self.y = None
    def reset(self): self.y = None
    def __call__(self, x):
        x = np.asarray(x, np.float32); self.y = x if self.y is None else self.y + self.a * (x - self.y); return self.y


class NumpyActor:
    name = 'numpy'
    def __init__(self, npz):
        z = np.load(npz)
        self.K, self.OD = int(z['K']), int(z['OD']); self.res, self.scale = bool(z['residual']), float(z['res_scale'])
        self.ff_gain, self.ff_idx = float(z['ff_gain']), int(z['ff_idx'])
        self.pitch_prior = float(z['pitch_prior']) if 'pitch_prior' in z.files else 0.0
        self.both = float(z['ff_both']) if 'ff_both' in z.files else 0.0; self.db = float(z['ff_deadband']) if 'ff_deadband' in z.files else 0.0
        self.mean = np.tile(z['mean'], self.K + 1).astype(np.float32); self.inv_std = (1.0 / np.tile(z['std'], self.K + 1)).astype(np.float32)
        self.L = [(z[f'W{i}'], z[f'b{i}']) for i in (0, 2, 4)]; self.A = self.L[-1][0].shape[1] // 2
    def normalise(self, x): return np.clip((x - self.mean) * self.inv_std, -5, 5).astype(np.float32)
    def core(self, z):
        for j, (W, b) in enumerate(self.L):
            z = z @ W + b
            if j < 2: z = np.maximum(z, 0)
        return z
    def head(self, y, x):
        A = len(y) // 2; mu = np.tanh(y[:A])                       # 2 (yaw) or 4 (yaw + pitch) actions
        if not self.res: return mu.astype(np.float32)
        ay = float(x[-self.OD + self.ff_idx]); m = max(abs(ay) - self.db, 0.0)
        prior = np.zeros(A, np.float32); prior[0] = np.clip(-1.0 + self.both * m, -1, 1); prior[1] = np.clip(-self.ff_gain * np.sign(ay) * m, -1, 1)
        if A == 4 and self.pitch_prior:                    # same as sim/rl/common.ff_baseline
            s = np.clip(-ay / 0.15, -1, 1); pL, pR = self.pitch_prior * max(s, 0.0), self.pitch_prior * max(-s, 0.0)
            prior[2], prior[3] = (pL + pR) / 2, (pL - pR) / 2
        return np.clip(prior + self.scale * mu, -1, 1).astype(np.float32)
    def __call__(self, x): return self.head(self.core(self.normalise(x)), x)


class CActor:
    name = 'c'
    def __init__(self, lib):
        self.lib = ctypes.CDLL(lib); f = self.lib.oasis_actor_forward
        f.argtypes = [ctypes.POINTER(ctypes.c_float)] * 2; f.restype = None
        n_in, n_out = ctypes.c_int(), ctypes.c_int(); self.OD = self.lib.oasis_actor_dims(ctypes.byref(n_in), ctypes.byref(n_out))
        self.n_in = n_in.value; self.A = n_out.value; self.out = np.zeros(self.A, np.float32)
        self._o = self.out.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
    def __call__(self, x):
        x = np.ascontiguousarray(x, np.float32)
        self.lib.oasis_actor_forward(x.ctypes.data_as(ctypes.POINTER(ctypes.c_float)), self._o)
        return self.out.copy()


class HailoActor:
    """HEF = the MLP core (normalised input [1, N_IN] -> 4 outputs). Requires HailoRT with the
    InferModel API (hailo_platform) on the Pi; written against the public API, NOT tested on a device."""
    name = 'hailo'
    def __init__(self, hef, npz):
        from hailo_platform import VDevice, FormatType            # noqa: F401  (import error -> backend unavailable)
        self.np = NumpyActor(npz)                                  # host-side normalisation / head
        self.vdev = VDevice(); self.model = self.vdev.create_infer_model(hef)
        self.model.input().set_format_type(FormatType.FLOAT32); self.model.output().set_format_type(FormatType.FLOAT32)
        self.cfg = self.model.configure()
        self.out = np.zeros(self.model.output().shape, np.float32)
        self.bind = self.cfg.create_bindings(); self.bind.output().set_buffer(self.out)
    def __call__(self, x):
        z = self.np.normalise(x).reshape(self.model.input().shape)
        self.bind.input().set_buffer(np.ascontiguousarray(z)); self.cfg.run([self.bind], 100)
        return self.np.head(self.out.reshape(-1)[:2 * self.np.A], x)


class RuleActor:
    """Tuned fallback rule (no network): the residual-RL prior alone, on the newest frame of the history.
    Same formula as sim/rl/common.ff_baseline and NumpyActor.head. Parameters: the model manifest (the prior the
    policy was trained around) or config control.rule (e.g. models/v6/rule.json best)."""
    name = 'rule'
    def __init__(self, OD, A, gain, lead_idx, both=0.0, deadband_g=0.0, pitch=0.0):
        self.OD, self.A, self.gain, self.idx, self.both, self.db, self.pitch = OD, A, float(gain), int(lead_idx), float(both), float(deadband_g), float(pitch)
    def __call__(self, x):
        ay = float(x[-self.OD + self.idx]); m = max(abs(ay) - self.db, 0.0)
        a = np.zeros(self.A, np.float32); a[0] = np.clip(-1.0 + self.both * m, -1, 1); a[1] = np.clip(-self.gain * np.sign(ay) * m, -1, 1)
        if self.A == 4 and self.pitch:
            s = np.clip(-ay / 0.15, -1, 1); pL, pR = self.pitch * max(s, 0.0), self.pitch * max(-s, 0.0)
            a[2], a[3] = (pL + pR) / 2, (pL - pR) / 2
        return a


def make_actor(cfg, log=print):
    """Instantiate the requested backends, check each against the numpy reference on stored
    calibration histories, benchmark, keep the fastest valid one."""
    ref = NumpyActor(cfg['npz'])
    xs = np.load(cfg['check_set']) if cfg.get('check_set') and os.path.exists(cfg['check_set']) else \
        (np.random.default_rng(0).normal(0, 1, (64, ref.K + 1, ref.OD)) * (1 / ref.inv_std[:ref.OD]) + ref.mean[:ref.OD]).reshape(64, -1).astype(np.float32)
    want = [ref(x) for x in xs[:64]]
    cands = []
    for b in cfg.get('backends', ['c', 'numpy']):
        try:
            a = ref if b == 'numpy' else CActor(cfg['c_lib']) if b == 'c' else HailoActor(cfg['hef'], cfg['npz'])
        except Exception as e:  # missing library / device
            log(f'actor backend {b}: unavailable ({type(e).__name__}: {e})'); continue
        err = max(float(np.abs(a(x) - w).max()) for x, w in zip(xs[:64], want))
        tol = cfg.get('tolerance', {}).get(b, 1e-4)
        t0 = time.perf_counter(); n = 300
        for i in range(n): a(xs[i % len(xs)])
        us = (time.perf_counter() - t0) / n * 1e6
        ok = err <= tol
        log(f'actor backend {b}: {us:.1f} us/call, max |a - ref| = {err:.2e} (tol {tol:g}) -> {"OK" if ok else "REJECTED"}')
        if ok: cands.append((us, a))
    if not cands: raise RuntimeError('no valid actor backend')
    return min(cands, key=lambda c: c[0])[1], ref


if __name__ == '__main__':          # on the Pi:  python -m oasis_rt.actor --backends hailo c numpy
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', default=os.path.join(HERE, '..', '..', 'model'))
    ap.add_argument('--backends', nargs='*', default=['hailo', 'c', 'numpy'])
    ap.add_argument('--c_lib', default=os.path.join(HERE, '..', 'build', 'liboasis_actor.so'))
    a = ap.parse_args()
    best, _ = make_actor(dict(npz=os.path.join(a.model, 'actor.npz'), check_set=os.path.join(a.model, 'check_set.npy'),
                              c_lib=a.c_lib, hef=os.path.join(a.model, 'actor_core.hef'), backends=a.backends,
                              tolerance={'numpy': 0.0, 'c': 1e-4, 'hailo': 0.1}))
    print('selected:', best.name)
