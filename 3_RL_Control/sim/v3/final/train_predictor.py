"""Train the BeamNG 1 s a_y predictor (oasis_rt.predictor.BeamNGPredictor) in the final simulator.

Data: the v7 env itself (dynamic BeamNG-class car, scenario mix, road preview with its errors, bridge-grade
sensors), passive seat, levels 0-2. Each control step logs the predictor's 31 features (the same function the Pi
runs) and the car's true a_y 0.1 ... 1.0 s ahead. Seeds 20000+ train, 21000+ validation (disjoint from the
policy's training, validation 3000+ and eval bank 1000+).
Model: residual MLP 31 -> 64 -> 64 -> 10 on top of the wheel prior (weights = None gives the prior back).

    OASIS_ENV=v3 OASIS_CALIB=models/v7/calib_v2.json OASIS_L1_SEQ=models/v5/l1_seq.csv OASIS_DR_OVERLAY=$(cat sim/rl/runs/v7_env_overlays.txt) \
        python -P sim/v3/final/train_predictor.py --out models/v7
"""
import argparse, json, os, sys, time
import numpy as np
from multiprocessing import Pool

H = os.path.dirname(os.path.abspath(__file__)); sys.path[:0] = [os.path.join(H, '..', '..', 'rl'), os.path.join(H, '..')]
G = 9.81; HZ = np.arange(1, 11) * 0.1


def episode(seed):
    from common import BolsterEnv
    lv = [0.0, 1.0, 2.0][seed % 3]
    e = BolsterEnv(level=lv, seed=seed); e.reset(); e.wheel.W = None          # features only (prior output)
    X, Y, P, A = [], [], [], []
    while e.t < e.T - 1.1:
        o, r, te, tr, info = e.step(np.array([-1.0, 0.0]))
        X.append(e.wheel.x); P.append(e.wheel.x[:10]); A.append(e.imu_vals['ay'][0] / G)
        Y.append([e._ay(e.t + h) / G for h in HZ])
        if tr: break
    k = info['params']['kind']
    return np.array(X, np.float32), np.array(Y, np.float32), k


def gen(seeds, workers):
    with Pool(workers) as p: R = p.map(episode, seeds, chunksize=4)
    X = np.concatenate([r[0] for r in R]); Y = np.concatenate([r[1] for r in R])
    K = np.concatenate([[r[2]] * len(r[0]) for r in R])
    return X, Y, K


def train(X, Y, Xv, Yv, epochs=40, seed=0):
    import torch, torch.nn as nn
    torch.manual_seed(seed)
    mu, sd = X.mean(0), X.std(0) + 1e-6
    # binary / constant inputs keep a unit scale
    sd[sd < 1e-3] = 1.0
    f = lambda a: torch.as_tensor((a - mu) / sd)
    x, y, xv, yv = f(X), torch.as_tensor(Y - X[:, :10]), f(Xv), torch.as_tensor(Yv - Xv[:, :10])
    net = nn.Sequential(nn.Linear(X.shape[1], 64), nn.ReLU(), nn.Linear(64, 64), nn.ReLU(), nn.Linear(64, 10))
    opt = torch.optim.Adam(net.parameters(), 1e-3, weight_decay=1e-5); best, state = 1e9, None
    n = len(x)
    for ep in range(epochs):
        perm = torch.randperm(n)
        for i in range(0, n, 4096):
            b = perm[i:i + 4096]; loss = ((net(x[b]) - y[b]) ** 2).mean()
            opt.zero_grad(); loss.backward(); opt.step()
        with torch.no_grad(): v = float(((net(xv) - yv) ** 2).mean())
        if v < best: best, state = v, {k: t.clone() for k, t in net.state_dict().items()}
        if ep % 5 == 0 or ep == epochs - 1: print(f'  epoch {ep:2d} val rmse {np.sqrt(v):.4f} g (best {np.sqrt(best):.4f})', flush=True)
    net.load_state_dict(state)
    W = [(net[i].weight.detach().numpy().T.copy(), net[i].bias.detach().numpy().copy()) for i in (0, 2, 4)]
    return W, mu, sd


def predict(W, mu, sd, X):
    h = (X - mu) / sd
    for i, (w, b) in enumerate(W):
        h = h @ w + b
        if i < 2: h = np.maximum(h, 0)
    return X[:, :10] + h


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--out', required=True); ap.add_argument('--n', type=int, default=1200)
    ap.add_argument('--nval', type=int, default=240); ap.add_argument('--workers', type=int, default=10); ap.add_argument('--epochs', type=int, default=40)
    a = ap.parse_args(); t0 = time.time()
    X, Y, K = gen(list(range(20000, 20000 + a.n)), a.workers); Xv, Yv, Kv = gen(list(range(21000, 21000 + a.nval)), a.workers)
    print(f'data: train {len(X)} / val {len(Xv)} samples, {time.time() - t0:.0f} s', flush=True)
    W, mu, sd = train(X, Y, Xv, Yv, a.epochs)
    import yaml
    ov = [p for p in os.environ['OASIS_DR_OVERLAY'].split(os.pathsep) if p]
    wheel = {}
    for p in ov: wheel.update(((yaml.safe_load(open(p)) or {}).get('v3', {}).get('predictor') or {}).get('params', {}) or {})
    np.savez(os.path.join(a.out, 'predictor.npz'), W0=W[0][0], b0=W[0][1], W1=W[1][0], b1=W[1][1], W2=W[2][0], b2=W[2][1],
             x_mean=mu.astype(np.float32), x_std=sd.astype(np.float32), wheel_params=json.dumps(wheel))
    Pv = predict(W, mu, sd, Xv); hold = np.repeat(Xv[:, 21:22], 10, 1)          # x[21] = current a_y (x[20] = road_valid)
    rm = lambda P, m: [round(float(v), 4) for v in np.sqrt(np.mean((P[m] - Yv[m]) ** 2, 0))]
    ev = {}
    for name, m in [('all', np.ones(len(Xv), bool)), ('road_valid', Xv[:, 20] > 0.5), ('no_road', Xv[:, 20] < 0.5)] + \
                   [(f'kind={k}', Kv == k) for k in sorted(set(Kv))]:
        if m.sum() < 100: continue
        ev[name] = dict(n=int(m.sum()), learned=rm(Pv, m), wheel_prior=rm(Xv[:, :10], m), hold=rm(hold, m))
    json.dump(dict(h_s=HZ.round(1).tolist(), eval=ev, n_train=len(X), n_val=len(Xv), features=31, hidden=[64, 64], wall_s=round(time.time() - t0),
                   note='RMS error [g] of a_y(t+h) on validation seeds 21000+'), open(os.path.join(a.out, 'predictor_eval.json'), 'w'), indent=1)
    for k, v in ev.items():
        print(f'{k:24s} n {v["n"]:7d} | h 0.1/0.3/0.5/0.7/1.0  learned ' + ' '.join(f'{v["learned"][i]:.3f}' for i in (0, 2, 4, 6, 9)) +
              ' | wheel ' + ' '.join(f'{v["wheel_prior"][i]:.3f}' for i in (0, 2, 4, 6, 9)) + ' | hold ' + ' '.join(f'{v["hold"][i]:.3f}' for i in (0, 2, 4, 6, 9)))


if __name__ == '__main__':
    main()
