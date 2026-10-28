"""SAC with an asymmetric (privileged) critic for the O.A.S.I.S bolster env.

  actor  : last K sensor observations (normalised)            -> deployable on the Pi
  critic : privileged truth (occupant state, true future a_y,
           episode parameters) + current observation + action -> sim only
  extras : CAPS smoothness regularisation (temporal + spatial), optional residual
           mode on top of the predictive feed-forward rule, curriculum on DR level.

  python sac_asym.py --steps 300000 --tag plain
  python sac_asym.py --steps 300000 --tag resid --residual
"""
import argparse, json, os, time
import numpy as np
import torch, torch.nn as nn, torch.nn.functional as F

from common import BolsterEnv, History, RunningNorm, ff_baseline

p = argparse.ArgumentParser()
p.add_argument('--steps', type=int, default=300_000)
p.add_argument('--tag', default='plain')
p.add_argument('--residual', action='store_true')
p.add_argument('--res_scale', type=float, default=0.5)
p.add_argument('--ff_gain', type=float, default=6.0)        # residual prior: d = -gain * a_y_pred(lead)
p.add_argument('--ff_lead_idx', type=int, default=3)        # 3 -> 0.4 s ahead; 6 -> 0.7 s (slow servo, v2)
p.add_argument('--pitch_prior', type=float, default=0.0)     # v3: prior pitch of the loaded side (-0.5 = upper pad in)
p.add_argument('--ff_both', type=float, default=0.0)         # prior: common squeeze per g beyond the deadband
p.add_argument('--ff_deadband', type=float, default=0.0)     # prior: |a_y pred| deadband [g]
p.add_argument('--K', type=int, default=8)              # history length (0.4 s at 20 Hz)
p.add_argument('--gamma', type=float, default=0.98)     # ~2.5 s effective horizon
p.add_argument('--lr', type=float, default=3e-4)
p.add_argument('--batch', type=int, default=256)
p.add_argument('--hidden', type=int, default=256)
p.add_argument('--target_entropy', type=float, default=-2.0)
p.add_argument('--caps_t', type=float, default=0.5)     # temporal smoothness weight
p.add_argument('--caps_s', type=float, default=0.2)     # spatial smoothness weight
p.add_argument('--warmup', type=int, default=5000)
p.add_argument('--level_max', type=float, default=2.0)
p.add_argument('--level_ramp', type=float, default=0.6)  # fraction of training to reach level_max
p.add_argument('--no_priv', action='store_true')         # ablation: symmetric critic
p.add_argument('--seed', type=int, default=0)
p.add_argument('--threads', type=int, default=2)
# stability fixes after the early runs diverged in their last 50k steps (critic loss 7 -> 4600,
# alpha 0.017 -> 0.40): the optimal common clamp c = -1 sits on the tanh boundary, the naive
# log(1 - tanh^2 + eps) term then explodes, entropy looks collapsed, alpha jumps, targets break.
p.add_argument('--legacy', action='store_true')          # reproduce the earlier unstable setup
p.add_argument('--reward_scale', type=float, default=20.0)  # passive seat ~ -0.008/step is far below the entropy term
p.add_argument('--mu_reg', type=float, default=1e-3)      # keep pre-tanh means off the saturated tails
p.add_argument('--grad_clip', type=float, default=10.0)
p.add_argument('--log_alpha_max', type=float, default=0.0)  # alpha <= 1
p.add_argument('--snap_every', type=int, default=25)      # episodes between kept snapshots (checkpoint selection)
# v8 "start from the tuned rule and adjust" (residual mode): the critic first learns the rule's values with the
# actor frozen at zero correction, exploration starts small, and the correction is pulled toward zero
p.add_argument('--actor_delay', type=int, default=0)      # steps of critic-only updates (actor = prior rule + noise)
p.add_argument('--init_log_std', type=float, default=0.0)  # initial log std of the residual (0 = v7: std 1)
p.add_argument('--warmup_noise', type=float, default=0.3)  # residual noise during the warmup steps
p.add_argument('--res_l2', type=float, default=0.0)       # x |tanh(mu)|^2: keep the deterministic correction small
p.add_argument('--device', default='auto')                 # auto: cuda when this torch build has it (~80 % of a step is the update)
args = p.parse_args()
if args.legacy:
    args.reward_scale, args.mu_reg, args.grad_clip, args.log_alpha_max = 1.0, 0.0, 0.0, 10.0
torch.set_num_threads(args.threads); torch.manual_seed(args.seed); np.random.seed(args.seed)
DEV = torch.device('cuda' if args.device == 'auto' and torch.cuda.is_available() else 'cpu' if args.device == 'auto' else args.device)
cpu_state = lambda m: {k: v.detach().cpu() for k, v in m.state_dict().items()}   # checkpoints load on CPU-only torch (eval, Pi export)
OUT = os.path.join(os.path.dirname(__file__), 'runs', args.tag); os.makedirs(OUT, exist_ok=True)
json.dump(vars(args), open(os.path.join(OUT, 'args.json'), 'w'), indent=1)

env = BolsterEnv(level=0, seed=args.seed)
o0, _ = env.reset()
OD, AD, PD = env.obs_dim, env.action_dim, env.PRIV_DIM
hist = History(OD, args.K); XA = OD * (args.K + 1)
XC = (0 if args.no_priv else PD) + OD
norm_o, norm_p = RunningNorm(OD), RunningNorm(PD)


def mlp(i, o, h=args.hidden):
    return nn.Sequential(nn.Linear(i, h), nn.ReLU(), nn.Linear(h, h), nn.ReLU(), nn.Linear(h, o))


class Actor(nn.Module):
    def __init__(self):
        super().__init__(); self.net = mlp(XA, 2 * AD)
        if args.residual:   # start at "no correction" so the prior rule is the initial policy
            nn.init.zeros_(self.net[-1].weight); nn.init.zeros_(self.net[-1].bias)
            with torch.no_grad(): self.net[-1].bias[AD:] = args.init_log_std
    def forward(self, x):
        mu, ls = self.net(x).chunk(2, -1)
        return mu, ls.clamp(-5, 1)
    def sample(self, x, with_pre=False):
        mu, ls = self(x); std = ls.exp()
        u = mu + std * torch.randn_like(mu); a = torch.tanh(u)
        logp = (-0.5 * ((u - mu) / std) ** 2 - ls - 0.5 * np.log(2 * np.pi)).sum(-1)
        if args.legacy:
            logp -= torch.log(1 - a ** 2 + 1e-6).sum(-1)
        else:   # log(1 - tanh(u)^2) = 2 (log 2 - u - softplus(-2u)), exact and finite for any u
            logp -= (2 * (np.log(2) - u - F.softplus(-2 * u))).sum(-1)
        return (a, logp, torch.tanh(mu), mu) if with_pre else (a, logp, torch.tanh(mu))


class Critic(nn.Module):
    def __init__(self):
        super().__init__(); self.q1, self.q2 = mlp(XC + AD, 1), mlp(XC + AD, 1)
    def forward(self, x, a):
        xa = torch.cat([x, a], -1); return self.q1(xa).squeeze(-1), self.q2(xa).squeeze(-1)


actor, critic, critic_t = Actor().to(DEV), Critic().to(DEV), Critic().to(DEV)
critic_t.load_state_dict(critic.state_dict())
log_alpha = torch.tensor(np.log(0.1), device=DEV, requires_grad=True)
oa, oc, oal = (torch.optim.Adam(m, lr=args.lr) for m in (actor.parameters(), critic.parameters(), [log_alpha]))

N = args.steps
# histories are rebuilt from the flat obs sequence at sample time (memory: N x OD, not N x K x OD)
B = dict(o=np.zeros((N, OD), np.float32), o2=np.zeros((N, OD), np.float32),
         start=np.zeros(N, np.int64),
         pr=np.zeros((N, PD), np.float32), pr2=np.zeros((N, PD), np.float32),
         a=np.zeros((N, AD), np.float32), r=np.zeros(N, np.float32))


def actor_in(xa_raw):   # normalise each stacked frame with the obs normaliser
    x = xa_raw.reshape(*xa_raw.shape[:-1], args.K + 1, OD)
    return ((x - norm_o.mean) / norm_o.std).clip(-5, 5).reshape(xa_raw.shape).astype(np.float32)


def critic_in(o_raw, pr_raw):
    on = ((o_raw - norm_o.mean) / norm_o.std).clip(-5, 5)
    if args.no_priv: return on.astype(np.float32)
    pn = ((pr_raw - norm_p.mean) / norm_p.std).clip(-5, 5)
    return np.concatenate([pn, on], -1).astype(np.float32)


def env_action(a_pol, o):
    if args.residual:
        return np.clip(ff_baseline(o, args.ff_gain, args.ff_lead_idx, AD, args.pitch_prior, args.ff_both, args.ff_deadband)
                       + args.res_scale * a_pol, -1, 1)
    return a_pol


def windows(idx):
    """History windows ending at i (actor input at i) and at i+1 (= next obs)."""
    off = np.arange(-args.K, 1)
    j = idx[:, None] + off[None, :]                       # (b, K+1)
    valid = j >= B['start'][idx][:, None]
    w = B['o'][np.clip(j, 0, None)] * valid[..., None]
    w2 = np.concatenate([w[:, 1:], B['o2'][idx][:, None]], 1)
    return w.reshape(len(idx), -1), w2.reshape(len(idx), -1)


def update(actor_on=True):
    idx = np.random.randint(0, n_stored, args.batch)
    T = lambda z: torch.as_tensor(z, device=DEV)
    w, w2 = windows(idx)
    xa, xa2 = T(actor_in(w)), T(actor_in(w2))
    xc, xc2 = T(critic_in(B['o'][idx], B['pr'][idx])), T(critic_in(B['o2'][idx], B['pr2'][idx]))
    a, r = T(B['a'][idx]), T(B['r'][idx])
    alpha = log_alpha.exp().detach()
    with torch.no_grad():
        a2, lp2, _ = actor.sample(xa2)
        q1t, q2t = critic_t(xc2, a2)
        y = r + args.gamma * (torch.min(q1t, q2t) - alpha * lp2)   # no terminals: time-limit only
    q1, q2 = critic(xc, a)
    lc = F.mse_loss(q1, y) + F.mse_loss(q2, y)
    oc.zero_grad(); lc.backward()
    if args.grad_clip > 0: nn.utils.clip_grad_norm_(critic.parameters(), args.grad_clip)
    oc.step()
    if not actor_on:                                       # --actor_delay: policy evaluation of the prior rule first
        soft_update()
        return lc.item(), 0.0, alpha.item()
    an, lp, mu, mu_pre = actor.sample(xa, with_pre=True)
    la = (alpha * lp - torch.min(*critic(xc, an))).mean() + args.mu_reg * (mu_pre ** 2).sum(-1).mean()
    if args.res_l2: la = la + args.res_l2 * (mu ** 2).sum(-1).mean()
    # CAPS (Mysore et al. 2021): smooth mean action over time and under small obs noise
    _, _, mu2 = actor.sample(xa2)
    _, _, mus = actor.sample(xa + 0.05 * torch.randn_like(xa))
    la = la + args.caps_t * ((mu - mu2.detach()) ** 2).sum(-1).mean() + args.caps_s * ((mu - mus) ** 2).sum(-1).mean()
    oa.zero_grad(); la.backward()
    if args.grad_clip > 0: nn.utils.clip_grad_norm_(actor.parameters(), args.grad_clip)
    oa.step()
    lal = -(log_alpha * (lp.detach() + args.target_entropy)).mean()
    oal.zero_grad(); lal.backward(); oal.step()
    with torch.no_grad(): log_alpha.clamp_(max=args.log_alpha_max)
    soft_update()
    return lc.item(), la.item(), alpha.item()


@torch.no_grad()
def soft_update(tau=0.005):   # target <- (1 - tau) target + tau critic, one fused kernel per device (GPU: launch-bound)
    torch._foreach_lerp_(list(critic_t.parameters()), list(critic.parameters()), tau)


print('device', DEV, torch.cuda.get_device_name(DEV) if DEV.type == 'cuda' else '', flush=True)
log = open(os.path.join(OUT, 'train_log.csv'), 'w')
log.write('step,episode,level,ret,rms_cm,trips,alpha,lc,la,wall\n')
t0 = time.time(); n_stored = 0; ep = 0; ep_start = 0
o, _ = env.reset(); xa = hist.reset(o); pr = env.privileged_vec()
ep_ret, ep_com, ep_trips, stats = 0.0, [], 0, (0, 0, 0)
for step in range(N):
    norm_o.update(o); norm_p.update(pr)
    if step < args.warmup:
        a_pol = np.zeros(AD) + args.warmup_noise * np.random.randn(AD) if args.residual else np.random.uniform(-1, 1, AD)
        a_pol = np.clip(a_pol, -1, 1)
    else:
        with torch.no_grad():
            a_pol = actor.sample(torch.as_tensor(actor_in(xa[None]), device=DEV))[0][0].cpu().numpy()
    o2, r, te, tr, info = env.step(env_action(a_pol, o))
    xa2 = hist.push(o2); pr2 = env.privileged_vec()
    k = n_stored
    B['o'][k], B['o2'][k], B['start'][k] = o, o2, ep_start
    B['pr'][k], B['pr2'][k], B['a'][k], B['r'][k] = pr, pr2, a_pol, r * args.reward_scale
    n_stored += 1
    ep_ret += r; ep_com.append(info['privileged']['y_com']); ep_trips += info.get('trip', 0)
    o, xa, pr = o2, xa2, pr2
    if step >= args.warmup:
        stats = update(actor_on=step >= args.actor_delay)
    if tr:
        ep += 1
        rms = 100 * np.sqrt(np.mean(np.square(ep_com)))
        log.write(f'{step},{ep},{env.level:.2f},{ep_ret:.3f},{rms:.3f},{ep_trips:.0f},{stats[2]:.4f},{stats[0]:.4f},{stats[1]:.4f},{time.time()-t0:.0f}\n'); log.flush()
        env.level = min(args.level_max, args.level_max * step / (args.level_ramp * N))
        o, _ = env.reset(); xa = hist.reset(o); pr = env.privileged_vec()
        ep_ret, ep_com, ep_trips = 0.0, [], 0; ep_start = step + 1
        if ep % 50 == 0:
            torch.save(dict(actor=cpu_state(actor), norm=norm_o.state(), args=vars(args), OD=OD, step=step),
                       os.path.join(OUT, 'actor.pt'))
        if ep % args.snap_every == 0 and step > args.warmup:
            os.makedirs(os.path.join(OUT, 'snap'), exist_ok=True)
            torch.save(dict(actor=cpu_state(actor), norm=norm_o.state(), args=vars(args), OD=OD, step=step),
                       os.path.join(OUT, 'snap', f'actor_{step:07d}.pt'))
torch.save(dict(actor=cpu_state(actor), norm=norm_o.state(), args=vars(args), OD=OD, step=N),
           os.path.join(OUT, 'actor.pt'))
torch.save(dict(critic=cpu_state(critic), normp=norm_p.state()), os.path.join(OUT, 'critic.pt'))
print('done', args.tag, 'episodes', ep, 'wall', round(time.time() - t0), 's')
