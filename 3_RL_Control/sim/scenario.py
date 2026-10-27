"""Vehicle excitation generator: L1 log replay + synthetic manoeuvres through a
kinematic-with-lag bicycle model. Returns time series at dt_physics."""
import os
import numpy as np
import pandas as pd

G = 9.81
_L1 = None
_SEQ = {}
SEQ_DIR = os.path.join(os.path.dirname(__file__), '..', 'data_beamng')


def _l1():
    global _L1
    if _L1 is None:
        p = os.path.join(os.path.dirname(__file__), '..', 'analysis', 'grid_10hz.csv')
        d = pd.read_csv(p)
        _L1 = d[['t', 'ax', 'ay', 'wz', 'v_gps']].interpolate().bfill().to_numpy()
    return _L1


def bicycle(sw, v, vp, dt):
    """sw: steering-wheel angle [rad]; v: speed [m/s] -> yaw rate, a_y (friction capped)."""
    r_ss = v * (sw / vp['steer_ratio']) / (vp['wheelbase_m'] * (1 + vp['understeer_K'] * v ** 2))
    r = np.zeros_like(sw); a = dt / vp['yaw_lag_tau_s']
    for i in range(1, len(sw)):
        r[i] = r[i - 1] + a * (r_ss[i] - r[i - 1])
    ay = np.clip(v * r, -vp['mu_road'] * G, vp['mu_road'] * G)
    return r, ay


def dynamic_bicycle(sw, v, vp, dt):
    """BeamNG-class car (vehicle.model: dynamic): single track with saturating tyres, a steering rack (first-order
    lag + rate limit + lock) and body roll. sw: steering-wheel angle [rad] (driver input), v: speed [m/s].
    Returns yaw rate r, the car's a_y (accelerometer without gravity, what BeamNG reports) and the roll angle
    phi [rad, + = outward for a_y > 0]: the occupant feels a_y + g sin(phi) on top.
      axle loads Fz = m g (w_f, 1 - w_f), cornering stiffness C = c * Fz (c per rad), F = mu Fz tanh(C alpha / (mu Fz))
      m (dvy/dt + v r) = Fyf + Fyr,  Iz dr/dt = a Fyf - b Fyr,  Iz = k_yaw m a b  (mass cancels)
    Below 3 m/s it blends into the kinematic model (the tyre model is singular at standstill)."""
    L = vp['wheelbase_m']; wf = vp['weight_front_frac']; a, b = (1 - wf) * L, wf * L
    cf, cr = vp['c_front_per_rad'], max(vp['c_rear_per_rad'], vp['c_front_per_rad'] + 0.5)   # understeer by build
    mu = vp['mu_road']; k_yaw = vp['k_yaw']
    dmax, rate, tau = vp['delta_max_rad'], vp['rack_rate_radps'], vp['rack_tau_s']
    wr, zr = 2 * np.pi * vp['roll_fn_hz'], vp['roll_zeta']; kr = np.deg2rad(vp['roll_grad_deg_per_g']) / G
    n = len(sw); r = np.zeros(n); ay = np.zeros(n); phi = np.zeros(n); vy = 0.0; d = 0.0; df = 0.0; p, pd_ = 0.0, 0.0
    fz_f, fz_r = G * wf, G * (1 - wf)                                   # per unit mass
    for i in range(1, n):
        dc = np.clip(sw[i] / vp['steer_ratio'], -dmax, dmax)
        df += (dc - df) * min(1.0, dt / tau)                             # rack lag
        d += np.clip(df - d, -rate * dt, rate * dt)                      # rack rate limit
        vv = max(v[i], 0.1)
        if vv < 3.0:                                                     # kinematic at walking speed
            rk = vv * np.tan(d) / L; w = vv / 3.0
        else:
            rk, w = 0.0, 1.0
        ve = max(vv, 3.0)
        af = d - (vy + a * r[i - 1]) / ve; ar = -(vy - b * r[i - 1]) / ve
        Ff = mu * fz_f * np.tanh(cf * fz_f * af / (mu * fz_f)); Fr = mu * fz_r * np.tanh(cr * fz_r * ar / (mu * fz_r))
        rd = (a * Ff - b * Fr) / (k_yaw * a * b)
        rdyn = r[i - 1] + rd * dt; vy += (Ff + Fr - ve * r[i - 1]) * dt
        r[i] = w * rdyn + (1 - w) * rk
        if w < 1.0: vy = b * r[i] * w                                    # kinematic side slip at low speed
        ay[i] = w * (Ff + Fr) + (1 - w) * vv * r[i]
        pdd = wr * wr * (kr * ay[i] - p) - 2 * zr * wr * pd_             # roll: phi -> roll gradient x a_y
        pd_ += pdd * dt; p += pd_ * dt; phi[i] = p
    return r, ay, phi


def steady_K(vp):
    """understeer coefficient [s^2/m^2] of the dynamic model's linear range: r/delta = v / (L (1 + K v^2))"""
    if vp.get('model') != 'dynamic': return vp['understeer_K']
    cr = max(vp['c_rear_per_rad'], vp['c_front_per_rad'] + 0.5)
    return (1 / vp['c_front_per_rad'] - 1 / cr) / (G * vp['wheelbase_m'])


def _smooth_step(x):
    x = np.clip(x, 0, 1); return x * x * (3 - 2 * x)


def synth(kind, rng, sp, vp, T, dt):
    n = int(T / dt); t = np.arange(n) * dt
    v = np.full(n, sp['speed_kph'] / 3.6)
    sw = np.zeros(n)
    ay_pk = sp['ay_peak_g'] * G
    L, N, K = vp['wheelbase_m'], vp['steer_ratio'], steady_K(vp)
    def sw_for(ay, vv):   # steady-state steering-wheel angle for a target a_y
        return ay * L * (1 + K * vv ** 2) * N / max(vv, 1.0) ** 2
    t_cur = rng.uniform(1.0, 3.0)
    while t_cur < T - 2:
        sgn = rng.choice([-1, 1])
        if kind == 'urban_turn':
            R = sp['turn_radius_m']; vv = min(np.sqrt(ay_pk * R), v[0]);
            ramp = max(0.4, ay_pk / (sp['jerk_peak_gps'] * G))
            hold = (np.pi / 2 * R) / max(vv, 1.0)
            i0 = int(t_cur / dt); seg = np.arange(n) * dt - t_cur
            prof = _smooth_step(seg / ramp) - _smooth_step((seg - ramp - hold) / ramp)
            v = np.where(prof > 0, np.minimum(v, vv), v)
            sw += sgn * sw_for(ay_pk, vv) * prof
            t_cur += 2 * ramp + hold + sp['straight_gap_s']
        elif kind == 'lane_change':
            P = sp['lane_change_period_s']; seg = t - t_cur
            m = (seg >= 0) & (seg <= P)
            sw[m] += sgn * sw_for(ay_pk, v[0]) * np.sin(2 * np.pi * seg[m] / P)
            t_cur += P + sp['straight_gap_s']
        elif kind == 'slalom':
            f = sp['slalom_freq_hz']; ncyc = rng.integers(2, 5); seg = t - t_cur
            m = (seg >= 0) & (seg <= ncyc / f)
            sw[m] += sgn * sw_for(ay_pk, v[0]) * np.sin(2 * np.pi * f * seg[m])
            t_cur += ncyc / f + sp['straight_gap_s']
        elif kind == 'steady_circle':
            seg = t - t_cur; dur = rng.uniform(5, 12)
            ramp = max(0.5, ay_pk / (sp['jerk_peak_gps'] * G))
            sw += sgn * sw_for(ay_pk, v[0]) * (_smooth_step(seg / ramp) - _smooth_step((seg - dur) / ramp))
            t_cur += dur + 2 * ramp + sp['straight_gap_s']
        else:  # random_steer: band-limited noise scaled to ay_peak
            e = rng.normal(0, 1, n); a = dt / 0.6; x = np.zeros(n)
            for i in range(1, n): x[i] = x[i - 1] + a * (e[i] * 3 - x[i - 1])
            sw = x / (np.abs(x).max() + 1e-9) * sw_for(ay_pk, v[0]); break
    # speed changes produce a_x; smooth v
    a = dt / 1.5
    for i in range(1, n): v[i] = v[i - 1] + a * (v[i] - v[i - 1])
    if vp.get('model') == 'dynamic':
        r, ay, phi = dynamic_bicycle(sw, v, vp, dt)
    else:
        (r, ay), phi = bicycle(sw, v, vp, dt), np.zeros(n)
    ax = np.gradient(v, dt) - vp['ax_coupling_g'] * np.abs(ay)   # braking while cornering
    bank = G * np.sin(np.deg2rad(vp['road_bank_deg']))
    if vp.get('model') == 'dynamic':                            # BeamNG a_y excludes gravity; the occupant feels roll + bank
        return dict(t=t, ay=ay, ax=ax, v=v, yaw=r, sw=sw, ay_occ=ay + G * np.sin(phi) + bank, roll=phi)
    ay = ay + bank
    return dict(t=t, ay=ay, ax=ax, v=v, yaw=r, sw=sw)


def l1_replay(rng, sp, T, dt):
    d = _l1(); tt = d[:, 0]
    ts = sp['l1_time_scale']; span = T / ts
    t0 = rng.uniform(tt[0], tt[-1] - span - 1)
    n = int(T / dt); t = np.arange(n) * dt
    src_t = t0 + t / ts
    out = {k: np.interp(src_t, tt, d[:, i]) for i, k in ((1, 'ax'), (2, 'ay'), (3, 'yaw'), (4, 'v'))}
    out['ay'] *= sp['l1_amp_scale']; out['yaw'] *= sp['l1_amp_scale']
    # 10 Hz log -> causal smoothing to avoid ZOH steps at 200 Hz
    a = dt / 0.08
    for k in ('ax', 'ay', 'yaw'):
        x = out[k]
        for i in range(1, n): x[i] = x[i - 1] + a * (x[i] - x[i - 1])
    out['sw'] = out['yaw'] * 2.7 * 13.5 / np.maximum(out['v'], 1.0)   # back-computed steering
    out['t'] = t
    return out


def load_sequence(path, dt):
    """Recorded sequence (beamng/convert.py format: t, ax, ay, yaw, v, sw) resampled to dt."""
    key = (os.path.abspath(path), dt)
    if key not in _SEQ:
        d = pd.read_csv(path)
        t = np.arange(0, d.t.iloc[-1], dt)
        _SEQ[key] = dict(t=t, **{k: np.interp(t, d.t, d[k]) for k in ('ax', 'ay', 'yaw', 'v', 'sw')})
    return _SEQ[key]


def library(seq_dir=SEQ_DIR):
    import glob
    return sorted(glob.glob(os.path.join(seq_dir, '*.csv')))


def seq_replay(rng, sp, T, dt, files):
    """Random window of a recorded sequence (BeamNG library). No amplitude scaling:
    the vehicle dynamics in the file are physically consistent."""
    ex = load_sequence(files[rng.integers(len(files))], dt)
    n = int(T / dt)
    if len(ex['t']) <= n:
        raise ValueError('sequence shorter than the episode')
    i0 = rng.integers(0, len(ex['t']) - n)
    out = {k: ex[k][i0:i0 + n].copy() for k in ('ax', 'ay', 'yaw', 'v', 'sw')}
    out['t'] = np.arange(n) * dt
    return out
