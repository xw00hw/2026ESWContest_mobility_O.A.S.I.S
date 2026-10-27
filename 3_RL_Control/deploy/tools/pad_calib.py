"""Side-pad force calibration with a hand-held scale (protocol: PAD_CALIB_PROTOCOL.md).

  record : on the Pi, STM32 connected. Live counts; press the pad THROUGH the scale + puck, hold
           steady, press Enter (captures the last 1.5 s of all 12 channels), type the scale reading.
             python deploy/tools/pad_calib.py record --port /dev/ttyACM0 --pad side_R --out pad_runs/R_y0_a.csv
             (--yaw 10 arms the STM32 and holds both bolsters at 10 deg yaw during the run; the operator at the
              bench is the supervisor then -- Ctrl-C = FB_OFF)
           --firmware fallback_v2 (default: OASIS_FALLBACK v2 via oasis_rt.fb_link) | control_v6 (legacy)
  fit    : counts vs N per channel -> pad_calib.json (runtime interlock gain, calib_v2 --pad_calib)
             python deploy/tools/pad_calib.py fit pad_runs/*.csv --out deploy/model/pad_calib.json
  demo   : synthetic run (FSR saturation + hysteresis + noise) to test the fitter
             python deploy/tools/pad_calib.py demo --out pad_runs/demo.csv

Besides the gain this checks the CHANNEL MAPPING: pressing side_R must move A1_C1 the most and
side_L A1_C3 (calib_v2 / runtime assume so; the car runs had no mat-position metadata).
"""
import argparse, csv, glob, json, os, sys, threading, time
from collections import deque
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'runtime'))
CHANNELS = [f'A{b}_C{c}' for b in range(3) for c in range(4)]
PADS = {'side_R': 'A1_C1', 'side_L': 'A1_C3'}
LEVELS_N = [0, 10, 20, 40, 60, 80, 100, 80, 60, 40, 20, 10, 0]
G0 = 9.80665
MODEL_G = 32.61            # calib_v2 G_side at nominal k (counts/N) -- what the runtime uses until this is run
BENCH_TIGHT = 15.68        # bench B1: side-diff counts per deg of tightening (analysis/corr_model.json)
CAR_CONSOLE = 7234.0       # car: side-diff counts per g toward the console


# ------------------------------------------------------------------ record (Pi)
class Reader(threading.Thread):
    """Reads the STM32 continuously (the scan is ~16 kB/s: blocking on input() would overflow the
    UART buffer), keeps a 10 s ring of pressure snapshots and sends PING (FSR stream needs < 3 s)."""
    def __init__(self, link):
        super().__init__(daemon=True); self.link = link; self.ring = deque(maxlen=1500); self.stop = False
    def run(self):
        last_ping = 0.0
        while not self.stop:
            self.link.poll(); now = time.monotonic()
            if self.link.pressure: self.ring.append((now, dict(self.link.pressure)))
            if now - last_ping > 0.5: self.link._write('PING'); last_ping = now
            time.sleep(0.01)
    def window(self, seconds):
        t1 = time.monotonic(); w = [p for t, p in list(self.ring) if t >= t1 - seconds]
        return {ch: np.array([p[ch] for p in w if ch in p], float) for ch in CHANNELS}


class FbReader(threading.Thread):
    """OASIS_FALLBACK v2: this thread alone owns the board (BoardLink is single-owner): polls, keeps the pressure
    ring, services the scanner PING; with a hold yaw it ARMs once from verified OFF, sets the target and renews the
    lease itself (no BeamNG here -- the operator at the bench supervises). Any link fault ends it."""
    def __init__(self, link, hold_yaw=None):
        super().__init__(daemon=True); self.link, self.hold = link, hold_yaw; self.ring = deque(maxlen=1500)
        self.stop = False; self.error = None; self.armed_at = None
    def run(self):
        L = self.link
        while not self.stop:
            L.poll()
            if self.hold:
                if not L.armed and self.armed_at is None and L.board_off() and L.arm(): self.armed_at = L.clock_ns()
                if L.armed: L.authorize(); L.send_targets(self.hold, 0.0, self.hold, 0.0)
            L.service()
            if L.pressure: self.ring.append((time.monotonic(), dict(L.pressure)))
            if L.fault: self.error = L.fault; break
            time.sleep(0.002)
    def window(self, seconds):
        t1 = time.monotonic(); w = [p for t, p in list(self.ring) if t >= t1 - seconds]
        return {ch: np.array([p[ch] for p in w if ch in p], float) for ch in CHANNELS}


def _open(a):
    if a.firmware == 'control_v6':
        from oasis_rt.stm32_link import Stm32Link, SerialTransport
        link = Stm32Link(SerialTransport(a.port), dict(yaw_max_deg=20, send_mode='event', deadband_deg=0.5, min_interval_s=0.25, keepalive_s=0))
        print('firmware:', link.connect())
        rd = Reader(link); rd.start(); time.sleep(1.0)
        if a.yaw:
            link.arm(); link.send_targets(a.yaw, 0.0, a.yaw, 0.0); print(f'bolsters -> yaw {a.yaw} deg; wait until still'); time.sleep(4)
        return link, rd, lambda: (link.hold(), link.close())
    from oasis_rt import fbstack
    from oasis_rt.fb_link import FbLink
    fb = fbstack.load(); link = FbLink(fb.serial_link.open_serial(a.port), fb, time.monotonic_ns); link.start(); t0 = time.monotonic()
    while not link.ready:
        link.poll(); link.service(); time.sleep(0.005)
        if link.fault or time.monotonic() - t0 > 5: sys.exit(f'board handshake failed: {link.fault or "timeout"}')
    print('firmware:', link.hello, 'tuning:', link.freeze())
    rd = FbReader(link, a.yaw or None); rd.start(); time.sleep(1.0)
    if a.yaw: print(f'bolsters -> yaw {a.yaw} deg (FB_ARM applies the bench neutral first); wait until still'); time.sleep(6)
    def close():
        rd.stop = True; rd.join(1.0); print('PWM OFF verified:', link.close(time.sleep))
    return link, rd, close


def record(a):
    link, rd, close = _open(a)
    ch = PADS[a.pad]; os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    new = not os.path.exists(a.out); f = open(a.out, 'a', newline=''); w = csv.writer(f)
    if new:
        w.writerow(['session', 'pad', 'channel', 'yaw_deg', 'placement', 'dir', 'step', 'scale_kg', 'F_N', 't'] +
                   [f'{x}_{c}' for c in CHANNELS for x in ('mean', 'std')] + ['n'])
    session = time.strftime('%Y%m%d_%H%M%S'); step = 0
    print(f'pad {a.pad} ({ch}), placement {a.placement}. Steps (target N): {LEVELS_N}')
    try:
        while step < len(LEVELS_N):
            target = LEVELS_N[step]; d = 'up' if step <= LEVELS_N.index(max(LEVELS_N)) else 'down'
            print(f'\n[{step + 1}/{len(LEVELS_N)}] target ~{target} N (= {target / G0:.1f} kg on the scale), {d}.'
                  ' Hold steady, then Enter.  (live, Ctrl-C to abort)')
            stop_live = threading.Event()
            def live():
                while not stop_live.is_set():
                    p = link.pressure
                    sys.stdout.write('\r  ' + '  '.join(f'{c}={p.get(c, "--"):>5}' if c != ch else f'[{c}={p.get(c, "--"):>5}]'
                                                       for c in ('A1_C0', 'A1_C1', 'A1_C2', 'A1_C3')) + '   ')
                    sys.stdout.flush(); time.sleep(0.2)
            th = threading.Thread(target=live, daemon=True); th.start(); input(); stop_live.set(); th.join()
            win = rd.window(1.5); x = win[ch]
            if len(x) < 10: print('  too few samples (stream?) -> redo'); continue
            print(f'  captured {ch}: mean {x.mean():.0f}, std {x.std():.1f} counts (n={len(x)})')
            if x.std() > max(30.0, 0.05 * abs(x.mean())): print('  WARNING: not steady')
            r = input('  scale reading in kg (r = redo, q = quit): ').strip().lower()
            if r == 'q': break
            if r == 'r' or not r: continue
            kg = float(r)
            w.writerow([session, a.pad, ch, a.yaw, a.placement, d, step, kg, kg * G0, f'{time.time():.3f}'] +
                       [f'{v:.2f}' for c in CHANNELS for v in ((win[c].mean(), win[c].std()) if len(win[c]) else (np.nan, np.nan))] + [len(x)])
            f.flush(); step += 1
    finally:
        rd.stop = True; close(); f.close()
        if getattr(rd, 'error', None): print('board link stopped:', rd.error)
    print('\nsaved', a.out)


# ------------------------------------------------------------------ demo data
def demo(a):
    rng = np.random.default_rng(a.seed); rows = []
    truth = {'A1_C1': dict(base=420, cmax=2600, F50=90), 'A1_C3': dict(base=380, cmax=2200, F50=110)}
    for pad, ch in PADS.items():
        for rep in range(2):
            for step, F in enumerate(LEVELS_N):
                d = 'up' if step <= LEVELS_N.index(max(LEVELS_N)) else 'down'
                Fm = F * (1 + rng.normal(0, 0.03)) if F else 0.0                       # what the scale showed
                row = [f'demo{rep}', pad, ch, 0, 'center', d, step, Fm / G0, Fm, step]
                for c in CHANNELS:
                    t = truth.get(c, dict(base=300, cmax=0, F50=100))
                    eff = Fm if c == ch else 0.08 * Fm if c in PADS.values() else 0.02 * Fm   # cross-talk
                    v = t['base'] + (t['cmax'] or 400) * eff / (eff + t['F50']) + (15 if d == 'down' and F else 0)
                    row += [v + rng.normal(0, 4), 6.0]
                rows.append(row + [40])
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    with open(a.out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['session', 'pad', 'channel', 'yaw_deg', 'placement', 'dir', 'step', 'scale_kg', 'F_N', 't'] +
                   [f'{x}_{c}' for c in CHANNELS for x in ('mean', 'std')] + ['n'])
        w.writerows(rows)
    print('wrote', a.out, '(truth: A1_C1 slope@0-80N ~', round(2600 * 90 / (40 + 90) ** 2, 1), 'counts/N)')


# ------------------------------------------------------------------ fit
def fit(a):
    import pandas as pd
    from scipy.optimize import curve_fit
    files = sorted({p for g in a.files for p in glob.glob(g)})
    D = pd.concat([pd.read_csv(p).assign(file=os.path.basename(p)) for p in files], ignore_index=True)
    out = dict(version='pad-calib-v1', created=time.strftime('%Y-%m-%d %H:%M'), files=files, model_G_counts_per_N=MODEL_G,
               pads={}, controller_counts_per_N={}, mapping_check={})
    for (pad, yaw, place), g in D.groupby(['pad', 'yaw_deg', 'placement']):
        ch = PADS[pad]; key = f'{pad}@yaw{yaw:g}/{place}'
        sess = []
        for s, gs in g.groupby('session'):                       # zero per session (drift between runs)
            z = gs[gs.F_N < 1.0][f'mean_{ch}']
            if len(z) == 0: continue
            sess.append(gs.assign(dc=gs[f'mean_{ch}'] - z.mean(), **{f'd_{c}': gs[f'mean_{c}'] - gs[gs.F_N < 1.0][f'mean_{c}'].mean() for c in CHANNELS}))
        G = pd.concat(sess); F = G.F_N.values; c = G.dc.values
        op = F <= a.fmax
        slope = float(np.sum(F[op] * c[op]) / np.sum(F[op] ** 2))                   # through zero, 0..fmax N
        r2 = float(1 - np.sum((c[op] - slope * F[op]) ** 2) / np.sum((c[op] - c[op].mean()) ** 2))
        try:
            (cmax, f50), _ = curve_fit(lambda f, cm, f5: cm * f / (f + f5), F, c, p0=[max(c.max() * 2, 1), 80], maxfev=20000)
        except RuntimeError:
            cmax, f50 = np.nan, np.nan
        sat = lambda f: cmax * f / (f + f50)
        err120 = float(100 * (slope * 120 - sat(120)) / sat(120)) if np.isfinite(cmax) else None
        up, dn = G[G.dir == 'up'], G[G.dir == 'down']
        hyst = None
        if len(up) > 2 and len(dn) > 2:
            fu = up.groupby('step').agg(F=('F_N', 'mean'), c=('dc', 'mean')).sort_values('F')
            fd_ = dn.groupby('step').agg(F=('F_N', 'mean'), c=('dc', 'mean')).sort_values('F')
            grid = fu.F.values[(fu.F.values >= fd_.F.min()) & (fu.F.values <= fd_.F.max()) & (fu.F.values > 1)]
            if len(grid): hyst = float(np.max(np.abs(np.interp(grid, fd_.F, fd_.c) - np.interp(grid, fu.F, fu.c))) / slope)
        per_sess = [float(np.sum(gs.F_N[gs.F_N <= a.fmax] * gs.dc[gs.F_N <= a.fmax]) / np.sum(gs.F_N[gs.F_N <= a.fmax] ** 2))
                    for _, gs in G.groupby('session')]
        xt = {cc: float(np.sum(F[op] * G[f'd_{cc}'].values[op]) / np.sum(F[op] ** 2)) for cc in CHANNELS}
        top = max(xt, key=lambda k: abs(xt[k]))
        out['pads'][key] = dict(channel=ch, counts_per_N_0_to_fmax=slope, fmax_N=a.fmax, r2_linear=r2,
                                saturating_fit=dict(cmax=float(cmax), F50_N=float(f50)), linear_force_error_at_120N_pct=err120,
                                hysteresis_N=hyst, sessions=len(per_sess), slope_per_session=per_sess,
                                slope_cv=float(np.std(per_sess) / np.mean(per_sess)) if len(per_sess) > 1 else None,
                                crosstalk_counts_per_N=xt, strongest_channel=top,
                                noise_std_counts=float(G[f'std_{ch}'].median()))
        out['mapping_check'][key] = dict(expected=ch, strongest=top, ok=top == ch)
        if yaw == 0 and place == 'center': out['controller_counts_per_N'][ch] = slope
    g = out['controller_counts_per_N']
    if {'A1_C1', 'A1_C3'} <= set(g):
        gm = 0.5 * (g['A1_C1'] + g['A1_C3'])
        out['derived'] = dict(G_mean_counts_per_N=gm, vs_model_G=gm / MODEL_G,
                              bench_tightening_N_per_deg=BENCH_TIGHT / gm, model_tightening_N_per_deg=0.4806,
                              car_console_N_per_g=CAR_CONSOLE / gm, model_console_N_per_g=221.9,
                              next='python sim/v2/calib_v2.py --pad_calib <this file>  (identifies contact k), then pipeline/rebuild.py')
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    json.dump(out, open(a.out, 'w'), indent=1)
    for k, v in out['pads'].items():
        print(f"{k:28s} {v['channel']}: {v['counts_per_N_0_to_fmax']:.1f} counts/N (R2 {v['r2_linear']:.3f}), "
              f"hyst {v['hysteresis_N'] if v['hysteresis_N'] is None else round(v['hysteresis_N'], 1)} N, strongest {v['strongest_channel']}")
    if 'derived' in out: print('derived:', {k: (round(v, 3) if isinstance(v, float) else v) for k, v in out['derived'].items()})
    bad = [k for k, v in out['mapping_check'].items() if not v['ok']]
    if bad: print('MAPPING CHECK FAILED (fix pressure.channels in config.yaml and calib FEATS):', bad)
    print('wrote', a.out)


def main():
    ap = argparse.ArgumentParser(); sp = ap.add_subparsers(dest='cmd', required=True)
    r = sp.add_parser('record'); r.add_argument('--port', default='/dev/ttyACM0'); r.add_argument('--pad', choices=PADS, required=True)
    r.add_argument('--out', required=True); r.add_argument('--yaw', type=float, default=0.0); r.add_argument('--placement', default='center')
    r.add_argument('--firmware', choices=('fallback_v2', 'control_v6'), default='fallback_v2')
    f = sp.add_parser('fit'); f.add_argument('files', nargs='+'); f.add_argument('--out', required=True); f.add_argument('--fmax', type=float, default=80.0)
    d = sp.add_parser('demo'); d.add_argument('--out', required=True); d.add_argument('--seed', type=int, default=0)
    a = ap.parse_args(); {'record': record, 'fit': fit, 'demo': demo}[a.cmd](a)


if __name__ == '__main__':
    main()
