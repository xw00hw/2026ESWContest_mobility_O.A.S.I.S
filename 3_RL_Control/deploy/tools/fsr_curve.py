"""FSR response curve (scale press test) -> STM32 counts, force, local gain.

    python deploy/tools/fsr_curve.py                                  # table, fit, local gains
    python deploy/tools/fsr_curve.py --counts 950 1350 1456           # force / gain at these readings
    python deploy/tools/fsr_curve.py --rm_uno 10000 --rm_stm 4700     # different divider on the STM bank
    python deploy/tools/fsr_curve.py --write                          # store the fit in the json (Pi runtime reads it)

Transfer rule (fsr_curve.json): the Uno ran at 5 V supply = 5 V reference, the STM32 bank's
divider and VREF+ share the 3V3 rail -> both ADCs measure the divider RATIO, so
    stm_counts = 4095 * uno_V / 5.0        (same divider resistor and topology)
The exact VREF+ (3.3 V nominal, VDDA rail on the Nucleo) only matters for showing volts. If the divider
differs, the FSR resistance R_fsr = R_m (1/ratio - 1) of a pull-down divider is carried over instead.
CHECK: the car drive read up to 2316 (A1_C1) / 2063 (A1_C3) counts, above this curve's
saturation (~1880-1920) -> the A1 bank's divider is NOT the Uno test's; remap with the real R_m values
(or --rm_ratio, >= ~1.5 from the car maxima) before using forces from this curve.

Model: counts(F) = cmax F^n / (F^n + F50^n) (Hill), fitted on force F = pressure x contact area.
Force here is the force on the calibration contact (d = 3.15 cm); on the seat the pad sees its own
contact area -- use pressure if the areas differ.
"""
import argparse, json, os
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT = os.path.join(HERE, 'fsr_curve.json')
FULL = 4095


class FsrCurve:
    def __init__(self, path=DEFAULT, rm_uno=None, rm_stm=None, rm_ratio=None):
        d = json.load(open(path)); c = d['conditions']
        self.meta = d; self.area = np.pi * (c['contact_diameter_cm'] / 200) ** 2
        self.kpa = np.array([p['kPa'] for p in d['points']]); self.F = self.kpa * 1e3 * self.area
        ratio = np.array([p['uno_V'] for p in d['points']]) / c['uno_supply_and_ref_V']
        if rm_uno and rm_stm: rm_ratio = rm_stm / rm_uno
        if rm_ratio:                                                 # same FSR, different pull-down resistor
            r_fsr_over_rm_uno = 1 / ratio - 1; ratio = rm_ratio / (rm_ratio + r_fsr_over_rm_uno)
        self.counts = ratio * FULL
        from scipy.optimize import curve_fit
        self.p, _ = curve_fit(self._hill, self.F, self.counts, p0=[1900, 5, 1], maxfev=20000)
        self.sat = float(self.counts.max())

    @staticmethod
    def _hill(f, cm, f50, n):
        f = np.maximum(np.asarray(f, float), 0.0)
        return cm * f ** n / (f ** n + f50 ** n)

    def counts_of(self, F):
        return self._hill(F, *self.p)

    def force_of(self, counts):
        """inverse; readings at / above the fitted cmax are saturated -> inf"""
        cm, f50, n = self.p; c = np.asarray(counts, float)
        with np.errstate(divide='ignore', invalid='ignore'):
            F = f50 * (c / np.maximum(cm - c, 1e-9)) ** (1 / n)
        return np.where(c >= cm, np.inf, np.where(c <= 0, 0.0, F))

    def gain_at(self, counts):
        """local counts per newton at a reading (what a linear model with counts_per_N should use there)"""
        F = self.force_of(counts); cm, f50, n = self.p
        return cm * n * f50 ** n * F ** (n - 1) / (F ** n + f50 ** n) ** 2

    def summary(self):
        cm, f50, n = self.p
        return dict(fit=dict(cmax_counts=float(cm), F50_N=float(f50), n=float(n),
                             max_abs_resid_counts=float(np.max(np.abs(self.counts - self.counts_of(self.F))))),
                    contact_area_cm2=self.area * 1e4, saturation_counts=self.sat,
                    table=[dict(kPa=float(k), F_N=round(float(f), 2), stm_counts=int(round(c)), stm_V_at_3V3=round(float(c) * 3.3 / FULL, 3),
                                gain_counts_per_N=round(float(self.gain_at(c)), 2)) for k, f, c in zip(self.kpa, self.F, self.counts)])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--curve', default=DEFAULT); ap.add_argument('--counts', type=float, nargs='*')
    ap.add_argument('--rm_uno', type=float); ap.add_argument('--rm_stm', type=float)
    ap.add_argument('--rm_ratio', type=float, help='R_m(STM bank) / R_m(Uno test) if only the ratio is known')
    ap.add_argument('--write', action='store_true', help='store fit + STM table in the curve json')
    a = ap.parse_args()
    C = FsrCurve(a.curve, a.rm_uno, a.rm_stm, a.rm_ratio); s = C.summary()
    if a.write:
        d = json.load(open(a.curve)); d['fit'] = dict(s['fit'], model='counts = cmax F^n / (F^n + F50^n), F on the calibration contact',
                                                     divider_remap=dict(rm_uno=a.rm_uno, rm_stm=a.rm_stm, rm_ratio=a.rm_ratio) if (a.rm_ratio or (a.rm_uno and a.rm_stm)) else None)
        d['stm_table'] = s['table']; d['saturation_counts_measured'] = s['saturation_counts']
        json.dump(d, open(a.curve, 'w'), indent=1); print('wrote fit into', a.curve)
    print(f"fit: counts = {s['fit']['cmax_counts']:.0f} F^{s['fit']['n']:.2f} / (F^n + {s['fit']['F50_N']:.2f}^n), "
          f"max resid {s['fit']['max_abs_resid_counts']:.0f} counts; contact {s['contact_area_cm2']:.2f} cm2")
    print(f"{'kPa':>8} {'F [N]':>8} {'STM counts':>11} {'V @3.3':>7} {'counts/N':>9}")
    for r in s['table']:
        print(f"{r['kPa']:8.2f} {r['F_N']:8.2f} {r['stm_counts']:11d} {r['stm_V_at_3V3']:7.3f} {r['gain_counts_per_N']:9.2f}")
    for c in a.counts or []:
        print(f"reading {c:.0f} counts -> {float(C.force_of(c)):.1f} N on the contact, local gain {float(C.gain_at(c)):.1f} counts/N")


if __name__ == '__main__':
    main()
