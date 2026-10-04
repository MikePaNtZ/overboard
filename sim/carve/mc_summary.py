"""Summary of a monte_carlo.py results CSV: pass/fail, the static-envelope
split (geometry / authority / controller) and PASS-run metrics.

Static envelope per run (steady speed on the constant grade):
  deck angle vs road = atan|g| + asin(R sin(alpha) / L),  strike at 18.6 deg
  current            = (13 + m) g sin(alpha) R / (0.7 kt_scale)
A FALL inside a comfortable envelope (>= 3 deg and >= 1.2x current) is the
controller's.

    $PY sim/carve/mc_summary.py RESULTS.csv [...]
"""
import collections
import csv
import sys

import numpy as np

R, STRIKE_DEG, BOARD_KG = 0.1454, 18.6, 13.0


def envelope(r):
    m = float(r['rider_kg'])
    a = np.arctan(abs(float(r['grade_pct'])) / 100)
    L = (0.75 * m + 0.135) / (BOARD_KG + m)
    geo = STRIKE_DEG - np.degrees(a + np.arcsin(R * np.sin(a) / L))
    i_ss = (BOARD_KG + m) * 9.81 * np.sin(a) * R / (0.7 * float(r['kt_scale']))
    return geo, float(r['amps']) / max(i_ss, 1e-6)


def zone(r):
    geo, ratio = envelope(r)
    return 'geometry' if geo < 3 else ('authority' if ratio < 1.2 else 'controller')


def main():
    for path in sys.argv[1:]:
        d = list(csv.DictReader(open(path)))
        print(path, dict(collections.Counter(r['status'] for r in d)),
              dict(collections.Counter(r['cause'] for r in d if r['cause'])))
        for z in ('geometry', 'authority', 'controller'):
            s = [r for r in d if zone(r) == z]
            print(f"  {z:10s} zone: {len(s):3d} runs, {sum(r['status'] != 'PASS' for r in s):3d} fail")
        p = [r for r in d if r['status'] == 'PASS']
        for k in ('overshoot_pct', 'peak_amps', 'sat_pct', 'peak_pitch_deg'):
            x = np.array([float(r[k]) for r in p])
            x = x[np.isfinite(x)]
            print(f"  PASS {k:15s} median {np.median(x):5.1f}  p90 {np.percentile(x, 90):5.1f}  max {x.max():5.1f}")


if __name__ == '__main__':
    main()
