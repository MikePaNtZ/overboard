"""One summary line for an authority-sweep run (see authority_sweep.sh)."""
import csv, sys
import numpy as np

name, base, amps = sys.argv[1], sys.argv[2], float(sys.argv[3])
d = dict(np.load(base + '.npz'))
t = d['t']
j = np.nonzero(np.diff(t) < 0)[0]
m = j[0] + 1 if len(j) else len(t)
d = {k: v[:m] for k, v in d.items()}
t = d['t']
s = 90.0 - d['px']
fell = (d['flags'].astype(int) & 4) > 0
v = np.hypot(d['vx'], d['vy'])
on_grade = (s > 14.0) & (s < 88.0) & ~fell
rows = list(csv.DictReader(open(base + '.csv')))
f = lambda k: np.array([float(r[k]) for r in rows])
tt, am, tp = f('sim_time_s'), f('applied_amps'), f('truth_pitch_deg')
n = np.nonzero(np.diff(tt) < 0)[0]
n = n[0] + 1 if len(n) else len(tt)
tt, am, tp = tt[:n], am[:n], tp[:n]
ok = ~(f('fallen')[:n] > 0)
reached = s[~fell].max() if (~fell).any() else 0.0
status = 'PASS' if (not fell.any() and reached > 85.0) else ('FELL s=%.0f m' % s[fell][0] if fell.any() else 'STALLED s=%.0f m' % reached)
vg = v[on_grade]
print(f"{name:14s} {status:16s} v_on_grade {vg.mean() if len(vg) else float('nan'):4.2f}±{vg.std() if len(vg) else 0:.2f} m/s  "
      f"|I| peak {np.abs(am[ok]).max() if ok.any() else 0:5.1f} A  saturated {100*np.mean(np.abs(am[ok]) >= 0.995*amps) if ok.any() else 0:4.1f}%  "
      f"|pitch| peak {np.abs(tp[ok]).max() if ok.any() else 0:4.1f} deg")
