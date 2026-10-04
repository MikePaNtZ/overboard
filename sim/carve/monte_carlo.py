"""Monte Carlo on the SpeedHoldLqr speed hold: free-run sim-host on one
constant grade, with the plant and the task varied by a Latin hypercube.

Varied (plant only -- the controller keeps its nominal 70 kg, Kt 0.7 design):
  rider_kg   55..110    ballast mass (sim-host --rider-mass splices the model)
  grade_pct  -25..+18   CLIMB-positive here; a 10 m flat run-in, then 80 m
  v_target   2..6 m/s   --speed-hold
  v_start    0..6 m/s   --start-speed
  amps       30..60 A   --max-current
  kt_scale   0.85..1.15 --kt-scale (true torque per commanded amp)
Fixed: --estimator-aiding grade-aware, passive rider, lean-steer plant.

One row per run in OUT/results.csv. PASS = no fall and the board reaches the
end of the grade. The failure cause comes from the host's handoff line (nose
strike, tail strike, tilt); it is 'saturation' when the motor was at its
limit for at least half of the second before the event.

    $PY sim/carve/monte_carlo.py OUT [--n 200] [--jobs 8] [--seed 1]
    $PY sim/carve/monte_carlo.py OUT --probe 'rider_kg=100,kt_scale=0.85'
"""
import argparse
import csv
import os
import re
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from types import SimpleNamespace

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import battery  # noqa: E402

RANGES = {
    'rider_kg': (55.0, 110.0),
    'grade_pct': (-25.0, 18.0),
    'v_target': (2.0, 6.0),
    'v_start': (0.0, 6.0),
    'amps': (30.0, 60.0),
    'kt_scale': (0.85, 1.15),
}
NOMINAL = dict(rider_kg=70.0, grade_pct=0.0, v_target=3.0, v_start=0.0, amps=40.0, kt_scale=1.0)
BOARD_KG = 13.0      # board alone: the sim's 83 kg total less the 70 kg ballast
RUN_IN_M, GRADE_M = 10.0, 80.0
S_END = RUN_IN_M + GRADE_M - 2.0   # "reached the end" (s = 90 - x)
WINDOW = (14.0, 88.0)              # on-grade window for speed and energy


def lhs(n, rng):
    """Latin hypercube in the RANGES box: one sample per stratum per axis."""
    out = []
    for lo, hi in RANGES.values():
        u = (rng.permutation(n) + rng.random(n)) / n
        out.append(lo + u * (hi - lo))
    return [dict(zip(RANGES, col)) for col in np.array(out).T]


def course(grade_pct, cache):
    """Course dir for a grade, rounded to 0.5 %. Built once, then reused."""
    g = round(grade_pct * 2) / 2
    d = cache / f"g{g:+.1f}"
    if not (d / 'course_hfield.bin').exists():
        preset, flag = ('steady_up', '--climb') if g > 0 else ('steady_down', '--descent')
        subprocess.run([os.environ['PY'], str(HERE / 'course.py'), str(d), '--preset', preset,
                        flag, f"{abs(g)}", '--r-sag', '100', '--r-crest', '100'],
                       check=True, capture_output=True)
    return g, d


def run_one(k, p, out, port):
    name = f"r{k:03d}"
    csv_path, log_path = out / 'runs' / f"{name}.csv", out / 'runs' / f"{name}.txt"
    secs = (RUN_IN_M + GRADE_M) / max(min(p['v_target'], p['v_start'] + 1), 1.0) + 25.0
    cmd = [os.environ['SIMHOST'], '--lean-steer', '--estimator-aiding', 'grade-aware',
           '--max-current', f"{p['amps']:.2f}", '--speed-hold', f"{p['v_target']:.3f}",
           '--rider-mass', f"{p['rider_kg']:.2f}", '--kt-scale', f"{p['kt_scale']:.4f}",
           '--start-speed', f"{p['v_start']:.3f}",
           '--spawn-x', '88', '--terrain', str(p['course'] / 'course_hfield.bin'),
           '--schedule-csv', str(out / 'passive.csv'),
           '--duration-secs', '3600', '--max-sim-secs', f"{min(secs, 90):.0f}", '--free-run',
           '--state-out-addr', f"127.0.0.1:{port}", '--input-in-addr', f"127.0.0.1:{port + 1}",
           '--stats-path', 'none', '--trace-csv', str(csv_path)]
    if not csv_path.exists():
        with open(log_path, 'w') as log:
            subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT, check=False)
    return dict(run=name, **{key: p[key] for key in RANGES}, **analyse(csv_path, log_path, p))


def analyse(csv_path, log_path, p):
    rows = list(csv.DictReader(open(csv_path)))
    g = lambda key: np.array([float(r[key]) for r in rows])
    t, v, amps, sat = g('sim_time_s'), g('forward_speed_m_s'), g('applied_amps'), g('saturated') > 0
    s, fallen, pitch = 90.0 - g('pos_x_m'), g('fallen') > 0, g('truth_pitch_deg')
    n = np.nonzero(np.diff(t) < 0)[0]
    n = n[0] + 1 if len(n) else len(t)
    log = open(log_path).read()
    m = re.search(r"handoff at t=([\d.]+)s -- (bumper strike|tilt) \(bumper \d+ N: nose (\d+) N, "
                  r"tail (\d+) N, tilt ([\d.]+) deg", log)
    # The host runs on past the course end; only the course counts.
    done = np.nonzero(s[:n] >= S_END)[0]
    n = done[0] + 1 if len(done) else n
    fall_i = np.nonzero(fallen[:n])[0]
    end = n
    cause = ''
    if m and float(m.group(1)) <= t[n - 1]:
        t_ev = float(m.group(1))
        end = min(end, int(np.searchsorted(t[:n], t_ev)))
        nose, tail = int(m.group(3)), int(m.group(4))
        cause = 'tilt' if m.group(2) == 'tilt' else ('nose strike' if nose >= tail else 'tail strike')
    if len(fall_i):
        end = min(end, fall_i[0])
        cause = cause or 'fall'
    end = max(end, 2)
    t, v, amps, sat, s, pitch = (x[:end] for x in (t, v, amps, sat, s, pitch))
    if cause:
        last = t >= t[-1] - 1.0
        if sat[last].mean() >= 0.5:
            cause = 'saturation (' + cause + ')'
    reached = s.max()
    status = 'PASS' if not cause and reached >= S_END else ('FALL' if cause else 'STALL')
    on = (s >= WINDOW[0]) & (s <= WINDOW[1])
    vt, v0 = p['v_target'], p['v_start']
    # Overshoot past the target, in the direction of the approach, after the
    # first crossing. Zero for a run that starts at the target.
    up = vt >= v0
    # A start speed is applied at t = 1 s (sim-host START_SPEED_AT_S).
    live = t > (1.01 if v0 > 0 else 0.0)
    cross = np.nonzero(live & (v >= vt if up else v <= vt))[0]
    if len(cross):
        tail_v = v[cross[0]:]
        over = (tail_v.max() - vt) if up else (vt - tail_v.min())
        overshoot = 100.0 * max(over, 0.0) / vt
    else:
        overshoot = float('nan')
    vg = v[on]
    a = SimpleNamespace(kt=0.7 * p['kt_scale'], ke=0.7 * p['kt_scale'], r_phase=0.12, p_idle=15.0,
                        regen_eff=0.8, series=20, parallel=2, cell_ah=4.0, r_cell=0.015, max_duty=0.9,
                        mass=BOARD_KG + p['rider_kg'], crr=0.015, cda=0.5)
    wh_km = float('nan')
    if on.sum() > 50:
        # applied_amps is the COMMANDED current; the true current is the same
        # (Kt error changes torque, not current).
        e = battery.analyse(t[on], amps[on], v[on], a)
        wh_km = e['wh_per_km']
    return dict(
        status=status, cause=cause, reached_m=round(float(reached), 1),
        t_event_s=round(float(t[-1]), 2) if cause else '',
        v_mean=round(float(vg.mean()), 3) if len(vg) else '',
        overshoot_pct=round(overshoot, 1),
        peak_amps=round(float(np.abs(amps).max()), 1),
        sat_pct=round(100.0 * float(sat.mean()), 1),
        peak_pitch_deg=round(float(np.abs(pitch).max()), 1),
        wh_per_km=round(wh_km, 1),
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out')
    ap.add_argument('--n', type=int, default=200)
    ap.add_argument('--jobs', type=int, default=8)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--probe', action='append', help="k=v,... over NOMINAL; repeatable")
    args = ap.parse_args()
    out = Path(args.out).resolve()
    (out / 'runs').mkdir(parents=True, exist_ok=True)
    (out / 'passive.csv').write_text("0,60,0,0,0,passive rider\n")
    if args.probe:
        plan = []
        for spec in args.probe:
            p = dict(NOMINAL)
            for kv in filter(None, spec.split(',')):
                key, val = kv.split('=')
                p[key] = float(val)
            plan.append(p)
    else:
        plan = lhs(args.n, np.random.default_rng(args.seed))
    cache = out / 'courses'
    for p in plan:
        p['grade_pct'], p['course'] = course(p['grade_pct'], cache)
    with ThreadPoolExecutor(args.jobs) as ex:
        futs = [ex.submit(run_one, k, p, out, 9000 + 2 * k) for k, p in enumerate(plan)]
        results = []
        for f in futs:
            r = f.result()
            results.append(r)
            print(f"{r['run']} {r['status']:5s} {r['cause']:26s} grade {r['grade_pct']:+5.1f} "
                  f"v {r['v_start']:.1f}->{r['v_target']:.1f} m {r['rider_kg']:5.1f} "
                  f"I {r['amps']:4.1f} kt {r['kt_scale']:.2f} | over {r['overshoot_pct']}% "
                  f"Ipk {r['peak_amps']} sat {r['sat_pct']}% {r['wh_per_km']} Wh/km", flush=True)
    with open(out / 'results.csv', 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=list(results[0]))
        w.writeheader()
        w.writerows(results)


if __name__ == '__main__':
    main()
