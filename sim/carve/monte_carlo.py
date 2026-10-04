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

Ground (--ground): 'plane' (default) rides sim-host --grade-course, the grade
by gravity on a flat plane. 'hfield' rides a course.py heightfield, whose
sphere contact chatters at every grid edge; kept to measure that effect.

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
# --plant-x7: the Fungineers X7 / Superflux HT / Thor301 build (sim-host
# --plant x7) with its mass-property uncertainty as dispersions (hardware
# track, 2026-10-04; most values inferred, so the spreads are wide). The motor
# limit spans the phase currents this build would be set to; climbs go to
# +25 % for Seattle streets.
X7_RANGES = {
    'rider_kg': (55.0, 110.0),
    'grade_pct': (-25.0, 25.0),
    'v_target': (2.0, 6.0),
    'v_start': (0.0, 6.0),
    'amps': (60.0, 100.0),
    'kt_scale': (0.63 / 0.7, 0.69 / 0.7),    # Kt 0.63-0.69 N.m/A
    'board_kg': (16.9, 19.9),                 # 18.4 +- 1.5
    'wheel_kg': (3.6, 5.4),                   # rotating part 4.5, with the assembly +- 1.5
    'wheel_spin': (0.0315, 0.0585),           # 0.045 kg m^2 +- 30 %
    'com_x': (0.0244 - 0.03, 0.0244 + 0.03),  # frame CoM behind the axle, m
    'com_z': (0.003 - 0.02, 0.003 + 0.02),
    'frame_i': (1.175 * 0.7, 1.175 * 1.3),    # frame inertia scale, +- 30 %
    'radius': (0.142, 0.150),                 # tyre radius, m (pressure, wear)
    # Nose/tail pad height: -0.0140..+0.0034 m spans a deck strike angle of
    # 18-21 deg (proxy pads strike at 20.4 deg; the real bumper probably
    # wraps lower, so the hardware track's honest range is 18-21 deg).
    'pad_z': (-0.0140, 0.0034),
}
X7_KEYS = ('board_kg', 'wheel_kg', 'wheel_spin', 'com_x', 'com_z', 'frame_i', 'radius', 'pad_z')
X7_R_PHASE_OHM = 0.0525   # Superflux HT, motor wizard
X7_CELL_AH = 5.0          # Molicel P50B, 20S2P = 10 Ah

NOMINAL = dict(rider_kg=70.0, grade_pct=0.0, v_target=3.0, v_start=0.0, amps=40.0, kt_scale=1.0)
BOARD_KG = 13.0      # board alone in the model; --board-kg changes it (sim-host --board-mass)
RUN_IN_M, GRADE_M = 10.0, 80.0
S_END = RUN_IN_M + GRADE_M - 2.0   # "reached the end" (s = 90 - x)
WINDOW = (14.0, 88.0)              # on-grade window for speed (energy: constant grade only)
# Plane ground: mean vertical-curve radius, m. Long on purpose: the plane
# model turns gravity instead of the road and has no Euler torque, so a tight
# curve asks the motor for torque a real road does not (13 A measured at
# R = 100 m, 4 m/s, 104 kg). At 400 m that term is < 0.4 N.m.
R_CURVE = 400.0
EXTRA_ARGS = []                    # extra sim-host flags (--host-arg)
# --rider-law: a rider model leans and the DEPLOYED balance law balances
# (sim-host --rider-speed), instead of the SpeedHoldLqr harness (--speed-hold).
RIDER_FLAG = ['--speed-hold']
# --tail-brake: tail pad friction on the road. Hard plastic about 0.3,
# rubber or urethane skid pads up to about 0.8 (typical, not measured).
TAIL_MU_RANGE = (0.3, 0.8)
TAIL_MU_NOMINAL = 0.6
TAIL_CONTACT_N = 50.0              # the host's own strike threshold
FLAT_GRADE_M = 60.0                # constant grade after the curve, plane ground


def run_in(grade_pct):
    """sim-host centres the vertical curve on the run-in end; this starts it
    at s = RUN_IN_M for every grade, so no board spawns on the curve."""
    return RUN_IN_M + 0.5 * R_CURVE * abs(grade_pct) / 100.0


def grade_start(grade_pct):
    """Where the constant grade begins, m."""
    return RUN_IN_M + R_CURVE * abs(grade_pct) / 100.0


def s_end(p, ground):
    """'Reached the end', m along the course."""
    return S_END if ground == 'hfield' else grade_start(p['grade_pct']) + FLAT_GRADE_M


def lhs(n, rng, ranges=None):
    """Latin hypercube in the box: one sample per stratum per axis."""
    ranges = ranges or RANGES
    out = []
    for lo, hi in ranges.values():
        u = (rng.permutation(n) + rng.random(n)) / n
        out.append(lo + u * (hi - lo))
    return [dict(zip(ranges, col)) for col in np.array(out).T]


def course(grade_pct, cache):
    """Course dir for a grade, rounded to 0.5 %. Built once, then reused."""
    g = round(grade_pct * 2) / 2
    d = cache / f"g{g:+.1f}"
    if not (d / 'course_hfield.bin').exists():
        preset, flag = ('steady_up', '--climb') if g > 0 else ('steady_down', '--descent')
        subprocess.run([os.environ['PY'], str(HERE / 'course.py'), str(d), '--preset', preset,
                        flag, f"{abs(g)}", '--r-sag', '100', '--r-crest', '100'],
                       check=True, capture_output=True)
        # sim-host reads only the .bin and metadata; the .npy is for Unreal.
        (d / 'course_height.npy').unlink(missing_ok=True)
    return g, d


def run_one(k, p, out, port, ground):
    name = f"r{k:03d}"
    csv_path, log_path = out / 'runs' / f"{name}.csv", out / 'runs' / f"{name}.txt"
    secs = s_end(p, ground) / max(min(p['v_target'], p['v_start'] + 1), 1.0) + 25.0
    cmd = [os.environ['SIMHOST'], '--lean-steer', '--estimator-aiding', 'grade-aware',
           '--max-current', f"{p['amps']:.2f}", RIDER_FLAG[0], f"{p['v_target']:.3f}",
           '--rider-mass', f"{p['rider_kg']:.2f}", '--kt-scale', f"{p['kt_scale']:.4f}",
           '--start-speed', f"{p['v_start']:.3f}",
           *(['--spawn-x', '88', '--terrain', str(p['course'] / 'course_hfield.bin')] if ground == 'hfield'
             else ['--grade-course', f"{run_in(p['grade_pct']):.3f},{p['grade_pct']:.3f},{R_CURVE}"]),
           '--schedule-csv', str(out / 'passive.csv'),
           '--duration-secs', '3600', '--max-sim-secs', f"{min(secs, 200):.0f}", '--free-run',
           '--state-out-addr', f"127.0.0.1:{port}", '--input-in-addr', f"127.0.0.1:{port + 1}",
           '--stats-path', 'none', '--trace-csv', str(csv_path)] + EXTRA_ARGS
    if 'tail_mu' in p:
        cmd += ['--tail-brake', '--tail-friction', f"{p['tail_mu']:.3f}"]
    if 'board_kg' in p and 'radius' in p:
        cmd += ['--plant', 'x7,' + ','.join(f"{k}={p[k]:.5f}" for k in X7_KEYS)]
    if not csv_path.exists():
        with open(log_path, 'w') as log:
            subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT, check=False)
    keys = list(RANGES) + [k for k in X7_KEYS if k in p] + (['tail_mu'] if 'tail_mu' in p else [])
    extra = {} if 'board_kg' in p else {'board_kg': BOARD_KG}
    return dict(run=name, **{key: p[key] for key in keys}, **extra,
                **analyse(csv_path, log_path, p, ground))


def analyse(csv_path, log_path, p, ground='plane'):
    rows = list(csv.DictReader(open(csv_path)))
    g = lambda key: np.array([float(r[key]) for r in rows])
    t, v, amps, sat = g('sim_time_s'), g('forward_speed_m_s'), g('applied_amps'), g('saturated') > 0
    # Distance along the course: the heightfield spawns at x = 88 (s = 2),
    # the plane at x = 0.
    s = (90.0 if ground == 'hfield' else 0.0) - g('pos_x_m')
    fallen, pitch = g('fallen') > 0, g('truth_pitch_deg')
    n = np.nonzero(np.diff(t) < 0)[0]
    n = n[0] + 1 if len(n) else len(t)
    log = open(log_path).read()
    m = re.search(r"handoff at t=([\d.]+)s -- (bumper strike|tilt) \(bumper \d+ N: nose (\d+) N, "
                  r"tail (\d+) N, tilt ([\d.]+) deg", log)
    # The host runs on past the course end; only the course counts.
    end_m = s_end(p, ground)
    done = np.nonzero(s[:n] >= end_m)[0]
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
    status = 'PASS' if not cause and reached >= end_m else ('FALL' if cause else 'STALL')
    # Rider warning (sim-host --authority-margin, --rider-reacts).
    first = lambda pat: (lambda mm: float(mm.group(1)) if mm else None)(re.search(pat, log))
    t_warn = first(r"rider warning (?:Pulse|Solid) at sim_t=([\d.]+)s")
    t_dismount = first(r"rider dismount at sim_t=([\d.]+)s")
    t_eased = first(r"rider eases off at sim_t=([\d.]+)s")
    if t_dismount is not None and status != 'PASS' and not (cause and t[-1] < t_dismount):
        status, cause = 'DISMOUNT', ''
    elif t_eased is not None and status == 'STALL' and abs(v[-1]) < 0.3:
        status = 'EASED STOP'
    warn_lead_s = round(t[-1] - t_warn, 2) if (cause and t_warn is not None and t_warn <= t[-1]) else ''
    # Tail braking (sim-host --tail-brake): the tail pad on the road is a
    # brake, not a fall. A run that ends at rest with the tail down stopped
    # on purpose, the way a rider stops with the tail.
    tail_pct, tail_first_s = 0.0, ''
    if 'tail_strike_n' in rows[0]:
        tail_on = (g('tail_strike_n')[:end] > TAIL_CONTACT_N)
        tail_pct = round(100.0 * float(tail_on.mean()), 1)
        if tail_on.any():
            tail_first_s = round(float(s[np.argmax(tail_on)]), 1)
            if status == 'STALL' and tail_on[-250:].any() and abs(v[-1]) < 0.3:
                status = 'TAIL STOP'
    on = (s >= WINDOW[0]) & (s <= end_m)
    flat_grade = (s >= grade_start(p['grade_pct']) + 2.0) & (s <= end_m)
    vt, v0 = p['v_target'], p['v_start']
    # Overshoot past the target, in the direction of the approach, after the
    # first crossing. Zero for a run that starts at the target.
    up = vt >= v0
    # A start speed is applied at t = 1 s (sim-host START_SPEED_AT_S).
    live = t > (1.01 if v0 > 0 else 0.0)
    tol = 0.01 * vt  # a run that settles just short of the target has crossed
    cross = np.nonzero(live & (v >= vt - tol if up else v <= vt + tol))[0]
    if len(cross):
        tail_v = v[cross[0]:]
        over = (tail_v.max() - vt) if up else (vt - tail_v.min())
        overshoot = 100.0 * max(over, 0.0) / vt
    else:
        overshoot = float('nan')
    # A run that reaches the end far too fast survived, but it is not a pass:
    # on a steep descent the braking saturated and the board ran away.
    v_max = float(np.abs(v[live]).max()) if live.any() else 0.0
    v_ref_max = max(vt, v0)  # a run may start above its target
    if status == 'PASS' and v_max > max(1.5 * v_ref_max, v_ref_max + 2.0):
        status = 'RUNAWAY'
    vg = v[on]
    x7 = 'board_kg' in p
    a = SimpleNamespace(kt=0.7 * p['kt_scale'], ke=0.7 * p['kt_scale'],
                        r_phase=X7_R_PHASE_OHM if x7 else 0.12, p_idle=15.0,
                        regen_eff=0.8, series=20, parallel=2, cell_ah=X7_CELL_AH if x7 else 4.0,
                        r_cell=0.015, max_duty=0.9,
                        mass=(p['board_kg'] if x7 else BOARD_KG) + p['rider_kg'], crr=0.015, cda=0.5)
    wh_km = float('nan')
    if flat_grade.sum() > 50:
        # applied_amps is the COMMANDED current; the true current is the same
        # (Kt error changes torque, not current).
        e = battery.analyse(t[flat_grade], amps[flat_grade], v[flat_grade], a)
        wh_km = e['wh_per_km']
    return dict(
        status=status, cause=cause, reached_m=round(float(reached), 1),
        t_event_s=round(float(t[-1]), 2) if cause else '',
        v_mean=round(float(vg.mean()), 3) if len(vg) else '',
        overshoot_pct=round(overshoot, 1), v_max=round(v_max, 2),
        peak_amps=round(float(np.abs(amps).max()), 1),
        sat_pct=round(100.0 * float(sat.mean()), 1),
        peak_pitch_deg=round(float(np.abs(pitch).max()), 1),
        wh_per_km=round(wh_km, 1),
        tail_pct=tail_pct, tail_first_s=tail_first_s,
        t_warn_s=t_warn if t_warn is not None else '', warn_lead_s=warn_lead_s,
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out')
    ap.add_argument('--n', type=int, default=200)
    ap.add_argument('--jobs', type=int, default=8)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--probe', action='append', help="k=v,... over NOMINAL; repeatable")
    ap.add_argument('--ground', choices=['plane', 'hfield'], default='plane')
    ap.add_argument('--host-arg', action='append', default=[], help='extra sim-host flag; repeatable')
    ap.add_argument('--port-base', type=int, default=9000, help='UDP ports; separate parallel sweeps')
    ap.add_argument('--board-kg', type=float, help='board mass without rider (model: 13 kg)')
    ap.add_argument('--rider-law', action='store_true',
                    help='rider model + deployed balance law (--rider-speed), not the speed-hold LQR')
    ap.add_argument('--plant-x7', action='store_true',
                    help='the X7 build plant, with its mass-property dispersions (X7_RANGES)')
    ap.add_argument('--tail-brake', action='store_true',
                    help='tail pad brakes (no handoff); samples tail friction too')
    args = ap.parse_args()
    EXTRA_ARGS.extend(args.host_arg)
    if args.rider_law:
        RIDER_FLAG[0] = '--rider-speed'
    if args.board_kg:
        global BOARD_KG
        BOARD_KG = args.board_kg
        EXTRA_ARGS.extend(['--board-mass', f"{args.board_kg}"])
    out = Path(args.out).resolve()
    (out / 'runs').mkdir(parents=True, exist_ok=True)
    (out / 'passive.csv').write_text("0,300,0,0,0,passive rider\n")
    if args.probe:
        plan = []
        for spec in args.probe:
            p = dict(NOMINAL)
            for kv in filter(None, spec.split(',')):
                key, val = kv.split('=')
                p[key] = float(val)
            plan.append(p)
    else:
        rng = np.random.default_rng(args.seed)
        plan = lhs(args.n, rng, X7_RANGES if args.plant_x7 else None)
        if args.tail_brake:
            # Drawn after the six RANGES columns, so those stay the runs of
            # the same seed without tail braking.
            lo, hi = TAIL_MU_RANGE
            u = (rng.permutation(args.n) + rng.random(args.n)) / args.n
            for p, ui in zip(plan, u):
                p['tail_mu'] = lo + ui * (hi - lo)
    if args.tail_brake and args.probe:
        for p in plan:
            p.setdefault('tail_mu', TAIL_MU_NOMINAL)
    if args.ground == 'hfield':
        for p in plan:
            p['grade_pct'], p['course'] = course(p['grade_pct'], out / 'courses')
    with ThreadPoolExecutor(args.jobs) as ex:
        futs = [ex.submit(run_one, k, p, out, args.port_base + 2 * k, args.ground) for k, p in enumerate(plan)]
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
