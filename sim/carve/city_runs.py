"""Closed-loop speed-hold runs on the city_hill course, recorded as the pose
tracks (.npz, wire v3) that the Unreal city level replays.

Each run is a passive rider on --speed-hold with lean-steer, grade-aware
estimator aiding and the smooth wheel contact on the course heightfield. Per
run: OUT/NAME/track.npz (record.py), trace.csv (sim-host trace) and host.txt
(sim-host log: rider warnings, dismount, handoff).

    $PY sim/carve/city_runs.py OUT [--only NAME ...]

The course directory is the one the render level uses (overboard-viz
carve-lab data). Nothing here computes physics outside MuJoCo.
"""
import argparse
import os
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

HERE = Path(__file__).resolve().parent
COURSE = Path('/Users/mike/projects/overboard-viz/out/carve-lab/data/courses/city_hill')
COURSE_LEN_M = 182.0

# name: (rider_kg, kt_scale, max_current_a, v_target, extra sim-host flags)
RUNS = {
    # Mike, 95 kg, on the hardware session's target motor (60 A at Kt 0.7).
    'mike_95kg_4ms': (95.0, 1.0, 60.0, 4.0, ['--authority-margin', 'warn']),
    # The Monte Carlo hazard: heavy rider, weak motor, the 12 % climb.
    'heavy_weak_no_warning': (110.0, 0.88, 35.0, 4.0, []),
    'heavy_weak_rider_reacts': (110.0, 0.88, 35.0, 4.0,
                                ['--authority-margin', 'limit', '--rider-reacts']),
    # Tail braking on the 15 % descent: a heavy rider on a small motor.
    'tail_brake_descent': (100.0, 0.95, 32.0, 5.0, ['--authority-margin', 'warn']),
    # A quick descent for the hero shot.
    'fast_descent_70kg': (70.0, 1.0, 60.0, 7.0, ['--authority-margin', 'warn']),
}


def run(name, spec, out, port):
    kg, kt, amps, v, extra = spec
    d = out / name
    d.mkdir(parents=True, exist_ok=True)
    (d / 'passive.csv').write_text("0,300,0,0,0,passive rider\n")
    secs = COURSE_LEN_M / v + 30.0
    rec = subprocess.Popen([os.environ['PY'], str(HERE / 'record.py'), str(port), str(d / 'track.npz'),
                            str(secs * 10 + 60)], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    cmd = [os.environ['SIMHOST'], '--lean-steer', '--estimator-aiding', 'grade-aware',
           '--max-current', f"{amps}", '--speed-hold', f"{v}", '--rider-mass', f"{kg}",
           '--kt-scale', f"{kt}", '--spawn-x', '88', '--terrain', str(COURSE / 'course_hfield.bin'),
           '--tail-brake', '--tail-friction', '0.6',
           '--schedule-csv', str(d / 'passive.csv'), '--duration-secs', '3600',
           '--max-sim-secs', f"{secs:.0f}", '--free-run',
           '--state-out-addr', f"127.0.0.1:{port}", '--input-in-addr', f"127.0.0.1:{port + 1}",
           '--stats-path', 'none', '--trace-csv', str(d / 'trace.csv'), *extra]
    with open(d / 'host.txt', 'w') as log:
        subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT, check=False)
    msg = rec.communicate()[0].strip()
    events = [l.split('sim-host: ', 1)[1] for l in open(d / 'host.txt')
              if any(k in l for k in ('rider warning', 'rider eases', 'rider dismount', 'handoff at'))]
    return name, msg, events


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out')
    ap.add_argument('--only', nargs='*')
    a = ap.parse_args()
    out = Path(a.out).resolve()
    names = a.only or list(RUNS)
    with ThreadPoolExecutor(len(names)) as ex:
        futs = [ex.submit(run, n, RUNS[n], out, 9300 + 2 * i) for i, n in enumerate(names)]
        for f in futs:
            name, msg, events = f.result()
            print(f"{name}: {msg}")
            for e in events[:8]:
                print(f"    {e}")


if __name__ == '__main__':
    sys.exit(main())
