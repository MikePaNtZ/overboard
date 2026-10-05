"""Wipeout and edge-case runs on the city_hill street, computed in MuJoCo and
rendered low-res in MuJoCo (render_pose.py).

Every run: the deployed balance law with grade compensation, the rider model
leaning to its target speed (10 cm reach), the X7 plant, the motor and pack
limits, tail braking, and --tumble (at a fall the rider comes off and MuJoCo
computes the slide). The rider IGNORES the warning unless the run says so.

    $PY sim/carve/wipeouts.py OUT [--only NAME ...] [--no-render]

Per run: OUT/NAME/pose.csv (sim-host --pose-out), trace.csv, host.txt, and
NAME.mp4. Nothing here computes physics outside MuJoCo.
"""
import argparse
import os
import subprocess
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

HERE = Path(__file__).resolve().parent
COURSE = Path('/Users/mike/projects/overboard-viz/out/carve-lab/data/courses/city_hill')
TOP_X = 88.0      # the start of the street (s = 2 m); downhill is -X
CLIMB_X = -5.0    # the flat intersection before the 12 % climb


def weave(t0, t1, amp, half_period, label):
    """Full-stick style reversals: steer +amp, -amp, ... from t0 to t1."""
    # The first turn is half as long, so the path swings about the centre line
    # (a full first turn drifts the board 6 m sideways, into the kerb).
    rows, t, sign, dt = [], t0, 1, half_period / 2
    while t < t1:
        rows.append((t, min(t + dt, t1), sign * amp, label))
        t, sign, dt = t + dt, -sign, half_period
    return rows


TD_FA = float(os.environ.get('TD_FA', '-0.7'))
TD_AMP = float(os.environ.get('TD_AMP', '0.3'))
CARVE_AMP = float(os.environ.get('CARVE_AMP', '0.2'))   # the hardest carve without a fall (0.25 falls)

# name: dict(title, rider kg, amps, v target, steer rows (t0, t1, steer, label),
#            extra flags, env, camera)
RUNS = {
    'kerb_hit': dict(
        title='Kerb hit at speed on the descent, 95 kg',
        kg=95, amps=90, v=7.0, x=TOP_X, extra=['--start-speed', '6'],
        steer=[(0, 3, 0.0, 'straight'), (3, 40, 0.08, 'drifts toward the kerb')],
        cam=dict(az=-60)),
    'nosedive': dict(
        title='Too fast down the hill, into the climb: nosedive, 95 kg, 60 A motor, warning ignored',
        kg=95, amps=60, v=14.0, x=TOP_X, extra=['--start-speed', '6'],
        steer=[(0, 40, 0.0, 'straight')],
        cam=dict(side=True, az=90)),
    'tail_drag': dict(
        title='Fast descent, hard brake: tail drag, then turns (45 A motor; 90 A brakes without the tail), 95 kg',
        # A scripted rider: leans back hard and holds it (the rider model's
        # speed loop swings its lean end to end after a hard brake).
        kg=95, amps=int(os.environ.get('TD_A', '45')), v=None, x=TOP_X, extra=['--start-speed', '8'],
        steer=[(0, 3, 0.0, 'fast', 0.0), (3, 6, 0.0, 'hard brake: lean back', TD_FA)]
              + [r + (TD_FA * 0.6,) for r in weave(6, 30, TD_AMP, 0.8, 'wild turns, still braking')],
        cam=dict(side=True, az=90, dist=5.5)),
    'hardest_carve': dict(
        title='Hardest carves that hold on the 15 % descent, 95 kg',
        kg=95, amps=90, v=6.0, x=TOP_X, extra=['--start-speed', '5'],
        steer=[(0, 4, 0.0, 'straight')] + weave(4, 16, CARVE_AMP, 1.1, 'hard carve'),
        cam=dict(dist=6.5)),
    'carve_fall': dict(
        title='Carve too hard at speed on the descent: fall, 95 kg',
        kg=95, amps=90, v=7.5, x=TOP_X, extra=['--start-speed', '5'],
        steer=[(0, 4, 0.0, 'straight')] + weave(4, 20, 0.35, 1.1, 'carve too hard'),
        cam=dict(dist=6.0)),
    'step_off': dict(
        title='Heavy rider, small motor, 12 % climb: warning, rider steps off',
        kg=110, amps=35, v=4.0, x=CLIMB_X, kt=0.88,
        extra=['--start-speed', '4', '--rider-reacts'],
        steer=[(0, 40, 0.0, 'straight')],
        cam=dict(side=True, az=90)),
}


def run(name, spec, out, port, render):
    d = out / name
    d.mkdir(parents=True, exist_ok=True)
    sched = d / 'steer.csv'
    # Rows: (t0, t1, steer, label) or (t0, t1, steer, label, fore_aft). The
    # fore/aft column is used only by a scripted rider (v=None, no rider model).
    sched.write_text(''.join(f"{r[0]},{r[1]},{r[4] if len(r) > 4 else 0},0,{r[2]},{r[3]}\n"
                             for r in spec['steer']))
    cmd = [os.environ['SIMHOST'], '--lean-steer', '--estimator-aiding', 'grade-aware',
           '--plant', 'x7', '--max-current', str(spec['amps']), '--rider-mass', str(spec['kg']),
           '--kt-scale', str(spec.get('kt', 0.658 / 0.7)),
           '--terrain', str(COURSE / 'course_hfield.bin'), '--spawn-x', str(spec['x']),
           '--tail-brake', '--tail-friction', '0.6', '--authority-margin', 'warn',
           *(['--rider-speed', str(spec['v'])] if spec['v'] else []), '--balance-comp', '--rider-reach', '0.10',
           '--motor-limits', '--tumble', '--pose-out', str(d / 'pose.csv'),
           '--stop-after-handoff', '4', '--schedule-csv', str(sched),
           '--free-run', '--max-sim-secs', '40', '--duration-secs', '3600', '--stats-path', 'none',
           '--state-out-addr', f'127.0.0.1:{port}', '--input-in-addr', f'127.0.0.1:{port + 1}',
           '--trace-csv', str(d / 'trace.csv'), *spec.get('extra', [])]
    env = {**os.environ, **spec.get('env', {})}
    with open(d / 'host.txt', 'w') as log:
        subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT, env=env, check=False)
    events = [l.split('sim-host: ', 1)[1].strip() for l in open(d / 'host.txt')
              if any(k in l for k in ('handoff at', 'comes off', 'steps off', 'dismount', 'rider eases',
                                      'rider warning', 'balance torque peak'))]
    if render:
        cam = spec.get('cam', {})
        args = [os.environ['PY'], str(HERE / 'render_pose.py'), str(d / 'pose.csv'), str(d / f'{name}.mp4'),
                '--title', spec['title'], '--t0', '1.0']
        if cam.get('side'):
            args.append('--side')
        for k in ('az', 'dist'):
            if k in cam:
                args += [f'--{k}', str(cam[k])]
        subprocess.run(args, check=False, capture_output=True)
    return name, events


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out')
    ap.add_argument('--only', nargs='*')
    ap.add_argument('--no-render', action='store_true')
    a = ap.parse_args()
    out = Path(a.out).resolve()
    names = a.only or list(RUNS)
    with ThreadPoolExecutor(len(names)) as ex:
        base = int(os.environ.get('PORT_BASE', '9400'))
        futs = [ex.submit(run, n, RUNS[n], out, base + 2 * i, not a.no_render) for i, n in enumerate(names)]
        for f in futs:
            name, events = f.result()
            print(f"{name}:")
            for e in events[:6]:
                print(f"    {e[:150]}")


if __name__ == '__main__':
    main()
