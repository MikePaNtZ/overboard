"""Measure the authored-terrain incline the board tolerates. ADR-0011 (f)/2."""
from __future__ import annotations
import csv, subprocess, sys
from pathlib import Path

REPO = Path("/Users/mike/projects/overboard")
OUT = Path("/private/tmp/claude-501/-Users-mike-projects-overboard/94162fbe-f42c-4c5a-98c3-d48ab1fac1dc/scratchpad")
INVERTED = 90.0


def run(name, scenario, **kw):
    out = OUT / f"i_{name}.csv"
    argv = ["cargo", "run", "--release", "-q", "-p", "sim-host", "--bin", "sim-host", "--",
            "--scripted-scenario", scenario, "--free-run", "--stats-path", "none",
            "--state-out-addr", "127.0.0.1:19801", "--input-in-addr", "127.0.0.1:19802",
            "--trace-csv", str(out)]
    for k, v in kw.items():
        argv += [f"--{k.replace('_','-')}", str(v)]
    p = subprocess.run(argv, cwd=REPO, capture_output=True, text=True)
    assert p.returncode == 0, p.stderr[-3000:]
    with out.open() as f:
        return [{k: float(v) for k, v in r.items()} for r in csv.DictReader(f)]


def inversion(rows):
    return next((r["sim_time_s"] for r in rows if abs(r["truth_pitch_deg"]) > INVERTED), None)


def healthy(rows):
    t = inversion(rows)
    return [r for r in rows if t is None or r["sim_time_s"] < t]


def summarise(rows):
    h = healthy(rows)
    return (inversion(rows),
            max(abs(r["truth_pitch_deg"]) for r in h),
            max(abs(r["proposed_amps"]) for r in h))


# 0. positive control: 0 must be a no-op, and the flag must actually do something.
a = run("zero_a", "full-stick", incline_deg=0.0, pitch_source="estimator")
b = run("zero_b", "full-stick", pitch_source="estimator")
same = all(x == y for ra, rb in zip(a, b) for x, y in zip(ra.values(), rb.values()))
print(f"--incline-deg 0 is bit-identical to omitting it: {same}")

which = sys.argv[1] if len(sys.argv) > 1 else "all"

if which in ("all", "idle"):
    print("\nIDLE on a slope (zero stick) -- does it just stand there?")
    print(f"{'incline':>8} {'inverted at':>12} {'peak pitch':>11} {'peak A':>8}")
    for inc in (0.0, 0.5, 1.0, 2.0, 3.0, 4.0, 5.0, 7.0, 10.0):
        t, p, amps = summarise(run(f"idle{inc}", "full-stick", incline_deg=inc,
                                   cmd_reserve=0.0, pitch_source="estimator"))
        print(f"{inc:8.2f} {('-' if t is None else f'{t:.3f}'):>12} {p:11.3f} {amps:8.2f}")

if which in ("all", "hold"):
    print("\nSHIPPED FULL-STICK HOLD on a slope (uphill positive, downhill negative)")
    print(f"{'incline':>8} {'inverted at':>12} {'peak pitch':>11} {'peak A':>8}")
    for inc in (-3.0, -2.0, -1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5, 2.0, 3.0):
        t, p, amps = summarise(run(f"hold{inc}", "full-stick", incline_deg=inc,
                                   pitch_source="estimator"))
        print(f"{inc:8.2f} {('-' if t is None else f'{t:.3f}'):>12} {p:11.3f} {amps:8.2f}")

if which in ("all", "rev"):
    print("\nSTICK REVERSAL AT SPEED on a slope -- the matrix's worst point")
    print(f"{'incline':>8} {'inverted at':>12} {'peak pitch':>11} {'peak A':>8}")
    for inc in (-2.0, -1.0, -0.5, -0.25, 0.0, 0.25, 0.5, 1.0, 2.0):
        t, p, amps = summarise(run(f"rev{inc}", "stick-reversal", incline_deg=inc,
                                   pitch_source="estimator"))
        print(f"{inc:8.2f} {('-' if t is None else f'{t:.3f}'):>12} {p:11.3f} {amps:8.2f}")
