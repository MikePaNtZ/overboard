from __future__ import annotations
import csv, subprocess, sys
from pathlib import Path

REPO = Path("/Users/mike/projects/overboard")
OUT = Path("/private/tmp/claude-501/-Users-mike-projects-overboard/94162fbe-f42c-4c5a-98c3-d48ab1fac1dc/scratchpad")
INVERTED = 90.0
SPEED_CAP_ONSET = 8.34


def run(name, scenario, **kw):
    out = OUT / f"w_{name}.csv"
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


def report(rows):
    t_inv = next((r["sim_time_s"] for r in rows if abs(r["truth_pitch_deg"]) > INVERTED), None)
    h = [r for r in rows if t_inv is None or r["sim_time_s"] < t_inv]
    sat = next((r["sim_time_s"] for r in h if r["saturated"] > 0.5), None)
    warn = next((r["sim_time_s"] for r in h if r["authority_warning"] > 0.5), None)
    sat_low = next((r["sim_time_s"] for r in h
                    if r["saturated"] > 0.5 and abs(r["forward_speed_m_s"]) < SPEED_CAP_ONSET), None)
    return (t_inv, max(abs(r["truth_pitch_deg"]) for r in h),
            max(abs(r["proposed_amps"]) for r in h),
            max(abs(r["forward_speed_m_s"]) for r in h), sat, sat_low, warn)


def sweep(title, scenario, inclines, **kw):
    print(f"\n{title}")
    print(f"{'incline':>8} {'invert':>8} {'pk pitch':>9} {'pk A':>7} {'pk m/s':>7} "
          f"{'sat':>7} {'sat<cap':>8} {'warn':>7}")
    for inc in inclines:
        t, p, a, v, sat, satlo, warn = report(run(f"{scenario}{inc}", scenario, incline_deg=inc, **kw))
        f = lambda x: "-" if x is None else f"{x:.2f}"
        print(f"{inc:8.2f} {f(t):>8} {p:9.3f} {a:7.2f} {v:7.3f} {f(sat):>7} {f(satlo):>8} {f(warn):>7}")


which = sys.argv[1] if len(sys.argv) > 1 else "all"
if which in ("all", "hold"):
    sweep("SHIPPED FULL-STICK HOLD (+ uphill / - downhill)", "full-stick",
          (-12, -10, -8, -6, -5, -4, -3, 0, 3, 4, 5, 6, 8, 10, 12), pitch_source="estimator")
if which in ("all", "rev"):
    sweep("STICK REVERSAL AT SPEED", "stick-reversal",
          (-8, -6, -5, -4, -3, -2, 0, 2, 3, 4, 5, 6, 8), pitch_source="estimator")
if which in ("all", "idle"):
    sweep("IDLE (zero stick) -- rolls downhill, nothing holds position", "full-stick",
          (1, 3, 5, 10, 15, 20, 25, 30), cmd_reserve=0.0, pitch_source="estimator")
