"""Scratch: how much do the residual slope / offset move across operating points?

Answers the only question that sets a pinned band honestly: what is the
measurement's OWN spread, so the band can be wider than the spread (or the pin
chatters) and no wider than it has to be (or the pin stops detecting drift).
"""
from __future__ import annotations

import csv
import math
import subprocess
import sys
from pathlib import Path

REPO = Path("/Users/mike/projects/overboard")
OUT = Path("/private/tmp/claude-501/-Users-mike-projects-overboard/94162fbe-f42c-4c5a-98c3-d48ab1fac1dc/scratchpad")
ACCEL_FF_GAIN = 0.0584
ONE_RAD_PER_G = math.degrees(1.0) / 9.80665


def run(name, scenario, **kw):
    out = OUT / f"{name}.csv"
    argv = [
        "cargo", "run", "--release", "-q", "-p", "sim-host", "--bin", "sim-host", "--",
        "--scripted-scenario", scenario, "--free-run", "--stats-path", "none",
        "--state-out-addr", "127.0.0.1:19701", "--input-in-addr", "127.0.0.1:19702",
        "--trace-csv", str(out),
    ]
    for k, v in kw.items():
        argv += [f"--{k.replace('_', '-')}", str(v)]
    p = subprocess.run(argv, cwd=REPO, capture_output=True, text=True)
    assert p.returncode == 0, p.stderr[-3000:]
    with out.open() as f:
        return [{k: float(v) for k, v in r.items()} for r in csv.DictReader(f)]


def fit(rows, t_lo=1.0, t_hi=15.0, half=50):
    xs, ys = [], []
    for i in range(half, len(rows) - half):
        if not t_lo <= rows[i]["sim_time_s"] <= t_hi:
            continue
        a_x = (rows[i + half]["forward_speed_m_s"] - rows[i - half]["forward_speed_m_s"]) / (
            rows[i + half]["sim_time_s"] - rows[i - half]["sim_time_s"]
        )
        xs.append(a_x - ACCEL_FF_GAIN * rows[i]["applied_amps"])
        ys.append(rows[i]["est_pitch_deg"] - rows[i]["truth_pitch_deg"])
    n = len(xs)
    sx, sy = sum(xs), sum(ys)
    slope = (n * sum(x * y for x, y in zip(xs, ys)) - sx * sy) / (n * sum(x * x for x in xs) - sx * sx)
    return slope, (sy - slope * sx) / n, n


def resting_residual(rows, t_lo, t_hi):
    """Mean est-truth over a window, degrees -- the trim in its plainest form."""
    vals = [r["est_pitch_deg"] - r["truth_pitch_deg"] for r in rows
            if t_lo <= r["sim_time_s"] <= t_hi]
    return sum(vals) / len(vals), min(vals), max(vals)


print(f"1 rad/g = {ONE_RAD_PER_G:.4f} deg per m/s^2\n")
print(f"{'case':38} {'slope':>8} {'dev%':>7} {'icept':>8} {'n':>6}")
cases = []
for u in (0.4, 0.5, 0.6, 0.7, 0.8, 0.9):
    cases.append((f"full-stick reserve={u}", "full-stick", dict(cmd_reserve=u, pitch_source="estimator")))
cases.append(("stick-reversal shipped", "stick-reversal", dict(pitch_source="estimator")))

results = {}
for label, scen, kw in cases:
    rows = run(label.replace(" ", "_").replace("=", ""), scen, **kw)
    s, b, n = fit(rows)
    results[label] = (s, b)
    print(f"{label:38} {s:8.3f} {100*(s-ONE_RAD_PER_G)/ONE_RAD_PER_G:+7.1f} {b:+8.3f} {n:6d}")

slopes = [s for s, _ in results.values()]
icepts = [b for _, b in results.values()]
print(f"\nslope   span {min(slopes):.3f} .. {max(slopes):.3f}  "
      f"(dev from 1 rad/g: {100*(min(slopes)-ONE_RAD_PER_G)/ONE_RAD_PER_G:+.1f}% .. "
      f"{100*(max(slopes)-ONE_RAD_PER_G)/ONE_RAD_PER_G:+.1f}%)")
print(f"intercept span {min(icepts):+.3f} .. {max(icepts):+.3f} deg")

# fit-window sensitivity on the reference case
print("\nfit-window sensitivity, full-stick reserve=0.6:")
rows = run("winsens", "full-stick", cmd_reserve=0.6, pitch_source="estimator")
for lo, hi in ((1.0, 15.0), (2.0, 15.0), (1.0, 12.0), (3.0, 18.0), (1.0, 20.0)):
    s, b, n = fit(rows, lo, hi)
    print(f"  [{lo:4.1f},{hi:4.1f}]  slope {s:7.3f}  intercept {b:+7.3f}  n={n}")

print("\nquiescent residual (no stick yet) -- full-stick run, t in [0.2,0.8]:")
m, lo, hi = resting_residual(rows, 0.2, 0.8)
print(f"  mean {m:+.4f} deg, range {lo:+.4f} .. {hi:+.4f}")
