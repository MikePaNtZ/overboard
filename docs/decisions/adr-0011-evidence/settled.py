"""The two candidate instruments, and their spread.

A. STATIC OFFSET -- zero commanded stick, late window. No acceleration, filter
   fully settled: whatever is left is the trim itself.
B. THE 1-rad/g IDENTITY -- settled tail of an accelerating hold: mean residual
   against mean atan(a_unaided/g). Ratio should be 1.00.
"""
from __future__ import annotations
import csv, math, subprocess
from pathlib import Path

REPO = Path("/Users/mike/projects/overboard")
OUT = Path("/private/tmp/claude-501/-Users-mike-projects-overboard/94162fbe-f42c-4c5a-98c3-d48ab1fac1dc/scratchpad")
G = 9.80665
ACCEL_FF_GAIN = 0.0584


def run(name, scenario, **kw):
    out = OUT / f"s_{name}.csv"
    argv = ["cargo", "run", "--release", "-q", "-p", "sim-host", "--bin", "sim-host", "--",
            "--scripted-scenario", scenario, "--free-run", "--stats-path", "none",
            "--state-out-addr", "127.0.0.1:19701", "--input-in-addr", "127.0.0.1:19702",
            "--trace-csv", str(out)]
    for k, v in kw.items():
        argv += [f"--{k.replace('_','-')}", str(v)]
    p = subprocess.run(argv, cwd=REPO, capture_output=True, text=True)
    assert p.returncode == 0, p.stderr[-3000:]
    with out.open() as f:
        return [{k: float(v) for k, v in r.items()} for r in csv.DictReader(f)]


def settled(rows, t_lo, t_hi, half=50):
    res, pred = [], []
    for i in range(half, len(rows) - half):
        t = rows[i]["sim_time_s"]
        if not t_lo <= t <= t_hi:
            continue
        a = (rows[i+half]["forward_speed_m_s"] - rows[i-half]["forward_speed_m_s"]) / (
            rows[i+half]["sim_time_s"] - rows[i-half]["sim_time_s"])
        au = a - ACCEL_FF_GAIN * rows[i]["applied_amps"]
        res.append(rows[i]["est_pitch_deg"] - rows[i]["truth_pitch_deg"])
        pred.append(math.degrees(math.atan(au / G)))
    return sum(res)/len(res), sum(pred)/len(pred), min(res), max(res)


print("A. STATIC OFFSET -- zero stick")
rows0 = run("zero", "full-stick", cmd_reserve=0.0, pitch_source="estimator")
print(f"   run length {rows0[-1]['sim_time_s']:.1f} s")
for lo, hi in ((3, 8), (5, 10), (8, 15), (10, 18), (3, 18)):
    m, p, mn, mx = settled(rows0, lo, hi)
    print(f"   [{lo:2d},{hi:2d}]  residual mean {m:+.4f}  min {mn:+.4f}  max {mx:+.4f}  (pred {p:+.4f})")

print("\nB. THE IDENTITY -- settled tail, ratio residual/atan(a_unaided/g)")
print(f"   {'case':30} {'resid':>9} {'pred':>9} {'ratio':>7}")
ratios = []
cases = [(f"full-stick r={u}", "full-stick", dict(cmd_reserve=u, pitch_source="estimator"))
         for u in (0.4, 0.6, 0.8, 1.0)]
cases += [("stick-reversal shipped", "stick-reversal", dict(pitch_source="estimator"))]
for label, scen, kw in cases:
    rows = run(label.replace(" ", "_").replace("=", "").replace(".", ""), scen, **kw)
    for lo, hi in ((12, 18),):
        m, p, _, _ = settled(rows, lo, hi)
        r = m / p if abs(p) > 1e-6 else float("nan")
        ratios.append(r)
        print(f"   {label:30} {m:+9.4f} {p:+9.4f} {r:7.4f}")

print(f"\n   ratio span {min(ratios):.4f} .. {max(ratios):.4f}")

print("\n   window sensitivity, full-stick r=0.8:")
rows = run("full-stick_r08", "full-stick", cmd_reserve=0.8, pitch_source="estimator")
for lo, hi in ((11, 15), (12, 18), (13, 19), (12, 16), (14, 20)):
    m, p, _, _ = settled(rows, lo, hi)
    print(f"     [{lo:2d},{hi:2d}]  resid {m:+.4f}  pred {p:+.4f}  ratio {m/p:.4f}")
