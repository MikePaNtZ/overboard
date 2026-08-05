"""Does the 42 A/unit peak-demand slope actually depend on the trim?"""
from __future__ import annotations
import csv, subprocess
from pathlib import Path

REPO = Path("/Users/mike/projects/overboard")
OUT = Path("/private/tmp/claude-501/-Users-mike-projects-overboard/94162fbe-f42c-4c5a-98c3-d48ab1fac1dc/scratchpad")
INVERTED = 90.0
FRACTIONS = (0.30, 0.50, 0.70, 0.90)


def run(name, **kw):
    out = OUT / f"t_{name}.csv"
    argv = ["cargo", "run", "--release", "-q", "-p", "sim-host", "--bin", "sim-host", "--",
            "--scripted-scenario", "full-stick", "--free-run", "--stats-path", "none",
            "--state-out-addr", "127.0.0.1:19831", "--input-in-addr", "127.0.0.1:19832",
            "--pitch-source", "estimator", "--trace-csv", str(out)]
    for k, v in kw.items():
        argv += [f"--{k.replace('_','-')}", str(v)]
    p = subprocess.run(argv, cwd=REPO, capture_output=True, text=True)
    assert p.returncode == 0, p.stderr[-3000:]
    with out.open() as f:
        return [{k: float(v) for k, v in r.items()} for r in csv.DictReader(f)]


def peak(rows):
    t = next((r["sim_time_s"] for r in rows if abs(r["truth_pitch_deg"]) > INVERTED), None)
    h = [r for r in rows if t is None or r["sim_time_s"] < t]
    return max(abs(r["proposed_amps"]) for r in h), t


print(f"{'bias':>7} {'slope':>8} {'d%':>7}  per-point peaks / inverted")
base = None
for bias in (-0.30, -0.20, -0.10, 0.0, 0.10, 0.20, 0.30):
    peaks, inv = [], []
    for u in FRACTIONS:
        pk, t = peak(run(f"{bias}_{u}", cmd_reserve=u, pitch_bias_deg=bias))
        peaks.append(pk)
        inv.append("-" if t is None else f"{t:.1f}")
    slope = sum(u * a for u, a in zip(FRACTIONS, peaks)) / sum(u * u for u in FRACTIONS)
    if bias == 0.0:
        base = slope
    d = "" if base is None else f"{100*(slope-base)/base:+7.2f}"
    print(f"{bias:7.2f} {slope:8.3f} {d:>7}  " +
          " ".join(f"{p:.2f}" for p in peaks) + "   inv: " + " ".join(inv))
