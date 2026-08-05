"""What does braking authority buy, and what does it cost?

Buys: stopping time and distance from cruise (brake-stop).
Costs: headroom at ADR-0011's worst matrix point (stick-reversal), which is a
braking event by the opposition test and therefore spends this same reserve.
"""
from __future__ import annotations
import csv, subprocess
from pathlib import Path

REPO = Path("/Users/mike/projects/overboard")
OUT = Path("/private/tmp/claude-501/-Users-mike-projects-overboard/94162fbe-f42c-4c5a-98c3-d48ab1fac1dc/scratchpad")
MAX_A, KT, KP = 40.0, 0.7, 140.0
import math
PITCH_CEIL = math.degrees(MAX_A * KT / KP)
BRAKE_T0 = 12.5  # ACCEPTANCE_SETTLE_S + BRAKE_BUILD_S


def run(name, scenario, **kw):
    out = OUT / f"b_{name}.csv"
    argv = ["cargo", "run", "--release", "-q", "-p", "sim-host", "--bin", "sim-host", "--",
            "--scripted-scenario", scenario, "--free-run", "--stats-path", "none",
            "--pitch-source", "estimator",
            "--state-out-addr", "127.0.0.1:19851", "--input-in-addr", "127.0.0.1:19852",
            "--trace-csv", str(out)]
    for k, v in kw.items():
        argv += [f"--{k.replace('_','-')}", str(v)]
    p = subprocess.run(argv, cwd=REPO, capture_output=True, text=True)
    assert p.returncode == 0, p.stderr[-3000:]
    with out.open() as f:
        return [{k: float(v) for k, v in r.items()} for r in csv.DictReader(f)]


def inverted(rows):
    return next((r["sim_time_s"] for r in rows if abs(r["truth_pitch_deg"]) > 90.0), None)


def healthy(rows):
    t = inverted(rows)
    return [r for r in rows if t is None or r["sim_time_s"] < t]


def stop_metrics(rows):
    """Time and distance from brake application to zero forward speed."""
    dt = rows[1]["sim_time_s"] - rows[0]["sim_time_s"]
    v0 = next(r["forward_speed_m_s"] for r in rows if abs(r["sim_time_s"] - BRAKE_T0) < 1e-3)
    dist, t_stop = 0.0, None
    for r in rows:
        if r["sim_time_s"] < BRAKE_T0:
            continue
        if t_stop is None:
            if r["forward_speed_m_s"] <= 0.0:
                t_stop = r["sim_time_s"] - BRAKE_T0
                break
            dist += r["forward_speed_m_s"] * dt
    return v0, t_stop, dist


print("BUYS -- brake-stop from cruise")
print(f"{'braking':>8} {'v0':>7} {'t_stop':>8} {'dist':>8} {'pk A':>7} {'pk lean':>8} {'inv':>6}")
base = None
for br in (0.80, 0.85, 0.90, 0.92, 0.95, 1.00):
    rows = run(f"stop{br}", "brake-stop", cmd_reserve_braking=br)
    v0, ts, d = stop_metrics(rows)
    h = healthy(rows)
    pk = max(abs(r["proposed_amps"]) for r in h)
    pl = max(abs(r["truth_pitch_deg"]) for r in h)
    if base is None:
        base = (ts, d)
    print(f"{br:8.2f} {v0:7.3f} {ts if ts is None else f'{ts:8.3f}':>8} {d:8.2f} "
          f"{pk:7.2f} {pl:8.2f} {str(inverted(rows) is not None):>6}")
print(f"  (baseline 0.80: t_stop {base[0]:.3f} s, distance {base[1]:.2f} m)")

print("\nCOSTS -- ADR-0011 worst matrix point (stick-reversal at speed)")
print(f"{'braking':>8} {'pk A':>7} {'A left':>7} {'pk lean':>8} {'deg left':>9} {'inv':>6} {'warn':>6}")
for br in (0.80, 0.85, 0.90, 0.92, 0.95, 1.00):
    rows = run(f"rev{br}", "stick-reversal", cmd_reserve_braking=br)
    h = healthy(rows)
    pk = max(abs(r["proposed_amps"]) for r in h)
    pl = max(abs(r["truth_pitch_deg"]) for r in h)
    warn = any(r["authority_warning"] > 0.5 for r in h)
    print(f"{br:8.2f} {pk:7.2f} {MAX_A-pk:7.2f} {pl:8.2f} {PITCH_CEIL-pl:9.2f} "
          f"{str(inverted(rows) is not None):>6} {str(warn):>6}")

print("\nREGRESSION GUARD -- full-stick hold must be untouched (accelerating only)")
a = run("fs080", "full-stick", cmd_reserve_braking=0.80)
b = run("fs100", "full-stick", cmd_reserve_braking=1.00)
same = [list(r.values()) for r in a] == [list(r.values()) for r in b]
print(f"  full-stick identical at braking reserve 0.80 vs 1.00: {same}")
