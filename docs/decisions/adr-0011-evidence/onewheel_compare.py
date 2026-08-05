"""How far are we from the CEO's cited Onewheel figure, and what sets the gap?"""
from __future__ import annotations
import csv, math, subprocess
from pathlib import Path

REPO = Path("/Users/mike/projects/overboard")
OUT = Path("/private/tmp/claude-501/-Users-mike-projects-overboard/94162fbe-f42c-4c5a-98c3-d48ab1fac1dc/scratchpad")

MAX_A, KT, R_EFF, MASS = 40.0, 0.7, 0.1454, 82.5
FT = 3.28084
BRAKE_T0 = 12.5


def run(name, scen, **kw):
    o = OUT / f"ow_{name}.csv"
    a = ["cargo", "run", "--release", "-q", "-p", "sim-host", "--bin", "sim-host", "--",
         "--scripted-scenario", scen, "--free-run", "--stats-path", "none",
         "--pitch-source", "estimator",
         "--state-out-addr", "127.0.0.1:19881", "--input-in-addr", "127.0.0.1:19882",
         "--trace-csv", str(o)]
    for k, v in kw.items():
        a += [f"--{k.replace('_','-')}", str(v)]
    p = subprocess.run(a, cwd=REPO, capture_output=True, text=True)
    assert p.returncode == 0, p.stderr[-3000:]
    return [{k: float(v) for k, v in r.items()} for r in csv.DictReader(o.open())]


def stop_from(rows, v_target):
    """Distance and time from the moment speed falls past v_target, to zero."""
    dt = rows[1]["sim_time_s"] - rows[0]["sim_time_s"]
    started, dist, t0 = False, 0.0, None
    for r in rows:
        if r["sim_time_s"] < BRAKE_T0:
            continue
        if not started:
            if r["forward_speed_m_s"] <= v_target:
                started, t0 = True, r["sim_time_s"]
            else:
                continue
        if r["forward_speed_m_s"] <= 0.0:
            return dist, r["sim_time_s"] - t0
        dist += r["forward_speed_m_s"] * dt
    return dist, None


print("THE PHYSICAL CEILING, before any tuning")
tau = MAX_A * KT
force = tau / R_EFF
a_max = force / MASS
print(f"  motor torque at the 40 A envelope : {tau:.1f} N*m")
print(f"  contact force (r_eff {R_EFF} m)   : {force:.1f} N")
print(f"  on {MASS} kg                       : {a_max:.2f} m/s^2 = {a_max/9.80665:.3f} g")
for v in (5.4, 6.7, 9.34):
    d = v * v / (2 * a_max)
    print(f"  ideal stop from {v:4.2f} m/s        : {d:5.1f} m = {d*FT:5.1f} ft "
          f"(at the ceiling, instantly, which the board cannot do)")

print("\nWHAT WE ACTUALLY DO (braking reserve 0.90)")
rows = run("stop", "brake-stop")
for v in (9.68, 6.7, 5.4):
    d, t = stop_from(rows, v)
    print(f"  measured stop from {v:4.2f} m/s   : {d:5.1f} m = {d*FT:5.1f} ft"
          + (f", {t:.2f} s" if t else ""))

peak_a = max(abs(r["proposed_amps"]) for r in rows if r["sim_time_s"] >= BRAKE_T0
             and r["forward_speed_m_s"] > 0)
print(f"\n  peak demand used during the stop  : {peak_a:.1f} A of {MAX_A:.0f} "
      f"({100*peak_a/MAX_A:.0f}% of envelope)")
print(f"  so peak decel achieved            : {peak_a*KT/R_EFF/MASS:.2f} m/s^2 "
      f"= {peak_a*KT/R_EFF/MASS/9.80665:.3f} g")

print("\nTHE CEO'S CITED REFERENCE (not independently verified here)")
print("  Onewheel quick stop: ~14 ft from cruise (12-15 mph = 5.4-6.7 m/s)")
for v in (5.4, 6.7):
    print(f"    implies {v*v/(2*4.27):.2f} m/s^2 = {v*v/(2*4.27)/9.80665:.2f} g from {v} m/s")
print("  NOTE: that figure is quoted for the TAIL-DRAG maneuver -- scraping the")
print("  bumper on the pavement. That is friction braking we do not model at all,")
print("  in addition to motor braking.")
