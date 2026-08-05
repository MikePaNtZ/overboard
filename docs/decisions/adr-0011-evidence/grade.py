"""C: free-roll downhill-forward (no corridor). D: steepest grade held at full stick."""
from __future__ import annotations
import csv, subprocess
from pathlib import Path

REPO = Path("/Users/mike/projects/overboard")
OUT = Path("/private/tmp/claude-501/-Users-mike-projects-overboard/94162fbe-f42c-4c5a-98c3-d48ab1fac1dc/scratchpad")


def run(name, **kw):
    out = OUT / f"g_{name}.csv"
    argv = ["cargo", "run", "--release", "-q", "-p", "sim-host", "--bin", "sim-host", "--",
            "--scripted-scenario", "full-stick", "--free-run", "--stats-path", "none",
            "--pitch-source", "estimator",
            "--state-out-addr", "127.0.0.1:19841", "--input-in-addr", "127.0.0.1:19842",
            "--trace-csv", str(out)]
    for k, v in kw.items():
        argv += [f"--{k.replace('_','-')}", str(v)]
    p = subprocess.run(argv, cwd=REPO, capture_output=True, text=True)
    assert p.returncode == 0, p.stderr[-3000:]
    with out.open() as f:
        return [{k: float(v) for k, v in r.items()} for r in csv.DictReader(f)]


print("C: released downhill-FORWARD (700 m of corridor), stick zero")
print(f"{'incline':>8} {'v end':>8} {'trend':>8} {'corridor?':>10}")
for phi in (-0.5, -1.0, -3.0, -5.0, -7.0, -10.0, -15.0):
    rows = run(f"c{phi}", incline_deg=phi, cmd_reserve=0.0)
    e, p0 = rows[-1], min(rows, key=lambda r: abs(r["sim_time_s"] - 16.5))
    braked = any(abs(r["applied_fore_aft"]) > 1e-9 for r in rows)
    print(f"{phi:8.2f} {e['forward_speed_m_s']:8.3f} "
          f"{(abs(e['forward_speed_m_s'])-abs(p0['forward_speed_m_s']))/2:+8.3f} {str(braked):>10}")

print("\nD: full stick FORWARD on an uphill -- steepest grade it holds")
print(f"{'incline':>8} {'v end':>9} {'v min':>9} {'peak A':>8}  verdict")
for phi in (2.0, 3.0, 4.0, 4.5, 5.0, 5.5, 6.0, 7.0, 8.0):
    rows = run(f"d{phi}", incline_deg=phi)
    # forward is -X and forward_speed is positive-forward; climbing = stays > 0
    end = rows[-1]["forward_speed_m_s"]
    vmin = min(r["forward_speed_m_s"] for r in rows if r["sim_time_s"] > 2.0)
    braked = any(abs(r["applied_fore_aft"] - r["shaped_fore_aft"]) > 1e-9 for r in rows)
    print(f"{phi:8.2f} {end:9.3f} {vmin:9.3f} "
          f"{max(abs(r['proposed_amps']) for r in rows):8.2f}  "
          f"{'CLIMBS' if vmin > 0 else 'slides back'}{'  [corridor!]' if braked else ''}")
