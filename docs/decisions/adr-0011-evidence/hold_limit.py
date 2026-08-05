"""Where does the speed cap stop bounding a downhill roll?

Zero stick, board simply released on a slope. Two things reported per run:
 * whether |speed| is still RISING over the last 2 s (runaway) or falling back
   toward the cap (held);
 * the free-roll check a = g*sin(phi), taken over the first 4 s while the cap
   has not yet engaged -- the sanity check that this is really a slope.
"""
from __future__ import annotations
import csv, math, subprocess
from pathlib import Path

REPO = Path("/Users/mike/projects/overboard")
OUT = Path("/private/tmp/claude-501/-Users-mike-projects-overboard/94162fbe-f42c-4c5a-98c3-d48ab1fac1dc/scratchpad")
G_MODEL = 9.81  # overboard_rider.xml's own <option gravity>


def run(name, **kw):
    out = OUT / f"h_{name}.csv"
    argv = ["cargo", "run", "--release", "-q", "-p", "sim-host", "--bin", "sim-host", "--",
            "--scripted-scenario", "full-stick", "--free-run", "--stats-path", "none",
            "--state-out-addr", "127.0.0.1:19821", "--input-in-addr", "127.0.0.1:19822",
            "--cmd-reserve", "0.0", "--trace-csv", str(out)]
    for k, v in kw.items():
        argv += [f"--{k.replace('_','-')}", str(v)]
    p = subprocess.run(argv, cwd=REPO, capture_output=True, text=True)
    assert p.returncode == 0, p.stderr[-3000:]
    with out.open() as f:
        return [{k: float(v) for k, v in r.items()} for r in csv.DictReader(f)]


def at(rows, t):
    return min(rows, key=lambda r: abs(r["sim_time_s"] - t))


print(f"{'incline':>8} {'a(0.5-4s)':>10} {'g sin phi':>10} {'v@16.5':>8} {'v@18.5':>8} "
      f"{'d|v|/dt':>9}  verdict")
for inc in (0.25, 0.5, 1.0, 2.0, 3.0, 4.0, 5.0, 5.5, 6.0, 6.5, 7.0, 8.0, 9.0, 10.0):
    rows = run(f"{inc}", incline_deg=inc, pitch_source="estimator")
    v1, v2 = at(rows, 0.5), at(rows, 4.0)
    a = abs(v2["forward_speed_m_s"] - v1["forward_speed_m_s"]) / (v2["sim_time_s"] - v1["sim_time_s"])
    e1, e2 = at(rows, 16.5), at(rows, 18.5)
    d = (abs(e2["forward_speed_m_s"]) - abs(e1["forward_speed_m_s"])) / (
        e2["sim_time_s"] - e1["sim_time_s"])
    verdict = "RUNAWAY" if d > 0.02 else "held"
    print(f"{inc:8.2f} {a:10.3f} {G_MODEL*math.sin(math.radians(inc)):10.3f} "
          f"{abs(e1['forward_speed_m_s']):8.3f} {abs(e2['forward_speed_m_s']):8.3f} "
          f"{d:+9.3f}  {verdict}")
