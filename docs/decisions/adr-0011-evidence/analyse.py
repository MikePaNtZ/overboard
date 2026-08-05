#!/usr/bin/env python3
"""Scratch analysis of sim-host --trace-csv output. Not part of the repo."""
from __future__ import annotations
import csv, math, sys
from pathlib import Path

DT = 0.002
FALLEN_DEG = 20.0
MAX_A = 40.0
KT = 0.7
KP = 140.0
PITCH_CEILING_DEG = math.degrees(MAX_A * KT / KP)  # 11.4592


def load(path):
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            rows.append({k: (float(v) if k not in ("seq",) else int(v)) for k, v in r.items()})
    return rows


def first_time(rows, pred):
    for r in rows:
        if pred(r):
            return r["sim_time_s"]
    return None


def summary(path, hold_from=None, hold_to=None):
    rows = load(path)
    t_flip = first_time(rows, lambda r: abs(r["truth_pitch_deg"]) > 90.0)
    t_fallen = first_time(rows, lambda r: abs(r["truth_pitch_deg"]) > FALLEN_DEG)
    t_warn = first_time(rows, lambda r: r["authority_warning"] > 0.5)
    t_sat = first_time(rows, lambda r: r["saturated"] > 0.5)
    # window for "healthy" statistics: before divergence (|pitch|>20) or whole run
    end = t_fallen if t_fallen is not None else 1e9
    pre = [r for r in rows if r["sim_time_s"] < end]
    peak_demand = max((abs(r["proposed_amps"]) for r in pre), default=0.0)
    peak_pitch = max((abs(r["truth_pitch_deg"]) for r in pre), default=0.0)
    peak_speed = max((abs(r["forward_speed_m_s"]) for r in pre), default=0.0)
    peak_est_err = max((abs(r["truth_pitch_deg"] - r["est_pitch_deg"]) for r in pre), default=0.0)
    # steady window
    if hold_from is not None:
        win = [r for r in pre if hold_from <= r["sim_time_s"] <= (hold_to or 1e9)]
    else:
        win = pre
    steady_pitch = (sum(r["truth_pitch_deg"] for r in win) / len(win)) if win else float("nan")
    steady_demand = (sum(abs(r["proposed_amps"]) for r in win) / len(win)) if win else float("nan")
    # kinematics
    dist = 0.0
    t8 = None
    for r in rows:
        dist += r["forward_speed_m_s"] * DT
        if t8 is None and r["forward_speed_m_s"] >= 8.0:
            t8 = r["sim_time_s"]
    dist15 = 0.0
    for r in rows:
        if r["sim_time_s"] <= 15.0:
            dist15 += r["forward_speed_m_s"] * DT
    return dict(
        path=Path(path).name, n=len(rows),
        t_flip=t_flip, t_fallen=t_fallen, t_warn=t_warn, t_sat=t_sat,
        peak_demand_a=peak_demand, peak_pitch_deg=peak_pitch, peak_speed=peak_speed,
        steady_pitch_deg=steady_pitch, steady_demand_a=steady_demand,
        peak_est_err_deg=peak_est_err,
        t_to_8=t8, dist_15s=dist15, dist_total=dist,
        current_headroom_a=MAX_A - peak_demand,
        pitch_headroom_deg=PITCH_CEILING_DEG - peak_pitch,
    )


def fmt(d):
    def g(k, f="{:.3f}"):
        v = d[k]
        return "n/a" if v is None else f.format(v)
    return (f"{d['path']:<38} n={d['n']:<6} flip@{g('t_flip')} fallen@{g('t_fallen')} "
            f"warn@{g('t_warn')} sat@{g('t_sat')} peakA={g('peak_demand_a','{:.2f}')} "
            f"peakPitch={g('peak_pitch_deg','{:.2f}')} steadyPitch={g('steady_pitch_deg','{:.2f}')} "
            f"steadyA={g('steady_demand_a','{:.2f}')} vmax={g('peak_speed','{:.2f}')} "
            f"t8={g('t_to_8')} d15={g('dist_15s','{:.2f}')} estErr={g('peak_est_err_deg','{:.2f}')}")


if __name__ == "__main__":
    hf = float(sys.argv[1]) if len(sys.argv) > 1 and sys.argv[1] != "-" else None
    ht = float(sys.argv[2]) if len(sys.argv) > 2 and sys.argv[2] != "-" else None
    for p in sys.argv[3:]:
        print(fmt(summary(p, hf, ht)))
