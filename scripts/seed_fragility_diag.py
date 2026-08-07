#!/usr/bin/env python3
"""DIAGNOSTIC ONLY -- reproduces (or refutes) the claim that terrain.py/hill.py
survive only at STAGE0_CUTBACK's shipped default seed (12345).

Sweeps STAGE0_CUTBACK.seed across N values (default 30, including 12345) on:
  * terrain.py at its own default grade (8%)
  * hill.py at its own default grade (10%), descent direction (matching
    HillParams.grade_pct's default sign)

Calls terrain.run()/hill.run() DIRECTLY with no controller_factory override,
so this measures exactly what ships (DEFAULT_MAX_PITCH_REF_RAD, default
KP/KD/kp_v/ki_v), not clamp_sweep.py's copy of those constants.

Nothing here changes any constant, gain, threshold or scenario default.
"""

from __future__ import annotations

import dataclasses
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from sim.scenarios.hill import HillParams, run as hill_run  # noqa: E402
from sim.scenarios.imperfections import STAGE0_CUTBACK  # noqa: E402
from sim.scenarios.terrain import TerrainParams, run as terrain_run  # noqa: E402

SEEDS = [12345] + list(range(0, 30))


def sweep_terrain():
    print("=== terrain.py @ default grade (8%) ===")
    print(f"{'seed':>6} {'survived':>9} {'t_strike':>9} {'struck':>8} {'phase':>9} "
          f"{'pk_pitch':>9} {'pk_pref':>8}")
    rows = []
    for seed in SEEDS:
        profile = dataclasses.replace(STAGE0_CUTBACK, seed=seed)
        r = terrain_run(TerrainParams(), profile=profile)
        m = r.metrics
        rows.append((seed, m.survived, m.t_strike_s))
        print(f"{seed:6d} {str(m.survived):>9} "
              f"{'-' if m.t_strike_s is None else f'{m.t_strike_s:9.3f}'} "
              f"{m.struck_end:>8} {m.struck_phase:>9} {m.peak_abs_pitch_deg:9.2f} "
              f"{m.peak_abs_pitch_ref_deg:8.2f}")
    n_survived = sum(1 for _, s, _ in rows if s)
    print(f"survival: {n_survived}/{len(rows)}")
    return rows


def sweep_hill():
    print("\n=== hill.py @ default grade (10%, descent) ===")
    print(f"{'seed':>6} {'survived':>9} {'t_strike':>9} {'struck':>8} {'pk_pitch':>9} "
          f"{'pk_pref':>8} {'v_final':>8}")
    rows = []
    for seed in SEEDS:
        profile = dataclasses.replace(STAGE0_CUTBACK, seed=seed)
        r = hill_run(HillParams(), profile=profile)
        m = r.metrics
        rows.append((seed, m.survived, m.t_strike_s))
        print(f"{seed:6d} {str(m.survived):>9} "
              f"{'-' if m.t_strike_s is None else f'{m.t_strike_s:9.3f}'} "
              f"{m.struck_end:>8} {m.peak_abs_pitch_deg:9.2f} "
              f"{m.peak_abs_pitch_ref_deg:8.2f} {m.v_final_m_s:8.2f}")
    n_survived = sum(1 for _, s, _ in rows if s)
    print(f"survival: {n_survived}/{len(rows)}")
    return rows


if __name__ == "__main__":
    sweep_terrain()
    sweep_hill()
