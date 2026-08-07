#!/usr/bin/env python3
"""DIAGNOSTIC ONLY. Follow-up to seed_fragility_diag.py's surprise: at seed
12345 (the shipped default), `terrain.py` at ITS OWN default (8%) already
fails via `run(TerrainParams())` / pytest -- not near-instant, at t=3.9s,
peak_pitch_ref PEGGED at the clamp. That is a DIFFERENT failure signature
than the campaign's claimed 0.25-2.0s settle-window collapse at sub-clamp
pitch_ref. This script isolates which grades/profiles reproduce which
signature, using the SAME scenario grades clamp_sweep.py's own sweeps use
(hill: 12/18%; terrain: 8/10/12/15/18%), plus the literal NOISELESS_CUTBACK
profile the campaign's docstring names.

Changes nothing on disk.
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

NOISELESS_CUTBACK = dataclasses.replace(
    STAGE0_CUTBACK, gyro_noise_rad_s=0.0, gyro_bias_rad_s=0.0, accel_noise_m_s2=0.0
)


def report(label, m, v_final=None):
    outcome = (f"STRUCK {m.struck_end}@{m.t_strike_s:.3f}s"
               if not m.survived else "survived")
    extra = f" v_final={v_final:.2f}" if v_final is not None else ""
    print(f"  {label:<45} {outcome:<22} pk_pitch={m.peak_abs_pitch_deg:6.2f} "
          f"pk_pref={m.peak_abs_pitch_ref_deg:5.2f}{extra}")


def hill_grades():
    print("=== hill.py: grade x profile (seed 12345 vs noiseless) ===")
    for grade in (10.0, 12.0, 18.0):
        for name, profile in (("STAGE0_CUTBACK seed=12345", STAGE0_CUTBACK),
                               ("NOISELESS_CUTBACK seed=12345", NOISELESS_CUTBACK)):
            r = hill_run(HillParams(grade_pct=grade), profile=profile)
            report(f"grade={grade:>5.1f}% {name}", r.metrics, r.metrics.v_final_m_s)


def hill_reseed_at_campaign_grades():
    print("\n=== hill.py: reseed STAGE0_CUTBACK at campaign's own grades (12/18%) ===")
    for grade in (12.0, 18.0):
        n_survived = 0
        for seed in range(30):
            profile = dataclasses.replace(STAGE0_CUTBACK, seed=seed)
            r = hill_run(HillParams(grade_pct=grade), profile=profile)
            if r.metrics.survived:
                n_survived += 1
            else:
                print(f"  grade={grade:.0f}% seed={seed}: STRUCK {r.metrics.struck_end} "
                      f"@{r.metrics.t_strike_s:.3f}s pk_pref={r.metrics.peak_abs_pitch_ref_deg:.2f}")
        print(f"  grade={grade:.0f}%: survived {n_survived}/30")


def terrain_grades():
    print("\n=== terrain.py: grade x profile (seed 12345 vs noiseless) ===")
    for grade in (8.0, 10.0, 12.0, 15.0, 18.0):
        for name, profile in (("STAGE0_CUTBACK seed=12345", STAGE0_CUTBACK),
                               ("NOISELESS_CUTBACK seed=12345", NOISELESS_CUTBACK)):
            r = terrain_run(TerrainParams(max_grade_pct=grade), profile=profile)
            report(f"grade={grade:>5.1f}% {name}", r.metrics)


if __name__ == "__main__":
    hill_grades()
    hill_reseed_at_campaign_grades()
    terrain_grades()
