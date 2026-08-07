#!/usr/bin/env python3
"""DIAGNOSTIC ONLY. Seed-sweeps the currently-PASSING, survival-dependent
pytest cases in test_hill.py and test_shuttle_run.py, to check contamination
(question 3): would any of these flip if the profile's seed changed?

Changes nothing on disk.
"""
from __future__ import annotations

import dataclasses
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from sim.scenarios.hill import HillParams, run as hill_run  # noqa: E402
from sim.scenarios.imperfections import STAGE0_CUTBACK, STAGE0_PLACEHOLDER  # noqa: E402
from sim.scenarios import shuttle_run  # noqa: E402

SEEDS = [12345] + list(range(20))


def check_hill(label, **kwargs):
    n_survived = 0
    fails = []
    for seed in SEEDS:
        profile = dataclasses.replace(STAGE0_CUTBACK, seed=seed)
        m = hill_run(HillParams(**kwargs), profile=profile).metrics
        if m.survived:
            n_survived += 1
        else:
            fails.append((seed, m.t_strike_s, m.struck_end))
    print(f"{label}: survived {n_survived}/{len(SEEDS)}"
          + (f"  FAILS: {fails}" if fails else ""))


def check_shuttle(label):
    n_survived = 0
    fails = []
    peak_prefs = []
    for seed in SEEDS:
        profile = dataclasses.replace(STAGE0_PLACEHOLDER, seed=seed)
        m = shuttle_run.run(profile=profile).metrics
        peak_prefs.append(m.peak_abs_pitch_ref_deg)
        if not m.nose_strike:
            n_survived += 1
        else:
            fails.append(seed)
    print(f"{label}: no-strike {n_survived}/{len(SEEDS)}  "
          f"peak_pref range [{min(peak_prefs):.2f}, {max(peak_prefs):.2f}]"
          + (f"  STRUCK seeds: {fails}" if fails else ""))


if __name__ == "__main__":
    print("=== test_hill.py currently-PASSING survival cases, reseeded ===")
    check_hill("grade=10 v_ref=2 (medium_descent[2.0])", grade_pct=10.0, v_ref_m_s=2.0, duration_s=6.0)
    check_hill("grade=10 v_ref=3 (medium_descent[3.0])", grade_pct=10.0, v_ref_m_s=3.0, duration_s=6.0)
    check_hill("grade=-5 v_ref=2 (medium_climb)", grade_pct=-5.0, v_ref_m_s=2.0, duration_s=6.0)
    check_hill("grade=10 v_ref=2 (climbing_is_harder: descent)", grade_pct=10.0, v_ref_m_s=2.0, duration_s=6.0)
    check_hill("grade=-10 v_ref=2 (climbing_is_harder: climb)", grade_pct=-10.0, v_ref_m_s=2.0, duration_s=6.0)
    check_hill("grade=15 v_ref=2 TRUTH (estimator headline, use_estimator=False)",
               grade_pct=15.0, v_ref_m_s=2.0, use_estimator=False, duration_s=6.0)

    print("\n=== test_shuttle_run.py::test_it_never_strikes_and_never_saturates, reseeded ===")
    check_shuttle("shuttle_run.run() default params")
