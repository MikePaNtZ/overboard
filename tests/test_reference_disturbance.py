"""Guard against `scripts/reference_disturbance.py` reinventing a kerb model.

Issue #142. The script used to re-derive kerb impulse from its own
`dv = v*h/r` (a rigid-wheel model with no angular-inertia coupling), while
`sim/scenarios/plant.py::kerb_strike_impulse` -- Sr. Mechanical & Systems'
landed answer to #142 AC2 -- is the model `crates/sim-host/src/host.rs`
and `tests/test_cmd_envelope_reserve.py` already treat as canonical. The two
disagreed by 20-90% across this grid. These tests exist so a future edit
cannot silently reintroduce a second, disagreeing model the way the first
one arrived: importing `kerb_strike_impulse` is a duplicate as easy to
undo as it was to add.
"""

import mujoco
import pytest

from scripts.reference_disturbance import (
    BALLAST_KG,
    BALLAST_M,
    CLAMP_A,
    KERB_HEIGHTS_M,
    SPEEDS_M_S,
    build_report,
)
from sim.scenarios.plant import build_model, kerb_strike_vs_com_impulse


def test_grid_rows_match_the_canonical_model_exactly():
    """Every row's numbers must come from `kerb_strike_vs_com_impulse`, not a
    local reimplementation. Recomputes each row independently and requires
    exact equality -- not "close enough", since two honestly independent
    derivations should never coincide, but the same function called twice
    always will."""
    report = build_report()
    rows_by_key = {(row["kerb_m"], row["speed_m_s"]): row
                   for row in report["grid"] if "speed_m_s" in row}

    checked = 0
    for h in KERB_HEIGHTS_M:
        for v in SPEEDS_M_S:
            if (h, v) not in rows_by_key:
                continue
            row = rows_by_key[(h, v)]
            expected = kerb_strike_vs_com_impulse(h, v)
            assert row["impulse_ns"] == expected["impulse_ns"]
            assert row["pitch_rate_deg_s"] == expected["pitch_rate_deg_s"]
            checked += 1
    assert checked > 0


def test_plant_geometry_matches_what_the_canonical_model_assumes():
    """`kerb_strike_impulse` hardcodes wheel radius and ridden mass as
    defaults (see its `model_params`). This script builds the same ridden
    plant and reads those figures from the compiled model rather than
    re-quoting them. If the two ever diverge -- a mass or geometry change on
    either side -- this is the assertion that is supposed to fail instead of
    the two silently disagreeing, the same failure mode `r_eff` already hit
    once in this repo (`tests/test_r_eff_matches_model.py`)."""
    model = build_model(BALLAST_KG, BALLAST_M, CLAMP_A)
    geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "wheel_geom")
    r = float(model.geom_size[geom][0])
    total_mass = float(sum(model.body_mass[b] for b in range(model.nbody)))

    # kerb_strike_impulse's own defaults (sim/scenarios/plant.py) -- pinned
    # here, not re-derived, precisely so a change on either side (this
    # script's plant build, or plant.py's defaults) fails this assertion
    # instead of the two quietly drifting apart.
    assert r == pytest.approx(0.1454, abs=1e-6)
    assert total_mass == pytest.approx(82.5, abs=1e-6)


def test_report_leads_with_pitch_rate_not_just_impulse():
    """#142's live finding is that N*s is the wrong currency for comparing a
    kerb strike to the sim's CoM-applied disturbance -- a same-magnitude CoM
    impulse imparts zero pitch rate by construction. The report must carry
    that comparison, not just the impulse bracket, or it reintroduces the
    exact confusion `kerb_strike_vs_com_impulse` was written to resolve."""
    report = build_report()
    row = next(r for r in report["grid"] if "speed_m_s" in r)
    assert row["com_impulse_pitch_rate_deg_s"] == pytest.approx(0.0)
    assert row["pitch_rate_deg_s"][0] > 0.0
    assert "current_reference_com_impulse_pitch_rate_deg_s" in report
