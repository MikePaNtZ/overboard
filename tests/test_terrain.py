"""Rolling terrain: the profile contract, the build guards, and the gate.

The headline this file exists to pin: **a rolling profile is harder than a
steady grade of the same steepness.** The uniform-grade gate passes the
estimator on a steady +10% descent; the same peak grade with a crest, a dip and
two transitions puts the board down.

GATE NUMBERS -- MEASURED VS ASSUMED (issue #24 AC5 audit, this session)
-------------------------------------------------------------------------
| Assertion                                              | Threshold                     | Observed this session | Status |
|---------------------------------------------------------|--------------------------------|------------------------|--------|
| default ride, 8% peak grade: `survived`/`reached_next_crest`/`held_speed` | must all be true | survived, reached the crest, held speed; dip reached before the crest (t_dip < t_crest) | MEASURED |
| the headline: steady 10% descent vs rolling 10% peak     | steady survives, rolling does not | steady: survived; rolling: struck (`struck_phase` one of descent/dip/ascent) | MEASURED -- this is the actual comparison the scenario exists to make, re-run rather than assumed |
| estimator costs the envelope, 10% peak, truth vs estimate | truth completes, estimate does not | truth: survived + reached crest; estimate: did not survive | MEASURED |
| estimator error lowest through the dip, 4%/8% peak       | `est_rms_dip < est_rms_descent` | reached the crest at both grades; dip RMS below descent RMS at both | MEASURED. The module docstring's own aside -- total RMS is roughly grade-independent, "~0.96 deg across 2-8%" -- was itself stated as measured when written and was not re-derived here |
| cutback never binds on the default 8%/24 m ride           | `cutback_binding_cycles == 0` | 0 | MEASURED |

No gaps found in this file: every GATE assertion already ties to a stated
comparison (steady vs rolling, truth vs estimate) rather than a bare
round-number threshold, so there was nothing here in the shape of `hill.py`'s
25 deg sanity ceiling to flag.

RE-MEASURED AFTER ISSUE #250'S ESTIMATOR FIX (this session)
-------------------------------------------------------------------------
Issue #250 fixed a real defect: a near-zero specific-force reading (any brief
unloading -- cresting a rise, a kerb, a dip) resolved through `atan2(0, -0)`
to a confident, ARBITRARY angle (most often 180 deg) that the old estimator
trusted immediately. `ComplementaryFilter` now refuses to fuse a
specific-force vector below `MIN_TRUSTED_ACCEL_MAG_M_S2` and coasts on the
gyro instead. That gate engages during the genuine near-unloading moments a
rolling profile's dip and transitions produce -- which this file's own
comparisons run straight through -- so three of the four GATE rows above no
longer hold at the grades they were pinned at. Re-measured rather than
patched to keep the old numbers:

- **Headline (steady vs rolling).** At 10% peak, both now survive -- the
  specific defect this scenario used to trip on is gone. Sweeping grade
  finds no window at all where steady survives and rolling does not: steady's
  own ceiling (unrelated to this fix -- reproduces identically on unfixed
  `master`) is 10.0%, while rolling's ceiling, now higher than before, is
  10.5%. There IS a repeatable, five-tenths-of-a-percent-wide window
  (10.1-10.5%) where the relationship is the exact **opposite** of the
  original headline: steady fails and rolling survives. That is what is
  pinned now (`test_the_rolling_vs_steady_headline_inverted_after_250`). Note
  the qualifier: this shows the specific bug was inflating the size (and
  apparently the sign) of the steady/rolling gap at the 10% test point, NOT
  that a rolling profile is now categorically easier in general, and NOT that
  the *original* "transitions cost margin" hypothesis is wrong -- neither has
  been re-examined here. A real, deliberate re-characterisation of whether
  and where a rolling profile is intrinsically harder than a steady grade,
  independent of this bug, is follow-up work, not done in this pass.
- **Estimator costs the envelope.** Re-derived at grade 11.0% (was 10.0%),
  the same shape of comparison as before (truth survives, estimate does not),
  in a five-sample-wide (10.6-11.4%) measured window -- 11.6% already flips
  back, the same non-monotonic-near-a-boundary signature issue #24 AC2's
  disturbance sweep already documented for this codebase, so the grade was
  chosen from the middle of the window, not its edge.
- **Estimator error through the dip.** This one did not just shift -- it
  inverted categorically. Swept 2-9% peak grade with the fix active: dip RMS
  is now HIGHER than descent RMS at every grade tested, not lower at any of
  them. The dip is also where the profile's curvature is most active (both
  transitions bracket it), so it is a plausible place for the new gate to
  engage more, but that is a hypothesis, not a measurement -- **not
  root-caused here**, matching this file's own precedent from issue #228 for
  not chasing a mechanism outside a pass's actual scope. Re-pinned to the
  newly measured (inverted) direction in
  `test_estimator_error_is_highest_through_the_dip`.

None of this touches ADR-0011's own text or its (f1)/(f2) trim pin -- flagged
for the COO in the PR this session opens, since it bears directly on issue
#227 (same file, same estimator, already an open question about whether the
freeze-and-pin generalises past flat ground) and, through it, on ADR-0011's
launch-hold status. Deciding whether any of that changes is not this session's
call.
"""

import math

import numpy as np
import pytest

from sim.scenarios.hill import HillParams
from sim.scenarios.hill import run as hill_run
from sim.scenarios.imperfections import IDEAL, STAGE0_CUTBACK
from sim.scenarios.terrain import (
    TerrainParams,
    amplitude_for_grade,
    build_terrain_model,
    run,
    summary_line,
    surface_grade_pct,
    surface_z,
)


# --------------------------------------------------------------------------
# PROFILE CONTRACT — arithmetic, no plant
# --------------------------------------------------------------------------

def test_the_profile_is_crest_dip_crest():
    """Exactly one period: starts high, bottoms out halfway, ends high."""
    L = 24.0
    A = amplitude_for_grade(10.0, L)
    assert surface_z(0.0, A, L) == pytest.approx(A)          # crest
    assert surface_z(-L / 2, A, L) == pytest.approx(-A)      # dip
    assert surface_z(-L, A, L) == pytest.approx(A)           # next crest


def test_the_steepest_point_is_the_requested_grade():
    """Parameterising by peak grade rather than amplitude is what lets this
    scenario be read against the uniform-grade envelope without arithmetic."""
    L = 24.0
    for want in (4.0, 8.0, 15.0):
        A = amplitude_for_grade(want, L)
        got = max(abs(surface_grade_pct(-s, A, L)) for s in np.linspace(0, L, 2001))
        assert got == pytest.approx(want, abs=1e-3)


def test_grade_sign_matches_the_uniform_hill_convention():
    """Positive = downhill-forward, as `hill.gravity_for_grade` signs it. If
    these disagreed, every cross-comparison in this file would be backwards and
    would look like a physics result rather than a sign error."""
    L = 24.0
    A = amplitude_for_grade(10.0, L)
    assert surface_grade_pct(-L / 4, A, L) > 0.0, "quarter point should be descending"
    assert surface_grade_pct(-3 * L / 4, A, L) < 0.0, "three-quarter point should climb"
    assert surface_grade_pct(0.0, A, L) == pytest.approx(0.0, abs=1e-9)
    assert surface_grade_pct(-L / 2, A, L) == pytest.approx(0.0, abs=1e-9)


def test_the_grade_reverses_through_the_dip():
    """The case a constant-grade run cannot produce: descending becomes
    climbing, continuously, through zero."""
    L, A = 24.0, amplitude_for_grade(10.0, 24.0)
    before = surface_grade_pct(-L / 2 + 1.0, A, L)
    after = surface_grade_pct(-L / 2 - 1.0, A, L)
    assert before > 0.0 and after < 0.0, "grade did not change sign across the dip"


# --------------------------------------------------------------------------
# BUILD GUARDS
# --------------------------------------------------------------------------

def test_the_heightfield_is_actually_terrain():
    """A silent no-op in the XML transform yields a model that reads as terrain
    and behaves as a flat plane -- every result in this module would then be a
    flat-ground result wearing a hill's name."""
    model, amp = build_terrain_model(TerrainParams(max_grade_pct=10.0))
    adr, ncol = int(model.hfield_adr[0]), int(model.hfield_ncol[0])
    span = float(np.ptp(np.asarray(model.hfield_data[adr:adr + ncol])))
    assert span > 0.9, f"heightfield normalised span {span:.4f} -- near flat"
    assert amp == pytest.approx(amplitude_for_grade(10.0, 24.0))


def test_the_board_starts_standing_on_the_crest():
    """Not buried in the ground and not dropped from a height -- either would
    make the first second of every run a contact transient."""
    import mujoco

    model, amp = build_terrain_model(TerrainParams())
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    frame = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "frame")
    axle_z = float(data.xpos[frame][2])
    assert axle_z == pytest.approx(amp + 0.1454, abs=1e-3)


def test_run_refuses_an_ideal_profile():
    with pytest.raises(ValueError, match="ideal"):
        run(TerrainParams(duration_s=4.0), profile=IDEAL)


def test_run_refuses_the_driverless_plant():
    with pytest.raises(ValueError, match="RIDDEN"):
        run(TerrainParams(ballast_mass_kg=0.0, duration_s=4.0))


# --------------------------------------------------------------------------
# GATE
# --------------------------------------------------------------------------

def test_the_board_rides_crest_to_crest():
    """**The gate.** Start on a crest, descend, cross the dip, climb to the next
    crest, arrive. On the attitude estimate, at the default 8% peak grade.
    """
    m = run(TerrainParams()).metrics
    assert m.survived, f"struck the {m.struck_end} in the {m.struck_phase} at {m.t_strike_s}s"
    assert m.reached_next_crest, (
        f"only got {m.fraction_completed * 100:.0f}% of the way -- surviving "
        "without completing is not doing the manoeuvre"
    )
    assert m.held_speed
    assert m.t_reached_dip_s is not None and m.t_reached_crest_s is not None
    assert m.t_reached_dip_s < m.t_reached_crest_s, "reached the crest before the dip"


def test_completing_is_asserted_separately_from_surviving():
    """A board that stalls halfway is upright and useless. `survived` alone
    would pass it, which is the same shape of hole as scoring a crashed run as
    holding speed."""
    m = run(TerrainParams(duration_s=6.0)).metrics
    assert m.survived, "6 s should not be enough to crash, only enough to be short"
    assert not m.reached_next_crest
    assert m.fraction_completed < 1.0


def test_the_rolling_vs_steady_headline_inverted_after_250():
    """**The headline, re-measured (issue #250).**

    Was `test_a_rolling_profile_is_harder_than_a_steady_grade_of_the_same_
    steepness`, asserting steady-10%-survives / rolling-10%-does-not. Issue
    #250 fixed a real defect (a near-zero specific-force reading resolving
    through `atan2(0, -0)` to a confident, arbitrary angle) that this
    scenario's dip and transitions were tripping on. Re-swept grade after the
    fix: there is no longer any grade where steady survives and rolling does
    not -- steady's own ceiling (10.0%, unrelated to this fix, reproduces
    identically on unfixed code) is now BELOW rolling's (10.5%). What is
    repeatably measured is the opposite relationship, in a real, five-tenths-
    wide window, pinned here rather than a knife-edge single grade.

    This does not establish that a rolling profile is now categorically
    easier in general, only that it is at this specific window -- a proper
    re-characterisation of the original "transitions cost margin" hypothesis
    is follow-up work, not done here. See this file's module docstring for
    the full accounting, and issue #227 for why this also bears on ADR-0011.
    """
    steady = hill_run(HillParams(grade_pct=10.3, v_ref_m_s=2.0, duration_s=10.0)).metrics
    rolling = run(TerrainParams(max_grade_pct=10.3)).metrics

    assert not steady.survived, (
        "steady 10.3% now survives -- re-sweep for the new crossover rather "
        "than editing this grade blind"
    )
    assert rolling.survived, (
        "rolling 10.3% no longer survives -- re-sweep for the new crossover "
        "rather than editing this grade blind"
    )


def test_the_estimator_costs_terrain_envelope():
    """Truth pitch rides a profile the estimate cannot. Same plant, same
    terrain, same commanded speed -- only the source of attitude differs.

    Re-derived at 11.0% peak grade (was 10.0%) after issue #250's estimator
    fix moved this envelope -- see this file's module docstring. Chosen from
    the middle of a measured 10.6-11.4% window, not its edge: 11.6% already
    flips back, the same non-monotonic-near-a-boundary signature issue #24
    AC2's disturbance sweep already documented elsewhere in this codebase.
    """
    truth = run(TerrainParams(max_grade_pct=11.0, use_estimator=False)).metrics
    est = run(TerrainParams(max_grade_pct=11.0, use_estimator=True)).metrics
    assert truth.survived and truth.reached_next_crest, (
        "truth pitch should complete an 11% rolling profile"
    )
    assert not est.survived, "the estimate should not"


@pytest.mark.parametrize("grade", [4.0, 8.0])
def test_estimator_error_is_highest_through_the_dip(grade):
    """Re-measured after issue #250's estimator fix (was `..._is_lowest...`,
    asserting the opposite direction).

    This did not just shift, it inverted categorically: swept 2-9% peak grade
    with the fix active, dip RMS is higher than descent RMS at every grade
    tested, not lower at any of them. The dip is also where the profile's
    curvature is most active (both transitions bracket it), so it is a
    plausible place for the fix's new near-zero-specific-force gate to engage
    more -- but that is a hypothesis, not a measurement. **Not root-caused
    here**, matching this file's own precedent from issue #228 for not
    chasing a mechanism outside a pass's actual scope.

    Note this still does NOT reproduce the uniform gate's `error ~= slope
    angle`; the module docstring's "~0.96 deg across 2-8%, roughly
    grade-independent" aside was not re-derived in this pass either.
    """
    m = run(TerrainParams(max_grade_pct=grade)).metrics
    assert m.reached_next_crest
    assert m.est_rms_dip_deg > m.est_rms_descent_deg, (
        f"dip {m.est_rms_dip_deg:.3f}° vs descent {m.est_rms_descent_deg:.3f}° -- "
        "this inverted after issue #250's fix; re-measure rather than editing "
        "this direction blind if it reproduces the OLD relationship again"
    )


def test_the_ride_is_reported_in_one_readable_line():
    """The deliverable is a scenario someone can read the outcome of, not a
    boolean. Used by the analysis scripts and by anyone looking for the edge."""
    text = summary_line(run(TerrainParams()))
    assert "crest" in text and "est RMS" in text and "descent" in text


def test_cutback_does_not_bind_on_the_default_ride():
    """Stated so the presence of a cutback model is never mistaken for evidence
    that cutback was the mechanism. At 8% over 24 m the board never gets fast
    enough to be derated; the estimator is the constraint, as everywhere else."""
    m = run(TerrainParams()).metrics
    assert m.imperfection_profile_id == STAGE0_CUTBACK.profile_id
    assert m.cutback_binding_cycles == 0


# --------------------------------------------------------------------------
# DETERMINISM
# --------------------------------------------------------------------------

def test_repeat_runs_are_bit_identical():
    """Same property `impulse_response` and `closed_loop` already pin. A
    rolling terrain run adds the heightfield contact and the crest/dip/crest
    transitions on top of hill.py's state, so it is checked here rather than
    assumed from either of those. Short duration -- determinism does not
    depend on whether the ride completes, only on repeatability."""
    p = TerrainParams(duration_s=6.0)
    a = run(p)
    b = run(p)
    assert np.array_equal(a.pitch_deg, b.pitch_deg)
    assert np.array_equal(a.v_m_s, b.v_m_s)
    assert np.array_equal(a.travel_m, b.travel_m)
    assert a.to_json_dict() == b.to_json_dict()


def test_capture_state_records_one_pose_per_sample():
    """Mirrors `impulse_response.run(capture_state=...)`: same parameter name,
    same shape, so the renderer treats both scenarios identically. Off by
    default, so nothing that gates on this scenario pays for the pose history,
    and `to_json_dict()` -- checked above -- never carries it into metrics.json.
    """
    p = TerrainParams(duration_s=6.0)
    off = run(p, capture_state=False)
    assert off.qpos.size == 0

    on = run(p, capture_state=True)
    assert on.qpos.shape[0] == len(on.t), (
        "one qpos row per recorded sample -- a mismatch would let the renderer "
        "silently film a trajectory that is not the one the metrics describe"
    )

    # Capturing must not perturb the run: the count check above cannot catch a
    # capture that changes sub-step or has a side effect, only that pinning the
    # trajectory itself against the capture_state=False run can.
    assert np.array_equal(off.t, on.t)
    assert np.array_equal(off.pitch_deg, on.pitch_deg)
    assert np.array_equal(off.travel_m, on.travel_m)
