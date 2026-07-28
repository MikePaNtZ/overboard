"""Acceptance gate for the bench-rig identification scenarios.

Wave-0 job: prove the identification METHOD (two-run kt/J fit, spin-down
friction split, hardware-residual replay) on a rig small enough to sit on a
desk, against a model whose joint-space inertias are known from geometry (see
bench_rig.xml's header).

MOST OF THIS RUNS ON `IDEAL`, AND THAT USED TO BE THE WHOLE STORY. It is not
enough, and the gap cost a factor of two.

This file previously said that `STAGE0_PLACEHOLDER` "is exercised in
`sim.scenarios.imperfections`'s own suite" and left it there. True, and
irrelevant: that suite checks the imperfections, not what the identification
does *under* them. So a `kt` fit that was 50% low on the default profile sat
green, because the only kt assertion was on the ideal run and its name said so.

The fix is a method-versus-signal-path split that actually holds:

  * `IDEAL` tests pin the ALGEBRA. If they fail, the two-run derivation or the
    geometry is wrong.
  * `STAGE0_PLACEHOLDER` tests pin the WINDOWING. If they fail, the fit is
    reading the current-loop transient instead of the ramp -- which is exactly
    the failure that shipped.

Any new fitted quantity needs both. A number that is only ever asserted on a
noise-free profile is a number nobody has checked.
"""

import csv

import mujoco
import numpy as np
import pytest

from sim.scenarios.bench_spinup import (
    MODEL_PATH,
    NAMEPLATE_KT_NM_PER_A,
    IdentifyParams,
    SpindownParams,
    build_bare_model,
    decay_settle_time_s,
    known_disc_inertia_kg_m2,
    load_model,
    replay,
    spindown,
    _FLYWHEEL_GEOM_RE,
    _joint_inertia,
    identify,
)
from sim.scenarios.imperfections import IDEAL, STAGE0_PLACEHOLDER


def load_model_xml() -> str:
    """The raw MJCF text, for tests that build variants by stripping geoms."""
    return MODEL_PATH.read_text()

#: The committed model's joint-space inertias, from bench_rig.xml's header --
#: geometry-derived, not tuned. A change to the flywheel or rotor geometry
#: should trip these and force the header's numbers to be updated alongside.
KNOWN_GOOD_J_BARE = 2.4065e-4
KNOWN_GOOD_J_LOADED = 1.8510e-3
KNOWN_GOOD_J_DISC = 1.6103e-3


@pytest.fixture(scope="module")
def loaded_model():
    return load_model()


@pytest.fixture(scope="module")
def bare_model():
    return build_bare_model()


# --------------------------------------------------------------------------
# The model itself
# --------------------------------------------------------------------------

def test_model_compiles_and_matches_known_inertias(loaded_model, bare_model):
    j_bare = _joint_inertia(bare_model)
    j_loaded = _joint_inertia(loaded_model)
    assert j_bare == pytest.approx(KNOWN_GOOD_J_BARE, abs=1e-6)
    assert j_loaded == pytest.approx(KNOWN_GOOD_J_LOADED, abs=1e-6)
    assert (j_loaded - j_bare) == pytest.approx(KNOWN_GOOD_J_DISC, abs=1e-6)
    assert known_disc_inertia_kg_m2() == pytest.approx(KNOWN_GOOD_J_DISC, abs=1e-6)


def test_bare_variant_actually_strips_the_flywheel_geom(loaded_model, bare_model):
    """Verified, not trusted -- a regex that silently failed to match would
    make the bare and loaded runs identical and the two-run fit a
    divide-by-zero."""
    assert mujoco.mj_name2id(bare_model, mujoco.mjtObj.mjOBJ_GEOM, "flywheel") < 0
    assert mujoco.mj_name2id(loaded_model, mujoco.mjtObj.mjOBJ_GEOM, "flywheel") >= 0
    assert _joint_inertia(bare_model) < _joint_inertia(loaded_model)


# --------------------------------------------------------------------------
# Mode 1 -- identify
# --------------------------------------------------------------------------

def test_identify_recovers_nameplate_kt_and_bare_inertia_on_ideal_run(loaded_model):
    result = identify(IdentifyParams(), loaded_model=loaded_model, profile=IDEAL)
    m = result.metrics

    assert m.kt_fit_nm_per_a == pytest.approx(NAMEPLATE_KT_NM_PER_A, rel=0.01)
    assert m.j_bare_fit_kg_m2 == pytest.approx(KNOWN_GOOD_J_BARE, rel=0.02)
    # The fit window must actually be a clean line, or the tolerances above
    # are meaningless -- a bad window would still average to *something*.
    assert m.r2_bare > 0.999
    assert m.r2_loaded > 0.999


def test_identify_recovers_kt_under_the_stage0_profile(loaded_model):
    """**The assertion whose absence let a factor-of-two error ship green.**

    kt used to come out at 0.02493 against a 0.05026 nameplate -- 50.4% low --
    on the DEFAULT profile, while every test here passed because every kt
    assertion was on the ideal run.

    The tolerance is 1%, the same as the ideal run, because after the fix there
    is no reason for the realistic profile to be worse: the only bias left is
    Coulomb (`tau_c/i`, ~0.5%), which both profiles share. A regression to the
    old windowing fails this by a factor of fifty, not by a hair.
    """
    m = identify(IdentifyParams(), loaded_model=loaded_model,
                 profile=STAGE0_PLACEHOLDER).metrics
    assert m.kt_fit_nm_per_a == pytest.approx(NAMEPLATE_KT_NM_PER_A, rel=0.01), (
        f"kt {m.kt_fit_nm_per_a:.5f} vs nameplate {NAMEPLATE_KT_NM_PER_A:.5f} "
        f"({m.kt_error_vs_nameplate_pct:.1f}% off) -- if this is ~50% low the fit "
        "window is back inside the actuation delay and current-loop lag"
    )
    # J_bare was ALWAYS right, even with the bug, because the suppression factor
    # cancels between numerator and denominator. Asserting it here documents that
    # it is not what regressed and not what proves the fix.
    assert m.j_bare_fit_kg_m2 == pytest.approx(KNOWN_GOOD_J_BARE, rel=0.02)


def test_the_r2_diagnostic_is_asserted_and_not_merely_reported(loaded_model):
    """R^2 fell 1.0000 -> 0.6927 while the bug was live, and nothing checked it.

    `_fit_line`'s docstring claims a bad window is "visible in the R^2 rather
    than silently baked into a slope". It was visible. It was just not looked at.

    Floored below the ideal run's 0.999 on purpose: the 500 Hz sensed-rate
    staircase is real and caps R^2 near 0.98. The point is that 0.69 -- a
    curve, not a line -- can no longer pass.
    """
    m = identify(IdentifyParams(), loaded_model=loaded_model,
                 profile=STAGE0_PLACEHOLDER).metrics
    assert m.r2_bare > 0.95, f"bare ramp is not a line: R^2={m.r2_bare:.4f}"
    assert m.r2_loaded > 0.95, f"loaded ramp is not a line: R^2={m.r2_loaded:.4f}"


def test_the_fit_window_opens_after_the_current_has_settled(loaded_model):
    """The mechanism, pinned so it cannot be "simplified" back to fitting from zero.

    Under Stage-0 there is 1 ms of actuation delay and a 1 ms current-loop time
    constant, so a settled window cannot start at t=0 -- and on IDEAL there is
    nothing to wait for, so it must start immediately. Both directions are
    asserted; checking only one would pass on a hard-coded offset.
    """
    stage0 = identify(IdentifyParams(), loaded_model=loaded_model,
                      profile=STAGE0_PLACEHOLDER).metrics
    ideal = identify(IdentifyParams(), loaded_model=loaded_model, profile=IDEAL).metrics

    assert stage0.fit_start_s > 0.005, (
        f"window opened at {stage0.fit_start_s*1e3:.1f} ms -- too early to have "
        "cleared a 1 ms delay plus ~5 time constants of a 1 ms current loop"
    )
    assert ideal.fit_start_s < 0.002, (
        f"window opened at {ideal.fit_start_s*1e3:.1f} ms on a profile with no "
        "delay and no lag -- the settle detector is not data-driven"
    )
    # The span is what was asked for, in both cases.
    for m in (stage0, ideal):
        assert m.fit_end_s - m.fit_start_s == pytest.approx(IdentifyParams().fit_span_s)

    # And the current the fit believes is the one that flowed, not the command.
    assert stage0.i_measured_mean_a == pytest.approx(stage0.commanded_current_a, rel=0.02), (
        "after settling, measured and commanded current should agree closely; a "
        "large gap means the window still straddles the transient"
    )


def test_fitting_from_zero_reproduces_the_original_50_percent_error(loaded_model):
    """The counterfactual, so the reason for the window is measured and not just
    asserted in a docstring.

    Deliberately reconstructs the OLD behaviour -- fit from t=0 over 5 ms
    against the COMMANDED current -- and shows it really does halve kt. If this
    ever stops being true, the delay/lag model changed and the windowing
    rationale needs revisiting rather than being trusted.
    """
    from sim.scenarios.bench_spinup import _fit_line, _run_constant_current

    p = IdentifyParams()
    bare = build_bare_model()
    t_b, w_b, _ = _run_constant_current(bare, p.commanded_current_a, p.sim_seconds,
                                        STAGE0_PLACEHOLDER)
    t_l, w_l, _ = _run_constant_current(loaded_model, p.commanded_current_a,
                                        p.sim_seconds, STAGE0_PLACEHOLDER)
    a1, r2_bare = _fit_line(t_b, w_b, 0.005)          # from zero, as it used to be
    a2, _ = _fit_line(t_l, w_l, 0.005)
    kt_old = known_disc_inertia_kg_m2() / (p.commanded_current_a * (1.0 / a2 - 1.0 / a1))

    assert kt_old < 0.6 * NAMEPLATE_KT_NM_PER_A, (
        f"the old from-zero window gave kt={kt_old:.5f}, not the ~50%-low result "
        "it is documented to give -- the delay/lag model has changed"
    )
    assert r2_bare < 0.8, (
        f"R^2={r2_bare:.4f} for the from-zero fit; the transient is supposed to be "
        "visibly curved, which is what makes R^2 a usable diagnostic"
    )


# --------------------------------------------------------------------------
# The bare variant: what the strip removes
# --------------------------------------------------------------------------

def test_the_bare_model_carries_no_floating_index_mark(loaded_model):
    """The index mark is drawn on the flywheel face, so it goes with the disc.

    Left behind, it renders as a white bar rotating in mid-air 52 mm off the
    shaft, where the disc used to be. Raised by Digital Content Production after
    it reached a render.
    """
    bare = build_bare_model()
    assert mujoco.mj_name2id(bare, mujoco.mjtObj.mjOBJ_GEOM, "index") < 0
    assert mujoco.mj_name2id(bare, mujoco.mjtObj.mjOBJ_GEOM, "flywheel") < 0
    # Still present on the loaded rig -- the strip must be the only thing that
    # removes it, or the mark has simply been deleted from the model.
    assert mujoco.mj_name2id(loaded_model, mujoco.mjtObj.mjOBJ_GEOM, "index") >= 0


def test_removing_the_index_mark_changes_no_fitted_quantity(loaded_model):
    """It is massless with collision off, so this is model coherence, not
    correctness -- and that claim is worth checking rather than repeating.

    Compares joint inertia of a bare model against one built by stripping only
    the flywheel. Identical means the mark never contributed, so no previously
    published J_bare or kt moves because of this change.
    """
    xml = load_model_xml()
    flywheel_only = mujoco.MjModel.from_xml_string(_FLYWHEEL_GEOM_RE.sub("", xml))
    assert _joint_inertia(build_bare_model()) == pytest.approx(
        _joint_inertia(flywheel_only), rel=1e-12
    )


# --------------------------------------------------------------------------
# Mode 2 -- spindown
# --------------------------------------------------------------------------

def test_spindown_recovers_friction_terms_on_ideal_run(loaded_model):
    result = spindown(SpindownParams(), model=loaded_model, profile=IDEAL)
    m = result.metrics

    assert m.b_placeholder_nm_s_per_rad == pytest.approx(1e-4)
    assert m.tau_c_placeholder_nm == pytest.approx(5e-3)
    assert m.b_fit_nm_s_per_rad == pytest.approx(m.b_placeholder_nm_s_per_rad, rel=0.01)
    assert m.tau_c_fit_nm == pytest.approx(m.tau_c_placeholder_nm, rel=0.01)
    assert m.r2 > 0.999


def test_spindown_lumped_fit_residual_changes_sign(loaded_model):
    """The defect the two-term fit exists to avoid: a viscous-only fit
    absorbs part of Coulomb into a biased b, and the leftover residual is
    NOT the same sign at both ends of the speed range."""
    result = spindown(SpindownParams(), model=loaded_model, profile=IDEAL)
    m = result.metrics
    assert m.lumped_residual_changes_sign, (
        f"lo={m.lumped_residual_low_speed_rad_s2}, hi={m.lumped_residual_high_speed_rad_s2}"
    )


def test_spindown_recovers_friction_terms_under_the_stage0_profile(loaded_model):
    """**The assertion whose absence let R^2 collapse to ~0.002 ship green** (#68).

    Before the fix, `spindown()` had no settle window: the least-squares fit
    ran from the instant current was commanded to zero, straight through the
    actuation-delay + current-loop-lag transient while real current was still
    decaying off the shaft. Under the default profile that produced an R^2 of
    ~0.002 -- noise, not a curve -- yet nothing asserted it, so the only
    spindown test in this suite (on `IDEAL`, where there is no transient to
    hide) stayed green regardless.

    Tolerances match the ideal run's (1%) because after the fix there is no
    reason STAGE0 should be worse: both profiles now fit over a settled
    window, and the only thing STAGE0 adds is a longer wait for the transient
    to clear, not a bias in the result.
    """
    m = spindown(SpindownParams(), model=loaded_model, profile=STAGE0_PLACEHOLDER).metrics
    assert m.r2 > 0.999, (
        f"R^2={m.r2:.4f} -- if this is near zero the fit window is back inside "
        "the current-loop's unwind transient"
    )
    assert m.b_fit_nm_s_per_rad == pytest.approx(m.b_placeholder_nm_s_per_rad, rel=0.01)
    assert m.tau_c_fit_nm == pytest.approx(m.tau_c_placeholder_nm, rel=0.01)


def test_the_spindown_fit_window_opens_after_the_current_has_decayed(loaded_model):
    """The mechanism, pinned so it cannot regress back to fitting from t=0.

    Under Stage-0 there is 1 ms of actuation delay plus a 1 ms current-loop
    time constant on the way DOWN to zero, just as there is on the way up in
    `identify()`, so a settled window cannot start at t=0. On IDEAL there is
    nothing to wait for, so it must start immediately. Both directions are
    asserted, exactly mirroring
    `test_the_fit_window_opens_after_the_current_has_settled` for `identify()`.
    """
    stage0 = spindown(SpindownParams(), model=loaded_model, profile=STAGE0_PLACEHOLDER).metrics
    ideal = spindown(SpindownParams(), model=loaded_model, profile=IDEAL).metrics

    assert stage0.fit_start_s > 0.005, (
        f"window opened at {stage0.fit_start_s*1e3:.1f} ms -- too early to have "
        "cleared the current-loop's unwind transient"
    )
    assert ideal.fit_start_s < 0.002, (
        f"window opened at {ideal.fit_start_s*1e3:.1f} ms on a profile with no "
        "delay and no lag -- the settle detector is not data-driven"
    )


def test_spindown_fitting_from_zero_reproduces_the_r2_collapse(loaded_model):
    """The counterfactual, so the reason for the window is measured and not
    just asserted in a docstring -- mirrors
    `test_fitting_from_zero_reproduces_the_original_50_percent_error` for
    `identify()`.

    `settle_fraction=0.0` reconstructs the pre-fix behaviour (window opens at
    the first sample, i.e. fits straight through the transient) without
    duplicating `spindown()`'s internals. If this stops reproducing a
    near-zero R^2, the current-loop model has changed and the windowing
    rationale needs revisiting rather than being trusted.
    """
    m = spindown(
        SpindownParams(settle_fraction=0.0), model=loaded_model, profile=STAGE0_PLACEHOLDER,
    ).metrics
    assert m.r2 < 0.1, (
        f"R^2={m.r2:.4f} fitting from t=0 -- expected a collapse close to the "
        "~0.002 reported in #68; the current-loop model may have changed"
    )
    assert m.b_error_pct > 20.0 and m.tau_c_error_pct > 20.0, (
        "expected the from-zero fit to badly mis-recover both friction terms, "
        f"got b_error={m.b_error_pct:.2f}%, tau_c_error={m.tau_c_error_pct:.2f}%"
    )


# --------------------------------------------------------------------------
# decay_settle_time_s -- unit-level, no sim
# --------------------------------------------------------------------------

def test_decay_settle_time_s_waits_for_the_last_violation():
    """"Stays within", not "first reaches": a current that dips under the
    threshold and bounces back out must not open the window early."""
    t = np.array([0.0, 1.0, 2.0, 3.0, 4.0, 5.0])
    i_measured = np.array([20.0, 0.05, 5.0, 0.05, 0.02, 0.01])  # dip at t=1, then back up at t=2
    t_start = decay_settle_time_s(t, i_measured, initial_current_a=20.0, fraction=0.99)
    assert t_start == pytest.approx(3.0), (
        "must skip past the t=2 bounce-back, not stop at the first dip under threshold"
    )


def test_decay_settle_time_s_opens_immediately_when_already_settled():
    t = np.array([0.0, 1.0, 2.0])
    i_measured = np.array([0.0, 0.0, 0.0])
    assert decay_settle_time_s(t, i_measured, initial_current_a=20.0, fraction=0.99) == 0.0


def test_decay_settle_time_s_raises_if_current_never_decays():
    t = np.array([0.0, 1.0, 2.0])
    i_measured = np.array([20.0, 19.0, 18.0])
    with pytest.raises(ValueError, match="never decayed"):
        decay_settle_time_s(t, i_measured, initial_current_a=20.0, fraction=0.99)


# --------------------------------------------------------------------------
# Mode 3 -- replay
# --------------------------------------------------------------------------

def _write_csv(path, t, cmd, reported, w):
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t_s", "commanded_current_a", "reported_current_a", "shaft_rate_rad_s"])
        writer.writerows(zip(t, cmd, reported, w))


def _synthetic_record(model, current_a=15.0, seconds=0.5, current_scale=1.0):
    """Drive `model` open-loop at a held current and log truth -- the
    self-consistency baseline a replay against the SAME model should
    reproduce almost exactly."""
    from sim.scenarios.bench_spinup import NAMEPLATE_KT_NM_PER_A as KT

    data = mujoco.MjData(model)
    dt = float(model.opt.timestep)
    mujoco.mj_forward(model, data)
    n = int(round(seconds / dt))
    ts, cmds, reported, ws = [], [], [], []
    for _ in range(n):
        data.ctrl[0] = current_a * KT
        mujoco.mj_step(model, data)
        ts.append(float(data.time))
        cmds.append(current_a)
        reported.append(current_a * current_scale)
        ws.append(float(data.qvel[0]))
    return np.asarray(ts), np.asarray(cmds), np.asarray(reported), np.asarray(ws)


def test_replay_self_consistency_is_near_zero(tmp_path, loaded_model):
    t, cmd, reported, w = _synthetic_record(loaded_model)
    p = tmp_path / "self_consistent.csv"
    _write_csv(p, t, cmd, reported, w)

    m = replay(p, profile=IDEAL, model=loaded_model)
    assert m.rms_residual_rad_s < 1e-6
    assert m.max_residual_rad_s < 1e-6
    assert m.current_tracking_ok


def test_replay_perturbed_inertia_shows_up_in_the_early_window(tmp_path, loaded_model):
    """An inertia error shows up EARLY (the whole trajectory has been
    accelerating wrong from the start); this pins that against the
    near-zero self-consistent baseline above."""
    from sim.scenarios.bench_spinup import MODEL_PATH

    xml_text = MODEL_PATH.read_text()
    perturbed_xml = xml_text.replace('density="4300"', 'density="6300"')
    assert perturbed_xml != xml_text, "the rotor_can density string did not match"
    perturbed_model = mujoco.MjModel.from_xml_string(perturbed_xml)
    assert _joint_inertia(perturbed_model) != pytest.approx(_joint_inertia(loaded_model))

    t, cmd, reported, w = _synthetic_record(perturbed_model)
    p = tmp_path / "perturbed.csv"
    _write_csv(p, t, cmd, reported, w)

    # Replay against the NOMINAL model -- a mismatch, on purpose.
    m = replay(p, profile=IDEAL, model=loaded_model)
    assert m.rms_residual_rad_s > 0.05
    assert m.rms_residual_early_rad_s > 0.05
    assert m.current_tracking_ok, "current itself was not perturbed here"


def test_replay_flags_current_tracking_failure(tmp_path, loaded_model):
    t, cmd, reported, w = _synthetic_record(loaded_model, current_scale=0.5)
    p = tmp_path / "derated.csv"
    _write_csv(p, t, cmd, reported, w)

    m = replay(p, profile=IDEAL, model=loaded_model)
    assert not m.current_tracking_ok
    assert m.current_max_divergence_a > 0.5


def test_replay_missing_csv_raises_a_clear_error(tmp_path):
    missing = tmp_path / "does_not_exist.csv"
    with pytest.raises(FileNotFoundError, match="shaft_rate_rad_s"):
        replay(missing, profile=IDEAL)
