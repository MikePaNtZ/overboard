"""Acceptance gate for the bench-rig identification scenarios.

Wave-0 job: prove the identification METHOD (two-run kt/J fit, spin-down
friction split, hardware-residual replay) on a rig small enough to sit on a
desk, against a model whose joint-space inertias are known from geometry (see
bench_rig.xml's header). Everything here runs on the noise-free `IDEAL`
profile so the method itself, not the signal path, is what is being checked;
`STAGE0_PLACEHOLDER` is exercised in `sim.scenarios.imperfections`'s own
suite.
"""

import csv

import mujoco
import numpy as np
import pytest

from sim.scenarios.bench_spinup import (
    NAMEPLATE_KT_NM_PER_A,
    IdentifyParams,
    SpindownParams,
    build_bare_model,
    known_disc_inertia_kg_m2,
    load_model,
    replay,
    spindown,
    _joint_inertia,
    identify,
)
from sim.scenarios.imperfections import IDEAL

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
