"""Guards the SECOND drag-model path: welding rider-scale ballast onto the
DRIVERLESS xml, rather than the model file (`overboard_rider.xml`) that
carries the derived constants natively.

`tests/test_rider_drag_model.py` proves the two model FILES carry the right
constants. This file proves the two BUILDER FUNCTIONS that weld ballast onto
`overboard_onewheel.xml` at runtime -- `plant.build_model` (impulse response,
hill, disturbance-envelope scenarios) and `terrain.build_terrain_model`
(rolling-terrain scenario) -- reach the same derivation, since that is the
confound (#207/#215-adjacent PR) that motivated `plant.correct_wheel_drag_
for_load` and `plant.add_rider_aero` in the first place: a welded-ballast run
used to inherit the DRIVERLESS xml's frictionloss literal regardless of how
much mass was bolted on top, and carried no aero term at all.

See `plant.correct_wheel_drag_for_load.__doc__` and `plant.add_rider_aero.
__doc__` for the mechanism; this file MEASURES what the compiled output of
those functions actually does.
"""

from __future__ import annotations

import mujoco
import numpy as np
import pytest

from sim.scenarios import plant, terrain

#: Duplicated from `tests/test_rider_drag_model.py` (itself duplicated from
#: `overboard_onewheel.xml`'s own `wheel_hinge` comment) for the same reason
#: `plant.py`'s own module-level `CRR` is duplicated rather than imported:
#: this constant plays the role of an INDEPENDENT belief about what Crr
#: should be, so a drift in `plant.CRR` away from it is caught here rather
#: than the test importing the very value it exists to check.
CRR = 0.02

#: Bearing + hub-motor iron/windage viscous coefficient -- a drivetrain
#: property, unmodified by either weld-path builder. Same value as
#: `overboard_onewheel.xml`/`overboard_rider.xml`'s own `wheel_hinge`
#: comment and `tests/test_rider_drag_model.py`.
BEARING_DAMPING_NM_S_RAD = 0.009

#: Air density, ISA sea level -- same duplication rationale as
#: `tests/test_rider_drag_model.py`'s own `RHO_AIR`.
RHO_AIR = 1.225

#: Same published onewheel-class reference bands as
#: `tests/test_rider_drag_model.py::REAL_TOTAL_N` -- these are external
#: anchors, not acceptance criteria this repo controls. See that file's
#: module docstring for the full caveat.
REAL_TOTAL_N = {
    1.0: (12.5, 20.8),
    4.0: (18.0, 29.0),
    8.34: (37.7, 58.6),
}

#: `fluidshape="ellipsoid" fluidcoef="0 0 0 0 0"` -- the exact null-override
#: substring `plant._NULL_FLUID_OVERRIDE` supplies. Matched verbatim (not
#: read off the module attribute) so this file's assertion that removing it
#: changes the compiled behaviour is a check on the OVERRIDE ITSELF, not on
#: whatever name happens to hold it today.
_NULL_FLUID_OVERRIDE = 'fluidshape="ellipsoid" fluidcoef="0 0 0 0 0"'


def _dof(model: mujoco.MjModel, joint_name: str) -> int:
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    assert jid >= 0, f"model has no joint named {joint_name!r}"
    return int(model.jnt_dofadr[jid])


def _isolated_qfrc_passive(model: mujoco.MjModel, dof: int, qvel_value: float) -> float:
    data = mujoco.MjData(model)
    data.qvel[:] = 0.0
    data.qvel[dof] = qvel_value
    mujoco.mj_forward(model, data)
    return float(data.qfrc_passive[dof])


def _frame_only_force_n(model: mujoco.MjModel, speed_m_s: float) -> float:
    """Same isolation trick as `test_rider_drag_model.py::
    _frame_only_aero_force_n`: drive only the frame's free-joint x-dof
    (forward = -x) and read `qfrc_passive` on that dof. `wheel_hinge` is a
    different dof, so it cannot leak in."""
    dof = _dof(model, "frame_free")
    data = mujoco.MjData(model)
    data.qvel[:] = 0.0
    data.qvel[dof] = -speed_m_s
    mujoco.mj_forward(model, data)
    return float(data.qfrc_passive[dof])


# ---------------------------------------------------------------------------
# Item 1 -- the derivation itself, recomputed from the COMPILED model.
# ---------------------------------------------------------------------------


def _expected_frictionloss(model: mujoco.MjModel) -> float:
    """`Crr * (compiled subtree mass of 'frame') * g * (compiled wheel
    radius)` -- read entirely off the model `correct_wheel_drag_for_load`
    just produced, never against a second, independently-typed number. This
    is the same relationship `plant.correct_wheel_drag_for_load` computes;
    reproducing it here (with THIS file's own independent `CRR`) is what
    makes a drift in mass, radius, or Crr on either side visible."""
    frame_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "frame")
    total_mass_kg = float(model.body_subtreemass[frame_id])
    wheel_gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "wheel_geom")
    r = float(model.geom_size[wheel_gid][0])
    g = float(np.linalg.norm(model.opt.gravity))
    return CRR * total_mass_kg * g * r


@pytest.mark.parametrize("ballast_mass_kg", [0.0, 70.0])
def test_build_model_frictionloss_matches_the_compiled_load(ballast_mass_kg):
    """`plant.build_model` re-derives `frictionloss` from what IT actually
    weighs, driverless or ridden -- not merely the driverless case, which is
    the confound this whole path exists to close (see this file's module
    docstring)."""
    model = plant.build_model(ballast_mass_kg, 0.75, 40.0)
    dof = _dof(model, "wheel_hinge")
    expected = _expected_frictionloss(model)
    assert model.dof_frictionloss[dof] == pytest.approx(expected, abs=1e-6), (
        f"build_model(ballast_mass_kg={ballast_mass_kg}) carries frictionloss "
        f"{model.dof_frictionloss[dof]:.4f} N*m, expected Crr*W*r = {expected:.4f} "
        "N*m from this model's OWN compiled load"
    )


@pytest.mark.parametrize("ballast_mass_kg", [0.0, 70.0])
def test_build_terrain_model_frictionloss_matches_the_compiled_load(ballast_mass_kg):
    """Same check, for the OTHER weld-path builder. `terrain.build_terrain_
    model` calls the same `correct_wheel_drag_for_load`, but it assembles its
    own XML independently (a heightfield ground rather than a plane) -- this
    proves the wiring survived that independent assembly too."""
    params = terrain.TerrainParams(ballast_mass_kg=ballast_mass_kg)
    model, _amp = terrain.build_terrain_model(params)
    dof = _dof(model, "wheel_hinge")
    expected = _expected_frictionloss(model)
    assert model.dof_frictionloss[dof] == pytest.approx(expected, abs=1e-6), (
        f"build_terrain_model(ballast_mass_kg={ballast_mass_kg}) carries "
        f"frictionloss {model.dof_frictionloss[dof]:.4f} N*m, expected Crr*W*r = "
        f"{expected:.4f} N*m from this model's OWN compiled load"
    )


def test_a_heavier_weld_path_model_carries_more_frictionloss_than_a_lighter_one():
    """Not just "matches some formula" -- the formula must actually respond
    to load. A frictionloss that matched `Crr*W*r` by accident (e.g. because
    `total_mass_kg` was silently always the driverless 12.5 kg) would pass
    the two tests above at ballast_mass_kg=0 and still be wrong at 70."""
    driverless = plant.build_model(0.0, 0.75, 40.0)
    ridden = plant.build_model(70.0, 0.75, 40.0)
    fl_driverless = driverless.dof_frictionloss[_dof(driverless, "wheel_hinge")]
    fl_ridden = ridden.dof_frictionloss[_dof(ridden, "wheel_hinge")]
    assert fl_ridden > 2.0 * fl_driverless, (
        f"welding 70 kg of ballast on only moved frictionloss from "
        f"{fl_driverless:.4f} to {fl_ridden:.4f} N*m -- the load-scaling this "
        "whole path exists for is not taking effect"
    )


# ---------------------------------------------------------------------------
# Item 2 -- the driverless/ridden split: exactly the confound this branch
# fixed, guarded so it cannot silently return.
# ---------------------------------------------------------------------------


def test_build_model_driverless_has_no_aero_the_ridden_case_does():
    driverless = plant.build_model(0.0, 0.75, 40.0)
    ridden = plant.build_model(70.0, 0.75, 40.0)
    assert driverless.opt.density == 0.0, (
        "a driverless build_model() call must not enable the fluid model -- "
        "see overboard_onewheel.xml's own header for why aero is deliberately "
        "absent on the bare driverless plant"
    )
    assert ridden.opt.density != 0.0, (
        "a welded-ballast build_model() call must enable the fluid model, or "
        "the run carries rider-scale MASS with no rider-scale aero drag"
    )


def test_build_terrain_model_driverless_has_no_aero_the_ridden_case_does():
    driverless, _ = terrain.build_terrain_model(terrain.TerrainParams(ballast_mass_kg=0.0))
    ridden, _ = terrain.build_terrain_model(terrain.TerrainParams(ballast_mass_kg=70.0))
    assert driverless.opt.density == 0.0, (
        "a driverless build_terrain_model() call must not enable the fluid model"
    )
    assert ridden.opt.density != 0.0, (
        "a welded-ballast build_terrain_model() call must enable the fluid model"
    )


# ---------------------------------------------------------------------------
# Item 3 -- the MuJoCo fluid trap, for THIS path's own null overrides.
# Highest-value test in this file: the trap is invisible on inspection, so
# this proves by MEASUREMENT that each override actually does something.
# ---------------------------------------------------------------------------


def test_weld_path_wheel_null_override_is_non_vacuous(monkeypatch):
    """`add_rider_aero` null-overrides `wheel_geom` (see its own docstring
    for why: enabling `<option density>` gives the legacy per-body fluid
    model a rotational grip on the spinning wheel that has nothing to do
    with `wheel_hinge`'s own frictionloss/damping). Neutralising the
    constant that supplies that override -- rather than reimplementing
    `build_model`'s assembly here -- measures the REAL code path.
    """
    overridden = plant.build_model(70.0, 0.75, 40.0)
    monkeypatch.setattr(plant, "_NULL_FLUID_OVERRIDE", "")
    stripped = plant.build_model(70.0, 0.75, 40.0)

    # Wheel spinning, frame (and therefore the rigidly-welded ballast) at
    # rest -- isolates wheel_geom's own rotational fluid contribution from
    # everything else on the model, including the still-intact ballast
    # override (which cannot contribute: it is not moving).
    omega = 8.34 / float(overridden.geom_size[
        mujoco.mj_name2id(overridden, mujoco.mjtObj.mjOBJ_GEOM, "wheel_geom")
    ][0])
    dof_over = _dof(overridden, "wheel_hinge")
    dof_stripped = _dof(stripped, "wheel_hinge")
    f_over = _isolated_qfrc_passive(overridden, dof_over, omega)
    f_stripped = _isolated_qfrc_passive(stripped, dof_stripped, omega)
    assert f_over != pytest.approx(f_stripped, abs=1e-9), (
        "removing the wheel_geom null override did not change wheel_hinge's "
        "passive force -- the override is not reaching the compiled model, so "
        "this whole path's fluid nulling is vacuous"
    )
    print(
        f"\nweld-path wheel legacy-fluid contamination avoided by the override: "
        f"{abs(f_stripped - f_over):.4f} N*m at top speed"
    )


def test_weld_path_ballast_null_override_is_non_vacuous(monkeypatch):
    """`BALLAST_FLUID_NULL_GEOM` is a STRING baked once at import time (an
    f-string over `_NULL_FLUID_OVERRIDE`), not re-evaluated per call, so
    neutralising `_NULL_FLUID_OVERRIDE` alone (as the previous test does)
    would not touch it. This isolates the ballast override specifically, by
    replacing that baked string directly, while leaving `wheel_geom`'s
    override intact on both sides -- so the whole difference below is
    attributable to the ballast override and nothing else."""
    overridden = plant.build_model(70.0, 0.75, 40.0)
    assert _NULL_FLUID_OVERRIDE in plant.BALLAST_FLUID_NULL_GEOM, (
        "BALLAST_FLUID_NULL_GEOM no longer contains the expected null-override "
        "substring -- update _NULL_FLUID_OVERRIDE in this test to match"
    )
    monkeypatch.setattr(
        plant, "BALLAST_FLUID_NULL_GEOM",
        plant.BALLAST_FLUID_NULL_GEOM.replace(_NULL_FLUID_OVERRIDE, ""),
    )
    stripped = plant.build_model(70.0, 0.75, 40.0)

    # Frame (and the rigidly-welded ballast) moving, wheel_hinge at rest --
    # isolates ballast's own legacy contribution PLUS the always-present
    # rider_aero_geom term, which is IDENTICAL between the two models (only
    # the ballast override differs), so the difference below is entirely
    # the ballast leak.
    speed = 8.34
    f_over = _frame_only_force_n(overridden, speed)
    f_stripped = _frame_only_force_n(stripped, speed)
    assert f_over != pytest.approx(f_stripped, abs=1e-9), (
        "removing BALLAST_FLUID_NULL_GEOM's override did not change the "
        "frame dof's passive force -- the override is not reaching the "
        "compiled model, so the ballast body's own fluid nulling is vacuous"
    )
    print(
        f"\nweld-path ballast legacy-fluid contamination avoided by the "
        f"override: {abs(f_stripped - f_over):.4f} N at {speed} m/s"
    )


# ---------------------------------------------------------------------------
# Item 4 -- the empirical anchors, for the weld path's own compiled output.
# `test_rider_drag_model.py` checks these against `overboard_rider.xml`;
# this checks the SAME bands against the model the scenarios that actually
# call `build_model`/`build_terrain_model` run on.
# ---------------------------------------------------------------------------


def _wheel_mechanical_drag_n(frictionloss_nm: float, v: float, r: float) -> float:
    return frictionloss_nm / r + BEARING_DAMPING_NM_S_RAD * v / (r**2)


def test_weld_path_total_drag_lands_inside_the_published_real_total_bands():
    model = plant.build_model(70.0, 0.75, 40.0)
    dof = _dof(model, "wheel_hinge")
    frictionloss = float(model.dof_frictionloss[dof])
    wheel_gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "wheel_geom")
    r = float(model.geom_size[wheel_gid][0])
    print(f"\n{'v (m/s)':>8} {'mech (N)':>9} {'aero (N)':>9} {'total (N)':>10} {'published band':>18}")
    for v, (lo, hi) in REAL_TOTAL_N.items():
        mech = _wheel_mechanical_drag_n(frictionloss, v, r)
        aero = abs(_frame_only_force_n(model, v))
        total = mech + aero
        print(f"{v:8.2f} {mech:9.3f} {aero:9.3f} {total:10.3f} {f'{lo:.1f}-{hi:.1f}':>18}")
        assert lo <= total <= hi, (
            f"weld-path model's total predicted drag at {v} m/s is {total:.2f} N, "
            f"outside the published real-total band {lo}-{hi} N -- report this "
            "discrepancy, do not adjust Crr/damping/CdA to force a match"
        )


def test_weld_path_effective_cda_lands_inside_the_targeted_band():
    """`add_rider_aero` reuses `overboard_rider.xml`'s aero geom VERBATIM, so
    this should land on the same measured CdA (~0.754 m^2, see
    `test_rider_drag_model.py::test_rider_aero_geom_achieves_the_targeted_
    cda`) -- checked here from THIS model's own fluid-force output, not
    assumed from that other file's measurement."""
    model = plant.build_model(70.0, 0.75, 40.0)
    cdas = []
    for v in (1.0, 4.0, 8.34):
        f = abs(_frame_only_force_n(model, v))
        cdas.append(2.0 * f / (RHO_AIR * v * v))
    spread = max(cdas) - min(cdas)
    assert spread < 1e-3, (
        f"weld-path measured CdA is not consistent across speed ({cdas}) -- "
        "the aero force is not a clean quadratic term, which points at a "
        "legacy fluid leak that item-3's overrides did not fully null"
    )
    measured_cda = sum(cdas) / len(cdas)
    assert 0.6 <= measured_cda <= 0.9, (
        f"weld-path measured CdA {measured_cda:.4f} m^2 is outside the "
        "targeted 0.6-0.9 m^2 upright-rider band"
    )
