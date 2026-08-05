"""The two-term wheel drag model + rider aerodynamic drag, MEASURED.

Replaces the old, undocumented `wheel_hinge damping="0.08"` -- a single
viscous coefficient standing in for THREE physically distinct drag
mechanisms with three different speed-scalings (constant, linear,
quadratic) -- with the two terms that actually belong on that joint
(`frictionloss` for rolling resistance, `damping` for bearing/motor losses)
plus a third term, rider aerodynamic drag, that does NOT belong on the
joint at all and was previously absent entirely.

See `sim/models/overboard_onewheel.xml` and `sim/models/overboard_rider.xml`
for the full derivation of every constant (Crr, the viscous coefficient, the
aero geometry) -- this file exists to MEASURE what the model actually does,
not to restate why the constants were chosen. In particular: this file does
NOT assert the rider's effective CdA from the XML config alone, per the
standing project rule against presenting an unverified number as verified --
`test_rider_aero_geom_achieves_the_targeted_cda` measures it from the
model's own fluid-force output.

THE VALIDATION TABLES BELOW ARE EXTERNAL ANCHORS, NOT ACCEPTANCE CRITERIA
--------------------------------------------------------------------------
The "real total" speed/force table and the ~29-33 N / ~10-11.5 Wh/km cruise
figure are published Onewheel-class reference points this repo does not
control and cannot re-derive from first principles. Landing inside them is
evidence the model is no longer three-to-five-times too low, not a new gate
on this repo's own control loop -- nothing here may be cited as an ADR-0011
acceptance criterion.
"""

from __future__ import annotations

from pathlib import Path

import mujoco
import pytest

REPO = Path(__file__).resolve().parents[1]
ONEWHEEL_XML = REPO / "sim" / "models" / "overboard_onewheel.xml"
RIDER_XML = REPO / "sim" / "models" / "overboard_rider.xml"

#: Air density, kg/m^3 -- ISA sea-level standard, this file's own
#: `<option density>` value. Duplicated here (no Rust/Python binding for a
#: raw MJCF attribute) so a drift in either place is caught rather than
#: silently producing a different CdA.
RHO_AIR = 1.225

#: The three "real total" reference speeds and their published low-high
#: onewheel-class drag bands (see this repo's own drag-model change log /
#: PR description for the source table). Newtons.
REAL_TOTAL_N = {
    1.0: (12.5, 20.8),
    4.0: (18.0, 29.0),
    8.34: (37.7, 58.6),
}

#: Rolling-resistance coefficient, centre of the 0.015-0.025 working band for
#: an 18 psi wide pneumatic tyre -- see either model file's `wheel_hinge`
#: comment for the citation.
CRR = 0.02

#: Bearing + hub-motor iron/windage viscous coefficient, centre of the
#: 0.006-0.012 N*m*s/rad band -- see either model file's `wheel_hinge`
#: comment.
BEARING_DAMPING_NM_S_RAD = 0.009

WHEEL_RADIUS_M = 0.1454


#: Mesh assets supplied in-memory, mirroring `sim/scenarios/plant.py::
#: build_model`'s own reasoning: `from_xml_string` needs the meshes handed to
#: it directly rather than resolved relative to a CWD that pytest does not
#: control.
_MESH_DIR = REPO / "sim" / "models" / "meshes" / "openwheel"
_ASSETS = {p.name: p.read_bytes() for p in _MESH_DIR.glob("*.stl")}


def _load(path: Path) -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_string(path.read_text(), _ASSETS)


def _load_string(xml: str) -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_string(xml, _ASSETS)


def _dof(model: mujoco.MjModel, joint_name: str) -> int:
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    assert jid >= 0, f"model has no joint named {joint_name!r}"
    return int(model.jnt_dofadr[jid])


def _body_mass_kg(model: mujoco.MjModel, *names: str) -> float:
    total = 0.0
    for name in names:
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        assert bid >= 0, f"model has no body named {name!r}"
        total += float(model.body_mass[bid])
    return total


# ---------------------------------------------------------------------------
# The wheel-hinge constants: are they the derived numbers, and do they round
# trip through the compiled model unchanged?
# ---------------------------------------------------------------------------


def test_onewheel_wheel_hinge_carries_the_driverless_derived_constants():
    """`overboard_onewheel.xml`'s own load: frame 8.0 kg + wheel 4.5 kg."""
    model = _load(ONEWHEEL_XML)
    dof = _dof(model, "wheel_hinge")
    weight_n = _body_mass_kg(model, "frame", "wheel") * 9.81
    expected_frictionloss = CRR * weight_n * WHEEL_RADIUS_M
    assert model.dof_frictionloss[dof] == pytest.approx(expected_frictionloss, abs=1e-3), (
        f"frictionloss should be Crr*W*r = {expected_frictionloss:.4f} N*m from this "
        f"model's own {weight_n:.2f} N total weight"
    )
    assert model.dof_damping[dof] == pytest.approx(BEARING_DAMPING_NM_S_RAD, abs=1e-6)


def test_rider_wheel_hinge_carries_the_ridden_derived_constants():
    """`overboard_rider.xml`'s own load: frame + wheel + ballast carrier + rider."""
    model = _load(RIDER_XML)
    dof = _dof(model, "wheel_hinge")
    weight_n = _body_mass_kg(model, "frame", "wheel", "ballast_fa_carrier", "ballast") * 9.81
    expected_frictionloss = CRR * weight_n * WHEEL_RADIUS_M
    assert model.dof_frictionloss[dof] == pytest.approx(expected_frictionloss, abs=1e-3), (
        f"frictionloss should be Crr*W*r = {expected_frictionloss:.4f} N*m from this "
        f"model's own {weight_n:.2f} N total weight"
    )
    assert model.dof_damping[dof] == pytest.approx(BEARING_DAMPING_NM_S_RAD, abs=1e-6)
    # Same drivetrain constant as the driverless model -- it is a property of
    # the bearings/motor, not of the load.
    onewheel_dof = _dof(_load(ONEWHEEL_XML), "wheel_hinge")
    onewheel_damping = _load(ONEWHEEL_XML).dof_damping[onewheel_dof]
    assert model.dof_damping[dof] == pytest.approx(onewheel_damping, abs=1e-9)


def test_onewheel_has_no_aerodynamic_drag_mechanism():
    """The driverless model must not pick up rider-sized (or ANY) aero --
    per its own header note, this is a deliberate omission, not a gap."""
    model = _load(ONEWHEEL_XML)
    assert model.opt.density == 0.0, (
        "overboard_onewheel.xml's <option> must not enable the fluid model -- "
        "see this file's own header for why aero is deliberately absent here"
    )


# ---------------------------------------------------------------------------
# Positive control: the null fluid overrides on wheel/ballast/carrier bodies
# actually zero what MuJoCo's legacy per-body model would otherwise add.
# ---------------------------------------------------------------------------


def _isolated_qfrc_passive(model: mujoco.MjModel, dof: int, qvel_value: float) -> float:
    data = mujoco.MjData(model)
    data.qvel[:] = 0.0
    data.qvel[dof] = qvel_value
    mujoco.mj_forward(model, data)
    return float(data.qfrc_passive[dof])


def test_wheel_null_fluid_override_actually_zeros_the_legacy_contribution():
    """Without the override MuJoCo's legacy per-body model gives the
    spinning wheel body its own nonzero rotational drag (on top of, and
    separate from, `wheel_hinge`'s own frictionloss/damping) -- this proves
    the override in `overboard_rider.xml` is not a no-op comment."""
    xml = RIDER_XML.read_text()
    overridden = _load_string(xml)
    stripped = _load_string(
        xml.replace('fluidshape="ellipsoid" fluidcoef="0 0 0 0 0"', "")
    )
    dof = _dof(overridden, "wheel_hinge")
    # A velocity high enough that the legacy model's contribution (if any)
    # is not lost in float noise. `wheel_hinge` also carries frictionloss/
    # damping, which is IDENTICAL between the two models (only the fluid
    # override differs), so any difference between them below is entirely
    # the legacy fluid contribution.
    omega = 8.34 / WHEEL_RADIUS_M
    f_over = _isolated_qfrc_passive(overridden, dof, omega)
    f_stripped = _isolated_qfrc_passive(stripped, dof, omega)
    assert f_over != pytest.approx(f_stripped, abs=1e-9), (
        "stripping the override did not change wheel_hinge's passive force -- "
        "the override is not reaching the model, so this whole test is vacuous"
    )
    print(
        f"\nwheel legacy-fluid contamination avoided by the override: "
        f"{abs(f_stripped - f_over):.4f} N*m at top speed"
    )


def test_ballast_and_carrier_null_overrides_zero_their_standalone_fluid_force():
    """`ballast` (70 kg rider mass) and `ballast_fa_carrier` (the near-
    massless linkage stage) must contribute NOTHING to the fluid force on
    their own -- all rider aero lives on `rider_aero_geom` alone, or it
    double-counts."""
    xml = RIDER_XML.read_text()
    overridden = _load_string(xml)
    stripped = _load_string(
        xml.replace('fluidshape="ellipsoid" fluidcoef="0 0 0 0 0"', "")
    )
    # Isolate ballast_fa (fore/aft slide) and ballast_lat (lateral slide)
    # dofs directly -- each is a single-dof slide joint whose own qvel
    # entirely determines that body's translational velocity. Both joints
    # ALSO declare their own weight-shift-actuator `damping="600"`, which is
    # unrelated to fluid drag and shows up in the same `qfrc_passive` slot --
    # so the assertion is that `f_over` equals exactly that known joint-
    # damping term (no fluid leak on top), not that it is zero outright.
    v = 5.0
    for joint in ("ballast_fa", "ballast_lat"):
        joint_id = mujoco.mj_name2id(overridden, mujoco.mjtObj.mjOBJ_JOINT, joint)
        joint_damping_only = -float(overridden.dof_damping[overridden.jnt_dofadr[joint_id]]) * v
        dof = _dof(overridden, joint)
        f_over = _isolated_qfrc_passive(overridden, dof, v)
        f_stripped = _isolated_qfrc_passive(stripped, dof, v)
        assert f_over == pytest.approx(joint_damping_only, abs=1e-9), (
            f"{joint}: overridden model should show ONLY the joint's own "
            f"weight-shift damping ({joint_damping_only:.4f} N), got {f_over}"
        )
        assert f_stripped != pytest.approx(joint_damping_only, abs=1e-6), (
            f"{joint}: the un-overridden model shows the same force as the "
            "overridden one, so this test cannot tell whether the override "
            "does anything"
        )
        leaked = f_stripped - joint_damping_only
        print(f"\n{joint}: null override zeros {leaked:.4f} N of legacy fluid force")


def test_stub_null_geoms_do_not_perturb_body_mass_or_inertia():
    """The `ballast_fa_carrier` stub geom and the null-override attributes on
    `wheel_geom`/`ballast_mass_geom` must not change the bodies' compiled
    mass/inertia -- every body in this file declares an explicit
    `<inertial>`, so a geom must never contribute to it, but this is
    verified rather than assumed."""
    model = _load(RIDER_XML)
    masses = {
        "frame": 8.0,
        "wheel": 4.5,
        "ballast_fa_carrier": 0.5,
        "ballast": 70.0,
    }
    for name, expected in masses.items():
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        assert model.body_mass[bid] == pytest.approx(expected, abs=1e-9), (
            f"{name}: mass changed to {model.body_mass[bid]}, expected {expected} -- "
            "a null-override or stub geom perturbed the compiled inertial"
        )


# ---------------------------------------------------------------------------
# The rider aero geom: MEASURED effective CdA, not asserted from config.
# ---------------------------------------------------------------------------


def _frame_only_aero_force_n(model: mujoco.MjModel, speed_m_s: float) -> float:
    """Isolate `rider_aero_geom`'s contribution by moving ONLY the frame's
    free-joint linear x-dof (forward = -x) and reading `qfrc_passive` on
    that same dof. `wheel_hinge` is a different degree of freedom (dofadr 6
    on this model), so it cannot leak into this measurement -- confirmed by
    `test_frame_dof_isolation_excludes_wheel_hinge` below."""
    data = mujoco.MjData(model)
    data.qvel[:] = 0.0
    data.qvel[0] = -speed_m_s  # forward = -x
    mujoco.mj_forward(model, data)
    return float(data.qfrc_passive[0])


def test_frame_dof_isolation_excludes_wheel_hinge():
    """Positive control for the isolation method itself: driving the wheel
    hinge alone (frame at rest) must not appear on the frame's own dof."""
    model = _load(RIDER_XML)
    dof = _dof(model, "wheel_hinge")
    data = mujoco.MjData(model)
    data.qvel[:] = 0.0
    data.qvel[dof] = 8.34 / WHEEL_RADIUS_M
    mujoco.mj_forward(model, data)
    assert data.qfrc_passive[0] == pytest.approx(0.0, abs=1e-12), (
        "spinning wheel_hinge alone produced a nonzero force on the frame's "
        "own free-joint dof -- the isolation this file relies on is invalid"
    )


def test_rider_aero_geom_achieves_the_targeted_cda():
    """MEASURED effective CdA from the compiled model's own fluid-force
    output -- not read off `rider_aero_geom`'s size attribute. Checked at
    three speeds so a non-quadratic contamination (e.g. a legacy per-body
    leak that was not fully nulled) would show up as CdA varying with
    speed rather than landing on one number."""
    model = _load(RIDER_XML)
    cdas = []
    print(f"\n{'v (m/s)':>8} {'F (N)':>8} {'CdA (m^2)':>10}")
    for v in (1.0, 4.0, 8.34):
        f = abs(_frame_only_aero_force_n(model, v))
        cda = 2.0 * f / (RHO_AIR * v * v)
        cdas.append(cda)
        print(f"{v:8.2f} {f:8.4f} {cda:10.5f}")
    spread = max(cdas) - min(cdas)
    assert spread < 1e-3, (
        f"measured CdA is not consistent across speed ({cdas}) -- the aero "
        "force is not a clean quadratic term"
    )
    measured_cda = sum(cdas) / len(cdas)
    assert 0.6 <= measured_cda <= 0.9, (
        f"measured CdA {measured_cda:.4f} m^2 is outside the targeted "
        "0.6-0.9 m^2 upright-rider band"
    )
    print(f"\nMEASURED effective CdA: {measured_cda:.4f} m^2 (target band 0.6-0.9 m^2)")


# ---------------------------------------------------------------------------
# Validation against the "real total" table and the Wh/km cruise anchor.
# Informational: prints the comparison and asserts landing inside the
# published band, but nothing here may be cited as an acceptance criterion.
# ---------------------------------------------------------------------------


def _wheel_mechanical_drag_n(frictionloss_nm: float, v: float) -> float:
    """`frictionloss/r` (constant, Coulomb) + `damping*v/r^2` (linear,
    viscous) -- the ground-equivalent force of the two `wheel_hinge` terms
    under a no-slip rolling assumption (torque / r)."""
    return frictionloss_nm / WHEEL_RADIUS_M + BEARING_DAMPING_NM_S_RAD * v / (WHEEL_RADIUS_M**2)


def test_total_predicted_drag_lands_inside_the_published_real_total_bands():
    model = _load(RIDER_XML)
    dof = _dof(model, "wheel_hinge")
    frictionloss = float(model.dof_frictionloss[dof])
    print(f"\n{'v (m/s)':>8} {'mech (N)':>9} {'aero (N)':>9} {'total (N)':>10} {'published band':>18}")
    for v, (lo, hi) in REAL_TOTAL_N.items():
        mech = _wheel_mechanical_drag_n(frictionloss, v)
        aero = abs(_frame_only_aero_force_n(model, v))
        total = mech + aero
        print(f"{v:8.2f} {mech:9.3f} {aero:9.3f} {total:10.3f} {f'{lo:.1f}-{hi:.1f}':>18}")
        assert lo <= total <= hi, (
            f"at {v} m/s the model's total predicted drag {total:.2f} N falls "
            f"outside the published real-total band {lo}-{hi} N -- report this "
            "discrepancy, do not adjust Crr/damping/CdA to force a match"
        )


#: Assumed drivetrain efficiency (battery-to-wheel), per the PR description's
#: own derivation of the 29-33 N / ~10-11.5 Wh/km anchor pair: mechanical
#: (wheel) work is ~80% of battery (wall-plug-equivalent) energy drawn. The
#: 29-33 N figure is already MECHANICAL (no factor needed); the 10-11.5
#: Wh/km figure is BATTERY-side, so this model's mechanical Wh/km must be
#: divided by this efficiency before comparing against it.
DRIVETRAIN_EFFICIENCY = 0.80


def test_wh_per_km_at_a_representative_cruise_speed_against_the_energy_anchor():
    """~29-33 N of average mechanical drag / ~10-11.5 Wh/km (battery) is the
    published Onewheel-class energy-audit anchor (GT 525 Wh over 32 mi,
    Pint 148 Wh over 8 mi, at ~80% drivetrain efficiency -- see this repo's
    own PR description for the derivation). It is an AVERAGE over a real
    ride's speed profile, not a single steady-state speed, so there is no
    single "the" cruise speed to check it against. This measures the
    model's own total drag/energy rate across a small band of plausible
    comfortable cruising speeds (as distinct from the 8.34 m/s TOP-speed
    cap) and reports where it crosses the anchor, rather than asserting one
    arbitrarily chosen speed is "the" cruise speed.
    """
    model = _load(RIDER_XML)
    dof = _dof(model, "wheel_hinge")
    frictionloss = float(model.dof_frictionloss[dof])
    print(f"\n{'v (m/s)':>8} {'F (N)':>8} {'mech Wh/km':>11} {'battery Wh/km':>14}")
    crossed_anchor = False
    for v in (4.5, 5.0, 5.2, 5.5, 6.0, 6.5):
        mech = _wheel_mechanical_drag_n(frictionloss, v)
        aero = abs(_frame_only_aero_force_n(model, v))
        total = mech + aero
        power_w = total * v
        # Wh to cover 1 km at this steady speed: (power * time) / 3600,
        # time = 1000 m / v. This is MECHANICAL (wheel-side) energy.
        mech_wh_per_km = power_w * (1000.0 / v) / 3600.0
        battery_wh_per_km = mech_wh_per_km / DRIVETRAIN_EFFICIENCY
        print(f"{v:8.2f} {total:8.3f} {mech_wh_per_km:11.3f} {battery_wh_per_km:14.3f}")
        if 29.0 <= total <= 33.0 and 10.0 <= battery_wh_per_km <= 11.5:
            crossed_anchor = True
    assert crossed_anchor, (
        "no speed in the swept comfortable-cruise range (4.5-6.5 m/s) lands "
        "the model's total drag AND battery-side Wh/km simultaneously inside "
        "the published anchor (29-33 N / 10-11.5 Wh/km) -- report this "
        "discrepancy rather than adjusting Crr/damping/CdA to force a match"
    )
