"""The plant: the onewheel model, optionally carrying a rider-scale mass.

Lives in `sim/scenarios/` rather than in a script because it IS the plant, and
both the experiment runner and the scenarios need it. A scenario importing the
model out of `scripts/` would have the dependency backwards.
"""

from __future__ import annotations

import functools
from pathlib import Path

MODEL_PATH = Path(__file__).resolve().parents[2] / "sim" / "models" / "overboard_onewheel.xml"


@functools.lru_cache(maxsize=1)
def R_MODEL_TO_ICD():
    """Rotation from the MuJoCo model frame to the ICD body frame (§10.1).

    Model: z-up, **forward = −X** (overboard_onewheel.xml). ICD: FRD,
    right-handed — x forward, y right, z down. Forward flips and up flips;
    right is shared. That is a 180° rotation about +Y.

    A frame change is a PROPER rotation. The determinant assertion is the whole
    reason this is a matrix and not a per-axis sign triple: `det = −1` is a
    reflection, it turns a right-handed frame into a left-handed one, and it is
    exactly the defect this replaced. A sign triple gives you nowhere to check.

    Built lazily and cached because this module imports numpy per-function on
    purpose (see the module docstring's note on dependency direction).
    """
    import numpy as np

    R = np.diag([-1.0, 1.0, -1.0])
    det = float(np.linalg.det(R))
    if abs(det - 1.0) > 1e-12:
        raise ValueError(
            f"model→ICD frame map must be a proper rotation (det = +1), got {det:+.6f}. "
            "A det = −1 map is a reflection and produces a left-handed frame, which "
            "ICD §10.1 forbids."
        )
    return R

#: Motor torque constant, N*m per amp. **UNFITTED PLACEHOLDER.**
#:
#: The MJCF actuator is a torque source; the controller commands current. This
#: is the conversion, and it is the first thing the bench must measure: a
#: current step of known magnitude against a known inertia gives kt directly
#: (Bench Test-Stand, Config A). 0.7 is an order-of-magnitude guess for a
#: hoverboard-class hub motor and nothing has been fitted to it.
#:
#: It is named rather than implicit because it was previously implicit at 1.0 --
#: amps written straight into a N*m channel -- which made `ctrlrange` (N*m) and
#: `max_current_a` (A) look like the same number and hid the unit error.
#:
#: The model's ctrlrange is derived from this: 40 A x 0.7 = 28 N*m.
KT_NM_PER_A = 0.7

#: Rider-proxy colours, from the overboard-web palette.
#:
#: MINT for the body, not ink: the board is dark navy, and a dark rider on a
#: dark board reads as one indistinct lump -- which is most of what was wrong
#: with the first attempt. Mint separates from both the board and the pale
#: ground. Amber helmet to tie to the deck grips.
_MINT = "0.165 0.682 0.592 1"
_AMBER = "0.949 0.635 0.290 1"
_CLOUD = "0.957 0.973 0.969 1"
_AMBER_DK = "0.769 0.408 0.094 1"
_INK = "0.086 0.137 0.180 1"


def rider_geoms(style: str, com_height: float) -> str:
    """Visual-only geometry for the rider proxy, in the ballast body's frame.

    **This changes no physics.** The ballast body carries an explicit
    `<inertial>`, so geoms contribute no mass whatever their shape, and every
    geom here is `contype=0 conaffinity=0`, so none of them collide. Adding
    them leaves the metrics bit-identical, which the caller asserts.

    On the choice of a stylised mannequin over a realistic human mesh: the
    model has **no rider dynamics at all**. This is a rigid lump bolted to the
    frame -- it does not articulate, shift weight, or absorb anything at the
    ankles and knees, which is most of what a real rider does. A photoreal
    figure in a clip that gets shared would imply a fidelity the physics does
    not have. A mannequin reads as *proxy* rather than *simulated rider*, which
    is the honest signal, and it costs no third-party asset or licence in a
    repo that has an unresolved licensing gap already (SR-OSS-1).

    Styled after the stick figure on the landing page rather than invented:
    that drawing is a big circular head on thin curved limbs, and its head is
    roughly a fifth of total height. Cartoon proportions are what make it read
    as a character instead of an anatomy diagram, and they are the reason a
    first attempt at "realistic" proportions with thick limbs looked like a
    pile of capsules.
    """
    if style == "sphere":
        return (f'<geom name="rider_mass" type="sphere" size="0.12" '
                f'rgba="{_AMBER}" contype="0" conaffinity="0" group="1"/>')

    deck = -com_height          # the deck sits at the axle
    ankle = deck + 0.03
    knee = deck + 0.34          # a onewheel stance is a crouch
    hip = deck + 0.66
    sh = deck + 1.06
    head = deck + 1.26          # head centre; r=0.135 puts the crown at ~1.40

    def g(n, frm, to, r, c=_MINT):
        return (f'<geom name="rider_{n}" type="capsule" fromto="{frm} {to}" '
                f'size="{r}" rgba="{c}" contype="0" conaffinity="0" group="1"/>')

    # Thin limbs, because the reference is a LINE drawing. The first version
    # used 0.055 and read as plumbing.
    LIMB, ARM = 0.032, 0.026
    hx = -0.04                  # head/torso lean, into the direction of travel
    parts = [
        # feet, fore and aft of the wheel on the two footpads
        g("foot_f", f"-0.27 0 {ankle}", f"-0.15 0 {ankle}", 0.038),
        g("foot_r", f"0.15 0 {ankle}", f"0.27 0 {ankle}", 0.038),
        # legs, knees bent outward -- two segments so the bend actually reads
        g("shin_f", f"-0.21 0 {ankle}", f"-0.17 0 {knee}", LIMB),
        g("thigh_f", f"-0.17 0 {knee}", f"-0.05 0 {hip}", LIMB),
        g("shin_r", f"0.21 0 {ankle}", f"0.17 0 {knee}", LIMB),
        g("thigh_r", f"0.17 0 {knee}", f"0.05 0 {hip}", LIMB),
        # spine, leaning very slightly into the direction of travel
        g("torso", f"0 0 {hip}", f"{hx} 0 {sh}", 0.052),

        # ARMS RUN FORE AND AFT, along the travel axis -- not across it.
        # A onewheel stance is sideways: the feet straddle the wheel along the
        # board's long axis, so the shoulders line up with that axis too and
        # the arms swing along it. Arms held out square to the direction of
        # travel is a scooter stance, and nobody rides like that.
        # Elbows carry a modest bend; enough to read as an arm, not a pose.
        g("uarm_f", f"{hx-0.06} 0.02 {sh}", f"-0.28 0.09 {sh - 0.13}", ARM),
        g("farm_f", f"-0.28 0.09 {sh - 0.13}", f"-0.40 0.14 {sh - 0.02}", ARM),
        g("uarm_r", f"{hx+0.06} -0.02 {sh}", f"0.26 -0.06 {sh - 0.15}", ARM),
        g("farm_r", f"0.26 -0.06 {sh - 0.15}", f"0.38 -0.02 {sh - 0.05}", ARM),

        g("neck", f"{hx} 0 {sh}", f"{hx} 0 {head - 0.10}", 0.030),
        # head: ~20% of total height, per the landing-page figure
        f'<geom name="rider_head" type="sphere" size="0.135" pos="{hx} 0 {head}" '
        f'rgba="{_CLOUD}" contype="0" conaffinity="0" group="1"/>',

        # --- helmet: a shell that stops above the brow, plus a peak ---------
        # Sat high and back so it reads as worn rather than as a bucket over
        # the whole head; the face has to stay visible or the smile is wasted.
        f'<geom name="rider_helmet" type="ellipsoid" size="0.147 0.150 0.125" '
        f'pos="{hx - 0.005} -0.008 {head + 0.052}" rgba="{_AMBER}" '
        f'contype="0" conaffinity="0" group="1"/>',
        f'<geom name="rider_peak" type="ellipsoid" size="0.085 0.075 0.016" '
        f'pos="{hx} 0.105 {head + 0.045}" rgba="{_AMBER_DK}" '
        f'contype="0" conaffinity="0" group="1"/>',

    ]

    # --- face: two eyes and a smile, projected ONTO the head sphere ---------
    # Cartoonish on purpose. It also does a job: at video scale the face is the
    # fastest way to see which way the rider is pointing.
    #
    # Features are placed by projecting onto the sphere rather than at a fixed
    # depth. A flat depth buries the middle of an arc inside the head -- which
    # is exactly what happened to the first smile, and it vanished.
    import math

    HEAD_R = 0.135

    def on_face(name, dx, dz, size):
        dy = math.sqrt(max(HEAD_R**2 - dx**2 - dz**2, 1e-6))
        return (f'<geom name="rider_{name}" type="sphere" size="{size}" '
                f'pos="{hx + dx:.4f} {dy:.4f} {head + dz:.4f}" rgba="{_INK}" '
                f'contype="0" conaffinity="0" group="1"/>')

    parts.append(on_face("eye_l", -0.050, 0.020, 0.024))
    parts.append(on_face("eye_r", 0.050, 0.020, 0.024))

    # Smile: an arc of beads. MuJoCo has no torus and a capsule is straight, so
    # the curve is sampled -- five beads reads as a grin at video scale.
    # Six beads over a wider arc. The face is seen at an angle in every camera,
    # so the far half foreshortens badly -- a narrow mouth reads as a smirk.
    for i in range(6):
        a = math.pi * (0.26 + 0.48 * i / 5.0)
        parts.append(on_face(f"smile{i}",
                             0.078 * math.cos(a),
                             -0.030 - 0.042 * math.sin(a),
                             0.0165))
    return "".join(parts)


# ---------------------------------------------------------------------------
# Terrain — a genuinely tilted ground plane
# ---------------------------------------------------------------------------
#
# WHY THIS EXISTS ALONGSIDE THE ROTATED-GRAVITY HILL, NOT INSTEAD OF IT
# ---------------------------------------------------------------------
# `hill.py` models a grade by rotating the gravity vector and leaving the
# ground flat. That was the right first call: it keeps the nose-strike contact
# geometry bit-identical to every other scenario, so an 18.57 deg strike on a
# hill means what it means on the flat. Changing the geometry to get a hill
# would have moved the one contact case the scenario exists to measure.
#
# It has two ceilings, and one of them is not a physics problem at all:
#
#   1. IT CANNOT BE FILMED. A gravity-rotated hill renders as a board on flat
#      ground accelerating for no visible reason. For the build log and the
#      design-doc video that footage is actively misleading -- it reads as a
#      bug, not a hill. A real tilted plane is the only version with a slope in
#      frame.
#   2. IT CAN ONLY EVER BE AN INFINITE UNIFORM PLANE. No crest, no grade
#      transition, no compression at the bottom of a dip. The transition ONTO a
#      slope is a plausible place for a balancer to fail and rotated gravity
#      cannot express it at all.
#
# Here gravity stays vertical and the ground tilts, so the two models are
# genuinely independent routes to the same answer and can be cross-checked
# against each other. `test_plant_terrain.py` does exactly that on the flat and
# at a mid grade. If they ever disagree, that disagreement is a finding.
#
# SIGN, MEASURED NOT ASSERTED
# ---------------------------
# Forward is -X. `grade_pct > 0` means DOWNHILL IS FORWARD, matching hill.py.
# That requires a rotation of -phi about +Y, giving a ground normal of
# (-sin phi, 0, cos phi). The opposite sign puts the hill the other way up, and
# the failure mode of getting it wrong is a scenario that quietly measures a
# climb while its filename says descent. Asserted against actual rolling
# direction in `test_positive_grade_rolls_forward_downhill`, not derived and
# trusted -- this repo has already shipped one frame convention that was
# "derived" and wrong.

_GROUND_GEOM = '<geom name="ground" type="plane" size="20 20 0.1" material="ground_mat"/>'
_FRAME_BODY = '<body name="frame" pos="0 0 0.1454">'


def slope_rad(grade_pct: float) -> float:
    """Slope angle from a percentage grade. `grade_pct = 100 * rise / run`."""
    import math

    return math.atan(grade_pct / 100.0)


def slope_normal(grade_pct: float):
    """Unit normal of the tilted ground plane, world frame.

    `(-sin phi, 0, cos phi)`. Verified against MuJoCo's compiled `geom_xmat`
    rather than assumed -- see `test_slope_normal_matches_the_compiled_model`.
    """
    import math

    import numpy as np

    phi = slope_rad(grade_pct)
    return np.array([-math.sin(phi), 0.0, math.cos(phi)])


def tilt_ground(xml: str, grade_pct: float) -> str:
    """Tilt the ground plane to `grade_pct` and stand the board back on it.

    Two edits, and the second is the one that is easy to forget. Tilting the
    plane alone leaves the axle at its flat-ground height, which is *inside*
    the slope: the perpendicular distance from the axle to a plane through the
    origin is `h*cos(phi)`, i.e. short of the rolling radius by `h*(1-cos phi)`.
    MuJoCo would resolve that as a penetration and spit the board out on the
    first step. The axle therefore rises to `r / cos(phi)`, which puts it
    exactly `r` from the tilted surface.

    Both replacements are checked, not trusted. A silent no-op here produces a
    model that looks tilted in the XML and behaves flat, which is far worse
    than an exception -- it is the same failure class as a strip that does not
    strip.
    """
    import math

    if grade_pct == 0.0:
        return xml

    phi_deg = math.degrees(slope_rad(grade_pct))
    tilted = _GROUND_GEOM.replace(
        'size="20 20 0.1"', f'size="20 20 0.1" euler="0 {-phi_deg:.9g} 0"'
    )
    out, n = xml.replace(_GROUND_GEOM, tilted), xml.count(_GROUND_GEOM)
    if n != 1:
        raise RuntimeError(
            f"expected exactly one ground plane geom to tilt, found {n}. The "
            "markup in overboard_onewheel.xml changed; update _GROUND_GEOM."
        )

    if out.count(_FRAME_BODY) != 1:
        raise RuntimeError(
            "could not find the frame body to raise onto the slope. Tilting the "
            "ground without raising the axle buries the wheel in the plane."
        )
    r = float(_FRAME_BODY.split('pos="0 0 ')[1].rstrip('">'))
    raised = _FRAME_BODY.replace(
        f'pos="0 0 {r}"', f'pos="0 0 {r / math.cos(slope_rad(grade_pct)):.9g}"'
    )
    return out.replace(_FRAME_BODY, raised)


def strike_angles_deg(model, grade_pct: float = 0.0) -> dict:
    """Pitch at which each bumper first reaches the ground, from geometry.

    Returns nose and tail SEPARATELY, in world nose-up-positive radians-as-
    degrees (ICD 10.1), plus the relative angle they share.

    WHY SEPARATELY. On the flat the two are symmetric -- both bumpers reach the
    ground at 18.57 deg, in opposite directions -- so one number was enough and
    `nose_strike_angle_deg` collapses them with a min(). On a slope they are
    NOT symmetric, because the board sits nose-down by the slope angle before
    it has pitched at all:

        board resting flat on the slope  =>  world pitch = -phi
        nose reaches the ground at        =>  world pitch = -(rel + phi)
        tail reaches the ground at        =>  world pitch = +(rel - phi)

    So a DESCENT (grade > 0) brings the tail strike closer and pushes the nose
    strike away; a CLIMB does the reverse. That is exactly the asymmetry the
    hill gate measured -- descents fail tail-down, climbs fail nose-down -- and
    it falls out of the geometry rather than having to be discovered per grade.

    The angle is derived from the collision hull's vertices against the tilted
    plane, so it stays correct if the meshes, the tire radius or the grade
    change. The critical vertex is NOT the bumper tip: the bumper curves up
    toward its nose, so the underside heel lands first, ~90 mm inboard.
    """
    import math

    import mujoco
    import numpy as np

    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "frame")
    axle = np.array(data.xpos[body], dtype=float)
    n = slope_normal(grade_pct)

    out = {}
    for name, key in (("front_bumper_geom", "nose"), ("rear_bumper_geom", "tail")):
        gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        mesh = model.geom_dataid[gid]
        adr, num = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
        verts = model.mesh_vert[adr : adr + num].astype(float)
        # hull vertices in frame-body coordinates, axle at the origin
        v = data.geom_xpos[gid] + verts @ data.geom_xmat[gid].reshape(3, 3).T - axle

        # Sweep world pitch. Nose-up-positive about +Y (ICD 10.1): a rotation of
        # +theta maps the body -x axis (the nose) toward +z.
        hit = None
        for deg in np.arange(0.0, 89.0, 0.01):
            th = math.radians(deg) * (-1.0 if key == "nose" else 1.0)
            c, s = math.cos(th), math.sin(th)
            R = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
            if float(((axle + v @ R.T) @ n).min()) <= 0.0:
                hit = math.copysign(deg, th)
                break
        out[key] = hit
    out["relative_deg"] = (
        None if out["tail"] is None else out["tail"] + math.degrees(slope_rad(grade_pct))
    )
    return out


#: Rolling-resistance coefficient. NOT re-derived here -- the centre of the
#: 0.015-0.025 working band for an 18 psi wide pneumatic tyre, cited and
#: derived in full in either model file's own `wheel_hinge` comment
#: (`overboard_onewheel.xml`, `overboard_rider.xml`). Kept here only so the
#: welded-ballast correction below can multiply it against THIS model's own
#: compiled load, rather than by trusting whichever load the XML literal it
#: inherited happened to be sized for.
CRR = 0.02


def correct_wheel_drag_for_load(model) -> float:
    """Re-derive `wheel_hinge`'s rolling-resistance `frictionloss` from what
    the COMPILED model actually weighs, and write it back.

    THE CONFOUND THIS CLOSES. `overboard_onewheel.xml` and
    `overboard_rider.xml` each carry a `frictionloss` literal sized for THEIR
    OWN total mass (12.5 kg -> 0.3565 N*m; 83.0 kg -> 2.368 N*m -- see either
    file's `wheel_hinge` comment for `frictionloss = Crr * W * r`). But
    `build_model`/`build_terrain_model` weld a rider-scale ballast onto the
    DRIVERLESS xml, so the model they hand back inherited the driverless
    12.5 kg literal regardless of how much mass was then bolted on top -- an
    83 kg welded-ballast run was carrying a rolling-resistance term sized for
    12.5 kg, understating it ~6.6x, and nothing in that path ever read the
    number back off the model it had actually produced to notice.

    This reads the load THE SAME WAY `tests/test_rider_drag_model.py` reads
    it to verify the two model files' own literals -- by body/joint/geom
    NAME off the compiled model, never a second literal -- so it can never
    silently drift the way the original constant did. Called
    UNCONDITIONALLY, ballast or none: the driverless case must re-derive to
    the SAME 0.3565 N*m the XML already carries, not merely be left alone,
    because "skip this on the model with no ballast" is exactly the kind of
    implicit assumption that let the confound in unnoticed the first time.
    """
    import mujoco

    frame_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "frame")
    if frame_id < 0:
        raise KeyError("model has no body named 'frame'")
    # `frame` is the one body directly under `worldbody` in both model files,
    # and everything else (wheel, ballast, ballast carrier) is its kinematic
    # descendant -- so its subtree mass IS the model's total supported mass,
    # by construction rather than by a second, separately-summed literal.
    total_mass_kg = float(model.body_subtreemass[frame_id])

    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "wheel_hinge")
    if jid < 0:
        raise KeyError("model has no joint named 'wheel_hinge'")
    dof = int(model.jnt_dofadr[jid])

    wheel_gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "wheel_geom")
    if wheel_gid < 0:
        raise KeyError("model has no geom named 'wheel_geom'")
    r = float(model.geom_size[wheel_gid][0])

    import numpy as np

    g = float(np.linalg.norm(model.opt.gravity))
    frictionloss = CRR * total_mass_kg * g * r
    model.dof_frictionloss[dof] = frictionloss
    return frictionloss


#: Rider aerodynamic drag, reused VERBATIM from `overboard_rider.xml`'s own
#: `rider_aero_geom` -- same box, same position, same (MuJoCo default, left
#: unspecified) fluid coefficient -- so a welded-ballast Python run gets the
#: SAME measured effective CdA (0.754 m^2, see
#: `tests/test_rider_drag_model.py::test_rider_aero_geom_achieves_the_targeted_cda`)
#: rather than a second, independently-derived number that could drift from
#: it. See `add_rider_aero`'s docstring for the mechanism and the MuJoCo trap
#: it has to null.
_RIDER_AERO_GEOM = (
    '<geom name="rider_aero_geom" type="box" size="0.15 0.3 0.8" pos="0 0 0.9" '
    'contype="0" conaffinity="0" group="1" rgba="0 0 0 0" fluidshape="ellipsoid"/>'
)

#: A null override: zero fluid coefficients on an explicit ellipsoid shape.
#: See `add_rider_aero`'s docstring for why anything at all is needed here.
_NULL_FLUID_OVERRIDE = 'fluidshape="ellipsoid" fluidcoef="0 0 0 0 0"'

#: A tiny, invisible, non-colliding stub -- for a ballast body whose own
#: geoms (`rider_geoms()`'s "figure"/"sphere" styles) carry no fluid override
#: of their own, and which therefore needs exactly one to null the legacy
#: per-body drag `add_rider_aero`'s `<option density>` would otherwise derive
#: from the ballast's own (rider-scale) mass and inertia. `size="0.001"` and
#: an explicit `<inertial>` on the ballast body it is added to means it
#: perturbs neither the render nor the compiled mass -- same reasoning as
#: `overboard_rider.xml`'s own `ballast_fa_carrier_fluid_null_geom`.
BALLAST_FLUID_NULL_GEOM = (
    '<geom name="ballast_fluid_null_geom" type="sphere" size="0.001" '
    f'contype="0" conaffinity="0" group="1" rgba="0 0 0 0" {_NULL_FLUID_OVERRIDE}/>'
)

#: The driverless model's own `<option>` line, matched verbatim so enabling
#: the fluid model is a targeted edit rather than a blind string splice.
_ONEWHEEL_OPTION = '<option gravity="0 0 -9.81" timestep="0.002" integrator="RK4"/>'

#: The tail of the driverless model's own `wheel_geom`, matched verbatim for
#: the same reason.
_ONEWHEEL_WHEEL_GEOM_TAIL = 'material="tire_mat" condim="4"/>'


def add_rider_aero(xml: str) -> str:
    """Give a welded-ballast driverless model the rider aerodynamic drag term
    it is otherwise missing ENTIRELY -- call only once ballast mass has
    actually been welded on; never on the bare driverless model, which
    `overboard_onewheel.xml`'s own header note says must carry none.

    THE GAP THIS CLOSES. `overboard_rider.xml` derives a rider-silhouette
    aero term (CdA 0.754 m^2) because a standing rider is most of a
    onewheel's frontal area. The Python scenarios that weld ballast onto
    `overboard_onewheel.xml` instead (`build_model`, `terrain.build_terrain_
    model`) carry rider-SCALE MASS but no aero drag whatever -- their
    high-speed results were wrong in the opposite direction from the
    frictionloss confound: too LITTLE drag, not too much.

    THE MUJOCO TRAP (see `overboard_rider.xml`'s own `<option>` comment, and
    this module's docstring for the empirical check that motivated it):
    turning on `density` activates MuJoCo's LEGACY per-body fluid model on
    every body that does not carry an explicit `fluidshape="ellipsoid"`
    override, INCLUDING bodies with no rendered geom at all. `wheel_geom` and
    a welded ballast body's own geoms carry no such override, so both get one
    here -- the wheel's on its existing geom, the ballast's via
    `BALLAST_FLUID_NULL_GEOM`, which the CALLER must splice into the ballast
    body it is building (this function cannot see that body's markup, since
    it is assembled by the caller, not here).

    Verified empirically (not by inspection) that a single override geom on
    a body suppresses the WHOLE body's legacy contribution regardless of how
    many other, non-overridden geoms that body also carries -- the same
    property `overboard_rider.xml`'s own null overrides rely on.
    """
    if xml.count(_ONEWHEEL_OPTION) != 1:
        raise RuntimeError(
            "expected exactly one <option> line matching the driverless "
            "model's own to enable the fluid model on -- the markup in "
            "overboard_onewheel.xml changed; update _ONEWHEEL_OPTION."
        )
    xml = xml.replace(
        _ONEWHEEL_OPTION,
        _ONEWHEEL_OPTION.replace("/>", ' density="1.225" viscosity="0"/>'),
        1,
    )

    if xml.count(_ONEWHEEL_WHEEL_GEOM_TAIL) != 1:
        raise RuntimeError(
            "expected exactly one wheel_geom to null-override -- the markup "
            "in overboard_onewheel.xml changed; update "
            "_ONEWHEEL_WHEEL_GEOM_TAIL."
        )
    xml = xml.replace(
        _ONEWHEEL_WHEEL_GEOM_TAIL,
        _ONEWHEEL_WHEEL_GEOM_TAIL.replace("/>", f" {_NULL_FLUID_OVERRIDE}/>"),
        1,
    )

    if xml.count('      <site name="imu"') != 1:
        raise RuntimeError(
            "expected exactly one imu site to anchor the aero geom before -- "
            "the markup in overboard_onewheel.xml changed."
        )
    return xml.replace(
        '      <site name="imu"', _RIDER_AERO_GEOM + '\n      <site name="imu"', 1
    )


def build_model(ballast_mass: float, ballast_height: float, clamp_a: float,
                rider_style: str = "figure", grade_pct: float = 0.0):
    """The stock model, optionally with a rigid rider-proxy mass above the axle.

    A rigid ballast is NOT a rider: a real one is compliant at the ankles and
    knees, which changes the dynamics substantially. It is the honest
    order-of-magnitude stand-in for "what happens when the centre of mass moves
    above the axle", which is the property that matters here.

    `grade_pct` tilts the GROUND and leaves gravity vertical (see `tilt_ground`).
    It defaults to 0.0, so every existing caller is unaffected. This is the
    filmable hill and the one that can later carry a crest or a grade
    transition; `hill.py`'s rotated-gravity model remains the reference for the
    infinite uniform plane, and the two are cross-checked against each other.
    """
    import mujoco

    xml = tilt_ground(MODEL_PATH.read_text(), grade_pct)
    if clamp_a is not None:
        lim = clamp_a * KT_NM_PER_A
        xml = xml.replace('ctrlrange="-28 28"', f'ctrlrange="{-lim:g} {lim:g}"')
    if ballast_mass > 0:
        # The geom is VISUAL ONLY -- contype/conaffinity 0 so it collides with
        # nothing, and the explicit <inertial> means it contributes no mass of
        # its own. Without it the ballast is invisible in the render, and an
        # archived clip would show a bare board while 70 kg sits above the
        # axle, which is exactly the kind of quietly misleading artifact this
        # archive exists to avoid.
        body = (
            f'\n      <body name="ballast" pos="0 0 {ballast_height}">'
            f'<inertial pos="0 0 0" mass="{ballast_mass}" '
            f'diaginertia="{ballast_mass * 0.15:.4f} {ballast_mass * 0.15:.4f} '
            f'{ballast_mass * 0.08:.4f}"/>'
            + rider_geoms(rider_style, ballast_height)
            + BALLAST_FLUID_NULL_GEOM
            + f'</body>\n'
        )
        xml = xml.replace('      <site name="imu"', body + '      <site name="imu"', 1)
        # Widen the field of view. The cameras are framed for a 0.3 m-tall
        # board; a ballast 0.75 m above the axle falls outside them, and a clip
        # that crops out the mass whose whole point is being there is worse
        # than no clip.
        # Swept, not derived. 58 framed the figure well in a bare render but
        # the pane's label bar then clipped the helmet, so this carries the
        # extra headroom the overlay eats.
        xml = xml.replace('fovy="26"', 'fovy="66"').replace('fovy="24"', 'fovy="64"')

        # RIDER AERO -- a welded-ballast run carries rider-scale MASS and must
        # therefore also carry rider-scale aerodynamic drag; see
        # `add_rider_aero`'s docstring for the gap this closes and the MuJoCo
        # trap it has to null. Gated on ballast_mass > 0, same as the ballast
        # body itself: the driverless model's own header note says it must
        # carry no aero whatever, and this must not be the thing that adds it.
        xml = add_rider_aero(xml)

    # A WORLD-FIXED camera, for scenarios about travel.
    #
    # Both stock cameras are `trackcom`: they follow the board, which is right
    # for a disturbance clip where the subject is the attitude. It is wrong for
    # the Shuttle Run, where the subject is the board MOVING -- tracked, a 6 m
    # round trip looks like a stationary board and the motion has to be read off
    # a number instead of seen.
    #
    # Framed to hold roughly -3..+3 m of travel from a fixed vantage.
    wide = (
        '\n    <camera name="wide" pos="0 6.3 2.2" '
        'xyaxes="-1 0 0 0 -0.258 0.966" fovy="34"/>\n  '
    )
    xml = xml.replace("</worldbody>", wide + "</worldbody>", 1)

    # Load from a string with the meshes supplied in-memory, rather than
    # writing a temporary MJCF next to the real one. A temp file in the model
    # directory has to be cleaned up on every exit path, and a stray one is
    # indistinguishable from a real model to the next person who looks.
    mesh_dir = MODEL_PATH.parent / "meshes" / "openwheel"
    assets = {p.name: p.read_bytes() for p in mesh_dir.glob("*.stl")}
    model = mujoco.MjModel.from_xml_string(xml, assets)
    # Re-derive wheel_hinge's rolling resistance from what THIS compiled
    # model actually weighs -- see `correct_wheel_drag_for_load`'s docstring
    # for the confound this closes. Unconditional: the driverless case (no
    # ballast) must derive to the SAME 0.3565 N*m the XML literal carries,
    # not merely be left alone.
    correct_wheel_drag_for_load(model)
    return model


def plant_summary(model) -> dict:
    """The two numbers that decide what kind of plant this is."""
    import mujoco

    d = mujoco.MjData(model)
    mujoco.mj_forward(model, d)
    axle_z = float(d.xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "frame")][2])
    total = float(sum(model.body_mass))
    com_z = float(sum(model.body_mass[i] * d.xipos[i][2] for i in range(model.nbody)) / total)
    l = com_z - axle_z
    return {
        "total_mass_kg": round(total, 2),
        "com_above_axle_mm": round(l * 1000.0, 1),
        # Positive mgl means gravity DESTABILISES -- an inverted pendulum.
        # Negative means it restores. The sign is the plant's character.
        "mgl_n_m_per_rad": round(total * 9.81 * l, 2),
        "inverted_pendulum": l > 0,
    }




def imu_readings(model, data) -> tuple:
    """Gyro and specific force at the IMU site, **converted to the ICD frame**.

    Looked up by sensor NAME rather than a hardcoded offset into `sensordata`:
    the sensor block has been reordered once already, and an index quietly
    pointing at the wrong sensor would hand the estimator plausible garbage.

    THE MODEL'S BODY FRAME IS NOT THE ICD'S. The model is **z-up with forward
    = −X** (see the FORWARD note in overboard_onewheel.xml); ICD §10.1 mandates
    **FRD, right-handed**: x forward, y right, z down. Feeding a raw sensor to
    an estimator that assumes FRD is about as bad as a sign error gets on a
    balancer.

    The map is therefore a **180° rotation about +Y**, `diag(−1, +1, −1)` —
    derived from the two frame definitions, not fitted. It is written as a
    matrix rather than a per-axis sign triple so that its determinant can be
    asserted: a frame change is a PROPER rotation, `det = +1`. A sign triple
    has nowhere to check handedness, which is how the previous value got in.

    HISTORY, because this cost a real result. This used to be
    `diag(+1, +1, −1)`, `det = −1` — a reflection, mapping a right-handed frame
    to a left-handed one, which ICD §10.1 forbids in as many words. It was
    "determined EMPIRICALLY against framequat truth" on quiet samples and
    reported ~2° RMS. That residual was not real acceleration; it was the frame
    error itself. The error vanishes at θ = 0 and grows as 2θ, so validating
    near upright is validating in the one regime where the bug is invisible —
    undisturbed estimator error is bit-identical under both maps. A test named
    `test_imu_frame_matches_truth` was cited as pinning this and was never
    written. See tests/test_frame_conformance.py, which does.

    Two channels were wrong, one root cause. Accel-derived pitch came out at
    exactly −θ, so the estimator fused a nose-up-positive gyro against a
    nose-down-positive accelerometer. `gyro[0]` — roll rate — was also
    inverted, and silently, because nothing consumes roll yet; it would have
    surfaced as a control inversion the day lean-to-steer was wired up.

    `gyro[1]` is the nose-up pitch rate and is the one component both maps
    agree on, which is why the pitch RATE path was always correct.
    """
    import mujoco
    import numpy as np

    out = []
    for name in ("frame_gyro", "frame_accel"):
        sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
        if sid < 0:
            raise KeyError(f"model has no sensor named {name!r}")
        adr, dim = int(model.sensor_adr[sid]), int(model.sensor_dim[sid])
        out.append(np.array(data.sensordata[adr : adr + dim], dtype=float))
    gyro, accel = out
    R = R_MODEL_TO_ICD()
    return R @ gyro, R @ accel


# ==========================================================================
# Kerb strike: what an obstacle is worth, in the currency the controller feels
# ==========================================================================
#
# Issue #142 AC2 asks this role what a kerb strike is worth, because the
# reference disturbance the #132 retune is scoped against -- 20 N*s -- is
# inherited from a scenario nominal and derived from nothing.
#
# WHAT IS DERIVED HERE, and what is deliberately not. This produces the
# impulse an obstacle delivers, from geometry and speed. It does NOT pick the
# reference disturbance: that is a duty-cycle judgement about what the board is
# for, and the honest output of the arithmetic is a RANGE plus the reason the
# range is wide. See `kerb_strike_impulse.__doc__` for the model and
# `KERB_STRIKE_VALIDITY` for where it stops being true.
#
# THE FIRST MODEL I WROTE WAS WRONG, and the way it was wrong is worth keeping.
# Conserving angular momentum about the step edge and letting the body rotate
# rigidly about it -- the textbook step-climb model -- gives dv = 0.25*v at a
# step height of ZERO. A zero-height step is not an obstacle, so any model that
# charges 0.25 v for it is broken, and it is broken in the direction that makes
# every number look alarming. The defect: rotation about the contact point is
# only forced when the wheel is BLOCKED. Nothing blocks it as h -> 0, and the
# wheel simply rolls on. Checking a limit that has a known answer is what
# caught it; the numbers themselves looked entirely plausible.

#: Where the rigid-wheel model stops describing reality. The sim's wheel is a
#: rigid cylinder (`sim/models/overboard_onewheel.xml`), but the real part is an
#: 11.5 in PNEUMATIC tyre, and a pneumatic tyre swallows an obstacle smaller
#: than its own deflection -- it deforms around the lip and rolls over it
#: instead of striking it. Below that height the rigid model does not
#: overestimate slightly, it describes a different event.
#:
#: **The deflection figure is an open input, not a measurement.** It wants the
#: tyre spec and a load-deflection check at riding pressure, which is this
#: role's to supply and does not exist yet. Until it does, anything derived
#: below ~20 mm is a bound, not an answer.
KERB_STRIKE_VALIDITY = {
    "tyre_deflection_mm": (10.0, 20.0),
    "assumes": "rigid wheel, impulsive contact, rider rigid through the strike",
    "excludes": "tyre compliance, suspension, rider articulation, wheel slip",
}


def kerb_strike_impulse(obstacle_height_m: float, speed_m_s: float,
                        model_params: dict | None = None) -> dict:
    """Impulse delivered by a square-edged obstacle, as a bracket.

    Two models, and the truth is between them, because how much the edge GRIPS
    is the thing neither one knows:

    * **Frictionless normal** (lower bound). The normal to a circular rim
      passes through the wheel centre, so the impulse has no moment about the
      axle: the wheel keeps spinning and the impulse enters the frame at the
      axle. Only the approach velocity along the edge->centre line is killed.
      Correct as h -> 0, where it goes to zero.
    * **Pivot** (upper bound). The edge grips completely and the body rotates
      rigidly about it. Wrong for small h -- see the module comment -- but
      right once the step is tall enough that the wheel cannot roll through.

    The two converge above roughly h/r = 0.5, which is the useful structural
    result: for a real kerb it does not matter which you believe, and for a
    pavement lip it matters enormously.

    Returns horizontal impulse (N*s), speed lost (m/s) and the pitch rate the
    strike imparts (deg/s) for each bound. **The pitch rate is the number that
    matters** -- see `kerb_strike_vs_com_impulse`.
    """
    import numpy as np

    p = {"mass_kg": 82.5, "inertia_pitch_kg_m2": 17.2374,
         "com_height_m": 0.77885, "wheel_radius_m": 0.1454,
         "wheel_inertia_kg_m2": 0.0595, "edge_friction": 1.0}
    p.update(model_params or {})
    M, Ic, zc = p["mass_kg"], p["inertia_pitch_kg_m2"], p["com_height_m"]
    r, Iw = p["wheel_radius_m"], p["wheel_inertia_kg_m2"]
    h, v = float(obstacle_height_m), float(speed_m_s)
    if not 0.0 <= h < r:
        raise ValueError(
            f"obstacle height {h} m is outside (0, r={r}). At h >= r the wheel "
            "strikes at or above its own centre: that is a wall, not a step, "
            "and no impulse model applies -- the board stops and the rider does "
            "not.")
    l = zc - r
    xE = np.sqrt(max(2.0 * r * h - h * h, 0.0))   # edge, ahead of the axle

    # -- frictionless normal ------------------------------------------------
    approach = v * xE / r
    P = approach / (1.0 / M + (l * l) * (xE * xE) / (r * r * Ic))
    j_lo = P * xE / r
    q_lo = np.degrees(P * l * xE / (r * Ic))

    # -- pivot, AND whether it is reachable ----------------------------------
    #
    # The pivot model does not just get harsh at small h, it becomes
    # UNPHYSICAL, and the discriminator is derivable rather than a matter of
    # taste. Pivoting means the edge arrests the wheel's roll, which takes a
    # tangential impulse alongside the normal one. Their ratio is the friction
    # coefficient the edge would have to supply:
    #
    #     h:        1 mm   5 mm   10 mm   20 mm   30 mm   50 mm   100 mm
    #     mu_req:   5.43   2.34    1.57    0.99    0.70    0.38     0.08
    #
    # Rubber on asphalt is mu ~ 1.0 -- the model's own `<geom friction>`. So
    # the pivot is unreachable below roughly 20 mm: the edge would slip and the
    # wheel would roll through. That is the same ~20 mm the tyre's own
    # deflection gives, arrived at independently, which is the strongest reason
    # to believe either of them.
    #
    # Below that the bracket collapses to the frictionless value. Reporting an
    # unreachable pivot as the top of a range would be exactly the thing the
    # module comment warns about: a wrong assumption dressed as a result.
    dz = zc - h
    IE = Ic + M * (xE * xE + dz * dz)
    omega = (M * v * dz + Iw * v / r) / IE
    j_hi = M * (v - omega * dz)
    q_hi = np.degrees(omega)

    Jvec = np.array([M * (omega * dz - v), M * omega * xE])   # impulse at the edge
    nhat = np.array([-xE, r - h]) / r                          # edge -> wheel centre
    Jn = float(Jvec @ nhat)
    Jt = float(np.linalg.norm(Jvec - Jn * nhat))
    mu_req = Jt / Jn if Jn > 1e-12 else float("inf")
    pivot_reachable = mu_req <= p["edge_friction"]

    # THE TWO MODELS ALSO CROSS near h/r ~ 0.6 -- as the step gets tall the
    # normal tilts steeply, so killing the approach along it costs MORE than
    # gripping and pivoting. Not a bug, but it does mean the bracket must be
    # ordered rather than assumed to be (frictionless, pivot).
    ends = (float(j_lo), float(j_hi)) if pivot_reachable else (float(j_lo),) * 2
    q_ends = (float(q_lo), float(q_hi)) if pivot_reachable else (float(q_lo),) * 2
    lo_hi = tuple(sorted(ends))
    q_lo_hi = tuple(sorted(q_ends))
    floor_mm = KERB_STRIKE_VALIDITY["tyre_deflection_mm"][1]

    return {
        "obstacle_height_m": h, "speed_m_s": v,
        "impulse_ns": lo_hi,
        "speed_lost_m_s": (lo_hi[0] / M, lo_hi[1] / M),
        "pitch_rate_deg_s": q_lo_hi,
        "pivot_reachable": bool(pivot_reachable),
        "pivot_friction_required": float(mu_req),
        "models_agree_within_pct": (
            100.0 * (lo_hi[1] - lo_hi[0]) / lo_hi[1] if lo_hi[1] > 0.0 else 0.0),
        "within_validity": floor_mm <= h * 1000.0 and h / r <= 0.6,
    }


def kerb_strike_vs_com_impulse(obstacle_height_m: float, speed_m_s: float) -> dict:
    """Why N*s is the wrong currency for comparing these two disturbances.

    `impulse_response.py` applies its 20 N*s through the CoM, deliberately and
    with a paragraph explaining why: at `application_height_m = 0.0` the
    disturbance is a PURE LINEAR impulse and the pitch response is the
    vehicle's own dynamics. Its docstring already flags the alternative --
    *"kept as a parameter for a follow-on 'shove'/curb-strike scenario; not the
    default"* -- and notes that the angular channel swamps the linear one.

    A kerb strike is that follow-on case. Its impulse lands at the contact,
    roughly 0.7 m BELOW the CoM, so it injects an angular impulse the CoM-
    applied disturbance has none of by construction. Matching the two on N*s
    matches them on the channel that matters least.

    So: **a reference disturbance quoted in N*s and fed to the existing
    scenario understates a kerb strike, and does so silently.** The comparison
    Controls needs is pitch rate imparted, not newton-seconds.
    """
    s = kerb_strike_impulse(obstacle_height_m, speed_m_s)
    lo, hi = s["pitch_rate_deg_s"]
    return {
        **s,
        "com_impulse_pitch_rate_deg_s": 0.0,
        "note": f"a CoM-applied impulse of the same {s['impulse_ns'][0]:.0f}-"
                f"{s['impulse_ns'][1]:.0f} N*s imparts ZERO initial pitch rate; "
                f"this strike imparts {lo:.0f}-{hi:.0f} deg/s. Same currency, "
                "different disturbance.",
    }
