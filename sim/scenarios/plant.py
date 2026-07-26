"""The plant: the onewheel model, optionally carrying a rider-scale mass.

Lives in `sim/scenarios/` rather than in a script because it IS the plant, and
both the experiment runner and the scenarios need it. A scenario importing the
model out of `scripts/` would have the dependency backwards.
"""

from __future__ import annotations

from pathlib import Path

MODEL_PATH = Path(__file__).resolve().parents[2] / "sim" / "models" / "overboard_onewheel.xml"

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


def build_model(ballast_mass: float, ballast_height: float, clamp_a: float,
                rider_style: str = "figure"):
    """The stock model, optionally with a rigid rider-proxy mass above the axle.

    A rigid ballast is NOT a rider: a real one is compliant at the ankles and
    knees, which changes the dynamics substantially. It is the honest
    order-of-magnitude stand-in for "what happens when the centre of mass moves
    above the axle", which is the property that matters here.
    """
    import mujoco

    xml = MODEL_PATH.read_text()
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
    return mujoco.MjModel.from_xml_string(xml, assets)


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

    THE MODEL'S BODY FRAME IS NOT THE ICD'S. MuJoCo's frame here is **z-up**;
    ICD §10.1 specifies FRD, **z-down**. Feeding the raw sensor to an estimator
    that assumes FRD gives an attitude offset by 180°, which is about as bad as
    a sign error gets on a balancer.

    The conversion is **z-negation on both vectors**, and it was determined
    EMPIRICALLY against `framequat` truth rather than derived — §10.3 makes the
    sim the arbiter for exactly this class of question, and a derivation that
    looked right had the accelerometer's x sign backwards. On quiet samples
    (specific force within 0.15 of 1 g, so gravity dominates) the converted
    reading recovers truth to about 2° RMS, the residual being real
    acceleration rather than a frame error. `test_imu_frame_matches_truth`
    pins it.

    The y axis is untouched, which is the component that matters: `gyro[1]` is
    the nose-up pitch rate in both frames.
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
    to_icd = np.array([1.0, 1.0, -1.0])
    return gyro * to_icd, accel * to_icd
