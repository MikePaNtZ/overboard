"""Impulse (disturbance) response scenario for the Overboard onewheel.

The canonical controls bring-up test: kick the board with a known impulse and
watch what it does. Open-loop (no controller) it noses into the ground;
closed-loop it should arrest the pitch and recover. Same scenario, both
regimes — which is what makes it a regression gate rather than a demo.

Design doc: "Overboard — Sim Test: Impulse Disturbance Response", GitHub #2.

THE TOPPLE CRITERION IS NOSE STRIKE, NOT A TILT ANGLE
-----------------------------------------------------
The original design called for "pitch exceeds 45 deg". That angle is not
reachable by this vehicle. With the axle held at the 145.4 mm tire radius, the
underside of the bumper reaches the ground after 18.6 deg of pitch (see
`nose_strike_angle_deg`, computed from the actual collision hull, not assumed).
The board physically cannot tilt further while upright — it noses in first.

So `nose_strike` is a real contact between a bumper geom and the ground plane.
That is the true failure mode, it falls out of the geometry rather than a tuned
constant, and 18.6 deg is the margin the balance controller actually has to
hold.

PITCH SIGN -- RESOLVED: ONE CONVENTION, THE ICD'S
-------------------------------------------------
Pitch is NOSE-UP-POSITIVE, in RADIANS at the controller seam, per BoardIo ICD
section 10.1. This module previously used nose-down-positive degrees, which
made the same physical law read `+K*pitch` here and `-K*pitch` in the ICD.

The ICD moved nothing; the sim did. Section 10 derives the convention from a
free-body argument rather than asserting it, and 10.3 names the sim as the
arbiter of the polarity gate -- so a sim that reports the opposite sign to the
document it is supposed to arbitrate is the defect.

Two conventions across this seam is the cross-seam sign error the ICD calls the
most dangerous bug in the system, and it has already caught this project once
(the v0.2 polarity gate was inverted). Converting only at the seam and letting
both live in one repo was considered and rejected.

DEGREES EXIST ONLY IN METRICS. The seam is radians; `frame_pitch_deg` is a thin
wrapper for human-readable output and the JSON. Do not widen its use.

The ACTUATOR sign was already reconciled: the wheel hinge sits on -Y so that
`amps > 0 => forward => nose up` per ICD 7.3, asserted by
test_motor_sign_matches_icd.

CURRENT IS AMPS; THE MODEL WANTS NEWTON-METRES
-----------------------------------------------
The MJCF actuator is a `motor` with `gear="1"`, so `data.ctrl` is TORQUE in
N*m. This scenario commands CURRENT in amps, because that is what the ICD and
the VESC speak. The conversion is `KT_NM_PER_A` below -- previously absent,
which silently baked in Kt = 1.0 N*m/A and made every torque-headroom
comparison in the design docs a comparison of two different units.
"""

from __future__ import annotations

import hashlib
from dataclasses import asdict, dataclass, field
from pathlib import Path

import mujoco
import numpy as np

from .imperfections import STAGE0_PLACEHOLDER, ImperfectionProfile, ImperfectionState

MODEL_PATH = Path(__file__).resolve().parents[2] / "sim" / "models" / "overboard_onewheel.xml"

#: Forward is -X in the Openwheel assembly frame (the front enclosure spans
#: x = -431.8..-145.4 mm). See the model header.
FORWARD = np.array([-1.0, 0.0, 0.0])

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

#: Impulse that topples the board open-loop, with margin. The knee is at
#: ~12.5 N*s, where the nose grazes the ground and the strike boolean is
#: genuinely marginal; 20 N*s sits ~60% clear of it so the CI gate is not
#: sitting on a knife edge. On 12.5 kg it is a 1.6 m/s delta-v — a firm shove.
NOMINAL_IMPULSE_NS = 20.0

#: Impulse that must NOT topple the board. Peaks around half the strike angle,
#: so the negative case has as much margin as the positive one. Together the
#: two prove the impulse is what causes the topple.
SUBTHRESHOLD_IMPULSE_NS = 6.0


@dataclass(frozen=True)
class ImpulseParams:
    """Everything that defines one run. Frozen: the metrics are only
    meaningful alongside the exact parameters that produced them."""

    magnitude_ns: float = NOMINAL_IMPULSE_NS
    """Impulse magnitude in N*s (force x duration)."""

    t0_s: float = 0.5
    """When the kick fires. Late enough that the board is provably at rest."""

    duration_s: float = 0.05
    """Held-force window. Short relative to the ~1.6 s pitch mode, so the
    result depends on the impulse and not on the force profile."""

    direction: tuple[float, float, float] = (-1.0, 0.0, 0.0)
    """Unit direction of the push. Forward (-X) to start; lateral is follow-on."""

    application_height_m: float = 0.0
    """Height above the frame's centre of mass at which the push lands.

    Defaults to 0.0 — the push acts through the CoM, so the disturbance is a
    pure LINEAR impulse and the pitch response is produced by the vehicle's own
    dynamics (wheel/frame coupling through the hinge, reacted at the contact
    patch). That is the standard disturbance-rejection formulation and the one
    worth characterising a controller against.

    Raising it also injects an ANGULAR impulse of r x J, and that channel
    swamps the linear one: at deck height (0.15 m) a 20 N*s push adds 3 N*m*s
    about a frame pitch inertia of only 0.40 kg*m^2 — a 430 deg/s kick that
    slams the nose down in a quarter of a metre, before the vehicle dynamics
    get to express themselves at all. The outcome then depends almost entirely
    on the lever arm, which is an arbitrary modelling choice. Kept as a
    parameter for a follow-on "shove"/curb-strike scenario; not the default."""

    sim_seconds: float = 8.0
    """Long enough to capture the strike and the settle behind it."""

    settle_band_deg: float = 2.0
    """Pitch band the board must stay inside to count as settled."""

    # Actuation delay used to live here as a whole-cycle count. It now belongs
    # to the imperfection profile (ICD 12), in seconds, alongside the rest of
    # the signal-path imperfections -- two places to configure the same physics
    # is how they end up disagreeing.


@dataclass
class ImpulseMetrics:
    """Scalar outcomes of one run. Serialised to metrics.json for CI and for
    the landing-page caption."""

    # --- the gate ---
    nose_strike: bool = False
    """The topple criterion: a bumper hull actually touched the ground. Named
    for what it is — open-loop the board scrapes its nose in and rolls on, it
    does not cartwheel, and calling that "toppled" would overstate it."""

    t_strike_s: float | None = None
    nose_strike_angle_deg: float = 0.0

    # --- disturbance response ---
    peak_abs_pitch_deg: float = 0.0
    """Largest absolute pitch excursion, either direction. Absolute because a
    controller overshoots past level and a one-sided maximum would miss it."""

    t_peak_s: float = 0.0
    pitch_rate_at_strike_dps: float | None = None
    speed_at_strike_ms: float | None = None
    max_penetration_mm: float = 0.0

    # --- closed-loop metrics (populated once a controller is wired in) ---
    settle_time_s: float | None = None
    steady_state_pitch_deg: float | None = None
    control_effort_a_s: float = 0.0

    # --- provenance ---
    imperfection_profile_id: str = ""
    travel_m: float = 0.0
    final_pitch_deg: float = 0.0
    model_sha256: str = ""
    mujoco_version: str = ""
    timestep_s: float = 0.0


@dataclass
class ImpulseResult:
    params: ImpulseParams
    metrics: ImpulseMetrics
    t: np.ndarray = field(default_factory=lambda: np.empty(0))
    pitch_deg: np.ndarray = field(default_factory=lambda: np.empty(0))
    pitch_rate_dps: np.ndarray = field(default_factory=lambda: np.empty(0))
    wheel_rate_rads: np.ndarray = field(default_factory=lambda: np.empty(0))
    travel_m: np.ndarray = field(default_factory=lambda: np.empty(0))
    motor_current_a: np.ndarray = field(default_factory=lambda: np.empty(0))

    qpos: np.ndarray = field(default_factory=lambda: np.empty(0))
    """Full state history, one row per step, when run(capture_state=True).

    The renderer REPLAYS this rather than re-stepping the physics, so the film
    is guaranteed to be of the same trajectory the metrics describe. It also
    keeps GL out of the physics path entirely: the CI gate needs no graphics
    stack, and a broken one can never change a pass/fail.
    """

    def to_json_dict(self) -> dict:
        return {"params": asdict(self.params), "metrics": asdict(self.metrics)}


def load_model(path: Path = MODEL_PATH) -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_path(str(path))


def frame_pitch_rad(model: mujoco.MjModel, data: mujoco.MjData) -> float:
    """Frame pitch about the lateral axis, NOSE-UP POSITIVE, radians (ICD 10.1).

    Read off the frame's world z-axis rather than the quaternion so the sign
    convention is inspectable rather than asserted.

    Derivation, given forward is -X (see FORWARD): a rotation of +phi about +Y
    maps the body -x axis -- the nose -- to (-cos phi, 0, +sin phi), whose z
    component is positive. So +phi lifts the nose. The same rotation maps the
    body z axis to (sin phi, 0, cos phi), giving R[0,2] = sin phi and
    R[2,2] = cos phi, hence atan2(R[0,2], R[2,2]) = phi.
    """
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "frame")
    R = data.xmat[body].reshape(3, 3)
    return float(np.arctan2(R[0, 2], R[2, 2]))


def frame_pitch_deg(model: mujoco.MjModel, data: mujoco.MjData) -> float:
    """Nose-up-positive pitch in degrees. **Metrics and humans only** -- the
    controller seam is radians (see the module docstring)."""
    return float(np.degrees(frame_pitch_rad(model, data)))


def nose_strike_angle_deg(model: mujoco.MjModel) -> float:
    """Pitch at which the bumper hull first reaches the ground, from geometry.

    Derived from the collision hull's vertices rather than assumed, so it stays
    correct if the meshes or the tire radius change. The critical vertex is NOT
    the bumper tip — the bumper curves up toward its nose, so the underside
    heel (x = -381 mm) lands first, ~90 mm inboard of the tip.
    """
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "frame")
    axle_h = float(data.xpos[body][2])

    best = np.inf
    for name in ("front_bumper_geom", "rear_bumper_geom"):
        gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        mesh = model.geom_dataid[gid]
        adr, num = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
        verts = model.mesh_vert[adr : adr + num].astype(float)
        # hull vertices in frame-body coordinates, axle at the origin
        p = data.geom_xpos[gid] + verts @ data.geom_xmat[gid].reshape(3, 3).T - data.xpos[body]
        x, z = p[:, 0], p[:, 2]
        # world height of each vertex at nose-down pitch a: h + x*sin(a) + z*cos(a)
        for deg in np.arange(0.0, 90.0, 0.01):
            a = np.radians(deg)
            if float((axle_h + x * np.sin(a) + z * np.cos(a)).min()) <= 0.0:
                best = min(best, deg)
                break
    return float(best)


def _bumper_ground_contact(model, data, ground_id, bumper_ids) -> float | None:
    """Deepest bumper<->ground penetration this step, or None if not touching."""
    depth = None
    for i in range(data.ncon):
        c = data.contact[i]
        hit = (c.geom1 == ground_id and c.geom2 in bumper_ids) or (
            c.geom2 == ground_id and c.geom1 in bumper_ids
        )
        if hit:
            depth = c.dist if depth is None else min(depth, c.dist)
    return depth


def run(
    params: ImpulseParams | None = None,
    model: mujoco.MjModel | None = None,
    controller=None,
    capture_state: bool = False,
    profile: ImperfectionProfile = STAGE0_PLACEHOLDER,
) -> ImpulseResult:
    """Run one impulse scenario.

    `controller` is the closed-loop hook:

        (t_s, pitch_rad, pitch_rate_rad_s, wheel_rate_rad_s) -> amps

    **Radians, nose-up-positive, per ICD 10.1** -- not degrees, and not the
    old nose-down convention. The return is CURRENT in amps; the scenario
    converts to torque via KT_NM_PER_A, and the actuator's ctrlrange bounds the
    result. The command takes effect `actuation_delay_cycles` steps later.

    Left None the scenario is open-loop -- the baseline the controller has to
    beat.

    Fully deterministic: fixed timestep, no stochastic terms, no wall clock.
    Two calls with equal params return bit-identical trajectories.
    """
    params = params or ImpulseParams()
    model = model or load_model()
    data = mujoco.MjData(model)

    frame = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "frame")
    ground = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "ground")
    bumpers = {
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, n)
        for n in ("front_bumper_geom", "rear_bumper_geom")
    }

    dt = model.opt.timestep
    n_steps = int(round(params.sim_seconds / dt))
    force = np.array(params.direction, dtype=float) * (params.magnitude_ns / params.duration_s)
    # A push landing `application_height_m` above the CoM also applies a torque
    # about it. r x F, with r purely vertical in the world frame.
    torque = np.cross(np.array([0.0, 0.0, params.application_height_m]), force)

    m = ImpulseMetrics(
        nose_strike_angle_deg=nose_strike_angle_deg(model),
        model_sha256=hashlib.sha256(MODEL_PATH.read_bytes()).hexdigest()[:16],
        mujoco_version=mujoco.__version__,
        timestep_s=float(dt),
    )

    ts, pitches, rates, wheel, travel, currents = [], [], [], [], [], []
    states: list[np.ndarray] = []
    x0 = float(data.xpos[frame][0])

    # The signal path between the plant and the controller: actuation delay,
    # current-loop lag, gyro noise, wheel-rate quantisation (ICD 12).
    imp = ImperfectionState(profile=profile, dt_s=dt)
    m.imperfection_profile_id = profile.profile_id

    for _ in range(n_steps):
        data.xfrc_applied[frame] = 0.0
        if params.t0_s <= data.time < params.t0_s + params.duration_s:
            data.xfrc_applied[frame][:3] = force
            data.xfrc_applied[frame][3:] = torque

        # WHAT THE CONTROLLER SEES vs WHAT ACTUALLY HAPPENED. The controller is
        # handed the corrupted signals; the trajectory below records TRUTH.
        # Logging the sensor's version would make it impossible to tell a plant
        # problem from a sensing one, which is most of what these runs are for.
        pitch = frame_pitch_rad(model, data)
        true_rate = float(data.qvel[4])  # +omega_y = nose-up rate (ICD 10.1)
        sensed_rate = imp.gyro(true_rate)
        sensed_wheel = imp.wheel_rate(float(data.qvel[6]), float(data.time))

        proposed = 0.0
        if controller is not None:
            proposed = float(controller(float(data.time), pitch, sensed_rate, sensed_wheel))

        # Delay and current-loop lag live here, not in the controller.
        current = imp.apply_current(proposed)

        # ctrl is TORQUE (motor actuator, gear=1); the controller speaks amps.
        data.ctrl[0] = current * KT_NM_PER_A

        mujoco.mj_step(model, data)

        pitch = frame_pitch_rad(model, data)
        pitch_rate = float(data.qvel[4])
        fwd = float(-(data.xpos[frame][0] - x0))  # forward is -x

        # Trajectories are stored in degrees: they feed plots, the JSON and the
        # settle band, all of which are human-facing. Radians stay at the seam.
        # Both conversions are named, so nothing below can reach for the radian
        # value and store it in a field whose name ends in _deg or _dps.
        pitch_deg = float(np.degrees(pitch))
        pitch_rate_dps = float(np.degrees(pitch_rate))
        ts.append(float(data.time))
        pitches.append(pitch_deg)
        rates.append(pitch_rate_dps)
        wheel.append(float(data.qvel[6]))
        travel.append(fwd)
        currents.append(current)
        if capture_state:
            states.append(data.qpos.copy())

        m.control_effort_a_s += abs(current) * dt
        # ABSOLUTE peak. Open-loop the board only ever pitches one way, so a
        # one-sided max was adequate; a controller overshoots the other way and
        # a one-sided max would not see it -- while this is the headline
        # acceptance number.
        if abs(pitch_deg) > m.peak_abs_pitch_deg:
            m.peak_abs_pitch_deg, m.t_peak_s = abs(pitch_deg), float(data.time)

        depth = _bumper_ground_contact(model, data, ground, bumpers)
        if depth is not None:
            m.max_penetration_mm = min(m.max_penetration_mm, depth * 1000.0)
            if not m.nose_strike:
                m.nose_strike = True
                m.t_strike_s = float(data.time)
                m.pitch_rate_at_strike_dps = pitch_rate_dps
                m.speed_at_strike_ms = float(-data.qvel[0])

    t = np.asarray(ts)
    pitch_arr = np.asarray(pitches)
    m.travel_m = float(travel[-1])
    m.final_pitch_deg = float(pitch_arr[-1])

    # Settling: last moment the board leaves the band, measured after the kick.
    after = t >= params.t0_s + params.duration_s
    outside = np.abs(pitch_arr[after]) > params.settle_band_deg
    if outside.any():
        last = float(t[after][np.flatnonzero(outside)[-1]])
        m.settle_time_s = None if last >= t[-1] - dt else last - params.t0_s
    else:
        m.settle_time_s = 0.0
    m.steady_state_pitch_deg = float(np.mean(pitch_arr[t >= t[-1] - 0.5]))

    return ImpulseResult(
        params=params,
        metrics=m,
        t=t,
        pitch_deg=pitch_arr,
        pitch_rate_dps=np.asarray(rates),
        wheel_rate_rads=np.asarray(wheel),
        travel_m=np.asarray(travel),
        motor_current_a=np.asarray(currents),
        qpos=np.asarray(states) if states else np.empty(0),
    )
