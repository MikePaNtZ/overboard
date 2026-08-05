"""Rolling terrain — crest, descent, dip, ascent, crest. One continuous ride.

The scenario the uniform-grade hill could not express, and the one that looks
like riding.

WHAT THE BOARD DOES
-------------------
It starts standing at the **crest of a hill**, rides **down** the far side,
through the **dip** at the bottom, and back **up** to the **crest of the next
hill**, where the run ends. One full cosine period of ground:

    z(x) = A cos(2 pi x / L)          A = amplitude, L = crest-to-crest

    s = 0        crest      slope 0, about to tip over the edge
    s = L/4      steepest   maximum descending grade
    s = L/2      dip        slope 0 again, but curvature reversed
    s = 3L/4     steepest   maximum climbing grade
    s = L        crest      arrives, ends

WHY THIS IS A HARDER TEST THAN A CONSTANT GRADE
-----------------------------------------------
`hill.py` models a slope by rotating gravity on flat ground. That is exact for
an infinite uniform plane and it was the right first model -- it preserved the
nose-strike contact geometry bit-identically, so a strike on a hill meant what a
strike meant everywhere else. But it can only ever be *one* grade, held forever,
and it is **invisible in a render**: the board appears on flat ground,
accelerating and tipping for no reason a viewer can see.

This model tilts the actual ground, so:

1. **The grade CHANGES under the board**, continuously. The controller never
   reaches the steady state a constant grade eventually allows, and the
   estimator's slope-absorption error is chasing a moving target rather than
   converging to a constant offset.
2. **At the dip the grade REVERSES.** Descending becomes climbing through zero.
   `hill.py` measured those as two separate runs and found they fail in opposite
   directions -- descents tail-down, climbs nose-down. Here the board has to
   pass through the handover in one continuous ride, which is the case neither
   uniform run could produce.
3. **Curvature is a load the flat model has none of.** Through the dip the
   contact point is on the inside of a curve, so holding the line needs centripetal
   force the board can only get by pushing harder into the ground -- exactly when
   the drive is fastest and cutback is most likely to bind.
4. **It can be filmed.** There is a hill in frame.

The terrain is a MuJoCo heightfield generated here, in Senior Controls' own
file, by string-transforming the model XML. `sim/models/` and
`sim/scenarios/plant.py` belong to Sr. Mechanical & Systems and are not touched;
this reuses their `rider_geoms` and constants by import, so a rename over there
fails loudly here rather than silently producing flat ground.
"""

from __future__ import annotations

import hashlib
import math
from dataclasses import asdict, dataclass, field
from pathlib import Path

import mujoco
import numpy as np

from .imperfections import (
    STAGE0_CUTBACK,
    ImperfectionProfile,
    ImperfectionState,
)
from .impulse_response import KT_NM_PER_A, _bumper_ground_contact, frame_pitch_rad
from .plant import (
    _FRAME_BODY,
    _GROUND_GEOM,
    MODEL_PATH,
    imu_readings,
    plant_summary,
    rider_geoms,
    spawn_at_sprung_equilibrium,
    wheel_dof,
)
from .rust_controller import DEFAULT_R_EFF_M as R_EFF_M

#: Samples along the ride. 512 over a ~30 m ride is ~6 cm between samples --
#: fine enough that the wheel never sees a facet, coarse enough to compile fast.
HFIELD_COLS = 512


def amplitude_for_grade(max_grade_pct: float, crest_to_crest_m: float) -> float:
    """Amplitude that makes the STEEPEST point of the profile a given grade.

    `z = A cos(2 pi x / L)`, so `dz/dx = -A (2 pi / L) sin(...)` and the maximum
    magnitude of the slope is `2 pi A / L`. Parameterising by peak grade rather
    than by amplitude is deliberate: the hill gate is expressed in grades, and
    this way the two scenarios can be compared without arithmetic in between.
    """
    return (max_grade_pct / 100.0) * crest_to_crest_m / (2.0 * math.pi)


def surface_z(x: float, amplitude_m: float, crest_to_crest_m: float) -> float:
    """Ground height at world x. `cos` is even, so travelling toward -x (the
    board's forward) sees the same profile as travelling toward +x."""
    return amplitude_m * math.cos(2.0 * math.pi * x / crest_to_crest_m)


def surface_grade_pct(x: float, amplitude_m: float, crest_to_crest_m: float) -> float:
    """Local grade at world x, SIGNED THE WAY `hill.py` SIGNS IT.

    Forward is -x. Positive means downhill-is-forward, matching
    `hill.gravity_for_grade`, so a number from here can be read against the hill
    envelope directly.
    """
    k = 2.0 * math.pi / crest_to_crest_m
    dz_dx = -amplitude_m * k * math.sin(k * x)
    # ds = -dx when moving forward, so dz/ds = -dz/dx; downhill-forward is
    # dz/ds < 0, which we report as POSITIVE grade.
    return 100.0 * dz_dx


@dataclass(frozen=True)
class TerrainParams:
    #: Grade at the steepest point of the profile, percent.
    #:
    #: 8% is the steepest profile the board completes on the attitude ESTIMATE.
    #: It fails at 10%, which the uniform-grade gate passes as a steady descent
    #: -- that gap is the headline result of this scenario, so the default sits
    #: deliberately at the edge rather than comfortably inside it. A default
    #: that always passed would be a clip, not a gate.
    max_grade_pct: float = 8.0
    #: Crest to crest, metres. One full period: crest, dip, crest.
    crest_to_crest_m: float = 24.0
    #: Commanded forward speed once rolling, m/s.
    v_ref_m_s: float = 2.0
    duration_s: float = 30.0
    settle_s: float = 2.0
    ramp_s: float = 2.0
    ballast_mass_kg: float = 70.0
    ballast_height_m: float = 0.75
    max_current_a: float = 40.0
    estimator_tau_s: float = 2.0
    use_estimator: bool = True
    accel_ff_current_source: int = 0
    rider_style: str = "figure"


@dataclass
class TerrainMetrics:
    # --- validity, outside the statistics ---
    survived: bool = False
    reached_next_crest: bool = False
    """Did it actually complete the ride? A board that stops in the dip has not
    done the manoeuvre, and a run that only asserts `survived` would pass."""

    # --- the ride ---
    travel_m: float = 0.0
    crest_to_crest_m: float = 0.0
    fraction_completed: float = 0.0
    t_reached_dip_s: float | None = None
    t_reached_crest_s: float | None = None

    # --- outcome ---
    nose_strike: bool = False
    struck_end: str = ""
    t_strike_s: float | None = None
    struck_phase: str = ""
    """Which part of the ride it failed on -- descent, dip, or ascent. The
    uniform-grade gate says descents fail tail-down and climbs nose-down; this
    says which one the board actually reaches first."""

    peak_abs_pitch_deg: float = 0.0
    max_grade_seen_pct: float = 0.0

    # --- speed authority ---
    held_speed: bool = False
    v_ref_m_s: float = 0.0
    max_speed_error_m_s: float = 0.0
    peak_abs_speed_m_s: float = 0.0

    # --- the estimator, per phase ---
    pitch_est_rms_deg: float = 0.0
    pitch_est_max_deg: float = 0.0
    est_rms_descent_deg: float = 0.0
    est_rms_dip_deg: float = 0.0
    est_rms_ascent_deg: float = 0.0
    est_error_at_dip_deg: float = 0.0
    """Estimator error at the moment the grade reverses. The handover is the
    case a constant-grade run cannot produce."""

    # --- drive ---
    cutback_binding_cycles: int = 0
    min_available_a: float = 0.0
    peak_abs_current_a: float = 0.0
    saturated_cycles: int = 0

    # --- provenance ---
    duration_s: float = 0.0
    model_sha256: str = ""
    mujoco_version: str = ""
    timestep_s: float = 0.0
    imperfection_profile_id: str = ""


@dataclass
class TerrainResult:
    params: TerrainParams
    metrics: TerrainMetrics
    plant: dict = field(default_factory=dict)
    t: np.ndarray = field(default_factory=lambda: np.empty(0))
    travel_m: np.ndarray = field(default_factory=lambda: np.empty(0))
    v_m_s: np.ndarray = field(default_factory=lambda: np.empty(0))
    grade_pct: np.ndarray = field(default_factory=lambda: np.empty(0))
    pitch_deg: np.ndarray = field(default_factory=lambda: np.empty(0))
    pitch_est_deg: np.ndarray = field(default_factory=lambda: np.empty(0))
    motor_current_a: np.ndarray = field(default_factory=lambda: np.empty(0))

    qpos: np.ndarray = field(default_factory=lambda: np.empty(0))
    """Full state history, one row per step, when run(capture_state=True).

    The renderer REPLAYS this rather than re-stepping the physics, so the film
    is guaranteed to be of the same trajectory the metrics describe. It also
    keeps GL out of the physics path entirely: the CI gate needs no graphics
    stack, and a broken one can never change a pass/fail.
    """

    def to_json_dict(self) -> dict:
        return {"params": asdict(self.params), "plant": self.plant,
                "metrics": asdict(self.metrics)}


def build_terrain_model(params: TerrainParams):
    """The stock model with its ground plane replaced by a rolling heightfield.

    Every replacement is COUNTED, not trusted. A silent no-op here yields a
    model that reads as terrain in the XML and behaves as a flat plane, which is
    strictly worse than an exception -- it is the same failure class as a strip
    that does not strip, and it would turn this whole scenario into an
    expensively-dressed flat-ground run.
    """
    amp = amplitude_for_grade(params.max_grade_pct, params.crest_to_crest_m)
    length = params.crest_to_crest_m

    # The board starts at x = 0 and travels toward -x, so the field must span a
    # full period in -x plus margin at both ends for settling and overrun.
    margin = 4.0
    x_lo, x_hi = -length - margin, margin
    radius_x = (x_hi - x_lo) / 2.0
    centre_x = (x_lo + x_hi) / 2.0

    xml = MODEL_PATH.read_text()

    asset = (
        f'\n    <hfield name="rolling" nrow="2" ncol="{HFIELD_COLS}" '
        f'size="{radius_x:.6f} 6.0 {2 * amp:.6f} 0.5"/>'
    )
    if xml.count("</asset>") != 1:
        raise RuntimeError("expected exactly one </asset> to extend with the hfield")
    xml = xml.replace("</asset>", asset + "\n  </asset>", 1)

    # `pos z = -amp` puts the mid-range of the field at z = 0, so the surface
    # runs between -amp and +amp and the crest sits at +amp.
    ground = (
        f'<geom name="ground" type="hfield" hfield="rolling" '
        f'pos="{centre_x:.6f} 0 {-amp:.6f}" material="ground_mat"/>'
    )
    if xml.count(_GROUND_GEOM) != 1:
        raise RuntimeError(
            f"expected exactly one ground plane geom to replace, found "
            f"{xml.count(_GROUND_GEOM)}. The markup in overboard_onewheel.xml "
            "changed; plant._GROUND_GEOM is stale."
        )
    xml = xml.replace(_GROUND_GEOM, ground, 1)

    # Stand the board on the crest. The crest is flat, so the axle sits exactly
    # one rolling radius above it -- no cos(phi) correction needed here, unlike
    # the tilted-plane case.
    if xml.count(_FRAME_BODY) != 1:
        raise RuntimeError("could not find the frame body to stand on the crest")
    r = float(_FRAME_BODY.split('pos="0 0 ')[1].rstrip('">'))
    xml = xml.replace(_FRAME_BODY, f'<body name="frame" pos="0 0 {r + amp:.6f}">', 1)

    if params.max_current_a is not None:
        lim = params.max_current_a * KT_NM_PER_A
        xml = xml.replace('ctrlrange="-28 28"', f'ctrlrange="{-lim:g} {lim:g}"')

    if params.ballast_mass_kg > 0:
        m_b, h_b = params.ballast_mass_kg, params.ballast_height_m
        body = (
            f'\n      <body name="ballast" pos="0 0 {h_b}">'
            f'<inertial pos="0 0 0" mass="{m_b}" '
            f'diaginertia="{m_b * 0.15:.4f} {m_b * 0.15:.4f} {m_b * 0.08:.4f}"/>'
            + rider_geoms(params.rider_style, h_b)
            + "</body>\n"
        )
        xml = xml.replace('      <site name="imu"', body + '      <site name="imu"', 1)

        # Widen the stock cameras, which are framed for a bare 0.3 m board and
        # crop out a rider 0.75 m above the axle.
        xml = xml.replace('fovy="26"', 'fovy="66"').replace('fovy="24"', 'fovy="64"')

    # A world-fixed camera that can hold the whole ride. The stock cameras are
    # `trackcom`: tracked, a crest-to-crest descent looks like a stationary
    # board on a scrolling texture, which is exactly the "hill you cannot see"
    # problem this scenario exists to fix. Pulled back and up to keep both
    # crests and the dip in frame.
    # Framed by trial against rendered output, not derived, and it is a
    # COMPROMISE rather than a good shot. Two constraints pull apart: seeing the
    # whole crest-to-crest profile needs distance, and at 8% a 24 m roller is
    # only 0.6 m tall, so from far enough back to hold both crests the hill
    # flattens toward a texture. Close enough for the rider to read, and the
    # profile leaves frame.
    #
    # That is physics, not framing -- the board's envelope caps the grade at
    # single digits, and a single-digit grade is a gentle hill. Set here so the
    # scenario is inspectable; **Digital Content Production owns the real shot**
    # and should not treat this as the intended composition.
    wide = (
        f'\n    <camera name="terrain" pos="{-length / 2:.3f} {length * 0.62:.3f} '
        f'{amp * 2 + 2.0:.3f}" xyaxes="-1 0 0 0 -0.22 0.98" fovy="34"/>\n  '
    )
    xml = xml.replace("</worldbody>", wide + "</worldbody>", 1)

    # Meshes supplied in memory rather than by writing a temp MJCF next to the
    # real model -- a stray temp file is indistinguishable from a real model to
    # whoever looks next. Keyed by both the bare name and the path the XML
    # actually asks for, since this string is compiled without a base directory.
    mesh_dir = MODEL_PATH.parent / "meshes" / "openwheel"
    assets = {f"meshes/openwheel/{p.name}": p.read_bytes()
              for p in mesh_dir.glob("*.stl")}
    model = mujoco.MjModel.from_xml_string(xml, assets)

    # Fill the elevation data. `data` is in [0, 1] and scales the field's
    # elevation, so (cos + 1) / 2 maps the profile onto the full range.
    adr = int(model.hfield_adr[0])
    ncol, nrow = int(model.hfield_ncol[0]), int(model.hfield_nrow[0])
    xs = np.linspace(x_lo, x_hi, ncol)
    row = (np.cos(2.0 * np.pi * xs / length) + 1.0) / 2.0
    model.hfield_data[adr:adr + nrow * ncol] = np.tile(row, nrow).astype(np.float32)

    # Verify the terrain is actually terrain. A flat field would make every
    # result in this module a flat-ground result wearing a hill's name.
    span = float(np.ptp(model.hfield_data[adr:adr + ncol]))
    if span < 0.9:
        raise RuntimeError(
            f"heightfield is nearly flat (normalised span {span:.4f}); the "
            "profile did not take"
        )
    return model, amp


def run(
    params: TerrainParams | None = None,
    profile: ImperfectionProfile = STAGE0_CUTBACK,
    controller_factory=None,
    capture_state: bool = False,
) -> TerrainResult:
    """Ride crest → dip → crest and measure whether control holds throughout."""
    params = params or TerrainParams()
    if profile.is_ideal():
        raise ValueError("refusing to gate on an ideal profile: it models nothing")

    model, amp = build_terrain_model(params)
    summary = plant_summary(model)
    if not summary["inverted_pendulum"]:
        raise ValueError(
            "rolling terrain requires a RIDDEN plant (centre of mass above the "
            f"axle). Got mgl {summary['mgl_n_m_per_rad']:+.2f}."
        )

    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    # Sit ON the tyre spring, not 4-5 mm above it -- a free-fall first sample
    # seeds the estimator with a phantom pitch. See the helper's docstring.
    spawn_at_sprung_equilibrium(model, data)
    frame = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "frame")
    ground = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "ground")
    bumper_ids = {
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, n): end
        for n, end in (("front_bumper_geom", "nose"), ("rear_bumper_geom", "tail"))
    }

    def v_ref_at(t: float) -> float:
        if t < params.settle_s:
            return 0.0
        if params.ramp_s <= 0.0:
            return params.v_ref_m_s
        return params.v_ref_m_s * min(1.0, (t - params.settle_s) / params.ramp_s)

    if controller_factory is None:
        from .rust_controller import RustController

        def controller_factory():
            return RustController(
                # 200 A/rad, 30 A/(rad/s) at kt = 0.7 N*m/A, re-denominated (#137).
                kp_nm_per_rad=140.0, kd_nm_per_rad_s=21.0,
                max_current_a=params.max_current_a,
                kp_v_rad_per_m_s=0.05, ki_v_rad_per_m=0.02,
                com_above_axle=True, v_ref_fn=v_ref_at,
                use_estimator=params.use_estimator,
                estimator_tau_s=params.estimator_tau_s,
                accel_ff_current_source=params.accel_ff_current_source,
            )

    dt = float(model.opt.timestep)
    n_steps = int(round(params.duration_s / dt))
    imp = ImperfectionState(profile=profile, dt_s=dt)
    length = params.crest_to_crest_m

    ts, travels, vs, grades, pitches, ests, currents = [], [], [], [], [], [], []
    states: list[np.ndarray] = []
    x0 = float(data.xpos[frame][0])
    flowing_a = 0.0
    m = TerrainMetrics()
    wheel_dof_idx = wheel_dof(model)

    with controller_factory() as ctl:
        for _ in range(n_steps):
            t = float(data.time)
            pitch_rad = frame_pitch_rad(model, data)
            wheel_true = float(data.qvel[wheel_dof_idx])
            true_gyro, true_accel = imu_readings(model, data)

            proposed = float(ctl(
                t, pitch_rad, imp.gyro(float(data.qvel[4])),
                imp.wheel_rate(wheel_true, t),
                gyro_rad_s=imp.gyro_vec(true_gyro),
                accel_m_s2=imp.accel_vec(true_accel),
                motor_current_a=flowing_a,
            ))

            available = profile.available_current_a(wheel_true)
            if abs(proposed) > available + 1e-9:
                m.cutback_binding_cycles += 1

            current = imp.apply_current(proposed, wheel_true)
            flowing_a = current
            data.ctrl[0] = current * KT_NM_PER_A
            mujoco.mj_step(model, data)

            x = float(data.xpos[frame][0])
            fwd = -(x - x0)
            pitch_deg = math.degrees(frame_pitch_rad(model, data))
            grade = surface_grade_pct(x, amp, length)

            ts.append(float(data.time))
            travels.append(fwd)
            vs.append(float(data.qvel[wheel_dof_idx]) * R_EFF_M)
            grades.append(grade)
            pitches.append(pitch_deg)
            ests.append(math.degrees(float(ctl.pitch_used_rad)))
            currents.append(current)
            if capture_state:
                states.append(data.qpos.copy())

            m.peak_abs_pitch_deg = max(m.peak_abs_pitch_deg, abs(pitch_deg))
            m.max_grade_seen_pct = max(m.max_grade_seen_pct, abs(grade))
            if m.t_reached_dip_s is None and fwd >= length / 2.0:
                m.t_reached_dip_s = float(data.time)
                m.est_error_at_dip_deg = ests[-1] - pitch_deg
            if m.t_reached_crest_s is None and fwd >= length:
                m.t_reached_crest_s = float(data.time)

            if _bumper_ground_contact(model, data, ground, set(bumper_ids)) is not None:
                m.nose_strike = True
                m.t_strike_s = float(data.time)
                ends = set()
                for i in range(data.ncon):
                    c = data.contact[i]
                    for a, b in ((c.geom1, c.geom2), (c.geom2, c.geom1)):
                        if a == ground and b in bumper_ids:
                            ends.add(bumper_ids[b])
                m.struck_end = ends.pop() if len(ends) == 1 else ("both" if ends else "")
                m.struck_phase = (
                    "descent" if fwd < length / 2.0 * 0.9
                    else "dip" if fwd < length / 2.0 * 1.1
                    else "ascent"
                )
                break

            if m.t_reached_crest_s is not None:
                break

        m.saturated_cycles = int(ctl.saturated_cycles)

    t_arr = np.asarray(ts)
    trav = np.asarray(travels)
    v_arr = np.asarray(vs)
    err = np.asarray(ests) - np.asarray(pitches)

    m.survived = not m.nose_strike
    m.reached_next_crest = m.t_reached_crest_s is not None
    m.travel_m = float(trav[-1])
    m.crest_to_crest_m = length
    m.fraction_completed = float(min(1.0, trav[-1] / length))

    cruise = t_arr >= params.settle_s + params.ramp_s
    if not cruise.any():
        cruise = np.zeros_like(t_arr, dtype=bool)
        cruise[-1] = True
    m.v_ref_m_s = params.v_ref_m_s
    m.peak_abs_speed_m_s = float(np.max(np.abs(v_arr[cruise])))
    m.max_speed_error_m_s = float(np.max(np.abs(v_arr[cruise]) - abs(params.v_ref_m_s)))
    m.held_speed = m.survived and m.max_speed_error_m_s < max(1.0, params.v_ref_m_s)

    m.pitch_est_rms_deg = float(np.sqrt(np.mean(err ** 2)))
    m.pitch_est_max_deg = float(np.max(np.abs(err)))

    def phase_rms(lo, hi):
        sel = (trav >= lo) & (trav < hi)
        return float(np.sqrt(np.mean(err[sel] ** 2))) if sel.any() else 0.0

    m.est_rms_descent_deg = phase_rms(0.0, length * 0.45)
    m.est_rms_dip_deg = phase_rms(length * 0.45, length * 0.55)
    m.est_rms_ascent_deg = phase_rms(length * 0.55, length)

    m.min_available_a = float(min(profile.available_current_a(w / R_EFF_M) for w in v_arr))
    m.peak_abs_current_a = float(np.max(np.abs(currents)))
    m.duration_s = float(t_arr[-1])
    m.timestep_s = dt
    m.mujoco_version = mujoco.__version__
    m.imperfection_profile_id = profile.profile_id
    m.model_sha256 = hashlib.sha256(Path(MODEL_PATH).read_bytes()).hexdigest()[:16]

    return TerrainResult(
        params=params, metrics=m, plant=summary, t=t_arr, travel_m=trav,
        v_m_s=v_arr, grade_pct=np.asarray(grades), pitch_deg=np.asarray(pitches),
        pitch_est_deg=np.asarray(ests), motor_current_a=np.asarray(currents),
        qpos=np.asarray(states) if states else np.empty(0),
    )


def summary_line(r: TerrainResult) -> str:
    m = r.metrics
    if m.nose_strike:
        outcome = f"{m.struck_end.upper()} in the {m.struck_phase} @{m.t_strike_s:.1f}s"
    elif m.reached_next_crest:
        outcome = f"reached the crest @{m.t_reached_crest_s:.1f}s"
    else:
        outcome = f"stalled at {m.fraction_completed * 100:.0f}%"
    return (
        f"max grade {r.params.max_grade_pct:>5.1f}%  v_ref {r.params.v_ref_m_s:>4.1f}  "
        f"{outcome:<34s} est RMS {m.pitch_est_rms_deg:>6.3f}°  "
        f"(descent {m.est_rms_descent_deg:>6.3f} / dip {m.est_rms_dip_deg:>6.3f} / "
        f"ascent {m.est_rms_ascent_deg:>6.3f})"
    )
