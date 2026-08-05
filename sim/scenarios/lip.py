"""Riding over a square-edged lip -- the CEO's own ask, "does it ride city-park
concrete?"

WHY NEITHER EXISTING SCENARIO ANSWERS THIS
-------------------------------------------
`impulse_response.py` injects a momentum kick through the CoM. That measures
disturbance rejection, but the disturbance never touches the ground -- there
is no contact geometry in it at all, so it cannot say anything about an edge
the wheel actually has to climb.

`terrain.py` is smooth BY CONSTRUCTION: a cosine crest-to-crest ride sampled
at ~6 cm between facets. Even its steepest local slope is a continuous
surface with no discontinuity in height. A pavement lip -- a kerb, a slab
joint, a driveway apron -- is the opposite of that: a step change in height
over a few millimetres of run. Riding over one is exactly what "rides city-
park concrete" means, and neither existing scenario can produce the event.

This scenario does: flat ground, one step UP, flat ground again.

WHY CRUISE IS A MODERATE 2 M/S, ON PURPOSE
-------------------------------------------
At full stick the board is already running against the ~zero pitch-authority
margin ADR-0011 measured -- there is no headroom left to also spend on a
transient from a struck lip. A gate that only passes below the speed anyone
would actually ride at would not be measuring the manoeuvre the CEO asked
about; it would be measuring a manoeuvre nobody performs. 2 m/s is the
speed this gate is scoped to: fast enough to be a real cruise, slow enough
that the lip event is the thing under test rather than the speed itself.
"""

from __future__ import annotations

import hashlib
import math
from dataclasses import asdict, dataclass, field
from pathlib import Path

import mujoco
import numpy as np

from . import terrain_import
from .imperfections import (
    STAGE0_CUTBACK,
    ImperfectionProfile,
    ImperfectionState,
)
from .impulse_response import KT_NM_PER_A, _bumper_ground_contact, frame_pitch_rad
from .plant import (
    _GROUND_GEOM,
    MODEL_PATH,
    imu_readings,
    plant_summary,
    rider_geoms,
    spawn_at_sprung_equilibrium,
    wheel_dof,
)
from .rust_controller import DEFAULT_R_EFF_M as R_EFF_M


@dataclass(frozen=True)
class LipParams:
    #: Height of the step, metres. Square-edged: zero run at the transition.
    lip_height_m: float = 0.010
    #: Pass the raw profile through `terrain_import.envelope_profile` before
    #: it becomes the hfield, modelling the tyre's own contact-patch filter.
    #: False leaves the edge perfectly square, at whatever resolution
    #: `post_spacing_m` samples it -- the harder case.
    enveloped: bool = False
    v_ref_m_s: float = 2.0
    settle_s: float = 2.0
    ramp_s: float = 2.0
    duration_s: float = 12.0
    #: Forward distance from spawn to the edge, metres.
    lip_at_m: float = 6.0
    #: Travel beyond the edge that counts as "cleared", metres.
    clear_margin_m: float = 2.0
    ballast_mass_kg: float = 70.0
    ballast_height_m: float = 0.75
    max_current_a: float = 40.0
    estimator_tau_s: float = 2.0
    use_estimator: bool = True
    rider_style: str = "figure"
    #: Spacing of the RAW profile posts, metres, before any enveloping. 5 mm
    #: makes a 20 mm lip face ~76 deg -- square for the wheel's purposes,
    #: while still being a finite run rather than a true discontinuity a
    #: heightfield cannot represent.
    post_spacing_m: float = 0.005


@dataclass
class LipMetrics:
    # --- validity, outside the statistics ---
    survived: bool = False
    cleared_lip: bool = False
    """Did the board travel past the edge by the clear margin?"""
    t_cleared_s: float | None = None

    # --- outcome ---
    nose_strike: bool = False
    struck_end: str = ""
    t_strike_s: float | None = None

    # --- the ride ---
    peak_abs_pitch_deg: float = 0.0
    min_speed_after_lip_m_s: float = 0.0
    """Minimum forward speed while travel is between the edge and the clear
    margin. 0.0 if that span was never reached."""

    # --- drive ---
    peak_abs_current_a: float = 0.0
    saturated_cycles: int = 0
    cutback_binding_cycles: int = 0

    # --- the suspension the gate exists to exercise ---
    peak_tyre_deflect_mm: float = 0.0
    """Max abs deviation of the `tyre_deflect` slide from its value at t=0."""

    # --- provenance ---
    duration_s: float = 0.0
    model_sha256: str = ""
    mujoco_version: str = ""
    timestep_s: float = 0.0
    imperfection_profile_id: str = ""


@dataclass
class LipResult:
    params: LipParams
    metrics: LipMetrics
    plant: dict = field(default_factory=dict)
    t: np.ndarray = field(default_factory=lambda: np.empty(0))
    travel_m: np.ndarray = field(default_factory=lambda: np.empty(0))
    v_m_s: np.ndarray = field(default_factory=lambda: np.empty(0))
    pitch_deg: np.ndarray = field(default_factory=lambda: np.empty(0))
    motor_current_a: np.ndarray = field(default_factory=lambda: np.empty(0))

    qpos: np.ndarray = field(default_factory=lambda: np.empty(0))
    """Full state history, one row per step, when run(capture_state=True)."""

    def to_json_dict(self) -> dict:
        return {"params": asdict(self.params), "plant": self.plant,
                "metrics": asdict(self.metrics)}


def build_lip_model(params: LipParams):
    """The stock model with its ground plane replaced by a square-edged step.

    Every replacement is COUNTED, not trusted -- see `terrain.build_terrain_model`
    for why a silent no-op here is worse than an exception.

    The board travels toward -X from x=0 (spawn). The field spans a full
    settle-and-cruise run before the edge, the edge itself, and the clear
    margin plus a further 10 m of run-out beyond it.
    """
    x_lo = -(params.lip_at_m + params.clear_margin_m + 10.0)
    x_hi = 4.0
    radius_x = (x_hi - x_lo) / 2.0
    centre_x = (x_lo + x_hi) / 2.0

    if params.lip_height_m <= 0.0:
        raise ValueError("lip_height_m must be > 0 -- a normalised hfield needs a range")

    # Raw square-edged profile: flat, then up, at the raw post spacing.
    n_raw = int(round((x_hi - x_lo) / params.post_spacing_m)) + 1
    xs_raw = np.linspace(x_lo, x_hi, n_raw)
    zs_raw = np.where(xs_raw > -params.lip_at_m, 0.0, params.lip_height_m)

    if params.enveloped:
        xs, zs = terrain_import.envelope_profile(xs_raw, zs_raw)
    else:
        xs, zs = xs_raw, zs_raw
    ncol = int(xs.size)

    xml = MODEL_PATH.read_text()

    asset = (
        f'\n    <hfield name="lip" nrow="2" ncol="{ncol}" '
        f'size="{radius_x:.6f} 6.0 {params.lip_height_m:.6f} 0.5"/>'
    )
    if xml.count("</asset>") != 1:
        raise RuntimeError("expected exactly one </asset> to extend with the hfield")
    xml = xml.replace("</asset>", asset + "\n  </asset>", 1)

    # Field base sits at z = 0, the same z the frame is already standing at
    # (see `_FRAME_BODY`) -- so the pre-lip flat matches the stock spawn
    # height exactly and the frame body needs no correction here.
    ground = (
        f'<geom name="ground" type="hfield" hfield="lip" '
        f'pos="{centre_x:.6f} 0 0" material="ground_mat"/>'
    )
    if xml.count(_GROUND_GEOM) != 1:
        raise RuntimeError(
            f"expected exactly one ground plane geom to replace, found "
            f"{xml.count(_GROUND_GEOM)}. The markup in overboard_onewheel.xml "
            "changed; plant._GROUND_GEOM is stale."
        )
    xml = xml.replace(_GROUND_GEOM, ground, 1)

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

        # Widen the stock cameras, framed for a bare 0.3 m board, so a rider
        # 0.75 m above the axle is not cropped out.
        xml = xml.replace('fovy="26"', 'fovy="66"').replace('fovy="24"', 'fovy="64"')

    # Meshes supplied in memory, matching `terrain.build_terrain_model`.
    mesh_dir = MODEL_PATH.parent / "meshes" / "openwheel"
    assets = {f"meshes/openwheel/{p.name}": p.read_bytes()
              for p in mesh_dir.glob("*.stl")}
    model = mujoco.MjModel.from_xml_string(xml, assets)

    # Fill the elevation data. `data` is in [0, 1]; the profile's xs already
    # run from x_lo to x_hi ascending, the same convention `terrain.py` fills
    # its row in (`np.linspace(x_lo, x_hi, ncol)`), so no reversal is needed.
    adr = int(model.hfield_adr[0])
    ncol_m, nrow = int(model.hfield_ncol[0]), int(model.hfield_nrow[0])
    row = (zs / params.lip_height_m).astype(np.float32)
    model.hfield_data[adr:adr + nrow * ncol_m] = np.tile(row, nrow)

    # Verify the ground is actually a step. A flat field would make every
    # result in this module a flat-ground result wearing a lip's name.
    span = model.hfield_data[adr:adr + ncol_m]
    lo, hi = float(span.min()), float(span.max())
    if hi != 1.0 or lo != 0.0:
        raise RuntimeError(
            f"heightfield did not normalise to [0, 1] (got [{lo}, {hi}]); "
            "the lip did not take"
        )
    return model


def run(
    params: LipParams | None = None,
    profile: ImperfectionProfile = STAGE0_CUTBACK,
    controller_factory=None,
    capture_state: bool = False,
) -> LipResult:
    """Settle, ramp to cruise, ride over the lip, and measure what happened."""
    params = params or LipParams()
    if profile.is_ideal():
        raise ValueError("refusing to gate on an ideal profile: it models nothing")

    model = build_lip_model(params)
    summary = plant_summary(model)
    if not summary["inverted_pendulum"]:
        raise ValueError(
            "the lip gate requires a RIDDEN plant (centre of mass above the "
            f"axle). Got mgl {summary['mgl_n_m_per_rad']:+.2f}."
        )

    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    # Sit ON the tyre spring, not above it -- see the helper's docstring. This
    # is REQUIRED here: the estimator seeds its attitude from the first accel
    # sample, and a free-fall first sample hands it a phantom pitch.
    spawn_at_sprung_equilibrium(model, data)

    frame = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "frame")
    ground = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "ground")
    bumper_ids = {
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, n): end
        for n, end in (("front_bumper_geom", "nose"), ("rear_bumper_geom", "tail"))
    }

    tyre_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "tyre_deflect")
    if tyre_jid < 0:
        raise ValueError("model has no joint named 'tyre_deflect'")
    tyre_qadr = int(model.jnt_qposadr[tyre_jid])
    deflect0 = float(data.qpos[tyre_qadr])

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
                kp_nm_per_rad=140.0, kd_nm_per_rad_s=21.0,
                max_current_a=params.max_current_a,
                kp_v_rad_per_m_s=0.05, ki_v_rad_per_m=0.02,
                com_above_axle=True, v_ref_fn=v_ref_at,
                use_estimator=params.use_estimator,
                estimator_tau_s=params.estimator_tau_s,
            )

    dt = float(model.opt.timestep)
    n_steps = int(round(params.duration_s / dt))
    imp = ImperfectionState(profile=profile, dt_s=dt)
    wheel_dof_idx = wheel_dof(model)
    clear_at_m = params.lip_at_m + params.clear_margin_m

    ts, travels, vs, pitches, currents = [], [], [], [], []
    states: list[np.ndarray] = []
    x0 = float(data.xpos[frame][0])
    flowing_a = 0.0
    peak_deflect_mm = 0.0
    m = LipMetrics()

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

            ts.append(float(data.time))
            travels.append(fwd)
            vs.append(float(data.qvel[wheel_dof_idx]) * R_EFF_M)
            pitches.append(pitch_deg)
            currents.append(current)
            if capture_state:
                states.append(data.qpos.copy())

            m.peak_abs_pitch_deg = max(m.peak_abs_pitch_deg, abs(pitch_deg))
            deflect_mm = abs(float(data.qpos[tyre_qadr]) - deflect0) * 1000.0
            peak_deflect_mm = max(peak_deflect_mm, deflect_mm)

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
                break

            if not m.cleared_lip and fwd >= clear_at_m:
                m.cleared_lip = True
                m.t_cleared_s = float(data.time)
                break

        m.saturated_cycles = int(ctl.saturated_cycles)

    t_arr = np.asarray(ts)
    trav = np.asarray(travels)
    v_arr = np.asarray(vs)

    m.survived = not m.nose_strike
    after_lip = (trav >= params.lip_at_m) & (trav <= clear_at_m)
    m.min_speed_after_lip_m_s = float(np.min(v_arr[after_lip])) if after_lip.any() else 0.0
    m.peak_abs_current_a = float(np.max(np.abs(currents))) if currents else 0.0
    m.peak_tyre_deflect_mm = peak_deflect_mm

    m.duration_s = float(t_arr[-1]) if t_arr.size else 0.0
    m.timestep_s = dt
    m.mujoco_version = mujoco.__version__
    m.imperfection_profile_id = profile.profile_id
    m.model_sha256 = hashlib.sha256(Path(MODEL_PATH).read_bytes()).hexdigest()[:16]

    return LipResult(
        params=params, metrics=m, plant=summary, t=t_arr, travel_m=trav,
        v_m_s=v_arr, pitch_deg=np.asarray(pitches), motor_current_a=np.asarray(currents),
        qpos=np.asarray(states) if states else np.empty(0),
    )


def summary_line(r: LipResult) -> str:
    m = r.metrics
    if m.nose_strike:
        outcome = f"STRUCK {m.struck_end}@{m.t_strike_s:.2f}s"
    elif m.cleared_lip:
        outcome = f"CLEARED @{m.t_cleared_s:.2f}s"
    else:
        outcome = "stalled"
    kind = "enveloped" if r.params.enveloped else "raw"
    return (
        f"lip {r.params.lip_height_m * 1000:>5.1f} mm  {kind:<9s}  {outcome:<18s}  "
        f"peak pitch {m.peak_abs_pitch_deg:>6.2f} deg  "
        f"min v after {m.min_speed_after_lip_m_s:>5.2f} m/s  "
        f"sat {m.saturated_cycles:>4d}  peak deflect {m.peak_tyre_deflect_mm:>5.2f} mm"
    )


if __name__ == "__main__":
    for _h in (0.005, 0.010, 0.020):
        for _env in (False, True):
            _r = run(LipParams(lip_height_m=_h, enveloped=_env))
            print(summary_line(_r))
