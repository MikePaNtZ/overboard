"""Hill scenarios — descending and climbing under closed-loop control.

This is the scenario where the gap between *"the board balances in sim"* and
*"this will hurt someone"* is narrowest, and it is the one the estimator work
never gated.

WHAT A GRADE IS, HERE
---------------------
The slope is modelled by **rotating gravity**, not by tilting the ground:

    m.opt.gravity = [-g·sin φ, 0, -g·cos φ]        (forward is -x)

For a board on a plane, a rotated gravity vector and a tilted plane are the
same problem — but tilting the geometry would move the nose-strike contact
case, which is the one thing this scenario exists to measure. Rotating gravity
leaves the strike geometry bit-identical to every other scenario, so an 18.57°
strike here means what it means everywhere else.

`grade_pct` is **signed, and the sign is measured rather than asserted**
(`test_positive_grade_is_downhill_forward`):

    grade > 0   downhill is forward   → commanding forward speed DESCENDS
    grade < 0   uphill is forward     → commanding forward speed CLIMBS

THE FAILURE THIS FINDS IS NOT A NOSEDIVE
----------------------------------------
The handoff predicted nosedives from 15% grade. Re-measured after the
model→ICD frame map was corrected, **nothing strikes up to 20%**. That is not
the good news it looks like. What actually breaks is *speed authority*:
commanded 1.0 m/s down a 10% grade, the board finishes above 11 m/s, and the
final speed is almost completely insensitive to the commanded speed.

The mechanism is the one the handoff derived, and it survives the frame fix
intact. `CommandFeedforward` predicts forward acceleration as `K_ff·I` and
subtracts it from the accelerometer. On a slope, much of that current is
fighting *gravity* rather than accelerating anything, so the feedforward
attributes gravity-holding torque to acceleration. The resulting apparent-tilt
error is, to first order, **the slope angle itself** — the estimator believes a
board on a hill is level. Holding "level" on a hill means accelerating down it.
Measured here: estimator RMS error tracks ≈0.87 × the slope angle across 5–20%.

So the gate is on **speed authority**, and nose strike is reported as the
secondary, more violent outcome rather than the primary one. A scenario that
only asserted `not nose_strike` would pass while the board ran away at 40 km/h.

CUTBACK IS NOT OPTIONAL HERE
----------------------------
`control-core`'s feedforward docstring states the gap plainly: a VESC derates
commanded torque through half a dozen layers and *"it would appear under load,
on a hill… The sim cannot show it, because the sim has no cutback."* A descent
without a cutback model is a descent with an infinitely strong motor, which is
the one thing hardware will not be. `run()` therefore **refuses a profile that
does not model cutback** unless you opt out in as many words.
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
from .impulse_response import (
    KT_NM_PER_A,
    _bumper_ground_contact,
    frame_pitch_rad,
    nose_strike_angle_deg,
)
from .plant import MODEL_PATH, build_model, imu_readings, plant_summary
from .rust_controller import DEFAULT_R_EFF_M as R_EFF_M

G = 9.81


def gravity_for_grade(grade_pct: float) -> tuple[float, float, float]:
    """World gravity vector for a grade, in percent. See the module docstring.

    Positive grade puts a gravity component along **forward** (-x), i.e.
    downhill is the direction the board drives when commanded forward.
    """
    phi = math.atan(grade_pct / 100.0)
    return (-G * math.sin(phi), 0.0, -G * math.cos(phi))


def _which_end(data, ground_id, bumper_ids: dict) -> str:
    """Which bumper is touching the ground: `"nose"`, `"tail"`, or `"both"`."""
    ends = set()
    for i in range(data.ncon):
        c = data.contact[i]
        for a, b in ((c.geom1, c.geom2), (c.geom2, c.geom1)):
            if a == ground_id and b in bumper_ids:
                ends.add(bumper_ids[b])
    if len(ends) == 1:
        return ends.pop()
    return "both" if ends else ""


def grade_angle_deg(grade_pct: float) -> float:
    """The slope as an angle. A 15% grade is 8.53°, not 15°, and conflating
    the two overstates the disturbance by ~75%."""
    return math.degrees(math.atan(grade_pct / 100.0))


@dataclass(frozen=True)
class HillParams:
    #: Signed grade in percent. Positive descends when driving forward.
    grade_pct: float = 10.0
    #: Commanded forward speed, m/s. Positive is forward in both directions of
    #: grade — a climb is commanded exactly like a descent, which is the point.
    v_ref_m_s: float = 2.0
    duration_s: float = 12.0

    #: Seconds held at v_ref = 0 before the speed command begins, so the board
    #: reaches equilibrium ON THE GRADE first.
    #:
    #: Not cosmetic. Without it the run starts with the board at the pose that
    #: balanced under *vertical* gravity while gravity is already rotated, and
    #: the speed command steps from 0 at the same instant — so t = 0 is a
    #: simultaneous attitude step and throttle step. The first version of this
    #: scenario had no settle and every climb "failed" inside 0.8 s; it was
    #: measuring that transient, not the board's ability to hold a hill. A real
    #: board is on the hill before it is asked to move.
    settle_s: float = 2.0

    #: Seconds to ramp the speed command from 0 to `v_ref_m_s`. A step demand
    #: is a disturbance in its own right and would confound the grade with the
    #: controller's step response.
    ramp_s: float = 2.0
    ballast_mass_kg: float = 70.0
    ballast_height_m: float = 0.75
    max_current_a: float = 40.0
    #: Estimator crossover. The handoff's recommended config is tau = 2 s with
    #: command feedforward, and this scenario defaults to it because a hill run
    #: on truth pitch cannot show the failure that makes hills dangerous.
    estimator_tau_s: float = 2.0
    use_estimator: bool = True
    #: 0 = commanded current, 1 = measured. Hardware must use 1; the default
    #: here matches the shipped default so the gate measures what ships.
    accel_ff_current_source: int = 0


@dataclass
class HillMetrics:
    # --- validity, computed OUTSIDE the statistics ---
    survived: bool = False
    """Did the run finish upright, with no bumper ever touching the ground?

    **Every other metric below is meaningless when this is False**, and the run
    stops the moment it goes False. A crashed board coasts to a halt, which
    makes `v_final ≈ 0` and reads as perfect speed holding — an earlier version
    of this file scored exactly that as `held_speed = True`. The handoff's §3.2
    lesson, relearned: validity checks must sit outside the statistics computed
    from the run, because "did it stay upright" catches what the statistics
    cannot."""

    # --- the gate: does it still obey the throttle? ---
    held_speed: bool = False
    """Survived AND kept the commanded speed. Requires `survived`; a crash can
    never report that it held speed."""

    max_speed_error_m_s: float = 0.0
    """Worst |v| − |v_ref| over the run. The headline number."""

    v_final_m_s: float = 0.0
    v_ref_m_s: float = 0.0
    peak_abs_speed_m_s: float = 0.0

    ran_away: bool = False
    """Speed diverged rather than settling: |v_final| exceeded twice the
    commanded speed plus 1 m/s. Deliberately crude — a runaway is not a
    marginal call, and a tight threshold here would invite tuning."""

    reversed_direction: bool = False
    """Finished travelling BACKWARDS while commanded forward. On a climb this
    is the characteristic failure: the board gives up and descends."""

    # --- the more violent secondary outcome ---
    nose_strike: bool = False
    """A bumper reached the ground. Detected by GEOMETRIC CONTACT, not by an
    angle threshold: the strike angle differs nose-down from tail-down, and an
    `abs(pitch) >= 18.57` test silently calls a tail-down fall a nose strike."""

    struck_end: str = ""
    """Which end went down — `"nose"` or `"tail"`. On a climb the board falls
    the other way, and reporting both as "nose strike" hides the mechanism."""

    t_strike_s: float | None = None
    pitch_at_strike_deg: float = 0.0
    peak_abs_pitch_deg: float = 0.0
    nose_strike_angle_deg: float = 0.0

    # --- why it failed: the estimator ---
    pitch_est_rms_deg: float = 0.0
    """RMS error of the attitude estimate against truth. On a hill this is
    expected to approach the slope angle — that IS the mechanism."""

    pitch_est_max_deg: float = 0.0
    slope_angle_deg: float = 0.0
    est_error_over_slope: float = 0.0
    """`pitch_est_rms_deg / slope_angle_deg`. Near 1.0 means the estimator has
    absorbed the entire slope into its attitude error, which is the predicted
    first-order behaviour."""

    # --- did the drive run out of authority? ---
    cutback_cycles: int = 0
    """Cycles where the available cap was below the nominal cap."""

    cutback_binding_cycles: int = 0
    """Cycles where the controller ASKED for more than cutback would pass.
    This is the one that matters: cutback that never binds changed nothing."""

    min_available_a: float = 0.0
    peak_abs_current_a: float = 0.0
    saturated_cycles: int = 0

    # --- provenance ---
    travel_m: float = 0.0
    duration_s: float = 0.0
    model_sha256: str = ""
    mujoco_version: str = ""
    timestep_s: float = 0.0
    imperfection_profile_id: str = ""


@dataclass
class HillResult:
    params: HillParams
    metrics: HillMetrics
    plant: dict = field(default_factory=dict)
    t: np.ndarray = field(default_factory=lambda: np.empty(0))
    v_m_s: np.ndarray = field(default_factory=lambda: np.empty(0))
    pitch_deg: np.ndarray = field(default_factory=lambda: np.empty(0))
    pitch_est_deg: np.ndarray = field(default_factory=lambda: np.empty(0))
    motor_current_a: np.ndarray = field(default_factory=lambda: np.empty(0))
    available_a: np.ndarray = field(default_factory=lambda: np.empty(0))
    travel_m: np.ndarray = field(default_factory=lambda: np.empty(0))

    def to_json_dict(self) -> dict:
        return {
            "params": asdict(self.params),
            "plant": self.plant,
            "metrics": asdict(self.metrics),
        }


def run(
    params: HillParams | None = None,
    profile: ImperfectionProfile = STAGE0_CUTBACK,
    controller_factory=None,
    allow_no_cutback: bool = False,
) -> HillResult:
    """Drive a grade at a commanded speed and measure whether control holds.

    `allow_no_cutback` exists so a test can isolate the estimator mechanism
    from the drive mechanism, and it must be passed explicitly. Defaulting it
    to True would mean a hill result could silently be produced against an
    infinitely strong motor, which is the exact class of quiet degradation this
    codebase has already shipped twice.
    """
    params = params or HillParams()

    if not profile.models_cutback() and not allow_no_cutback:
        raise ValueError(
            f"profile {profile.profile_id!r} does not model cutback, and a descent "
            "against a motor that never derates is not a hill test -- see "
            "control-core's feedforward docstring. Use STAGE0_CUTBACK, or pass "
            "allow_no_cutback=True if you are deliberately isolating the estimator."
        )
    if profile.is_ideal():
        raise ValueError("refusing to gate on an ideal profile: it models nothing")

    model = build_model(
        params.ballast_mass_kg, params.ballast_height_m, params.max_current_a
    )
    model.opt.gravity[:] = gravity_for_grade(params.grade_pct)

    summary = plant_summary(model)
    if not summary["inverted_pendulum"]:
        raise ValueError(
            "Hill requires a RIDDEN plant (centre of mass above the axle); the outer "
            f"loop's coupling inverts below it. Got mgl {summary['mgl_n_m_per_rad']:+.2f}."
        )

    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    frame = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "frame")
    ground = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "ground")
    bumper_ids = {
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, n): end
        for n, end in (("front_bumper_geom", "nose"), ("rear_bumper_geom", "tail"))
    }

    def v_ref_at(t: float) -> float:
        """Zero through the settle, then a linear ramp to the target."""
        if t < params.settle_s:
            return 0.0
        if params.ramp_s <= 0.0:
            return params.v_ref_m_s
        frac = min(1.0, (t - params.settle_s) / params.ramp_s)
        return params.v_ref_m_s * frac

    if controller_factory is None:
        from .rust_controller import RustController

        def controller_factory():
            return RustController(
                v_ref_fn=v_ref_at,
                # 200 A/rad, 30 A/(rad/s) at kt = 0.7 N*m/A, re-denominated (#137).
                kp_nm_per_rad=140.0,
                kd_nm_per_rad_s=21.0,
                max_current_a=params.max_current_a,
                kp_v_rad_per_m_s=0.05,
                ki_v_rad_per_m=0.02,
                com_above_axle=True,
                use_estimator=params.use_estimator,
                estimator_tau_s=params.estimator_tau_s,
                accel_ff_current_source=params.accel_ff_current_source,
            )

    dt = float(model.opt.timestep)
    n_steps = int(round(params.duration_s / dt))
    imp = ImperfectionState(profile=profile, dt_s=dt)
    strike_deg = nose_strike_angle_deg(model)

    ts, vs, pitches, ests, currents, avails, travels = [], [], [], [], [], [], []
    x0 = float(data.xpos[frame][0])
    flowing_a = 0.0
    m = HillMetrics()

    with controller_factory() as ctl:
        for _ in range(n_steps):
            t = float(data.time)
            pitch_rad = frame_pitch_rad(model, data)
            true_rate = float(data.qvel[4])
            wheel_true = float(data.qvel[6])

            sensed_rate = imp.gyro(true_rate)
            sensed_wheel = imp.wheel_rate(wheel_true, t)
            true_gyro, true_accel = imu_readings(model, data)

            proposed = float(ctl(
                t, pitch_rad, sensed_rate, sensed_wheel,
                gyro_rad_s=imp.gyro_vec(true_gyro),
                accel_m_s2=imp.accel_vec(true_accel),
                motor_current_a=flowing_a,
            ))

            # The wheel rate the DRIVE sees, for cutback. Truth rather than the
            # quantised estimate: cutback happens inside the drive, which knows
            # its own ERPM exactly.
            available = profile.available_current_a(wheel_true)
            if available < profile.max_current_a - 1e-9:
                m.cutback_cycles += 1
            if abs(proposed) > available + 1e-9:
                m.cutback_binding_cycles += 1

            current = imp.apply_current(proposed, wheel_true)
            flowing_a = current
            data.ctrl[0] = current * KT_NM_PER_A
            mujoco.mj_step(model, data)

            pitch_deg = math.degrees(frame_pitch_rad(model, data))
            est_deg = math.degrees(float(ctl.pitch_used_rad))
            v = float(data.qvel[6]) * R_EFF_M
            fwd = float(-(data.xpos[frame][0] - x0))

            ts.append(float(data.time))
            vs.append(v)
            pitches.append(pitch_deg)
            ests.append(est_deg)
            currents.append(current)
            avails.append(available)
            travels.append(fwd)

            if abs(pitch_deg) > m.peak_abs_pitch_deg:
                m.peak_abs_pitch_deg = abs(pitch_deg)

            # STOP AT THE CRASH. Integrating past a strike produces statistics
            # from a board tumbling on the ground -- the sweep that caught this
            # reported a 179.97 deg "peak pitch" and a 271 deg estimator RMS,
            # numbers that describe wreckage rather than control.
            if _bumper_ground_contact(model, data, ground, set(bumper_ids)) is not None:
                m.nose_strike = True
                m.t_strike_s = float(data.time)
                m.pitch_at_strike_deg = pitch_deg
                m.struck_end = _which_end(data, ground, bumper_ids)
                break

        m.saturated_cycles = int(ctl.saturated_cycles)

    t_arr = np.asarray(ts)
    v_arr = np.asarray(vs)
    pitch_arr = np.asarray(pitches)
    est_arr = np.asarray(ests)
    cur_arr = np.asarray(currents)
    avail_arr = np.asarray(avails)
    travel_arr = np.asarray(travels)

    err = est_arr - pitch_arr
    slope = abs(grade_angle_deg(params.grade_pct))

    # Score the CRUISE window only. The settle and the ramp are the scenario
    # setting itself up; including them would charge the controller for not yet
    # having been asked to move.
    cruise_from = params.settle_s + params.ramp_s
    cruise = t_arr >= cruise_from
    if not cruise.any():
        # Crashed before cruise. Every speed number below is meaningless, and
        # `survived` already says so -- use the last sample rather than an
        # empty slice so the metrics stay readable.
        cruise = np.zeros_like(t_arr, dtype=bool)
        cruise[-1] = True

    m.v_ref_m_s = params.v_ref_m_s
    m.v_final_m_s = float(v_arr[-1])
    m.peak_abs_speed_m_s = float(np.max(np.abs(v_arr[cruise])))
    m.max_speed_error_m_s = float(
        np.max(np.abs(v_arr[cruise]) - abs(params.v_ref_m_s))
    )
    m.survived = not m.nose_strike
    m.ran_away = abs(m.v_final_m_s) > 2.0 * abs(params.v_ref_m_s) + 1.0
    m.reversed_direction = params.v_ref_m_s > 0.0 and m.v_final_m_s < -0.1
    # `survived` first, and not as a formality: a crashed board coasts to rest,
    # so every speed test below passes trivially on wreckage.
    m.held_speed = m.survived and (not m.ran_away) and (not m.reversed_direction)

    m.nose_strike_angle_deg = float(strike_deg)
    m.pitch_est_rms_deg = float(np.sqrt(np.mean(np.square(err))))
    m.pitch_est_max_deg = float(np.max(np.abs(err)))
    m.slope_angle_deg = float(slope)
    m.est_error_over_slope = float(m.pitch_est_rms_deg / slope) if slope > 1e-9 else 0.0

    m.min_available_a = float(np.min(avail_arr))
    m.peak_abs_current_a = float(np.max(np.abs(cur_arr)))
    m.travel_m = float(travel_arr[-1])
    m.duration_s = float(params.duration_s)
    m.timestep_s = dt
    m.mujoco_version = mujoco.__version__
    m.imperfection_profile_id = profile.profile_id
    m.model_sha256 = hashlib.sha256(
        Path(MODEL_PATH).read_bytes()
    ).hexdigest()[:16]

    return HillResult(
        params=params, metrics=m, plant=summary, t=t_arr, v_m_s=v_arr,
        pitch_deg=pitch_arr, pitch_est_deg=est_arr, motor_current_a=cur_arr,
        available_a=avail_arr, travel_m=travel_arr,
    )


def sweep(
    grades_pct=(-15.0, -10.0, -5.0, 5.0, 10.0, 15.0, 20.0),
    speeds_m_s=(1.0, 2.0, 3.0),
    profile: ImperfectionProfile = STAGE0_CUTBACK,
    duration_s: float = 12.0,
    **param_overrides,
) -> list[HillResult]:
    """Margin sweep across grade × commanded speed.

    Reports where it breaks rather than passing or failing at one point, which
    is what the handoff asked for and what a single-grade gate cannot give.
    Both signs of grade are swept by default: a climb fails differently from a
    descent, and only sweeping descents would miss it entirely.
    """
    out = []
    for g in grades_pct:
        for v in speeds_m_s:
            out.append(run(
                HillParams(grade_pct=g, v_ref_m_s=v, duration_s=duration_s,
                           **param_overrides),
                profile=profile,
            ))
    return out


def margin_table(results: list[HillResult]) -> str:
    """Human-readable sweep summary. Used by the analysis scripts and by
    anyone trying to see the edge rather than a boolean."""
    lines = [
        f"{'grade':>7s} {'v_ref':>6s} {'outcome':>12s} {'v_end':>8s} {'held':>5s} "
        f"{'peak°':>7s} {'est RMS':>8s} {'err/slope':>10s} {'cutback':>8s}",
        "-" * 82,
    ]
    for r in results:
        m = r.metrics
        if m.nose_strike:
            outcome = f"{m.struck_end.upper()} @{m.t_strike_s:.1f}s"
        else:
            outcome = "upright"
        lines.append(
            f"{r.params.grade_pct:>6.0f}% {r.params.v_ref_m_s:>6.1f} {outcome:>12s} "
            f"{m.v_final_m_s:>8.2f} {('yes' if m.held_speed else 'NO'):>5s} "
            f"{m.peak_abs_pitch_deg:>7.2f} "
            f"{m.pitch_est_rms_deg:>8.3f} {m.est_error_over_slope:>10.2f} "
            f"{m.cutback_binding_cycles:>8d}"
        )
    return "\n".join(lines)
