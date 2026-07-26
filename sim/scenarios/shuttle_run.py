"""Shuttle Run — commanded velocity, reversal, and return to home.

Drive out, stop and hold, drive back past the start, stop and hold, then return
to where you began. The impulse scenario asks "can it cope with something done
*to* it"; this asks **"will it do what it is told"**, which is a different
question and tests things the impulse cannot:

* **Setpoint tracking**, not just regulation to zero.
* **Direction reversal** — the sign of the whole chain under *command*. This
  project has already found one inverted motor sign and one sign that inverts
  with the plant, so a test that drives both ways earns its place.
* **The pauses are the sharp part.** They prove the board *stops and holds*
  rather than coasting to a halt that happens to look right.
* **Return-to-home error is one legible scalar** — closed-loop position
  accuracy in centimetres, a far better headline than a pitch angle.

RIDDEN PLANT ONLY. The cascade is gated on a board carrying rider-scale mass,
because the outer loop's pitch→velocity coupling inverts with the centre of
mass and its authority collapses without one (experiment log E03, E05). Running
this on the driverless board would be measuring a configuration the controller
is explicitly not built for.

No disturbance is injected. The command *is* the disturbance.

HOW RETURN-TO-HOME WORKS, AND WHY IT IS NOT A SEPARATE FEATURE
--------------------------------------------------------------
The outer loop integrates `(v - v_ref)`, which is position error against the
commanded trajectory `∫v_ref dt`. So a velocity profile whose integral is zero
brings the board home on its own -- there is no position controller here, and
adding one would be duplicating the integrator that already exists.
"""

from __future__ import annotations

import hashlib
from dataclasses import asdict, dataclass, field
from pathlib import Path

import mujoco
import numpy as np

from .imperfections import STAGE0_PLACEHOLDER, ImperfectionProfile, ImperfectionState
from .impulse_response import KT_NM_PER_A, frame_pitch_rad
from .plant import MODEL_PATH, build_model, plant_summary

#: Cruise speed for every leg, m/s. Modest on purpose: the outer loop's pitch
#: reference is clamped at 5 deg, which caps acceleration at about
#: g*tan(5 deg) = 0.86 m/s^2, so a leg needs roughly a second at each end just
#: to get up to speed and back down.
CRUISE_M_S = 0.6

#: Seconds to ramp between rest and cruise. A step change in the setpoint asks
#: the outer loop for an acceleration it cannot deliver, and it answers by
#: pinning its reference at the clamp -- which is how E05's divergence started.
RAMP_S = 0.6


@dataclass(frozen=True)
class Leg:
    """One commanded movement, or a pause."""

    label: str
    distance_m: float = 0.0
    """Signed. Positive is forward. Zero means hold station for `hold_s`."""

    hold_s: float = 0.0


#: How long each pause lasts. **Sized from the outer loop's settling time, not
#: chosen for pacing.** Measured on this route:
#:
#:     hold 2 s -> return 13.3 cm, creep 34.4 cm
#:     hold 4 s -> return  5.0 cm, creep 16.6 cm
#:     hold 6 s -> return  0.9 cm, creep 10.3 cm
#:
#: The outer loop is deliberately slow (its actuator is the inner loop's
#: setpoint), so position takes ~6 s to converge. With a short pause the
#: "creep" figure is really measuring convergence, and the scenario would be
#: scoring the board for not having finished stopping. 4 s is the compromise:
#: long enough that the metric mostly means what it says, short enough to keep
#: the clip watchable.
HOLD_S = 4.0

#: The route. Out, back past the start, then home -- so the return leg is a
#: real correction rather than a mirror of the outbound one, and the board has
#: to have been on BOTH sides of home before it finishes. A route that simply
#: retraced itself could be passed by a controller with an inverted sign.
DEFAULT_ROUTE: tuple[Leg, ...] = (
    Leg("settle", hold_s=1.0),
    Leg("out", distance_m=+2.0),
    Leg("hold-out", hold_s=HOLD_S),
    Leg("back", distance_m=-3.0),
    Leg("hold-back", hold_s=HOLD_S),
    Leg("home", distance_m=+1.0),
    Leg("hold-home", hold_s=HOLD_S),
)


@dataclass(frozen=True)
class ShuttleParams:
    route: tuple[Leg, ...] = DEFAULT_ROUTE
    cruise_m_s: float = CRUISE_M_S
    ramp_s: float = RAMP_S
    ballast_mass_kg: float = 70.0
    ballast_height_m: float = 0.75
    max_current_a: float = 40.0
    #: Band the board must stay inside during a pause to count as holding.
    hold_band_m: float = 0.10


@dataclass
class ShuttleMetrics:
    # --- the gate ---
    return_error_m: float = 0.0
    """How far from home it finished. The headline number."""

    max_tracking_error_m: float = 0.0
    """Worst deviation from the commanded position at any instant."""

    max_hold_drift_m: float = 0.0
    """Worst movement during the SETTLED part of a pause -- does it creep?

    Measured over the second half of each hold window only. The first half is
    the board still decelerating from the previous leg, which is not creep and
    scoring it as such would conflate "has not stopped yet" with "cannot hold"."""

    # --- did it actually do the manoeuvre ---
    max_forward_m: float = 0.0
    min_forward_m: float = 0.0
    """Extremes reached. Together these prove it went out AND came back past
    home, rather than sitting still and scoring a good return error."""

    # --- effort and margin ---
    peak_abs_pitch_deg: float = 0.0
    peak_abs_pitch_ref_deg: float = 0.0
    peak_current_a: float = 0.0
    rms_torque_nm: float = 0.0
    saturated_cycles: int = 0
    nose_strike: bool = False

    # --- provenance ---
    duration_s: float = 0.0
    model_sha256: str = ""
    mujoco_version: str = ""
    timestep_s: float = 0.0
    imperfection_profile_id: str = ""


@dataclass
class ShuttleResult:
    params: ShuttleParams
    metrics: ShuttleMetrics
    plant: dict = field(default_factory=dict)
    t: np.ndarray = field(default_factory=lambda: np.empty(0))
    pos_m: np.ndarray = field(default_factory=lambda: np.empty(0))
    pos_cmd_m: np.ndarray = field(default_factory=lambda: np.empty(0))
    v_m_s: np.ndarray = field(default_factory=lambda: np.empty(0))
    v_ref_m_s: np.ndarray = field(default_factory=lambda: np.empty(0))
    pitch_deg: np.ndarray = field(default_factory=lambda: np.empty(0))
    pitch_ref_deg: np.ndarray = field(default_factory=lambda: np.empty(0))
    motor_current_a: np.ndarray = field(default_factory=lambda: np.empty(0))
    qpos: np.ndarray = field(default_factory=lambda: np.empty(0))

    def to_json_dict(self) -> dict:
        return {
            "params": {**asdict(self.params), "route": [asdict(l) for l in self.params.route]},
            "plant": self.plant,
            "metrics": asdict(self.metrics),
        }


class VelocityProfile:
    """Trapezoidal speed schedule over a route, plus the position it commands.

    Built once and sampled by time, so the same profile drives the controller
    and defines the trajectory the result is scored against. Deriving the
    target from anything else would let the two drift apart and quietly make
    the tracking error meaningless.
    """

    def __init__(self, params: ShuttleParams) -> None:
        self.segments: list[tuple[float, float, float, float, str]] = []
        t = 0.0
        for leg in params.route:
            if leg.distance_m == 0.0:
                self.segments.append((t, t + leg.hold_s, 0.0, 0.0, leg.label))
                t += leg.hold_s
                continue

            sign = 1.0 if leg.distance_m > 0 else -1.0
            dist = abs(leg.distance_m)
            # The ramp SLOPE is the invariant, not the ramp duration: both the
            # trapezoid and the short-hop triangle accelerate at the same rate,
            # so a leg does not become more aggressive just for being short.
            accel = params.cruise_m_s / params.ramp_s

            # The two ramps of a full trapezoid together cover cruise*ramp.
            if dist >= params.cruise_m_s * params.ramp_s:
                peak, ramp = params.cruise_m_s, params.ramp_s
                cruise_t = (dist - peak * ramp) / peak
            else:
                # Too short to reach cruise: a triangle with a lower peak.
                # Distance under a triangle is peak*ramp with peak = accel*ramp,
                # so ramp = sqrt(dist/accel). Reducing the ramp WITHOUT also
                # reducing the peak -- the bug this replaces -- commands more
                # distance than was asked for.
                ramp = float(np.sqrt(dist / accel))
                peak = accel * ramp
                cruise_t = 0.0
            v = sign * peak

            self.segments.append((t, t + ramp, 0.0, v, f"{leg.label}:ramp-up"))
            t += ramp
            self.segments.append((t, t + cruise_t, v, v, f"{leg.label}:cruise"))
            t += cruise_t
            self.segments.append((t, t + ramp, v, 0.0, f"{leg.label}:ramp-down"))
            t += ramp
        self.duration_s = t

    def v_ref(self, t: float) -> float:
        for t0, t1, v0, v1, _ in self.segments:
            if t0 <= t < t1:
                if t1 - t0 <= 0.0:
                    return v1
                return float(v0 + (v1 - v0) * (t - t0) / (t1 - t0))
        return 0.0

    def is_hold(self, t: float) -> bool:
        for t0, t1, _, _, label in self.segments:
            if t0 <= t < t1:
                return label.startswith("hold")
        return False

    def is_settled_hold(self, t: float) -> bool:
        """Inside the second half of a pause -- past the point where the board
        should still be decelerating."""
        for t0, t1, _, _, label in self.segments:
            if t0 <= t < t1:
                return label.startswith("hold") and t >= t0 + 0.5 * (t1 - t0)
        return False


def run(
    params: ShuttleParams | None = None,
    controller_factory=None,
    capture_state: bool = False,
    profile: ImperfectionProfile = STAGE0_PLACEHOLDER,
) -> ShuttleResult:
    """Drive the route and measure how well it was followed.

    `controller_factory` takes the profile and returns a controller callable;
    injected so a test can substitute gains without this module knowing what a
    `RustController` is.
    """
    params = params or ShuttleParams()
    vprofile = VelocityProfile(params)

    model = build_model(params.ballast_mass_kg, params.ballast_height_m, params.max_current_a)

    # Refuse the driverless plant outright rather than producing a plausible
    # -looking trajectory that means nothing. The outer loop needs the centre
    # of mass ABOVE the axle: below it the pitch->velocity coupling inverts and
    # the authority collapses ~200x, so the loop pegs its reference at the
    # clamp and diverges (experiment log E03, E05). Asked for by mistake once
    # already, via a CLI default of zero ballast, and it quietly returned a
    # 22.6 m "return error" instead of saying no.
    summary = plant_summary(model)
    if not summary["inverted_pendulum"]:
        raise ValueError(
            "Shuttle Run requires a RIDDEN plant (centre of mass above the axle). "
            f"Got CoM {summary['com_above_axle_mm']:+.1f} mm relative to the axle, "
            f"mgl {summary['mgl_n_m_per_rad']:+.2f}. The outer loop's coupling inverts "
            "below the axle and its speed authority collapses -- see E03/E05. "
            "Set ballast_mass_kg to a rider-scale value."
        )

    data = mujoco.MjData(model)
    frame = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "frame")
    ground = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "ground")
    bumpers = {
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, n)
        for n in ("front_bumper_geom", "rear_bumper_geom")
    }

    if controller_factory is None:
        from .rust_controller import RustController

        def controller_factory(vprofile):
            return RustController(
                kp_a_per_rad=200.0,
                kd_a_per_rad_s=30.0,
                max_current_a=params.max_current_a,
                kp_v_rad_per_m_s=0.05,
                ki_v_rad_per_m=0.02,
                v_ref_fn=vprofile.v_ref,
                com_above_axle=True,
            )

    dt = float(model.opt.timestep)
    n_steps = int(round(vprofile.duration_s / dt))
    imp = ImperfectionState(profile=profile, dt_s=dt)
    x0 = float(data.xpos[frame][0])

    m = ShuttleMetrics(
        duration_s=vprofile.duration_s,
        model_sha256=hashlib.sha256(MODEL_PATH.read_bytes()).hexdigest()[:16],
        mujoco_version=mujoco.__version__,
        timestep_s=dt,
        imperfection_profile_id=profile.profile_id,
    )

    ts, pos, pos_cmd, vs, vrefs, pitches, prefs, currents = [], [], [], [], [], [], [], []
    states: list[np.ndarray] = []
    commanded_pos = 0.0
    hold_anchor: float | None = None

    with controller_factory(vprofile) as ctl:
        for _ in range(n_steps):
            t = float(data.time)
            # Corrupted signals to the controller; truth to the trajectory.
            pitch = frame_pitch_rad(model, data)
            rate = imp.gyro(float(data.qvel[4]))
            wheel = imp.wheel_rate(float(data.qvel[6]), t)

            current = imp.apply_current(float(ctl(t, pitch, rate, wheel)))
            data.ctrl[0] = current * KT_NM_PER_A
            mujoco.mj_step(model, data)

            commanded_pos += vprofile.v_ref(t) * dt
            fwd = float(-(data.xpos[frame][0] - x0))
            v = float(data.qvel[6]) * 0.14605

            ts.append(float(data.time))
            pos.append(fwd)
            pos_cmd.append(commanded_pos)
            vs.append(v)
            vrefs.append(vprofile.v_ref(t))
            pitch_deg = float(np.degrees(frame_pitch_rad(model, data)))
            pitches.append(pitch_deg)
            prefs.append(float(np.degrees(ctl.pitch_ref_rad)))
            currents.append(current)
            if capture_state:
                states.append(data.qpos.copy())

            m.max_tracking_error_m = max(m.max_tracking_error_m, abs(fwd - commanded_pos))
            m.max_forward_m = max(m.max_forward_m, fwd)
            m.min_forward_m = min(m.min_forward_m, fwd)
            m.peak_abs_pitch_deg = max(m.peak_abs_pitch_deg, abs(pitch_deg))
            m.peak_current_a = max(m.peak_current_a, abs(current))

            # Hold quality, over the SETTLED half of each pause. Anchoring at
            # the start of the window would score the tail of the previous
            # leg's deceleration as drift.
            settled = vprofile.is_settled_hold(t)
            if settled:
                if hold_anchor is None:
                    hold_anchor = fwd
                m.max_hold_drift_m = max(m.max_hold_drift_m, abs(fwd - hold_anchor))
            else:
                hold_anchor = None

            for i in range(data.ncon):
                c = data.contact[i]
                if (c.geom1 == ground and c.geom2 in bumpers) or (
                    c.geom2 == ground and c.geom1 in bumpers
                ):
                    m.nose_strike = True

        m.saturated_cycles = ctl.saturated_cycles
        m.peak_abs_pitch_ref_deg = float(np.degrees(ctl.peak_abs_pitch_ref_rad))

    tau = np.abs(np.asarray(currents)) * KT_NM_PER_A
    m.rms_torque_nm = float(np.sqrt((tau**2).mean()))
    m.return_error_m = abs(float(pos[-1]))

    return ShuttleResult(
        params=params,
        metrics=m,
        plant=plant_summary(model),
        t=np.asarray(ts),
        pos_m=np.asarray(pos),
        pos_cmd_m=np.asarray(pos_cmd),
        v_m_s=np.asarray(vs),
        v_ref_m_s=np.asarray(vrefs),
        pitch_deg=np.asarray(pitches),
        pitch_ref_deg=np.asarray(prefs),
        motor_current_a=np.asarray(currents),
        qpos=np.asarray(states) if states else np.empty(0),
    )
