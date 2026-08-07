"""The engage sequence: how the motor goes from OFF to DRIVING.

THE DEFECT THIS REPLACES
-------------------------
`terrain.py` used to place the board analytically at `r + amp` (a level,
airborne pose) and hand it straight to a running controller. `data.ncon == 0`
at t=0 -- the board was never touching the ground. The accelerometer
correctly read `[0, 0, 0]` (free fall), the complementary filter computed
`atan2(0, -0) = 180 deg`, the controller believed the board was inverted and
saturated at the current rail, and a real crash followed within ~0.25 s.
`use_estimator=False` survived the identical plant, which is what pinned this
as an estimator response to a bad initial condition rather than a control-law
defect.

HOW A REAL ONEWHEEL STARTS
---------------------------
1. Power on -- electronics live, motor not engaged.
2. Rider places the tail foot first; the board rests nose-down on its front
   bumper, in ground contact.
3. Rider places the front foot, activating the front footpad's pressure
   sensors.
4. Rider stands up, rotating the board to level; when it reaches level with
   the rider detected, the motor engages.

At engagement the board is therefore in ground contact, fully loaded, at
rest, and level -- never in free fall, and control never starts on an
unloaded board. This module is the state machine that gates driving the
motor on that sequence, plus the geometry that derives the nose-down rest
pose from the same collision hull `plant.strike_angles_deg` already sweeps.

WHAT IS MODELLED AND WHAT IS NOT
---------------------------------
There is no footpad sensor model here -- that is out of scope. "Rider
present" is a single boolean the caller supplies (`terrain.py`'s `run()`
drives it as a scripted step function: False before the mount event, True
after). Likewise there is no rider-muscle simulation of "standing up": the
frame's own rigid-ballast model (see `plant.py`'s module docstring) has no
means to apply an internal torque that rotates itself, so the scenario
scripts the stand-up as a discrete kinematic event -- see `terrain.run()`'s
own step loop -- rather than deriving it from forces this plant does not have.

WHO USES THIS. `terrain.py` -- the scenario the defect above actually broke.
`hill.py` and `shuttle_run.py` do not exhibit the free-fall defect (their
spawn already sits in contact), and wiring the SAME sequence into them was
tried and reverted: their own settle/route timing is tuned around zero
comparable startup delay, and the ~2.6 s mount+stand-up+settle overhead here
collided with it badly enough to regress both files' own test suites and, via
`shuttle_run.py`, `test_estimator.py` as well. See this session's report for
the numbers; consistency across all three scenarios remains an open follow
-up, not something this module should be assumed to already provide.
"""

from __future__ import annotations

import enum

from .imperfections import STAGE0_PLACEHOLDER


class EngageState(enum.Enum):
    """Minimum states asked for, plus `ARMED` as the waiting room between
    "rider is on" and "conditions for engagement are actually met" -- reads
    better than a two-state machine once there is a real gap between the two
    (rider present but still leaning, say), even though every scenario here
    currently closes that gap in a single simulation step."""

    DISENGAGED = "disengaged"
    ARMED = "armed"
    ENGAGED = "engaged"


#: How near level the outer loop must read before the motor is allowed to
#: engage, radians. **NEEDS A HARDWARE-GATE NUMBER** -- no measured figure
#: exists for "level enough to engage" yet, same convention as
#: `imperfections.STAGE0_PLACEHOLDER`'s ICD Sec 12 placeholders. Chosen here
#: as `rust_controller.DEFAULT_MAX_PITCH_REF_RAD` (5 deg) rather than an
#: unrelated fresh number: it is already the outer loop's own definition of
#: "the most lean the board is ever asked to hold" in normal riding, so
#: "level enough to hand control to that loop" reusing it is at least
#: internally consistent. It is still a CHOICE, not a derivation, and a
#: Stage-0 bench measurement of the real engage sensor's own threshold should
#: replace it.
LEVEL_THRESHOLD_RAD = 0.087   # 5 degrees; see docstring above

#: Wheel speed at or below which the wheel counts as "at rest" for the engage
#: gate, m/s. DERIVED, not chosen: one wheel-rate sensor quantum (1 ERPM =
#: 0.00698 rad/s, ICD Sec 10.5, `imperfections.STAGE0_PLACEHOLDER`) times the
#: model's own effective rolling radius. Below this the real sensor cannot
#: distinguish "moving" from "stationary" at all, so it is the tightest bound
#: that means anything physically rather than an arbitrarily small epsilon.
AT_REST_SPEED_M_S = STAGE0_PLACEHOLDER.wheel_rate_quantum_rad_s * 0.1454

#: Speed below which the heel-lift dismount can fire once the rider is no
#: longer detected, m/s. A real Onewheel's own published figure -- "above
#: ~1 mph a single footpad sensor keeps it engaged; below ~1 mph with one
#: sensor it disengages" -- not a modelling choice. 1 mph = 0.44704 m/s
#: exactly.
DISENGAGE_SPEED_M_S = 0.44704


def transition(
    state: EngageState,
    rider_present: bool,
    pitch_rad: float,
    speed_m_s: float,
    *,
    level_threshold_rad: float = LEVEL_THRESHOLD_RAD,
    at_rest_speed_m_s: float = AT_REST_SPEED_M_S,
    disengage_speed_m_s: float = DISENGAGE_SPEED_M_S,
) -> EngageState:
    """One step of the engage state machine.

    `DISENGAGED`/`ARMED` -> `ENGAGED` requires all three at once: rider
    present, near level, wheel at rest. Evaluated fresh every call rather than
    requiring one call per hop, so a caller whose scripted mount event already
    satisfies all three the instant the rider is detected engages in the same
    step rather than being forced to spend one step "in transit" through
    `ARMED` for no physical reason.

    `ENGAGED` -> `DISENGAGED` is the heel-lift rule: rider no longer detected
    AND below the dismount speed. Losing the rider above that speed keeps the
    single remaining sensor's engagement, matching the real board.

    While disengaged or armed, **the caller must command zero current** --
    this function only says what state the board is in, never drives the
    motor itself.
    """
    if state is EngageState.ENGAGED:
        if (not rider_present) and abs(speed_m_s) < disengage_speed_m_s:
            return EngageState.DISENGAGED
        return EngageState.ENGAGED

    if not rider_present:
        return EngageState.DISENGAGED

    near_level = abs(pitch_rad) <= level_threshold_rad
    at_rest = abs(speed_m_s) <= at_rest_speed_m_s
    if near_level and at_rest:
        return EngageState.ENGAGED
    return EngageState.ARMED


#: A plausible dwell for "rider places the tail foot, then the front foot" --
#: a scenario PACING choice, not a control constant, in the same spirit as
#: `hill.HillParams.settle_s`/`shuttle_run.RAMP_S` (also chosen for pacing
#: rather than measured). It only decides when in sim-time the scripted
#: mount sequence described in the module docstring begins; it does not
#: enter the control law or the engage gate above.
MOUNT_DWELL_S = 0.3

#: How long the scripted "rider stands up, rotating the board to level" takes,
#: seconds. Also a pacing choice, not measured -- but NOT an arbitrary one in
#: one respect: it has to be slow enough that the implied pitch rate
#: (`resting_pitch_deg / STAND_UP_S`) is a plausible gyro reading rather than
#: a discontinuity. At the flat-ground rest angle (~18.6 deg) and this
#: duration that is ~37 deg/s, a believable lean rate. See each scenario's
#: `run()` for why this has to be a RAMP and not an instantaneous snap: the
#: estimator's gyro channel needs a real, continuous rate to integrate, or it
#: is handed a rate discontinuity its filter cannot have anticipated and it
#: engages already wrong by the full rest angle -- which is precisely how the
#: first version of this sequence re-created a milder copy of the defect
#: this whole module exists to fix.
STAND_UP_S = 0.5


def resting_pitch_deg(model, grade_pct: float = 0.0) -> float:
    """Nose-down rest pitch, world frame, degrees -- DERIVED from the same
    collision-hull sweep `plant.strike_angles_deg` already does, not picked.

    THE ARITHMETIC. `plant.strike_angles_deg(model, grade_pct)['nose']` is the
    world pitch (nose-up-positive) at which the front bumper's hull first
    touches a ground plane through the world origin with the given grade's
    normal -- exactly the pose in the module docstring's step 2: nose down,
    resting on the front bumper.

    WHY THE WHEEL STAYS IN CONTACT AT THAT SAME POSE, for free. The wheel
    hinges relative to the frame about the same axis (+Y, lateral) that this
    pitch itself rotates about, and the wheel's collision geometry is
    rotationally symmetric about that axis. Rotating the whole assembly about
    its own spin axis does not change the sagittal silhouette the wheel
    presents to the ground, so the axle's clearance above a flat local ground
    is a function of its WORLD HEIGHT alone, never of pitch. Concretely: the
    axle sits at `r` (flat ground) or `r + amp` (the terrain crest, locally
    flat) regardless of this angle, which is the property
    `strike_angles_deg` itself relies on (it sweeps pitch at a FIXED axle
    position read once via `mj_forward`) and the reason spawning at this
    angle produces a wheel that is STILL exactly tangent, not lifted or
    buried -- the bumper and the wheel touch down together, a tripod, which
    is the real board's resting stance.

    Callers on a locally-flat surface (the terrain crest; hill.py's flat
    ground under rotated gravity) want `grade_pct=0.0` here regardless of
    their own scenario's grade parameter -- the GROUND GEOMETRY at the spawn
    point is flat in both cases; it is gravity or the terrain far from the
    spawn point that is not.
    """
    from .plant import strike_angles_deg

    return float(strike_angles_deg(model, grade_pct)["nose"])
