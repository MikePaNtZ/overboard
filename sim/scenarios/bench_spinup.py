"""Bench-rig identification scenarios — Wave-0 "does the method work at all".

Three independent jobs against `sim/models/bench_rig.xml` (read its header
first; it states what does and does not transfer to the board):

    identify   two-run kt / rotor-inertia fit, bare vs flywheel-loaded
    spindown   viscous (b) / Coulomb (tau_c) friction fit from a decay
    replay     residual of the sim against a measured-hardware CSV

None of this predicts anything about the board. It proves the *identification
method* -- constant-current ramp, spin-down decay, residual-against-hardware --
on a rig small enough to sit on a desk, before it is pointed at a motor twenty
times the size that we do not yet have a bench for.

THE TWO-RUN kt / J_rotor FIT
-----------------------------
Held current, near w=0, friction is a small correction rather than the whole
story:

    kt*i - b*w - tau_c*sign(w) = J*dw/dt

so the initial slope of w(t) is alpha ~= kt*i/J. Two runs at the SAME current
-- bare rotor, then with the flywheels added -- give two equations in two
unknowns once `J_disc` is treated as known. As of #66 that is two goBILDA
3628-0032-0082 flywheels, and "known" means the manufacturer's published
axial inertia (1651 g*cm^2 each), not a weighed-and-measured custom disc --
see `known_disc_inertia_kg_m2` and bench_rig.xml's header for why a
solid-cylinder derivation would be wrong for this part:

    kt*i = J_bare*alpha_1
    kt*i = (J_bare + J_disc)*alpha_2
    =>  kt = J_disc / (i * (1/alpha_2 - 1/alpha_1))
        J_bare = kt*i/alpha_1

The bare variant is built by REGEX-STRIPPING the two `flywheel_a`/`flywheel_b`
bodies out of the XML string and compiling with `mujoco.MjModel.from_xml_string`
-- `MjsGeom`/`MjsBody` has no `.delete()` and hand-rolling `MjSpec` surgery for
two bodies is not worth the fragility it buys. The strip is verified, not
trusted: `build_bare_model` asserts both flywheel geoms are actually gone and
that the resulting joint inertia is lower than the loaded rig's. A silent
failure to strip would make the two runs identical, which is a divide-by-zero
waiting to happen rather than a loud one.

kt IS BIASED BY A SMALL, KNOWN AMOUNT. The algebra above assumes zero
friction at the initial instant; Coulomb does not vanish at w=0, so the fit
actually recovers `kt - tau_c/i`, not kt exactly. Choosing i large enough that
`tau_c/i` is a small fraction of kt (see `IdentifyParams.commanded_current_a`)
keeps that bias under the acceptance threshold rather than pretending it does
not exist. `J_bare`, by contrast, comes out UNBIASED by this same algebra --
the friction term cancels between the two runs exactly, which is a genuine
property of the method and not a coincidence of these particular numbers.

kt WAS ALSO BIASED BY A MUCH LARGER AMOUNT, AND THAT ONE IS FIXED HERE
-----------------------------------------------------------------------
The Coulomb bias above is ~0.5%. There was a second bias roughly a HUNDRED
times larger, and the asymmetry between what it does to kt and what it does to
`J_bare` is worth understanding because it is the same asymmetry as above
running the other way.

The current does not arrive when commanded. Under `STAGE0_PLACEHOLDER` there
is 1 ms of actuation delay and a 1 ms current-loop time constant, against what
used to be a 5 ms fit window. So for the first fifth of that window there was
little or no torque at the shaft, and the 500 Hz feedback update laid a 2 ms
staircase on top. Both ramps were suppressed by roughly the same factor
k ~ 0.5, and:

  * `J_bare = kt*i/alpha_1` -- k appears in numerator and denominator, so it
    CANCELS. J_bare was accurate to 0.1% the whole time.
  * `kt = J_disc / (i*(1/alpha_2 - 1/alpha_1))` -- scales LINEARLY with k. It
    came out at 0.02493 against a 0.05026 nameplate: 50.4% low, a factor of
    two, on the default profile.

The slope RATIO survived (7.70 with imperfections, 7.68 without), which is why
the two-run structure still worked and only the absolute scale was lost. That
is also why this hid so well: everything self-consistent stayed self-consistent
and only the one number with an external reference was wrong.

THE FIX HAS TWO PARTS, AND BOTH ARE ABOUT NOT ASSUMING THE COMMAND WAS OBEYED.

1. FIT WHERE THE RAMP IS ACTUALLY A RAMP. The window now starts when the
   measured current has settled to within `settle_fraction` of the command,
   detected FROM THE CURRENT TRACE rather than computed from the profile --
   see `settle_time_s`. That distinction matters: on hardware there is no
   profile object to consult, only a current sensor, so a data-driven rule is
   the only one that transfers. The rig's whole job is to characterise this
   signal path, so it must not require the answer as an input.

2. FIT AGAINST THE MEASURED CURRENT, NOT THE COMMANDED ONE. `kt*i = J*alpha`
   is a statement about the current that actually flowed. Using the command
   assumes a perfect current loop, which is precisely the assumption the bench
   exists to test. The mean measured current over the fit window is the
   consistent estimator here, and it makes the fit robust to any current-path
   imperfection rather than only to the two currently modelled.

Result: kt error on the default profile goes 50.4% -> 0.13%, and both
profiles now land on the same ~0.1-0.5% floor, which IS the Coulomb bias above
-- i.e. the only bias left is the one the algebra predicts. `test_identify_
recovers_kt_under_the_stage0_profile` pins this so it cannot regress; its
absence is why a factor-of-two error shipped green.

RAISED BY Digital Content Production, who found it while building the render
and correctly declined to quote a fitted kt in published material until it was
resolved. The diagnostic that should have caught it did fire -- R^2 fell from
1.0000 to 0.6927 -- but nothing asserted it, so the suite stayed green. Both
R^2 values are now floored in the tests.

SPIN-DOWN: b AND tau_c ARE FIT SEPARATELY, NOT LUMPED
------------------------------------------------------
A single lumped friction coefficient fits a decelerating flywheel badly:
Coulomb dominates at low speed, viscous at high speed, and a lumped term is
neither. It shows up as a residual that changes SIGN across the speed range --
the diagnostic this module reports (`lumped_residual_changes_sign`) rather
than asserts and trusts. The two-term fit instead regresses the logged
acceleration directly against `[w, sign(w)]`:

    J*dw/dt = -b*w - tau_c*sign(w)
    =>  qacc = -(b/J)*w - (tau_c/J)*sign(w)

`J` is read directly off the compiled model (`_joint_inertia`) rather than
re-derived here -- that is mode 1's job, not this one's.

SPIN-DOWN HAD THE SAME WINDOWING BUG AS identify(), AND IT WAS WORSE (#68)
---------------------------------------------------------------------------
`identify()` opens its fit window once the measured current has settled INTO
a step (`settle_time_s`). Until this fix, `spindown()` had no equivalent for
the mirror-image transient: the command drops to zero, but under
`STAGE0_PLACEHOLDER` the actuation delay and current-loop lag mean the
current actually delivered decays away from `spin_up_current_a` rather than
vanishing instantly. For the first few milliseconds of the "decay", the shaft
is still seeing several amps -- and unlike `identify()`'s ramp, which is
merely suppressed by a roughly constant factor, this puts a torque transient
an order of magnitude larger than the genuine friction deceleration straight
into a regression whose design matrix (`[w, sign(w)]`) has no column to
absorb it. Measured result: R^2 ~ 0.002 under the default profile -- noise,
not a curve (issue #68, raised by Senior Controls while dry-running the
Stage-0B runbook, `spindown()` being this role's file).

The fix is the same shape as `identify()`'s, mirrored for a falling rather
than a rising current: `decay_settle_time_s` waits for the measured current
to decay to within `settle_fraction` of zero and STAY there, data-driven off
the current trace exactly like `settle_time_s` -- there is no profile object
on real hardware, only a current sensor, and this rig's whole job is to
characterise that signal path. Both the two-term and the lumped-fit
diagnostic now run over the post-settle window only. On `IDEAL` the window
opens at the very first sample (nothing to wait for) so behaviour there is
unchanged; `test_spindown_recovers_friction_terms_under_the_stage0_profile`
pins the STAGE0 recovery so this cannot regress silently the way the original
bug did.

No published friction figure was ever derived through the STAGE0 path: the
only spindown assertions in this suite ran on `IDEAL`, and the Stage-0B
runbook dry run (`scripts/stage0b_runbook.py::step_coast_down`) used `IDEAL`
as an explicit, documented workaround pending this fix. There is nothing to
correct downstream -- only the workaround to remove, which is Senior
Controls' file to change.

CATEGORY ERROR GUARD
---------------------
This bench motor's kt is ~0.05 N*m/A (a 190 kv 6374). The onewheel hub
motor's is `plant.KT_NM_PER_A` = 0.7 (an unfitted guess, an order of magnitude
different machine). If a fitted bench kt ever lands within ~20% of
`plant.KT_NM_PER_A` that is a coincidence, not a validation -- nothing in this
module writes a bench-fitted value into `plant.py`, and nothing should.

WHAT ImperfectionProfile MEANS HERE
-------------------------------------
`STAGE0_PLACEHOLDER` (the default) degrades the COMMANDED current through the
same actuation delay / current-loop lag the board's own drive has, and
degrades the LOGGED shaft rate through the same quantisation / update rate --
because a real bench identification only ever sees the sensor's version of
events, never MuJoCo truth. `IDEAL` is for bring-up and for the numeric
acceptance tests, where the method itself is what is being checked and the
signal path must not be the thing under test.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import re
from dataclasses import asdict, dataclass, field
from pathlib import Path

import mujoco
import numpy as np

from .imperfections import IDEAL, STAGE0_PLACEHOLDER, ImperfectionProfile, ImperfectionState

REPO = Path(__file__).resolve().parents[2]
MODEL_PATH = REPO / "sim" / "models" / "bench_rig.xml"
OUT_DIR = REPO / "sim" / "out" / "bench"

#: Nameplate kt for the bench motor (190 kv 6374): kt = 9.5493 / kv. This is
#: the GROUND TRUTH the sim uses to turn commanded current into the shaft
#: torque `data.ctrl` actually wants -- i.e. it stands in for the bench
#: motor's real electromagnetic torque constant, which `identify` then tries
#: to recover from the response, exactly as a real bench would from measured
#: current and speed.
#:
#: NOT plant.KT_NM_PER_A. That is the onewheel hub motor's (guessed) kt,
#: ~0.7 N*m/A -- a different machine by more than an order of magnitude. See
#: the module docstring's category-error guard.
NAMEPLATE_KT_NM_PER_A = 9.5493 / 190.0

#: Flywheel identity, independent of the compiled model -- the manufacturer's
#: published mass and axial inertia for the goBILDA 3628-0032-0082 (#66), not
#: derived from a radius/thickness/density geometry guess (a same-mass solid
#: disc would be ~30% low; see bench_rig.xml's header). Kept as named
#: constants rather than read back off the compiled model, because reading it
#: off the model would make `J_disc` circular with the very quantity the
#: two-run fit is solving for.
FLYWHEEL_COUNT = 2  # both purchased units are on the shaft; see #65
FLYWHEEL_MASS_KG = 0.152
FLYWHEEL_INERTIA_KG_M2 = 1.651e-4  # manufacturer, about the shaft axis, per unit

#: Threshold for `current_tracking_ok`. Past this the drive has derated by
#: more than plausible measurement jitter -- ICD noise floors on reported
#: current are a few hundred mA -- and every other residual in a replay run
#: is meaningless until this is understood.
CURRENT_TRACKING_TOLERANCE_A = 0.5

#: Matches the whole `<body name="flywheel_a">...</body>` /
#: `<body name="flywheel_b">...</body>` blocks, not just a geom -- since #66,
#: each flywheel's manufacturer-sourced inertia lives on an `<inertial>` at
#: body scope, so a self-closing-geom regex can no longer isolate it. The
#: index mark is nested inside `flywheel_b` (see bench_rig.xml's header), so
#: stripping these two bodies removes it too, by construction rather than by
#: a second regex that could drift out of sync.
_FLYWHEEL_BODY_RE = re.compile(r'\s*<body name="flywheel_[ab]".*?</body>', re.DOTALL)

#: The index mark, kept as its own regex even though `_FLYWHEEL_BODY_RE`
#: already removes it (it is nested inside `flywheel_b`) -- this one isolates
#: JUST the mark, for `test_removing_the_index_mark_changes_no_fitted_quantity`,
#: which strips only this geom to confirm it is really massless rather than
#: assuming it.
#:
#: It is massless with collision off, so `J_bare` and every fitted number are
#: unaffected either way; this is model coherence, not correctness. But a plant
#: model that renders as a floating decal is a plant model that will eventually
#: be believed about something else. The mark exists so rotation is legible, and
#: a bare on-axis rotor has nothing to make legible.
#:
#: Raised by Digital Content Production, who hit it rendering the bare variant.
_INDEX_GEOM_RE = re.compile(r'<geom name="index"[^>]*/>')


def known_disc_inertia_kg_m2() -> float:
    """`J_disc` from the manufacturer's published figure, NOT from geometry.

    Two goBILDA 3628-0032-0082 flywheels (#66), each 1651 g*cm^2 about the
    shaft axis per the datasheet. They sum directly because both are coaxial
    with the shaft: offsets along the axis of rotation contribute nothing to
    inertia about that axis (see bench_rig.xml's header note on the
    eccentric-offset defect this replaced) -- an off-axis disc would need a
    parallel-axis correction this does not have.
    """
    return FLYWHEEL_COUNT * FLYWHEEL_INERTIA_KG_M2


def load_model(path: Path = MODEL_PATH) -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_path(str(path))


def _joint_inertia(model: mujoco.MjModel) -> float:
    """Joint-space inertia about the shaft hinge, kg*m^2.

    The rig has one hinge and no other DOF, so the (1x1) mass matrix IS this
    number -- exactly the `J` the `alpha ~= kt*i/J` approximation assumes.
    """
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    full_m = np.zeros((model.nv, model.nv))
    mujoco.mj_fullM(model, data, full_m)
    return float(full_m[0, 0])


def strip_flywheel_geom(xml: str) -> str:
    """Remove the two flywheel bodies -- and the index mark nested inside
    `flywheel_b` -- from the MJCF string.

    A regex, not `MjSpec` surgery: `MjsGeom`/`MjsBody` has no `.delete()`, and
    building the bare variant through spec editing is not worth the fragility
    for two bodies. The caller MUST verify the strip actually worked -- see
    `build_bare_model` -- because a silent no-op here would make the bare and
    loaded runs identical and turn the two-run fit into a divide-by-zero.
    """
    stripped, n = _FLYWHEEL_BODY_RE.subn("", xml)
    if n != 2:
        raise RuntimeError(
            f"expected exactly two <body name=\"flywheel_[ab]\">...</body> blocks "
            f"to strip, found {n}. The bench_rig.xml flywheel markup may have "
            "changed; update the regex."
        )
    return stripped


def build_bare_model(xml_text: str | None = None) -> mujoco.MjModel:
    """The bare-rotor variant used as run 1 of the two-run identification.

    Verified, not trusted: asserts both flywheel geoms are actually gone and
    that the resulting joint inertia is lower than a loaded rig's would be.
    Either assertion failing means the strip silently did nothing.
    """
    xml_text = xml_text if xml_text is not None else MODEL_PATH.read_text()
    bare = mujoco.MjModel.from_xml_string(strip_flywheel_geom(xml_text))
    assert mujoco.mj_name2id(bare, mujoco.mjtObj.mjOBJ_GEOM, "flywheel_a_disc") < 0, (
        "flywheel_a geom is still present after stripping -- the regex did not match"
    )
    assert mujoco.mj_name2id(bare, mujoco.mjtObj.mjOBJ_GEOM, "flywheel_b_disc") < 0, (
        "flywheel_b geom is still present after stripping -- the regex did not match"
    )
    assert mujoco.mj_name2id(bare, mujoco.mjtObj.mjOBJ_GEOM, "index") < 0, (
        "index mark survived the flywheel strip -- it is nested inside flywheel_b, "
        "so the bare model would render a decal floating in mid-air"
    )
    return bare


def settle_time_s(
    t: np.ndarray, i_measured: np.ndarray, commanded_a: float, fraction: float,
) -> float:
    """First instant after which the measured current STAYS within `fraction`
    of the command.

    Data-driven on purpose. The obvious alternative -- compute
    `actuation_delay_s + 5*current_loop_tau_s` from the profile -- gives the
    same answer in sim and is useless on hardware, where there is no profile
    object, only a current sensor. Since characterising this very signal path
    is the rig's job, a rule that needs the answer as an input is circular.

    "Stays within" rather than "first reaches": a staircased or noisy current
    can clip the threshold once on the way up. The last violation is what
    bounds the transient, so the search runs from the end.
    """
    ok = np.asarray(i_measured) >= fraction * commanded_a
    if not ok.any():
        raise ValueError(
            f"measured current never reached {fraction:.0%} of the commanded "
            f"{commanded_a} A. Either the drive is saturating or the run is too "
            "short to contain a settled ramp; the fit would be of the transient."
        )
    violations = np.flatnonzero(~ok)
    idx = 0 if violations.size == 0 else int(violations[-1]) + 1
    if idx >= len(t):
        raise ValueError(
            "measured current settles only at the very last sample -- no ramp "
            "left to fit. Increase IdentifyParams.sim_seconds."
        )
    return float(t[idx])


def decay_settle_time_s(
    t: np.ndarray, i_measured: np.ndarray, initial_current_a: float, fraction: float,
) -> float:
    """First instant after which the measured current, released from
    `initial_current_a` when the command drops to zero, STAYS within
    `1 - fraction` of zero.

    The decay-side twin of `settle_time_s` (see #68): that one waits for
    current to rise INTO a held command; this one waits for it to fall AWAY
    from one, because the actuation delay and current-loop lag delay the
    fall exactly as they delay the rise. Same "stays within" search from the
    end for the same reason -- a staircased or noisy current can cross the
    threshold once on the way down and bounce back before it has genuinely
    settled, and the LAST crossing is what bounds the transient.

    Data-driven on purpose, exactly like `settle_time_s`: on hardware there
    is no profile object to consult for how long the current loop takes to
    unwind, only a current sensor, and characterising that signal path is
    this rig's job.
    """
    threshold = (1.0 - fraction) * abs(initial_current_a)
    ok = np.abs(np.asarray(i_measured)) <= threshold
    if not ok.any():
        raise ValueError(
            f"measured current never decayed within {1.0 - fraction:.0%} of "
            f"{initial_current_a} A of zero. Either the current-loop lag "
            "exceeds SpindownParams.decay_s or the run is too short to "
            "contain a settled decay."
        )
    violations = np.flatnonzero(~ok)
    idx = 0 if violations.size == 0 else int(violations[-1]) + 1
    if idx >= len(t):
        raise ValueError(
            "measured current settles only at the very last sample -- no "
            "decay left to fit friction against. Increase SpindownParams.decay_s."
        )
    return float(t[idx])


def _fit_line(
    t: np.ndarray, y: np.ndarray, window_s: float, start_s: float = 0.0,
) -> tuple[float, float]:
    """Least-squares slope of `y` vs `t` over `start_s <= t <= window_s`, plus R^2.

    A window fit, not a two-sample finite difference -- so a bad window (too
    long, catching friction or a current cutback) is visible in the R^2
    rather than silently baked into a slope from two noisy points.

    `start_s` defaults to 0.0 so the pre-existing two-argument call is
    unchanged, but `identify` always passes an explicit start: fitting from
    t=0 through an actuation delay and a current-loop lag is what produced a
    factor-of-two kt error (see the module docstring).
    """
    mask = (t >= start_s) & (t <= window_s)
    tt, yy = t[mask], y[mask]
    if len(tt) < 2:
        raise ValueError(
            f"fit window [{start_s}s, {window_s}s] contains fewer than 2 samples"
        )
    design = np.vstack([tt, np.ones_like(tt)]).T
    (slope, intercept), *_ = np.linalg.lstsq(design, yy, rcond=None)
    pred = slope * tt + intercept
    ss_res = float(np.sum((yy - pred) ** 2))
    ss_tot = float(np.sum((yy - yy.mean()) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0.0 else 1.0
    return float(slope), float(r2)


def _run_constant_current(
    model: mujoco.MjModel, current_a: float, seconds: float, profile: ImperfectionProfile,
) -> tuple[np.ndarray, np.ndarray]:
    """Drive `model` with a held commanded current; log time, SENSED shaft rate,
    and the current ACTUALLY delivered.

    The sensed rate runs through the same quantisation and update-rate a real
    bench identification would see, not MuJoCo truth.

    The delivered current is returned because `kt*i = J*alpha` is a statement
    about the current that flowed, not the one that was asked for. Returning
    only the command would force every caller to assume a perfect current loop
    -- the assumption this rig exists to test.
    """
    data = mujoco.MjData(model)
    dt = float(model.opt.timestep)
    n_steps = int(round(seconds / dt))
    imp = ImperfectionState(profile=profile, dt_s=dt)
    mujoco.mj_forward(model, data)

    ts, ws, i_meas = [], [], []
    for _ in range(n_steps):
        current = imp.apply_current(current_a)
        data.ctrl[0] = current * NAMEPLATE_KT_NM_PER_A
        mujoco.mj_step(model, data)
        ts.append(float(data.time))
        ws.append(imp.wheel_rate(float(data.qvel[0]), float(data.time)))
        i_meas.append(current)
    return np.asarray(ts), np.asarray(ws), np.asarray(i_meas)


# ---------------------------------------------------------------------------
# Mode 1 -- identify
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class IdentifyParams:
    """Everything that defines one identify run. Frozen: the fitted numbers
    are only meaningful alongside the exact parameters that produced them."""

    commanded_current_a: float = 20.0
    """Held constant across both runs. Large enough that the Coulomb bias in
    the kt fit (tau_c/i, see the module docstring) stays under ~0.5% of kt
    while remaining well inside the rig's 40 A / 2.0 N*m envelope."""

    fit_span_s: float = 0.014
    """Length of the alpha fit window, measured FROM the settle instant rather
    than from t=0 -- see `settle_time_s` and the module docstring.

    Replaces the old `fit_window_s = 0.005`, which started at zero and so spent
    its first fifth inside the actuation delay and current-loop lag. That is
    the factor-of-two kt error. The name changed deliberately: the semantics
    are different, and a silently redefined `fit_window_s` would be worse than
    a loud `AttributeError` for any caller that reaches for it.

    14 ms is chosen against measurement, not taste. Too short and the 500 Hz
    sensed-rate staircase leaves too few distinct updates to fit a slope
    through (10 ms gives five); too long and viscous drag, which grows with w
    unlike Coulomb, starts bending the ramp -- visible as J_bare error creeping
    up with span. Measured kt error by span on the default profile: 0.23% at
    10 ms, 0.13% at 14 ms, 0.31% at 20 ms, 0.95% at 30 ms."""

    settle_fraction: float = 0.99
    """Fraction of the commanded current the measured current must reach, and
    stay at, before the fit window opens.

    0.99 is ~5 time constants of the current loop. 0.95 would open the window
    ~3 tau in, while the ramp is still visibly curved; 0.999 buys nothing and
    costs samples off the front of a finite run."""

    sim_seconds: float = 0.04
    """Long enough to contain the settle transient AND the full fit span with
    margin (7 ms + 14 ms on the default profile). Was 0.02, which only just
    cleared the old 5 ms window; the fit now starts later, so the run has to
    end later. `identify` asserts the window actually fits inside the data
    rather than letting a short run silently truncate it."""


@dataclass
class IdentifyMetrics:
    kt_fit_nm_per_a: float = 0.0
    j_bare_fit_kg_m2: float = 0.0
    j_disc_known_kg_m2: float = 0.0
    alpha_bare_rad_s2: float = 0.0
    alpha_loaded_rad_s2: float = 0.0
    r2_bare: float = 0.0
    r2_loaded: float = 0.0

    #: The fit window actually used, in seconds. DERIVED FROM THE DATA, not a
    #: parameter, so it has to be reported or the fit is not reproducible: the
    #: start comes from where the measured current settled (`settle_time_s`).
    #: Anything cross-checking these slopes must fit over the same interval --
    #: comparing a post-settle slope against a from-zero one is a ~40%
    #: "divergence" that is really two different signals.
    fit_start_s: float = 0.0
    fit_end_s: float = 0.0

    #: Mean current that actually flowed over the fit window. This, not
    #: `commanded_current_a`, is the `i` in `kt*i = J*alpha` -- see the module
    #: docstring on why using the command cost a factor of two.
    i_measured_mean_a: float = 0.0

    #: Against the nameplate/geometry references -- informational, NOT the
    #: acceptance criterion for a bare-motor J (there is no independent
    #: reference for that; see the module docstring on why J_bare comes out
    #: unbiased rather than being validated against a "known" number).
    kt_error_vs_nameplate_pct: float = 0.0

    imperfection_profile_id: str = ""
    commanded_current_a: float = 0.0
    model_sha256: str = ""
    mujoco_version: str = ""
    timestep_s: float = 0.0


@dataclass
class IdentifyResult:
    params: IdentifyParams
    metrics: IdentifyMetrics
    t_bare: np.ndarray = field(default_factory=lambda: np.empty(0))
    w_bare: np.ndarray = field(default_factory=lambda: np.empty(0))
    t_loaded: np.ndarray = field(default_factory=lambda: np.empty(0))
    w_loaded: np.ndarray = field(default_factory=lambda: np.empty(0))

    def to_dict(self) -> dict:
        return {"mode": "identify", "params": asdict(self.params), "metrics": asdict(self.metrics)}


def identify(
    params: IdentifyParams | None = None,
    loaded_model: mujoco.MjModel | None = None,
    profile: ImperfectionProfile = STAGE0_PLACEHOLDER,
) -> IdentifyResult:
    """Bare-vs-loaded two-run kt / J_bare fit. See the module docstring."""
    params = params or IdentifyParams()
    loaded_model = loaded_model or load_model()
    bare_model = build_bare_model()
    assert _joint_inertia(bare_model) < _joint_inertia(loaded_model), (
        "bare rig has no less inertia than the loaded rig -- the flywheel strip "
        "did not actually remove any mass"
    )

    t_bare, w_bare, i_bare = _run_constant_current(
        bare_model, params.commanded_current_a, params.sim_seconds, profile
    )
    t_loaded, w_loaded, i_loaded = _run_constant_current(
        loaded_model, params.commanded_current_a, params.sim_seconds, profile
    )

    # Open the window only once the current has settled in BOTH runs. Taking the
    # later of the two keeps the two fits over the same interval, which is what
    # makes the slope ratio -- the quantity the whole two-run method rests on --
    # a comparison of like with like.
    i_cmd = params.commanded_current_a
    t_start = max(
        settle_time_s(t_bare, i_bare, i_cmd, params.settle_fraction),
        settle_time_s(t_loaded, i_loaded, i_cmd, params.settle_fraction),
    )
    t_end = t_start + params.fit_span_s
    if t_end > float(t_bare[-1]):
        raise ValueError(
            f"fit window [{t_start*1e3:.1f}, {t_end*1e3:.1f}] ms runs past the "
            f"{float(t_bare[-1])*1e3:.1f} ms run. Increase sim_seconds; a truncated "
            "window would silently fit fewer samples than asked for."
        )

    alpha1, r2_1 = _fit_line(t_bare, w_bare, t_end, start_s=t_start)
    alpha2, r2_2 = _fit_line(t_loaded, w_loaded, t_end, start_s=t_start)

    # The current that actually flowed over the fit window, not the command.
    # Averaged across both runs: they share a profile and a held command, so the
    # traces are the same to within the loop's own settling, and one number keeps
    # the two-equation algebra below in the form the docstring derives.
    def _mean_i(t, i_meas):
        return float(np.asarray(i_meas)[(t >= t_start) & (t <= t_end)].mean())

    i_eff = 0.5 * (_mean_i(t_bare, i_bare) + _mean_i(t_loaded, i_loaded))

    j_disc = known_disc_inertia_kg_m2()
    kt_fit = j_disc / (i_eff * (1.0 / alpha2 - 1.0 / alpha1))
    j_bare_fit = kt_fit * i_eff / alpha1

    m = IdentifyMetrics(
        kt_fit_nm_per_a=kt_fit,
        j_bare_fit_kg_m2=j_bare_fit,
        j_disc_known_kg_m2=j_disc,
        alpha_bare_rad_s2=alpha1,
        alpha_loaded_rad_s2=alpha2,
        r2_bare=r2_1,
        r2_loaded=r2_2,
        kt_error_vs_nameplate_pct=abs(kt_fit - NAMEPLATE_KT_NM_PER_A) / NAMEPLATE_KT_NM_PER_A * 100.0,
        fit_start_s=t_start,
        fit_end_s=t_end,
        i_measured_mean_a=i_eff,
        imperfection_profile_id=profile.profile_id,
        commanded_current_a=i_cmd,
        model_sha256=hashlib.sha256(MODEL_PATH.read_bytes()).hexdigest()[:16],
        mujoco_version=mujoco.__version__,
        timestep_s=float(loaded_model.opt.timestep),
    )
    return IdentifyResult(
        params=params, metrics=m, t_bare=t_bare, w_bare=w_bare, t_loaded=t_loaded, w_loaded=w_loaded,
    )


# ---------------------------------------------------------------------------
# Mode 2 -- spindown
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class SpindownParams:
    spin_up_current_a: float = 20.0
    spin_up_s: float = 0.05
    """Just enough to get the flywheel moving before current is cut -- the
    spin-up itself is not measured."""

    decay_s: float = 2.0
    """Long enough to cover a wide speed range (this rig's damping time
    constant is ~18 s, so the decay is nowhere near complete, but the speed
    swept is still wide enough to separate the two friction terms)."""

    settle_fraction: float = 0.9999
    """Fraction of the way from `spin_up_current_a` to zero the measured
    current must decay -- and stay -- before the friction-fit window opens.

    Mirrors `IdentifyParams.settle_fraction` (see #68), but tighter, and for
    a reason specific to this fit: the friction regression's design matrix is
    `[w, sign(w)]`, with no column for a current term, so ANY residual
    current-loop torque left in the window is pure unmodelled signal, not a
    small bias the way Coulomb is for `identify()`'s kt. `identify()` can
    afford 0.99 because its 14 ms window is itself tiny; this decay runs for
    `decay_s` (2 s by default), so there is no cost to waiting for the
    transient to genuinely die out.

    Measured on the default profile (fraction -> window opens at / R^2 /
    b error / tau_c error):

        0.99     ->  7.0 ms  /  R^2=0.635   /  4.51%   /  1.95%
        0.999    -> 10.0 ms  /  R^2=0.996   /  0.389%  /  0.165%
        0.9999   -> 12.5 ms  /  R^2=0.9999  /  0.043%  /  0.015%
        0.99999  -> 15.5 ms  /  R^2=0.99999 /  0.0049% /  0.0058%

    0.99 is what `identify()` uses and is not nearly tight enough here --
    0.2 A of residual current (1% of the 20 A spin-up) still produces ~5.4
    rad/s^2 of unmodelled torque, comparable to the friction deceleration
    itself. 0.9999 clears the same R^2 > 0.999 bar the ideal run holds, with
    a wide margin still visible in the 0.99999 row; going tighter buys very
    little further and starts trading away decay data for no real gain."""


@dataclass
class SpindownMetrics:
    w0_rad_s: float = 0.0
    w_end_rad_s: float = 0.0

    b_fit_nm_s_per_rad: float = 0.0
    tau_c_fit_nm: float = 0.0
    r2: float = 0.0

    #: Where the friction-fit window actually started, seconds from current-cut
    #: (t=0 of the decay). DERIVED FROM THE DATA (`decay_settle_time_s`), not a
    #: parameter -- reported for the same reason `identify()` reports
    #: `fit_start_s`: a fit is not reproducible unless the window it used is
    #: stated alongside it. On `IDEAL` this is ~0 (nothing to wait for); under
    #: `STAGE0_PLACEHOLDER` it is the time the current-loop transient took to
    #: unwind. See #68.
    fit_start_s: float = 0.0

    #: What the model was actually built with -- read off the compiled model
    #: rather than hardcoded, so a change to bench_rig.xml's placeholders
    #: changes what this is checked against automatically.
    b_placeholder_nm_s_per_rad: float = 0.0
    tau_c_placeholder_nm: float = 0.0
    b_error_pct: float = 0.0
    tau_c_error_pct: float = 0.0

    #: The lumped-fit diagnostic (module docstring): a single viscous-only
    #: term absorbs part of the Coulomb term into a biased b, and the
    #: residual it leaves behind changes SIGN between low and high speed.
    #: A diagnostic that never changes sign is not doing its job.
    lumped_b_only_nm_s_per_rad: float = 0.0
    lumped_residual_low_speed_rad_s2: float = 0.0
    lumped_residual_high_speed_rad_s2: float = 0.0
    lumped_residual_changes_sign: bool = False

    imperfection_profile_id: str = ""
    model_sha256: str = ""
    mujoco_version: str = ""
    timestep_s: float = 0.0


@dataclass
class SpindownResult:
    params: SpindownParams
    metrics: SpindownMetrics
    t: np.ndarray = field(default_factory=lambda: np.empty(0))
    w: np.ndarray = field(default_factory=lambda: np.empty(0))
    qacc: np.ndarray = field(default_factory=lambda: np.empty(0))

    def to_dict(self) -> dict:
        return {"mode": "spindown", "params": asdict(self.params), "metrics": asdict(self.metrics)}


def spindown(
    params: SpindownParams | None = None,
    model: mujoco.MjModel | None = None,
    profile: ImperfectionProfile = STAGE0_PLACEHOLDER,
) -> SpindownResult:
    """Spin up, cut current to zero, log the decay, fit b and tau_c
    separately. See the module docstring for why they are not lumped."""
    params = params or SpindownParams()
    model = model or load_model()
    data = mujoco.MjData(model)
    dt = float(model.opt.timestep)
    imp = ImperfectionState(profile=profile, dt_s=dt)
    mujoco.mj_forward(model, data)

    n_spin = int(round(params.spin_up_s / dt))
    for _ in range(n_spin):
        current = imp.apply_current(params.spin_up_current_a)
        data.ctrl[0] = current * NAMEPLATE_KT_NM_PER_A
        mujoco.mj_step(model, data)

    n_decay = int(round(params.decay_s / dt))
    t0 = float(data.time)
    ts, ws, accs, i_meas = [], [], [], []
    for _ in range(n_decay):
        current = imp.apply_current(0.0)
        data.ctrl[0] = current * NAMEPLATE_KT_NM_PER_A
        mujoco.mj_step(model, data)
        ts.append(float(data.time) - t0)
        ws.append(imp.wheel_rate(float(data.qvel[0]), float(data.time)))
        accs.append(float(data.qacc[0]))
        i_meas.append(current)

    t = np.asarray(ts)
    w = np.asarray(ws)
    acc = np.asarray(accs)
    i_dec = np.asarray(i_meas)
    j = _joint_inertia(model)

    # The friction fit must not see the current-loop's own unwind -- see the
    # module docstring (#68). Data-driven, exactly like identify()'s settle
    # window: wait for the MEASURED current, not the profile, to say the
    # transient is over.
    t_start = decay_settle_time_s(t, i_dec, params.spin_up_current_a, params.settle_fraction)
    fit_mask = t >= t_start
    w_fit = w[fit_mask]
    acc_fit = acc[fit_mask]

    # Two-term fit: qacc = -(b/J)*w - (tau_c/J)*sign(w), over the settled
    # window only.
    sgn = np.sign(w_fit)
    design = np.vstack([w_fit, sgn]).T
    (c1, c2), *_ = np.linalg.lstsq(design, acc_fit, rcond=None)
    b_fit = -c1 * j
    tau_c_fit = -c2 * j
    pred = design @ np.array([c1, c2])
    ss_res = float(np.sum((acc_fit - pred) ** 2))
    ss_tot = float(np.sum((acc_fit - acc_fit.mean()) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0.0 else 1.0

    # Lumped (viscous-only) fit -- the defect the two-term fit replaces. Same
    # settled window, so the diagnostic is a fair comparison against the
    # two-term fit above rather than against a different slice of the data.
    b_lump = -float(np.dot(w_fit, acc_fit) / np.dot(w_fit, w_fit))
    resid_lump = acc_fit - (-b_lump * w_fit)
    order = np.argsort(w_fit)
    k = max(1, len(order) // 20)  # slowest / fastest 5% of the logged speeds
    resid_lo = float(resid_lump[order[:k]].mean())
    resid_hi = float(resid_lump[order[-k:]].mean())

    b_true = float(model.dof_damping[0])
    tau_c_true = float(model.dof_frictionloss[0])

    m = SpindownMetrics(
        w0_rad_s=float(w[0]),
        w_end_rad_s=float(w[-1]),
        b_fit_nm_s_per_rad=b_fit,
        tau_c_fit_nm=tau_c_fit,
        r2=r2,
        fit_start_s=t_start,
        b_placeholder_nm_s_per_rad=b_true,
        tau_c_placeholder_nm=tau_c_true,
        b_error_pct=abs(b_fit - b_true) / b_true * 100.0 if b_true else 0.0,
        tau_c_error_pct=abs(tau_c_fit - tau_c_true) / tau_c_true * 100.0 if tau_c_true else 0.0,
        lumped_b_only_nm_s_per_rad=b_lump,
        lumped_residual_low_speed_rad_s2=resid_lo,
        lumped_residual_high_speed_rad_s2=resid_hi,
        lumped_residual_changes_sign=(resid_lo > 0.0) != (resid_hi > 0.0),
        imperfection_profile_id=profile.profile_id,
        model_sha256=hashlib.sha256(MODEL_PATH.read_bytes()).hexdigest()[:16],
        mujoco_version=mujoco.__version__,
        timestep_s=dt,
    )
    return SpindownResult(params=params, metrics=m, t=t, w=w, qacc=acc)


# ---------------------------------------------------------------------------
# Mode 3 -- replay
# ---------------------------------------------------------------------------

_REPLAY_COLUMNS = ("t_s", "commanded_current_a", "reported_current_a", "shaft_rate_rad_s")


@dataclass
class ReplayMetrics:
    """Residual of the sim against a measured-hardware CSV. Flat and
    JSON-serialisable, for the metrics artifact."""

    rms_residual_rad_s: float = 0.0
    max_residual_rad_s: float = 0.0

    #: The FIRST 20% of the record -- an inertia/kt error shows up early, a
    #: friction error late, and lumping them together hides which one it is.
    rms_residual_early_rad_s: float = 0.0
    max_residual_early_rad_s: float = 0.0

    #: The cutback detector. If the drive silently derated, the sim is being
    #: asked to reproduce a torque the hardware never applied, and every
    #: residual above is meaningless.
    current_rms_divergence_a: float = 0.0
    current_max_divergence_a: float = 0.0
    current_tracking_ok: bool = False

    n_samples: int = 0
    imperfection_profile_id: str = ""
    model_sha256: str = ""
    mujoco_version: str = ""
    timestep_s: float = 0.0

    def to_dict(self) -> dict:
        return asdict(self)


def _read_replay_csv(path: Path) -> dict[str, np.ndarray]:
    if not path.exists():
        raise FileNotFoundError(
            f"replay CSV not found: {path}. Expected columns: {', '.join(_REPLAY_COLUMNS)}"
        )
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        fieldnames = reader.fieldnames or []
        missing = [c for c in _REPLAY_COLUMNS if c not in fieldnames]
        if missing:
            raise ValueError(
                f"{path} is missing column(s) {missing}; expected {', '.join(_REPLAY_COLUMNS)}"
            )
        rows = list(reader)
    if not rows:
        raise ValueError(f"{path} has a header but no data rows")
    return {c: np.array([float(r[c]) for r in rows], dtype=float) for c in _REPLAY_COLUMNS}


def replay(
    csv_path: str | Path,
    profile: ImperfectionProfile = STAGE0_PLACEHOLDER,
    model: mujoco.MjModel | None = None,
) -> ReplayMetrics:
    """Drive the sim with the RECORDED commanded current (sample-and-hold
    between samples) and report the residual against the recorded shaft rate.

    `model` defaults to the committed bench_rig.xml; a caller with a fitted
    variant (different rotor density, say) can pass one in to check the fit
    against the hardware record it came from.
    """
    csv_path = Path(csv_path)
    cols = _read_replay_csv(csv_path)
    t_meas = cols["t_s"]
    cmd = cols["commanded_current_a"]
    reported = cols["reported_current_a"]
    measured_w = cols["shaft_rate_rad_s"]

    model = model or load_model()
    data = mujoco.MjData(model)
    dt = float(model.opt.timestep)
    imp = ImperfectionState(profile=profile, dt_s=dt)
    mujoco.mj_forward(model, data)

    n_steps = int(round(float(t_meas[-1]) / dt))
    sim_t = np.empty(n_steps + 1)
    sim_w = np.empty(n_steps + 1)
    sim_t[0] = 0.0
    sim_w[0] = float(data.qvel[0])
    for i in range(1, n_steps + 1):
        t = i * dt
        # Sample-and-hold: the most recent recorded command at or before `t`.
        idx = max(0, int(np.searchsorted(t_meas, t, side="right")) - 1)
        current = imp.apply_current(float(cmd[idx]))
        data.ctrl[0] = current * NAMEPLATE_KT_NM_PER_A
        mujoco.mj_step(model, data)
        sim_t[i] = t
        sim_w[i] = imp.wheel_rate(float(data.qvel[0]), t)

    sim_w_at_meas = np.interp(t_meas, sim_t, sim_w)
    residual = sim_w_at_meas - measured_w
    rms = float(np.sqrt((residual**2).mean()))
    mx = float(np.abs(residual).max())

    n_early = max(1, int(round(0.2 * len(residual))))
    early = residual[:n_early]
    rms_early = float(np.sqrt((early**2).mean()))
    mx_early = float(np.abs(early).max())

    current_div = reported - cmd
    cur_rms = float(np.sqrt((current_div**2).mean()))
    cur_max = float(np.abs(current_div).max())
    tracking_ok = cur_max <= CURRENT_TRACKING_TOLERANCE_A

    return ReplayMetrics(
        rms_residual_rad_s=rms,
        max_residual_rad_s=mx,
        rms_residual_early_rad_s=rms_early,
        max_residual_early_rad_s=mx_early,
        current_rms_divergence_a=cur_rms,
        current_max_divergence_a=cur_max,
        current_tracking_ok=tracking_ok,
        n_samples=len(t_meas),
        imperfection_profile_id=profile.profile_id,
        model_sha256=hashlib.sha256(MODEL_PATH.read_bytes()).hexdigest()[:16],
        mujoco_version=mujoco.__version__,
        timestep_s=dt,
    )


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", choices=("identify", "spindown", "replay"))
    ap.add_argument("--current-a", type=float, default=IdentifyParams().commanded_current_a,
                    help="identify: held commanded current, A")
    ap.add_argument("--spin-current-a", type=float, default=SpindownParams().spin_up_current_a,
                    help="spindown: current during the spin-up, A")
    ap.add_argument("--decay-s", type=float, default=SpindownParams().decay_s,
                    help="spindown: how long to log the decay, s")
    ap.add_argument("--csv", type=Path, help="replay: measured-hardware CSV path")
    ap.add_argument("--ideal", action="store_true",
                    help="use the IDEAL profile instead of STAGE0_PLACEHOLDER "
                    "(bring-up only -- see imperfections.py)")
    ap.add_argument("--out-dir", type=Path, default=OUT_DIR)
    args = ap.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    profile = IDEAL if args.ideal else STAGE0_PLACEHOLDER

    if args.mode == "identify":
        result = identify(IdentifyParams(commanded_current_a=args.current_a), profile=profile)
        m = result.metrics
        print(f"=== bench identify  [profile={m.imperfection_profile_id}]")
        print(f"  kt fit          {m.kt_fit_nm_per_a:.5f} N*m/A "
              f"(nameplate {NAMEPLATE_KT_NM_PER_A:.5f}, {m.kt_error_vs_nameplate_pct:.3f}% off)")
        print(f"  J_bare fit      {m.j_bare_fit_kg_m2:.6e} kg*m^2")
        print(f"  J_disc (known)  {m.j_disc_known_kg_m2:.6e} kg*m^2")
        print(f"  alpha bare/loaded  {m.alpha_bare_rad_s2:.2f} / {m.alpha_loaded_rad_s2:.2f} rad/s^2")
        print(f"  R^2 bare/loaded    {m.r2_bare:.6f} / {m.r2_loaded:.6f}")
        out = result.to_dict()
    elif args.mode == "spindown":
        params = SpindownParams(spin_up_current_a=args.spin_current_a, decay_s=args.decay_s)
        result = spindown(params, profile=profile)
        m = result.metrics
        print(f"=== bench spindown  [profile={m.imperfection_profile_id}]")
        print(f"  fit window opens at {m.fit_start_s*1e3:.2f} ms "
              "(current-loop transient excluded)")
        print(f"  speed swept     {m.w0_rad_s:.2f} -> {m.w_end_rad_s:.2f} rad/s")
        print(f"  b fit           {m.b_fit_nm_s_per_rad:.6e} N*m*s/rad "
              f"(placeholder {m.b_placeholder_nm_s_per_rad:.2e}, {m.b_error_pct:.3f}% off)")
        print(f"  tau_c fit       {m.tau_c_fit_nm:.6e} N*m "
              f"(placeholder {m.tau_c_placeholder_nm:.2e}, {m.tau_c_error_pct:.3f}% off)")
        print(f"  R^2             {m.r2:.6f}")
        print(f"  lumped (viscous-only) fit  b={m.lumped_b_only_nm_s_per_rad:.4e}, "
              f"residual sign change: {m.lumped_residual_changes_sign} "
              f"(lo={m.lumped_residual_low_speed_rad_s2:+.4f}, "
              f"hi={m.lumped_residual_high_speed_rad_s2:+.4f})")
        out = result.to_dict()
    else:
        if args.csv is None:
            ap.error("--csv is required for replay mode")
        m = replay(args.csv, profile=profile)
        print(f"=== bench replay  [profile={m.imperfection_profile_id}]  {args.csv}")
        print(f"  shaft-rate RMS / max residual        {m.rms_residual_rad_s:.4f} / "
              f"{m.max_residual_rad_s:.4f} rad/s")
        print(f"  early-window RMS / max residual       {m.rms_residual_early_rad_s:.4f} / "
              f"{m.max_residual_early_rad_s:.4f} rad/s")
        print(f"  commanded-vs-reported current RMS/max {m.current_rms_divergence_a:.4f} / "
              f"{m.current_max_divergence_a:.4f} A")
        if not m.current_tracking_ok:
            print("  *** CURRENT TRACKING FAILED -- drive appears to have derated. "
                  "every residual above is meaningless until this is explained. ***")
        out = {"mode": "replay", "metrics": m.to_dict()}

    mpath = args.out_dir / f"bench_{args.mode}.json"
    mpath.write_text(json.dumps(out, indent=2) + "\n")
    print(f"wrote {mpath}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
