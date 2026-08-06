"""Attitude estimator, measured against MuJoCo truth.

**The acceptance criterion is NOT met yet, deliberately recorded as such.** The
mini design doc set ≤0.5° RMS / ≤1.0° peak; the v1 complementary filter achieves
0.75° / 2.5° on the ridden plant under disturbance, and the shortfall is concentrated
exactly where the doc predicted it would be — under acceleration, where the
accelerometer's gravity reference is corrupted by the board's own motion.

These tests pin the *measured* behaviour rather than the target, so the gap
stays visible and any change to it is deliberate. `test_the_ac_is_not_met_yet`
is the one to delete when accel compensation lands.
"""

import numpy as np
import pytest

from sim.scenarios.impulse_response import NOMINAL_IMPULSE_NS, ImpulseParams, run
from sim.scenarios.plant import build_model
from sim.scenarios.rust_controller import RustController

TAU = 1.0


@pytest.fixture(scope="module")
def ridden():
    return build_model(70.0, 0.75, 40.0)


OFF, ACTIVE, SHADOW = 0, 1, 2

# THE FRAME-MAP REBASELINE, CLOSED 2026-07-27 (GH #21).
#
# `plant.imu_readings` used to convert MuJoCo axes to the ICD body frame with
# `diag(+1,+1,-1)` -- determinant -1, a reflection, not a rotation. It inverted
# the accelerometer's x channel, so the complementary filter fused a
# nose-up-positive gyro against a nose-down-positive accelerometer. PR #12
# corrected it to `diag(-1,+1,-1)`.
#
# Four results in this file were measured through that reflection and were held
# as `xfail(strict=True)` by the fixing PR rather than re-tuned inside it --
# correctly, since re-tuning would have re-authored this role's conclusions
# inside someone else's frame-map change. They are now RE-DERIVED against the
# corrected rotation, each with its measured value and the reasoning in its own
# docstring. Two of them changed meaning rather than magnitude:
#
#   * closing the loop no longer destabilises the board; it costs margin
#   * wheel-odometry aiding no longer falls over, so the evidence that command
#     feedforward was NECESSARY is gone
#
# What did NOT get re-tuned: the acceptance criteria. The shuttle AC (0.10 m) is
# missed at 0.2331 m and is now its own strict xfail, so loosening a regression
# bound could not quietly retire a requirement.


def _run(model, use_estimator, tau=TAU, secs=8.0, impulse=NOMINAL_IMPULSE_NS):
    with RustController(
        # 200 A/rad, 30 A/(rad/s) at kt = 0.7 N*m/A, re-denominated (#137).
        kp_nm_per_rad=140.0, kd_nm_per_rad_s=21.0, max_current_a=40.0,
        kp_v_rad_per_m_s=0.05, ki_v_rad_per_m=0.02, com_above_axle=True,
        use_estimator=use_estimator, estimator_tau_s=tau,
    ) as c:
        return run(ImpulseParams(magnitude_ns=impulse, sim_seconds=secs),
                   model=model, controller=c)


def test_running_on_truth_reports_no_estimator_error(ridden):
    """The control: with the estimator off, `pitch_used` IS truth."""
    r = _run(ridden, OFF)
    assert r.metrics.pitch_est_rms_deg < 0.05


def test_shadow_mode_does_not_perturb_the_outcome(ridden):
    """Shadow must be observation-only, or every accuracy number measured
    through it is contaminated by the thing it was supposed to isolate."""
    off = _run(ridden, OFF)
    shadow = _run(ridden, SHADOW)
    assert np.allclose(off.pitch_deg, shadow.pitch_deg, atol=1e-9)
    assert shadow.metrics.pitch_est_rms_deg > 0.1, "shadow should still be estimating"


def test_the_estimator_is_actually_in_the_control_path(ridden):
    """The mutation test for a bug that already happened.

    An earlier revision computed the estimate, reported it in `pitch_used_rad`,
    and then regulated on truth anyway. Every metric looked healthy — the
    reported estimator error was a plausible 1.13° — while the estimator was
    entirely inert. Comparing the estimate against truth cannot catch that;
    only comparing the resulting MOTION can.
    """
    truth = _run(ridden, OFF)
    est = _run(ridden, ACTIVE)
    assert not np.allclose(truth.pitch_deg, est.pitch_deg, atol=1e-6), (
        "identical trajectories mean the estimator is not driving the controller"
    )


def test_the_filter_substantially_beats_the_raw_accelerometer(ridden):
    """Otherwise it is an atan2 with extra steps.

    The raw accelerometer sees ~4.4° RMS on this trajectory, dominated by
    excursions past 50° during the impulse; the filter should be several times
    better because it only believes the accelerometer slowly.
    """
    # Measured in SHADOW, so this is estimator accuracy alone rather than
    # accuracy tangled with whatever the closed loop then does about it.
    r = _run(ridden, SHADOW)
    assert r.metrics.pitch_est_rms_deg < 1.5, "filter is not filtering"
    assert r.metrics.pitch_est_max_deg < 4.0


def test_the_estimator_meets_the_ac_on_an_undisturbed_board(ridden):
    """Half the AC is already met, and saying so is the point.

    Quiet, the filter runs at ~0.10 deg RMS -- comfortably inside the 0.5 deg
    budget, and consistent with the predicted bias*tau term. So the shortfall
    under disturbance is NOT a broken filter or a bad crossover; it is the
    accelerometer corruption, isolated.
    """
    r = _run(ridden, SHADOW, impulse=0.0)
    assert r.metrics.pitch_est_rms_deg <= 0.5
    assert r.metrics.pitch_est_max_deg <= 1.0


def test_closing_the_loop_on_the_v1_estimator_costs_margin_but_survives(ridden):
    """**Re-derived 2026-07-27. The result this test used to record is gone.**

    It asserted `nose_strike` -- that driving the loop from the v1 estimate
    put the board on the ground. That was measured through an IMU→ICD frame map
    with `det = -1`, which inverted the accelerometer's x channel and made the
    filter fuse a nose-up-positive gyro against a nose-down-positive
    accelerometer. PR #12 corrected the map. Re-measured on the impulse:

        mode      strike   peak |pitch|   est RMS   est max
        truth      no          6.000°     0.013°    0.073°
        shadow     no          6.000°     0.261°    1.454°
        ACTIVE     no          7.712°     0.312°    1.404°

    So the destabilisation was the frame bug, not the feedback mechanism. **The
    mechanism itself is still real and still measurable** -- closing the loop
    costs ~29% of peak pitch and ~19% of estimator RMS against shadow -- but it
    is now a margin cost, not a crash.

    Asserted as a BAND, not a threshold. A floor because the feedback penalty
    vanishing entirely would mean the estimator stopped being in the loop, which
    this codebase has shipped twice; a ceiling because a large penalty would
    mean the destabilisation is back.

    Where the old headline went: the estimator DOES still lose the board, on a
    hill rather than on an impulse. `tests/test_hill.py` gates that -- truth
    pitch holds a 15% descent, the estimate puts the tail down at 1.1 s.
    """
    active = _run(ridden, ACTIVE).metrics
    shadow = _run(ridden, SHADOW).metrics

    assert not active.nose_strike, (
        "the board struck while closed-loop on the estimate. That is the "
        "pre-frame-fix behaviour returning -- do not re-baseline this, diagnose it"
    )

    # The feedback penalty: real, bounded, and required to be non-zero.
    assert active.peak_abs_pitch_deg > shadow.peak_abs_pitch_deg * 1.05, (
        f"closed loop ({active.peak_abs_pitch_deg:.3f}°) cost nothing against "
        f"shadow ({shadow.peak_abs_pitch_deg:.3f}°). Feedback through a noisy "
        "estimate is not free -- if it looks free, check the estimate is "
        "actually driving the loop"
    )
    assert active.peak_abs_pitch_deg < shadow.peak_abs_pitch_deg * 2.0, (
        f"closed loop cost {active.peak_abs_pitch_deg / shadow.peak_abs_pitch_deg:.2f}x "
        "the peak pitch of shadow -- the destabilisation mechanism is growing again"
    )
    assert active.pitch_est_rms_deg < 0.5, (
        f"estimator RMS {active.pitch_est_rms_deg:.3f}° closed-loop, was 0.312°"
    )


@pytest.mark.xfail(
    strict=True,
    reason="v1 complementary filter misses the AC (0.5 deg RMS / 1.0 deg peak) "
    "because the accelerometer's gravity reference is corrupted by the board's "
    "own acceleration -- predicted as open question 1 in the mini design doc. "
    "The fix is to subtract the wheel-odometry-derived linear term, deferred "
    "from v1 on purpose. Delete this xfail when that lands.",
)
def test_the_ac_is_not_met_yet(ridden):
    r = _run(ridden, SHADOW)
    assert r.metrics.pitch_est_rms_deg <= 0.5
    assert r.metrics.pitch_est_max_deg <= 1.0


def test_estimator_error_is_worse_while_accelerating(ridden):
    """The diagnosis, asserted.

    If the error ever stops correlating with acceleration, the cause has
    changed and the planned fix may no longer be the right one.
    """
    # A genuinely undisturbed run, not a short window -- a 3 s run still
    # contains the impulse at t = 0.5 s and is not quiet at all.
    quiet = _run(ridden, SHADOW, impulse=0.0)
    disturbed = _run(ridden, SHADOW, impulse=NOMINAL_IMPULSE_NS)
    assert quiet.metrics.pitch_est_rms_deg < 0.5 * disturbed.metrics.pitch_est_rms_deg, (
        f"quiet {quiet.metrics.pitch_est_rms_deg:.2f} deg vs "
        f"disturbed {disturbed.metrics.pitch_est_rms_deg:.2f} deg -- if these are "
        "close, acceleration is no longer the dominant error source and the "
        "planned fix may be wrong"
    )


def test_a_longer_crossover_trades_lag_for_noise_rejection(ridden):
    """Both ends of the tau trade are real, so neither extreme is a free win."""
    short = _run(ridden, SHADOW, tau=0.3)
    long = _run(ridden, SHADOW, tau=4.0)
    assert short.metrics.pitch_est_max_deg > long.metrics.pitch_est_max_deg


def test_estimator_runs_are_deterministic(ridden):
    a = _run(ridden, SHADOW)
    b = _run(ridden, SHADOW)
    assert a.to_json_dict()["metrics"] == b.to_json_dict()["metrics"]


# ---------------------------------------------------------------------------
# The shuttle: wiring, and the fix that closes the loop
# ---------------------------------------------------------------------------

SHUTTLE_GAINS = dict(
    # 200 A/rad, 30 A/(rad/s) at kt = 0.7 N*m/A, re-denominated (#137).
    kp_nm_per_rad=140.0, kd_nm_per_rad_s=21.0, max_current_a=40.0,
    kp_v_rad_per_m_s=0.05, ki_v_rad_per_m=0.02, com_above_axle=True,
)

WHEEL_ODOMETRY, COMMAND_FEEDFORWARD = 1, 2


def _shuttle(**kw):
    from sim.scenarios.shuttle_run import ShuttleParams
    from sim.scenarios.shuttle_run import run as shuttle_run

    return shuttle_run(
        ShuttleParams(),
        controller_factory=lambda vp: RustController(
            **SHUTTLE_GAINS, v_ref_fn=vp.v_ref, **kw),
    )


def test_the_shuttle_runs_the_estimator_BY_DEFAULT():
    """**The default is the claim.** A headline scenario must not measure the
    control law with the hardest part of the problem deleted.

    The shuttle used to default to truth pitch -- a board that knows its own
    tilt exactly, which no hardware will ever be. A fresh session could
    believe the estimator was running when it was not, and the published
    return error was flattering because of it.

    Gated here because the whole test suite passed before and after the default
    was flipped -- every other test supplies its own controller factory, so
    nothing exercised the default at all. A default nothing tests is a default
    that will silently revert.

    Checked by OUTCOME rather than by inspecting the factory -- see
    `test_the_estimator_is_actually_in_the_control_path`'s docstring for why
    comparing the estimate against truth cannot catch "computed but not
    consumed"; only comparing the resulting motion can. Compared against a
    TRUTH-PITCH RUN COMPUTED FRESH, not a historical magnitude: the drag-model
    correction moved both numbers (and did not move them the same way -- truth
    got WORSE, the estimator got BETTER), and pinning either as a hardcoded
    identity token would have made that correction read as a regression rather
    than the improvement it was. `test_the_default_shuttle_configuration_is_
    the_recommended_one` covers the second half of the original claim -- that
    the default IS the recommended tau=2 s + command-feedforward configuration
    -- as its own test, for the same reason.
    """
    from sim.scenarios.shuttle_run import run as shuttle_run

    default = shuttle_run().metrics
    truth = _shuttle(use_estimator=OFF).metrics
    assert not default.nose_strike

    # Materially different from a truth-pitch run computed THIS SESSION, not
    # compared against either run's number from a different drag model.
    assert default.return_error_m != pytest.approx(truth.return_error_m, rel=0.2, abs=1e-3), (
        f"default shuttle returned {default.return_error_m:.4f} m, indistinguishable "
        f"from a truth-pitch run measured fresh ({truth.return_error_m:.4f} m) -- "
        "the estimator is not in the default path"
    )


def test_the_default_shuttle_configuration_is_the_recommended_one():
    """The default must BE the recommended tau=2 s + command-feedforward
    configuration, not merely produce a number that happens to be similar.

    Split out of `test_the_shuttle_runs_the_estimator_BY_DEFAULT` so the two
    claims -- "the estimator is selected at all" and "it is THIS estimator
    configuration" -- fail independently and legibly. Compared against a
    `recommended` run computed in this same test, not a pinned literal, so a
    genuine improvement in either configuration cannot register as a failure
    here -- only an actual divergence between "default" and "recommended" can.
    """
    from sim.scenarios.shuttle_run import run as shuttle_run

    default = shuttle_run().metrics
    recommended = _shuttle(use_estimator=ACTIVE, estimator_tau_s=2.0,
                           estimator_accel_aiding=COMMAND_FEEDFORWARD).metrics
    assert default.return_error_m == pytest.approx(recommended.return_error_m, abs=1e-9), (
        "the default is no longer the recommended tau=2 s + command-feedforward "
        "configuration"
    )


def test_the_shuttle_hands_the_controller_a_live_imu():
    """The regression for a bug that ran clean and reported garbage.

    `RustController.__call__` takes the raw IMU as OPTIONAL keyword arguments.
    Omitting them does not raise -- it leaves the observation's gyro and
    accelerometer at the zeros they were constructed with, so a complementary
    filter evaluates atan2(0, -0) forever and returns a confident 180 deg. The
    shuttle did exactly this for its whole life while the impulse scenario did
    not, so the estimator looked scenario-dependently broken.

    Asserting on estimator ACCURACY would not have caught it cleanly, because a
    bad estimate is also what a genuinely bad estimator produces. This asserts
    on the wiring itself: the controller must be handed a plausible gravity
    vector on every single cycle.
    """
    import numpy as np

    seen = []

    class Spy:
        pitch_ref_rad = 0.0
        pitch_used_rad = 0.0
        saturated_cycles = 0
        cycles = 0
        peak_abs_pitch_ref_rad = 0.0

        def __enter__(self):
            return self

        def __exit__(self, *exc):
            return False

        def __call__(self, t, pitch, rate, wheel, gyro_rad_s=None,
                     accel_m_s2=None, motor_current_a=None):
            seen.append((gyro_rad_s, accel_m_s2))
            return 0.0

    from sim.scenarios.shuttle_run import ShuttleParams
    from sim.scenarios.shuttle_run import run as shuttle_run

    shuttle_run(ShuttleParams(), controller_factory=lambda vp: Spy())

    assert seen, "the scenario never called the controller"
    assert all(a is not None and g is not None for g, a in seen), (
        "the shuttle called the controller without the raw IMU -- the estimator "
        "would read atan2(0, -0) and report a constant ~180 deg"
    )
    mags = [float(np.linalg.norm(a)) for _, a in seen]
    assert min(mags) > 5.0, (
        f"weakest specific force was {min(mags):.3f} m/s^2; an accelerometer on a "
        "board that is upright-ish should always feel most of 1 g"
    )


def test_command_feedforward_closes_the_loop_on_the_shuttle():
    """**Re-derived 2026-07-27.** The qualitative result survived the frame-map
    fix; the number did not.

    Driving the real control law from a fused attitude estimate, the board
    completes the shuttle run without striking. Measured through the real
    scenario and the real Rust controller over the FFI -- no replica in the path.

    Return error **0.0226 m → 0.2331 m** under the corrected `diag(-1,+1,-1)`
    rotation. The old 0.0226 m was measured through a reflection and is void.

    **The 0.10 m acceptance criterion is NOT met**, and this test no longer
    pretends otherwise -- see `test_the_shuttle_ac_is_not_met_yet` below, which
    records the gap as an explicit xfail rather than hiding it behind a loosened
    threshold. What is asserted here is the qualitative claim that still holds:
    it completes, and it stays within a bound that would catch a regression.
    """
    est = _shuttle(use_estimator=1, estimator_tau_s=2.0,
                   estimator_accel_aiding=COMMAND_FEEDFORWARD)
    assert not est.metrics.nose_strike
    assert est.metrics.return_error_m < 0.35, (
        f"finished {est.metrics.return_error_m:.4f} m from home; the re-derived "
        "figure is 0.2331 m and this bound exists to catch a regression, not to "
        "certify the AC"
    )


@pytest.mark.xfail(
    strict=True,
    reason="Return error is 0.2331 m against a 0.10 m AC. Recorded as a visible "
    "gap rather than a loosened threshold: the shuttle AC is missed by 2.3x, and "
    "the same v1 estimator loses a 15% descent outright (tests/test_hill.py). "
    "Delete this xfail when accel compensation lands.",
)
def test_the_shuttle_ac_is_not_met_yet():
    """The acceptance criterion, asserted so its failure stays visible.

    Re-baselining `test_command_feedforward_closes_the_loop_on_the_shuttle` to
    0.35 m would have quietly retired a requirement. Splitting the AC into its
    own strict xfail keeps the regression guard and the unmet requirement as two
    separate, separately-reviewable facts.
    """
    est = _shuttle(use_estimator=1, estimator_tau_s=2.0,
                   estimator_accel_aiding=COMMAND_FEEDFORWARD)
    assert est.metrics.return_error_m < 0.10


def test_both_aiding_sources_survive_the_shuttle_and_feedforward_is_only_slightly_better():
    """**Re-derived 2026-07-27. This is the one that changed most, and the
    honest version of it is much weaker than what it replaced.**

    This test asserted that wheel-odometry aiding FELL OVER at tau = 2 s --
    strike True, 120.84 m from home -- while command feedforward completed. That
    comparison, one configuration fatal and the other the fix, was the evidence
    the feedforward work rested on.

    Through the corrected frame map, **both survive**:

        wheel odometry      no strike, 0.2783 m
        command feedforward no strike, 0.2331 m

    A 0.045 m difference between two configurations that both work. **The
    evidence that command feedforward was NECESSARY is gone.** It is still
    marginally better and there is a real mechanism for why -- odometry
    differentiates a quantised 100 Hz speed through a 50 ms low pass, and the
    residual lands near the loop's crossover -- but "better by 4.5 cm on one
    scenario" is a different claim from "the fix", and it has to be
    re-established elsewhere rather than assumed here.

    Deliberately NOT asserting a ratio between the two. At this margin the
    ordering is the finding; a ratio would be fitting noise.
    """
    odo = _shuttle(use_estimator=1, estimator_tau_s=2.0,
                   estimator_accel_aiding=WHEEL_ODOMETRY)
    ff = _shuttle(use_estimator=1, estimator_tau_s=2.0,
                  estimator_accel_aiding=COMMAND_FEEDFORWARD)

    assert not odo.metrics.nose_strike, (
        "wheel-odometry aiding struck. It used to, through the reflected frame "
        "map; if it does again, something regressed rather than reverted"
    )
    assert not ff.metrics.nose_strike
    assert ff.metrics.return_error_m < odo.metrics.return_error_m, (
        f"command feedforward ({ff.metrics.return_error_m:.4f} m) is supposed to "
        f"beat wheel odometry ({odo.metrics.return_error_m:.4f} m); if the "
        "ordering flips, the reason to prefer it is gone entirely"
    )
    assert odo.metrics.return_error_m - ff.metrics.return_error_m < 0.15, (
        "the gap between the two aiding sources has widened well beyond the "
        "0.045 m measured. That would be a real finding -- re-measure the "
        "frequency response in notebook 4 rather than adjusting this bound"
    )


def test_a_long_crossover_costs_accuracy():
    """**Re-derived 2026-07-27.** The trade survived; the threshold was set
    against a confounded baseline.

    tau >= 10 s leans on the accelerometer less. It costs position accuracy,
    because a gyro bias `b` leaves a steady attitude error `b*tau` that the
    outer loop turns into drift.

    Measured: 0.9567 m at tau = 10 s against 0.2331 m at tau = 2 s -- a
    **4.10x** cost. The old bar was 5x, which the pre-fix numbers cleared and
    these do not. Re-derived to 3x, which is comfortably below the measured
    4.10x and still far above noise.

    The trade is the point: there are two ways to survive this scenario and they
    cost different things. That is why the estimator has a crossover knob at all.
    """
    long_tau = _shuttle(use_estimator=1, estimator_tau_s=10.0,
                        estimator_accel_aiding=WHEEL_ODOMETRY)
    good = _shuttle(use_estimator=1, estimator_tau_s=2.0,
                    estimator_accel_aiding=COMMAND_FEEDFORWARD)
    assert not long_tau.metrics.nose_strike
    assert long_tau.metrics.return_error_m > 3 * good.metrics.return_error_m, (
        f"tau=10 s drifted {long_tau.metrics.return_error_m:.4f} m against "
        f"{good.metrics.return_error_m:.4f} m at tau=2 s. The trade should be "
        "visible: surviving via a long crossover ought to cost substantially "
        "more position accuracy than surviving via better aiding"
    )


# ---------------------------------------------------------------------------
# Which current the feedforward believes (DR-CTRL-5)
# ---------------------------------------------------------------------------

COMMANDED, MEASURED = 0, 1


def test_the_default_current_source_survives_a_backend_that_reports_nothing():
    """Why the default is `commanded` even though hardware wants `measured`.

    `motor_current_a` is an optional-in-practice field: a backend that never
    populates it leaves a plausible-looking ZERO, and a feedforward told to
    trust it then predicts no acceleration at all -- silently degrading to no
    aiding, which at tau = 2 s does not fly. This codebase has already lost a
    day to exactly that shape of bug (the shuttle's unwired IMU), so the
    default must be the one that cannot fail quietly.
    """
    from sim.scenarios.shuttle_run import ShuttleParams
    from sim.scenarios.shuttle_run import run as shuttle_run

    class Blind(RustController):
        """A backend that forgets to report measured current."""

        def __call__(self, t, pitch, rate, wheel, gyro_rad_s=None,
                     accel_m_s2=None, motor_current_a=None):
            return super().__call__(t, pitch, rate, wheel, gyro_rad_s=gyro_rad_s,
                                    accel_m_s2=accel_m_s2, motor_current_a=None)

    r = shuttle_run(
        ShuttleParams(),
        controller_factory=lambda vp: Blind(
            **SHUTTLE_GAINS, v_ref_fn=vp.v_ref, use_estimator=1,
            estimator_tau_s=2.0, estimator_accel_aiding=COMMAND_FEEDFORWARD,
            accel_ff_current_source=MEASURED),
    )
    assert r.metrics.nose_strike, (
        "trusting an unpopulated motor_current_a should fail LOUDLY. If this "
        "now survives, the default in ObParamsV2 can safely become MEASURED"
    )


def test_sim_cannot_tell_the_two_current_sources_apart():
    """**A negative result, recorded so it is not mistaken for a positive one.**

    On hardware the feedforward must use MEASURED current, because a VESC
    derates commanded torque silently through half a dozen cutback layers
    (DR-CTRL-5). In sim the two are nearly identical, differing only by the
    actuation delay and current-loop lag -- **because the sim has no cutback
    model at all**.

    So this test does not vindicate the hardware choice; it documents that the
    sim is incapable of vindicating it. Making that case testable needs a
    PROPORTIONAL derate row in the imperfection profile. The existing hard
    current cap is not a substitute: capping hard enough to matter destroys
    torque authority, and the resulting crash is the missing torque rather than
    the corrupted estimate.

    "Indistinguishable" is measured RELATIVE to a pair the sim treats as
    genuinely different (the two accel-aiding SOURCES, wheel-odometry vs
    command-feedforward -- see `test_both_aiding_sources_survive_the_shuttle_
    and_feedforward_is_only_slightly_better`), not against a fixed absolute
    epsilon in metres. A fixed epsilon was calibrated against the old drag
    model's much larger return-error scale; the corrected drag model shrank
    every return error in this file, which shrank the commanded-vs-measured
    gap too but left the epsilon unchanged, so an unrelated physics correction
    could fail this "negative result" outright. The relative form is what the
    claim actually is: this gap is small compared to a gap that IS real.
    """
    from sim.scenarios.shuttle_run import ShuttleParams
    from sim.scenarios.shuttle_run import run as shuttle_run

    def go(source):
        return shuttle_run(
            ShuttleParams(),
            controller_factory=lambda vp: RustController(
                **SHUTTLE_GAINS, v_ref_fn=vp.v_ref, use_estimator=1,
                estimator_tau_s=2.0, estimator_accel_aiding=COMMAND_FEEDFORWARD,
                accel_ff_current_source=source),
        )

    cmd, meas = go(COMMANDED), go(MEASURED)
    assert not cmd.metrics.nose_strike and not meas.metrics.nose_strike
    current_source_gap = abs(cmd.metrics.return_error_m - meas.metrics.return_error_m)

    # The reference for "distinguishable": two accel-aiding SOURCES the sim
    # already knows are genuinely different, computed fresh in this test
    # rather than pinned, so both sides of the comparison move together under
    # a future physics change.
    odo = _shuttle(use_estimator=1, estimator_tau_s=2.0,
                   estimator_accel_aiding=WHEEL_ODOMETRY)
    ff = _shuttle(use_estimator=1, estimator_tau_s=2.0,
                  estimator_accel_aiding=COMMAND_FEEDFORWARD)
    distinguishable_gap = abs(odo.metrics.return_error_m - ff.metrics.return_error_m)

    assert current_source_gap < 0.5 * distinguishable_gap, (
        f"commanded-vs-measured current gap ({current_source_gap:.4f} m) is not "
        f"small relative to a pair the sim treats as genuinely different "
        f"({distinguishable_gap:.4f} m) -- if these have diverged, the sim has "
        "gained something cutback-like and this test should become a real "
        "comparison rather than a caveat"
    )


def test_the_scenarios_report_a_measured_current_that_is_not_the_command():
    """The field exists, is populated, and is genuinely a different signal.

    It was declared in the observation from the start and left at zero by every
    scenario -- so any backend consuming it would have read a constant zero.
    """
    import numpy as np

    from sim.scenarios.shuttle_run import ShuttleParams
    from sim.scenarios.shuttle_run import run as shuttle_run

    seen = []

    class Spy(RustController):
        def __call__(self, t, pitch, rate, wheel, gyro_rad_s=None,
                     accel_m_s2=None, motor_current_a=None):
            amps = super().__call__(t, pitch, rate, wheel, gyro_rad_s=gyro_rad_s,
                                    accel_m_s2=accel_m_s2,
                                    motor_current_a=motor_current_a)
            seen.append((amps, motor_current_a))
            return amps

    shuttle_run(ShuttleParams(),
                controller_factory=lambda vp: Spy(**SHUTTLE_GAINS, v_ref_fn=vp.v_ref))

    measured = np.array([m for _, m in seen], dtype=float)
    commanded = np.array([c for c, _ in seen], dtype=float)
    assert np.abs(measured).max() > 1.0, "measured current is still all zeros"
    # Delay plus current-loop lag: they track, but they are not the same series.
    assert np.abs(measured - commanded).max() > 0.5, (
        "measured current is echoing the command -- ICD 12 calls that "
        "non-conforming, because it hides the lag the controller must tolerate"
    )
