//! The scripted input schedules, and the two ways of playing them back.
//!
//! These tables used to live inside `src/bin/send-input.rs`, which could only
//! ever play them against a **wall clock** over UDP. That turned out to be
//! load-bearing in a way nobody intended -- see [`crate::host::HostConfig::
//! scripted_scenario`] and the "why this module exists" note below -- so the
//! tables moved here, into the library, where the host itself can also play
//! them **indexed on simulated time**.
//!
//! # Why this module exists (issue #190)
//!
//! `send-input` paced itself with `thread::sleep(20 ms)` per packet and
//! indexed the schedule off `Instant::elapsed()`. Both halves of that are
//! wrong on a loaded, non-realtime dev host, and the second one hid the
//! first:
//!
//! - macOS timer coalescing turns a requested 20 ms sleep into a much longer
//!   one, so the "50 Hz" sender actually delivered **7-13 Hz**, measured.
//! - `sim-host` zeroes a `weight_shift_*`/`steer` input that is older than
//!   [`crate::host::INPUT_STALENESS_TIMEOUT`] (100 ms). A 7-13 Hz sender sits
//!   directly ON that threshold, so a random fraction of every run's ticks
//!   ran with the input **zeroed** -- a de-rated version of the schedule that
//!   varies run to run.
//!
//! The result is that a scripted run's effective aggression was a property of
//! how loaded the laptop was, not of the schedule. Playing a schedule against
//! `sim_time_s` inside the host removes the wall clock, the socket, the
//! staleness gate and the sender process all at once, so the same schedule
//! produces the same run every time -- which is what the plant (fixed
//! timestep, RK4, no stochastic terms) was always able to deliver.
//!
//! `send-input` still exists and still sends over UDP -- it is the only tool
//! that exercises the actual wire -- but it now paces on absolute deadlines
//! (the same rule [`crate::pacer`] documents) instead of accumulating drift.

/// One schedule row: `(start_s, end_s, weight_shift_fore_aft,
/// weight_shift_lateral, steer, label)`. Values are within `[-1, 1]`;
/// `sim-host` scales them onto the model's actual ballast/yaw ranges.
pub type Row = (f64, f64, f32, f32, f32, &'static str);

/// A named, immutable input schedule.
pub type Schedule = &'static [Row];

/// Issue #161 W2's acceptance-criterion schedule: settle, lean forward
/// (accelerate), release (coast), lean back (decelerate then reverse),
/// release again, lean+steer (turn) -- "drives forward, slows and reverses
/// under lean, and turns", exercised once each.
///
/// Deliberately left alone by every `s-curve` revision: this is the
/// regression schedule, not a viewing one.
pub const DEFAULT_SCHEDULE: Schedule = &[
    (0.0, 1.5, 0.0, 0.0, 0.0, "settle"),
    (1.5, 4.5, 0.6, 0.0, 0.0, "lean forward (accelerate)"),
    (4.5, 6.5, 0.0, 0.0, 0.0, "release (coast)"),
    (
        6.5,
        10.0,
        -0.6,
        0.0,
        0.0,
        "lean back (decelerate, then reverse)",
    ),
    (10.0, 12.0, 0.0, 0.0, 0.0, "release (coast reversed)"),
    (12.0, 14.0, 0.0, 0.8, 0.6, "lean right + steer (turn)"),
    (14.0, 15.0, 0.0, 0.0, 0.0, "release"),
];

// --- s-curve, Revision 4 -------------------------------------------------
//
// The launch-footage schedule (PR #183, commit 542f3a5). Its full revision
// history -- Revisions 1 through 4, with every measured value that did and
// did not ship -- is in `docs/decisions/` and in this crate's git history;
// what follows is the landed configuration plus the one correction issue
// #190 forced on the record.
//
// # CORRECTION (issue #190): the stability results this schedule was
// # accepted on were measured against a DE-RATED delivery of it
//
// Revisions 2, 3 and 4 each reported "no instability at any point measured",
// and each of those measurements was taken through `send-input`'s
// wall-clock-paced UDP sender. That sender was delivering 7-13 Hz, not the
// 50 Hz it claimed, against a 100 ms host staleness timeout -- so a large,
// run-varying fraction of every one of those runs held `weight_shift_fore_aft
// = 0` instead of the scheduled value (see this module's header). The
// measured `ballast_fa` in a de-rated run sits around 0.031-0.034 m; in a
// faithfully-delivered one it sits at 0.046 m and rising.
//
// Played faithfully -- which is what [`crate::host::HostConfig::
// scripted_scenario`] now does -- this schedule's build phase drives the
// board past the motor-current envelope and it flips. That is a property of
// the schedule and the control law, NOT of the harness; the harness's only
// role was to hide it. Nothing here has been tuned to make the flip go away.
/// Settle before the build starts.
pub const SCURVE4_SETTLE_S: f64 = 0.5;
/// Hard build duration. Restored to Revision 3's rejected-then-correct 15.0 s
/// once the invented ~200 m distance line was withdrawn.
pub const SCURVE4_BUILD_S: f64 = 15.0;
/// Full stick ("do not be shy with `fore_aft` during the build").
///
/// **This is the constant issue #190 is about.** Held for 15 s and actually
/// delivered, full stick was past what the 40 A / 28 N*m actuator envelope
/// could hold the frame against once the board was up to speed --
/// **NO LONGER TRUE at the 60 A / 42 N*m envelope (issue:
/// realistic-motor-torque)**: on the deployed estimator path this same
/// schedule now survives the full build with peak pitch around 10-11 deg;
/// see `host.rs`'s `CMD_ENVELOPE_RESERVE` doc comment for the measurement.
/// The truth-fed reading of issue #190/ADR-0011 criterion (f) still
/// eventually flips, just later.
pub const SCURVE4_BUILD_LEAN: f32 = 1.0;
/// Zero: any sustained in-weave trim keeps adding speed rather than holding
/// it, more so at higher entry speed (Revision 2's finding).
pub const SCURVE4_WEAVE_TRIM: f32 = 0.0;
/// Full-lobe duration. 4.5 s was measured and rejected -- at this steer and
/// yaw gain a single lobe wound past a full 360 deg rotation (520.8 deg of
/// measured swing), which is spinning, not carving.
pub const SCURVE4_LOBE_S: f64 = 2.8;
/// Entry (and exit) half-lobe duration -- roughly half [`SCURVE4_LOBE_S`],
/// the ratio every prior revision centred acceptably at.
pub const SCURVE4_ENTRY_S: f64 = 1.4;
/// Lateral and steer stick, at their limits.
pub const SCURVE4_LATERAL: f32 = 1.0;
/// Lateral and steer stick, at their limits.
pub const SCURVE4_STEER: f32 = 1.0;

const B: f64 = SCURVE4_SETTLE_S + SCURVE4_BUILD_S;
const E1: f64 = B + SCURVE4_ENTRY_S;
const L1: f64 = E1 + SCURVE4_LOBE_S;
const L2: f64 = E1 + 2.0 * SCURVE4_LOBE_S;
const E2: f64 = L2 + SCURVE4_ENTRY_S;

/// `--scenario s-curve`: hold a decent forward speed and weave wide, for
/// watching the board CARVE. This is the launch-footage schedule -- read the
/// correction block above it before quoting any stability result measured
/// against it.
pub const S_CURVE_SCHEDULE: Schedule = &[
    (0.0, SCURVE4_SETTLE_S, 0.0, 0.0, 0.0, "settle"),
    (
        SCURVE4_SETTLE_S,
        B,
        SCURVE4_BUILD_LEAN,
        0.0,
        0.0,
        "lean forward (hard build)",
    ),
    (
        B,
        E1,
        SCURVE4_WEAVE_TRIM,
        SCURVE4_LATERAL,
        SCURVE4_STEER,
        "carve right (half lobe, enter)",
    ),
    (
        E1,
        L1,
        SCURVE4_WEAVE_TRIM,
        -SCURVE4_LATERAL,
        -SCURVE4_STEER,
        "carve left (hold the line)",
    ),
    (
        L1,
        L2,
        SCURVE4_WEAVE_TRIM,
        SCURVE4_LATERAL,
        SCURVE4_STEER,
        "carve right (hold the line)",
    ),
    (
        L2,
        E2,
        SCURVE4_WEAVE_TRIM,
        -SCURVE4_LATERAL,
        -SCURVE4_STEER,
        "straighten (exit lobe)",
    ),
    (E2, E2 + 1.5, 0.0, 0.0, 0.0, "release (coast)"),
];

// --- ADR-0011 exit criterion (a): the named acceptance matrix ------------
//
// "The board does not invert across a named test matrix" -- the ADR names
// three entries, because "from any starting state" was untestable as
// written. Two of the three are below; the third (full stick during a kerb
// strike) is `FULL_STICK_SCHEDULE` plus a `HostConfig::disturbance`, since a
// kerb is a disturbance and not an input schedule.
//
// All three are STRAIGHT-LINE, `steer = 0`, `lateral = 0`. That is not a
// simplification: the flip these exist to catch happens on the straight, and
// carving into them would add a yaw channel this crate's own header calls
// non-physical to a measurement that has to stand up.

/// Settle time before any acceptance schedule starts driving. Matches
/// [`SCURVE4_SETTLE_S`] so the two are comparable tick for tick.
pub const ACCEPTANCE_SETTLE_S: f64 = 0.5;

/// Criterion (a) entry 1: **full stick from rest**, held. The defect issue
/// #190 root-caused, reduced to the shortest schedule that reproduces it --
/// no carve, no lateral, nothing to argue about.
///
/// 15 s of hold, matching [`SCURVE4_BUILD_S`]: the measured divergence is
/// ~5.4 s in, and a hold that outlasts it by 3x leaves no room for "it was
/// about to recover".
pub const FULL_STICK_SCHEDULE: Schedule = &[
    (0.0, ACCEPTANCE_SETTLE_S, 0.0, 0.0, 0.0, "settle"),
    (
        ACCEPTANCE_SETTLE_S,
        ACCEPTANCE_SETTLE_S + 15.0,
        1.0,
        0.0,
        0.0,
        "full stick from rest (hold)",
    ),
];

/// Criterion (a) entry 2: **full stick reversal at speed** -- the ADR's
/// worst case, and the one it records as NEVER TESTED.
///
/// Why it is the worst case, in the ADR's words: *commanded deceleration and
/// gravity load the same side*. Two further mechanisms make it worse than
/// that summary suggests, both visible in this host's own code:
///
/// 1. **The speed cap does not attenuate it.** The cap only withdraws
///    authority when the stick and the current motion have the SAME sign
///    (`accelerating_same_direction`). A reversal is by definition
///    opposite-signed, so it passes through at full authority at any speed
///    -- including above the 8.34 m/s onset where every surviving run in the
///    ADR's data was being unloaded by that cap.
/// 2. **It is a step, not a ramp.** The ballast actuator's `timeconst` is
///    0.15 s and the stick goes from one rail to the other in one tick.
///
/// The schedule runs the reversal in BOTH directions, in one deterministic
/// run, because the ADR names the reverse-to-forward one and the
/// forward-to-reverse one is the entry condition for it:
///
/// - build forward at full stick to the speed cap,
/// - slam to full reverse (forward-to-reverse, at speed),
/// - hold through zero and back up to speed in reverse,
/// - slam to full forward (**reverse-to-forward, at speed** -- the ADR's
///   named case),
/// - release.
pub const STICK_REVERSAL_SCHEDULE: Schedule = &[
    (0.0, ACCEPTANCE_SETTLE_S, 0.0, 0.0, 0.0, "settle"),
    (
        ACCEPTANCE_SETTLE_S,
        10.5,
        1.0,
        0.0,
        0.0,
        "full stick forward (build to the cap)",
    ),
    (
        10.5,
        20.5,
        -1.0,
        0.0,
        0.0,
        "SLAM full reverse (forward-to-reverse reversal at speed)",
    ),
    (
        20.5,
        30.5,
        1.0,
        0.0,
        0.0,
        "SLAM full forward (reverse-to-forward reversal at speed)",
    ),
    (30.5, 32.5, 0.0, 0.0, 0.0, "release"),
];

// --- Braking authority: the CEO's report from driving the build ----------
//
// "I would say you should be able to stop faster by leaning back [...] but in
// general I should be able to cut a tight turn by braking and turning at the
// same time while keeping speed."
//
// Neither of those is measurable against the schedules above: `full-stick`
// only accelerates, and `stick-reversal` slams through zero into a reverse
// standing start, so the stop is over before it can be measured and is then
// buried under a re-acceleration. These two isolate the stop.

/// How long the two braking schedules build speed before braking. Long
/// enough to settle at the speed cap on the shipped reserve (the board
/// reaches ~9.15 m/s at ~12 s), so the brake starts from cruise rather than
/// from the middle of an acceleration ramp.
pub const BRAKE_BUILD_S: f64 = 12.0;

/// How long the brake is held. Deliberately long enough to carry the board
/// through zero and into reverse: the stop itself is the measurement, and
/// cutting the schedule at the stop would hide anything that happens right
/// after it.
pub const BRAKE_HOLD_S: f64 = 8.0;

/// **Stop from cruise.** Build to the speed cap at full stick, then full aft
/// stick and hold. The reference for "how fast can this thing stop".
pub const BRAKE_STOP_SCHEDULE: Schedule = &[
    (0.0, ACCEPTANCE_SETTLE_S, 0.0, 0.0, 0.0, "settle"),
    (
        ACCEPTANCE_SETTLE_S,
        ACCEPTANCE_SETTLE_S + BRAKE_BUILD_S,
        1.0,
        0.0,
        0.0,
        "build to the speed cap",
    ),
    (
        ACCEPTANCE_SETTLE_S + BRAKE_BUILD_S,
        ACCEPTANCE_SETTLE_S + BRAKE_BUILD_S + BRAKE_HOLD_S,
        -1.0,
        0.0,
        0.0,
        "full aft stick (brake to a stop, then reverse)",
    ),
    (
        ACCEPTANCE_SETTLE_S + BRAKE_BUILD_S + BRAKE_HOLD_S,
        ACCEPTANCE_SETTLE_S + BRAKE_BUILD_S + BRAKE_HOLD_S + 2.0,
        0.0,
        0.0,
        0.0,
        "release",
    ),
];

/// **Brake and turn together**, which is the manoeuvre the CEO reported as
/// not working: identical to [`BRAKE_STOP_SCHEDULE`] except that full steer
/// and full lateral lean go in at the same instant as the brake.
///
/// Carrying a yaw channel into a measurement is normally something this
/// module refuses (see the acceptance-matrix note above), and the reason
/// stands: yaw here is kinematically injected and declared non-physical. It
/// is admitted HERE because the question being asked is explicitly about
/// feel — does braking cost the turn, does turning cost the stop — and that
/// question does not exist without the yaw channel. Nothing measured on this
/// schedule may be cited as a stability result.
pub const BRAKE_TURN_SCHEDULE: Schedule = &[
    (0.0, ACCEPTANCE_SETTLE_S, 0.0, 0.0, 0.0, "settle"),
    (
        ACCEPTANCE_SETTLE_S,
        ACCEPTANCE_SETTLE_S + BRAKE_BUILD_S,
        1.0,
        0.0,
        0.0,
        "build to the speed cap",
    ),
    (
        ACCEPTANCE_SETTLE_S + BRAKE_BUILD_S,
        ACCEPTANCE_SETTLE_S + BRAKE_BUILD_S + BRAKE_HOLD_S,
        -1.0,
        1.0,
        1.0,
        "full aft stick AND full steer (brake into a turn)",
    ),
    (
        ACCEPTANCE_SETTLE_S + BRAKE_BUILD_S + BRAKE_HOLD_S,
        ACCEPTANCE_SETTLE_S + BRAKE_BUILD_S + BRAKE_HOLD_S + 2.0,
        0.0,
        0.0,
        0.0,
        "release",
    ),
];

/// **Turn at cruise, no brake** — the control for [`BRAKE_TURN_SCHEDULE`].
/// Without it, "the turn is tighter when braking" has nothing to be tighter
/// than.
pub const CRUISE_TURN_SCHEDULE: Schedule = &[
    (0.0, ACCEPTANCE_SETTLE_S, 0.0, 0.0, 0.0, "settle"),
    (
        ACCEPTANCE_SETTLE_S,
        ACCEPTANCE_SETTLE_S + BRAKE_BUILD_S,
        1.0,
        0.0,
        0.0,
        "build to the speed cap",
    ),
    (
        ACCEPTANCE_SETTLE_S + BRAKE_BUILD_S,
        ACCEPTANCE_SETTLE_S + BRAKE_BUILD_S + BRAKE_HOLD_S,
        1.0,
        1.0,
        1.0,
        "hold speed AND full steer (turn at cruise)",
    ),
    (
        ACCEPTANCE_SETTLE_S + BRAKE_BUILD_S + BRAKE_HOLD_S,
        ACCEPTANCE_SETTLE_S + BRAKE_BUILD_S + BRAKE_HOLD_S + 2.0,
        0.0,
        0.0,
        0.0,
        "release",
    ),
];

/// Every schedule a `--scenario`/`--scripted-scenario` flag accepts, by name.
pub const BY_NAME: &[(&str, Schedule)] = &[
    ("default", DEFAULT_SCHEDULE),
    ("s-curve", S_CURVE_SCHEDULE),
    ("full-stick", FULL_STICK_SCHEDULE),
    ("stick-reversal", STICK_REVERSAL_SCHEDULE),
    ("brake-stop", BRAKE_STOP_SCHEDULE),
    ("brake-turn", BRAKE_TURN_SCHEDULE),
    ("cruise-turn", CRUISE_TURN_SCHEDULE),
];

/// Looks up a schedule by its command-line name.
pub fn by_name(name: &str) -> Option<Schedule> {
    BY_NAME.iter().find(|(n, _)| *n == name).map(|(_, s)| *s)
}

/// Comma-separated list of the accepted names, for error messages.
pub fn names() -> String {
    BY_NAME
        .iter()
        .map(|(n, _)| *n)
        .collect::<Vec<_>>()
        .join(", ")
}

/// When the last row of `schedule` ends.
pub fn total_duration_s(schedule: Schedule) -> f64 {
    schedule.iter().map(|s| s.1).fold(0.0, f64::max)
}

/// `(fore_aft, lateral, steer, label)` at schedule time `t`. Outside every
/// row -- before the first or after the last -- everything is zero.
pub fn value_at(schedule: Schedule, t: f64) -> (f32, f32, f32, &'static str) {
    for &(t0, t1, fa, lat, steer, label) in schedule {
        if t0 <= t && t < t1 {
            return (fa, lat, steer, label);
        }
    }
    (0.0, 0.0, 0.0, "end")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_named_schedule_resolves_and_is_contiguous() {
        for (name, sched) in BY_NAME {
            assert!(by_name(name).is_some(), "{name} does not resolve");
            assert!(!sched.is_empty(), "{name} is empty");
            for w in sched.windows(2) {
                assert!(
                    (w[0].1 - w[1].0).abs() < 1e-9,
                    "{name}: gap or overlap between rows ending {} and starting {}",
                    w[0].1,
                    w[1].0
                );
            }
        }
    }

    #[test]
    fn value_at_is_zero_outside_the_schedule() {
        let after = total_duration_s(S_CURVE_SCHEDULE) + 1.0;
        assert_eq!(value_at(S_CURVE_SCHEDULE, after), (0.0, 0.0, 0.0, "end"));
        assert_eq!(value_at(S_CURVE_SCHEDULE, -1.0), (0.0, 0.0, 0.0, "end"));
    }

    /// The build phase is 15 s of unchanging full-stick lean. Issue #190's
    /// whole point is that the flip happens ~7 s INTO that phase, so no
    /// amount of phase-boundary timing jitter can be what causes it.
    #[test]
    fn the_s_curve_build_phase_is_a_long_constant_full_stick_hold() {
        let (fa, _, _, label) = value_at(S_CURVE_SCHEDULE, SCURVE4_SETTLE_S + 0.5);
        assert_eq!(fa, SCURVE4_BUILD_LEAN);
        assert!(label.contains("build"));
        let (fa_late, _, _, _) = value_at(S_CURVE_SCHEDULE, B - 0.5);
        assert_eq!(fa_late, SCURVE4_BUILD_LEAN, "still full stick 14.5 s later");
        // The measured divergence is ~5.4 s into this phase. A schedule whose
        // build is shorter than that would stop reproducing issue #190 while
        // still looking like the same scenario, so pin the property that
        // makes the finding hold: the phase outlasts the divergence.
        let divergence_s = 5.4;
        assert!(
            SCURVE4_BUILD_S > divergence_s,
            "the build phase must outlast the measured divergence"
        );
    }
}
