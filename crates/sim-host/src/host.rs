//! The 500 Hz control loop, dedicated thread, UDP in/out (issue #161).
//!
//! Wires the SAME `control-core`/`safety` objects `control-ffi::ob_controller_
//! update` wires for the Python-hosted scenario, and `board-app-driverless`'s
//! `impulse-response-rust` wires for the Rust-hosted one -- reached here
//! through `hal` (`SimBackend::wait_observe`/`apply`) exactly like that
//! binary, just paced against a UDP client instead of a fixed step count.
//! Nothing here is a new control law.
//!
//! # W2 (issue #161/#169): the RIDDEN plant, and why it does not station-keep
//!
//! This host steps `sim/models/overboard_rider.xml`, not the driverless
//! onewheel model W1 used -- a ridden plant with a rider-scale ballast on
//! two slide joints. `weight_shift_fore_aft` drives the fore/aft joint
//! DIRECTLY AND PHYSICALLY (see that model's own header for the full
//! rationale and the sign convention): shifting the ballast forward moves
//! the effective centre of mass ahead of the axle, and the inner pitch loop
//! below -- which only ever holds the FRAME level, never a ground-speed
//! setpoint -- answers that by accelerating the wheel forward to keep the
//! frame upright. That is the entire mechanism. There is no outer velocity
//! loop here, and none is coming this weekend: `control_core::VelocityLoop`
//! exists and is deliberately unused.
//!
//! **The board does not station-keep, and is not supposed to.** A real
//! onewheel does not either -- lean forward to accelerate, level off to
//! coast at whatever speed that reached, lean back to decelerate or
//! reverse. An earlier revision of this file's controller-config comment
//! called this loop "pure station-keeping balance", which was true of W1's
//! driverless-plant, `pitch_ref`-only inner loop but became actively wrong
//! the moment this file switched to a ridden plant with a driven ballast --
//! nothing in this loop opposes net forward motion, and nothing should.
//!
//! # Issue #163: the heading now lives INSIDE the plant
//!
//! This host used to carry a synthetic heading alongside the physics: a
//! `yaw_rad` integrated from `steer`, composed onto MuJoCo's truth quaternion
//! on the way out, with the ground path dead-reckoned from real forward speed
//! projected along it. MuJoCo's own board only ever translated along one axis
//! underneath, so its true position bore no relation to what a renderer drew
//! -- which is why the drivable corridor had to be a soft host-side lean
//! rather than actual collision geometry.
//!
//! The same steering law now writes its increment straight into the plant's
//! free joint before each physics step (see the yaw block at the bottom of
//! [`run`], and `sim_backend::SimBackend::inject_kinematic_yaw`). **The
//! collision goal never required yaw to be physically GENERATED -- only
//! MuJoCo's pose to be AUTHORITATIVE**, and those are different problems.
//! Steering is still commanded rather than emergent: no tire model produces
//! this turn, and the `Playable Sim` declaration keeps saying so. But
//! position, ground path and every contact are now simulated at that heading,
//! nothing is dead-reckoned, and the MJCF is untouched.
//!
//! ## What it measured
//!
//! Every figure below is master vs this branch, run through
//! `--scripted-scenario` (issue #190/#191's sim-time-indexed player, so the
//! input path has no socket and no wall clock in it) and decoded off the wire
//! at full float precision -- not `wire-probe --csv`, whose 6-decimal
//! rounding cannot demonstrate bit identity.
//!
//! **Zero steer is bit-identical.** `--startup-kick` with no scenario and no
//! sender, so `steer` is 0 for the whole run; 7,854 common ticks. `pitch`,
//! `quat` (all four), `wheel_angle`, `wheel_rate`, `motor_current`,
//! `rider_fore_aft`, `rider_lateral`, `pos_z`, `sim_time` and `flags` are ALL
//! bit-identical. The plant is untouched, which is exactly what the
//! `dyaw == 0` gate exists to guarantee.
//!
//! Only `pos_x`/`pos_y` differ, which is the point of the change: those are
//! where the dead-reckoned path used to be reported and are now MuJoCo's own.
//! **The gap between them is the size of the error the old design was
//! shipping**: 14.7 mm over 7.37 m of travel (0.2%) in x, and 8.2 um in y --
//! where dead reckoning reported y as exactly 0.0, because it could not
//! represent lateral motion at all.
//!
//! **With steer on, the plant lands where the reckoning said it would.**
//! `--scripted-scenario default`, 9,000 ticks, tick-for-tick: `pitch` differs
//! by at most 2.1e-9 rad (1.2e-7 deg), `wheel_rate` by 4.8e-7 rad/s,
//! `motor_current` by 3.9e-7 A, the largest quaternion component by 8.3e-5,
//! and the commanded heading by 1.7e-4 rad. The path itself: 4.4 cm over a
//! 16 m run (0.28%) in x, 1.5 cm in y. The pitch envelope is IDENTICAL to
//! master's on the same schedule, -5.842..+6.390 deg -- the same envelope the
//! corridor-brake run recorded, and well inside the 10 deg carving limit.
//!
//! The reconstruction that comparison rests on is itself checked: replaying
//! the deleted dead-reckoning integral offline against master's own run
//! reproduces master's reported track to 0.0000 m, so it is the deleted
//! block and not an approximation of it. Truth attitude tracks the commanded
//! heading to 0.19 deg, and master's own truth attitude differs from its
//! `yaw_rad` by the same 0.19 deg -- that residual is the plant's own
//! contact-driven yaw, not the injection fighting the physics.
//!
//! **The `s-curve` flip (issue #190) is untouched.** Driven deterministically,
//! master and branch cross 20, 90 and 170 deg of pitch on the IDENTICAL tick
//! (seq 2933 / t = 5.868 s, seq 3070, seq 3216), and every physics column is
//! bit-identical for the first 7,375 ticks -- the whole zero-steer portion of
//! that run, which is where the flip happens. The two traces only separate
//! once `steer` goes non-zero at t = 14.75 s, by which point both boards are
//! already tumbling and diverge chaotically. That flip is issue #190's, and
//! nothing here moves it.

//! # ADR-0011: the command-envelope reserve, and the warning that was being
//! # thrown away
//!
//! Holding full forward stick from rest inverts this board (issue #190).
//! [`CMD_ENVELOPE_RESERVE`] is the fix ADR-0011 specifies -- one multiply on
//! the fore/aft stick, upstream of everything, derived from the actuator
//! envelope rather than tuned to the measured cliff. [`AUTHORITY_
//! UTILISATION_WARN`] is criterion (c): this file used to compute the safety
//! envelope's saturation bit every cycle and discard it at the binding, while
//! `FALLEN` -- which trips about a second after the outcome is decided -- was
//! the only thing anyone was told.
//!
//! **Two of the ADR's criteria are NOT met and no value of the constant would
//! meet them.** Fed MuJoCo truth (criterion (f)) the board inverts at every
//! stick fraction down to 0.05; the reserve buys time-to-inversion, not
//! survival. And during a full-stick hold the board cannot ride out much more
//! than a 1 mm pavement lip (criterion (a), kerb entry). Both are measured in
//! `tests/test_cmd_envelope_reserve.py` and recorded there as strict
//! `xfail`s. Read [`CMD_ENVELOPE_RESERVE`] and [`PitchSource`] before quoting
//! any margin out of this file.
//!
//! Everything under "verification only" on [`HostConfig`] exists to take
//! those measurements -- free running, a trace CSV, a pitch source, a static
//! pitch bias, a scheduled disturbance, and a reserve override. None of it is
//! reachable on a deployed run, and all of it is on the command line rather
//! than in a test fixture, because an acceptance number nobody else can
//! re-take is not a measurement.

use crate::pacer::Pacer;
use crate::wire::{self, InputIn, StateOut};
use board_types::{
    Command, Faults, ImuSample, Params, Saturation, DEFAULT_R_EFF_M, RAD_S_PER_ERPM,
};
use control_core::{CommandFeedforward, ComplementaryFilter, Estimator, PitchRegulator};
use hal::BoardObserve;
use hal_actuate::BoardActuate;
use safety::Envelope;
use sim_backend::SimBackend;
use std::io::ErrorKind;
use std::net::{SocketAddr, UdpSocket};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

/// Nanoseconds per control cycle -- 500 Hz, matching `sim-backend`'s own
/// `CYCLE_NS` (ICD SS11.2). Duplicated the same way that crate duplicates
/// `KT_NM_PER_A` against the model, because there is no public constant to
/// import; [`run`] asserts it against `SimBackend::run_metadata()` on open
/// rather than trusting the duplication silently.
pub const CYCLE_NS: u64 = 2_000_000;
const DT_S: f64 = CYCLE_NS as f64 * 1e-9;

/// The ridden rider model this host steps (issue #161 W2) -- NOT the
/// driverless onewheel model `sim-backend::SimBackend::with_params` defaults
/// to. Same env-macro pattern `sim-backend`'s own (private) `model_path()`
/// uses, since this crate is at the same `crates/<name>` depth.
fn rider_model_path() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../sim/models/overboard_rider.xml")
}

// --- Controller config, ridden variant (issue #161 W2) -- REUSED verbatim
// from `sim/scenarios/shuttle_run.py`'s own `controller_factory`, the repo's
// only existing tuned cascade config for a rider-scale plant. That scenario
// also runs an outer `VelocityLoop` this host does NOT enable (see this
// file's header) -- the inner-loop gains and estimator config below are
// reused unchanged regardless, because they are the closest available
// precedent for THIS plant's mass/inertia, not for the outer loop's absence.
const KP_NM_PER_RAD: f32 = 140.0;
const KD_NM_PER_RAD_S: f32 = 21.0;
/// Motor torque constant, N*m per amp -- DERIVED (issue: real-motor-
/// constants), not fitted or guessed. Real Onewheels (Pint/XR/GT) share the
/// "Hypercore" hub motor; VESC FOC detection on that motor measures flux
/// linkage lambda = 27.93 mWb and pole pairs p = 15 (30 poles). For a PMSM,
/// `Kt = 1.5 * p * lambda`:
///     Kt = 1.5 * 15 * 0.02793 = 0.6284 N*m/A
/// Corroborated independently from measured stopping distances (82.5 kg
/// rider+board, r = 0.1454 m): Pint 3.15 m/s^2 -> 260 N -> 37.8 N*m implies
/// 60.1 A of MOTOR current at this Kt; GT 3.37 m/s^2 -> 278 N -> 40.4 N*m
/// implies 64.3 A -- both landing in the same ~60-64 A band [`MAX_CURRENT_A`]
/// derives from, a second measurement agreeing with the FOC-detected value
/// rather than a fit to either. See `sim/scenarios/plant.py::KT_NM_PER_A` for
/// the full derivation. Replaces the previous unfitted placeholder of 0.7,
/// which this derivation shows was 11.4% optimistic.
const KT_NM_PER_A: f32 = 0.6284;
/// Envelope clamp on commanded current, amps.
///
/// **RE-DERIVED (issue: real-motor-constants), not a chosen number.** At
/// `KT_NM_PER_A` = 0.6284, 60 A gives `tau_max` = 60 * 0.6284 = 37.704 N*m,
/// which lands almost exactly on the measured-braking anchor of 37.8 N*m
/// (Pint's implied torque -- see [`KT_NM_PER_A`]'s own doc comment): 37.704
/// is 0.25% below it. 60 A is therefore the value the evidence itself
/// points at, not merely the value this constant already carried -- it
/// previously moved 40 -> 60 A on an "Onewheel-XR-class peak" annotation
/// that was banked without validation (issue: realistic-motor-torque); this
/// derivation is the validation that annotation never got, and it happens
/// to confirm the same number.
const MAX_CURRENT_A: f32 = 60.0;
const ESTIMATOR_TAU_S: f32 = 2.0;
/// Command-feedforward gain, m/s^2 per amp. `control_ffi::ob_controller_new`'s
/// own hardcoded fallback for `kt` 0.6284 N*m/A (DERIVED, issue:
/// real-motor-constants -- see `KT_NM_PER_A`'s own doc comment; was 0.7 N*m/A
/// when this constant was first derived) / (`r_eff` 0.1454 m * 82.5 kg total
/// ridden mass) -- 8 kg frame + 4.5 kg wheel + 0.5 kg ballast carrier + 70 kg
/// rider = 82.5 kg, matching `overboard_rider.xml` exactly:
///     0.6284 / (0.1454 * 82.5) = 0.0524
/// Re-derived from the corrected kt, not left at the value the old kt
/// produced -- `shuttle_run.py`'s tuned controller relies on this same FFI
/// fallback rather than passing its own gain, so this host does too,
/// duplicated here because this host does not link `control-ffi`.
const ACCEL_FF_GAIN_M_S2_PER_A: f32 = 0.0524;

/// `weight_shift_fore_aft` / `weight_shift_lateral`, both clamped to
/// `[-1, 1]` on the wire, map linearly onto this range -- the SAME +/-0.05 m
/// range `overboard_rider.xml`'s `ballast_fa` / `ballast_lat` joints
/// declare, so full-stick deflection lands exactly on the joint's own limit
/// rather than a separately chosen number that could silently drift from it.
const BALLAST_RANGE_M: f32 = 0.05;

/// Non-physical game channel, SHAPED by [`ROLL_FULL_YAW_AUTHORITY_RAD`]
/// below -- the simulated wheel is a cylinder and cannot physically carve
/// (issue #161). The real lean-steer controller is Tuesday.
///
/// **Unchanged in value and in law by issue #163's kinematic in-plant yaw
/// injection.** What changed is only where the resulting heading is APPLIED:
/// this same `steer * k * v * roll_authority` rate is now integrated into
/// MuJoCo's own free joint before each physics step, instead of being
/// composed onto the plant's output afterwards. The steering feel is signed
/// off and must not move; see [`run`]'s own yaw block.
///
/// # Revision history (moved through 1.5 -> 3.0 -> speed-proportional)
///
/// Started at 1.5 rad/s flat. Raised to 3.0 (issue #161 follow-up, CEO
/// request via the COO: "be more aggressive" for the launch-capture
/// scripted scenarios) -- a flat rad/s gain, still.
///
/// **Then the CEO drove it himself and found the real problem with a flat
/// gain**: "you can just straight up turn yourself around a bit too fast in
/// a way that I don't think you could actually do on a onewheel... it would
/// be on a very large turning radius." A flat rad/s yaw rate is independent
/// of ground speed -- the board could spin in place at a dead stop, and
/// turned equally fast at any speed, backwards from every real vehicle
/// (turn radius shrinks as you slow down, not the other way round). Fixed
/// by making yaw rate proportional to ground speed instead -- see
/// [`YAW_CURVATURE_PER_STEER_RAD_PER_M`], which replaces this constant
/// (the "gain" is no longer a flat rad/s; it's curvature per metre
/// travelled). Still an invented number for a declared non-physical
/// channel -- this changes nothing about the honesty position, only the
/// shape of the invented behaviour.
///
/// Turn radius = ground speed / yaw rate = `1 / (steer * k)` with this new
/// formulation -- roughly CONSTANT for a given `steer`, regardless of
/// speed, which is what leaning into a carve actually does; at zero speed,
/// yaw rate is zero too -- you cannot carve a stationary onewheel.
///
/// `k` (this constant) is picked for a believable radius at full steer, not
/// derived: 0.15 rad/m gives a tightest radius of `1 / 0.15` ≈ 6.7 m at full
/// steer, at ANY speed -- a wide, committed arc rather than a go-kart-tight
/// spin, closer to "very large turning radius" than the old flat gain's
/// spin-in-place. "Tune for feel, he will judge" -- this is a first
/// approximation, picked to be conservative ("reduce overall authority" per
/// the COO) rather than to hit an exact number; see this file's own
/// measured results for what it produces against the new speed cap.
const YAW_CURVATURE_PER_STEER_RAD_PER_M: f32 = 0.15;

/// How much tighter the turn gets at a standstill than at top speed, as a
/// multiple of [`YAW_CURVATURE_PER_STEER_RAD_PER_M`].
///
/// # Why this exists
///
/// [`YAW_CURVATURE_PER_STEER_RAD_PER_M`]'s own doc comment says the defect it
/// fixed was a board that "turned equally fast at any speed, backwards from
/// every real vehicle (**turn radius shrinks as you slow down**, not the other
/// way round)". It then made yaw RATE proportional to speed, which gives a
/// radius that is *constant* — better than the original, but still not the
/// behaviour the comment describes. Radius never shrank.
///
/// The CEO found the gap by driving it: *"in general i should be able to cut a
/// tight turn by breaking and turning at same time while keeping speed."* With
/// a speed-independent radius, braking into a turn cannot tighten it by any
/// amount, so the manoeuvre does not exist.
///
/// # What it does
///
/// Curvature ramps linearly from `1x` at [`YAW_TIGHTEN_REF_SPEED_M_S`] and
/// above, to this multiple at a standstill. At full steer and
/// `roll_authority` 1.0 that is a 6.67 m radius at top speed and 3.33 m as the
/// board comes to rest, passing through ~4.6 m at 5 m/s.
///
/// **The stationary board still cannot carve.** Yaw rate keeps its `|v|`
/// factor, so it remains zero at zero speed however tight the curvature gets
/// — the property `full_stick_at_zero_ground_speed_does_not_turn_the_board`
/// pins, and the one the original flat-gain formulation got wrong.
///
/// # Status
///
/// A FEEL number on a declared non-physical channel, picked for the manoeuvre
/// the CEO described and not derived from anything. 2x is deliberately modest:
/// it is a noticeable tightening under braking rather than a pivot. Nothing
/// here is a physics claim and no control decision may be tuned from it.
const YAW_LOW_SPEED_TIGHTEN: f32 = 2.0;

/// Speed at or above which the turn is at its widest, m/s — the reference
/// point [`YAW_LOW_SPEED_TIGHTEN`] ramps down from.
///
/// [`MAX_GROUND_SPEED_M_S`], so the ramp spans exactly the board's usable
/// speed range and the tightening is felt across all of it rather than only
/// in the last stretch before a stop. Above the cap the curvature simply
/// stays at its widest.
const YAW_TIGHTEN_REF_SPEED_M_S: f32 = MAX_GROUND_SPEED_M_S;

/// Ground-speed cap, m/s -- issue #161 follow-up, the CEO's explicit ask:
/// "look at the top speed of a Onewheel XR and set that limit +10% and cap
/// that." Future Motion's own widely-published Onewheel XR spec lists a top
/// speed of 19 mph (8.49 m/s); +10% = 20.9 mph = 9.34 m/s.
///
/// **Cited from well-known, publicly-repeated Onewheel XR spec material,
/// NOT a live re-verified lookup** -- this environment has no web access
/// this session, and the COO's own instruction was explicit ("verify that
/// figure rather than taking mine... this number is about to become a
/// public-facing claim of realism"). Flagging the distinction rather than
/// presenting this as independently re-confirmed: **spot-check the current
/// Future Motion Onewheel XR spec page against 19 mph before this cap is
/// treated as a defensible public claim**, per the standing project rule
/// against presenting an unverified number as verified.
///
/// Enforced as an authority cap on `weight_shift_fore_aft`, not a clamp on
/// the physical wheel speed itself -- this plant has no outer velocity loop
/// (see this file's own header) and no other mechanism that HOLDS a speed,
/// so the only way to cap it is to remove the rider's ability to keep
/// commanding forward lean once at the limit, the same way a real
/// Onewheel's pushback works. See the ballast-target section below for the
/// implementation and [`SPEED_CAP_MARGIN_M_S`] for why it ramps rather than
/// cuts off sharply. This also supersedes the "rip" scripted-scenario
/// tuning: `send-input`'s s-curve peaked at 18.5 m/s, roughly double this
/// cap -- that clip is now historical, not a target to preserve.
const MAX_GROUND_SPEED_M_S: f32 = 9.34;

/// How far below [`MAX_GROUND_SPEED_M_S`] the fore/aft accelerating
/// authority starts ramping down, so the cap is a smooth taper rather than
/// a hard on/off wall that could chatter (cross the cap, lose all forward
/// authority, decay slightly under drag, regain authority, repeat). Not
/// bench-tuned -- a documented default.
///
/// **Measured** (`wire-probe --csv`, sustained full lean from rest): speed
/// climbs smoothly and peaks at 9.95 m/s -- about 6.5% over the 9.34 m/s
/// cap, then settles back down through 9.3 m/s within about a second as the
/// authority ramp (and the ballast's own rate limit) catch up -- before
/// continued carving pulls it down further. Reported honestly rather than
/// re-tuned to hit the cap exactly: a documented, bounded overshoot, not an
/// unbounded one. Tighten this margin (or start the ramp earlier) if a
/// tighter cap is wanted; a smaller margin trades a harder-edged feel for
/// less overshoot.
const SPEED_CAP_MARGIN_M_S: f32 = 1.0;

/// The speed at which [`SPEED_CAP_MARGIN_M_S`]'s authority ramp starts
/// withdrawing accelerating fore/aft authority -- 8.34 m/s. Named because
/// ADR-0011's loss-of-authority warning discriminates on it (see
/// [`AUTHORITY_UTILISATION_WARN`]), not merely because the speed cap
/// arithmetic uses it.
const SPEED_CAP_ONSET_M_S: f32 = MAX_GROUND_SPEED_M_S - SPEED_CAP_MARGIN_M_S;

/// Fraction of full fore/aft stick the command map actually delivers --
/// ADR-0011 exit criterion (b), "cap commanded lean, changing nothing else".
/// Applied to `weight_shift_fore_aft` UPSTREAM of the estimator, the
/// regulator and the safety envelope, so it is pure input shaping: no gain
/// moves, `MAX_CURRENT_A` does not move (that would be a claim about
/// hardware), `MAX_GROUND_SPEED_M_S` does not move (rejected as dominated --
/// it costs top speed and this does not).
///
/// # DERIVED, NOT TUNED -- and specifically NOT tuned to the measured cliff
///
/// Holding full forward stick from rest inverts this board in ~6.5 s
/// (issue #190). The flip boundary was measured between 0.95 and 0.97 of
/// full stick, and **tuning to that boundary is forbidden**: it rests partly
/// on the estimator's operating trim and partly on `wheel_hinge`'s
/// undocumented `damping="0.08"`. A constant sitting on either is a constant
/// that moves when somebody changes something unrelated.
///
/// The derivation instead runs off the actuator envelope, per the
/// command-map rule ADR-0011 propagates to the hardware spec -- *the
/// stick->setpoint map SHALL be derived from the actuator envelope minus a
/// stated disturbance reserve*:
///
/// 1. **Measured** -- peak fore/aft current demand at
///    [`PEAK_DEMAND_A_PER_UNIT_STICK`] = **49.79 A per unit**, re-measured
///    for `KT_NM_PER_A`'s correction to a derived 0.6284 (issue:
///    real-motor-constants; supersedes the 44.68 A/unit figure taken at the
///    unfitted kt=0.7); see that constant's own doc comment for the method,
///    for why this DOES scale cleanly as `1/kt` (unlike an earlier
///    measurement taken before `ACCEL_FF_GAIN_M_S2_PER_A` was re-derived
///    alongside it), and for why "linear in stick" is an approximation
///    rather than a near-exact fit. Full stick therefore demands ~49.8 A
///    against a **60 A** envelope: **the present map still does not
///    over-command the actuator** -- less margin than the 44.68 A/unit
///    figure gave (83.0% utilisation vs 74.5%), but not over the line, and
///    the opposite of the 40 A finding this whole mechanism was built to
///    answer (see [`PEAK_DEMAND_A_PER_UNIT_STICK`]: 49.79/40 = 124.5%, an
///    over-command).
/// 2. **Stated reserve** -- hold peak demand to
///    [`STATED_ENVELOPE_RESERVE_FRACTION`] = 84% of [`MAX_CURRENT_A`], a
///    policy input rather than a measurement, and labelled as one.
/// 3. **Derived:** `0.84 * 60 A / 49.79 A per unit = 1.012`, which is **out of
///    the domain a shaping fraction can sensibly occupy** -- multiplying the
///    stick by more than 1.0 would command MORE than full stick, which is not
///    a reserve, it is amplification. [`CMD_ENVELOPE_RESERVE`] therefore
///    SATURATES at its domain ceiling, **1.00 -- i.e. no shaping at all.**
///    This is a finding, not a tuning choice: at 37.704 N*m, ADR-0011's founding
///    premise (full stick over-commands the actuator) is gone, so the
///    mechanism this constant drives degenerates to a no-op -- **only just:
///    the real, lower kt pushed the raw figure to within 1.2% of 1.0 (down
///    from the old kt=0.7 belief's 1.128), close enough that a slightly
///    higher `PEAK_DEMAND_A_PER_UNIT_STICK` measurement would flip this
///    mechanism back to actually shaping something.** At the 40 A
///    envelope the same arithmetic gives `0.84 * 40 / 49.79 = 0.675` --
///    lower than ADR-0011's third ratification's 0.752 figure (which was
///    taken at kt=0.7) -- so at 40 A the mechanism is NOT a no-op and would
///    need to ship at 0.67, not the previously-shipped 0.80, if 40 A were the
///    shipped envelope. Pinned as
///    executable arithmetic (including the clamp) by
///    `the_command_envelope_reserve_is_the_stated_reserve_divided_by_the_
///    measured_slope`, not merely written down here. **Whether the reserve
///    mechanism should therefore be retired, and what that does to
///    [`CMD_ENVELOPE_RESERVE_BRAKING`]'s "braking gets more authority than
///    accelerating" premise (issue #218/#219), is an open decision-record
///    question this patch does not resolve** -- see that constant's own
///    doc comment.
///
/// # TRIM-DERIVED, not geometry-derived -- read this before porting it
///
/// The step above says "measured", and the word is load-bearing in a way the
/// phrase *derived from the actuator envelope* can easily be read past.
/// [`PEAK_DEMAND_A_PER_UNIT_STICK`] was measured **at the estimator's current
/// operating trim**, and it is a property of that trim, not of the vehicle.
/// Measured in `test_f2_the_peak_demand_slope_is_trim_derived_not_geometry_
/// derived`: shifting the trim by 0.10 deg moves the slope by ~5.2%, which
/// is already outside the 5% band its own provenance check allows.
///
/// So this constant is honest for the frozen trim and for nothing else. It
/// does not travel to hardware, to a retuned filter, or to a different
/// operating point on its own authority. Describing it as geometry-derived --
/// as something the actuator envelope alone fixes -- would be a
/// rationalisation, and ADR-0011's second ratification says so in those
/// words. The trim it rests on is pinned by
/// `test_f1_the_estimator_trim_is_pinned_so_a_retune_cannot_move_it_silently`
/// precisely so this constant cannot go stale quietly; ADR-0011 names any
/// retune that moves that band as a blocking prerequisite for the
/// headroom-based fix, which is the version of this that would NOT be
/// trim-derived.
///
/// # What it buys, measured -- AT THE OLD 40 A / 28 N*m ENVELOPE
///
/// **Stale as of the 60 A / 42 N*m envelope (issue: realistic-motor-torque)
/// -- kept verbatim below as the historical record the 0.80 constant was
/// actually taken from, NOT re-run at 60 A.** `CMD_ENVELOPE_RESERVE` is now
/// 1.00 (see above), so there is no "shaped" column to re-measure against --
/// the comparison this table drew is between two settings of a mechanism
/// that no longer shapes anything. See the crate-level re-measurement
/// (`tests/test_cmd_envelope_reserve.py`, sim-host CLI) for the 60 A numbers.
///
/// Against the highest stick fraction that survived unshaped (0.95 -- 1.00
/// inverts, so it cannot be the baseline), over a 60 s full-stick hold on the
/// deployed estimator path:
///
/// | | 0.95 (unshaped baseline) | 0.80 (shipped) |
/// |---|---|---|
/// | peak demand | 40.25 A -- envelope reached | **33.41 A (83.5%)** |
/// | peak lean | 9.94 deg | **7.96 deg** |
/// | pitch reserve vs the 11.46 deg ceiling | 13% | **31%** |
///
/// The ceiling was `MAX_CURRENT_A * KT / KP` = 28/140 = 0.2 rad = 11.46 deg
/// at 40 A; at 60 A it is 42/140 = 0.3 rad = **17.19 deg**.
///
/// # What it costs, measured -- AT THE OLD 40 A / 28 N*m ENVELOPE
///
/// Also stale for the same reason -- kept as the historical record, not
/// re-run at 60 A.
///
/// Top speed is **unchanged**: the board settles at 9.150 m/s against the
/// baseline's 9.176 m/s (-0.3%), because `MAX_GROUND_SPEED_M_S` governs it
/// and a shaped full stick still reaches the cap. Time to 8 m/s from rest
/// rises 5.910 s -> 6.832 s (**+0.92 s**) and distance over the first 15 s
/// falls 108.83 m -> 100.23 m (**-7.9%**).
///
/// # What it does NOT buy -- read this before quoting the numbers above
///
/// The reserve satisfies criterion (b) and two of criterion (a)'s three
/// matrix entries **on the deployed estimator path**. It does not satisfy
/// the other two, and no value of it would:
///
/// - **Criterion (f) -- fed MuJoCo truth -- fails at every value.** Sweeping
///   this constant from 1.00 down to 0.05 moves time-to-inversion from 4.42 s
///   to 28.67 s and never removes it; only exactly zero stick survives.
///   Feeding the regulator truth deletes the acceleration reference the lean
///   is generated from, so a sustained forward lean has no equilibrium and
///   the reserve buys TIME, not survival. ADR-0011's second ratification
///   replaces this criterion with freeze-and-pin for exactly that reason.
/// - **Criterion (a)'s kerb-strike entry fails** above roughly a 1 mm lip
///   struck at the worst point of a full-stick hold.
///
/// Both are measured in `tests/test_cmd_envelope_reserve.py`, which records
/// them as strict `xfail`s -- the criteria are written there as they should
/// read, and they will turn red the day somebody fixes the underlying
/// problem, which is the only way a known failure stays known.
///
/// # SATURATED AT 1.00 -- issue: realistic-motor-torque (60 A / 42 N*m under
/// the old kt=0.7), re-derived and STILL SATURATED under issue:
/// real-motor-constants (60 A / 37.704 N*m at the corrected kt=0.6284)
///
/// **This is the shipped value, not a target.** Step 3 of the derivation
/// above computes 1.012 from the measured slope -- above 1.0, which is not a
/// valid shaping fraction (it would command more than full stick). The
/// constant clamps to the domain ceiling instead, which arithmetically means
/// **no shaping at all**: `full_stick_over_commands_the_envelope_...`'s own
/// premise (that unshaped demand exceeds the envelope) is still FALSE and
/// that test is written to say so rather than to hide it. The margin over
/// 1.0 shrank with the real, lower kt (1.012 vs the old belief's 1.128), which
/// is the direction the CEO's directive predicted -- less torque authority
/// means less margin, even where the conclusion does not flip -- and it is
/// now thin enough (1.2% above the clamp) that this conclusion should be
/// treated as fragile, not settled. Every "what it
/// buys" number above this note is therefore a description of the OLD (40 A)
/// mechanism, not of what ships now -- read the note on that section.
const CMD_ENVELOPE_RESERVE: f32 = 1.00;

/// The command-envelope reserve spent when the stick OPPOSES the current
/// motion — i.e. when the rider is braking rather than accelerating.
///
/// # Why braking gets its own number at all
///
/// [`CMD_ENVELOPE_RESERVE`] was derived from a peak-demand slope measured on
/// the **forward full-stick hold**, where the board accelerates from rest and
/// full stick over-commands the envelope by 5%. Applying that same 0.80 to a
/// braking command was never derived, only inherited: the shaping was one
/// multiply and the sign never entered it. The cost is that the rider gets
/// 80% of the lean they asked for when trying to stop, for a reason that only
/// ever applied to setting off.
///
/// The CEO's report from driving the build is the ask this answers: *"you
/// should be able to stop faster by leaning back [...] but we don't need to
/// overdo it."*
///
/// # Why the test is OPPOSITION, not sign
///
/// The naive version of this is "aft stick gets more authority". That
/// reintroduces the ADR-0011 defect in mirror image: full aft from rest is a
/// backward standing start, which over-commands the envelope exactly as the
/// forward one does. Braking is not a direction, it is a *relationship*
/// between the command and the motion, so the test is `stick * speed < 0` —
/// the same one the speed cap already uses.
///
/// # What it costs, and why this value
///
/// Braking authority is not free: it is spent out of the same envelope, and
/// the worst point in ADR-0011's acceptance matrix — the full reverse-to-
/// forward stick reversal at speed — is a braking event by this definition.
/// Raising this constant eats that entry's headroom directly, and the two
/// cannot be separated, because braking hard from speed IS that load case.
///
/// At 0.90 the stop is 9.3% quicker and 7.7% shorter, and the reversal's
/// pitch headroom falls from 1.72 deg to **0.47 deg** (current headroom 4.60
/// A to 3.41 A). **That is thin, and it is a deliberate CEO decision taken
/// with those numbers in front of him**, not a default — recorded here
/// because a future reader finding 0.47 deg of margin deserves to know it was
/// chosen rather than stumbled into. The sweep is in
/// `tests/test_braking_authority.py`: 0.95 exceeds the pitch ceiling outright
/// and 1.00 inverts the board.
///
/// ADR-0011 criterion (b) quotes the pre-change margin and needs amending.
///
/// Deliberately NOT raised to 1.0. At 1.0 the braking command is unshaped,
/// which reproduces the over-command the reserve exists to remove — the
/// direction it acts in is the only thing that changed.
///
/// # UNCHANGED at issue: realistic-motor-torque, and now BELOW
/// [`CMD_ENVELOPE_RESERVE`] -- flagged, not resolved
///
/// This is still a CEO policy call made against a PITCH/inversion ceiling
/// (the reversal-at-speed case), not against a current over-command --
/// unlike [`CMD_ENVELOPE_RESERVE`], nothing about its own derivation moved
/// when `MAX_CURRENT_A` went to 60 A, so it is left at 0.90 rather than
/// retuned. But [`CMD_ENVELOPE_RESERVE`] saturating at 1.00 means the
/// relationship this constant's whole premise depended on -- "braking gets
/// MORE authority than accelerating" (issue #218/#219) -- is now INVERTED:
/// 0.90 < 1.00, so an unshaped accelerating command now gets more authority
/// than a braking one. Whether braking should also saturate to 1.00, stay at
/// 0.90, or something else is a decision for whoever owns issue #218/#219 and
/// ADR-0011, not resolved here -- see the compile-time invariant just below,
/// which is relaxed rather than silently made to lie about this.
const CMD_ENVELOPE_RESERVE_BRAKING: f32 = 0.90;

/// Ground speed below which no command counts as braking, m/s.
///
/// Without this the opposition test fires on the standing start it is meant
/// to exclude. **Measured, not guessed:** a full-stick launch from rest rocks
/// the board BACKWARD to -0.085 m/s between t = 0.504 s and t = 1.112 s
/// before it moves off, so for six tenths of a second the forward stick
/// genuinely opposes the motion and would have drawn the braking reserve.
/// That would have changed ADR-0011's `full-stick` acceptance run — the one
/// the estimator trim and [`PEAK_DEMAND_A_PER_UNIT_STICK`] are both pinned
/// against — for a manoeuvre with no braking in it at all.
///
/// 0.25 m/s is about 3x that transient. Well under walking pace, so no stop
/// a rider would call a stop begins below it, and well above anything the
/// launch rock produces. The property that matters is pinned by
/// `the_braking_reserve_is_not_spent_on_a_standing_start`.
const BRAKING_RESERVE_MIN_SPEED_M_S: f32 = 0.25;

/// The braking reserve's bounds, enforced by the COMPILER where the
/// underlying premise still holds.
///
/// 1.0 would reproduce the over-command the reserve exists to remove, with
/// only the direction it acts in changed. And a full braking command must
/// not demand the whole envelope on its own, or there is nothing left for
/// the disturbance the reserve is named for.
///
/// **The THIRD bound this block used to enforce -- `BRAKING >
/// CMD_ENVELOPE_RESERVE`, "braking must have more authority than
/// accelerating" -- is REMOVED, not relaxed to pass.** At the 60 A / 37.704
/// N*m envelope [`CMD_ENVELOPE_RESERVE`] saturates at its domain ceiling, 1.00,
/// so no value of `CMD_ENVELOPE_RESERVE_BRAKING` below 1.0 can satisfy it;
/// enforcing it here would force this constant to 1.0 as a side effect of a
/// compiler check, which is exactly the "tune it to make the build pass"
/// move ADR-0011 forbids for its sibling constant. See
/// [`CMD_ENVELOPE_RESERVE_BRAKING`]'s own doc comment -- this is an open
/// decision-record question (issue #218/#219), not resolved by this patch.
const _: () = {
    assert!(
        CMD_ENVELOPE_RESERVE_BRAKING < 1.0,
        "an unshaped braking command is exactly the over-command ADR-0011's reserve \
         was introduced to remove"
    );
    assert!(
        CMD_ENVELOPE_RESERVE_BRAKING * PEAK_DEMAND_A_PER_UNIT_STICK < MAX_CURRENT_A,
        "a full braking command must not demand the whole actuator envelope on its own"
    );
};

/// The measurement [`CMD_ENVELOPE_RESERVE`] is derived FROM, in amps of peak
/// fore/aft current demand per unit of fore/aft stick.
///
/// A named constant rather than a number in a sentence, so the derivation is
/// arithmetic a test can check rather than prose a reader has to trust --
/// see `the_command_envelope_reserve_is_the_stated_reserve_divided_by_the_
/// measured_slope`.
///
/// **Re-measured for `KT_NM_PER_A`'s correction to a derived 0.6284 (issue:
/// real-motor-constants)** -- the previous 44.68 A/unit figure (40-vs-60 A
/// campaign, issue: campaign-40-vs-60) was taken at the unfitted kt=0.7
/// placeholder. Amps demanded for a given torque scale as `1/kt`, so a lower
/// kt raises this slope even though nothing about `KP_NM_PER_RAD`,
/// `KD_NM_PER_RAD_S` or the plant's dynamics changed: `44.68 * 0.7 / 0.6284`
/// predicts ≈49.77 A/unit.
///
/// **The first re-measurement did NOT match that prediction (46.61 A/unit),
/// and the gap was a second missed propagation, not a genuine non-linearity:
/// [`ACCEL_FF_GAIN_M_S2_PER_A`] is ALSO derived from `KT_NM_PER_A` (`kt /
/// (r_eff * mass)`) and was still carrying its old value (0.0584, derived
/// from kt=0.7) when that measurement was taken.** A mismatched accel-aiding
/// gain feeds the estimator a systematically wrong acceleration correction,
/// which changes the closed-loop transient and hence this slope -- an
/// artifact of an inconsistent build, not a property of the real motor.
/// Once `ACCEL_FF_GAIN_M_S2_PER_A` was corrected to 0.0524 (the value
/// consistent with the SAME kt), re-measuring gave 49.79 A/unit -- matching
/// the `1/kt` prediction to within 0.04%, which is the confirmation that the
/// slope IS just amps-per-torque scaling once every kt-derived constant
/// agrees, not a new non-linearity.
///
/// Four runs of `--scripted-scenario full-stick --pitch-source estimator` at
/// stick fractions 0.30/0.50/0.70/0.90 (the same points
/// `tests/test_cmd_envelope_reserve.py::test_peak_current_demand_is_linear_
/// in_stick_at_the_documented_slope` checks), peak `|proposed_amps|` taken
/// PRE-envelope; least squares through the origin gives **49.79 A/unit**.
///
/// **The per-point ratio is not flat.** The corrected drag model's constant
/// `Crr` (Coulomb) term is a fixed offset independent of stick, so it
/// dominates proportionally more at small stick than at large -- see the
/// 44.68 A/unit measurement's own note on this (superseded by the figure
/// above, but the shape of the non-linearity is unchanged). The least-
/// squares-through-origin fit is therefore not a description of a genuinely
/// linear relationship -- it is the specific 4-point convention this repo's
/// own test already uses, kept for continuity with that test and because the
/// constant this feeds is used at FULL stick, where the local ratio is
/// closest to this global fit. A reader re-deriving this from scratch should
/// not assume linearity holds away from that region.
///
/// This slope IS sensitive to `KT_NM_PER_A` (via the `1/kt` amps-per-torque
/// relationship above, AND via `ACCEL_FF_GAIN_M_S2_PER_A`'s own dependence on
/// it), unlike the 44.68 A/unit measurement's own claim that raising
/// `MAX_CURRENT_A` alone does not move it -- that claim is still true (this
/// slope is pre-clamp and does not depend on the envelope it is compared
/// against), but it does not extend to kt, which enters the amps conversion
/// directly. What DOES move as a result: at the derived kt, full stick's
/// ~49.8 A demand is 83.0% of the 60 A envelope; at 40 A it would be 124.5%,
/// an over-command. See [`CMD_ENVELOPE_RESERVE`].
///
/// Measured on the ESTIMATOR path deliberately, even though ADR-0011
/// criterion (f) runs acceptance against MuJoCo truth. A command map is a
/// property of the command path, and the estimator is the only signal path a
/// real board has; more decisively, the truth-fed runs have no steady state
/// to take a peak demand FROM (see the criterion (f) results in
/// `tests/test_cmd_envelope_reserve.py`), so the slope is not measurable
/// there at all.
const PEAK_DEMAND_A_PER_UNIT_STICK: f32 = 49.79;

/// The stated disturbance reserve [`CMD_ENVELOPE_RESERVE`] is derived
/// against: peak fore/aft demand is held to this fraction of
/// [`MAX_CURRENT_A`], leaving the rest for disturbance rejection.
///
/// **A stated policy input, not a measurement** -- inherited from ADR-0011,
/// and labelled as such rather than dressed up as derived. What is derived
/// is the stick scale that delivers it. 0.84 leaves 16% (6.4 A) of the
/// envelope.
///
/// It is worth saying plainly what 6.4 A does and does not cover: it is
/// about 0.2 rad/s of pitch-rate rejection through `KD`, which is a small
/// disturbance. The reserve is sized against the envelope because that is
/// the number this repo can defend; it is NOT sized against the repo's own
/// reference disturbance, which is far larger than the envelope can answer
/// at any stick setting (issue #142, and criterion (a)'s kerb-strike entry).
const STATED_ENVELOPE_RESERVE_FRACTION: f32 = 0.84;

/// The derivation, enforced by the COMPILER rather than by a reader.
///
/// [`CMD_ENVELOPE_RESERVE`] is not an independent number: it is the stated
/// reserve fraction times the envelope, divided by the measured slope,
/// rounded to two figures **and clamped to 1.00** -- a shaping fraction above
/// 1.0 is not a smaller reserve, it is amplification past full stick, which
/// is not a value this constant may take regardless of what the arithmetic
/// says. Editing any one of the four inputs in isolation is a build error,
/// which is the point -- ADR-0011 forbids moving the constant to make a
/// scenario pass, and a rule enforced at compile time cannot be forgotten by
/// a session that never read the ADR. The clamp does not weaken that: it is
/// exercised right now (60 A against 49.79 A/unit gives a raw 1.012), it is a fixed
/// domain bound rather than a per-session escape hatch, and it is itself
/// pinned by `the_command_envelope_reserve_is_the_stated_reserve_divided_by_
/// the_measured_slope`.
///
/// The tolerance is half a rounding step. Two figures is what the
/// measurement supports; see [`PEAK_DEMAND_A_PER_UNIT_STICK`]'s spread.
const _: () = {
    let raw = STATED_ENVELOPE_RESERVE_FRACTION * MAX_CURRENT_A / PEAK_DEMAND_A_PER_UNIT_STICK;
    let derived = if raw > 1.0 { 1.0 } else { raw };
    let err = CMD_ENVELOPE_RESERVE - derived;
    assert!(
        err < 0.005 && err > -0.005,
        "CMD_ENVELOPE_RESERVE is no longer STATED_ENVELOPE_RESERVE_FRACTION * \
         MAX_CURRENT_A / PEAK_DEMAND_A_PER_UNIT_STICK, clamped to 1.00 and rounded to \
         two figures. Re-derive the constant from the measurement; do not adjust the \
         measurement to fit the constant."
    );
};

/// Time constant of the low-pass filter on authority utilisation, seconds --
/// the loss-of-authority warning's detector (ADR-0011 exit criterion (c)).
///
/// Not bench-tuned, and bounded rather than picked: it has to be long
/// relative to the 2 ms control period (or the warning chatters on a single
/// noisy cycle) and short relative to the lead time it exists to preserve
/// (or the filter eats the warning it is supposed to give). 0.1 s is 50
/// control cycles and 4% of the measured lead -- comfortably inside both
/// bounds, which is the whole requirement on it.
const AUTHORITY_UTILISATION_TAU_S: f32 = 0.1;

/// Filtered `|proposed current| / MAX_CURRENT_A` above which the host warns
/// that it is running out of pitch authority -- ADR-0011 exit criterion (c).
///
/// # The discriminator is the SPEED, not the saturation
///
/// Saturation alone is not a fault on this board. ADR-0011's measured
/// finding: **every run that saturated above [`SPEED_CAP_ONSET_M_S`]
/// survived, and every run that saturated below it flipped.** Once torque
/// is clamped `tau = -kp*theta` no longer holds and pitch is open-loop
/// unstable, so saturation is survivable if and only if something is
/// already unloading the board when it happens -- above the onset the speed
/// cap is doing exactly that. A warning that fired on saturation above the
/// onset would be noise, and noise is how a warning gets ignored.
///
/// # Why filtered utilisation rather than the raw saturation bit
///
/// The raw bit (`Saturation::Yes` out of `safety::Envelope::apply`, which
/// this host used to discard at the `let (bounded_cmd, _sat)` binding) fires
/// only once the envelope has ALREADY bound. Filtered utilisation crosses
/// 0.85 while the controller still has authority left, which is the
/// difference between a warning and a post-mortem.
///
/// # Measured lead, against the reference ADR-0011 uses
///
/// `--scripted-scenario full-stick --cmd-reserve 1.0`, estimator path. The
/// reference event is the FIRST ENVELOPE SATURATION -- the ADR's table puts
/// it at 4.92 s and quotes `FALLEN` at -0.95 s, which is exactly `FALLEN`
/// minus that:
///
/// | event | sim time | lead over the committed point |
/// |---|---|---|
/// | this warning | 3.000 s | **+1.92 s** |
/// | first saturation (the reference) | 4.920 s | 0 |
/// | `FALLEN` (20 deg) | 5.868 s | **-0.95 s** |
/// | inverted past 90 deg | 6.142 s | -1.22 s |
///
/// So the warning leads `FALLEN` by **2.87 s**. ADR-0011 predicted 2.69 s of
/// lead over the committed point; the measurement is **1.92 s** with this
/// filter and threshold, and 2.02 s unfiltered. Recorded as measured rather
/// than reconciled -- the ADR's figure is not reproducible from the
/// definition it states, and quoting it anyway would be exactly the habit
/// that ADR exists to break.
///
/// # What this warning is NOT
///
/// It is a diagnostic, not a safety control. The board it warns about cannot
/// ride out a 2 mm pavement lip during a full-stick hold (criterion (a)'s
/// kerb entry, `tests/test_cmd_envelope_reserve.py`), and 1.92 s of notice
/// does not change that. It also does not reach the game: the state-out wire
/// is fixed by ADR-0010 until it expires, so this goes to the host's log and
/// to the acceptance trace and nowhere else. Putting it on the wire is a
/// schema change and needs that ADR's expiry or the COO's sign-off.
const AUTHORITY_UTILISATION_WARN: f32 = 0.85;

/// Roll magnitude, radians, at which the roll-shaped yaw limiter reaches
/// full authority (issue #161 W2 item 4).
///
/// **MEASURED, not guessed -- and the honest number is tiny.** The first cut
/// of this constant (10 deg) was a placeholder that turned out to be ~400x
/// too high: issue #169's follow-up measured 0.267 deg of yaw over the whole
/// `send-input` turn phase against a 10 deg threshold, which is what a
/// limiter that never leaves its floor looks like. Diagnosed by holding a
/// steady balance controller and commanding `ballast_lat` directly
/// (`sim/models/overboard_rider.xml`, full stick = 0.05 m target): the
/// actuator DOES reach its commanded position (~0.0401 m at 0.8 stick, ~full
/// convergence, ruling out a wiring/scaling bug) -- but the resulting
/// steady-state roll tops out at only **~0.032 deg at full stick (1.0)**.
/// The widened wheel geom (issue #161 W2, same PR) is the reason: a much
/// wider flat cylinder rim sitting on the ground plane resists tipping far
/// more than the original narrow tire did, and a 70 kg / 0.05 m lateral
/// shift's ~24.5 N*m of roll torque is nowhere near enough to peel it.
///
/// Recalibrated to slightly below that measured ceiling (not the ceiling
/// itself) so a genuinely full-stick lateral command reliably saturates
/// `roll_authority` to 1.0 with some margin, rather than sitting just under
/// it. This is a real physical limit of the current geometry, not a
/// placeholder -- but see [`YAW_AUTHORITY_FLOOR`] below for why this
/// threshold alone does NOT make yaw usable.
const ROLL_FULL_YAW_AUTHORITY_RAD: f32 = 0.025 * std::f32::consts::PI / 180.0;

/// Floor on `roll_authority`, below which the roll gate would otherwise clamp
/// `steer` to near-zero.
///
/// **Read this before touching the roll gate again.** At the widened wheel's
/// achievable roll (~0.03 deg, see [`ROLL_FULL_YAW_AUTHORITY_RAD`]'s doc
/// comment), a PURE roll-gated limiter is not "shaped by lean" in any
/// perceptible sense -- lean this small is not something a player can feel
/// or control, so without a floor the gate would function as an near-binary
/// on/off switch that happens to correlate weakly with the lateral stick,
/// not a smooth lean-to-turn feel. That is not what issue #161 W2 asked
/// for, and pretending otherwise in a comment is exactly the mistake #169
/// already caught twice in this crate.
///
/// So: **this gate is now largely cosmetic, and steer is effectively the
/// primary driver of yaw this weekend.** The floor guarantees full `steer`
/// always produces a usable turn (tens of degrees over a few seconds, not
/// tenths) regardless of how much roll the geometry can actually deliver;
/// the roll term still nudges authority up smoothly from the floor toward
/// 1.0 as lean increases, which is the connection to lean the design intends
/// to keep and what the real lean-steer controller (Tuesday) inherits --
/// it just cannot be the WHOLE story on this geometry. 0.35 is chosen so
/// full steer alone clears a "tens of degrees" turn even at zero lean
/// (`0.6 (steer) * 1.5 rad/s * 0.35 * a few seconds`); not bench-tuned.
const YAW_AUTHORITY_FLOOR: f32 = 0.35;

/// How long a stale input is still trusted before the host zeroes it rather
/// than continuing to act on a value from a client that may have gone away.
/// 50 control cycles (100 ms at 500 Hz): generous relative to any plausible
/// Unreal send rate, tight relative to a human's reaction time. A documented
/// default, not a bench-tuned number.
///
/// **Not as generous as that reasoning assumed** (issue #190): this crate's
/// own `send-input`, which claimed 50 Hz, was measured delivering 7-13 Hz on
/// a loaded dev laptop -- straddling this threshold, so a run-varying
/// fraction of every scripted run silently ran with the input zeroed. That is
/// fixed in the sender (absolute-deadline pacing) rather than by widening
/// this timeout, which is a safety property and stays where it is. Public
/// so a harness can quote the number it has to beat.
pub const INPUT_STALENESS_TIMEOUT: Duration = Duration::from_millis(100);

/// Placeholder threshold for the state-out FALLEN bit.
/// `sim/scenarios/disturbance_envelope.py` derives the REAL nose-strike
/// angle (18.57 deg) from the model's collision hulls; this crate has no
/// Rust binding to that geometry query, so this is a fixed proxy near that
/// value, not the real contact test -- good enough to prove "the board is
/// clearly down", not precise enough to gate a published claim.
const FALLEN_PITCH_RAD: f32 = 20.0 * std::f32::consts::PI / 180.0;

/// Soft, host-side drivable-corridor bounds, checked against **MuJoCo's own
/// truth position** -- issue #161 follow-up, item 4: the CEO wants the board
/// to stop passing through walls, curbs and map boundaries.
///
/// **CHANGED (issue #163, kinematic in-plant yaw injection): this used to be
/// checked in the DEAD-RECKONED frame, and no longer is.** The original
/// version of this comment explained at length why MuJoCo-frame bounds could
/// not work: `pos`'s x/y on the wire were dead-reckoned from real forward
/// speed projected along a synthetic heading, MuJoCo's own plant never turned
/// (it only ever translated along its own -X axis), so the position the
/// physics engine occupied bore no relation to where the board appeared on
/// screen. That is no longer true. The host now rotates the board's free
/// joint inside the plant every tick, so MuJoCo integrates the ground path
/// itself and its truth position IS the on-screen position. These bounds are
/// therefore now checked against `truth_pos_x_m`/`truth_pos_y_m` directly.
///
/// This remains a SOFT corridor -- a lean applied against travel, not a
/// contact -- because the model still declares no wall geometry. What has
/// changed is that adding some would now work: collision geometry in the MJCF
/// would be tested against the same position the renderer draws, which is the
/// thing the dead-reckoned design foreclosed.
///
/// Derived from the actual UE <-> MuJoCo origin mapping the COO supplied
/// (`OB_City`'s `BoardActor`, printed on launch: "MuJoCo origin -> UE
/// (-3880.0, -7450.0, -275.0) cm, yaw 90.0 deg"), not guessed outright, for
/// the ONE figure this repo already has independent measurement for: the
/// lateral half-width reuses `send-input`'s own already-measured value for
/// the SAME spawn point ("the road at OB_City's spawn carries road-level
/// surface to ~8.6 m either side" -- that crate's Revision 3 doc comments).
/// The longitudinal bounds have no equivalent measured figure anywhere in
/// this repo, so they are a deliberately generous placeholder -- wide
/// enough that no run measured so far against this issue (the longest,
/// 335 m, `send-input`'s Revision 4) comes close to it in either direction.
/// **Named constants specifically so the COO can tighten or widen them
/// from footage**, per their own explicit instruction.
///
/// **Verified the enforcement itself, not just the arithmetic** (`wire-probe
/// --csv`): since no scripted scenario on hand reaches these production
/// bounds (see above), `CORRIDOR_X_MIN_M` was temporarily tightened to -3.0
/// m for the measurement run and `send-input`'s `default` schedule (the
/// stable, already-validated AC schedule -- not `s-curve`) driven straight
/// at it. The edge-triggered log fired exactly once, at `(-3.0, 0.0)`;
/// the reported position (then dead-reckoned, now truth) overshot to -10.3 m
/// under residual momentum before the
/// brake arrested it, settling to a steady -9.4..-10.3 m band for the rest
/// of the run rather than continuing to run away -- an active brake against
/// momentum, not a teleport back to the line, exactly as designed. Pitch
/// stayed within -5.8..+6.4 deg throughout: the corridor brake itself did
/// not destabilize the board. Bound reverted to -700.0 after the
/// measurement.
///
/// **Re-verified after the switch to truth position** (issue #163), by the
/// identical method -- `CORRIDOR_X_MIN_M` temporarily tightened to -3.0 m,
/// `send-input`'s `default` schedule driven at it, bound reverted afterwards.
/// The edge-triggered log fired exactly once, again at `(-3.0, 0.0)`; truth
/// `x` overshot to -7.6 m under residual momentum, then held at -7.3 m for
/// the rest of the run, and pitch stayed within -4.7..+6.2 deg. Same
/// behaviour, and the overshoot differs from the -10.3 m above only because
/// the board is carrying a different speed at the crossing -- `send-input`
/// is a wall-clock sender against a tick-paced host, so no two runs of the
/// same schedule cross the line at the same moment.
const CORRIDOR_X_MIN_M: f64 = -700.0;
const CORRIDOR_X_MAX_M: f64 = 50.0;
const CORRIDOR_HALF_WIDTH_M: f64 = 8.6;

/// Fore/aft lean the corridor forces once the board is outside
/// [`CORRIDOR_X_MIN_M`]/[`CORRIDOR_X_MAX_M`]/[`CORRIDOR_HALF_WIDTH_M`],
/// opposing whatever direction it was travelling -- an actual brake, not
/// merely "stop accelerating further", which alone would let the board
/// coast on out of the corridor under its own residual speed. Same order
/// of magnitude as `send-input.rs`'s own `SCHEDULE`'s "lean back
/// (decelerate)" value (0.6); not bench-tuned.
const CORRIDOR_BRAKE_LEAN: f32 = 0.6;

/// A ONE-TIME startup disturbance, applied near the beginning of a run when
/// [`HostConfig::startup_kick`] is set. Not a general disturbance API -- the
/// input wire carries no such field (issue #161) -- and OFF BY DEFAULT
/// (issue #169): on the ridden plant, an ungated kick was measured handing
/// the board roughly 1.1 m/s before a player had touched anything, which is
/// enough to leave a finite Unreal level in the first few seconds of load.
/// Kept available, not deleted, for `wire-probe`/diagnostic runs that want a
/// guaranteed disturbance to show recovery from. Same magnitude and duration
/// `impulse-response-rust` uses (issue #107 AC6: `NOMINAL_IMPULSE_NS` = 20
/// N*s over 0.05 s), reused rather than invented; direction and application
/// point mirror that binary's own `ImpulseParams` defaults too (force along
/// -X, zero torque).
const STARTUP_KICK_T0_S: f64 = 1.0;
const STARTUP_KICK_DURATION_S: f64 = 0.05;
const STARTUP_KICK_FORCE_N: [f64; 3] = [-(20.0 / STARTUP_KICK_DURATION_S), 0.0, 0.0];

/// An ON-DEMAND disturbance (issue #161 follow-up, item 5): "make falls
/// testable" -- a repeatable, rising-edge-triggered "knock the board over
/// now" via [`wire::INPUT_FLAG_KICK`], reusing this same
/// `apply_external_force` mechanism the startup kick above already gates.
/// Deliberately a SEPARATE, much larger magnitude, not a reuse of
/// `STARTUP_KICK_FORCE_N` -- that one is sized to be RECOVERABLE (its own
/// doc comment: "a guaranteed disturbance to show recovery FROM"), and
/// measurement confirmed it as such on this plant: `wire-probe --csv`
/// against the 20 N*s/0.05 s startup kick alone never crossed
/// `FALLEN_PITCH_RAD` -- pitch stayed within a few degrees and recovered.
/// This one is sized to reliably NOT be recoverable. Measured
/// (`wire-probe --csv`'s new `fallen` column, decoded off the actual wire
/// bytes -- not re-derived from `pitch_rad` -- board at rest, zero
/// weight-shift/steer throughout, the ONLY input the whole run): 400 N*s
/// over 0.05 s (20x the startup kick) crossed `FALLEN_PITCH_RAD` (20 deg)
/// ~0.056 s after the kick's own force-application window (measured against
/// SIMULATED time, i.e. tick count * `DT_S` -- a verification run under this
/// dev sandbox's CPU contention showed the sender's own wall clock and the
/// host's simulated clock can drift apart by multiple real seconds, so this
/// crate's other schedule-timed tools, e.g. `send-input`'s wall-clock
/// `--kick-at`, should not be trusted for tight timing correlation here).
/// The board then went past a full flip (+-180 deg) and stayed there for
/// the rest of a 10 s run -- genuinely unrecoverable, not borderline. Same
/// direction/torque convention as the startup kick (force along -X, zero
/// applied torque -- the pitching moment comes from the wheel-ground
/// contact being below the force's application point, not from any
/// deliberately-applied torque).
const FALL_KICK_DURATION_S: f64 = 0.05;
const FALL_KICK_FORCE_N: [f64; 3] = [-(400.0 / FALL_KICK_DURATION_S), 0.0, 0.0];

/// How much simulated time a scripted run keeps stepping for after its
/// schedule has finished, so the outcome of the last phase is visible rather
/// than the process exiting mid-manoeuvre. Same intent as `send-input`'s own
/// `--kick-at` tail.
const SCRIPTED_RUN_TAIL_S: f64 = 3.0;

/// Default path for the host's own missed-deadline/tick counters -- internal
/// tooling for `wire-probe`, NOT part of the Unreal wire (issue #161's wire
/// table has no room for either, and must not grow one for our own
/// convenience). Overwritten atomically (write-then-rename) a few times a
/// second; best-effort, since losing a write here must never interrupt the
/// control loop.
pub const DEFAULT_STATS_PATH: &str = "/tmp/overboard-sim-host-stats.txt";

/// What [`run`] needs to know before it starts.
pub struct HostConfig {
    /// Where state-out packets are sent.
    pub state_out_addr: SocketAddr,
    /// Where the input-in socket binds/listens.
    pub input_in_addr: SocketAddr,
    /// `None` runs forever; `Some(d)` stops after `d` has elapsed --
    /// primarily for tests and verification runs, not something a deployed
    /// host would set.
    pub duration: Option<Duration>,
    /// `None` disables the stats file entirely.
    pub stats_path: Option<PathBuf>,
    /// Applies the one-time startup disturbance if true. See
    /// [`STARTUP_KICK_T0_S`]'s doc comment for why this defaults to false.
    pub startup_kick: bool,
    /// **Verification only.** When set, the host plays this
    /// [`crate::scenario`] schedule itself, indexed on SIMULATED time, and
    /// ignores the input socket entirely for `weight_shift_*`/`steer`.
    ///
    /// # Why this exists (issue #190)
    ///
    /// The scripted scenarios were previously only playable through
    /// `send-input`, which indexed them on a WALL clock and shipped them over
    /// UDP into a 100 ms staleness gate. On a loaded, non-realtime dev host
    /// that made the effective input sequence a function of machine load:
    /// the same command produced a materially different run each time, and
    /// "this scenario is stable" was being concluded from a de-rated
    /// delivery of it (see [`crate::scenario`]'s header for the measured
    /// numbers).
    ///
    /// Indexing on `sim_time_s` removes the wall clock, the socket, the
    /// sender process and the staleness gate in one move. The plant is a
    /// fixed-timestep RK4 integrator with no stochastic terms, so with the
    /// input schedule pinned to tick count the whole run becomes repeatable
    /// -- which is what makes "does this schedule flip the board?" a question
    /// with an answer, rather than a coin flip.
    ///
    /// This is NOT how a deployed host runs, and it is not a substitute for
    /// `send-input`: the UDP path is the one Unreal actually uses, and only
    /// `send-input` exercises it.
    pub scripted_scenario: Option<crate::scenario::Schedule>,

    /// **Verification only.** Stops the run after this much SIMULATED time,
    /// independently of `duration`'s wall clock. A free-running acceptance
    /// sweep has no wall clock worth bounding, and a scripted run's own
    /// schedule length is not always the window a measurement wants.
    pub max_sim_time_s: Option<f64>,

    /// **Verification only.** Where the regulator's pitch and pitch rate come
    /// from. See [`PitchSource`]; ADR-0011 exit criterion (f) is why this
    /// exists.
    pub pitch_source: PitchSource,

    /// **Verification only.** A static pitch offset, DEGREES, added to
    /// whatever [`HostConfig::pitch_source`] hands the regulator. Positive is
    /// nose-up, matching ICD 10.1.
    ///
    /// This is the robustness test ADR-0011 criterion (f) was reaching for.
    /// The ADR asks for acceptance "with the estimator bias removed" and
    /// implements that as "feed the regulator MuJoCo truth" -- but the
    /// measured `est - truth` residual is not a static bias at all: settled,
    /// it is `atan(a_unaided / g)` to within 3%, i.e. 1 radian per g, i.e.
    /// the complementary filter reading the APPARENT VERTICAL. Replacing it
    /// with truth therefore does not remove an error, it deletes an
    /// acceleration reference and leaves a pitch-only regulator (see
    /// `PitchSource::PlantTruth`).
    ///
    /// Injecting a worst-case STATIC bias on top of the real signal path is
    /// the test that actually asks "does this margin survive an estimator
    /// that is wrong?" without silently changing which controller is under
    /// test. Both are run and both are reported;
    /// `tests/test_cmd_envelope_reserve.py` has the results.
    ///
    /// Second use, ADR-0011 (f2): a small offset here MOVES THE OPERATING
    /// TRIM while changing nothing else, which is how the peak-demand slope
    /// behind [`CMD_ENVELOPE_RESERVE`] is shown to be trim-derived rather
    /// than geometric.
    pub pitch_bias_deg: f32,

    /// **Verification only.** Runs the loop as fast as the CPU allows,
    /// skipping the [`Pacer`] entirely.
    ///
    /// The plant is a fixed-timestep RK4 integrator with no stochastic terms
    /// and the scripted schedule is indexed on simulated time, so a scripted
    /// free run produces bit-identically the same trajectory as a paced one
    /// -- it just produces it in a fraction of a second instead of in real
    /// time, which is what makes an acceptance SWEEP (stick fraction x
    /// scenario x pitch source) a thing that fits in a test rather than in an
    /// afternoon. Missed deadlines are not counted in this mode: there are no
    /// deadlines.
    ///
    /// NOT how a deployed host runs. The UDP state-out is still sent every
    /// tick, so a listener would see the whole run arrive at once.
    pub free_run: bool,

    /// **Verification only.** Fraction of full fore/aft stick to deliver,
    /// overriding [`CMD_ENVELOPE_RESERVE`].
    ///
    /// Exists so that constant's own provenance is re-measurable rather than
    /// merely asserted: sweeping this IS the "peak demand is linear at N amps
    /// per unit stick" measurement its doc comment cites. `None` uses the
    /// shipped constant.
    pub cmd_envelope_reserve: Option<f32>,

    /// **Verification only.** Overrides [`CMD_ENVELOPE_RESERVE_BRAKING`] --
    /// the reserve spent when the stick opposes the motion. Sweeping this is
    /// how that constant's cost against ADR-0011's worst matrix point is
    /// measured rather than assumed.
    ///
    /// `None` falls back to [`HostConfig::cmd_envelope_reserve`] if THAT is
    /// set, and only then to the shipped constant. So `--cmd-reserve X` on
    /// its own still scales the whole fore/aft command, which is what every
    /// sweep written before braking had its own reserve assumes -- including
    /// `--cmd-reserve 0` meaning "no stick at all".
    pub cmd_envelope_reserve_braking: Option<f32>,

    /// **Verification only.** A scheduled external disturbance -- see
    /// [`Disturbance`]. `None` applies none, which is the deployed
    /// behaviour.
    pub disturbance: Option<Disturbance>,

    /// **Verification only.** Ground incline, DEGREES, positive uphill in the
    /// board's forward direction. Zero is flat and leaves the plant's gravity
    /// untouched.
    ///
    /// ADR-0011's second ratification makes the authored world one of the
    /// three conditions the launch hold exits on: *the authored world is
    /// constrained to what the controller survives, and the constraint is
    /// encoded as a checkable asset rule.* An asset rule needs a number, the
    /// number is an incline, and this is how it gets measured
    /// (`tests/test_incline_tolerance.py`). It is deliberately a MEASUREMENT
    /// knob and not a scenario parameter: no shipped configuration sets it.
    ///
    /// Applied by [`sim_backend::SimBackend::set_incline_deg`], which rotates
    /// `mjModel::opt.gravity` rather than the ground geom -- see there for why
    /// that is the same problem and not an approximation of it.
    pub incline_deg: f64,

    /// **Verification only.** Multiplies the model's `wheel_hinge`
    /// `frictionloss` (the derived rolling-resistance term) by this factor,
    /// taking effect at the next `open()`. `1.0` (the default) leaves it
    /// exactly as the MJCF declares it -- no shipped configuration sets this
    /// to anything else.
    ///
    /// ADR-0011 criterion (g), re-derived: the launch-hold sweep originally
    /// varied the old lumped, undocumented `wheel_hinge damping="0.08"` by an
    /// arbitrary 0.5x-2.0x. That drag model was replaced by a physically
    /// derived pair (`frictionloss` for rolling resistance, `damping` for
    /// bearing/motor losses -- see `sim/models/overboard_rider.xml`), and the
    /// remaining physical uncertainty now lives entirely in Crr, which has
    /// PUBLISHED BOUNDS. This is the instrument that re-derived sweep needs.
    ///
    /// Applied by [`sim_backend::SimBackend::set_crr_scale`], which scales
    /// the joint's CURRENT resolved `dof_frictionloss` rather than asserting
    /// a literal, the same reasoning [`HostConfig::incline_deg`] follows for
    /// gravity.
    pub crr_scale: f64,

    /// **Verification only.** Writes one CSV row per control cycle to this
    /// path when the run ends. Buffered in memory and written once, never
    /// during the loop: a 500 Hz control thread does not do file I/O per
    /// tick, and the trace exists to measure the loop, not to perturb it.
    pub trace_path: Option<PathBuf>,
}

/// Where the regulator's attitude comes from -- ADR-0011 exit criterion (f).
///
/// The default is the only one a deployed host may use. `PlantTruth` exists
/// because the ADR's FIRST ratification required every acceptance pass to
/// hold **with the estimator bias removed**, and on this plant that is the
/// measured-WORSE case, not the better one: feeding the controller MuJoCo
/// truth makes the board flip EARLIER.
///
/// **The reason is not an accidental bias, and the ADR's second ratification
/// corrects that reading.** The `est - truth` residual is the apparent
/// vertical, `atan(a/g)` -- 1 radian per g, which is the same 5.84 deg per
/// m/s^2 the ADR derives geometrically as the lean this board must hold to
/// sustain acceleration `a`, because it is the same physics. The estimator is
/// supplying the textbook balance-vehicle lean. Substituting truth therefore
/// does not de-bias the loop, it deletes the only mechanism generating that
/// lean and leaves a pitch-only regulator -- a different controller, not a
/// cleaner one. What criterion (f) was right about is that nothing designed
/// this and nothing held it in place; (f1)/(f2) fix that by pinning it
/// (`tests/test_cmd_envelope_reserve.py`), and (f3) makes it explicit as a
/// `theta_ref = atan(a_des / g)` feedforward, at which point this instrument
/// becomes meaningful again.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PitchSource {
    /// The `ComplementaryFilter`, fed raw IMU through `hal` -- the real
    /// signal path, and the only one a real board has.
    #[default]
    Estimator,
    /// MuJoCo's own body-frame pitch and pitch rate, straight from the
    /// plant. Physically unavailable on hardware; an acceptance-run
    /// instrument only.
    PlantTruth,
}

/// A scheduled external force/torque disturbance, world frame, applied to the
/// `frame` body over a fixed window of SIMULATED time.
///
/// Generic on purpose. The disturbance ADR-0011's test matrix needs is a kerb
/// strike, and the kerb derivation this repo trusts lives in ONE place --
/// `sim/scenarios/plant.py::kerb_strike_impulse` (issue #142/#147, Sr.
/// Mechanical & Systems' call, not Controls'). Re-implementing that geometry
/// in Rust would give this repo two kerb models that could disagree, so this
/// host takes the derived impulse as a parameter and stays dumb about where
/// it came from; `tests/test_cmd_envelope_reserve.py` calls the Python
/// derivation and passes the result in.
#[derive(Debug, Clone, Copy)]
pub struct Disturbance {
    /// Simulated time the force window opens, seconds.
    pub t0_s: f64,
    /// How long it stays open, seconds. Impulse delivered is `force * this`.
    pub duration_s: f64,
    /// World-frame force, newtons.
    pub force_n: [f64; 3],
    /// World-frame torque about the frame body's own origin, N*m.
    pub torque_nm: [f64; 3],
}

impl Default for HostConfig {
    fn default() -> Self {
        HostConfig {
            state_out_addr: wire::state_out_addr(),
            input_in_addr: wire::input_in_addr(),
            duration: None,
            stats_path: Some(PathBuf::from(DEFAULT_STATS_PATH)),
            startup_kick: false,
            scripted_scenario: None,
            max_sim_time_s: None,
            pitch_source: PitchSource::Estimator,
            pitch_bias_deg: 0.0,
            free_run: false,
            cmd_envelope_reserve: None,
            cmd_envelope_reserve_braking: None,
            disturbance: None,
            incline_deg: 0.0,
            crr_scale: 1.0,
            trace_path: None,
        }
    }
}

/// One control cycle's worth of acceptance-run instrumentation. See
/// [`HostConfig::trace_path`]; the column list and its order are the CSV
/// header written by [`write_trace`].
#[derive(Debug, Clone, Copy)]
struct TraceRow {
    seq: u64,
    sim_time_s: f64,
    /// Fore/aft stick as the schedule (or the socket) supplied it, before
    /// [`CMD_ENVELOPE_RESERVE`].
    stick_fore_aft: f32,
    /// After the reserve, before the speed cap and the corridor brake.
    shaped_fore_aft: f32,
    /// What actually reached `set_ballast_targets`, after everything.
    applied_fore_aft: f32,
    /// MuJoCo truth, de-yawed, degrees, nose-up positive.
    truth_pitch_deg: f32,
    /// The complementary filter's belief, degrees -- recorded even on a
    /// `PitchSource::PlantTruth` run (the estimator still runs; it is simply
    /// not the regulator's input) so the bias itself stays visible.
    est_pitch_deg: f32,
    /// ... and the rate it believes, which is the regulator's D-term input on
    /// a deployed run. Recorded alongside the truth rate because the two are
    /// the pair criterion (f) swaps, and a swap nobody can see the size of is
    /// not a measurement.
    est_pitch_rate_deg_s: f32,
    truth_pitch_rate_deg_s: f32,
    forward_speed_m_s: f32,
    /// PRE-envelope demand, amps. The number the reserve is derived from.
    proposed_amps: f32,
    /// POST-envelope command, amps.
    applied_amps: f32,
    /// `safety::Envelope` reported the clamp bound this cycle.
    saturated: bool,
    /// `|proposed_amps| / MAX_CURRENT_A`, unfiltered.
    utilisation: f32,
    /// ... and low-passed at [`AUTHORITY_UTILISATION_TAU_S`].
    utilisation_filtered: f32,
    /// The loss-of-authority warning is asserted this cycle.
    authority_warning: bool,
    /// The wire's `FALLEN` bit, for the lead-time comparison.
    fallen: bool,
}

/// What a finished (or interrupted-by-error) run produced.
#[derive(Debug, Clone, Copy, Default)]
pub struct RunSummary {
    pub ticks: u64,
    pub missed_deadlines: u64,
}

/// Everything that can stop [`run`] early.
#[derive(Debug)]
pub enum HostError {
    Backend(board_types::IoError),
    Io(std::io::Error),
}

impl std::fmt::Display for HostError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            HostError::Backend(e) => write!(f, "sim backend error: {e:?}"),
            HostError::Io(e) => write!(f, "I/O error: {e}"),
        }
    }
}

impl std::error::Error for HostError {}

impl From<std::io::Error> for HostError {
    fn from(e: std::io::Error) -> Self {
        HostError::Io(e)
    }
}

/// The most recent input the host has heard from Unreal, plus the bits this
/// host does not yet act on (accepted, clamped, and logged on receipt --
/// issue #161 -- so a later pass only has to connect them to something).
#[derive(Debug, Clone, Copy, Default)]
struct LatestInput {
    weight_shift_fore_aft: f32,
    weight_shift_lateral: f32,
    steer: f32,
    armed_bit: bool,
    reset_bit: bool,
    kick_bit: bool,
}

impl From<InputIn> for LatestInput {
    fn from(p: InputIn) -> Self {
        let flags = p.flags;
        LatestInput {
            weight_shift_fore_aft: p.weight_shift_fore_aft,
            weight_shift_lateral: p.weight_shift_lateral,
            steer: p.steer,
            armed_bit: flags & wire::INPUT_FLAG_ARM != 0,
            reset_bit: flags & wire::INPUT_FLAG_RESET != 0,
            kick_bit: flags & wire::INPUT_FLAG_KICK != 0,
        }
    }
}

/// Spawns the control loop on its own dedicated thread (issue #161: "not on
/// the main thread") and returns the join handle. The caller decides what to
/// do with the calling thread -- `src/bin/sim-host.rs` just joins it.
/// Body-frame pitch and roll, radians, from the `frame` body's world rotation
/// matrix (row-major `xmat`) and the world-frame heading currently baked into
/// the plant.
///
/// Pitch is `atan2(R[0][2], R[2][2])` -- exactly
/// `sim/scenarios/impulse_response.py::frame_pitch_rad`'s formula against the
/// identical array, nose-up positive per ICD 10.1 -- and roll is the same
/// atan2-of-a-tilted-axis derivation applied to the Y-Z plane (about local X)
/// instead of the X-Z plane (about local Y). Both are exact only when the
/// OTHER angle is near zero: 3D rotations do not commute and this is not a
/// true Euler decomposition. Acceptable for the roll-shaped yaw limiter's
/// "cheap stopgap" status (issue #161 W2 item 4); the real lean-steer
/// controller needs a better one.
///
/// **`yaw_rad` must be removed FIRST, and that is why this function exists**
/// (issue #163). Those formulas assume the world x/y axes still line up with
/// the body's, which was true only while the board had no yaw freedom at all
/// -- precisely what the kinematic in-plant yaw injection changed. `xmat` is
/// now `Rz(yaw) * R_body`, whose third column mixes body pitch and body roll
/// by the heading angle, so after a 90 deg turn the raw formulas would report
/// the board's ROLL as its PITCH and `FALLEN` would fire on an upright board.
/// Left-multiplying by `Rz(-yaw)` -- which is all the two `deyawed_*` lines
/// are -- recovers the body-frame values the ICD and the roll gate both mean.
///
/// At `yaw_rad == 0` this reduces to `1.0 * xmat[k] + 0.0 * xmat[j]`, i.e.
/// bit-identically the pre-#163 expressions; `deyaw_at_zero_yaw_is_a_no_op`
/// pins that.
fn body_pitch_roll_rad(xmat: &[f64; 9], yaw_rad: f32) -> (f32, f32) {
    let (yaw_s, yaw_c) = yaw_rad.sin_cos();
    let deyawed_02 = yaw_c * xmat[2] as f32 + yaw_s * xmat[5] as f32;
    let deyawed_12 = -yaw_s * xmat[2] as f32 + yaw_c * xmat[5] as f32;
    (
        deyawed_02.atan2(xmat[8] as f32),
        deyawed_12.atan2(xmat[8] as f32),
    )
}

/// The command-envelope reserve, as a function (ADR-0011 criterion (b)).
///
/// One multiply, extracted from the loop body only so the property that
/// matters can be stated as a test rather than as a comment: this is a
/// LINEAR scaling of the stick, symmetric about zero, and it changes nothing
/// else. In particular it is not a clamp -- a clamp would leave full stick
/// untouched right up to a limit and then bite, which is a different feel and
/// a different failure mode.
fn shape_fore_aft_command(stick: f32, reserve: f32) -> f32 {
    stick * reserve
}

/// Like [`shape_fore_aft_command`], but spends the BRAKING reserve when the
/// command opposes the current motion. See [`CMD_ENVELOPE_RESERVE_BRAKING`].
///
/// The opposition test is `stick * speed < 0`, deliberately the same one
/// [`speed_capped_fore_aft`] already uses, so there is one definition of
/// "this command opposes the motion" in this file rather than two that can
/// drift apart.
///
/// **At rest the accelerating reserve applies** (`stick * 0.0 >= 0.0`), which
/// is the case that matters for safety: full aft from a standstill is a
/// backward standing start, not a stop, and it over-commands the envelope
/// exactly as the forward one does. That is the mirror of the defect
/// ADR-0011 was called for, and it keeps the accelerating reserve.
fn shape_fore_aft_command_directional(
    stick: f32,
    reserve: f32,
    braking_reserve: f32,
    last_forward_speed_m_s: f32,
) -> f32 {
    if last_forward_speed_m_s.abs() > BRAKING_RESERVE_MIN_SPEED_M_S
        && stick * last_forward_speed_m_s < 0.0
    {
        shape_fore_aft_command(stick, braking_reserve)
    } else {
        shape_fore_aft_command(stick, reserve)
    }
}

/// The speed cap's authority ramp, as a function -- unchanged in behaviour,
/// extracted so ADR-0011 criterion (a)'s reversal entry can be stated as a
/// unit test as well as a sim run.
///
/// The property worth pinning: a REVERSAL is never attenuated. The cap only
/// withdraws authority when the stick and the current motion share a sign,
/// so a stick slammed from one rail to the other at speed passes through at
/// FULL authority, at any speed, including above the onset where the cap is
/// the only thing unloading the board. That is why the reversal is the
/// matrix's worst case and not merely another entry in it.
fn speed_capped_fore_aft(stick: f32, last_forward_speed_m_s: f32) -> f32 {
    let speed_headroom_m_s = MAX_GROUND_SPEED_M_S - last_forward_speed_m_s.abs();
    let accel_authority = (speed_headroom_m_s / SPEED_CAP_MARGIN_M_S).clamp(0.0, 1.0);
    if stick * last_forward_speed_m_s >= 0.0 {
        stick * accel_authority
    } else {
        stick
    }
}

/// The loss-of-authority warning's trigger (ADR-0011 criterion (c)):
/// filtered authority utilisation over threshold, AND below the speed cap's
/// onset.
///
/// Both halves are load-bearing and the second is the one that is easy to
/// drop. Saturation above [`SPEED_CAP_ONSET_M_S`] is survivable -- the cap is
/// already unloading the board when it happens -- and every run in ADR-0011's
/// data that saturated above the onset survived. A warning without the speed
/// term would fire on all of those, which is a warning nobody reads.
/// Curvature per unit steer at this ground speed, rad/m — widest at
/// [`YAW_TIGHTEN_REF_SPEED_M_S`] and above, tightening toward a standstill by
/// [`YAW_LOW_SPEED_TIGHTEN`]. See that constant for why.
fn yaw_curvature_per_steer(forward_speed_m_s: f32) -> f32 {
    let slowness = 1.0 - (forward_speed_m_s.abs() / YAW_TIGHTEN_REF_SPEED_M_S).clamp(0.0, 1.0);
    YAW_CURVATURE_PER_STEER_RAD_PER_M * (1.0 + (YAW_LOW_SPEED_TIGHTEN - 1.0) * slowness)
}

/// The whole yaw law, in one place.
///
/// Extracted so the unit tests exercise the SHIPPED expression rather than a
/// copy of it — the same reason [`speed_capped_fore_aft`] is a function. The
/// zero-speed property below is the one this crate got wrong once already,
/// and a test asserting it against a re-typed formula would not have caught
/// it.
fn yaw_rate_rad_s(steer: f32, forward_speed_m_s: f32, roll_authority: f32) -> f32 {
    steer * yaw_curvature_per_steer(forward_speed_m_s) * forward_speed_m_s.abs() * roll_authority
}

fn authority_warning_active(utilisation_filtered: f32, forward_speed_m_s: f32) -> bool {
    utilisation_filtered > AUTHORITY_UTILISATION_WARN
        && forward_speed_m_s.abs() < SPEED_CAP_ONSET_M_S
}

pub fn spawn(cfg: HostConfig) -> std::thread::JoinHandle<Result<RunSummary, HostError>> {
    std::thread::Builder::new()
        .name("sim-host-control".into())
        .spawn(move || run(cfg))
        .expect("sim-host: failed to spawn the control thread")
}

/// Runs the 500 Hz closed loop until `cfg.duration` elapses (or forever, if
/// `None`), or until a backend/I-O error stops it. Blocking -- call this
/// from a spawned thread, not the process's main thread, unless the caller
/// has nothing else to do on main either.
pub fn run(cfg: HostConfig) -> Result<RunSummary, HostError> {
    let params = Params {
        kp_nm_per_rad: KP_NM_PER_RAD,
        kd_nm_per_rad_s: KD_NM_PER_RAD_S,
        kt_nm_per_a: KT_NM_PER_A,
        max_current_a: MAX_CURRENT_A,
        ..Params::default()
    };

    let mut backend = SimBackend::with_model_path(params, rider_model_path());
    // Before `open()`, which is where the tilt is applied -- see
    // `HostConfig::incline_deg`.
    backend.set_incline_deg(cfg.incline_deg);
    // Same reasoning, same timing -- see `HostConfig::crr_scale`.
    backend.set_crr_scale(cfg.crr_scale);
    backend.open().map_err(HostError::Backend)?;
    // Armed unconditionally at startup, the same way every other Rust-hosted
    // harness in this repo arms (`impulse-response-rust`, `sim-backend`'s own
    // tests): there is no synthetic Unreal client during a verification run,
    // and this host does not gate balancing on the input socket's `arm` bit
    // -- see `LatestInput::armed_bit`, tracked and logged but not wired to
    // anything.
    let _disarm = backend.arm().map_err(HostError::Backend)?;

    // CYCLE_NS is a duplicated constant, not derived -- checked against
    // sim-backend's own control rate the same way that crate checks
    // KT_NM_PER_A against the model's ctrlrange, rather than trusted blind.
    let reported_hz = backend.run_metadata().control_rate_hz as f64;
    let expected_hz = 1e9 / CYCLE_NS as f64;
    assert!(
        (reported_hz - expected_hz).abs() < 1e-6,
        "sim-host: CYCLE_NS ({CYCLE_NS} ns / {expected_hz} Hz) does not match \
         sim-backend's own control rate ({reported_hz} Hz)"
    );

    let mut envelope = Envelope::new(params);
    envelope.arm();

    let regulator = PitchRegulator::new(KP_NM_PER_RAD, KD_NM_PER_RAD_S);
    let mut estimator = ComplementaryFilter::with_trust_band(ESTIMATOR_TAU_S, 0.0);
    let accel_ff = CommandFeedforward::new(ACCEL_FF_GAIN_M_S2_PER_A);
    // Last cycle's POST-envelope commanded current, amps -- the feedforward's
    // input (mode 2 / "commanded", matching `shuttle_run.py`'s own default
    // `accel_ff_current_source`, rather than the measured-current mode
    // `control-ffi`'s doc recommends for hardware). One cycle old by
    // construction, same as `control-ffi::ObController::last_amps`.
    let mut last_amps: f32 = 0.0;

    let out_socket = UdpSocket::bind("127.0.0.1:0")?;
    let in_socket = UdpSocket::bind(cfg.input_in_addr)?;
    // The 500 Hz loop must never block on recv (issue #161).
    in_socket.set_nonblocking(true)?;

    let mut latest_input = LatestInput::default();
    let mut latest_input_at: Option<Instant> = None;
    // Last logged scripted-schedule phase label, so a scripted run announces
    // phase changes once rather than every tick (issue #190).
    let mut scripted_label: &'static str = "";
    let mut prev_armed_bit = false;
    let mut prev_reset_bit = false;
    let mut prev_kick_bit = false;
    // `Some(t)` while an on-demand fall kick (issue #161 follow-up, item 5)
    // is in its force-application window, `t` being the sim time it started
    // -- mirrors `STARTUP_KICK_T0_S`'s fixed window but anchored to whenever
    // the rising edge arrived rather than a fixed offset from run start.
    // `None` both before the first trigger and again once a window has
    // finished, so a later rising edge can retrigger it.
    let mut fall_kick_window_start_s: Option<f64> = None;
    let mut prev_outside_corridor = false;
    // The heading this host has injected into the plant so far, radians,
    // unbounded (it is a running total, not an angle in `[-pi, pi]`) -- the
    // integral of the yaw law at the bottom of the loop body. Two readers:
    // the state-out wire's `yaw_rad` field, whose contract is exactly this
    // continuous running total (see `wire::StateOut::yaw_rad`), and the
    // attitude de-rotation below, which needs to know how much world-frame
    // yaw is currently baked into MuJoCo's own quaternion before it can read
    // body pitch and roll back out of it.
    let mut yaw_rad: f32 = 0.0;
    let mut wheel_angle_rad: f32 = 0.0;
    // Previous tick's ground speed, m/s, signed (positive = forward) -- used
    // to gate THIS tick's `weight_shift_fore_aft` against MAX_GROUND_SPEED_M_S
    // before the ballast target is set (which happens before this tick's own
    // fresh speed is known -- see the "Ballast targets" section below). One
    // cycle old by construction, the same lag `last_amps` already has for the
    // command-feedforward estimator.
    let mut last_forward_speed_m_s: f32 = 0.0;
    // Latest TRUE MuJoCo x/y -- now BOTH the wire's `pos` and the corridor
    // check's input (issue #163), as well as `write_stats`'s. Kept in a
    // variable across ticks because the corridor check runs at the top of the
    // loop body, before this tick's own observation, and so reads the
    // PREVIOUS tick's truth -- the same one-cycle lag `last_forward_speed_m_s`
    // has, for the same reason.
    let mut truth_pos_x_m: f64 = 0.0;
    let mut truth_pos_y_m: f64 = 0.0;

    // --- ADR-0011 (b) and (c) state ------------------------------------
    // The fore/aft command scale actually in force this run. Normally the
    // shipped constant; a verification sweep overrides it (see
    // HostConfig::cmd_envelope_reserve, which is how the constant's own
    // provenance measurement is taken).
    let cmd_envelope_reserve = cfg.cmd_envelope_reserve.unwrap_or(CMD_ENVELOPE_RESERVE);
    // `--cmd-reserve` alone still means "scale the WHOLE fore/aft command by
    // this", which is what every sweep that predates the braking reserve
    // assumes -- `PEAK_DEMAND_A_PER_UNIT_STICK`'s provenance sweep among them,
    // and `--cmd-reserve 0` as the way to say "no stick at all". Without this
    // fallback a run asking for zero stick still gets a braking command the
    // moment the board rolls backwards, which is exactly how it was found.
    let cmd_envelope_reserve_braking = cfg
        .cmd_envelope_reserve_braking
        .or(cfg.cmd_envelope_reserve)
        .unwrap_or(CMD_ENVELOPE_RESERVE_BRAKING);
    let pitch_bias_rad = cfg.pitch_bias_deg.to_radians();
    // Low-passed |proposed current| / MAX_CURRENT_A. Starts at zero, which is
    // true: an unarmed board at rest is asking for nothing.
    let mut utilisation_filtered: f32 = 0.0;
    // One-pole coefficient for AUTHORITY_UTILISATION_TAU_S at this cycle
    // rate, precomputed rather than recomputed 500 times a second.
    let utilisation_alpha = DT_S as f32 / (AUTHORITY_UTILISATION_TAU_S + DT_S as f32);
    // Edge state for the loss-of-authority warning, so it announces itself
    // once per episode rather than 500 times a second -- the same
    // rising-edge discipline the corridor warning above uses.
    let mut prev_authority_warning = false;
    let mut trace: Vec<TraceRow> = Vec::new();
    if cfg.trace_path.is_some() {
        // A 30 s scripted run at 500 Hz is 15,000 rows; reserving avoids
        // reallocating inside the control loop.
        trace.reserve(16_384);
    }

    let start = Instant::now();
    let mut pacer = Pacer::new(Duration::from_nanos(CYCLE_NS), start);
    // Headroom over InputIn::WIRE_SIZE so an oversized datagram is caught as
    // a WrongSize mismatch by InputIn::from_bytes rather than silently
    // truncated by a too-small recv buffer.
    let mut recv_buf = [0u8; InputIn::WIRE_SIZE + 16];
    let mut last_stats_write = Instant::now();
    let mut ticks: u64 = 0;
    // Pre-step sim time, mirroring impulse-response-rust's own `t_known_s`:
    // the window check below must use the time as of the START of this
    // tick, since `apply_external_force` only takes effect on the NEXT
    // `wait_observe()` (see [`STARTUP_KICK_T0_S`]).
    let mut t_known_s: f64 = 0.0;

    loop {
        if let Some(d) = cfg.duration {
            if start.elapsed() >= d {
                break;
            }
        }
        // A scripted run also stops on SIMULATED time (issue #190), so its
        // length is a property of the schedule rather than of how fast the
        // laptop happened to be -- the same reason the schedule itself is
        // indexed on sim time. `cfg.duration`, if set, still applies as a
        // wall-clock backstop.
        if let Some(sched) = cfg.scripted_scenario {
            if t_known_s > crate::scenario::total_duration_s(sched) + SCRIPTED_RUN_TAIL_S {
                break;
            }
        }
        if let Some(limit) = cfg.max_sim_time_s {
            if t_known_s > limit {
                break;
            }
        }

        // Drain every pending datagram; only the most recent VALID one
        // matters (issue #161: "Use the most recent packet received").
        loop {
            match in_socket.recv_from(&mut recv_buf) {
                Ok((n, _src)) => match InputIn::from_bytes(&recv_buf[..n]) {
                    Ok(pkt) => {
                        eprintln!(
                            "sim-host: input seq={} weight_fore_aft={:.3} weight_lateral={:.3} \
                             steer={:.3} arm={} reset={}",
                            { pkt.seq },
                            { pkt.weight_shift_fore_aft },
                            { pkt.weight_shift_lateral },
                            { pkt.steer },
                            (pkt.flags & wire::INPUT_FLAG_ARM) != 0,
                            (pkt.flags & wire::INPUT_FLAG_RESET) != 0,
                        );
                        latest_input = LatestInput::from(pkt);
                        latest_input_at = Some(Instant::now());
                    }
                    // Fail loudly, drop the packet, never misparse it as a
                    // float (issue #161).
                    Err(e) => eprintln!("sim-host: dropping malformed input packet: {e}"),
                },
                Err(e) if e.kind() == ErrorKind::WouldBlock => break,
                Err(e) => {
                    eprintln!("sim-host: input socket recv error: {e}");
                    break;
                }
            }
        }

        let stale = latest_input_at
            .map(|t| t.elapsed() > INPUT_STALENESS_TIMEOUT)
            .unwrap_or(true);
        // weight_shift_*/steer hold last, then zero after the staleness
        // timeout (issue #161) -- UNLESS a scripted scenario is driving this
        // run (issue #190), in which case the schedule is read straight off
        // SIMULATED time and the socket's stick values are ignored entirely.
        // The flag bits (arm/reset/kick) still come from the wire either way,
        // so an on-demand kick can still be injected into a scripted run.
        let (steer, stick_fore_aft, weight_shift_lateral) = match cfg.scripted_scenario {
            Some(sched) => {
                let (fa, lat, st, label) = crate::scenario::value_at(sched, t_known_s);
                if label != scripted_label {
                    eprintln!(
                        "sim-host: scripted sim_t={t_known_s:6.2}s -> {label} \
                         (fore_aft={fa:+.2} lateral={lat:+.2} steer={st:+.2})"
                    );
                    scripted_label = label;
                }
                (st, fa, lat)
            }
            None if stale => (0.0, 0.0, 0.0),
            None => (
                latest_input.steer,
                latest_input.weight_shift_fore_aft,
                latest_input.weight_shift_lateral,
            ),
        };

        // --- COMMAND-ENVELOPE RESERVE (ADR-0011 exit criterion (b)) ------
        //
        // The whole fix, in one multiply. Deliberately placed HERE, at the
        // point where a stick value enters the host and before anything else
        // touches it: upstream of the speed cap, the corridor brake, the
        // ballast actuator, the plant, the estimator, the regulator and the
        // safety envelope. That ordering is the reason this is a zero-risk
        // change rather than a retune -- every downstream block sees a
        // smaller stick and is otherwise bit-identical, no gain moves, and
        // nothing in the control law learns that a limit exists.
        //
        // Fore/aft only. The failure ADR-0011 is about is a fore/aft pitch
        // authority failure; lateral stick moves `ballast_lat`, whose
        // measured full-stick effect on this geometry is 0.03 deg of roll
        // (see ROLL_FULL_YAW_AUTHORITY_RAD) and which consumes no wheel
        // torque at all. Scaling it would cost steering feel to buy nothing.
        //
        // Braking spends its own reserve (CMD_ENVELOPE_RESERVE_BRAKING).
        // Gated on `last_forward_speed_m_s` -- one cycle old, the same lag
        // the speed cap below runs on, and for the same reason.
        let weight_shift_fore_aft = shape_fore_aft_command_directional(
            stick_fore_aft,
            cmd_envelope_reserve,
            cmd_envelope_reserve_braking,
            last_forward_speed_m_s,
        );

        // `arm`/`reset` bits: accepted and tracked, logged on change rather
        // than every tick. Neither gates anything yet -- the host self-arms
        // unconditionally (see the comment on `backend.arm()` above), and
        // there is no reset implementation to wire `reset` into. `stale`
        // zeroes both the same way it zeroes weight_shift/steer.
        let input_armed_bit = !stale && latest_input.armed_bit;
        let input_reset_bit = !stale && latest_input.reset_bit;
        if input_reset_bit && !prev_reset_bit {
            eprintln!("sim-host: input reset bit set -- not implemented yet, ignoring");
        }
        if input_armed_bit != prev_armed_bit {
            eprintln!(
                "sim-host: input arm bit is now {input_armed_bit} \
                 (host self-arms regardless of this bit)"
            );
        }
        prev_armed_bit = input_armed_bit;
        prev_reset_bit = input_reset_bit;

        // On-demand fall kick (issue #161 follow-up, item 5): rising-edge
        // triggered, like `reset` above, so holding the bit does not restart
        // it every tick. Does not retrigger while a window is already in
        // progress (`fall_kick_window_start_s` only goes back to `None`
        // once that window's force-application section below has finished
        // it) -- one kick per press.
        let input_kick_bit = !stale && latest_input.kick_bit;
        if input_kick_bit && !prev_kick_bit && fall_kick_window_start_s.is_none() {
            eprintln!("sim-host: input kick bit set -- inducing a fall");
            fall_kick_window_start_s = Some(t_known_s);
        }
        prev_kick_bit = input_kick_bit;

        // Speed cap (issue #161 follow-up, MAX_GROUND_SPEED_M_S's own doc
        // comment) -- attenuates weight_shift_fore_aft, not the wheel speed
        // itself: if the board is already moving in the SAME direction this
        // input would accelerate it (or is at rest), ramp authority down to
        // zero over the last SPEED_CAP_MARGIN_M_S below the cap. Braking or
        // reversing (opposite sign to current motion) is never touched.
        // Gated on last_forward_speed_m_s -- one cycle old, since this
        // tick's fresh speed is not known until wait_observe() below.
        let capped_weight_shift_fore_aft =
            speed_capped_fore_aft(weight_shift_fore_aft, last_forward_speed_m_s);

        // Corridor boundary (issue #161 follow-up, item 4) -- see
        // CORRIDOR_X_MIN_M's doc comment for the bounds and for why this is a
        // soft lean rather than contact. Checked against MuJoCo TRUTH position
        // as of the END of the PREVIOUS tick (issue #163: this used to read
        // the dead-reckoned path, which no longer exists) -- the same
        // one-cycle lag the speed cap above uses, and for the same reason.
        let outside_corridor = !(CORRIDOR_X_MIN_M..=CORRIDOR_X_MAX_M).contains(&truth_pos_x_m)
            || truth_pos_y_m.abs() > CORRIDOR_HALF_WIDTH_M;
        if outside_corridor && !prev_outside_corridor {
            eprintln!(
                "sim-host: LEFT THE DRIVABLE CORRIDOR at ({truth_pos_x_m:.1}, \
                 {truth_pos_y_m:.1}) m -- arresting forward lean"
            );
        } else if prev_outside_corridor && !outside_corridor {
            eprintln!("sim-host: back inside the drivable corridor");
        }
        prev_outside_corridor = outside_corridor;
        // Overrides the speed cap's own result, not just `weight_shift_
        // fore_aft` -- an active brake against whatever direction the
        // board was travelling, not merely "stop accelerating further"
        // (which alone would let it coast on out under residual speed).
        // Lateral/steer are untouched -- only forward travel is arrested.
        let corridor_enforced_fore_aft = if outside_corridor {
            if last_forward_speed_m_s > 0.0 {
                -CORRIDOR_BRAKE_LEAN
            } else if last_forward_speed_m_s < 0.0 {
                CORRIDOR_BRAKE_LEAN
            } else {
                0.0
            }
        } else {
            capped_weight_shift_fore_aft
        };

        // Ballast targets -- weight_shift_fore_aft/lateral drive
        // overboard_rider.xml's two ballast actuators DIRECTLY AND
        // PHYSICALLY (see this file's header). Set every cycle, mirroring
        // apply_external_force's own "call every cycle or a stale value
        // persists" convention.
        backend.set_ballast_targets(
            corridor_enforced_fore_aft * BALLAST_RANGE_M,
            weight_shift_lateral * BALLAST_RANGE_M,
        );

        // One-time startup kick, only when explicitly enabled (issue #169)
        // -- see STARTUP_KICK_T0_S's doc comment. Checked against the
        // PRE-step time, mirroring impulse_response.py's `if t0 <= data.time
        // < t0 + duration` (also checked there before that iteration's
        // mj_step).
        let in_kick_window = cfg.startup_kick
            && (STARTUP_KICK_T0_S..STARTUP_KICK_T0_S + STARTUP_KICK_DURATION_S)
                .contains(&t_known_s);
        // On-demand fall kick (issue #161 follow-up, item 5) -- same
        // pre-step-time window check as the startup kick above, just
        // anchored to `fall_kick_window_start_s` instead of a fixed
        // `STARTUP_KICK_T0_S`. Cleared back to `None` once the window has
        // elapsed so a later rising edge can retrigger it (see where it is
        // set, above).
        let in_fall_kick_window = fall_kick_window_start_s
            .is_some_and(|t0| (t0..t0 + FALL_KICK_DURATION_S).contains(&t_known_s));
        if let Some(t0) = fall_kick_window_start_s {
            if t_known_s >= t0 + FALL_KICK_DURATION_S {
                fall_kick_window_start_s = None;
            }
        }
        // A scheduled acceptance-run disturbance (ADR-0011 criterion (a),
        // "full stick during a kerb strike"), on the same pre-step-time
        // window rule as the two kicks above. Unlike them it carries a
        // TORQUE as well as a force: a kerb acts at the wheel, well below the
        // CoM, and the angular channel is the one that topples the board --
        // see `Disturbance` and issue #142's finding that quoting a kerb in
        // N*s alone understates it silently.
        let in_disturbance_window = cfg
            .disturbance
            .is_some_and(|d| (d.t0_s..d.t0_s + d.duration_s).contains(&t_known_s));
        let (force, torque) = if in_disturbance_window {
            let d = cfg.disturbance.expect("checked by is_some_and above");
            (d.force_n, d.torque_nm)
        } else if in_kick_window {
            (STARTUP_KICK_FORCE_N, [0.0; 3])
        } else if in_fall_kick_window {
            (FALL_KICK_FORCE_N, [0.0; 3])
        } else {
            ([0.0; 3], [0.0; 3])
        };
        backend.apply_external_force(force, torque);

        let obs = backend.wait_observe().map_err(HostError::Backend)?;
        t_known_s = obs.t_recv_ns as f64 * 1e-9;

        // Controller: raw IMU -> estimate -> regulate -> envelope. Mode 2
        // (command feedforward), matching `shuttle_run.py`'s tuned ridden
        // config -- "the recommended configuration" per that scenario's own
        // comment. `pitch_ref` is always 0: no outer loop (see this file's
        // header).
        let sample = obs.newest_imu().copied().unwrap_or(ImuSample::ZERO);
        let wheel_rate_rad_s = obs.erpm * RAD_S_PER_ERPM;
        // Real forward ground speed, m/s, signed -- computed here (rather
        // than only down in the dead-reckoning block that used to be its
        // only reader) because the speed-proportional yaw rate below now
        // needs it too. Kept for the NEXT tick's speed-cap gate (see
        // `last_forward_speed_m_s` above) at the end of this loop body.
        let forward_speed_m_s = wheel_rate_rad_s * DEFAULT_R_EFF_M;
        last_forward_speed_m_s = forward_speed_m_s;
        let aiding = accel_ff.predict(last_amps);
        // The estimator runs on EVERY run, including a `PitchSource::
        // PlantTruth` acceptance run where the regulator is not listening to
        // it -- it is the only signal path a real board has, and an
        // acceptance trace that stopped recording it would stop being able to
        // show how big the bias criterion (f) is neutralising actually is.
        let attitude = estimator.update(std::slice::from_ref(&sample), aiding);

        // Ground truth, never fed to the controller on a DEPLOYED run
        // (DR-OBS-1) -- reported because "the board is actually up" is what
        // the state-out wire needs to prove, and truth proves it more
        // directly than the controller's own (estimator-mediated) belief.
        // Same formula `impulse-response-rust` / `sim/scenarios/
        // impulse_response.py::frame_pitch_rad` use, against the same
        // underlying xmat.
        //
        // **Read here rather than after the wire block, where it used to
        // live** (ADR-0011 criterion (f)): a `PitchSource::PlantTruth`
        // acceptance run needs it BEFORE the regulator, not after. Nothing
        // between the old and new positions touched the plant or `yaw_rad`
        // -- `backend.apply` only buffers a current for the next step -- so
        // the values are bit-identical to the ones the old ordering read.
        let xmat = backend.truth_frame_xmat();
        // ATTITUDE MUST BE DE-YAWED BEFORE PITCH/ROLL COME OUT OF IT (issue
        // #163). Both readings below are `atan2` on the frame's world z-axis,
        // and that derivation assumes the world x/y axes still line up with
        // the body's -- true when the board had no yaw freedom at all, which
        // is exactly what the kinematic injection changed. `xmat` is now
        // `Rz(yaw) * R_body`; its third column mixes body pitch and body roll
        // by the heading angle, so after a 90 deg turn the raw formula would
        // report the board's ROLL as its PITCH, and `FALLEN` would fire on a
        // board that is upright. Removing the heading first (`Rz(-yaw) *
        // xmat`, applied to the two elements the formulas read) recovers the
        // body-frame values the ICD and the roll gate both mean.
        //
        // `yaw_rad` here is the heading currently baked into the plant -- this
        // tick's own increment is computed and injected at the BOTTOM of the
        // loop body, after this. At `yaw_rad == 0` (zero steer, all run) this
        // reduces to `1.0 * xmat[k] + 0.0 * xmat[j]`, i.e. bit-identically the
        // pre-#163 expressions.
        let (pitch_rad, roll_rad) = body_pitch_roll_rad(&xmat, yaw_rad);
        let truth_pitch_rate_rad_s = backend.truth_body_pitch_rate_rad_s();

        // --- ADR-0011 criterion (f): the estimator bias, removed ---------
        //
        // On a deployed run this is the `Estimator` arm and the regulator
        // sees exactly what it always saw. On an acceptance run it is
        // `PlantTruth`, and the regulator sees MuJoCo -- which on this plant
        // is the measured-WORSE case, because it deletes the apparent-vertical
        // lean the estimate carries rather than de-biasing it. See
        // `PitchSource`.
        let (source_pitch_rad, regulated_pitch_rate_rad_s) = match cfg.pitch_source {
            PitchSource::Estimator => (attitude.pitch_rad, attitude.pitch_rate_rad_s),
            PitchSource::PlantTruth => (pitch_rad, truth_pitch_rate_rad_s),
        };
        // Zero on any deployed run, so this whole line is a no-op there --
        // see HostConfig::pitch_bias_deg.
        let regulated_pitch_rad = source_pitch_rad + pitch_bias_rad;
        let proposed_torque_nm =
            regulator.update(regulated_pitch_rad, regulated_pitch_rate_rad_s, 0.0);
        // The single kt division -- the actuation boundary (issue #137).
        let proposed_amps = proposed_torque_nm / KT_NM_PER_A;
        // --- ADR-0011 criterion (c): the saturation bit STOPS being thrown
        // --- away here ---------------------------------------------------
        //
        // This binding used to read `let (bounded_cmd, _sat) = ...`. The
        // ADR names that discard by line number: the board's own "I have run
        // out of authority" signal existed, was computed every cycle, and was
        // dropped on the floor, while `FALLEN` -- which trips about a second
        // AFTER the outcome is decided -- was the only thing anyone was told.
        let (bounded_cmd, saturation) = envelope.apply(
            Command::MotorCurrent {
                amps: proposed_amps,
            },
            Faults::NONE,
        );
        let saturated = saturation == Saturation::Yes;
        backend.apply(&bounded_cmd).map_err(HostError::Backend)?;
        // POST-envelope current, not the proposal -- the plant only ever
        // sees the clamped value (same reasoning `control-ffi`'s own
        // `ctl.last_amps` update carries).
        last_amps = match bounded_cmd {
            Command::MotorCurrent { amps } => amps,
            Command::RemoteSpeed { .. } => 0.0,
        };

        // Authority utilisation: how much of the actuator envelope the
        // controller is ASKING for, which is the pre-envelope demand and not
        // the post-envelope command -- the clamped value can never exceed
        // 1.0 and so can never warn about anything. Low-passed at
        // AUTHORITY_UTILISATION_TAU_S.
        let utilisation = proposed_amps.abs() / MAX_CURRENT_A;
        utilisation_filtered += (utilisation - utilisation_filtered) * utilisation_alpha;
        // The discriminator is the SPEED (see AUTHORITY_UTILISATION_WARN):
        // saturation above the speed cap's onset is survivable, because the
        // cap is already unloading the board when it happens. Below it,
        // nothing is.
        let authority_warning = authority_warning_active(utilisation_filtered, forward_speed_m_s);
        if authority_warning && !prev_authority_warning {
            eprintln!(
                "sim-host: LOSS OF PITCH AUTHORITY IMMINENT at sim_t={t_known_s:.3}s -- \
                 filtered authority utilisation {:.0}% (demand {proposed_amps:+.1} A of \
                 {MAX_CURRENT_A:.0} A) at {forward_speed_m_s:+.2} m/s, below the \
                 {SPEED_CAP_ONSET_M_S:.2} m/s speed-cap onset. Pitch {:+.1} deg.",
                utilisation_filtered * 100.0,
                pitch_rad.to_degrees(),
            );
        } else if prev_authority_warning && !authority_warning {
            eprintln!(
                "sim-host: pitch authority recovered at sim_t={t_known_s:.3}s \
                 (filtered utilisation {:.0}%)",
                utilisation_filtered * 100.0,
            );
        }
        prev_authority_warning = authority_warning;

        // wheel_angle_rad: there is no absolute wheel-angle channel on `hal`
        // (ICD carries ERPM/tacho, the same as real VESC telemetry) -- this
        // dead-reckons it from the rate `hal` already reports, exactly the
        // way a real host would have to. Not a fabricated value: it is an
        // honest integral of an actually-measured rate.
        wheel_angle_rad += wheel_rate_rad_s * DT_S as f32;

        let pos_f64 = backend.truth_frame_xpos();
        truth_pos_x_m = pos_f64[0];
        truth_pos_y_m = pos_f64[1];
        // MuJoCo's own attitude, sent to the wire UNMODIFIED (issue #163).
        // The host used to compose a synthetic heading onto this quaternion
        // on its way out, because the plant had no yaw of its own; the
        // heading now lives inside the plant, so there is nothing left to
        // bolt on and the wire carries plain MuJoCo truth.
        let quat_f64 = backend.truth_frame_xquat();
        let quat = [
            quat_f64[0] as f32,
            quat_f64[1] as f32,
            quat_f64[2] as f32,
            quat_f64[3] as f32,
        ];

        // MuJoCo truth, straight through (issue #163). Nothing is
        // dead-reckoned any more: the board's heading is injected into the
        // plant before each step (see the bottom of this loop body), so
        // MuJoCo integrates the ground path itself, against its own contacts
        // and collision geometry, and this IS where the board is.
        let pos = [
            truth_pos_x_m as f32,
            truth_pos_y_m as f32,
            pos_f64[2] as f32,
        ];

        // Wire v2 (issue #161 follow-up): the ACTUAL ballast joint
        // positions -- CEO feedback was "there is no rider, and the turn
        // is not discernible", and a renderer cannot pose a rider from data
        // it does not have. NOT the commanded target (`weight_shift_*` *
        // BALLAST_RANGE_M`) -- the actuator is rate-limited
        // (`overboard_rider.xml`'s `timeconst`), so it lags a step change,
        // and sending the real joint value means that lag is visible
        // honestly rather than hidden behind an instantaneous command. Small
        // by construction (measured ~0.04 m at 0.8 stick) -- this host does
        // NOT amplify it for legibility; that is the renderer's job, as its
        // own declared non-physical channel, not this crate's to fake.
        let (rider_fore_aft_m, rider_lateral_m) = backend.truth_ballast_positions();

        let mut flags = wire::STATE_FLAG_ARMED | wire::STATE_FLAG_VALID;
        let fallen = pitch_rad.abs() > FALLEN_PITCH_RAD;
        if fallen {
            flags |= wire::STATE_FLAG_FALLEN;
        }

        if cfg.trace_path.is_some() {
            trace.push(TraceRow {
                seq: ticks,
                sim_time_s: t_known_s,
                stick_fore_aft,
                shaped_fore_aft: weight_shift_fore_aft,
                applied_fore_aft: corridor_enforced_fore_aft,
                truth_pitch_deg: pitch_rad.to_degrees(),
                est_pitch_deg: attitude.pitch_rad.to_degrees(),
                est_pitch_rate_deg_s: attitude.pitch_rate_rad_s.to_degrees(),
                truth_pitch_rate_deg_s: truth_pitch_rate_rad_s.to_degrees(),
                forward_speed_m_s,
                proposed_amps,
                applied_amps: last_amps,
                saturated,
                utilisation,
                utilisation_filtered,
                authority_warning,
                fallen,
            });
        }

        let state = StateOut {
            magic: wire::STATE_MAGIC,
            schema_version: wire::STATE_SCHEMA_VERSION,
            flags,
            seq: ticks,
            sim_time_s: obs.t_recv_ns as f64 * 1e-9,
            pos,
            quat,
            wheel_angle_rad,
            wheel_rate_rad_s,
            pitch_rad,
            yaw_rad,
            motor_current_a: obs.motor_current_a,
            rider_fore_aft_m,
            rider_lateral_m,
        };
        out_socket.send_to(&state.to_bytes(), cfg.state_out_addr)?;

        // --- KINEMATIC IN-PLANT YAW INJECTION (issue #163) --------------
        //
        // The steering law below is UNCHANGED -- same constants, same roll
        // shaping, same sign, same speed-proportional curvature. What changed
        // is where its output goes. It used to integrate a `yaw_rad` that the
        // physics never saw, and the host then dead-reckoned the ground path
        // and composed the heading onto the outgoing quaternion, because
        // MuJoCo's own board only ever translated along one axis. Both of
        // those are gone. The increment is now written straight into the
        // plant's free joint, immediately below, so the very next physics step
        // integrates the board's translation along the new heading itself.
        //
        // **The reframe that motivates it: the collision goal never needed yaw
        // to be physically GENERATED, only MuJoCo's pose to be
        // AUTHORITATIVE.** Those are different problems and the second is far
        // smaller. Steering is still commanded rather than emergent -- no tire
        // model produces this turn, and this file is still the only place the
        // heading comes from -- but position, ground path and every contact
        // downstream of it are now genuinely simulated at that heading, and
        // collision geometry in the MJCF would finally be tested against the
        // position the renderer actually draws. Nothing is dead-reckoned any
        // more. The MJCF is untouched, which is the entire point of doing it
        // this way.
        //
        // **REJECTED: making the wheel carve by geometry** (replacing the
        // cylinder with a sphere, ellipsoid or torus so a lean migrates the
        // contact patch and friction generates the turn). Recorded here
        // because it is the obvious idea and it is a trap on this plant:
        // the board is currently roll-stable ONLY because the wide cylinder
        // rim physically cannot tip (measured full-stick roll is 0.03 deg --
        // see ROLL_FULL_YAW_AUTHORITY_RAD). Any laterally-migrating contact
        // profile converts the board into a roll-axis inverted pendulum about
        // the contact point, and there is NO roll/lean controller in this
        // repo yet, so it would simply fall over sideways. That option is
        // gated on lean-steer; they are one epic, not two. Also, a torus mesh
        // fails silently rather than loudly: MuJoCo convex-hulls meshes, and
        // the convex hull of a torus is a rounded-rim disc.
        //
        // Placed here, at the END of the loop body, rather than at the top:
        // this way everything already sent on the wire above describes the
        // state as OBSERVED, and the injection is unambiguously the last
        // thing that happens before the next `wait_observe()` steps the
        // physics. `roll_rad` and `forward_speed_m_s` are this tick's own
        // fresh values, exactly as the pre-#163 law used.
        //
        // `roll_authority` scales the law's magnitude between
        // YAW_AUTHORITY_FLOOR (steer alone, however little the player is
        // leaning) and 1.0 (at/above ROLL_FULL_YAW_AUTHORITY_RAD's measured,
        // physically-achievable roll). Read BOTH constants' doc comments
        // before touching it -- on the current (widened) wheel geometry,
        // achievable roll is ~0.03 deg, so `steer` is effectively the primary
        // driver of yaw, NOT roll; the floor is what makes that honest
        // instead of a limiter that reads as "off".
        let roll_authority = YAW_AUTHORITY_FLOOR
            + (1.0 - YAW_AUTHORITY_FLOOR)
                * (roll_rad.abs() / ROLL_FULL_YAW_AUTHORITY_RAD).clamp(0.0, 1.0);
        // SIGN (issue #161 follow-up, CEO-reported bug: "turns me left when
        // I turn right"): increasing yaw_rad is a positive rotation about +Z,
        // which in this Z-up right-handed frame is counter-clockwise viewed
        // from above -- LEFT, by this model's own "right = +y" convention
        // (see the `imu` site comment in `overboard_rider.xml`). So positive
        // `steer` (stick-right) must DECREASE yaw_rad -- `-`, not `+`. See
        // `wire.rs`'s `InputIn::steer` doc comment for the wire-level
        // convention this implies.
        //
        // SPEED-PROPORTIONAL (issue #161 follow-up, CEO's own diagnosis: "you
        // can just straight up turn yourself around... too fast").
        // `YAW_CURVATURE_PER_STEER_RAD_PER_M`'s own doc comment has the full
        // reasoning; the short version is that yaw RATE scales with
        // `forward_speed_m_s`, so turn radius stops depending on speed (as a
        // real vehicle's does). At a standstill this is exactly zero, and the
        // gate below then makes the whole injection a literal no-op.
        let yaw_rate_rad_s = yaw_rate_rad_s(steer, forward_speed_m_s, roll_authority);
        let dyaw_rad = -yaw_rate_rad_s * DT_S as f32;
        // GATED ON EXACT ZERO, deliberately. With no steer (or no ground
        // speed) the plant must evolve bit-identically to the pre-#163 code:
        // this loop injects nothing, writes nothing, and the only remaining
        // difference on the wire is that `pos` reports MuJoCo's own x/y
        // instead of a reckoned one. `SimBackend::inject_kinematic_yaw`
        // re-checks the same condition; the gate is repeated here so that
        // `yaw_rad` and the plant can never disagree about whether a tick's
        // increment was applied.
        if dyaw_rad != 0.0 {
            yaw_rad += dyaw_rad;
            backend.inject_kinematic_yaw(dyaw_rad as f64);
        }

        ticks += 1;

        if let Some(path) = &cfg.stats_path {
            if last_stats_write.elapsed() >= Duration::from_millis(100) {
                write_stats(
                    path,
                    ticks,
                    pacer.missed_deadlines(),
                    truth_pos_x_m,
                    truth_pos_y_m,
                );
                last_stats_write = Instant::now();
            }
        }

        // A free run has no deadlines to miss: the pacer is skipped entirely
        // rather than consulted and ignored, so its missed-deadline counter
        // stays at the truthful zero instead of reporting one miss per tick
        // for a mode in which "late" is not defined. See
        // `HostConfig::free_run`.
        if !cfg.free_run {
            let sleep_for = pacer.wait_for_next(Instant::now());
            if sleep_for.is_zero() {
                eprintln!(
                    "sim-host: missed deadline at tick {ticks} (total missed so far: {})",
                    pacer.missed_deadlines()
                );
            } else {
                std::thread::sleep(sleep_for);
            }
        }
    }

    if let Some(path) = &cfg.stats_path {
        write_stats(
            path,
            ticks,
            pacer.missed_deadlines(),
            truth_pos_x_m,
            truth_pos_y_m,
        );
    }

    if let Some(path) = &cfg.trace_path {
        write_trace(path, &trace)?;
        eprintln!(
            "sim-host: wrote {} trace rows to {}",
            trace.len(),
            path.display()
        );
    }

    let _ = backend.close();

    Ok(RunSummary {
        ticks,
        missed_deadlines: pacer.missed_deadlines(),
    })
}

/// Writes the acceptance-run trace ([`HostConfig::trace_path`]) as CSV, once,
/// after the loop has stopped.
///
/// Unlike [`write_stats`] this one is NOT best-effort: a trace is the
/// evidence an ADR-0011 acceptance number is quoted from, and a measurement
/// whose output silently failed to be written is worse than no measurement.
/// The error propagates.
///
/// Full `f32` precision (`{:?}`) on every physical column on purpose:
/// `wire-probe --csv`'s 6-decimal rounding is enough to draw a graph and not
/// enough to demonstrate that two runs are bit-identical, and this crate has
/// already been caught out once by exactly that difference (issue #163's own
/// equivalence work).
fn write_trace(path: &std::path::Path, rows: &[TraceRow]) -> Result<(), HostError> {
    use std::fmt::Write as _;
    let mut out = String::with_capacity(rows.len() * 160 + 256);
    out.push_str(
        "seq,sim_time_s,stick_fore_aft,shaped_fore_aft,applied_fore_aft,truth_pitch_deg,\
         est_pitch_deg,est_pitch_rate_deg_s,truth_pitch_rate_deg_s,forward_speed_m_s,proposed_amps,applied_amps,\
         saturated,utilisation,utilisation_filtered,authority_warning,fallen\n",
    );
    for r in rows {
        let _ = writeln!(
            out,
            "{},{:?},{:?},{:?},{:?},{:?},{:?},{:?},{:?},{:?},{:?},{:?},{},{:?},{:?},{},{}",
            r.seq,
            r.sim_time_s,
            r.stick_fore_aft,
            r.shaped_fore_aft,
            r.applied_fore_aft,
            r.truth_pitch_deg,
            r.est_pitch_deg,
            r.est_pitch_rate_deg_s,
            r.truth_pitch_rate_deg_s,
            r.forward_speed_m_s,
            r.proposed_amps,
            r.applied_amps,
            r.saturated as u8,
            r.utilisation,
            r.utilisation_filtered,
            r.authority_warning as u8,
            r.fallen as u8,
        );
    }
    std::fs::write(path, out).map_err(HostError::Io)
}

/// Best-effort, atomic (write-then-rename) write of the host's own counters
/// AND true MuJoCo ground position, for `wire-probe` (or anyone else) to
/// pick up. Never allowed to interrupt the control loop -- a failure here is
/// silently swallowed ON PURPOSE (unlike a missed deadline or a malformed
/// input packet, which issue #161 requires surfacing loudly): this file is
/// internal tooling, not the wire. `truth_pos_x_m`/`truth_pos_y_m` are kept
/// here for continuity with the tooling that already reads them -- as of
/// issue #163 they are the SAME values the wire's `pos` now carries, since
/// the dead-reckoned path is gone and MuJoCo's own position is authoritative.
fn write_stats(
    path: &std::path::Path,
    ticks: u64,
    missed_deadlines: u64,
    truth_pos_x_m: f64,
    truth_pos_y_m: f64,
) {
    let tmp = path.with_extension("tmp");
    let contents = format!(
        "ticks={ticks}\nmissed_deadlines={missed_deadlines}\ntruth_pos_x_m={truth_pos_x_m}\ntruth_pos_y_m={truth_pos_y_m}\n"
    );
    let _ = std::fs::write(&tmp, contents).and_then(|_| std::fs::rename(&tmp, path));
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build `Rz(yaw) * Ry(pitch) * Rx(-roll)`, row-major, the same layout
    /// MuJoCo's `xmat` uses. This is the composition [`body_pitch_roll_rad`]
    /// has to invert.
    ///
    /// The `-roll` is not a typo. This crate's roll is measured about the
    /// frame's FORWARD axis, which is its local **-X** (see
    /// `overboard_onewheel.xml`: "FORWARD IS -X"), so a positive roll here is
    /// a negative rotation about +X. Building the reference matrix in the same
    /// convention the function under test reports keeps the sign flip in one
    /// place instead of scattering `-` through every assertion.
    fn xmat_from(yaw: f64, pitch: f64, roll: f64) -> [f64; 9] {
        let (sy, cy) = yaw.sin_cos();
        let (sp, cp) = pitch.sin_cos();
        let (sr, cr) = (-roll).sin_cos();
        [
            cy * cp,
            cy * sp * sr - sy * cr,
            cy * sp * cr + sy * sr,
            sy * cp,
            sy * sp * sr + cy * cr,
            sy * sp * cr - cy * sr,
            -sp,
            cp * sr,
            cp * cr,
        ]
    }

    /// Guard 1's unit-test half: with no heading injected, the de-yaw must be
    /// a LITERAL no-op -- not "close to", but the identical bits the pre-#163
    /// code produced. Zero steer has to leave every wire field it can reach
    /// untouched, and `1.0 * x + 0.0 * y` is only bit-safe as long as nobody
    /// "simplifies" the expression later.
    #[test]
    fn deyaw_at_zero_yaw_is_a_no_op() {
        for &(p, r) in &[(0.0, 0.0), (0.12, -0.03), (-0.31, 0.007), (0.0, 0.4)] {
            let xmat = xmat_from(0.0, p, r);
            let (pitch, roll) = body_pitch_roll_rad(&xmat, 0.0);
            // Bit-for-bit against the exact expressions this replaced.
            assert_eq!(pitch, (xmat[2] as f32).atan2(xmat[8] as f32));
            assert_eq!(roll, (xmat[5] as f32).atan2(xmat[8] as f32));
        }
    }

    /// THE regression the in-plant injection introduces if de-yawing is
    /// forgotten: a turned board reports its ROLL as its PITCH, so `FALLEN`
    /// fires on a board that is perfectly upright.
    ///
    /// Concretely: 12 deg of body roll, no body pitch, yawed 90 deg. The raw
    /// pre-#163 formula reads the frame's world z-axis in world x, which after
    /// a quarter turn IS the roll -- it would report ~12 deg of pitch. The
    /// de-yawed reading must report ~0.
    #[test]
    fn a_turned_board_does_not_report_its_roll_as_pitch() {
        let yaw = std::f64::consts::FRAC_PI_2;
        let roll = 12.0f64.to_radians();
        let xmat = xmat_from(yaw, 0.0, roll);

        let naive_pitch = (xmat[2] as f32).atan2(xmat[8] as f32);
        assert!(
            naive_pitch.abs() > 0.15,
            "this test proves nothing unless the naive formula is badly wrong here \
             (got {naive_pitch} rad)"
        );

        let (pitch, r) = body_pitch_roll_rad(&xmat, yaw as f32);
        assert!(
            pitch.abs() < 1e-5,
            "de-yawed pitch should be ~0 on an unpitched board, got {pitch} rad"
        );
        assert!(
            (r - roll as f32).abs() < 1e-5,
            "de-yawed roll should be the body roll ({roll} rad), got {r}"
        );
    }

    /// The de-yaw must recover both angles across a range of headings, not
    /// just the one the previous test happens to pick.
    #[test]
    fn deyaw_recovers_body_pitch_and_roll_at_any_heading() {
        let pitch = 0.09f64;
        let roll = 0.02f64;
        for &yaw in &[0.0f64, 0.4, -1.1, 2.6, -3.0, 5.9] {
            let xmat = xmat_from(yaw, pitch, roll);
            let (p, r) = body_pitch_roll_rad(&xmat, yaw as f32);
            assert!(
                (p - pitch as f32).abs() < 1e-4 && (r - roll as f32).abs() < 1e-4,
                "yaw={yaw}: recovered ({p}, {r}), wanted ({pitch}, {roll})"
            );
        }
    }

    // --- ADR-0011 criterion (b): the command-envelope reserve -----------

    /// The constant is DERIVED. This test is the derivation, executable:
    /// stated reserve fraction x envelope / measured slope. If somebody
    /// re-measures the slope and forgets to move the constant -- or moves the
    /// constant to make a scenario pass, which is the failure ADR-0011
    /// explicitly forbids -- this goes red.
    ///
    /// The shipped constant is the derived value rounded to two figures, and
    /// the tolerance here is half a rounding step. Two figures is not
    /// laziness: the measured slope's own per-run spread is 41.6-43.5 A/unit
    /// (see [`PEAK_DEMAND_A_PER_UNIT_STICK`]), so a constant quoted to three
    /// figures would be claiming a precision the measurement does not have.
    #[test]
    fn the_command_envelope_reserve_is_the_stated_reserve_divided_by_the_measured_slope() {
        let raw = STATED_ENVELOPE_RESERVE_FRACTION * MAX_CURRENT_A / PEAK_DEMAND_A_PER_UNIT_STICK;
        // Clamped to 1.00: a shaping fraction above 1.0 would command MORE
        // than full stick, which is not a smaller reserve, it is
        // amplification. At the 60 A / 37.704 N*m envelope the raw formula
        // gives 1.012 -- see CMD_ENVELOPE_RESERVE's own doc comment for why
        // that is itself the finding, not a bug in this clamp.
        let derived = if raw > 1.0 { 1.0 } else { raw };
        assert!(
            (CMD_ENVELOPE_RESERVE - derived).abs() <= 0.005,
            "the shipped constant ({CMD_ENVELOPE_RESERVE}) is not the derived value \
             ({derived}, raw {raw} before the 1.00 clamp) rounded to two figures. It is \
             DERIVED -- {STATED_ENVELOPE_RESERVE_FRACTION} x {MAX_CURRENT_A} A / \
             {PEAK_DEMAND_A_PER_UNIT_STICK} A-per-unit, clamped -- and ADR-0011 forbids \
             moving it to make a scenario pass. Re-measure the slope and the constant \
             follows; the reverse is not allowed."
        );
    }

    /// **INVERTED as of the 60 A / 42 N*m envelope (issue:
    /// realistic-motor-torque), STILL INVERTED at 60 A / 37.704 N*m under the
    /// corrected kt (issue: real-motor-constants) -- read before assuming
    /// this still says what its name implies.** The defect this constant
    /// used to document, stated as arithmetic: at full stick the UNSHAPED
    /// command map asked for more current than the 40 A actuator had. At
    /// 60 A it still does not, even with the real, lower kt costing headroom
    /// (49.79 A/unit measured demand vs a 60 A envelope, not the 44.68 A/unit
    /// figure taken at the old kt=0.7) -- `unshaped` below is LESS than
    /// `MAX_CURRENT_A`, not more. This test asserts the CURRENT state (no
    /// over-command) rather than being deleted or quietly inverted, per this
    /// codebase's convention of pinning a changed finding rather than erasing
    /// the evidence it changed. See `CMD_ENVELOPE_RESERVE`'s doc comment for
    /// what this means for ADR-0011.
    #[test]
    fn full_stick_no_longer_over_commands_the_envelope_at_the_60a_ceiling() {
        let unshaped = shape_fore_aft_command(1.0, 1.0) * PEAK_DEMAND_A_PER_UNIT_STICK;
        assert!(
            unshaped < MAX_CURRENT_A,
            "ADR-0011's founding premise (full stick over-commands the actuator) was \
             re-derived to be false at this envelope, but the measured demand \
             ({unshaped} A) no longer supports even that -- re-check the 60 A / 37.704 \
             N*m re-derivation if this fires"
        );

        // With CMD_ENVELOPE_RESERVE saturated at 1.00 (no shaping), the
        // "shaped" command IS the unshaped one -- there is nothing left to
        // check separately here, which is itself the point: the reserve
        // mechanism is a no-op at this envelope.
        let shaped =
            shape_fore_aft_command(1.0, CMD_ENVELOPE_RESERVE) * PEAK_DEMAND_A_PER_UNIT_STICK;
        assert_eq!(
            shaped, unshaped,
            "CMD_ENVELOPE_RESERVE is documented as saturated at 1.00 (no-op); a shaped \
             value different from unshaped means it moved off that saturation"
        );
    }

    /// The reserve is a linear scaling and NOT a clamp. A clamp would leave
    /// small stick inputs untouched and bite only near the rails, which is a
    /// different control feel and a different failure mode -- and it is the
    /// shape somebody reaches for when "cap the lean" is read casually.
    #[test]
    fn the_reserve_scales_the_whole_stick_range_and_is_symmetric() {
        for &u in &[0.0f32, 0.1, 0.25, 0.5, 0.75, 1.0] {
            let shaped = shape_fore_aft_command(u, CMD_ENVELOPE_RESERVE);
            assert_eq!(
                shaped,
                u * CMD_ENVELOPE_RESERVE,
                "u={u} is not scaled linearly"
            );
            assert_eq!(
                shape_fore_aft_command(-u, CMD_ENVELOPE_RESERVE),
                -shaped,
                "u={u}: at a given reserve the shaping must not depend on the SIGN of the \
                 stick; a one-sided cap is an asymmetric-authority bug that only shows up \
                 under a hard recovery in one direction"
            );
        }
    }

    /// The braking reserve is not a one-sided cap either -- the property the
    /// test above protects still holds, one level up.
    ///
    /// `shape_fore_aft_command_directional` is asymmetric in
    /// (stick, motion) TOGETHER, not in the sign of the stick: braking is a
    /// relationship between the command and the motion. So flipping both
    /// signs must give exactly the mirrored command, at every speed. A board
    /// that stopped harder going forwards than backwards would be the
    /// asymmetric-authority bug in a new place.
    #[test]
    fn the_braking_reserve_is_symmetric_under_flipping_stick_and_motion_together() {
        for &v in &[0.0f32, 0.1, 0.25, 0.3, 1.0, 5.0, 9.34, 12.0] {
            for &u in &[0.0f32, 0.1, 0.5, 1.0] {
                let forward = shape_fore_aft_command_directional(
                    u,
                    CMD_ENVELOPE_RESERVE,
                    CMD_ENVELOPE_RESERVE_BRAKING,
                    v,
                );
                let mirrored = shape_fore_aft_command_directional(
                    -u,
                    CMD_ENVELOPE_RESERVE,
                    CMD_ENVELOPE_RESERVE_BRAKING,
                    -v,
                );
                assert_eq!(
                    forward, -mirrored,
                    "u={u}, v={v}: braking is not mirror-symmetric"
                );
            }
        }
    }

    /// Below the speed gate NOTHING draws the braking reserve, whatever the
    /// signs say.
    ///
    /// This is the property that keeps ADR-0011's `full-stick` acceptance run
    /// -- the run the estimator trim and `PEAK_DEMAND_A_PER_UNIT_STICK` are
    /// both pinned against -- bit-identical under any braking reserve. The
    /// launch transient really does move the board backward under forward
    /// stick (measured: to -0.085 m/s for six tenths of a second), so without
    /// the gate that run would silently become a braking run.
    #[test]
    fn the_braking_reserve_is_not_spent_on_a_standing_start() {
        // Every speed here is inside the gate, and -0.085 is the measured
        // launch-transient extreme the gate exists to cover.
        for &v in &[0.0f32, -0.085, 0.085, -0.24, 0.24] {
            for &u in &[-1.0f32, -0.5, 0.5, 1.0] {
                assert_eq!(
                    shape_fore_aft_command_directional(
                        u,
                        CMD_ENVELOPE_RESERVE,
                        CMD_ENVELOPE_RESERVE_BRAKING,
                        v,
                    ),
                    shape_fore_aft_command(u, CMD_ENVELOPE_RESERVE),
                    "u={u} at {v} m/s drew the braking reserve below the gate"
                );
            }
        }
    }

    /// Opposing the motion above the gate DOES draw it, or the constant is
    /// inert. The companion to the test above: together they pin both sides
    /// of the gate rather than only the safe one.
    #[test]
    fn opposing_the_motion_above_the_gate_draws_the_braking_reserve() {
        for &v in &[0.3f32, 1.0, 5.0, 9.34] {
            assert_eq!(
                shape_fore_aft_command_directional(
                    -1.0,
                    CMD_ENVELOPE_RESERVE,
                    CMD_ENVELOPE_RESERVE_BRAKING,
                    v,
                ),
                -CMD_ENVELOPE_RESERVE_BRAKING,
                "full aft at {v} m/s forward did not draw the braking reserve"
            );
        }
    }

    /// Full stick still reaches the speed cap, so the reserve costs no top
    /// speed -- which is the entire reason ADR-0011 rejected lowering
    /// `MAX_GROUND_SPEED_M_S` as dominated. Stated here as the property that
    /// makes it true: a shaped full stick is still far enough above zero that
    /// the cap, not the command map, is what stops the board.
    #[test]
    fn a_shaped_full_stick_is_still_governed_by_the_speed_cap_not_by_the_command_map() {
        let shaped = shape_fore_aft_command(1.0, CMD_ENVELOPE_RESERVE);
        // Well below the cap, the cap does not touch it: the command map is
        // the only thing acting.
        assert_eq!(speed_capped_fore_aft(shaped, 0.0), shaped);
        // At the cap, the cap takes all of it, whatever the command map left.
        assert_eq!(speed_capped_fore_aft(shaped, MAX_GROUND_SPEED_M_S), 0.0);
    }

    // --- ADR-0011 criterion (a): why the reversal is the worst case ------

    /// The reversal entry of the matrix, as a property rather than a run:
    /// the speed cap NEVER attenuates a reversal, at any speed. This is what
    /// makes "full reverse-to-forward stick reversal at speed" the worst
    /// case -- the board is above the onset, where every surviving run in
    /// ADR-0011's data was being unloaded by the cap, and the cap is not
    /// acting.
    #[test]
    fn the_speed_cap_never_attenuates_a_stick_reversal() {
        for &v in &[1.0f32, 5.0, 8.34, 9.34, 12.0] {
            // Travelling forward, stick slammed back: full authority.
            assert_eq!(
                speed_capped_fore_aft(-1.0, v),
                -1.0,
                "braking from +{v} m/s was attenuated"
            );
            // ... and the mirror image.
            assert_eq!(
                speed_capped_fore_aft(1.0, -v),
                1.0,
                "braking from -{v} m/s was attenuated"
            );
        }
    }

    /// ... whereas an input that would accelerate the board further IS
    /// attenuated, all the way to zero at the cap. Pinned alongside the test
    /// above so the pair reads as the asymmetry it is.
    #[test]
    fn the_speed_cap_withdraws_accelerating_authority_over_the_last_margin() {
        assert_eq!(speed_capped_fore_aft(1.0, 0.0), 1.0);
        assert_eq!(speed_capped_fore_aft(1.0, SPEED_CAP_ONSET_M_S), 1.0);
        let half = speed_capped_fore_aft(1.0, SPEED_CAP_ONSET_M_S + 0.5 * SPEED_CAP_MARGIN_M_S);
        assert!(
            (half - 0.5).abs() < 1e-6,
            "the ramp should be linear across the margin, got {half}"
        );
        assert_eq!(speed_capped_fore_aft(1.0, MAX_GROUND_SPEED_M_S), 0.0);
        assert_eq!(speed_capped_fore_aft(1.0, MAX_GROUND_SPEED_M_S + 5.0), 0.0);
    }

    // --- ADR-0011 criterion (c): the loss-of-authority warning -----------

    /// The discriminator, both halves. Utilisation alone is not the trigger
    /// and speed alone is not the trigger; the warning is the conjunction,
    /// and dropping the speed half turns it into noise on every survivable
    /// high-speed saturation.
    #[test]
    fn the_authority_warning_fires_only_on_high_utilisation_below_the_speed_cap_onset() {
        let hot = AUTHORITY_UTILISATION_WARN + 0.05;
        let cold = AUTHORITY_UTILISATION_WARN - 0.05;
        assert!(authority_warning_active(hot, 2.0), "hot and slow must warn");
        assert!(
            authority_warning_active(hot, -2.0),
            "the discriminator is speed MAGNITUDE -- a board reversing at 2 m/s is as \
             unloaded as one driving at 2 m/s, which is to say not at all"
        );
        assert!(
            !authority_warning_active(hot, SPEED_CAP_ONSET_M_S + 0.1),
            "saturation above the onset is survivable and warning on it is noise"
        );
        assert!(
            !authority_warning_active(cold, 2.0),
            "utilisation below the threshold must not warn"
        );
        assert!(
            !authority_warning_active(AUTHORITY_UTILISATION_WARN, 2.0),
            "the trigger is strictly greater than the threshold"
        );
    }

    /// The filter has to be fast enough to preserve the lead time it exists
    /// to give and slow enough not to chatter on one noisy cycle. Both bounds
    /// asserted, because the constant is chosen by them rather than tuned.
    #[test]
    fn the_authority_filter_is_bounded_by_the_cycle_below_and_the_lead_time_above() {
        let cycle_s = DT_S as f32;
        assert!(
            AUTHORITY_UTILISATION_TAU_S > 10.0 * cycle_s,
            "a filter shorter than ~10 control cycles is not a filter"
        );
        // The lead this warning is quoted at, measured on the estimator path
        // with the reserve disabled -- see AUTHORITY_UTILISATION_WARN's doc
        // comment and `tests/test_cmd_envelope_reserve.py`.
        let measured_lead_s = 1.92_f32;
        assert!(
            AUTHORITY_UTILISATION_TAU_S < 0.2 * measured_lead_s,
            "a filter comparable to the lead time eats the warning it is meant to give"
        );
    }

    /// A first-order filter cannot overshoot its input, so the warning can
    /// never fire on a utilisation the controller never asked for. Cheap, and
    /// it is the property that makes the filtered signal safe to quote.
    #[test]
    fn the_filtered_utilisation_never_exceeds_a_constant_input() {
        let alpha = DT_S as f32 / (AUTHORITY_UTILISATION_TAU_S + DT_S as f32);
        let input = 0.9_f32;
        let mut y = 0.0_f32;
        for _ in 0..10_000 {
            y += (input - y) * alpha;
            assert!(y <= input, "filter overshot: {y} > {input}");
        }
        assert!(
            (y - input).abs() < 1e-3,
            "the filter must actually converge to its input, got {y}"
        );
    }

    /// The steering law's standstill property, which the speed-proportional
    /// curvature formulation exists to give: full stick at zero ground speed
    /// produces EXACTLY zero heading change, so the injection guarded by it is
    /// a literal no-op. You cannot carve a stationary onewheel.
    ///
    /// Through the SHIPPED `yaw_rate_rad_s` rather than a re-typed copy of the
    /// expression: the low-speed tightening below multiplies curvature up as
    /// speed falls, and a test that re-typed the formula could not tell you
    /// whether the `|v|` factor that makes this zero had survived it.
    #[test]
    fn full_steer_at_a_standstill_injects_nothing() {
        for &steer in &[1.0f32, -1.0, 0.6] {
            let dyaw = -yaw_rate_rad_s(steer, 0.0, 1.0) * DT_S as f32;
            assert_eq!(dyaw, 0.0, "steer={steer} at rest must not turn the board");
        }
    }

    /// The turn tightens as the board slows -- the CEO's "cut a tight turn by
    /// braking and turning at the same time".
    ///
    /// Asserted as the ORDERING plus the two endpoints, not as a table of
    /// radii: the magnitude is a feel number and pinning it would turn a
    /// tuning knob into a threshold, but the direction is the whole point and
    /// a sign error here would be invisible in play until someone tried to
    /// carve.
    #[test]
    fn the_turn_radius_shrinks_as_the_board_slows() {
        let radius = |v: f32| 1.0 / (yaw_curvature_per_steer(v) * 1.0);
        let top = radius(YAW_TIGHTEN_REF_SPEED_M_S);
        let rest = radius(0.0);
        assert!(
            (top - 1.0 / YAW_CURVATURE_PER_STEER_RAD_PER_M).abs() < 1e-4,
            "at and above the reference speed the turn must be the width it always was, \
             or this change is a general steering retune wearing a low-speed label"
        );
        assert!(
            (rest - top / YAW_LOW_SPEED_TIGHTEN).abs() < 1e-4,
            "the standstill radius must be exactly the tighten factor, got {rest} vs {top}"
        );
        // Monotone all the way down, with no step. Walked from the reference
        // speed DOWN to rest, which is the direction the property is stated
        // in: each slower sample must turn at least as tightly as the one
        // before it.
        let mut previous = f32::INFINITY;
        for i in (0..=20).rev() {
            let v = YAW_TIGHTEN_REF_SPEED_M_S * (i as f32) / 20.0;
            let r = radius(v);
            assert!(r <= previous + 1e-6, "radius grew as speed fell at v={v}");
            previous = r;
        }
        // Above the reference speed it stays at its widest rather than
        // inverting into an ever-wider turn.
        assert!((radius(YAW_TIGHTEN_REF_SPEED_M_S * 2.0) - top).abs() < 1e-4);
    }

    /// Tightening the low-speed turn must not resurrect the spin-in-place the
    /// speed-proportional formulation was introduced to kill. Yaw RATE must
    /// still fall to zero with speed, even though curvature is rising.
    #[test]
    fn yaw_rate_still_falls_to_zero_with_speed_despite_the_tightening() {
        let mut previous = 0.0f32;
        for i in 0..=20 {
            let v = YAW_TIGHTEN_REF_SPEED_M_S * (i as f32) / 20.0;
            let rate = yaw_rate_rad_s(1.0, v, 1.0);
            assert!(
                rate >= previous - 1e-6,
                "yaw rate is not monotone in speed at v={v}"
            );
            previous = rate;
        }
        assert_eq!(yaw_rate_rad_s(1.0, 0.0, 1.0), 0.0);
    }
}
