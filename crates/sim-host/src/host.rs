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

use crate::pacer::Pacer;
use crate::wire::{self, InputIn, StateOut};
use board_types::{Command, Faults, ImuSample, Params, DEFAULT_R_EFF_M, RAD_S_PER_ERPM};
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
const KT_NM_PER_A: f32 = 0.7;
const MAX_CURRENT_A: f32 = 40.0;
const ESTIMATOR_TAU_S: f32 = 2.0;
/// Command-feedforward gain, m/s^2 per amp. `control_ffi::ob_controller_new`'s
/// own hardcoded fallback for `kt` 0.7 N*m/A / (`r_eff` 0.1454 m * 82.5 kg
/// total ridden mass) -- 8 kg frame + 4.5 kg wheel + 0.5 kg ballast carrier +
/// 70 kg rider = 82.5 kg, matching `overboard_rider.xml` exactly.
/// `shuttle_run.py`'s tuned controller relies on this same FFI fallback
/// rather than passing its own gain, so this host does too, duplicated here
/// because this host does not link `control-ffi`.
const ACCEL_FF_GAIN_M_S2_PER_A: f32 = 0.0584;

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
        }
    }
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
        let (steer, weight_shift_fore_aft, weight_shift_lateral) = match cfg.scripted_scenario {
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
        let speed_headroom_m_s = MAX_GROUND_SPEED_M_S - last_forward_speed_m_s.abs();
        let accel_authority = (speed_headroom_m_s / SPEED_CAP_MARGIN_M_S).clamp(0.0, 1.0);
        let accelerating_same_direction = weight_shift_fore_aft * last_forward_speed_m_s >= 0.0;
        let capped_weight_shift_fore_aft = if accelerating_same_direction {
            weight_shift_fore_aft * accel_authority
        } else {
            weight_shift_fore_aft
        };

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
        let force = if in_kick_window {
            STARTUP_KICK_FORCE_N
        } else if in_fall_kick_window {
            FALL_KICK_FORCE_N
        } else {
            [0.0; 3]
        };
        backend.apply_external_force(force, [0.0; 3]);

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
        let attitude = estimator.update(std::slice::from_ref(&sample), aiding);
        let proposed_torque_nm =
            regulator.update(attitude.pitch_rad, attitude.pitch_rate_rad_s, 0.0);
        // The single kt division -- the actuation boundary (issue #137).
        let proposed_amps = proposed_torque_nm / KT_NM_PER_A;
        let (bounded_cmd, _sat) = envelope.apply(
            Command::MotorCurrent {
                amps: proposed_amps,
            },
            Faults::NONE,
        );
        backend.apply(&bounded_cmd).map_err(HostError::Backend)?;
        // POST-envelope current, not the proposal -- the plant only ever
        // sees the clamped value (same reasoning `control-ffi`'s own
        // `ctl.last_amps` update carries).
        last_amps = match bounded_cmd {
            Command::MotorCurrent { amps } => amps,
            Command::RemoteSpeed { .. } => 0.0,
        };

        // wheel_angle_rad: there is no absolute wheel-angle channel on `hal`
        // (ICD carries ERPM/tacho, the same as real VESC telemetry) -- this
        // dead-reckons it from the rate `hal` already reports, exactly the
        // way a real host would have to. Not a fabricated value: it is an
        // honest integral of an actually-measured rate.
        wheel_angle_rad += wheel_rate_rad_s * DT_S as f32;

        // Ground truth, never fed to the controller above (DR-OBS-1) --
        // reported because "the board is actually up" is what the state-out
        // wire needs to prove, and truth proves it more directly than the
        // controller's own (estimator-mediated) belief. Same formula
        // `impulse-response-rust` / `sim/scenarios/impulse_response.py::
        // frame_pitch_rad` use, against the same underlying xmat.
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
        if pitch_rad.abs() > FALLEN_PITCH_RAD {
            flags |= wire::STATE_FLAG_FALLEN;
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
        let yaw_rate_rad_s =
            steer * YAW_CURVATURE_PER_STEER_RAD_PER_M * forward_speed_m_s.abs() * roll_authority;
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

    if let Some(path) = &cfg.stats_path {
        write_stats(
            path,
            ticks,
            pacer.missed_deadlines(),
            truth_pos_x_m,
            truth_pos_y_m,
        );
    }

    let _ = backend.close();

    Ok(RunSummary {
        ticks,
        missed_deadlines: pacer.missed_deadlines(),
    })
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

    /// The steering law's standstill property, which the speed-proportional
    /// curvature formulation exists to give: full stick at zero ground speed
    /// produces EXACTLY zero heading change, so the injection guarded by it is
    /// a literal no-op. You cannot carve a stationary onewheel.
    #[test]
    fn full_steer_at_a_standstill_injects_nothing() {
        for &steer in &[1.0f32, -1.0, 0.6] {
            let roll_authority = 1.0f32;
            let yaw_rate = steer * YAW_CURVATURE_PER_STEER_RAD_PER_M * 0.0f32 * roll_authority;
            let dyaw = -yaw_rate * DT_S as f32;
            assert_eq!(dyaw, 0.0, "steer={steer} at rest must not turn the board");
        }
    }
}
