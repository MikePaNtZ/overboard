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

use crate::pacer::Pacer;
use crate::wire::{self, InputIn, StateOut};
use board_types::{Command, Faults, ImuSample, Params, RAD_S_PER_ERPM};
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

/// Non-physical game channel: full `steer` deflection integrates `yaw_rad`
/// at this rate, SHAPED by [`ROLL_FULL_YAW_AUTHORITY_RAD`] below -- the
/// simulated wheel is a cylinder and cannot physically carve (issue #161).
/// The real lean-steer controller is Tuesday.
const YAW_RATE_GAIN_RAD_S: f32 = 1.5;

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
const INPUT_STALENESS_TIMEOUT: Duration = Duration::from_millis(100);

/// Placeholder threshold for the state-out FALLEN bit.
/// `sim/scenarios/disturbance_envelope.py` derives the REAL nose-strike
/// angle (18.57 deg) from the model's collision hulls; this crate has no
/// Rust binding to that geometry query, so this is a fixed proxy near that
/// value, not the real contact test -- good enough to prove "the board is
/// clearly down", not precise enough to gate a published claim.
const FALLEN_PITCH_RAD: f32 = 20.0 * std::f32::consts::PI / 180.0;

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
}

impl Default for HostConfig {
    fn default() -> Self {
        HostConfig {
            state_out_addr: wire::state_out_addr(),
            input_in_addr: wire::input_in_addr(),
            duration: None,
            stats_path: Some(PathBuf::from(DEFAULT_STATS_PATH)),
            startup_kick: false,
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
        }
    }
}

/// Spawns the control loop on its own dedicated thread (issue #161: "not on
/// the main thread") and returns the join handle. The caller decides what to
/// do with the calling thread -- `src/bin/sim-host.rs` just joins it.
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
    let mut prev_armed_bit = false;
    let mut prev_reset_bit = false;
    let mut yaw_rad: f32 = 0.0;
    let mut wheel_angle_rad: f32 = 0.0;

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
        // timeout (issue #161).
        let steer = if stale { 0.0 } else { latest_input.steer };
        let weight_shift_fore_aft = if stale {
            0.0
        } else {
            latest_input.weight_shift_fore_aft
        };
        let weight_shift_lateral = if stale {
            0.0
        } else {
            latest_input.weight_shift_lateral
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

        // Ballast targets -- weight_shift_fore_aft/lateral drive
        // overboard_rider.xml's two ballast actuators DIRECTLY AND
        // PHYSICALLY (see this file's header). Set every cycle, mirroring
        // apply_external_force's own "call every cycle or a stale value
        // persists" convention.
        backend.set_ballast_targets(
            weight_shift_fore_aft * BALLAST_RANGE_M,
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
        let force = if in_kick_window {
            STARTUP_KICK_FORCE_N
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
        let pitch_rad = (xmat[2] as f32).atan2(xmat[8] as f32);
        // Roll (rotation about the frame's forward/-X axis) -- the same
        // atan2-of-a-tilted-axis derivation `pitch_rad` uses, applied to the
        // Y-Z plane (roll, about local X) instead of the X-Z plane (pitch,
        // about local Y). Exact only when pitch is near zero, since 3D
        // rotations don't commute and this is not a true Euler decomposition
        // -- acceptable for the roll-shaped yaw limiter's "cheap stopgap"
        // status (issue #161 W2 item 4); the real lean-steer controller
        // (Tuesday) needs a better one.
        let roll_rad = (xmat[5] as f32).atan2(xmat[8] as f32);
        let pos_f64 = backend.truth_frame_xpos();
        let quat_f64 = backend.truth_frame_xquat();
        let pos = [pos_f64[0] as f32, pos_f64[1] as f32, pos_f64[2] as f32];
        let quat = [
            quat_f64[0] as f32,
            quat_f64[1] as f32,
            quat_f64[2] as f32,
            quat_f64[3] as f32,
        ];

        // yaw_rad: NON-PHYSICAL game channel (issue #161). `steer` supplies
        // the sign/direction; `roll_authority` scales its magnitude between
        // YAW_AUTHORITY_FLOOR (steer alone, however little the player is
        // leaning) and 1.0 (at/above ROLL_FULL_YAW_AUTHORITY_RAD's measured,
        // physically-achievable roll). Read BOTH constants' doc comments
        // before touching this line -- on the current (widened) wheel
        // geometry, achievable roll is ~0.03 deg, so `steer` is effectively
        // the primary driver of yaw this weekend, NOT roll; the floor is
        // what makes that honest instead of a limiter that reads as "off".
        let roll_authority = YAW_AUTHORITY_FLOOR
            + (1.0 - YAW_AUTHORITY_FLOOR)
                * (roll_rad.abs() / ROLL_FULL_YAW_AUTHORITY_RAD).clamp(0.0, 1.0);
        yaw_rad += steer * YAW_RATE_GAIN_RAD_S * roll_authority * DT_S as f32;

        let mut flags = wire::STATE_FLAG_ARMED | wire::STATE_FLAG_VALID;
        if pitch_rad.abs() > FALLEN_PITCH_RAD {
            flags |= wire::STATE_FLAG_FALLEN;
        }

        let state = StateOut {
            magic: wire::STATE_MAGIC,
            schema_version: wire::SCHEMA_VERSION,
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
        };
        out_socket.send_to(&state.to_bytes(), cfg.state_out_addr)?;

        ticks += 1;

        if let Some(path) = &cfg.stats_path {
            if last_stats_write.elapsed() >= Duration::from_millis(100) {
                write_stats(path, ticks, pacer.missed_deadlines());
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
        write_stats(path, ticks, pacer.missed_deadlines());
    }

    let _ = backend.close();

    Ok(RunSummary {
        ticks,
        missed_deadlines: pacer.missed_deadlines(),
    })
}

/// Best-effort, atomic (write-then-rename) write of the host's own counters,
/// for `wire-probe` to pick up. Never allowed to interrupt the control loop
/// -- a failure here is silently swallowed ON PURPOSE (unlike a missed
/// deadline or a malformed input packet, which issue #161 requires surfacing
/// loudly): this file is internal tooling, not the wire.
fn write_stats(path: &std::path::Path, ticks: u64, missed_deadlines: u64) {
    let tmp = path.with_extension("tmp");
    let contents = format!("ticks={ticks}\nmissed_deadlines={missed_deadlines}\n");
    let _ = std::fs::write(&tmp, contents).and_then(|_| std::fs::rename(&tmp, path));
}
