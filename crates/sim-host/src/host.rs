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
/// Hamilton product of two `[w, x, y, z]` quaternions.
///
/// Order is `a` then `b` read right-to-left: the result applies `b`'s rotation first, then `a`'s.
/// Used to compose the synthetic heading (`a`) onto MuJoCo's truth attitude (`b`) -- see the call
/// site for why that order, and not the other one, is the correct composition.
fn quat_mul(a: [f32; 4], b: [f32; 4]) -> [f32; 4] {
    let [aw, ax, ay, az] = a;
    let [bw, bx, by, bz] = b;
    [
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ]
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
    let mut prev_armed_bit = false;
    let mut prev_reset_bit = false;
    let mut yaw_rad: f32 = 0.0;
    let mut wheel_angle_rad: f32 = 0.0;
    // Dead-reckoned game-ground path -- see the PARTIALLY SYNTHETIC POSITION
    // block further down for why this exists and what it costs. f64: this
    // accumulates every tick for the whole run, and f32 would visibly drift
    // over a multi-minute session the way `wheel_angle_rad` (f32, but reset
    // every run and never compared against a long-run reference) does not
    // need to guard against.
    let mut dr_pos_x_m: f64 = 0.0;
    let mut dr_pos_y_m: f64 = 0.0;
    // Latest TRUE MuJoCo x/y, kept for `write_stats` -- the out-of-band
    // channel that keeps ground truth available now that `pos`'s x/y on the
    // wire itself are dead-reckoned, not MuJoCo truth. See the PARTIALLY
    // SYNTHETIC POSITION block below.
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
        truth_pos_x_m = pos_f64[0];
        truth_pos_y_m = pos_f64[1];
        let quat_f64 = backend.truth_frame_xquat();
        // MuJoCo's own attitude: pitch and roll, and NO yaw -- this plant has no yaw degree of
        // freedom at all. The heading is bolted on below, after `yaw_rad` is updated.
        let quat_truth = [
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
        // Updated BEFORE the position dead-reckoning below, so that
        // reckoning always projects against this tick's freshest heading.
        let roll_authority = YAW_AUTHORITY_FLOOR
            + (1.0 - YAW_AUTHORITY_FLOOR)
                * (roll_rad.abs() / ROLL_FULL_YAW_AUTHORITY_RAD).clamp(0.0, 1.0);
        yaw_rad += steer * YAW_RATE_GAIN_RAD_S * roll_authority * DT_S as f32;

        // --- PARTIALLY SYNTHETIC POSITION (issue #161/#163) -------------
        //
        // MuJoCo only ever translates this plant along its own -X axis
        // (there is no lateral/carving force anywhere in this model --
        // `yaw_rad` never touches the physics). Sending MuJoCo's own
        // x/y straight through, the way W1/W2's first cut did, is
        // therefore honest about the SIM but dishonest about the GAME: on
        // screen the board would spin to face `yaw_rad` while sliding
        // along its original straight line underneath -- a car spinning
        // out, not a board carving -- and that reads as a physics bug to
        // anyone watching, on the single artifact (Monday's footage) this
        // whole channel exists to protect.
        //
        // So `pos`'s x/y are DEAD-RECKONED here, in the host (never in
        // Unreal -- the renderer computes no board state, ADR-0009 and
        // `overboard-game`'s own README are explicit about that boundary):
        // REAL forward ground speed (`wheel_rate_rad_s * DEFAULT_R_EFF_M`,
        // straight off `hal`, nothing invented) is projected along the
        // SYNTHETIC heading (`yaw_rad`) and integrated every tick. The
        // result is a curved path that matches where the board is
        // pointing -- exactly as partially synthetic as the heading that
        // drives it, no more and no less.
        //
        // z is untouched: vertical position stays real MuJoCo truth (the
        // wheel's own rolling motion in the sagittal plane needs no
        // reckoning -- MuJoCo already integrates that correctly, which is
        // the entire property this block exists to compensate for the
        // LACK of in the lateral plane).
        //
        // This is a NEW non-physical channel, same status as `yaw_rad`/
        // `steer` -- it needs its own line in the `Playable Sim` channel
        // declaration (issue #163), flagged in this PR's body rather than
        // added to `docs/vocabulary/` directly (Archivist's path, not
        // this crate's). True MuJoCo x/y is NOT deleted -- see
        // `write_stats`'s `truth_pos_x_m`/`truth_pos_y_m` lines below,
        // which keep it available out-of-band for anything downstream
        // that needs ground truth rather than the game path.
        let heading_x = -yaw_rad.cos();
        let heading_y = -yaw_rad.sin();
        let forward_speed_m_s = wheel_rate_rad_s * DEFAULT_R_EFF_M;
        dr_pos_x_m += (forward_speed_m_s * heading_x) as f64 * DT_S;
        dr_pos_y_m += (forward_speed_m_s * heading_y) as f64 * DT_S;
        let pos = [dr_pos_x_m as f32, dr_pos_y_m as f32, pos_f64[2] as f32];

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

        // --- SYNTHETIC HEADING, APPLIED TO ATTITUDE TOO (issue #161/#163) ------------
        //
        // The dead reckoning above curves the PATH along `yaw_rad`. Sending MuJoCo's raw
        // quaternion alongside it -- which is what the first cut of this block did -- makes the
        // board travel that curve without ever turning to face it: it crabs sideways, nose fixed,
        // which reads as a physics bug just as loudly as the failure the comment above describes.
        // The two are the same mismatch with opposite signs, and fixing only one of them swaps
        // which half is wrong.
        //
        // So the same synthetic heading that steers the path also rotates the body. Composed
        // HERE, in MuJoCo's frame and before the wire, so `MuJoCoToUnreal`'s handedness flip
        // applies to it exactly as it does to the truth attitude -- the renderer keeps computing
        // nothing (ADR-0009), and there is one heading in the system rather than two that can
        // disagree.
        //
        // Yaw is about +Z, applied on the LEFT: world-frame heading first, then the board's own
        // pitch/roll within it. Right-multiplying would apply the heading in the board's tilted
        // local frame, so a leaning board would yaw about its own tilted axis and drift out of
        // the ground plane. The sign is fixed by the dead reckoning it must agree with: rotating
        // about +Z by `yaw_rad` maps (-1, 0) to (-cos, -sin), which IS (heading_x, heading_y).
        //
        // `quat_truth` is not discarded -- write_stats still logs the unmodified MuJoCo attitude
        // alongside truth_pos_x_m/truth_pos_y_m, so ground truth stays available out-of-band.
        let (yaw_sin_half, yaw_cos_half) = (yaw_rad * 0.5).sin_cos();
        let quat = quat_mul([yaw_cos_half, 0.0, 0.0, yaw_sin_half], quat_truth);

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
/// internal tooling, not the wire. `truth_pos_x_m`/`truth_pos_y_m` exist
/// because `pos`'s x/y on the wire are now dead-reckoned, not MuJoCo truth
/// (see the PARTIALLY SYNTHETIC POSITION block in `run`) -- ground truth
/// must stay reachable for anything downstream that needs it.
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

    /// Rotate `v` by quaternion `q` (`q v q*`).
    fn rotate(q: [f32; 4], v: [f32; 3]) -> [f32; 3] {
        let qv = [0.0, v[0], v[1], v[2]];
        let qc = [q[0], -q[1], -q[2], -q[3]];
        let r = quat_mul(quat_mul(q, qv), qc);
        [r[1], r[2], r[3]]
    }

    fn yaw_quat(yaw_rad: f32) -> [f32; 4] {
        let (s, c) = (yaw_rad * 0.5).sin_cos();
        [c, 0.0, 0.0, s]
    }

    #[test]
    fn quat_mul_identity_is_identity() {
        // 90 deg about +Y. Spelled with the constant rather than 0.7071068, which clippy
        // (correctly) flags as an approximation of it.
        const H: f32 = std::f32::consts::FRAC_1_SQRT_2;
        let q = [H, 0.0, H, 0.0];
        let i = [1.0, 0.0, 0.0, 0.0];
        for (a, b) in quat_mul(i, q).iter().zip(q.iter()) {
            assert!((a - b).abs() < 1e-6, "identity * q != q");
        }
        for (a, b) in quat_mul(q, i).iter().zip(q.iter()) {
            assert!((a - b).abs() < 1e-6, "q * identity != q");
        }
    }

    /// THE invariant this whole fix exists for: the attitude on the wire must point the board
    /// along the heading the position dead reckoning is steering it down. Before the fix, `quat`
    /// was MuJoCo's raw attitude, which has no yaw at all -- so the board crabbed along a curved
    /// path with its nose fixed. `yaw_rad` was transmitted and used by nobody.
    ///
    /// The board's nose is its LOCAL -X (see `overboard_onewheel.xml`: "FORWARD IS -X"), and the
    /// dead reckoning drives it along `(-cos yaw, -sin yaw)`. Those must be the same direction.
    #[test]
    fn synthetic_heading_rotates_the_body_it_steers() {
        let level = [1.0f32, 0.0, 0.0, 0.0];
        for &yaw in &[0.0f32, 0.3, -0.7, 1.9, -2.8, 3.0] {
            let quat = quat_mul(yaw_quat(yaw), level);
            let nose = rotate(quat, [-1.0, 0.0, 0.0]);

            // Exactly the two lines the position integrator uses.
            let heading_x = -yaw.cos();
            let heading_y = -yaw.sin();

            assert!(
                (nose[0] - heading_x).abs() < 1e-5 && (nose[1] - heading_y).abs() < 1e-5,
                "yaw={yaw}: nose points ({}, {}) but the path is steered along ({heading_x}, {heading_y})",
                nose[0],
                nose[1]
            );
        }
    }

    /// The heading must be composed on the LEFT (world frame), not the right (body frame).
    ///
    /// With the board rolled, right-multiplying yaws it about its own tilted axis, which lifts the
    /// nose out of the ground plane and makes the rendered heading disagree with the flat path the
    /// dead reckoning integrates. Left-multiplying keeps the two consistent: whatever the board's
    /// attitude, its nose still projects onto the heading the position is following.
    #[test]
    fn heading_is_applied_in_the_world_frame_not_the_body_frame() {
        let roll = 0.35f32; // rolled board, as during a carve
        let (s, c) = (roll * 0.5).sin_cos();
        let rolled = [c, s, 0.0, 0.0]; // rotation about local X
        let yaw = 0.9f32;

        let correct = rotate(quat_mul(yaw_quat(yaw), rolled), [-1.0, 0.0, 0.0]);
        let wrong = rotate(quat_mul(rolled, yaw_quat(yaw)), [-1.0, 0.0, 0.0]);

        let (hx, hy) = (-yaw.cos(), -yaw.sin());
        assert!(
            (correct[0] - hx).abs() < 1e-5 && (correct[1] - hy).abs() < 1e-5,
            "left-composed heading should match the dead-reckoned one"
        );
        // Guard against someone "simplifying" the order later: on a rolled board the two differ.
        assert!(
            (wrong[0] - hx).abs() > 1e-3 || (wrong[1] - hy).abs() > 1e-3,
            "body-frame composition must NOT agree here, or this test proves nothing"
        );
    }
}
