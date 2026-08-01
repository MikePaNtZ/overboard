//! The 500 Hz control loop, dedicated thread, UDP in/out (issue #161).
//!
//! Wires the SAME `control-core`/`safety` objects `control-ffi::ob_controller_
//! update` wires for the Python-hosted scenario, and `board-app-driverless`'s
//! `impulse-response-rust` wires for the Rust-hosted one -- reached here
//! through `hal` (`SimBackend::wait_observe`/`apply`) exactly like that
//! binary, just paced against a UDP client instead of a fixed step count.
//! Nothing here is a new control law.

use crate::pacer::Pacer;
use crate::wire::{self, InputIn, StateOut};
use board_types::{Command, Faults, ImuSample, Params, DEFAULT_R_EFF_M, RAD_S_PER_ERPM};
use control_core::{ComplementaryFilter, Estimator, PitchRegulator, WheelAccelEstimator};
use hal::BoardObserve;
use hal_actuate::BoardActuate;
use safety::Envelope;
use sim_backend::SimBackend;
use std::io::ErrorKind;
use std::net::{SocketAddr, UdpSocket};
use std::path::PathBuf;
use std::time::{Duration, Instant};

/// Nanoseconds per control cycle -- 500 Hz, matching `sim-backend`'s own
/// `CYCLE_NS` (ICD SS11.2). Duplicated the same way that crate duplicates
/// `KT_NM_PER_A` against the model, because there is no public constant to
/// import; [`run`] asserts it against `SimBackend::run_metadata()` on open
/// rather than trusting the duplication silently.
pub const CYCLE_NS: u64 = 2_000_000;
const DT_S: f64 = CYCLE_NS as f64 * 1e-9;

// --- Controller config -- identical to
// `board-app-driverless/src/bin/impulse-response-rust.rs`'s constants (issue
// #107 AC6), which mirror `sim/scenarios/rust_controller.py`'s own defaults.
// No outer velocity loop: W1 is pure station-keeping balance, not driving.
const KP_NM_PER_RAD: f32 = 56.0;
const KD_NM_PER_RAD_S: f32 = 7.7;
const KT_NM_PER_A: f32 = 0.7;
const MAX_CURRENT_A: f32 = 40.0;
const ESTIMATOR_TAU_S: f32 = 1.0;
const WHEEL_ACCEL_TAU_S: f32 = 0.05;

/// Non-physical game channel: full `steer` deflection integrates `yaw_rad`
/// at this rate. An arbitrary "game feel" placeholder for W1 -- the
/// simulated wheel is a cylinder and cannot physically carve (issue #161).
/// W2's lean-steer controller replaces this with something real.
const YAW_RATE_GAIN_RAD_S: f32 = 1.5;

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

/// A ONE-TIME startup disturbance, applied near the beginning of every run.
/// Not a general disturbance API -- the input wire carries no such field
/// (issue #161) -- and not required by the wire spec itself. It exists so a
/// verification run (`wire-probe`) demonstrates the controller actually
/// RECOVERING from a push, rather than sitting at a motionless, never-
/// disturbed equilibrium the whole time: with nothing to react to,
/// `pitch_rad` would report exactly 0.0 forever, which satisfies the wire
/// but proves nothing about the control loop. Same magnitude and duration
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
    /// primarily for tests and the verification run in issue #161's PR,
    /// not something a deployed host would set.
    pub duration: Option<Duration>,
    /// `None` disables the stats file entirely.
    pub stats_path: Option<PathBuf>,
}

impl Default for HostConfig {
    fn default() -> Self {
        HostConfig {
            state_out_addr: wire::state_out_addr(),
            input_in_addr: wire::input_in_addr(),
            duration: None,
            stats_path: Some(PathBuf::from(DEFAULT_STATS_PATH)),
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

/// The most recent input the host has heard from Unreal, plus the bits W1
/// does not yet act on (accepted, clamped, and logged on receipt -- issue
/// #161 -- so W2 only has to connect them to something).
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

    let mut backend = SimBackend::with_params(params);
    backend.open().map_err(HostError::Backend)?;
    // Armed unconditionally at startup, the same way every other Rust-hosted
    // harness in this repo arms (`impulse-response-rust`, `sim-backend`'s own
    // tests): there is no synthetic Unreal client during the issue #161
    // verification run, and W1 explicitly does not gate balancing on the
    // input socket's `arm` bit -- see `LatestInput::armed_bit`, tracked and
    // logged but not yet wired to anything (W2).
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
    let mut wheel_accel = WheelAccelEstimator::new(WHEEL_ACCEL_TAU_S);

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
        // weight_shift_* and steer hold last, then zero after the staleness
        // timeout (issue #161). weight_shift_* is read, clamped (already
        // done in InputIn::from_bytes) and logged above, but does not yet
        // drive the model (W1 scope) -- wired here so W2 only has to
        // connect it to an actuator.
        let steer = if stale { 0.0 } else { latest_input.steer };
        let _weight_shift_fore_aft = if stale {
            0.0
        } else {
            latest_input.weight_shift_fore_aft
        };
        let _weight_shift_lateral = if stale {
            0.0
        } else {
            latest_input.weight_shift_lateral
        };

        // `arm`/`reset` bits: accepted and tracked (W1 scope), logged on
        // change rather than every tick. Neither gates anything yet -- the
        // host self-arms unconditionally (see the comment on `backend.arm()`
        // above), and there is no reset implementation to wire `reset` into
        // (W2). `stale` zeroes both the same way it zeroes weight_shift/steer.
        let input_armed_bit = !stale && latest_input.armed_bit;
        let input_reset_bit = !stale && latest_input.reset_bit;
        if input_reset_bit && !prev_reset_bit {
            eprintln!("sim-host: input reset bit set -- W1 does not implement reset yet, ignoring");
        }
        if input_armed_bit != prev_armed_bit {
            eprintln!(
                "sim-host: input arm bit is now {input_armed_bit} \
                 (host self-arms regardless of this bit in W1)"
            );
        }
        prev_armed_bit = input_armed_bit;
        prev_reset_bit = input_reset_bit;

        // One-time startup kick -- see STARTUP_KICK_T0_S's doc comment.
        // Checked against the PRE-step time, mirroring impulse_response.py's
        // `if t0 <= data.time < t0 + duration` (also checked there before
        // that iteration's mj_step).
        let in_kick_window =
            (STARTUP_KICK_T0_S..STARTUP_KICK_T0_S + STARTUP_KICK_DURATION_S).contains(&t_known_s);
        let force = if in_kick_window {
            STARTUP_KICK_FORCE_N
        } else {
            [0.0; 3]
        };
        backend.apply_external_force(force, [0.0; 3]);

        let obs = backend.wait_observe().map_err(HostError::Backend)?;
        t_known_s = obs.t_recv_ns as f64 * 1e-9;

        // Controller: raw IMU -> estimate -> regulate -> envelope. Mode 1
        // (wheel odometry aiding), matching `impulse-response-rust` and
        // `RustController()`'s own default (issue #121).
        let sample = obs.newest_imu().copied().unwrap_or(ImuSample::ZERO);
        let wheel_rate_rad_s = obs.erpm * RAD_S_PER_ERPM;
        let aiding = wheel_accel.update(wheel_rate_rad_s * DEFAULT_R_EFF_M, DT_S as f32);
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

        // wheel_angle_rad: there is no absolute wheel-angle channel on `hal`
        // (ICD carries ERPM/tacho, the same as real VESC telemetry) -- this
        // dead-reckons it from the rate `hal` already reports, exactly the
        // way a real host would have to. Not a fabricated value: it is an
        // honest integral of an actually-measured rate.
        wheel_angle_rad += wheel_rate_rad_s * DT_S as f32;
        // yaw_rad: NON-PHYSICAL game channel (issue #161) -- see
        // YAW_RATE_GAIN_RAD_S's doc comment.
        yaw_rad += steer * YAW_RATE_GAIN_RAD_S * DT_S as f32;

        // Ground truth, never fed to the controller above (DR-OBS-1) --
        // reported because "the board is actually up" is what the state-out
        // wire needs to prove, and truth proves it more directly than the
        // controller's own (estimator-mediated) belief. Same formula
        // `impulse-response-rust` / `sim/scenarios/impulse_response.py::
        // frame_pitch_rad` use, against the same underlying xmat.
        let xmat = backend.truth_frame_xmat();
        let pitch_rad = (xmat[2] as f32).atan2(xmat[8] as f32);
        let pos_f64 = backend.truth_frame_xpos();
        let quat_f64 = backend.truth_frame_xquat();
        let pos = [pos_f64[0] as f32, pos_f64[1] as f32, pos_f64[2] as f32];
        let quat = [
            quat_f64[0] as f32,
            quat_f64[1] as f32,
            quat_f64[2] as f32,
            quat_f64[3] as f32,
        ];

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
