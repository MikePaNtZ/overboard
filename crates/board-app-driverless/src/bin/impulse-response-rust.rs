//! Closed-loop impulse-response, hosted through the `hal` seam -- issue #107
//! (I1c) AC6.
//!
//! `tests/test_rust_hosted_impulse_response.py` builds this once (`cargo
//! build --release -p board-app-driverless --bin impulse-response-rust`)
//! and shells out to it, exactly the way `test_rust_python_plant_replay_
//! equivalence.py` shells out to `plant-replay` -- this binary asserts
//! nothing itself; the comparison against the Python-hosted result lives in
//! that test.
//!
//! # Why this exists as its own binary rather than reusing `board-app-driverless`
//!
//! `board-app-driverless`'s own loop still calls `control_core::Controller`,
//! which is an intentional, pre-existing stub that always returns
//! `Command::ZERO` (see that crate's doc comment: "the real law lands with
//! the stand phase"). Issue #107 explicitly excludes any change to the
//! control law, so this binary does not touch that stub -- instead it wires
//! `control_core::PitchRegulator` + `ComplementaryFilter` +
//! `CommandFeedforward` + `safety::Envelope` directly, the same objects
//! `control-ffi::ob_controller_update` wires for the Python-hosted scenario,
//! reached the `hal` way (`wait_observe()` / `apply()`) instead of the C ABI.
//! Nothing here is a new control law -- it is the existing one, driven
//! through the other seam.
//!
//! # Why `estimator_accel_aiding` mode 2 (command feedforward), not mode 1
//! (wheel odometry)
//!
//! `hal::Observation` carries raw IMU and `motor_current_a` (DR-OBS-1) but no
//! wheel-rate/ERPM channel yet -- `control_core::Controller`'s real wiring
//! (a separate, later increment) is where that plumbing belongs, not a
//! throwaway harness. Mode 2 needs only the current already on the
//! observation, so it is the config this harness can host faithfully today.
//! `tests/test_rust_hosted_impulse_response.py` configures its Python-hosted
//! comparator identically (same mode, same gain, same gate), rather than
//! reusing `test_closed_loop.py`'s wheel-odometry default -- the two are not
//! interchangeable and comparing across the difference would not test this
//! seam.
//!
//! # Output
//!
//! Four ASCII floats, one per line, in this exact order:
//! `peak_abs_pitch_deg`, `t_peak_s`, `settle_time_s` (`-1.0` means "never
//! settled", mirroring the Python side's `None`), `final_pitch_deg`.

use board_types::{Command, Faults, ImuSample, Params};
use control_core::{CommandFeedforward, ComplementaryFilter, Estimator, PitchRegulator};
use hal::BoardObserve;
use hal_actuate::BoardActuate;
use safety::Envelope;
use sim_backend::SimBackend;
use std::process::ExitCode;

// --- ImpulseParams defaults (issue #107 AC6), mirroring
// sim/scenarios/impulse_response.py::ImpulseParams's own defaults. Duplicated
// the same way board_types::DEFAULT_R_EFF_M duplicates the model's tire
// radius -- this binary has no Python binding to read the dataclass from
// directly, so the values are pinned here and the comparison test asserts
// against the SAME Python-side defaults, not a second, independently-chosen
// set.
const NOMINAL_IMPULSE_NS: f64 = 20.0;
const T0_S: f64 = 0.5;
const DURATION_S: f64 = 0.05;
const SIM_SECONDS: f64 = 8.0;
const SETTLE_BAND_DEG: f64 = 2.0;
/// `direction=(-1,0,0)` and `application_height_m=0.0` in `ImpulseParams`,
/// so `torque = cross([0,0,0], force) = 0` always -- only `force` is nonzero.
const FORCE_N: [f64; 3] = [-(NOMINAL_IMPULSE_NS / DURATION_S), 0.0, 0.0];

// --- Controller config (issue #107 AC6). Kp/Kd/max_current_a/r_eff_m match
// sim/scenarios/rust_controller.py's DEFAULT_* constants; com_above_axle
// matches the driverless plant (mass below the axle); estimator_tau_s and
// the feedforward gain fallback match control-ffi::ob_controller_new's own
// defaults for these modes. See the module doc comment for why aiding mode 2
// (command feedforward) is used instead of `RustController()`'s own default
// (mode 1, wheel odometry).
const KP_A_PER_RAD: f32 = 80.0;
const KD_A_PER_RAD_S: f32 = 11.0;
const MAX_CURRENT_A: f32 = 40.0;
const ESTIMATOR_TAU_S: f32 = 1.0;
/// `control-ffi::ob_controller_new`'s own fallback when
/// `accel_ff_gain_m_s2_per_a <= 0.0` -- kept identical here so both hosts run
/// the exact same numeric gain, not two independently-chosen ones.
const ACCEL_FF_GAIN_M_S2_PER_A: f32 = 0.0584;

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().collect();
    let [_, out_path] = args.as_slice() else {
        eprintln!("usage: impulse-response-rust <out.txt>");
        return ExitCode::FAILURE;
    };

    let params = Params {
        kp: KP_A_PER_RAD,
        kd: KD_A_PER_RAD_S,
        max_current_a: MAX_CURRENT_A,
        ..Params::default()
    };

    let mut backend = SimBackend::with_params(params);
    if let Err(e) = backend.open() {
        eprintln!("impulse-response-rust: backend open failed: {e:?}");
        return ExitCode::FAILURE;
    }
    let _disarm = match backend.arm() {
        Ok(h) => h,
        Err(e) => {
            eprintln!("impulse-response-rust: backend arm failed: {e:?}");
            return ExitCode::FAILURE;
        }
    };

    let mut envelope = Envelope::new(params);
    envelope.arm();

    let regulator = PitchRegulator::new(KP_A_PER_RAD, KD_A_PER_RAD_S);
    let mut estimator = ComplementaryFilter::with_trust_band(ESTIMATOR_TAU_S, 0.0);
    let feedforward = CommandFeedforward::new(ACCEL_FF_GAIN_M_S2_PER_A);
    let mut last_amps: f32 = 0.0;

    // dt used only to size the run and to mirror
    // sim/scenarios/impulse_response.py's `n_steps = round(sim_seconds/dt)`.
    // The onewheel model's dt (0.002 s) equals CYCLE_NS (2e6 ns / 500 Hz), so
    // this backend's stepping ratio (AC2) is exactly 1: one hal cycle is one
    // mj_step, matching the Python scenario's own per-mj_step loop 1:1.
    let dt_s = 0.002_f64;
    let n_steps = (SIM_SECONDS / dt_s).round() as u64;

    // Time at which the NEXT wait_observe()'s mj_step will begin -- what
    // Python calls `data.time` at the TOP of its loop, before that
    // iteration's mj_step. mjData starts at t=0 after open() (Plant::open +
    // the AC8 mj_forward priming call), matching a fresh mujoco.MjData(model).
    let mut t_known_s: f64 = 0.0;

    let mut peak_abs_pitch_deg: f64 = 0.0;
    let mut t_peak_s: f64 = 0.0;
    let mut last_outside_band_s: Option<f64> = None;
    let mut final_pitch_deg: f64 = 0.0;

    for _ in 0..n_steps {
        // Disturbance window check against the PRE-step time, mirroring
        // impulse_response.py's `if t0 <= data.time < t0 + duration` (checked
        // there before that iteration's mj_step too).
        let in_window = (T0_S..T0_S + DURATION_S).contains(&t_known_s);
        let force = if in_window { FORCE_N } else { [0.0; 3] };
        backend.apply_external_force(force, [0.0; 3]);

        let obs = match backend.wait_observe() {
            Ok(o) => o,
            Err(e) => {
                eprintln!("impulse-response-rust: wait_observe failed: {e:?}");
                return ExitCode::FAILURE;
            }
        };
        t_known_s = obs.t_recv_ns as f64 * 1e-9;

        // TRUTH, for METRICS ONLY -- never fed to the controller (DR-OBS-1:
        // hal carries raw IMU only). Same formula as
        // sim/scenarios/impulse_response.py::frame_pitch_rad, against the
        // same underlying mjData::xmat array.
        let xmat = backend.truth_frame_xmat();
        let pitch_rad = xmat[2].atan2(xmat[8]);
        let pitch_deg = pitch_rad.to_degrees();

        // Controller: raw IMU -> estimate -> regulate -> envelope, the same
        // chain control-ffi::ob_controller_update runs, reached here through
        // hal instead of the C ABI.
        let sample = obs.newest_imu().copied().unwrap_or(ImuSample::ZERO);
        let aiding = feedforward.predict(last_amps);
        let attitude = estimator.update(std::slice::from_ref(&sample), aiding);
        let proposed = regulator.update(attitude.pitch_rad, attitude.pitch_rate_rad_s, 0.0);
        let (bounded_cmd, _envelope_sat) =
            envelope.apply(Command::MotorCurrent { amps: proposed }, Faults::NONE);
        last_amps = match bounded_cmd {
            Command::MotorCurrent { amps } => amps,
            Command::RemoteSpeed { .. } => 0.0,
        };

        if let Err(e) = backend.apply(&bounded_cmd) {
            eprintln!("impulse-response-rust: apply failed: {e:?}");
            return ExitCode::FAILURE;
        }

        if pitch_deg.abs() > peak_abs_pitch_deg {
            peak_abs_pitch_deg = pitch_deg.abs();
            t_peak_s = t_known_s;
        }
        if t_known_s >= T0_S + DURATION_S && pitch_deg.abs() > SETTLE_BAND_DEG {
            last_outside_band_s = Some(t_known_s);
        }
        final_pitch_deg = pitch_deg;
    }

    if let Err(e) = backend.close() {
        eprintln!("impulse-response-rust: backend close failed: {e:?}");
        return ExitCode::FAILURE;
    }

    // Mirrors impulse_response.py's settle_time_s: None (never left the band
    // for good) if the last excursion is within one step of the run's end;
    // 0.0 if it never left at all; otherwise seconds since the kick.
    let settle_time_s = match last_outside_band_s {
        None => 0.0,
        Some(last) if last >= SIM_SECONDS - dt_s - 1e-9 => -1.0,
        Some(last) => last - T0_S,
    };

    let contents =
        format!("{peak_abs_pitch_deg}\n{t_peak_s}\n{settle_time_s}\n{final_pitch_deg}\n");
    if let Err(e) = std::fs::write(out_path, contents) {
        eprintln!("impulse-response-rust: could not write output file '{out_path}': {e}");
        return ExitCode::FAILURE;
    }

    ExitCode::SUCCESS
}
