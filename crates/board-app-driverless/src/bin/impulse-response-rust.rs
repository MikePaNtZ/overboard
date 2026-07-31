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
//! # Why `estimator_accel_aiding` mode 1 (wheel odometry), matching
//! `RustController()`'s own default
//!
//! An earlier revision of this module's doc claimed `hal::Observation`
//! "carries raw IMU and `motor_current_a` (DR-OBS-1) but no wheel-rate/ERPM
//! channel yet" and used mode 2 (command feedforward) instead. **That claim
//! was wrong** (issue #121): `Observation` has carried `erpm` since DR-OBS-1;
//! `sim-backend` simply left it at its default. Issue #121 populates `erpm`
//! from the plant's `wheel_hinge` joint velocity, so this harness now hosts
//! the SAME mode `RustController()` defaults to and `test_closed_loop.py`
//! gates on, rather than a mode chosen only because a field was empty.
//! `tests/test_rust_hosted_impulse_response.py` configures its Python-hosted
//! comparator identically (same mode, same gain, same gate).
//!
//! # Output
//!
//! Four ASCII floats, one per line, in this exact order:
//! `peak_abs_pitch_deg`, `t_peak_s`, `settle_time_s` (`-1.0` means "never
//! settled", mirroring the Python side's `None`), `final_pitch_deg`.

use board_types::{Command, Faults, ImuSample, Params, DEFAULT_R_EFF_M, RAD_S_PER_ERPM};
use control_core::{ComplementaryFilter, Estimator, PitchRegulator, WheelAccelEstimator};
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
// wheel_accel_tau_s match control-ffi::ob_controller_new's own defaults /
// RustController()'s own defaults (issue #121: mode 1, wheel odometry, is
// now the mode both hosts run -- see the module doc comment).
//
// Torque-denominated (issue #137): 80 A/rad, 11 A/(rad/s) at kt = 0.7 N*m/A
// -- the same numbers this binary shipped before #137, re-denominated. `KT`
// is the one place that conversion happens, applied once below right before
// `Command::MotorCurrent` is built, mirroring `control-ffi`'s boundary.
const KP_NM_PER_RAD: f32 = 56.0;
const KD_NM_PER_RAD_S: f32 = 7.7;
const KT_NM_PER_A: f32 = 0.7;
const MAX_CURRENT_A: f32 = 40.0;
const ESTIMATOR_TAU_S: f32 = 1.0;
/// `RustController()`'s own default (`sim/scenarios/rust_controller.py`,
/// `wheel_accel_tau_s`) -- kept identical here so both hosts filter the wheel
/// speed derivative with the same time constant, not two independently
/// chosen ones.
const WHEEL_ACCEL_TAU_S: f32 = 0.05;

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().collect();
    let [_, out_path] = args.as_slice() else {
        eprintln!("usage: impulse-response-rust <out.txt>");
        return ExitCode::FAILURE;
    };

    let params = Params {
        kp_nm_per_rad: KP_NM_PER_RAD,
        kd_nm_per_rad_s: KD_NM_PER_RAD_S,
        kt_nm_per_a: KT_NM_PER_A,
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

    let regulator = PitchRegulator::new(KP_NM_PER_RAD, KD_NM_PER_RAD_S);
    let mut estimator = ComplementaryFilter::with_trust_band(ESTIMATOR_TAU_S, 0.0);
    let mut wheel_accel = WheelAccelEstimator::new(WHEEL_ACCEL_TAU_S);

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
        // hal instead of the C ABI. Aiding (mode 1, wheel odometry, issue
        // #121): `obs.erpm` round-tripped back through the same ICD-sourced
        // ratio `sim-backend` used to produce it, into ground speed via
        // `r_eff_m`, then differentiated -- the same
        // `control-ffi::ob_controller_update` chain, `(None,
        // wheel_accel.as_mut())` branch, just reached natively.
        let sample = obs.newest_imu().copied().unwrap_or(ImuSample::ZERO);
        let wheel_rate_rad_s = obs.erpm * RAD_S_PER_ERPM;
        let aiding = wheel_accel.update(wheel_rate_rad_s * DEFAULT_R_EFF_M, dt_s as f32);
        let attitude = estimator.update(std::slice::from_ref(&sample), aiding);
        let proposed_torque_nm =
            regulator.update(attitude.pitch_rad, attitude.pitch_rate_rad_s, 0.0);
        // The single kt division -- the actuation boundary (issue #137).
        let proposed_amps = proposed_torque_nm / KT_NM_PER_A;
        let (bounded_cmd, _envelope_sat) =
            envelope.apply(Command::MotorCurrent { amps: proposed_amps }, Faults::NONE);

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
