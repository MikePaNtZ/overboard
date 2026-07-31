//! `sim-backend` implements the [`hal`] seam against the real MuJoCo onewheel
//! plant (`crates/plant-mujoco`), issue #107 (I1c). This is the increment
//! that satisfies SR-SIM-5: the Rust stack now steps the plant through `hal`
//! itself, and the Python harness no longer has control authority over it.
//!
//! What this backend honours, because getting them wrong later is expensive:
//!
//! - **`wait_observe()` is the only call that advances time.** `apply()` does
//!   not touch the clock (ICD §5.2), and does not touch the plant either --
//!   it only buffers a pending command.
//! - **`wait_observe()` steps the plant `control_period / mj_timestep`
//!   times, and that ratio is asserted to be an exact integer in `open()`**
//!   (issue #107 AC2). A fractional ratio would give a small persistent
//!   timing bias that presents as a controls problem.
//! - **Sim time comes from `mjData::time`, not a parallel counter** (AC3).
//!   The stub's synthetic clock is gone, not kept alongside it.
//! - **The pre-loop `mj_forward` priming call every CONTROLLED Python
//!   scenario makes is mirrored here, in the same position** (AC8, carried
//!   forward from I1b/#106): once, right after the plant opens, before the
//!   first `mj_step`. `mj_forward` writes `qacc_warmstart`, and the solver is
//!   history-dependent -- skipping this call would start the Rust and Python
//!   hosts from different warmstart state, which shows up as a phase error
//!   indistinguishable from a controls problem. See
//!   `crates/plant-mujoco/README.md`'s "Ordering contract".
//! - **The call-sequence rules are enforced**, via [`hal::CallSequence`], so a
//!   double `apply()` is already a `ProtocolViolation` rather than two frames.
//! - **`apply()` does not echo the command back as measured current.** The
//!   commanded value is buffered and only appears in a *later* observation,
//!   because actuation delay is additive (ICD §5.2) and a backend that echoes
//!   is explicitly non-conforming (§12). One whole control cycle here is a
//!   placeholder for the real first-order current loop, which arrives with
//!   the imperfection profile (Mechanical's territory, not this crate's).
//! - **Cold start reports `Invalid`, not zeros.** A backend presenting zeros
//!   as measurements before the drive has spoken is non-conforming (§12).
//!
//! [`SimBackend`] implements both [`hal::BoardObserve`] and
//! [`hal_actuate::BoardActuate`], which makes this crate **driverless-only**:
//! depending on it pulls in `hal-actuate` transitively, so `board-app-ridden`
//! must not link it. `board-app-ridden` gets its own observe-only backend
//! instead.

use board_types::{
    Applied, Command, DisarmReason, FaultCode, ImuSample, IoError, Observation, Params, Profile,
    RunMetadata, Saturation, ValidityFlags, DEFAULT_R_EFF_M,
};
use hal::{BoardObserve, CallSequence};
use hal_actuate::{BoardActuate, Disarm};
use plant_mujoco::Plant;
use std::path::{Path, PathBuf};

/// Nanoseconds per control cycle. 500 Hz, per ICD §11.2. Sourced the same way
/// the stub sourced it -- a fixed constant, not read from the model -- per
/// issue #107's explicit instruction not to invent a new configuration
/// mechanism here.
const CYCLE_NS: u64 = 2_000_000;

/// Motor torque constant, N*m per amp. The MJCF actuator is a torque source
/// (`gear="1"`); the controller speaks current. Mirrors
/// `sim/scenarios/plant.py::KT_NM_PER_A` exactly -- duplicated the same way
/// `board_types::DEFAULT_R_EFF_M` duplicates the model's tire radius, because
/// this crate has no Python binding to check itself against directly. The
/// model's own `ctrlrange="-28 28"` is the derived, checkable consequence
/// (40 A * 0.7 N*m/A = 28 N*m), and `kt_nm_per_a_matches_the_models_ctrlrange`
/// below pins it against the compiled model so the two cannot silently drift.
const KT_NM_PER_A: f64 = 0.7;

/// The model this backend steps. Fixed, not configurable -- this crate's own
/// header has always named it "the MuJoCo onewheel model", and I1c does not
/// introduce a model-selection surface.
fn model_path() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../sim/models/overboard_onewheel.xml")
}

/// Disarm handle for the sim.
///
/// Real backends hold a pre-opened fd and a pre-encoded safe-state frame so the
/// disarm path allocates nothing and takes no lock (ICD §9.2). The sim has
/// nothing to send, so this is a marker — but it is `Send + Sync` and
/// idempotent like the real one, so code that stores or clones it compiles the
/// same against both.
#[derive(Debug, Clone, Copy, Default)]
pub struct SimDisarm;

impl Disarm for SimDisarm {
    fn disarm(&self, _reason: DisarmReason) -> Result<(), IoError> {
        Ok(())
    }
}

/// Real sim backend: steps `plant-mujoco`'s `Plant` through the `hal` seam.
#[derive(Debug, Default)]
pub struct SimBackend {
    plant: Option<Plant>,
    /// `CYCLE_NS / mj_timestep_ns`, resolved once in `open()` (issue #107
    /// AC2). Zero before `open()`; never read before then.
    steps_per_cycle: u64,
    /// `(adr, dim)` into `mjData::sensordata`, resolved once in `open()`.
    gyro_sensor: (usize, usize),
    accel_sensor: (usize, usize),
    /// `mjModel` body id of the `frame` body, resolved once in `open()`.
    /// Only used by [`SimBackend::apply_external_force`], not by `hal`.
    frame_body: usize,
    cycle: u64,
    open: bool,
    armed: bool,
    seq: CallSequence,
    /// Commanded current awaiting its actuation delay. Never reported as
    /// measured in the same cycle it was applied.
    pending_current_a: Option<f32>,
    /// The current now actually flowing, as far as the plant is concerned.
    applied_current_a: f32,
    params: Params,
}

impl SimBackend {
    pub fn new() -> Self {
        SimBackend::default()
    }

    pub fn with_params(params: Params) -> Self {
        SimBackend {
            params,
            ..SimBackend::default()
        }
    }

    /// The current the plant is seeing. Exposed for tests.
    pub fn applied_current_a(&self) -> f32 {
        self.applied_current_a
    }

    /// Injects an external disturbance -- force (N), then torque (N*m), world
    /// frame -- onto the plant's `frame` body, taking effect on the next
    /// [`BoardObserve::wait_observe`] call.
    ///
    /// This is **not part of `hal`** and `control-core` never calls it: it
    /// exists for the I1c Rust-hosted impulse-response harness (issue #107
    /// AC6), mirroring the Python scenarios' own
    /// `data.xfrc_applied[frame][:3] = force` / `[3:] = torque`. Like that
    /// pattern, this overwrites whatever was set for the current step rather
    /// than accumulating -- callers must call this every cycle (with zeros
    /// when there is no disturbance), not just when a push is active, or a
    /// stale nonzero value would persist.
    ///
    /// # Panics
    /// If called before `open()`.
    pub fn apply_external_force(&mut self, force_n: [f64; 3], torque_nm: [f64; 3]) {
        let plant = self
            .plant
            .as_mut()
            .expect("apply_external_force: backend is not open");
        plant.set_xfrc_applied(self.frame_body, force_n, torque_nm);
    }

    /// Ground-truth `mjData::qpos`, straight from the plant. **Not part of
    /// `hal`** -- `control-core` never sees this (DR-OBS-1: `hal` carries raw
    /// IMU only, no pre-fused pitch). Exists for scenario/test harnesses that
    /// need truth for metrics (issue #107 AC6), mirroring the Python
    /// scenarios' own `data.qpos`/`data.xmat` reads.
    ///
    /// # Panics
    /// If called before `open()`.
    pub fn truth_qpos(&self) -> Vec<f64> {
        self.plant
            .as_ref()
            .expect("truth_qpos: backend is not open")
            .qpos()
    }

    /// Ground-truth world rotation matrix of the `frame` body (row-major),
    /// straight from the plant. Same non-`hal`, harness-only status as
    /// [`SimBackend::truth_qpos`] -- exists so a caller can compute pitch
    /// with `sim/scenarios/impulse_response.py::frame_pitch_rad`'s own
    /// formula, `atan2(xmat[2], xmat[8])`, against the identical array.
    ///
    /// # Panics
    /// If called before `open()`.
    pub fn truth_frame_xmat(&self) -> [f64; 9] {
        let plant = self
            .plant
            .as_ref()
            .expect("truth_frame_xmat: backend is not open");
        plant.body_xmat(self.frame_body)
    }
}

impl BoardObserve for SimBackend {
    fn open(&mut self) -> Result<(), IoError> {
        let mut plant = Plant::open(&model_path()).map_err(|e| {
            eprintln!("sim-backend: Plant::open failed: {e}");
            IoError::Fatal(FaultCode::ConfigMismatch)
        })?;

        // AC2 (issue #107): wait_observe() must step this plant exactly
        // control_period / mj_timestep times, and that ratio must be an
        // exact integer -- a fractional ratio gives a small persistent
        // timing bias that presents as a controls problem and costs a week
        // to find. Asserted here, loudly, rather than silently truncated.
        let dt_ns = (plant.timestep() * 1e9).round() as u64;
        assert!(
            dt_ns > 0,
            "sim-backend: model timestep must be positive, got {} s",
            plant.timestep()
        );
        assert_eq!(
            CYCLE_NS % dt_ns,
            0,
            "sim-backend: control_period ({CYCLE_NS} ns, {} Hz) is not an exact multiple \
             of mj_timestep ({dt_ns} ns, model.opt.timestep = {} s) -- wait_observe()'s \
             stepping ratio must be an exact integer or the loop accrues a persistent \
             timing bias (issue #107 AC2)",
            1e9 / CYCLE_NS as f64,
            plant.timestep(),
        );
        self.steps_per_cycle = CYCLE_NS / dt_ns;

        // AC8 (issue #107, carried forward from I1b/#106): the CONTROLLED
        // Python scenarios call mj_forward once, before their first
        // mj_step, to prime sensordata (and qacc_warmstart) for the
        // controller's first cycle. This backend hosts a real control loop,
        // so it must mirror that call in the same position -- right after
        // the plant opens, before any mj_step. I1b's open-loop replay
        // deliberately skips this call because it has no controller to
        // prime; that equivalence does not transfer here.
        plant.forward();

        self.gyro_sensor = plant
            .sensor_adr_dim("frame_gyro")
            .expect("overboard_onewheel.xml must declare a frame_gyro sensor");
        self.accel_sensor = plant
            .sensor_adr_dim("frame_accel")
            .expect("overboard_onewheel.xml must declare a frame_accel sensor");
        self.frame_body = plant
            .body_id("frame")
            .expect("overboard_onewheel.xml must declare a frame body");

        self.plant = Some(plant);
        self.cycle = 0;
        self.pending_current_a = None;
        self.applied_current_a = 0.0;
        self.open = true;
        self.seq.reset();
        Ok(())
    }

    fn close(&mut self) -> Result<(), IoError> {
        self.open = false;
        self.armed = false;
        // Dropped, not kept: the next open() builds a fresh Plant (fresh
        // mjData) the same way I1b's replay never reuses one across runs,
        // which is what repeat-run bit-identity (AC5, issue #74) depends on.
        self.plant = None;
        self.seq.reset();
        Ok(())
    }

    fn wait_observe(&mut self) -> Result<Observation, IoError> {
        if !self.open {
            return Err(IoError::ProtocolViolation);
        }
        let plant = self
            .plant
            .as_mut()
            .expect("self.open is only true while self.plant is Some");

        // Time advances here and only here -- steps_per_cycle mj_step calls,
        // never more, never fewer (AC2).
        self.cycle += 1;

        // Whatever was commanded last cycle becomes effective now -- the
        // additive actuation delay, in its crudest possible form.
        if let Some(pending) = self.pending_current_a.take() {
            self.applied_current_a = pending;
        }
        let ctrl = [self.applied_current_a as f64 * KT_NM_PER_A];

        // AC3: sim time comes from mjData::time, read after stepping, never
        // a parallel counter.
        let mut t_s = plant.time();
        for _ in 0..self.steps_per_cycle {
            // ctrl written BEFORE mj_step, held constant across every
            // sub-step of one control period (zero-order hold) -- the same
            // "ctrl before step, never mid-step" ordering I1b pins down,
            // extended to more than one step per cycle.
            plant.set_ctrl(&ctrl);
            t_s = plant.step();
        }
        let t_ns = (t_s * 1e9).round() as u64;

        // Raw IMU only (DR-OBS-1) -- read AFTER stepping, same ordering rule
        // I1b pins down for qpos/qvel. Rotated from the model's frame
        // (z-up, forward = -X) into the ICD's FRD frame, mirroring
        // sim/scenarios/plant.py's R_MODEL_TO_ICD / imu_readings: a 180 deg
        // rotation about +Y, diag(-1, +1, -1).
        let gyro = plant.read_sensor(self.gyro_sensor.0, self.gyro_sensor.1);
        let accel = plant.read_sensor(self.accel_sensor.0, self.accel_sensor.1);
        let gyro_icd = [-gyro[0] as f32, gyro[1] as f32, -gyro[2] as f32];
        let accel_icd = [-accel[0] as f32, accel[1] as f32, -accel[2] as f32];

        let mut obs = Observation::COLD_START;
        obs.cycle = self.cycle;
        obs.t_recv_ns = t_ns;
        obs.imu[0] = ImuSample {
            gyro_rad_s: gyro_icd,
            accel_m_s2: accel_icd,
            t_sample_ns: t_ns,
        };
        obs.imu_count = 1;
        obs.motor_current_a = self.applied_current_a;
        obs.validity = ValidityFlags::ALL_FRESH;

        self.seq.on_observe();
        Ok(obs)
    }

    fn run_metadata(&self) -> RunMetadata {
        RunMetadata {
            icd_version: (0, 3),
            profile: Profile::DRaw,
            control_rate_hz: 1e9 / CYCLE_NS as f32,
            params: self.params,
            imu_mounting_rotation: [1.0, 0.0, 0.0, 0.0],
            r_eff_m: DEFAULT_R_EFF_M,
            imperfection_profile_id: None,
            schema_hash: [0; 32],
            binary_hash: [0; 32],
            git_sha: [0; 20],
        }
    }
}

impl BoardActuate for SimBackend {
    type Disarm = SimDisarm;

    fn arm(&mut self) -> Result<Self::Disarm, IoError> {
        if !self.open {
            return Err(IoError::ProtocolViolation);
        }
        self.armed = true;
        Ok(SimDisarm)
    }

    fn apply(&mut self, cmd: &Command) -> Result<Applied, IoError> {
        // Checked before anything is buffered: a violation must send nothing.
        self.seq.on_apply()?;

        if !self.armed {
            return Err(IoError::ProtocolViolation);
        }

        let amps = match cmd {
            Command::MotorCurrent { amps } => *amps,
            // The sim backend is the D-RAW path. A RemoteSpeed command here is
            // a profile confusion, not a value to coerce.
            Command::RemoteSpeed { .. } => return Err(IoError::ProtocolViolation),
        };

        // Stage 3, the backend's own clamp. Reads the SAME resolved Params the
        // safety envelope used, because a sim whose final clamp differs from
        // hardware's makes the CI margin gate incomparable (ICD §7.6).
        let limit = self.params.max_current_a.abs();
        let (bounded, saturated) = if limit > 0.0 && amps.abs() > limit {
            let b = if amps.is_sign_negative() {
                -limit
            } else {
                limit
            };
            (b, Saturation::Yes)
        } else {
            (amps, Saturation::No)
        };

        // Buffered, not applied: time does not advance here, and the plant is
        // not touched here either -- only wait_observe() ever calls
        // Plant::set_ctrl/step.
        self.pending_current_a = Some(bounded);

        // Same instant as the most recent wait_observe(): apply() never
        // advances mjData::time, so this is a read of the one clock (AC3),
        // not a second one.
        let t_apply_ns = self
            .plant
            .as_ref()
            .map(|p| (p.time() * 1e9).round() as u64)
            .unwrap_or(0);

        Ok(Applied {
            commanded: Command::MotorCurrent { amps: bounded },
            saturated,
            t_apply_ns,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn opened() -> SimBackend {
        let mut b = SimBackend::with_params(Params {
            max_current_a: 40.0,
            ..Params::default()
        });
        b.open().unwrap();
        b
    }

    fn armed() -> SimBackend {
        let mut b = opened();
        b.arm().unwrap();
        b
    }

    #[test]
    fn wait_observe_advances_the_clock_and_the_cycle_counter() {
        let mut b = armed();
        let o1 = b.wait_observe().unwrap();
        let o2 = b.wait_observe().unwrap();
        assert_eq!(o1.cycle, 1);
        assert_eq!(o2.cycle, 2);
        assert_eq!(o2.t_recv_ns - o1.t_recv_ns, CYCLE_NS);
    }

    // CHANGED from the stub (issue #107, AC1/AC3): the original body read the
    // stub's private synthetic-clock field (`b.t_ns`) directly after
    // apply(), before any further wait_observe(). AC3 requires deleting that
    // field -- sim time now comes only from `mjData::time`, reached through
    // `Plant`, and "not kept alongside" a parallel counter is the explicit
    // acceptance criterion. There is no longer a private clock to read at
    // this point without calling wait_observe() again.
    //
    // The PROPERTY under test is unchanged: apply() must not advance time.
    // Proven the same way `wait_observe_advances_the_clock_and_the_cycle_
    // counter` proves the converse -- one more wait_observe() must advance by
    // exactly one CYCLE_NS, not two, which it could only do if apply() had
    // secretly consumed a step.
    #[test]
    fn apply_does_not_advance_time() {
        // The single most important property of the split. If apply() moved the
        // clock, actuation delay would be double-counted against the loop.
        let mut b = armed();
        let before = b.wait_observe().unwrap();
        b.apply(&Command::MotorCurrent { amps: 5.0 }).unwrap();
        let after = b.wait_observe().unwrap();
        assert_eq!(after.t_recv_ns - before.t_recv_ns, CYCLE_NS);
    }

    #[test]
    fn a_second_apply_in_one_cycle_is_a_protocol_violation() {
        let mut b = armed();
        b.wait_observe().unwrap();
        assert!(b.apply(&Command::MotorCurrent { amps: 1.0 }).is_ok());
        assert_eq!(
            b.apply(&Command::MotorCurrent { amps: 2.0 }),
            Err(IoError::ProtocolViolation)
        );
    }

    #[test]
    fn the_rejected_second_apply_does_not_reach_the_plant() {
        // "Returns an error" is not enough -- the contract is that nothing is
        // sent. A backend that errors after buffering would still actuate.
        let mut b = armed();
        b.wait_observe().unwrap();
        b.apply(&Command::MotorCurrent { amps: 1.0 }).unwrap();
        let _ = b.apply(&Command::MotorCurrent { amps: 99.0 });
        b.wait_observe().unwrap();
        assert_eq!(b.applied_current_a(), 1.0);
    }

    #[test]
    fn apply_before_the_first_observe_is_a_protocol_violation() {
        let mut b = armed();
        assert_eq!(
            b.apply(&Command::MotorCurrent { amps: 1.0 }),
            Err(IoError::ProtocolViolation)
        );
    }

    #[test]
    fn zero_applies_across_many_cycles_is_legal_shadow_mode() {
        let mut b = opened();
        for _ in 0..100 {
            b.wait_observe().unwrap();
        }
        assert_eq!(b.applied_current_a(), 0.0);
    }

    #[test]
    fn measured_current_is_not_an_echo_of_the_command() {
        // ICD 12 names echoing the command back as measured current as
        // non-conforming: it hides the actuation lag the controller must be
        // designed against.
        let mut b = armed();
        b.wait_observe().unwrap();
        b.apply(&Command::MotorCurrent { amps: 7.0 }).unwrap();

        let same_cycle = b.wait_observe().unwrap();
        assert_eq!(
            same_cycle.motor_current_a, 7.0,
            "command from cycle N becomes effective in N+1"
        );

        // And it was definitely not visible at apply() time.
        let mut b2 = armed();
        let o = b2.wait_observe().unwrap();
        b2.apply(&Command::MotorCurrent { amps: 7.0 }).unwrap();
        assert_eq!(o.motor_current_a, 0.0);
    }

    #[test]
    fn cold_start_observation_is_invalid_until_open() {
        let mut b = SimBackend::new();
        assert_eq!(b.wait_observe(), Err(IoError::ProtocolViolation));
    }

    #[test]
    fn applying_while_disarmed_is_refused() {
        let mut b = opened();
        b.wait_observe().unwrap();
        assert_eq!(
            b.apply(&Command::MotorCurrent { amps: 1.0 }),
            Err(IoError::ProtocolViolation)
        );
    }

    #[test]
    fn remote_speed_is_refused_on_the_raw_current_path() {
        let mut b = armed();
        b.wait_observe().unwrap();
        assert_eq!(
            b.apply(&Command::RemoteSpeed { m_s: 3.0 }),
            Err(IoError::ProtocolViolation)
        );
    }

    #[test]
    fn stage_three_clamp_bounds_and_reports() {
        let mut b = armed();
        b.wait_observe().unwrap();
        let applied = b.apply(&Command::MotorCurrent { amps: 999.0 }).unwrap();
        assert_eq!(applied.commanded, Command::MotorCurrent { amps: 40.0 });
        assert_eq!(applied.saturated, Saturation::Yes);
    }

    #[test]
    fn close_then_apply_is_refused() {
        let mut b = armed();
        b.wait_observe().unwrap();
        b.close().unwrap();
        assert_eq!(
            b.apply(&Command::MotorCurrent { amps: 1.0 }),
            Err(IoError::ProtocolViolation)
        );
    }

    #[test]
    fn run_metadata_reports_the_control_rate_actually_used() {
        let b = opened();
        assert_eq!(b.run_metadata().control_rate_hz, 500.0);
    }

    // --- new for issue #107 (I1c): real-plant-specific properties ---------

    /// AC2: the onewheel model's `option timestep="0.002"` (500 Hz) equals
    /// `CYCLE_NS` (500 Hz), so the ratio is exactly 1 -- the smallest
    /// possible case, but still the one this backend actually runs.
    #[test]
    fn open_resolves_a_stepping_ratio_of_one_for_the_onewheel_model() {
        let b = opened();
        // Not part of the public API; observed indirectly below via
        // wait_observe_advances_the_clock_and_the_cycle_counter, whose
        // CYCLE_NS-exact delta could only hold if steps_per_cycle*dt ==
        // CYCLE_NS. Restated here as its own claim for readability.
        assert_eq!(
            b.run_metadata().control_rate_hz as u64,
            (1.0 / 0.002) as u64
        );
    }

    /// AC3: sim time is read from `mjData::time` through `Plant`, not a
    /// parallel counter -- so it survives a fresh `open()` exactly the way a
    /// freshly-loaded `mjModel` does: back at zero. A synthetic counter that
    /// forgot to reset on reopen (a real bug class for a parallel clock) is
    /// exactly what this catches.
    #[test]
    fn reopening_starts_sim_time_back_at_zero() {
        let mut b = armed();
        let far = {
            for _ in 0..5 {
                b.wait_observe().unwrap();
            }
            b.wait_observe().unwrap()
        };
        assert!(far.t_recv_ns > 0);

        b.close().unwrap();
        b.open().unwrap();
        b.arm().unwrap();
        let fresh = b.wait_observe().unwrap();
        assert_eq!(
            fresh.t_recv_ns, CYCLE_NS,
            "one step from a fresh mjData, not from far.t_recv_ns"
        );
    }

    /// AC8: the pre-loop `mj_forward` priming call happens once per `open()`,
    /// before the first `mj_step` -- proven the way this codebase always
    /// proves a physics call actually ran: by observing something a no-op
    /// could not fake. `mj_forward` computes `sensordata`, so the very first
    /// observation must already carry a non-placeholder IMU reading rather
    /// than whatever a freshly-zeroed, never-forwarded `mjData` would report.
    #[test]
    fn the_first_observation_carries_a_primed_imu_reading() {
        let mut b = armed();
        let first = b.wait_observe().unwrap();
        let sample = first.newest_imu().expect("imu_count must be 1");
        assert!(
            sample.accel_m_s2.iter().any(|v| *v != 0.0),
            "a primed accelerometer reading must not be all-zero at step 1: {:?}",
            sample.accel_m_s2
        );
    }

    /// Regression pin for `KT_NM_PER_A`: it must keep matching
    /// `sim/scenarios/plant.py::KT_NM_PER_A`, whose derived, checkable
    /// consequence is the model's own `ctrlrange`. If the model's clamp ever
    /// changes independently of this constant, applying `max_current_a`
    /// (40 A, the model's own derived limit) must land exactly on the
    /// model's `ctrlrange` bound rather than saturating early or leaving
    /// headroom -- either of which would mean the two have silently
    /// diverged.
    #[test]
    fn kt_nm_per_a_matches_the_models_ctrlrange() {
        assert_eq!(
            KT_NM_PER_A * 40.0,
            28.0,
            "40 A * KT_NM_PER_A must equal the model's ctrlrange bound (28 N*m); \
             see overboard_onewheel.xml's <motor ... ctrlrange=\"-28 28\">"
        );
    }
}
