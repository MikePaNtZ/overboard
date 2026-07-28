//! `sim-backend` implements the [`hal`] seam against the MuJoCo onewheel model.
//!
//! **STUB — it does not step MuJoCo yet.** It advances a synthetic clock and
//! returns synthetic observations, which is enough to exercise the ICD §5.2
//! call-sequence contract end to end but is *not* a plant. Wiring `mj_step`
//! against `sim/models/overboard_onewheel.xml` through FFI is the next
//! increment (I1), and until it lands SR-SIM-5 stays unmet.
//!
//! What this stub does honour, because getting them wrong later is expensive:
//!
//! - **`wait_observe()` is the only call that advances time.** `apply()` does
//!   not touch the clock (ICD §5.2).
//! - **The call-sequence rules are enforced**, via [`hal::CallSequence`], so a
//!   double `apply()` is already a `ProtocolViolation` rather than two frames.
//! - **`apply()` does not echo the command back as measured current.** The
//!   commanded value is buffered and only appears in a *later* observation,
//!   because actuation delay is additive (ICD §5.2) and a backend that echoes
//!   is explicitly non-conforming (§12). One cycle here is a placeholder; the
//!   real first-order current loop and its ~1 ms lag arrive with the
//!   imperfection profile.
//! - **Cold start reports `Invalid`, not zeros.** A backend presenting zeros as
//!   measurements before the drive has spoken is non-conforming (§12).
//!
//! [`SimBackend`] implements both [`hal::BoardObserve`] and
//! [`hal_actuate::BoardActuate`], which makes this crate **driverless-only**:
//! depending on it pulls in `hal-actuate` transitively, so `board-app-ridden`
//! must not link it. `board-app-ridden` gets its own observe-only backend
//! instead.

use board_types::{
    Applied, Command, DisarmReason, ImuSample, IoError, Observation, Params, Profile, RunMetadata,
    Saturation, ValidityFlags,
};
use hal::{BoardObserve, CallSequence};
use hal_actuate::{BoardActuate, Disarm};

/// Nanoseconds per control cycle. 500 Hz, per ICD §11.2.
const CYCLE_NS: u64 = 2_000_000;

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

/// Stub sim backend: synthetic observations, no MuJoCo FFI yet.
#[derive(Debug, Default)]
pub struct SimBackend {
    cycle: u64,
    t_ns: u64,
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

    /// The current the (eventual) plant is seeing. Exposed for tests.
    pub fn applied_current_a(&self) -> f32 {
        self.applied_current_a
    }
}

impl BoardObserve for SimBackend {
    fn open(&mut self) -> Result<(), IoError> {
        self.open = true;
        self.seq.reset();
        Ok(())
    }

    fn close(&mut self) -> Result<(), IoError> {
        self.open = false;
        self.armed = false;
        self.seq.reset();
        Ok(())
    }

    fn wait_observe(&mut self) -> Result<Observation, IoError> {
        if !self.open {
            return Err(IoError::ProtocolViolation);
        }

        // Time advances here and only here.
        self.cycle += 1;
        self.t_ns += CYCLE_NS;

        // Whatever was commanded last cycle becomes effective now -- the
        // additive actuation delay, in its crudest possible form.
        if let Some(pending) = self.pending_current_a.take() {
            self.applied_current_a = pending;
        }

        let mut obs = Observation::COLD_START;
        obs.cycle = self.cycle;
        obs.t_recv_ns = self.t_ns;
        obs.imu[0] = ImuSample {
            gyro_rad_s: [0.0; 3],
            accel_m_s2: [0.0, 0.0, -9.81],
            t_sample_ns: self.t_ns,
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
            r_eff_m: 0.14605,
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

        // Buffered, not applied: time does not advance here.
        self.pending_current_a = Some(bounded);

        Ok(Applied {
            commanded: Command::MotorCurrent { amps: bounded },
            saturated,
            t_apply_ns: self.t_ns,
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

    #[test]
    fn apply_does_not_advance_time() {
        // The single most important property of the split. If apply() moved the
        // clock, actuation delay would be double-counted against the loop.
        let mut b = armed();
        let before = b.wait_observe().unwrap();
        b.apply(&Command::MotorCurrent { amps: 5.0 }).unwrap();
        let t_after_apply = b.t_ns;
        assert_eq!(t_after_apply, before.t_recv_ns);
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
}
