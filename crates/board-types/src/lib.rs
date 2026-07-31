//! Shared plain data for the Overboard control stack.
//!
//! Kept `no_std`-friendly (no allocation, no `std::time`, no clock reads) so it
//! can be shared with a bare-metal or PREEMPT_RT-hosted binary. Timing is
//! always passed in explicitly — nothing in this crate reads a clock.
//!
//! Per BoardIo ICD §6.1 these types are **always compiled**, in every profile.
//! Only the wire encoders and the `BoardActuate` implementations are excluded
//! from the ridden build, which is what lets shadow mode compile at all (§6.6).
//!
//! Units, frames and signs are the ICD's, not this crate's: body frame is FRD
//! (x forward, y right, z down), SI throughout, `f32` signals and `u64`
//! nanosecond time. **Field names carry their units** — `gyro_rad_s`,
//! `motor_current_a` — which is deliberate: the ICD calls a sign or unit error
//! across this seam the most dangerous defect class in the system, and a name
//! that states its unit is harder to misread than a comment that does.
#![no_std]

use serde::{Deserialize, Serialize};

/// Loaded rolling radius, metres — the FFI-boundary / ridden-mode default for
/// [`RunMetadata::r_eff_m`].
///
/// Must equal the sim model's compiled `wheel_geom` radius
/// (`sim/models/overboard_onewheel.xml`) and the Python-side
/// `sim.scenarios.rust_controller.DEFAULT_R_EFF_M`, which is checked directly
/// against the compiled model by `tests/test_r_eff_matches_model.py`. This
/// constant is the single Rust-side source — every `crates/` site that needs
/// a default `r_eff_m` imports it rather than hand-copying the literal
/// (issue #89; a stale `0.14605` m hand-copy, physically impossible as a
/// *loaded* radius larger than the tyre's own unloaded geometry, is what
/// #76/#89 replaced it for).
///
/// This is a sim-consistency placeholder, not a bench measurement — Stage-0's
/// eventual bench-measured loaded rolling radius (ICD §10.5) supersedes it.
pub const DEFAULT_R_EFF_M: f32 = 0.1454;

/// Mechanical wheel rate, rad/s, per 1 ERPM. ICD §10.5: "1 ERPM = 6.98e-3
/// rad/s" — the same ratio `sim/scenarios/imperfections.py` names
/// `wheel_rate_quantum_rad_s` (the size of one ERPM step in the wheel-rate
/// domain). The single Rust-side source for that conversion, so
/// `erpm <-> wheel_rate_rad_s` round-trips read the same number everywhere
/// rather than each site hand-copying the literal (issue #121).
///
/// **Not derived from an independently-known pole-pair count.** VESC ERPM is
/// electrical RPM (`mechanical_rpm * pole_pairs`), so this ratio implicitly
/// encodes the motor's pole-pair count — back-solving `(2*pi/60) / 0.00698`
/// lands close to 15, a plausible pole-pair count for a hoverboard-class hub
/// motor, but **that back-derivation has not been confirmed against the
/// actual motor** and is not asserted as fact here. The ICD's ratio is used
/// directly instead of an independently guessed pole-pair count, per the
/// project rule against fabricating a protocol constant that cannot be
/// verified.
pub const RAD_S_PER_ERPM: f32 = 0.00698;

// ---------------------------------------------------------------------------
// Commands — ICD §7.5
// ---------------------------------------------------------------------------

/// A setpoint for the drive.
///
/// Two variants because there are two interface profiles, and they are not
/// interchangeable: `MotorCurrent` is the raw-current path the Rust stack owns
/// (D-RAW), while `RemoteSpeed` is a Refloat package command (D-REMOTE) that is
/// explicitly **non-balancing** (SR-CTRL-6).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum Command {
    /// D-RAW. Motor amps, board sign convention (ICD §10):
    /// **positive = forward wheel acceleration ⇒ nose pitches up.**
    MotorCurrent { amps: f32 },
    /// D-REMOTE. Forward ground-speed setpoint, SI. The backend owns the
    /// conversion to km/h and the `/127` wire quantisation.
    RemoteSpeed { m_s: f32 },
}

impl Command {
    /// The zero-effort raw-current command.
    pub const ZERO: Command = Command::MotorCurrent { amps: 0.0 };
}

/// Whether a clamp bound this cycle.
///
/// `Unknown` is not laziness: in D-REMOTE, Refloat's internal ±10 A clamp is
/// never reported back, so the honest answer is that we cannot tell — and
/// `control-core` must therefore not drive anti-windup from it (ICD §7.6).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Saturation {
    No,
    Yes,
    Unknown,
}

/// What was actually commanded, known synchronously from `apply()`.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Applied {
    /// The command after all clamping stages (ICD §7.6).
    pub commanded: Command,
    pub saturated: Saturation,
    pub t_apply_ns: u64,
}

// ---------------------------------------------------------------------------
// Observation — ICD §8
// ---------------------------------------------------------------------------

/// Maximum IMU samples carried in one [`Observation`].
///
/// Covers a 4 kHz ODR at a 500 Hz loop with margin (ICD §8.1).
pub const MAX_IMU_SAMPLES: usize = 8;

/// One inertial sample, raw. Not fused — see [`Observation`].
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ImuSample {
    /// Body-frame angular rate. `[1]` is ω_y: **positive = nose-up rate**.
    pub gyro_rad_s: [f32; 3],
    pub accel_m_s2: [f32; 3],
    /// When the quantity was true, **in the monotonic timebase**. The backend
    /// has already converted this from the IMU's own oscillator (ICD §5.3);
    /// `control-core` never learns the IMU has a separate clock.
    pub t_sample_ns: u64,
}

impl ImuSample {
    pub const ZERO: ImuSample = ImuSample {
        gyro_rad_s: [0.0; 3],
        accel_m_s2: [0.0; 3],
        t_sample_ns: 0,
    };
}

/// Freshness of a signal group.
///
/// `Invalid` includes "never received since `open()`" — the cold-start state,
/// which is a real window on hardware and must be modelled in sim (ICD §12).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Validity {
    Fresh,
    Stale,
    Invalid,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct ValidityFlags {
    pub imu: Validity,
    pub drive: Validity,
    pub slow: Validity,
}

impl ValidityFlags {
    pub const ALL_FRESH: ValidityFlags = ValidityFlags {
        imu: Validity::Fresh,
        drive: Validity::Fresh,
        slow: Validity::Fresh,
    };
    pub const ALL_INVALID: ValidityFlags = ValidityFlags {
        imu: Validity::Invalid,
        drive: Validity::Invalid,
        slow: Validity::Invalid,
    };
}

/// One control cycle's view of the world, as of `t_sample`.
///
/// **Raw inertial data only** (DR-OBS-1). No pre-fused pitch crosses this seam:
/// fusing in the backend would put the estimator on the far side of the
/// boundary and let sim and hardware fork silently at the most safety-relevant
/// point in the stack.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Observation {
    // ---- timing ----
    /// Increments by exactly 1 per `wait_observe()` return, regardless of
    /// missed instants (ICD §5.2).
    pub cycle: u64,
    /// When the Pi had the bytes.
    pub t_recv_ns: u64,
    /// Control instants skipped since the previous return. **Per-cycle, not
    /// cumulative** — 0 in the nominal case.
    pub missed_cycles: u32,
    pub overrun: bool,

    // ---- inertial: every sample since the last cycle, oldest first ----
    pub imu: [ImuSample; MAX_IMU_SAMPLES],
    pub imu_count: u8,
    /// Samples were dropped (FIFO or capacity). The backend keeps the
    /// **newest** and sets this; it never silently truncates.
    pub imu_overflow: bool,
    pub imu_temp_c: f32,

    // ---- drive (single wheel) ----
    pub erpm: f32,
    /// **Not** the frame age. At low speed a hall-edge-derived ERPM can be far
    /// staler than the frame carrying it (ICD §8.4).
    pub erpm_effective_age_ns: u32,
    pub motor_current_a: f32,
    pub duty: f32,
    /// Raw e-revs, unconverted (ICD §8.3).
    pub tacho_raw: i32,
    pub drive_frame_age_ns: u32,

    // ---- slow / housekeeping ----
    pub v_batt: f32,
    pub temp_fet_c: f32,
    pub temp_motor_c: f32,
    pub slow_age_ns: u32,

    // ---- health ----
    pub validity: ValidityFlags,
    pub fault_word: u32,
}

impl Observation {
    /// An all-zero observation with everything marked `Invalid`.
    ///
    /// This is the honest cold-start value: a backend that has not yet heard
    /// from the drive must not report zeros as if they were measurements.
    pub const COLD_START: Observation = Observation {
        cycle: 0,
        t_recv_ns: 0,
        missed_cycles: 0,
        overrun: false,
        imu: [ImuSample::ZERO; MAX_IMU_SAMPLES],
        imu_count: 0,
        imu_overflow: false,
        imu_temp_c: 0.0,
        erpm: 0.0,
        erpm_effective_age_ns: 0,
        motor_current_a: 0.0,
        duty: 0.0,
        tacho_raw: 0,
        drive_frame_age_ns: 0,
        v_batt: 0.0,
        temp_fet_c: 0.0,
        temp_motor_c: 0.0,
        slow_age_ns: 0,
        validity: ValidityFlags::ALL_INVALID,
        fault_word: 0,
    };

    /// The IMU samples actually present this cycle, oldest first.
    ///
    /// Always iterate this rather than indexing `imu` directly — the array is
    /// fixed-size and only the first `imu_count` entries are real. Consuming
    /// every sample is required (DR-CTRL-2); sub-sampling a ≥1 kHz IMU at a
    /// 500 Hz loop aliases the estimator's primary input.
    pub fn imu_samples(&self) -> &[ImuSample] {
        let n = (self.imu_count as usize).min(MAX_IMU_SAMPLES);
        &self.imu[..n]
    }

    /// The newest IMU sample, if any. `dt` is derived from consecutive values
    /// of this sample's timestamp across cycles — **never** assumed `1/rate`
    /// (ICD §5.2).
    pub fn newest_imu(&self) -> Option<&ImuSample> {
        self.imu_samples().last()
    }
}

// ---------------------------------------------------------------------------
// Faults and errors
// ---------------------------------------------------------------------------

/// Bitfield of fault conditions, as a plain `u32` newtype so it stays
/// `no_std`-friendly with no external bitflags dependency.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct Faults(pub u32);

impl Faults {
    pub const NONE: Faults = Faults(0);
    pub const OVER_CURRENT: Faults = Faults(1 << 0);
    pub const OVER_TILT: Faults = Faults(1 << 1);
    pub const SENSOR_STALE: Faults = Faults(1 << 2);
    pub const COMMS_TIMEOUT: Faults = Faults(1 << 3);

    pub fn contains(&self, other: Faults) -> bool {
        (self.0 & other.0) == other.0
    }

    pub fn insert(&mut self, other: Faults) {
        self.0 |= other.0;
    }

    pub fn is_clear(&self) -> bool {
        self.0 == 0
    }
}

/// A fault severe enough to end the run.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum FaultCode {
    BusDown,
    ImuLost,
    ConfigMismatch,
    Unknown(u32),
}

/// Why motion authority was given up.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum DisarmReason {
    Commanded,
    EnvelopeBreach,
    Deadline,
    StaleData,
    FaultReported,
    Panic,
    Drop,
}

/// Backend I/O failure — ICD §8.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum IoError {
    /// Recoverable: skip this cycle and keep running. Increments
    /// `missed_cycles`.
    Transient,
    /// Not recoverable. The caller **must** disarm.
    Fatal(FaultCode),
    /// A §5.2 call-sequence violation. The backend sent nothing.
    ProtocolViolation,
}

// ---------------------------------------------------------------------------
// Parameters and run metadata
// ---------------------------------------------------------------------------

/// Tunable controller and envelope parameters.
///
/// One resolved `Params` is read by all three clamp stages and is stamped into
/// the run metadata (ICD §7.6, §6.2). **The sim backend's final clamp must
/// equal the hardware backend's, from this same struct** — otherwise a CI
/// margin gate is not comparable to hardware, which breaks the whole "sim as a
/// margin instrument" bet.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Params {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    /// Envelope clamp on commanded current, amps.
    pub max_current_a: f32,
    /// Envelope trip on absolute pitch, radians.
    pub max_abs_pitch_rad: f32,
}

impl Default for Params {
    fn default() -> Self {
        Params {
            kp: 0.0,
            ki: 0.0,
            kd: 0.0,
            max_current_a: 0.0,
            max_abs_pitch_rad: 0.0,
        }
    }
}

/// Which interface profile a binary was built for.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Profile {
    /// Raw current, Rust owns motion authority.
    DRaw,
    /// Refloat setpoint, non-balancing.
    DRemote,
    /// Observe only — no motion authority exists in the binary.
    Observe,
}

/// Backend-owned run provenance, stamped into every MCAP header — ICD §6.2.
///
/// Identifiers are fixed-size byte arrays rather than strings so this stays
/// `no_std` with no allocator. Hashes are stored raw, not hex: a git SHA-1 is
/// 20 bytes and a SHA-256 is 32, and formatting is a presentation concern.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct RunMetadata {
    pub icd_version: (u16, u16),
    pub profile: Profile,
    pub control_rate_hz: f32,
    pub params: Params,
    /// IMU mounting rotation as a quaternion `[w, x, y, z]`. Applied in the
    /// backend, never hard-coded and never applied twice (ICD §10.4). The sim
    /// models a **non-identity** rotation by default so the path is always
    /// exercised.
    pub imu_mounting_rotation: [f32; 4],
    /// Loaded rolling radius, metres — a Stage-0 calibrated value, not the
    /// nominal tyre radius (ICD §10.5).
    pub r_eff_m: f32,
    /// Which sim imperfection profile produced this run. `None` on hardware.
    pub imperfection_profile_id: Option<[u8; 32]>,
    pub schema_hash: [u8; 32],
    pub binary_hash: [u8; 32],
    pub git_sha: [u8; 20],
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_r_eff_m_is_the_documented_145_4mm() {
        // Regression pin mirroring
        // tests/test_r_eff_matches_model.py::test_model_wheel_radius_is_the_documented_145_4mm
        // on the Python side. This crate has no MuJoCo binding to check
        // against the compiled model directly; that check lives in Python.
        assert_eq!(DEFAULT_R_EFF_M, 0.1454);
    }

    #[test]
    fn rad_s_per_erpm_matches_the_icd_quantum_sim_scenarios_imperfections_uses() {
        // Regression pin against sim/scenarios/imperfections.py's
        // `wheel_rate_quantum_rad_s` (STAGE0_PLACEHOLDER / cutback profiles) --
        // same ICD §10.5 ratio, so the two cannot silently drift apart. No
        // Python binding here, so this crate can only pin the literal.
        assert_eq!(RAD_S_PER_ERPM, 0.00698);
    }

    #[test]
    fn command_zero_is_zero_amps() {
        assert_eq!(Command::ZERO, Command::MotorCurrent { amps: 0.0 });
    }

    #[test]
    fn faults_bitfield_roundtrip() {
        let mut f = Faults::NONE;
        assert!(f.is_clear());
        f.insert(Faults::OVER_TILT);
        assert!(f.contains(Faults::OVER_TILT));
        assert!(!f.contains(Faults::OVER_CURRENT));
    }

    #[test]
    fn cold_start_is_invalid_not_zero_valued_truth() {
        // A backend that has not heard from the drive must not present zeros
        // as measurements. ICD 12 calls that non-conforming.
        let obs = Observation::COLD_START;
        assert_eq!(obs.validity, ValidityFlags::ALL_INVALID);
        assert_eq!(obs.imu_count, 0);
        assert!(obs.imu_samples().is_empty());
        assert!(obs.newest_imu().is_none());
    }

    #[test]
    fn imu_samples_exposes_only_the_populated_prefix() {
        let mut obs = Observation::COLD_START;
        for (i, s) in obs.imu.iter_mut().enumerate() {
            s.t_sample_ns = 100 + i as u64;
        }
        obs.imu_count = 3;

        let got = obs.imu_samples();
        assert_eq!(got.len(), 3);
        assert_eq!(got[0].t_sample_ns, 100);
        // Oldest first, so the newest is the last populated entry -- not the
        // last array slot, which still holds a stale sample.
        assert_eq!(obs.newest_imu().unwrap().t_sample_ns, 102);
    }

    #[test]
    fn imu_count_beyond_capacity_cannot_slice_out_of_bounds() {
        // A miscounting backend is a bug, but it must not be able to panic the
        // control loop from the far side of the seam.
        let mut obs = Observation::COLD_START;
        obs.imu_count = 250;
        assert_eq!(obs.imu_samples().len(), MAX_IMU_SAMPLES);
    }
}
