//! Shared types for the Overboard control stack.
//!
//! Kept `no_std`-friendly (no allocation, no std::time, no clock reads) so it
//! can eventually be shared with a bare-metal or PREEMPT_RT-hosted binary
//! without pulling in std. Timing is always passed in explicitly via
//! [`Observation::timestamp_ns`] — nothing in this crate reads a clock.
#![no_std]

use serde::{Deserialize, Serialize};

/// Single-wheel current setpoint sent to the drive (VESC, eventually).
///
/// Positive values drive the wheel forward; the sign convention matches the
/// hub motor's forward direction.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Command {
    /// Requested wheel current, in amps.
    pub current: Amps,
}

impl Command {
    /// The zero-effort command: no current requested.
    pub const ZERO: Command = Command { current: Amps(0.0) };
}

/// One cycle's worth of sensor + actuator state, as reported by a
/// [`hal::BoardIo`] backend (sim or hardware).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Observation {
    /// Raw gyroscope reading, rad/s, body frame [x, y, z].
    pub gyro: [f32; 3],
    /// Raw accelerometer reading, m/s^2, body frame [x, y, z].
    pub accel: [f32; 3],
    /// Wheel angular velocity, rad/s.
    pub wheel_angular_velocity: RadPerSec,
    /// The current actually applied last cycle, in amps.
    pub applied_current: Amps,
    /// Monotonic timestamp of this observation, nanoseconds. The backend
    /// owns the clock; control-core never reads one directly and instead
    /// derives dt from consecutive timestamps.
    pub timestamp_ns: u64,
    /// Monotonically increasing cycle counter, for detecting dropped/repeated
    /// cycles independent of timestamps.
    pub cycle: u64,
    /// Bitfield of active faults, see [`Faults`].
    pub fault_word: u32,
    /// Whether this observation is fresh/trustworthy. A backend may set this
    /// to `false` (e.g. on a stale sensor read) without necessarily raising
    /// a fault.
    pub valid: bool,
}

impl Observation {
    /// Returns `true` if this observation is older than `max_age_ns` relative
    /// to `now_ns`, or is otherwise marked invalid.
    pub fn is_stale(&self, now_ns: u64, max_age_ns: u64) -> bool {
        !self.valid || now_ns.saturating_sub(self.timestamp_ns) > max_age_ns
    }
}

/// Bitfield of fault conditions. Stored as a plain `u32` newtype (no
/// external bitflags dependency) so it stays `no_std`-friendly.
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

/// Tunable controller/safety parameters. Stub — real cascaded-controller
/// gains land with the actual control-core implementation.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Params {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    /// Maximum allowed wheel current, amps. Used by `safety` to clamp.
    pub max_current: Amps,
}

impl Default for Params {
    fn default() -> Self {
        Params {
            kp: 0.0,
            ki: 0.0,
            kd: 0.0,
            max_current: Amps(0.0),
        }
    }
}

/// Amps, as an SI-unit newtype to keep current values from being confused
/// with other f32 quantities at the type level.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Serialize, Deserialize)]
pub struct Amps(pub f32);

/// Radians, an SI-unit newtype for angles.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Serialize, Deserialize)]
pub struct Radians(pub f32);

/// Radians per second, an SI-unit newtype for angular velocity.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Serialize, Deserialize)]
pub struct RadPerSec(pub f32);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn command_zero_is_zero_amps() {
        assert_eq!(Command::ZERO.current, Amps(0.0));
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
    fn staleness_detects_old_timestamp() {
        let obs = Observation {
            gyro: [0.0; 3],
            accel: [0.0; 3],
            wheel_angular_velocity: RadPerSec(0.0),
            applied_current: Amps(0.0),
            timestamp_ns: 0,
            cycle: 0,
            fault_word: 0,
            valid: true,
        };
        assert!(obs.is_stale(1_000_000, 100));
        assert!(!obs.is_stale(50, 100));
    }
}
