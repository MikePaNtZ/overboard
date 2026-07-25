//! `sim-backend` implements [`hal::BoardIo`] against the MuJoCo onewheel
//! model.
//!
//! TODO(next milestone): this is currently a STUB. It does not step MuJoCo
//! at all — it returns synthetic all-zero observations with an incrementing
//! synthetic clock/cycle counter, just enough to exercise the `hal` seam
//! end-to-end. Wiring this up to actually step `sim/models/overboard_onewheel.xml`
//! via MuJoCo's C API (through FFI, likely `mujoco-rs` or hand-rolled
//! bindgen) is the next milestone and is intentionally out of scope here.

use board_types::{Command, Observation, RadPerSec};
use hal::BoardIo;

/// Nanoseconds per synthetic cycle. Arbitrary until real MuJoCo stepping
/// (with the model's actual timestep) replaces this stub.
const CYCLE_NS: u64 = 1_000_000; // 1 kHz

/// Stub sim backend: synthetic zeros, incrementing clock. No MuJoCo FFI yet.
#[derive(Debug, Default)]
pub struct SimBackend {
    cycle: u64,
    timestamp_ns: u64,
}

impl SimBackend {
    pub fn new() -> Self {
        SimBackend::default()
    }
}

impl BoardIo for SimBackend {
    fn cycle(&mut self, cmd: &Command) -> Observation {
        self.cycle += 1;
        self.timestamp_ns += CYCLE_NS;
        Observation {
            gyro: [0.0; 3],
            accel: [0.0; 3],
            wheel_angular_velocity: RadPerSec(0.0),
            applied_current: cmd.current,
            timestamp_ns: self.timestamp_ns,
            cycle: self.cycle,
            fault_word: 0,
            valid: true,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use board_types::Amps;

    #[test]
    fn cycle_increments_clock_and_echoes_command() {
        let mut backend = SimBackend::new();
        let cmd = Command { current: Amps(2.5) };
        let obs1 = backend.cycle(&cmd);
        let obs2 = backend.cycle(&cmd);
        assert_eq!(obs1.cycle, 1);
        assert_eq!(obs2.cycle, 2);
        assert!(obs2.timestamp_ns > obs1.timestamp_ns);
        assert_eq!(obs1.applied_current, Amps(2.5));
    }
}
