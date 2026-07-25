//! Pure, deterministic control logic. No I/O, no clock reads — dt is
//! derived from timestamps carried on [`Observation`].
//!
//! Depends on `board-types` only (not `hal`), so it can be unit-tested and
//! fuzzed without any backend in the loop.
#![no_std]

use board_types::{Command, Observation, Params};

/// Stub cascaded balance controller.
///
/// TODO: replace with the real cascaded (inner rate loop / outer tilt loop)
/// controller. For now this always returns [`Command::ZERO`] so the sim
/// checkpoint demonstrates an *uncontrolled* onewheel toppling over.
#[derive(Debug, Default)]
pub struct Controller {
    #[allow(dead_code)]
    params: Params,
}

impl Controller {
    pub fn new(params: Params) -> Self {
        Controller { params }
    }

    /// Compute the next command from the latest observation.
    ///
    /// Currently a stub: always zero effort, regardless of `obs`.
    pub fn step(&mut self, _obs: &Observation) -> Command {
        Command::ZERO
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stub_controller_always_returns_zero() {
        let mut ctrl = Controller::new(Params::default());
        let obs = Observation {
            gyro: [1.0, 2.0, 3.0],
            accel: [0.0, 0.0, 9.8],
            wheel_angular_velocity: board_types::RadPerSec(5.0),
            applied_current: board_types::Amps(1.0),
            timestamp_ns: 1_000,
            cycle: 1,
            fault_word: 0,
            valid: true,
        };
        assert_eq!(ctrl.step(&obs), Command::ZERO);
    }
}
