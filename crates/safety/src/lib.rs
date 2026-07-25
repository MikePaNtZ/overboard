//! Pure safety envelope: arm/disarm gating and command clamping.
//!
//! No I/O — this sits between `control-core`'s output and whatever `hal`
//! backend applies it, so it can veto or clamp a command before it ever
//! reaches the wheel.
#![no_std]

use board_types::{Command, Faults, Params};

/// Arm/disarm state of the safety envelope.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ArmState {
    Disarmed,
    Armed,
}

/// Stub safety envelope. Currently identity: armed passes the command
/// through unchanged, disarmed always zeroes it. TODO: real tilt/current
/// limits and fault-triggered disarm land with the actual controller.
#[derive(Debug)]
pub struct Envelope {
    state: ArmState,
    #[allow(dead_code)]
    params: Params,
}

impl Envelope {
    pub fn new(params: Params) -> Self {
        Envelope {
            state: ArmState::Disarmed,
            params,
        }
    }

    pub fn arm(&mut self) {
        self.state = ArmState::Armed;
    }

    pub fn disarm(&mut self) {
        self.state = ArmState::Disarmed;
    }

    /// Apply the safety envelope to a proposed command given current faults.
    ///
    /// Stub behavior: identity when armed and fault-free, `Command::ZERO`
    /// otherwise.
    pub fn apply(&self, cmd: Command, faults: Faults) -> Command {
        if self.state == ArmState::Armed && faults.is_clear() {
            cmd
        } else {
            Command::ZERO
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn disarmed_always_zeroes() {
        let env = Envelope::new(Params::default());
        let cmd = Command {
            current: board_types::Amps(3.0),
        };
        assert_eq!(env.apply(cmd, Faults::NONE), Command::ZERO);
    }

    #[test]
    fn armed_and_fault_free_passes_through() {
        let mut env = Envelope::new(Params::default());
        env.arm();
        let cmd = Command {
            current: board_types::Amps(3.0),
        };
        assert_eq!(env.apply(cmd, Faults::NONE), cmd);
    }

    #[test]
    fn armed_with_fault_zeroes() {
        let mut env = Envelope::new(Params::default());
        env.arm();
        let cmd = Command {
            current: board_types::Amps(3.0),
        };
        assert_eq!(env.apply(cmd, Faults::OVER_TILT), Command::ZERO);
    }
}
