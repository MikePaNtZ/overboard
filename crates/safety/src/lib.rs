//! Pure safety envelope: arm/disarm gating and command clamping.
//!
//! No I/O — this sits between `control-core`'s output and whatever `hal`
//! backend applies it, so it can veto or clamp a command before it ever
//! reaches the wheel.
#![no_std]

use board_types::{Command, Faults, Params, Saturation};

/// Arm/disarm state of the safety envelope.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ArmState {
    Disarmed,
    Armed,
}

/// The authoritative safety envelope — stage 2 of ICD §7.6.
///
/// Disarmed or faulted zeroes the command outright; armed clamps it to the
/// resolved [`Params`] limits and reports whether a limit bound.
///
/// Still to come with the controller: a tilt trip driven by
/// `max_abs_pitch_rad` (which needs a pitch estimate that does not yet exist)
/// and fault-triggered automatic disarm.
#[derive(Debug)]
pub struct Envelope {
    state: ArmState,
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
    /// This is **stage 2** of the three-stage clamp (ICD §7.6) — the
    /// authoritative envelope, and the one that must behave identically in sim
    /// and on hardware (DR-SAFE-1), which is why it is pure and reads its
    /// limits from the same resolved [`Params`] both backends are given.
    ///
    /// Returns the bounded command together with whether a limit actually bound
    /// this cycle, because the controller's anti-windup has to know
    /// synchronously; `Applied.saturated` is the OR of this and the backend's
    /// stage-3 clamp.
    pub fn apply(&self, cmd: Command, faults: Faults) -> (Command, Saturation) {
        if self.state != ArmState::Armed || !faults.is_clear() {
            // Disarmed or faulted is not "saturated" -- it is no authority at
            // all. Reporting Yes here would feed a clamp signal into
            // anti-windup for a command that was never a candidate.
            return (Command::ZERO, Saturation::No);
        }

        match cmd {
            Command::MotorCurrent { amps } => {
                let limit = self.params.max_current_a.abs();
                if amps.abs() > limit {
                    let bounded = if amps.is_sign_negative() {
                        -limit
                    } else {
                        limit
                    };
                    (Command::MotorCurrent { amps: bounded }, Saturation::Yes)
                } else {
                    (cmd, Saturation::No)
                }
            }
            // D-REMOTE: Refloat's internal clamp is never reported back, so we
            // genuinely cannot say whether one bound (ICD §7.6).
            Command::RemoteSpeed { .. } => (cmd, Saturation::Unknown),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn armed(max_current_a: f32) -> Envelope {
        let mut env = Envelope::new(Params {
            max_current_a,
            ..Params::default()
        });
        env.arm();
        env
    }

    #[test]
    fn disarmed_always_zeroes() {
        let env = Envelope::new(Params {
            max_current_a: 10.0,
            ..Params::default()
        });
        let cmd = Command::MotorCurrent { amps: 3.0 };
        assert_eq!(
            env.apply(cmd, Faults::NONE),
            (Command::ZERO, Saturation::No)
        );
    }

    #[test]
    fn armed_and_fault_free_passes_an_in_range_command_through() {
        let env = armed(10.0);
        let cmd = Command::MotorCurrent { amps: 3.0 };
        assert_eq!(env.apply(cmd, Faults::NONE), (cmd, Saturation::No));
    }

    #[test]
    fn armed_with_fault_zeroes() {
        let env = armed(10.0);
        let cmd = Command::MotorCurrent { amps: 3.0 };
        assert_eq!(
            env.apply(cmd, Faults::OVER_TILT),
            (Command::ZERO, Saturation::No)
        );
    }

    #[test]
    fn over_limit_current_is_clamped_and_reported_saturated() {
        let env = armed(10.0);
        assert_eq!(
            env.apply(Command::MotorCurrent { amps: 25.0 }, Faults::NONE),
            (Command::MotorCurrent { amps: 10.0 }, Saturation::Yes)
        );
    }

    #[test]
    fn the_clamp_is_symmetric_about_zero() {
        // A one-sided clamp would brake but never drive, which on a balancer
        // is an asymmetric authority bug that only shows up under a hard
        // recovery in one direction.
        let env = armed(10.0);
        assert_eq!(
            env.apply(Command::MotorCurrent { amps: -25.0 }, Faults::NONE),
            (Command::MotorCurrent { amps: -10.0 }, Saturation::Yes)
        );
    }

    #[test]
    fn exactly_at_the_limit_is_not_saturated() {
        let env = armed(10.0);
        assert_eq!(
            env.apply(Command::MotorCurrent { amps: 10.0 }, Faults::NONE),
            (Command::MotorCurrent { amps: 10.0 }, Saturation::No)
        );
    }

    #[test]
    fn disarmed_is_reported_as_unsaturated_not_clamped() {
        // No authority is not the same as a bound limit. Reporting Yes would
        // feed anti-windup a clamp signal for a command that never had a
        // chance to be applied.
        let env = Envelope::new(Params {
            max_current_a: 10.0,
            ..Params::default()
        });
        let (_, sat) = env.apply(Command::MotorCurrent { amps: 999.0 }, Faults::NONE);
        assert_eq!(sat, Saturation::No);
    }

    #[test]
    fn remote_speed_saturation_is_unknown_never_no() {
        // Refloat clamps internally and never reports it, so claiming No would
        // be a lie the controller might drive anti-windup from (ICD 7.6).
        let env = armed(10.0);
        let (_, sat) = env.apply(Command::RemoteSpeed { m_s: 99.0 }, Faults::NONE);
        assert_eq!(sat, Saturation::Unknown);
    }
}
