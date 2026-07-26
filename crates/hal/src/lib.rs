//! `hal` is the seam between `control-core` and whatever is actually driving
//! the wheel: a MuJoCo sim (`sim-backend`) or real hardware (a future
//! `hw-backend`).
//!
//! # The time and causality contract — ICD §5.2, normative
//!
//! ```text
//! loop {
//!     let obs = io.wait_observe()?;      // BLOCKS to the control instant; ADVANCES the plant
//!     let cmd = controller.update(&obs); // pure; reads no clock
//!     let applied = io.apply(&cmd)?;     // ENQUEUES; does NOT advance time
//! }
//! ```
//!
//! - [`BoardObserve::wait_observe`] is the **sole** time-advancing call. In sim
//!   it steps physics to the next control instant; on hardware it blocks on the
//!   IMU/timer.
//! - [`BoardActuate::apply`] enqueues and **never** advances time. Actuation
//!   delay is **additive** on top of the structural loop delay, not inclusive
//!   of it.
//! - Zero or one `apply()` per `wait_observe()`. **Zero is legal** — it is the
//!   shadow-mode loop shape (§6.6).
//!
//! This split replaced a single `cycle(cmd) -> Observation` call (DR-BOARDIO-1,
//! amended). That signature conflated observing with actuating, which is what
//! made pre/post-command state ambiguous — and on a balancer, phase error is
//! indistinguishable from negative damping.
//!
//! # Motion authority
//!
//! [`BoardObserve`] and [`BoardActuate`] are separate traits so a binary can
//! link the first without the second: shadow mode then fails to compile if it
//! ever tries to actuate, rather than being prevented by a runtime check
//! (DR-MODE-1).
//!
//! > **Deferred:** ICD §6.3 requires `BoardActuate` implementations and the
//! > wire encoders to live in a *separate crate* the ridden binary does not
//! > depend on, with CI asserting the dependency graph. That is not yet done —
//! > there is no rider, no hardware and no ridden profile to protect. The trait
//! > split here is the part that has present value; the crate split must land
//! > **before the ballasted-dummy bench gate**, and DR-MODE-1 stays unmet until
//! > it does.
#![no_std]

use board_types::{Applied, Command, DisarmReason, IoError, Observation, RunMetadata};

/// Observation and metadata. The ridden binary links exactly this and nothing
/// more.
pub trait BoardObserve {
    fn open(&mut self) -> Result<(), IoError>;
    fn close(&mut self) -> Result<(), IoError>;

    /// Block until the next control instant, then return the freshest
    /// observation.
    ///
    /// **The sole time-advancing call.** The backend owns the clock;
    /// `control-core` never reads one.
    fn wait_observe(&mut self) -> Result<Observation, IoError>;

    /// Backend-owned run provenance for the MCAP header (ICD §6.2).
    fn run_metadata(&self) -> RunMetadata;
}

/// A disarm capability that outlives the control thread.
///
/// ICD §9.2 requires disarm to be callable from a wedged or panicking loop,
/// from a watchdog thread, a signal handler or a panic hook. That rules out
/// `disarm(&mut self)` on the backend — `&mut self` means only the thread that
/// owns the backend may call it, which is precisely the thread that is stuck.
///
/// Realised as an associated type rather than the ICD's concrete struct so each
/// backend can carry its own pre-opened handle and pre-encoded safe-state
/// frame. The contract is unchanged.
///
/// Implementations **must not** allocate, lock, or block unboundedly —
/// everything needed is prepared at `arm()` time — and **must** be idempotent,
/// because several paths may race to call it.
pub trait Disarm: Send + Sync {
    fn disarm(&self, reason: DisarmReason) -> Result<(), IoError>;
}

/// Motion authority. Kept separate from [`BoardObserve`] so it can be left
/// unlinked entirely.
pub trait BoardActuate: BoardObserve {
    /// The disarm capability handed out by [`BoardActuate::arm`].
    type Disarm: Disarm;

    /// Enable motion, returning the disarm handle.
    fn arm(&mut self) -> Result<Self::Disarm, IoError>;

    /// Enqueue a command. Does **not** advance time. Returns the post-clamp
    /// value so anti-windup can be driven synchronously (ICD §7.6).
    fn apply(&mut self, cmd: &Command) -> Result<Applied, IoError>;
}

/// Enforces the §5.2 call-sequence rules: zero-or-one `apply()` per
/// `wait_observe()`, and never an `apply()` before the first `wait_observe()`.
///
/// Lives here rather than in each backend so sim and hardware cannot drift on
/// the one rule whose violation is silent. Both are contract violations that
/// must return [`IoError::ProtocolViolation`] **without sending a frame** —
/// sending two current setpoints in one control period is a real actuation
/// fault, not a bookkeeping error.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct CallSequence {
    observed: bool,
    applied_this_cycle: bool,
}

impl CallSequence {
    pub const fn new() -> Self {
        CallSequence {
            observed: false,
            applied_this_cycle: false,
        }
    }

    /// Record that a control instant was reached. Re-arms the `apply()` budget.
    pub fn on_observe(&mut self) {
        self.observed = true;
        self.applied_this_cycle = false;
    }

    /// Consume this cycle's `apply()` budget.
    ///
    /// Returns `Err(ProtocolViolation)` if no observation has happened yet, or
    /// if this cycle already applied. The caller must send nothing in that
    /// case.
    pub fn on_apply(&mut self) -> Result<(), IoError> {
        if !self.observed || self.applied_this_cycle {
            return Err(IoError::ProtocolViolation);
        }
        self.applied_this_cycle = true;
        Ok(())
    }

    /// Reset to the pre-`open()` state.
    pub fn reset(&mut self) {
        *self = CallSequence::new();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn apply_before_any_observe_is_a_protocol_violation() {
        let mut seq = CallSequence::new();
        assert_eq!(seq.on_apply(), Err(IoError::ProtocolViolation));
    }

    #[test]
    fn one_apply_per_observe_is_allowed() {
        let mut seq = CallSequence::new();
        seq.on_observe();
        assert_eq!(seq.on_apply(), Ok(()));
    }

    #[test]
    fn a_second_apply_in_the_same_cycle_is_a_protocol_violation() {
        let mut seq = CallSequence::new();
        seq.on_observe();
        assert_eq!(seq.on_apply(), Ok(()));
        assert_eq!(seq.on_apply(), Err(IoError::ProtocolViolation));
    }

    #[test]
    fn zero_applies_is_legal_and_is_the_shadow_mode_shape() {
        // Shadow mode computes a command every cycle and applies none of them.
        // Many observes in a row must never trip the guard.
        let mut seq = CallSequence::new();
        for _ in 0..1000 {
            seq.on_observe();
        }
        assert_eq!(seq.on_apply(), Ok(()));
    }

    #[test]
    fn the_budget_re_arms_on_the_next_observe() {
        let mut seq = CallSequence::new();
        for _ in 0..3 {
            seq.on_observe();
            assert_eq!(seq.on_apply(), Ok(()));
            assert_eq!(seq.on_apply(), Err(IoError::ProtocolViolation));
        }
    }

    #[test]
    fn reset_returns_to_the_pre_open_state() {
        let mut seq = CallSequence::new();
        seq.on_observe();
        seq.reset();
        assert_eq!(seq.on_apply(), Err(IoError::ProtocolViolation));
    }
}
