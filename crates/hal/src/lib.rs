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
//! - `hal_actuate::BoardActuate::apply` enqueues and **never** advances time.
//!   Actuation delay is **additive** on top of the structural loop delay, not
//!   inclusive of it.
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
//! Observing and actuating are separate **crates**, not just separate traits:
//! `BoardObserve` lives here; `BoardActuate` and `Disarm` live in
//! `hal-actuate`, which the ridden binary must not depend on (ICD §6.3,
//! DR-MODE-1). A trait split alone does not survive `cargo build
//! --all-features` or a transitive dependency — cargo features unify across
//! the whole graph, but **absence of a dependency does not unify**. The
//! `xtask` gate asserts `hal-actuate` is unreachable from `board-app-ridden`
//! by walking `cargo metadata`'s resolve graph over normal edges; see
//! `crates/xtask`.
#![no_std]

use board_types::{IoError, Observation, RunMetadata};

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
