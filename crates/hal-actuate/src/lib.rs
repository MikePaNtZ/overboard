//! Motion authority — the half of the `hal` seam the ridden binary must never
//! link (BoardIo ICD §6.3, DR-MODE-1).
//!
//! [`BoardActuate`] and [`Disarm`] used to live in `hal` itself, split out
//! only by trait. That is not the property this project needs: a trait split
//! survives a careful reviewer, but not `cargo build --workspace
//! --all-features`, not a transitive dependency, not a dev-dependency of a
//! shared crate. Cargo features are additive and unify across the whole
//! dependency graph; **absence of a dependency does not unify**, which is why
//! motion authority lives in its own crate instead.
//!
//! `crates/xtask`'s gate walks `cargo metadata`'s resolve graph from
//! `board-app-ridden` over normal edges (dev- and build-dependencies excluded
//! on purpose — they don't ship) and fails the build if this crate is
//! reachable. `crates/canary-ridden` is the positive control: it deliberately
//! depends on this crate so the gate has something it is required to catch.
#![no_std]

use board_types::{Applied, Command, DisarmReason, IoError};
use hal::BoardObserve;

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

/// Motion authority. Kept in a separate crate from [`hal::BoardObserve`] so a
/// binary can be built without ever linking it — DR-MODE-1's shadow-mode
/// shape, enforced at the dependency graph rather than by a runtime check.
pub trait BoardActuate: BoardObserve {
    /// The disarm capability handed out by [`BoardActuate::arm`].
    type Disarm: Disarm;

    /// Enable motion, returning the disarm handle.
    fn arm(&mut self) -> Result<Self::Disarm, IoError>;

    /// Enqueue a command. Does **not** advance time. Returns the post-clamp
    /// value so anti-windup can be driven synchronously (ICD §7.6).
    fn apply(&mut self, cmd: &Command) -> Result<Applied, IoError>;
}
