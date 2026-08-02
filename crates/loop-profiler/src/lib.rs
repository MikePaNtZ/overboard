//! `loop-profiler` -- can this host hold a 500 Hz control loop?
//!
//! Stage 0B's headline deliverable is command→actuation latency (AC-6), and
//! that needs a drive on a bus. This crate answers the question that comes
//! before it and gates it: **does the platform hold the cadence at all** --
//! AC-5's scheduler-jitter criterion, plus the cost of the control law
//! itself. It needs no CAN hardware, no motor, and no MuJoCo, so it runs in
//! CI on every commit (AC-12's no-Pi coverage) and on the Pi the hour it
//! arrives.
//!
//! # It cannot turn a motor
//!
//! Structurally, not by convention: this crate does not depend on
//! `hal-actuate`, so it has no `apply()` to call and no backend to arm. That
//! is what puts it in the reference doc §6 category of runs which do NOT
//! need the CEO stood over the deadman -- alongside `cyclictest`, `vcan`
//! work, and image builds.
//!
//! # It is not a substitute for `cyclictest`
//!
//! `cyclictest` measures the kernel; this measures **our loop on that
//! kernel** -- the same absolute-deadline pacing, the same estimator,
//! regulator, feedforward and envelope that the board runs. Run both. They
//! disagreeing is informative: `cyclictest` clean and this dirty means the
//! problem is ours.

pub mod profile;
pub mod report;
pub mod rt;
pub mod stats;

pub use profile::{Config, Report};
