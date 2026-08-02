//! `sim-host` runs the real closed-loop board (issue #161, wire spine): it
//! opens the same `plant-mujoco` + `control-ffi`-equivalent closed loop
//! `sim-backend` already hosts at 500 Hz (`crates/sim-backend`'s own doc
//! comment, and `tests/test_rust_hosted_impulse_response.py`), on its own
//! dedicated thread, and speaks the fixed v1 UDP wire to Unreal:
//! [`wire::StateOut`] out to `127.0.0.1:9601`, [`wire::InputIn`] in on
//! `127.0.0.1:9602`.
//!
//! # Library + thin binary (SR-GAME-15)
//!
//! This crate is a library ([`host::run`]/[`host::spawn`]) with a thin
//! binary wrapper (`src/bin/sim-host.rs`). The day something wants this loop
//! in-process instead of over a socket, linking `sim-host` is a build-graph
//! decision, not a rewrite.
//!
//! `src/bin/wire-probe.rs` is a second, separate binary -- the CEO-visible
//! verification tool for the wire this crate speaks. It only depends on
//! [`wire`], never on [`host`]: it is a standalone UDP listener, not a
//! client of this library.

pub mod host;
pub mod pacer;
pub mod scenario;
pub mod wire;

pub use host::{run, spawn, HostConfig, HostError, RunSummary, CYCLE_NS};
