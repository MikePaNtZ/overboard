//! Positive control for `crates/xtask`'s dependency-graph gate.
//!
//! This binary is deliberately shaped like `board-app-ridden` but links
//! `hal-actuate` AND `plant-mujoco` anyway — the exact mistakes the real
//! crate split, and `hal`'s `#![no_std]`-ness, exist to catch (issue #1: a
//! copy-pasted `main.rs` that keeps the `BoardActuate` import, or a "just
//! for now" dependency that never gets removed; issue #91: a native physics
//! dependency leaking into the ridden binary the same way). The gate is
//! required to flag this crate for BOTH of them. If it stops flagging
//! `canary-ridden` — because a marker crate was renamed and the gate wasn't
//! updated, or the walk logic regressed — that is exactly the silent
//! failure mode this canary is here to turn loud instead.
//!
//! Not built or run as part of the normal loop; `cargo build --workspace`
//! compiles it (proving both graph edges are real, not just declared and
//! unused), and `xtask` is what actually inspects it.

use board_types::{
    Applied, Command, DisarmReason, IoError, Observation, RunMetadata, DEFAULT_R_EFF_M,
};
use hal::BoardObserve;
use hal_actuate::{BoardActuate, Disarm};

struct CanaryDisarm;

impl Disarm for CanaryDisarm {
    fn disarm(&self, _reason: DisarmReason) -> Result<(), IoError> {
        Ok(())
    }
}

/// A backend that -- wrongly, for anything shaped like the ridden binary --
/// implements both halves of the seam.
#[derive(Default)]
struct CanaryBackend;

impl BoardObserve for CanaryBackend {
    fn open(&mut self) -> Result<(), IoError> {
        Ok(())
    }
    fn close(&mut self) -> Result<(), IoError> {
        Ok(())
    }
    fn wait_observe(&mut self) -> Result<Observation, IoError> {
        Ok(Observation::COLD_START)
    }
    fn run_metadata(&self) -> RunMetadata {
        RunMetadata {
            icd_version: (0, 3),
            profile: board_types::Profile::DRaw,
            control_rate_hz: 500.0,
            params: Default::default(),
            imu_mounting_rotation: [1.0, 0.0, 0.0, 0.0],
            r_eff_m: DEFAULT_R_EFF_M,
            imperfection_profile_id: None,
            schema_hash: [0; 32],
            binary_hash: [0; 32],
            git_sha: [0; 20],
        }
    }
}

impl BoardActuate for CanaryBackend {
    type Disarm = CanaryDisarm;

    fn arm(&mut self) -> Result<Self::Disarm, IoError> {
        Ok(CanaryDisarm)
    }

    fn apply(&mut self, cmd: &Command) -> Result<Applied, IoError> {
        Ok(Applied {
            commanded: *cmd,
            saturated: board_types::Saturation::No,
            t_apply_ns: 0,
        })
    }
}

fn main() {
    println!(
        "canary-ridden: this binary links hal-actuate AND plant-mujoco on purpose -- \
         the gate in `cargo run -p xtask -- gate` must fail if it does not flag both."
    );
    let mut backend = CanaryBackend;
    let _ = backend.open();
    let _disarm = backend.arm();
    let _ = backend.apply(&Command::ZERO);

    // Real use of plant-mujoco's linked symbol, not just a declared-but-dead
    // dependency -- proves the graph edge is something the linker actually
    // has to resolve.
    println!(
        "canary-ridden: linked libmujoco reports version {}",
        plant_mujoco::mujoco_version_string()
    );
}
