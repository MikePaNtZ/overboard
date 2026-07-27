//! Positive control for `crates/xtask`'s dependency-graph gate.
//!
//! This binary is deliberately shaped like `board-app-ridden` but links
//! `hal-actuate` anyway — the exact mistake the real crate split exists to
//! catch (issue #1: a copy-pasted `main.rs` that keeps the `BoardActuate`
//! import, or a "just for now" dependency that never gets removed). The gate
//! is required to flag this crate. If it stops flagging `canary-ridden` —
//! because the marker crate was renamed and the gate wasn't updated, or the
//! walk logic regressed — that is exactly the silent failure mode this
//! canary is here to turn loud instead.
//!
//! Not built or run as part of the normal loop; `cargo build --workspace`
//! compiles it (proving the graph edge is real, not just declared and
//! unused), and `xtask` is what actually inspects it.

use board_types::{Applied, Command, DisarmReason, IoError, Observation, RunMetadata};
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
            r_eff_m: 0.14605,
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
        "canary-ridden: this binary links hal-actuate on purpose -- \
         the gate in `cargo run -p xtask -- gate` must fail if it does not flag this crate."
    );
    let mut backend = CanaryBackend;
    let _ = backend.open();
    let _disarm = backend.arm();
    let _ = backend.apply(&Command::ZERO);
}
