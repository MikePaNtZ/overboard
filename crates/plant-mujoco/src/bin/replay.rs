//! Open-loop `ctrl` replay binary for I1b (issue #106).
//!
//! `tests/test_rust_python_plant_replay_equivalence.py` builds this once (`cargo build --release
//! -p plant-mujoco --bin plant-replay`) and then shells out to it for every
//! scenario model, exactly the way `sim/scenarios/rust_controller.py` shells
//! out to `control-ffi` for the closed-loop gate. It exists to make the
//! Rust-hosted half of the equivalence check reachable from `pytest`, which
//! is where the actual bit-identical comparison against the Python-hosted
//! plant lives -- this binary itself asserts nothing.
//!
//! ```text
//! plant-replay <model.xml> <ctrl.f64le> <out.f64le>
//! ```
//!
//! `ctrl.f64le` is `n_steps * nu` little-endian `f64`s, row-major by step
//! (step 0's `nu` values, then step 1's, ...) -- a **recorded** sequence, not
//! generated here, so both hosts replay the identical bytes and any
//! divergence in the output can only come from `mj_step` itself, never from
//! two languages computing the input differently.
//!
//! `out.f64le` is written the same way: for each step, `1 + nq + nv`
//! little-endian `f64`s -- `time`, then `qpos`, then `qvel` -- in the exact
//! order documented in `crates/plant-mujoco/README.md`'s "Ordering contract".

use plant_mujoco::Plant;
use std::env;
use std::path::PathBuf;
use std::process::ExitCode;

fn main() -> ExitCode {
    let args: Vec<String> = env::args().collect();
    let [_, model_path, ctrl_path, out_path] = args.as_slice() else {
        eprintln!("usage: plant-replay <model.xml> <ctrl.f64le> <out.f64le>");
        return ExitCode::FAILURE;
    };

    let mut plant = match Plant::open(&PathBuf::from(model_path)) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("plant-replay: Plant::open({model_path}) failed: {e}");
            return ExitCode::FAILURE;
        }
    };

    let nu = plant.nu();
    let ctrl_bytes = std::fs::read(ctrl_path)
        .unwrap_or_else(|e| panic!("plant-replay: could not read ctrl file '{ctrl_path}': {e}"));
    if ctrl_bytes.len() % 8 != 0 {
        panic!(
            "plant-replay: ctrl file '{ctrl_path}' is {} bytes, not a whole \
             number of f64s",
            ctrl_bytes.len()
        );
    }
    let ctrl: Vec<f64> = ctrl_bytes
        .as_chunks::<8>()
        .0
        .iter()
        .map(|c| f64::from_le_bytes(*c))
        .collect();
    if nu == 0 || !ctrl.len().is_multiple_of(nu) {
        panic!(
            "plant-replay: ctrl file holds {} f64s, not a whole number of \
             {nu}-wide (mjModel::nu) steps",
            ctrl.len()
        );
    }
    let n_steps = ctrl.len() / nu;

    // 1 (time) + nq + nv f64s per step. Nothing here reads sensordata --
    // this is an OPEN-LOOP replay (I1b's explicit scope: no controller), so
    // there is no "sensor read before vs after step" seam to pin down and
    // deliberately no `mj_forward` call before the loop either, which would
    // otherwise perturb `qacc_warmstart` ahead of the first real `mj_step`
    // and make this replay depend on a call the Python side never makes.
    let mut out = Vec::with_capacity(n_steps * (1 + plant.nq() + plant.nv()) * 8);
    for step in 0..n_steps {
        // ctrl written BEFORE mj_step, mirroring every Python scenario's
        // `data.ctrl[...] = ...` line immediately preceding
        // `mujoco.mj_step(model, data)` -- never split across mj_step1 /
        // mj_step2, and never written mid-step.
        plant.set_ctrl(&ctrl[step * nu..(step + 1) * nu]);
        let time = plant.step();

        // qpos/qvel read AFTER mj_step returns, same as every Python
        // scenario reads `data.qpos`/`data.qvel` only once mj_step has run.
        let qpos = plant.qpos();
        let qvel = plant.qvel();

        out.extend_from_slice(&time.to_le_bytes());
        for v in &qpos {
            out.extend_from_slice(&v.to_le_bytes());
        }
        for v in &qvel {
            out.extend_from_slice(&v.to_le_bytes());
        }
    }

    if let Err(e) = std::fs::write(out_path, &out) {
        eprintln!("plant-replay: could not write output file '{out_path}': {e}");
        return ExitCode::FAILURE;
    }

    ExitCode::SUCCESS
}
