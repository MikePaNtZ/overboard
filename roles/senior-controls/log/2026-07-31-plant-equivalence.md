# I1b: open-loop plant equivalence, Rust-hosted vs Python-hosted MuJoCo

Issue #106, depends on I1a (#91, merged #111). The gate this whole workstream needs before any
closed-loop metric comparison (I1c, #107) can be trusted: an ordering bug in the plant wrapper
must be distinguishable from real physical divergence, and the only way to make that true is a
bit-identical, open-loop, no-controller comparison underneath the closed-loop one.

**`crates/plant-mujoco`** extended with the `ctrl`-in / `qpos`+`qvel`-out surface I1a's `Plant`
didn't need: `nq()`/`nv()`/`nu()`, `set_ctrl(&[f64])`, `qpos()`/`qvel()` (`src/lib.rs`), backed
by four new tiny functions in `src/shim.c` (`plant_mujoco_{nq,nv,nu,set_ctrl,get_qpos,get_qvel}`)
— still opaque-handle-only, no `#[repr(C)]` mirrors. New bin target `plant-replay`
(`src/bin/replay.rs`, named explicitly in `Cargo.toml` rather than left at the file-derived
default) replays a recorded `ctrl` sequence open-loop and writes `[time, qpos, qvel]` per step as
raw little-endian `f64`s to a file — the Rust-hosted half of the comparison, reachable from
`pytest` the same way `sim/scenarios/rust_controller.py` shells out to `control-ffi`.

**`tests/test_rust_python_plant_replay_equivalence.py`**: the actual gate. Generates a deterministic pseudo-random
`ctrl` sequence ONCE in Python (fixed seed, scaled to each model's own `actuator_ctrlrange`),
writes it as raw bytes, replays those identical bytes through both hosts — Python in-process
(`mujoco.MjData`/`mujoco.mj_step`, mirroring `sim/scenarios/impulse_response.py`'s per-step
ordering exactly), Rust via `cargo run -p plant-mujoco --bin plant-replay` (through `cargo`, not
the raw binary, so the linked libmujoco resolves the same way `cargo test` already does) — and
asserts `qpos`/`qvel`/`time` are `numpy.array_equal` (bit-identical, no tolerance) at every step,
on both `sim/models/overboard_onewheel.xml` and `sim/models/bench_rig.xml`.

**N = 2000 steps.** At `bench_rig`'s `dt=0.0005s` that's 1.0s of sim time; at
`overboard_onewheel`'s `dt=0.002s` that's 4.0s — both several times past the ~0.5s window the
issue names as where a one-tick ordering bug's trajectories visibly separate, and both far more
than "ten steps".

**The two hosts already agreed — nothing needed reconciling.** All six seams the issue calls
out (`mj_step` vs the `mj_step1`/`mj_step2` split; `ctrl` before vs within the step; state read
before vs after; fresh `mjData` vs `mj_resetData`; `qacc_warmstart` carry-over; substepping) were
already the same choice on both sides once written down side by side — see the "Ordering
contract" table in `crates/plant-mujoco/README.md`. In particular: neither host calls
`mj_forward` before the first `mj_step` (the controlled scenarios do, to prime `sensordata` for
a controller this replay has none of — calling it here would have primed `qacc_warmstart`
asymmetrically), and both use a freshly allocated `mjData` rather than `mj_resetData` on a reused
one (`mj_makeData`'s own header comment implies it already sets the initial configuration).

**Deliberate violation, shown failing, not asserted.** Swapped `plant-replay`'s per-step order to
`step()` then `set_ctrl()` (ctrl written one tick late) and re-ran. Both models failed
immediately: `qpos diverged at step 0 of 2000` on both. `bench_rig`'s output made the bug legible
by eye — the buggy Rust `qpos` sequence was exactly the Python sequence shifted by one step (the
buggy row 0 equalled Python's row -1 i.e. the initial state's *next* torque application landed a
tick late throughout). Reverted; re-ran clean. Full before/after transcript in the PR body.

**Left out, flagged rather than fixed:** issue #112 (macOS local linking — the pip wheel's
`.dylib` install name not matching its filename on some wheel builds) is untouched, as scoped.
Documented the local-dev workaround (`cargo run`/`cargo test`, never the raw built artifact) in
the README rather than working around it silently, since `_rust_hosted_replay` needed to make
the same choice.

**ICD**: the "Ordering contract" table needs mirroring into the canonical Notion ICD entry —
flagged in the PR body, not attempted here (this role cannot reach Notion).

Verified: `cargo fmt --all -- --check` clean; `cargo clippy --workspace --all-targets -- -D
warnings` clean; `cargo build --workspace --all-targets` clean; `cargo test --workspace` — all
green including 7 `plant-mujoco` unit tests (3 new: dimensions, `set_ctrl` changes the
trajectory, `set_ctrl` panics on wrong length); `cargo run -p xtask -- gate` — `plant-mujoco`
still unreachable from `board-app-ridden`; `cargo build --release -p control-ffi` then `pytest
tests/ -q` → 261 passed, 2 xfailed (258 + the 3 new `test_rust_python_plant_replay_equivalence.py` tests). All run
with `MUJOCO_DIR` pointed at a local venv's pip-installed `mujoco==3.10.0` package directory
(macOS; see #112 note above for why that env var was needed locally when CI's wheel-probe
fallback alone would suffice on Linux).

PR references #106 (I1b). Not merged — auto-merge deliberately not enabled per this session's
dispatch instructions.
