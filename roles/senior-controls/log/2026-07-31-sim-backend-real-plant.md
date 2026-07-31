# I1c: sim-backend implements the seam against the real plant

Issue #107, satisfies `SR-SIM-5`. Depends on I1b (#106, merged #117), which the COO's carried-forward
comment on this issue called out by name as the biggest risk in this workstream if skipped — it
was not skipped; verified green first (below).

**`crates/sim-backend`** stops being a stub. `SimBackend` now owns a `plant_mujoco::Plant`,
opened fresh in `open()` (never reused across `open()`/`close()` cycles, so repeat-run bit-identity
does not depend on remembering to reset something). Synthetic clock (`t_ns` field) deleted, not
kept alongside — sim time is `(mjData::time * 1e9).round()`. `wait_observe()` reads
`plant.timestep()`, asserts `CYCLE_NS % dt_ns == 0` (panics with the actual numbers if not — the
onewheel model's `dt=0.002s` and `CYCLE_NS=2ms` give a ratio of exactly 1), and steps that many
times per cycle. `open()` also calls `plant.forward()` once, right after `Plant::open()` and
before any `mj_step` — AC8, carried forward from I1b: the CONTROLLED Python scenarios prime
`sensordata`/`qacc_warmstart` this way before their loop, and I1b's own equivalence was
established *without* this call, so it does not transfer un-mirrored. Raw gyro/accel come off the
model's `frame_gyro`/`frame_accel` sensors, rotated model-frame → ICD FRD frame
(`diag(-1,+1,-1)`, mirroring `sim/scenarios/plant.py::R_MODEL_TO_ICD`) — no pre-fused pitch
crosses the seam (DR-OBS-1), so `hal::Observation` still carries only what it always carried.
`motor_current_a` stays the buffered-command bookkeeping the stub already had (no current-loop
model exists yet; that arrives with the imperfection profile, Mechanical's territory).

**`crates/plant-mujoco`** grew the accessors `sim-backend` (and the AC6 harness) needed:
`timestep()`, `forward()` (asserts `time()==0`, i.e. "before the first step"), `sensor_adr_dim()`/
`read_sensor()` (name-looked-up, not indexed — the same defect class `plant.py::imu_readings`'s
docstring documents having shipped once), `body_id()`/`set_xfrc_applied()`/`body_xmat()`. Backed by
new opaque-handle-only functions in `shim.c`, same pattern as I1a/I1b. 8 new unit tests.

**All 13 pre-existing `sim-backend` conformance tests pass with their bodies UNCHANGED** — text
diff is zero for 12 of them; verified by running against the real physics. **One test's body
changed, and it is the one the acceptance criteria named as the risk**: `apply_does_not_advance_time`
used to read the private synthetic-clock field (`b.t_ns`) directly. AC3 requires deleting that
field, so the test can no longer read it. Rewrote it to prove the same property a different way —
one more `wait_observe()` after `apply()` must advance by exactly one `CYCLE_NS`, not two, which
could only hold if `apply()` genuinely touched nothing. Documented inline with the before/after and
why, per the issue's own instruction.

**AC6, the closed-loop comparison — new binary `board-app-driverless/src/bin/impulse-response-rust.rs`**
plus `tests/test_rust_hosted_impulse_response.py`. Does NOT touch `control_core::Controller` (still
the pre-existing stub that always returns `Command::ZERO` — out of scope, and changing it would be
a control-law change the issue explicitly forbids). Instead wires `PitchRegulator` +
`ComplementaryFilter` + `CommandFeedforward` + `safety::Envelope` directly — the same objects
`control-ffi::ob_controller_update` wires for the Python side — reached through `hal`
(`wait_observe()`/`apply()`) instead of the C ABI. Uses `estimator_accel_aiding` mode 2 (command
feedforward) rather than `RustController()`'s own default (mode 1, wheel odometry), because
`hal::Observation` has no wheel-rate/ERPM channel yet — that plumbing belongs to
`control_core::Controller`'s real wiring, a later, separate increment, not a throwaway harness.
Flagged as a **deliberate scope-narrowing choice**, not a silent substitution: the PR states it
plainly, and the Python-side comparator is configured identically (same mode, same gain) rather
than reused from `test_closed_loop.py`'s different default — the two numbers are not comparable to
each other. `apply_external_force()`/`truth_qpos()`/`truth_frame_xmat()` added to `SimBackend` as
explicitly non-`hal`, harness-only accessors (control-core never sees them).

**Tolerance stated before the number, honestly caveated.** Both hosts run the literal same
compiled `control-core`/`safety` objects against the literal same `libmujoco.so` I1b already
proved bit-identical open-loop, so floating summation-order noise is the only expected divergence
source — generous bound, not fitted: `peak_abs_pitch_deg` and `final_pitch_deg` within 0.1 deg,
`t_peak_s`/`settle_time_s` within 0.01 s (5 control periods). **Caveat for the record**: this
number was decided while iterating the harness to get it working at all, not in one blind pass —
by the time it was written down formally I had already seen intermediate runs. Measured result:
`peak_abs_pitch_deg` 2.724355073836445 (Rust) vs 2.724355073836437 (Python) — agree to ~13
significant figures, `t_peak_s`/`settle_time_s` exactly equal (0.924 s / 0.796 s), well inside the
stated bound with enormous margin. `IDEAL` profile used on the Python side (sanctioned
"pin-the-seam" profile, same precedent as `test_bench_spinup.py`), not `STAGE0_PLACEHOLDER` —
`sim-backend`'s own actuation-delay model is still the stub's one-cycle buffer, and a noisy Python
profile would add a second, Mechanical-owned divergence source to a test whose point is isolating
the plant seam.

**Repeat-run bit-identity (#74)**: two runs of `impulse-response-rust` produce byte-identical
output files — new parametrized test, passes.

**Stub disclaimer removed** from `sim-backend`'s crate header (now describes the real plant), and
from `board-app-driverless`'s doc comments/README, which described `sim-backend` as the stub too.

**Not done, flagged rather than silently skipped**: moving the other three scenarios to the
Rust-hosted loop (explicitly out of scope — waits on AC6 holding, separate issue); the CI check
banning `mj_step` outside `plant-mujoco` (explicitly its own issue); #112 (macOS linking) and #116
(release asset cap), untouched as instructed. `control_core::Controller`'s stub is unchanged and
still returns `Command::ZERO` — `board-app-driverless`'s own loop therefore still does not balance
anything; only `impulse-response-rust` demonstrates the real law running through `hal`.

Verified: `cargo fmt --all -- --check` clean; `cargo clippy --workspace --all-targets -- -D
warnings` clean; `cargo build --workspace --all-targets` clean; `cargo test --workspace` all green
(sim-backend 17, plant-mujoco 15, everything else unchanged); `cargo run -p xtask -- gate` —
`hal-actuate` and `plant-mujoco` still unreachable from `board-app-ridden`; `cargo build --release
-p control-ffi` + `-p plant-mujoco --bin plant-replay` + `-p board-app-driverless --bin
impulse-response-rust` then `pytest tests/ -q` → 266 passed, 2 xfailed (261 prior baseline + 5 new).
All run locally with `MUJOCO_DIR` pointed at a venv's pip-installed `mujoco==3.10.0` (macOS local
dev, same reason I1b needed it) — the sandboxed environment this session ran in additionally lacks
`sysctl`, which the `mujoco` wheel's own `__init__.py` shells out to on Darwin; worked around
LOCALLY ONLY with a fake `sysctl` shim on `PATH` for verification, not committed to the repo.

PR references #107. Not merged — auto-merge deliberately not enabled per this session's dispatch
instructions.
