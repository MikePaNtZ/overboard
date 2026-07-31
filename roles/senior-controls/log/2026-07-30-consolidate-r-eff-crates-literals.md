# Consolidate the four `crates/` r_eff_m=0.14605 FFI-boundary literals onto 0.1454

Issue #89, the ridden/hardware-path half of #76's finding: `0.14605` m — a stale nominal-tyre-
radius guess, physically impossible as a *loaded* rolling radius since it exceeds the tyre's own
unloaded geometric radius (`0.1454` m, the sim model's actual `wheel_geom` size) — was hand-copied
independently in six places across `crates/` (`sim-backend`, `control-ffi` ×3, `canary-ridden`,
`board-app-ridden`).

**One constant, one owner.** Added `board_types::DEFAULT_R_EFF_M` (`crates/board-types/src/lib.rs`)
— `board-types` was already a shared dependency of all four affected crates, so no new dependency
edge — and switched every site to import it instead of re-declaring the literal. A Rust unit test
pins `DEFAULT_R_EFF_M == 0.1454`, mirroring the Python-side regression pin; a new Python test
(`tests/test_r_eff_matches_model.py::test_rust_ffi_boundary_constant_matches_the_python_side`)
parses the Rust source for the literal and asserts it against
`sim.scenarios.rust_controller.DEFAULT_R_EFF_M`, so the two language boundaries cannot drift apart
silently again — no Python-Rust interop existed to check this any more directly.

**Re-derived, not just swapped:** `control-ffi`'s fallback feedforward gain (`kt 0.7 N·m/A / (r_eff
× 82.5 kg ridden)`) recomputed against `0.1454` m: `0.0581` → `0.0584`, a 0.45% shift — the same
magnitude #76 found on the sim side, and it does not change which regime the gain lands in.

`cargo build --workspace`, `cargo test --workspace`, `cargo clippy --workspace --all-targets -- -D
warnings`, full `pytest tests/` (257 passed, 2 xfailed, no regressions), and
`python3 .github/policy_check.py` all pass clean.

**Deliberately left out:** whether the BoardIo ICD §10.5 entry for `r_eff_m` agrees with `0.1454` —
that document lives in Notion, unreachable from this session, and #76 already flagged the same gap
on the sim side rather than guessing. Issue #88 (the earlier, less-specified duplicate of this same
finding) is superseded by this PR closing #89.

PR TBD, issue #89.
