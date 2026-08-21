# 2026-08-21 — `plant-replay`'s `chunks_exact(8)` trips a new clippy lint

Found while babysitting PR #291 (a docs-only change touching zero Rust files): its `rust`
required check failed with

```
error: using `chunks_exact` with a constant chunk size
  --> crates/plant-mujoco/src/bin/replay.rs:56:10
= note: `-D clippy::chunks-exact-to-as-chunks` implied by `-D warnings`
```

## Why this isn't #291's problem, and why it's real

`.github/workflows/ci.yml` installs Rust via `dtolnay/rust-toolchain@stable` — a floating
channel, not a pin. `#291`'s diff is one new Markdown file; `crates/plant-mujoco/src/bin/
replay.rs` is untouched by it, so the failure is toolchain drift (`stable` picked up a new
clippy lint, `clippy::chunks_exact_to_as_chunks`, first enforced at clippy 1.98.0 per the
error's own doc link) hitting pre-existing code, not anything #291 introduced. That also means
it isn't only #291's problem: the `rust` check is required, `stable` keeps moving, and every
other currently-open PR will hit the same wall the next time its CI reruns.

## The fix

Applied clippy's own suggested rewrite: `chunks_exact(8)` → `as_chunks::<8>()`. Both silently
discard a trailing partial chunk in exactly the same way (`chunks_exact`'s documented behaviour,
and `as_chunks`'s remainder half, `.1`, which the code never reads) — behaviour-preserving, not
just lint-silencing.

## Verification

- `cargo clippy -p plant-mujoco --all-targets -- -D warnings` and
  `cargo clippy --workspace --all-targets -- -D warnings` — both clean.
- `cargo test -p plant-mujoco --all-targets` — 17/17 (unchanged).
- `pip install mujoco==3.10.0` (this sandbox had no MuJoCo installed) then
  `cargo build --release -p plant-mujoco --bin plant-replay` +
  `python3 -m pytest tests/test_rust_python_plant_replay_equivalence.py` — 3/3, including the
  actual bit-identical byte comparison against the Python-hosted plant, confirming the rewrite
  changes nothing observable.
- `cargo test --workspace`, `cargo fmt --all -- --check`, `cargo run -p xtask -- gate`,
  `python3 .github/policy_check.py` — all clean (policy: advisory-only warnings, pre-existing).

## Deliberately left out

- Did not pin the CI toolchain away from `stable` — that's a `.github/workflows/ci.yml` change
  with much larger consequences (reproducibility vs. staying current) than a one-line lint fix
  warrants deciding unilaterally. Flagging it below instead.

## Also found, out of scope, flagged rather than fixed

- **The floating `dtolnay/rust-toolchain@stable` pin is a standing landmine.** Any future stable
  Rust release that adds a new default-warn clippy lint will break the required `rust` check for
  every open PR simultaneously, with zero relationship to what that PR actually changed — exactly
  what happened here. Pinning to a specific stable version (bumped deliberately, the same
  discipline `requirements-sim.txt` already applies to MuJoCo/numpy) would turn this from a
  surprise into a scheduled decision. Not done here: this is `.github/workflows/ci.yml`, which is
  Senior Controls turf per `roles/senior-controls/CONTEXT.md`, but the tradeoff (staying on
  current stable vs. pinning) is a real decision, not a mechanical fix, so it isn't bundled into
  this PR.
