# CI fix — publish-sim-artifact missing the Rust-hosted impulse-response build step (PR TBD)

Found while watching PR #122's checks: `publish-sim-artifact` was failing on that PR, and on
every push to master since #120 (I1c) merged — confirmed via `actions_list`/`get_job_logs`
against the last three master runs (`d1dfddd`, `14559c4`, `32f4d71`), all `conclusion: failure`
on this job specifically, while `sim` and `rust` stayed green.

**Root cause:** #120 added `crates/board-app-driverless/src/bin/impulse-response-rust.rs` and
`tests/test_rust_hosted_impulse_response.py`, and wired a "Build the Rust-hosted impulse-response
binary" step into the `sim` job (`.github/workflows/ci.yml:158-163`) — but the `publish-sim-artifact`
job's own "Emit claims manifest" step also runs `pytest tests/` (per its existing comment) and
never got the equivalent build step. The new test file's own "fail rather than skip" binary-exists
check (added by #117's precedent) correctly failed the suite there, silently, because
`publish-sim-artifact` is deliberately not a required status check (CLAUDE.md) — so nothing
blocked on it and nobody saw it.

**Fix:** added the same `cargo build --release -p board-app-driverless --bin
impulse-response-rust` step to `publish-sim-artifact`, mirroring the `sim` job's existing step
and comment style. Verified locally: rebuilt all three binaries
(`control-ffi`/`plant-replay`/`impulse-response-rust`) and ran `scripts/emit_claims.py` (the exact
step that was failing) — 266 passed, 2 xfailed, `wrote 1 claim to sim/out/claims.json`.

**Deliberately kept separate from PR #122** (the delay-budget work) rather than bundled in —
this is a pre-existing, unrelated regression, not something #122's diff touches or caused.

**Not fixed, flagged instead:** whether `publish-sim-artifact` should become a required check now
that it would actually be green — that's a policy/CODEOWNERS-adjacent question outside a single
CI-step fix, and not this PR's job to decide.
