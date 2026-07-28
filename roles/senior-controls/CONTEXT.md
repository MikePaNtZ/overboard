# Senior Controls — working context

- **Worktree:** `~/projects/overboard-controls` (ADR-0006: one worktree per role, never share a working directory)
- **Registry:** [`docs/decisions/ROLES.md`](../../docs/decisions/ROLES.md)
- **Read first:** [`docs/decisions/INDEX.md`](../../docs/decisions/INDEX.md)

## Current sub-goals
- **Was blocking the first public announcement, now partly cleared (PR TBD, issue #24 AC1):**
  `RustController`'s `use_estimator` default flipped `False` → `True`, so the driverless
  disturbance-rejection gate (`tests/test_closed_loop.py`), the ridden cascade gate, and
  `scripts/render_scenario.py --compare` — the exact script that produces the artifact CI
  publishes to `sim-latest` — now all run on the attitude **estimate**, not ground truth, by
  default. hill.py/terrain.py/shuttle_run.py already did this; the driverless impulse gate,
  which is what the published render and its `impulse_closed_loop_metrics.json` actually show,
  did not. One pinned regression threshold moved with it and was re-measured rather than
  loosened blindly: `test_there_is_large_margin_on_actuation_delay` was pinned at 6.80 deg
  (truth pitch) and is now 9.71 deg (the honest number) — still clear of the 18.57 deg strike
  angle, re-baselined to `< 10.5`.
  **AC2 also now cleared (#67):** the disturbance-rejection envelope is mapped, not one fixed
  magnitude. **AC3 also now cleared (see the decision-log entry below):** the `r_eff` tyre-ground
  question — couldn't be justified, so fixed. **AC4 addressed in a separate open PR (#74, not
  yet merged as of this entry):** determinism audited across all four scenarios. **Still open —
  issue #24's one remaining AC:** an audit marking every acceptance number in the scenario docs
  as measured vs. assumed. Its own well-scoped increment, not a blocker for any of the above.
- Closed-loop control is in sim; the estimator now closes the loop on the driverless impulse gate
  and the ridden cascade too, not only the shuttle.

## Turf notes
- Owns `crates/`, `tests/`, `scripts/`, most of `sim/` — but **not** `sim/models/`,
  `sim/scenarios/plant.py` or `imperfections.py` (Mechanical: the fidelity contract).
- **Inside `sim/models/`:** `<sensor>` and `<actuator>` elements are yours and need no row.
  Mass, inertia, geometry, contact and friction are Mechanical's and do.
- `.github/workflows/ci.yml` is yours. `.github/policy_check.py` and `CODEOWNERS` are the COO's.

## Decisions made (append as you go)

- **Issue #24, AC3 — `r_eff` tyre-ground justification (PR TBD).** Could not be justified, so
  fixed rather than justified. `R_EFF_M`/`DEFAULT_R_EFF_M` (converts `wheel_hinge` angular rate
  to forward speed, and current to force, in `hill.py`/`terrain.py`/`shuttle_run.py`/
  `rust_controller.py`/`tests/test_closed_loop.py`/two `scripts/analyse_estimator*.py`) was
  hand-copied as `0.14605` m in every one of those places, one of them (`hill.py`) claiming in a
  comment that it "matches the tire in the model header." It did not: the model's actual
  `wheel_geom` radius (`sim/models/overboard_onewheel.xml`, Mechanical's turf, not edited) is
  `0.1454` m, the mesh-derived, enclosure-clearance figure that file's own header documents. A
  *loaded* rolling radius bigger than the tire's own unloaded geometric radius is not physically
  sensible regardless of provenance — compression under load only ever shrinks it — so this was
  a stale figure (an 11.5" OD / 2 nominal-spec guess) that predated the mesh integration and was
  never checked against the model everything else in this repo is measured against.
  Consolidated to one constant (`rust_controller.DEFAULT_R_EFF_M`), imported everywhere instead
  of re-declared, and added `tests/test_r_eff_matches_model.py`, which reads the *compiled*
  model's `wheel_geom` size directly rather than re-asserting a literal, so this cannot silently
  drift again. Full suite re-run clean (205 passed, 2 xfailed, no regressions; no pinned
  threshold moved — the shift is 0.45%, inside every existing tolerance).
  **Deliberately left out, flagged rather than fixed:** the same `0.14605` literal also appears,
  independently, four times in `crates/` (`sim-backend`, `control-ffi` ×2, `canary-ridden`,
  `board-app-ridden`) as FFI-boundary defaults — real hardware/ridden-mode code, not sim
  scenarios, and touching it needs its own increment and a decision on which crate should own a
  shared constant (this PR does not invent one). Not part of issue #24's AC3, which is scoped to
  the sim scenarios; reported separately in the PR as an out-of-scope finding.
  **Could not verify:** what the BoardIo ICD §10.5 entry for `r_eff_m` actually specifies — that
  document lives in Notion, which this session has no access to. This fix only pins internal
  consistency against the sim model; it does not claim to have confirmed or superseded whatever
  Stage-0's eventual bench measurement will produce.
  AC5 (measured-vs-assumed audit of every scenario-doc acceptance number) remains open, its own
  increment.

- **Issue #24, AC2 — disturbance-rejection envelope (PR TBD).** Added
  `sim/scenarios/disturbance_envelope.py` (`sweep_closed_loop`, `EnvelopeResult`),
  `tests/test_disturbance_envelope.py`, and `scripts/disturbance_envelope.py`. A grid sweep
  (20 N*s steps, `NOMINAL_IMPULSE_NS` to 320 N*s) on the driverless closed-loop board, not a
  bisection to the exact recovery point: measured directly that `nose_strike` is **not
  monotonic** in impulse magnitude near the 18.57 deg strike angle (280 and 300 N*s struck,
  290 and 320 did not — `peak_abs_pitch_deg` sits within ~1 deg of the strike angle across
  that whole band), so a bisection would converge on an arbitrary knife's-edge crossing that
  moves with any small upstream change without the controller's real margin having changed. The
  pinned number is the honest measured one: recovery boundary **260 N*s**, first sampled
  failure **280 N*s** — 13x `NOMINAL_IMPULSE_NS`. Re-run twice to confirm the sweep itself is
  deterministic before pinning.
  **Deliberately left out:** the ridden/cascade plant (issue #24 only asked about the
  driverless gate here — `test_closed_loop.py`'s ridden section is a separate, larger
  configuration space); the `r_eff` justification, determinism audit and assumption audit are
  issue #24's other three ACs, each its own increment.

- **Issue #32, Stage-0B Pi image design (`docs/design-pi-image-stage0b.md`).** Design only, no
  implementation. Repo boundary: a `pi/` directory **in this repo**, argued on the *runtime
  contract* (kernel flavour, `isolcpus` layout, RT priority budget, CAN naming/bitrate, systemd
  expectations) — assumptions the Rust code encodes and only this repo's CI can test end-to-end.
  Needs one COO `CODEOWNERS` line; `scripts/pi/` is the non-blocking fallback. Deliverable:
  **an image + checksum + provenance, produced exclusively by a CI-scripted flow** — never a
  flash-time provisioning script, which would make live-mirror state a runtime input.
  **Kernel finding that matters:** Raspberry Pi *do* officially package PREEMPT_RT
  (`linux-image-rpi-v8-rt`), so RT on Pi is no longer a community build — but **there is no
  `-2712-rt` flavour**, so RT on a Pi 5 means the generic 4K-page v8 kernel with no
  `NO_HZ_FULL`. Verified by unpacking the .deb: RP1 support and the whole CAN stack are intact
  in that flavour, so the design holds. **Biggest open risk: an unresolved Pi 5 report of RP1
  SPI transactions spiking to 1.5–2 ms under load — a full control period at 500 Hz.** Answered
  by carrying USB-CAN as a second transport (a control variable, not a contingency) and by
  running a `spidev` reproduction *before* the CAN HAT is bought.
  **Deliberately left open rather than invented:** the §7.2 current limits are `PROVISIONAL`
  placeholders — no Stage-0A data exists yet — and Little FOCer command-timeout semantics are
  unverified, so the safety case rests on the deadman and the controller current ceiling, not
  on the software timeout.

- **Issue #1, crate-exclusion boundary (PR TBD).** Split `hal` -> `hal` (observe) +
  `hal-actuate` (motion authority); split `board-app` -> `board-app-ridden` +
  `board-app-driverless`; added `vesc-wire` (decode-only), `vesc-tx` (encode-only),
  `canary-ridden` (positive control) and `xtask` (the `cargo metadata`
  dependency-graph gate). `board-app-ridden` has no observe-only hardware
  backend yet, so it runs against a local `ShadowBackend` placeholder —
  intentional, not an oversight; a real one is later, unrelated work.
  **Deliberately left undone, flagged rather than fixed silently:** `vesc-wire`
  / `vesc-tx` carry no real VESC byte layouts — there is no hardware yet to
  verify one against, and fabricating protocol constants from memory into a
  crate that will gate real actuation was judged worse than leaving them
  honest stubs. ICD §6.3's "drop symbol scanning" line is a Notion-only edit
  this session had no Notion access to make. `README.md` still says
  `cargo run -p board-app` — stale after the split, but `README.md` is CEO
  turf, not mine to edit.

- **Issue #52, virtual CAN harness (PR TBD).** New crate `can-harness`: a
  self-managed `vcan` bring-up/teardown (via `socketcan`'s netlink API, no
  `ip link` shell-out), a simulated VESC-shaped responder on a background
  thread, and transport-level round-trip + timeout tests. **Deliberately did
  not fill in `vesc-wire`/`vesc-tx`'s real byte layouts.** Tried to source
  them from the official (non-firmware) "VESC 6 CAN Formats" PDF at
  vesc-project.com — every fetch of that site 403'd from this session, for
  every page on the domain, not just that one file, so I could not confirm
  it either way. Left the stubs as they were and tested the transport layer
  (sign/scale byte fidelity through a real vcan round trip) instead, per the
  issue's own fallback instruction.
  **Deliberately left undone:** did not add a privileged CI step to make the
  vcan tests run for real. This sandbox returns `Operation not supported`
  for `vcan` interface creation even under `sudo` — looks like the container
  kernel has no `vcan` support at all, not just a permissions gap — so I had
  no way to verify a privileged step would actually work on the GH-hosted
  runner, and didn't want to gamble a required check on an unverified guess.
  The crate rides the existing unprivileged `cargo test --workspace` step,
  where every vcan test is expected to print `SKIP` and pass — which is
  itself the acceptance criterion for "vcan unavailable." Turning that into
  a live run on `ubuntu-latest` (it likely needs `CAP_NET_ADMIN`, which a
  plain job step doesn't have either) is flagged, not fixed, here.

- **Issue #54, Stage-0B design doc split (PR TBD).** Design doc was at 39,633/40,000 chars —
  367 bytes of headroom, one sentence from breaking the build. Split into the decision doc
  (D1–D6, rationale, rejected alternatives; now ~19.8k) and a new companion
  `docs/design-pi-image-stage0b-reference.md` (~24.9k) carrying the schemas, exact version
  pins, AC table, verifiability table, credentials, data path and open-question ledger — same
  shape as the existing `-verification.md` split. Nothing cut, only relocated; every internal
  cross-reference updated to resolve across the new file. `TURF-OVERRIDE`'d against `docs/`
  (COO turf) on the same precedent the original design doc used for issue #32.
  **Left for a future pass, not invented here:** the design doc landed at 19,814 chars — under
  ADR-0008's 20,000 warning line, but not with the multi-thousand-character margin the rest of
  the split enjoys. Getting real headroom there would mean moving Safety §7.2–7.4 material
  (already done) plus trimming further into D1/D2/D3/D6, which starts trading decision
  rationale for size and wasn't worth doing without a second opinion.
- **Issue #27, O4 — Stage-0B bench-test runbook (`docs/runbook-stage0b-bench.md`).** The
  Pi-executed counterpart to Stage 0A's human checklist: ordered steps with purpose/falsifies/
  pass-fail, abort criteria stated before the first powered step, a small JSON log schema
  traceable to `git_sha`, and a sim dry run (`scripts/stage0b_runbook.py`,
  `tests/test_stage0b_runbook.py`) exercising the sim-representable steps (current-step
  response, coast-down, command→actuation latency) before hardware exists. AC-6's thresholds
  (p99.9 ≤ 1 ms, max ≤ 2 ms, ≥10⁵ cycles) are reused verbatim from the ratified
  `docs/design-pi-image-stage0b.md`, not re-derived.
  **Finding, flagged rather than fixed:** `bench_spinup.spindown()` (Sr. Mechanical & Systems'
  turf) has no equivalent of `identify()`'s data-driven `settle_time_s` windowing, so fitting
  its decay under `STAGE0_PLACEHOLDER` runs the least-squares fit straight through the
  actuation-delay + current-loop-lag transient at the start of the decay — R² collapsed to
  ~0.002 (noise, not a curve) in testing here. The coast-down dry run uses `IDEAL` instead,
  the only profile `spindown()` is actually validated against today.
  **Deliberately left out:** the CAN round-trip step (2) is not re-implemented — it depends on
  `can-harness` (issue #52, PR #53, not yet merged) and is documented as covered there rather
  than duplicated. No `--hardware` mode exists; there is no Pi image yet to run it against
  (O3, issues #51/#52 still open), and a stub with nothing to execute it would be exactly the
  unverifiable code this project rules out.

- **Issue #62, target-gate `can-harness`'s `socketcan` dependency (PR TBD).** Follow-up from
  #53: `socketcan` was an unconditional workspace dependency, so `cargo build --workspace` failed
  on macOS (the CEO's machine). Moved `socketcan` to a
  `[target.'cfg(target_os = "linux")'.dependencies]` table in `crates/can-harness/Cargo.toml` and
  `#[cfg(target_os = "linux")]`-gated every item in `src/lib.rs` that touches it (`responder`,
  `vcan`, `to_can_frame`/`from_can_frame`, their unit test); `tests/vcan_stack.rs` gets
  `#![cfg(target_os = "linux")]` so it compiles to zero tests off Linux instead of failing to
  compile against a crate with no socketcan items. **Verified for real, not assumed:** this
  sandbox has no macOS runner, so verification used `rustup target add x86_64-apple-darwin` and
  `cargo check`/`cargo clippy --workspace --all-targets --target x86_64-apple-darwin -- -D
  warnings` — both clean, and `can-harness` compiles to an empty crate on that target as
  intended. Linux side re-verified unchanged: `cargo test -p can-harness` still runs all four
  vcan tests and prints `SKIP` for each (no `CAP_NET_ADMIN`/`vcan` module in this sandbox, the
  expected unprivileged outcome), `cargo run -p xtask -- gate` still passes.

_Older entries collapsed above this line as the log grows; nothing predates the crate-exclusion entry._

## Known dead ends

- **Fetching vesc-project.com for the VESC 6 CAN Formats PDF (2026-07-28).**
  `WebFetch` returned HTTP 403 for every URL tried on that domain (the PDF
  itself, and two different documentation pages) — looked like the fetcher
  was being blocked outright rather than any one page being gated. Did not
  find a working alternative source for real, non-GPL-firmware VESC CAN
  byte layouts in the time this session had. Next session: worth trying a
  different fetch path (cache, a mirror, or asking Mike for a manual pull)
  before assuming the constants are simply unobtainable.
