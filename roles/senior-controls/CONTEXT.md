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
  **Still open — issue #24's other four ACs, deliberately not attempted in the same PR:**
  disturbance-rejection **envelope** (currently one fixed magnitude, `NOMINAL_IMPULSE_NS = 20`,
  not a sweep to a recovery boundary); the `r_eff` tyre-ground justification (geometry is
  Mechanical's turf — this role can investigate and report, not edit `sim/models/`); confirming
  every scenario already emits bit-identical metrics JSON (spot-checked true for impulse and
  closed-loop via existing determinism tests, not re-verified for hill/terrain/shuttle); and an
  audit marking every acceptance number in the scenario docs as measured vs. assumed. Each is
  its own well-scoped increment, not a blocker for this one.
- Closed-loop control is in sim; the estimator now closes the loop on the driverless impulse gate
  and the ridden cascade too, not only the shuttle.

## Turf notes
- Owns `crates/`, `tests/`, `scripts/`, most of `sim/` — but **not** `sim/models/`,
  `sim/scenarios/plant.py` or `imperfections.py` (Mechanical: the fidelity contract).
- **Inside `sim/models/`:** `<sensor>` and `<actuator>` elements are yours and need no row.
  Mass, inertia, geometry, contact and friction are Mechanical's and do.
- `.github/workflows/ci.yml` is yours. `.github/policy_check.py` and `CODEOWNERS` are the COO's.

## Decisions made (append as you go)

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

_Older: nothing recorded before this entry._

## Known dead ends

_Nothing recorded yet._
