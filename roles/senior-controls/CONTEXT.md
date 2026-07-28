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
  **AC2 also now cleared (PR TBD):** the disturbance-rejection envelope is mapped, not one
  fixed magnitude — see the decision-log entry below. **Still open — issue #24's other three
  ACs, deliberately not attempted in the same PR:** the `r_eff` tyre-ground justification
  (geometry is Mechanical's turf — this role can investigate and report, not edit
  `sim/models/`); confirming every scenario already emits bit-identical metrics JSON
  (spot-checked true for impulse and closed-loop via existing determinism tests, not
  re-verified for hill/terrain/shuttle); and an audit marking every acceptance number in the
  scenario docs as measured vs. assumed. Each is its own well-scoped increment, not a blocker
  for this one.
- Closed-loop control is in sim; the estimator now closes the loop on the driverless impulse gate
  and the ridden cascade too, not only the shuttle.

## Turf notes
- Owns `crates/`, `tests/`, `scripts/`, most of `sim/` — but **not** `sim/models/`,
  `sim/scenarios/plant.py` or `imperfections.py` (Mechanical: the fidelity contract).
- **Inside `sim/models/`:** `<sensor>` and `<actuator>` elements are yours and need no row.
  Mass, inertia, geometry, contact and friction are Mechanical's and do.
- `.github/workflows/ci.yml` is yours. `.github/policy_check.py` and `CODEOWNERS` are the COO's.

## Decisions made (append as you go)

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

_Older: nothing recorded before this entry._

## Known dead ends

_Nothing recorded yet._
