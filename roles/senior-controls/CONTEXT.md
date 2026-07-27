# Senior Controls — working context

- **Worktree:** `~/projects/overboard-controls` (ADR-0006: one worktree per role, never share a working directory)
- **Registry:** [`docs/decisions/ROLES.md`](../../docs/decisions/ROLES.md)
- **Read first:** [`docs/decisions/INDEX.md`](../../docs/decisions/INDEX.md)

## Current sub-goals
- **Blocking the first public announcement:** re-run the sim with the imperfect-sensor profile.
  Every number marketing would publish today comes from a sim that knows its own tilt perfectly.
  The numbers will get worse; we must publish the honest ones first. Nobody told this role it
  was holding up a launch — that silence is the actual defect.
- Closed-loop control is in sim; the estimator closes the loop on the shuttle.

## Turf notes
- Owns `crates/`, `tests/`, `scripts/`, most of `sim/` — but **not** `sim/models/`,
  `sim/scenarios/plant.py` or `imperfections.py` (Mechanical: the fidelity contract).
- **Inside `sim/models/`:** `<sensor>` and `<actuator>` elements are yours and need no row.
  Mass, inertia, geometry, contact and friction are Mechanical's and do.
- `.github/workflows/ci.yml` is yours. `.github/policy_check.py` and `CODEOWNERS` are the COO's.

## Decisions made (append as you go)

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
