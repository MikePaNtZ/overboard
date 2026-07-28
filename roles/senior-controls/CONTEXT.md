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
