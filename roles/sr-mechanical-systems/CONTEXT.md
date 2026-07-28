# Sr. Mechanical & Systems — working context

- **Worktree:** `~/projects/overboard-mech` (ADR-0006: one worktree per role, never share a working directory)
- **Registry:** [`docs/decisions/ROLES.md`](../../docs/decisions/ROLES.md)
- **Read first:** [`docs/decisions/INDEX.md`](../../docs/decisions/INDEX.md)

## Current sub-goals
- **The BoM is the critical path and the CEO has called it out.** It was written as a reasoning
  document; you cannot shop from a reasoning document. Split into two versioned sheets, identical
  columns — `Part · Part number · Qty · Unit price · Link · Status · Rev` — as **BoM-BENCH-001**
  (test stand, ~$200–250) and **BoM-BOARD-001**. All prose moves to a linked decision log.
- **$0 ordered to date.** Hardware ordered / delivered is the first row of the board doc.
- Verify a Pi 5 / RP1-compatible CAN HAT exists and ships before any board is bought. Blocks everything.

## Turf notes
- Owns `sim/models/`, `sim/scenarios/plant.py`, `imperfections.py`, `bench_*`, `tests/test_bench_*`.
- A bench-fitted `kt` must never be written into the board model. What transfers from the bench
  is the **imperfection profile**, not motor numbers.

## Decisions made (append as you go)

**2026-07-26 — The BoM is delivered as a spreadsheet, not a document.** Four sheets (Read me /
BoM / Tools / Inventory), clickable Status dropdowns, real vendor links, spares as a deliberate
column. Wave 0A (bench mechanical) + 0B (bench signal path) ≈ $1,282, or $1,582 with the
controller; wave 1 (the board) ≈ $900 and stays unordered. Delivered as a file to the CEO rather
than committed — binaries do not go in git, and the generator script is the reproducible artefact.
*Deviation to reconcile:* this sub-goal asked for two separately versioned sheets
(`BoM-BENCH-001` / `BoM-BOARD-001`). It shipped as one sheet with wave sections plus a filter,
because the Inventory sheet has to reconcile against a single ref space. If the CEO wants the
split, it is a small change to the generator.

**2026-07-26 — The Pi 5 CAN HAT question is closed, and it was the thing blocking everything.**
Waveshare 2-CH **isolated** CAN FD HAT (MCP2518FD + MCP2562FD), listed Pi 5 compatible, mainline
`mcp251xfd`, `dtoverlay=mcp251xfd,spi0-0,interrupt=25`. The RP1 breakage that killed older
overlays does not apply. Take the isolated variant — it shares a ground with a motor controller.
Known caveat, `raspberrypi/linux` #6407 reports receive latency on this driver: that is not a
reason to avoid it, it is the first thing the rig should measure.

**2026-07-26 — There is no Little FOCer Rev4.** The shipping product is **Rev3.1**, so the open
thread about "Rev4 continuous current" was mis-specified and is closed on the merits: Rev3.1 is
100 A continuous (ext. cooled) / 250 A peak, far above our DC budget even after the 1.45×
re-derivation. It is $300 and was **sold out** at MakersPEV — long-lead, order early.
**Hard floor: 28 V minimum input.** A 24 V bench supply will not run it.

**2026-07-26 — The bench mount is an L, not a single plate.** `bench_rig.xml` draws it as one
vertical panel "clamped flat to the desk", which is not buildable — you cannot clamp a vertical
panel to a horizontal desktop. Real bracket: base plate the clamps grip + riser carrying the
motor, joined by two short angles. The model geom is flagged visual-only (treated as rigid
ground), so **no sim number changes**; this is a buildability fix. Bench motor shaft is **8 mm
keyed** (3×3×20), M4 mount — not 10 mm.

**2026-07-26 — Frame conventions are derived, never fitted.** `plant.imu_readings` shipped
`diag(+1,+1,−1)` — determinant −1, a *reflection*, not a rotation — which produced a left-handed
frame against ICD §10.1's explicit "right-handed". It inverted accel-derived pitch to exactly −θ
and inverted roll rate (latent: nothing reads roll until lean-to-steer). Correct map is
`diag(−1,+1,−1)`, a 180° rotation about +Y, because the model is z-up with **forward = −X**.
Amended ICD §10.1/§10.3: the sim arbitrates **dynamics, not handedness**, and every frame
conversion must be pinned by a derivation-based test. Merged as #12.

**2026-07-26 — Fit the settled ramp, and the measured current.** Bench `kt` came out **50.4% low**
on the default profile: 1 ms actuation delay + 1 ms current-loop τ against a 5 ms window
suppressed both ramps by k≈0.5, which *cancels* in `J_bare` (`kt·i/α₁`) but scales `kt`
linearly. Fix is two parts — open the window when the **measured** current has settled, detected
from the current trace rather than the profile (hardware has a sensor, not a profile object), and
fit against measured rather than commanded current. 50.4% → 0.13%. Merged as #13.

**2026-07-26 — Tilted ground exists alongside rotated gravity, not instead of it.** Controls'
rotated-gravity hill was the right first call and stays. Added a genuinely tilted plane because
rotated gravity **cannot be filmed** (renders as flat ground with the board accelerating for no
reason) and can only ever be an infinite uniform plane. The two are independent formulations and
now cross-check: free-roll agreement 0.24–1.78% to 20% grade, bit-identical on the flat.
`strike_angles_deg()` returns nose and tail separately; the descent-fails-tail / climb-fails-nose
asymmetry Controls measured falls out of geometry, because a board resting flat on a slope is
already at world pitch −φ. PR #18.

**2026-07-28 — `bench_rig.xml` now matches the parts actually purchased (#66).** Flywheel geometry
was a 75 mm/12 mm custom aluminium disc estimate; the confirmed part is two goBILDA
3628-0032-0082 units (82 mm OD, 152 g, 1651 g·cm² each, manufacturer-published). A same-mass solid
disc at that radius computes to ~1277 g·cm² — 30% low — because the real part is a hub-and-rim
casting, not a uniform disc, so each flywheel now carries an explicit `<inertial>` sourced directly
from the datasheet rather than a density-derived guess. Plate stock corrected to the confirmed
12″×12″ ¼″ 6061-T651 sheet (thickness 6 mm→6.35 mm exact); clamps confirmed as IRWIN Quick-Grip
mini, noted as a second candidate resonance source alongside the riser (#64). Rotor can/hub stay
geometry-derived estimates — no manufacturer figure exists for them, that's what the bare-run
measurement is for.

**Before/after, measured not assumed:** J_disc 1.6103e-3 → 3.3020e-4 kg·m² (matches 2×1651 g·cm²
exactly), J_loaded 1.8510e-3 → 5.7085e-4 kg·m², ratio J_disc/J_bare 6.69 → 1.37. That drop is
expected, not a regression — #65 already flagged 1.1–1.65 from these same confirmed figures and
owns the decision of whether to accept it or add inertia; this issue only made the model match
what's on the shaft. `sim/scenarios/bench_spinup.py`'s `known_disc_inertia_kg_m2()` now returns the
manufacturer figure too (was a radius/thickness/density formula), since real hardware would know
this from a datasheet, not a ruler. 202 tests pass, same count as master — updated in place, not
added, since the affected assertions are model-inertia values, not new coverage.

## Known dead ends

- **Do not lift the board clear of the ground to isolate accelerometer geometry.** A free-floating
  body is in free fall, so the accelerometer correctly reads ~0 and every derived angle is
  atan2 of noise. Use an analytic specific force (`+g·ẑ_world` rotated into the body frame), or
  a supported quasi-static pose at small tilt.

- **Do not validate a frame convention on quiet, near-upright samples.** A sign error vanishes at
  θ=0 and grows as 2θ, so quiet-sample validation is validation in the one regime where the defect
  is invisible. Undisturbed estimator error was bit-identical before and after the frame fix. This
  is exactly how `diag(+1,+1,−1)` shipped with a recorded "~2° RMS residual" that *was* the bug.

- **Do not monkeypatch a symbol at one import site and call it a blast radius.** Patching
  `impulse_response.imu_readings` missed `shuttle_run.py`, which imports it separately — I reported
  1 affected test when the real number was 4, and had to correct an escalation row. Patch at the
  source module or edit the file.

- **Do not tilt the ground without raising the axle.** Perpendicular distance from the un-raised
  axle to a plane through the origin is `h·cos φ` — short of the rolling radius — so MuJoCo
  resolves it as penetration and ejects the board on step one. The XML still looks correct, and
  the failure presents as a physics problem rather than a placement one.

- **A bench-fitted `kt` must never reach `plant.KT_NM_PER_A`.** Restating it here because it is the
  single most likely category error in this project, and the guard is a comment, not code.
