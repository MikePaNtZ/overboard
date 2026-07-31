# Sr. Mechanical & Systems — working context

- **Worktree:** `~/projects/overboard-mech` (ADR-0006: one worktree per role, never share a working directory)
- **Registry:** [`docs/decisions/ROLES.md`](../../docs/decisions/ROLES.md)
- **Read first:** [`docs/decisions/INDEX.md`](../../docs/decisions/INDEX.md)

## Current sub-goals

**Everything the previous list named is closed.** The BoM shipped, the CAN HAT question is
answered, and wave-0 parts are purchased — `bench_rig.xml` was matched to what was actually
bought (#72). Those three sat here as live priorities for four days after they were done, with
the decisions recording their own closure directly below. **If you finish a sub-goal, delete it
here in the same pass** — a stale priority list is read as the current one by the next session.

- **The fidelity contract does not reach the Rust host yet.** `crates/sim-backend` now steps the
  real plant through the `hal` seam (#107/#120) and carries no imperfection profile at all —
  `imperfection_profile_id: None`, raw MuJoCo truth to the IMU, a one-cycle stub for actuation
  delay. Until that closes, SR-SIM-3's "no ideal-only mode in CI" does not hold on the Rust path
  and **no margin claim may be gated through it.** `crates/` is Controls' turf, so the wiring is
  theirs; the contract is mine and is delivered — see the conformance-vector decision below.
- **Every number in `STAGE0_PLACEHOLDER` / `STAGE0_CUTBACK` is still a placeholder.** The shape is
  the claim; the breakpoints are not measured. This is the next real mech deliverable and it is
  gated on hardware being physically in hand, not on anything in the repo.
- **Ask to be in the sim-fidelity roadmap session (#33).** It is filed CEO/COO and defines
  "representative enough" numerically — that is the fidelity contract, which is this role's.
  Being absent from the session that scopes your own surface is how turf gets decided by default.

## Turf notes
- Owns `sim/models/`, `sim/scenarios/plant.py`, `imperfections.py`, `bench_*`, `tests/test_bench_*`.
- A bench-fitted `kt` must never be written into the board model. What transfers from the bench
  is the **imperfection profile**, not motor numbers.

## Decisions made (edit in place — completed work goes in log/, not here)

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

**2026-07-28 — The riser is undersized for the >50 Hz target, and a modest gusset fixes it.**
`sim/scenarios/bench_riser.py` models the 200 mm riser as a cantilever with the motor + hub +
both confirmed goBILDA flywheels (152 g each, #65/#66) hanging off the tip, weak-axis (out-of-
plane) bending only — the axis the flywheel overhang loads, not the axis motor torque loads.
First mode comes out **39.5–42.6 Hz** across an assumed 500–650 g motor-mass range (no datasheet
in this repo states the real mass, and this environment has no web access to find one — the
honest fix is a kitchen-scale weighing, not a better guess). That is below Runbook §3.2's 50 Hz
target: **INADEQUATE as designed.** A diagonal gusset from the base plate to 80 mm up the riser
(cut from the 12×12 stock's ~60% spare per #66, no new purchase) raises it to **86.7–93.7 Hz** —
comfortably adequate. Both figures are a first-mode ROM (cantilever-beam closed form + Rayleigh
mass correction), not an FEA; a tap test with a phone microphone is the cheap way to confirm it
at the bench, and `describe_bench_signature()` says what a real riser mode looks like in the
§6c step-response data so it isn't mistaken for plant behaviour. PR pending, issue #64.

**2026-07-28 — `spindown()` gets the same settle window `identify()` got, mirrored for a decay.**
Flagged by Senior Controls (#68) while dry-running the Stage-0B runbook: unlike `identify()`,
`spindown()` had no data-driven settle window, so its friction fit ran straight through the
actuation-delay + current-loop-lag transient at the START of the coast-down. Under the default
profile that collapsed R² to ~0.002 — noise, not a curve — because the friction regression's
design matrix (`[w, sign(w)]`) has no column to absorb the transient's torque, unlike `identify()`
where the same transient only ever scaled a slope. Fix: `decay_settle_time_s`, the decay-side twin
of `settle_time_s`, waits for the *measured* current to decay to within `settle_fraction` of zero
and stay there before either the two-term or the lumped-fit diagnostic runs. Needed a tighter
`settle_fraction` than `identify()`'s 0.99 (0.9999) because there is no current column to absorb
residual bias here — 0.99 still left R²=0.635; 0.9999 gives R²=0.9999, b/tau_c error <0.05%, at a
window-open time (12.5 ms) that costs nothing against a 2 s decay. No published friction figure
had ever been derived through the STAGE0 path — only `IDEAL` was ever asserted — so nothing
downstream needed correcting, only the `scripts/stage0b_runbook.py::step_coast_down` `IDEAL`
workaround needs removing now, which is Senior Controls' file. PR #70.

**2026-07-31 — The imperfection profile crosses to Rust as generated vectors, and only the
deterministic half is bit-identical.** Controls' `sim-backend` deferred the profile to this role
by name in its own module doc, which is a handoff in a code comment — not a lane. Converted to
GitHub issue (work request) with the contract delivered as executable conformance vectors from
`conformance_vectors()`, rather than prose Controls would have to re-derive semantics from.
Two calls worth remembering:
*(a) Deterministic rows bit-identical, stochastic rows statistical.* Cutback, saturation,
transport delay, current-loop lag, quantisation and hold have exactly one right answer and are
pinned to the digit. Gyro/accel noise comes off numpy's PCG64; requiring Rust to reproduce that
stream bitwise would mean reimplementing PCG64 and its normal-variate algorithm to put the
project's strictest cross-language requirement on its *least* consequential row. Noise conforms
distributionally, on its own seeded stream.
*(b) Generated, never committed.* There is no mech-owned path a JSON fixture belongs in —
`/tests/` and `/sim/` default to Controls, `sim/models/` is MJCF — and the BoM already set the
precedent that the generator is the artefact. Emit with
`python -m sim.scenarios.imperfections --emit-conformance-vectors`.
The vectors' sharpest tooth: `np.round` is round-half-to-**even**, Rust's `f64::round` is
half-away-from-zero. They disagree at exactly the half-quantum values a quantiser lands on
constantly — 0.5→0 not 1, 2.5→2 not 3 — by one whole ERPM, in a direction that flips with the
value. Four of the nine half-quantum vectors differ. That divergence would have surfaced as an
unreproducible Rust conformance failure weeks later.

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
