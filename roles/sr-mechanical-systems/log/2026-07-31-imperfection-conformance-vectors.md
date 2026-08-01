# The imperfection profile crosses to Rust as generated vectors, and only the deterministic half is bit-identical

`crates/sim-backend` now steps the real plant through the `hal` seam (#107/#120) and carries no
imperfection profile at all — `imperfection_profile_id: None`, raw MuJoCo truth to the IMU, a
one-whole-cycle stub for actuation delay. Controls parked that correctly and said so, but the
deferral lived in its module doc — *"the real first-order current loop, which arrives with the
imperfection profile (Mechanical's territory, not this crate's)"*. **A handoff in a code comment
is not a lane.** Until it closes, SR-SIM-3's "no ideal-only mode in CI" does not hold on the
Rust-hosted path and no margin claim may be gated through it.

`crates/` is Controls' turf, so the wiring is theirs (issue #129, eight numeric ACs). The contract
is this role's and shipped as `conformance_vectors()` — reference input→output vectors for every
deterministic row of both non-ideal profiles at both timesteps — rather than prose Controls would
have to re-derive the semantics from.

**Decision (a): deterministic rows bit-identical, stochastic rows statistical.** Cutback,
saturation, transport delay, current-loop lag, quantisation and hold have exactly one right answer
and are pinned to the digit. Gyro/accel noise comes off numpy's PCG64; requiring Rust to reproduce
that stream bitwise would mean reimplementing PCG64 *and* its normal-variate algorithm, putting the
project's strictest cross-language requirement on its least consequential row. Noise conforms
distributionally, on its own seeded stream.

**Decision (b): generated, never committed.** No mech-owned path suits a JSON fixture — `/tests/`
and `/sim/` default to Controls, `sim/models/` is MJCF — and the BoM already set the precedent that
the generator is the artefact. `python -m sim.scenarios.imperfections --emit-conformance-vectors`.

The set's sharpest tooth: `np.round` is round-half-to-**even**, Rust's `f64::round` is
half-away-from-zero. They disagree at exactly the half-quantum values a quantiser lands on
constantly — 0.5→0 not 1, 2.5→2 not 3 — by one whole ERPM, in a direction that flips with the
value. Four of the nine half-quantum vectors differ. Found later, that presents as an
unreproducible Rust conformance failure that looks like a physics bug.

PR #128, issue #129 (depends on #121 for the wheel-rate half — quantising a channel that is always
zero passes every vector by accident).
