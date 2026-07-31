# Delay budget — replacing AC-6's plant-ignorant threshold

<!--
covers:
  - scripts/analyse_control.py
  - tests/test_delay_budget_stage0b.py
-->

- **Status:** Proposed — analysis, one AC-6 threshold amended. No implementation change.
- **Owner:** Senior Controls.
- **Closes:** [#113](https://github.com/MikePaNtZ/overboard/issues/113).
- **Amends:** [`design-pi-image-stage0b-reference.md`](./design-pi-image-stage0b-reference.md)
  §AC-6, per the parent design's own §4 admission — *"500 Hz is an ICD number, not
  physics"* — while a pre-registered hardware go/no-go was still expressed as fractions of
  that ICD number.

## 0. The problem, in one line

AC-6 gated Stage-0B hardware on `p99.9 ≤ 1 ms, max ≤ 2 ms` — halves and wholes of the 2 ms
control period. The period is real, but nothing in that threshold refers to the **plant**: a
2 ms spike is disqualifying only if the vehicle cannot tolerate a 2 ms spike, and nobody had
measured that. This derives the number the threshold should have been.

## 1. The unstable pole (AC1)

The vehicle being built is the **RIDDEN** configuration — `BALLAST_KG, BALLAST_M = 70.0, 0.75`
in `tests/test_closed_loop.py`, the same plant the ridden-cascade gates already run against.
Unlike the bare driverless board (whose battery and hub motor put the centre of mass *below*
the axle — passively stable, per `sim/models/overboard_onewheel.xml`'s own header), a rider
puts the centre of mass **above** the axle: a genuine inverted pendulum.

`scripts/analyse_control.py:unstable_pole()` reads the compiled model directly rather than
hand-summing constants — `mgl` here is asserted to equal `plant_summary()`'s own figure, since
both come from the same `mj_forward` state:

| Quantity | Value | Source |
|---|---|---|
| Total mass | 82.5 kg | `plant_summary()` (70 kg ballast + 8 kg frame + 4.5 kg wheel) |
| CoM above axle | 633.5 mm | `plant_summary()`, MuJoCo subtree CoM |
| `mgl` (destabilising stiffness) | 512.67 N·m/rad | `plant_summary()` |
| `I` (pitch inertia about the axle) | 50.342 kg·m² | own-body `Iyy` + parallel axis, per body |
| **Unstable pole `p = sqrt(mgl/I)`** | **3.191 rad/s** | this analysis |

**Validity range, stated rather than assumed:** this is the standard small-angle
inverted-pendulum-on-a-pivot linearisation. It deliberately ignores the wheel's
rolling/translation coupling — that is the outer loop's job (`VelocityLoop`), not this pole's,
and is exactly why the inner-loop-only ridden test "rides away" (`test_closed_loop.py`) instead
of falling over: the simple pendulum model does not capture that failure mode at all. This
number is an order-of-magnitude tipping-mode estimate, not a full multi-body derivation.

Two standard bounds follow from `p` alone, independent of gain choice:

- **Fundamental delay-margin ceiling, `1/p ≈ 313 ms`** — the RHP-pole/delay theoretical limit;
  no controller can stabilise this plant against a pure delay beyond this, regardless of tuning.
- **Textbook robust-design target, `0.2/p ≈ 63 ms`** — the standard `τ·p ≲ 0.2` rule of thumb
  for a comfortable margin.

**Both bounds turned out to be loose** — see §3. They bound what *any* controller could
achieve; they say nothing about what the actual tuned, current-clamped, estimator-driven
closed loop achieves. That gap is itself the reason to measure rather than stop at the
analytic pole.

**Why this doesn't route through `kt`.** `KT_NM_PER_A = 0.7` (`sim/scenarios/plant.py`) is
explicitly flagged in-repo as an **unfitted guess**. A crossover-frequency derivation from the
current gains (`kp_a_per_rad × kt`) would inherit that fragility directly. `p` above needs no
`kt` at all — it is pure mass/geometry/gravity — which is why it is trustworthy where a
gain-based crossover estimate would not be.

## 2. The delay budget, line items (AC2)

| Line item | Value | Status |
|---|---|---|
| Sampling + ZOH (500 Hz, `T = 2 ms`, ≈`1.5T`) | ~3 ms | **DETERMINISTIC** — follows from the ICD-fixed loop rate, not measured |
| Current-loop lag (`ImperfectionProfile.current_loop_tau_s`) | 1 ms | **PROVISIONAL placeholder** (`STAGE0_PLACEHOLDER`) — no Stage-0A bench data yet (`docs/runbook-stage0b-bench.md`) |
| CAN transport bit time (extended frame, 8-byte payload, 500 kbit/s per `docs/runbook-stage0a-bench.md`) | ~0.26–0.32 ms | **COMPUTED** from the CAN 2.0B protocol + documented bitrate. The real VESC `SET_CURRENT` frame size is **unconfirmed** — `vesc-wire`/`vesc-tx` are honest stubs (issue #1, issue #52) — so this is a generic protocol bound, not a measurement of our actual frame |
| Sensor transport (Pi 5 SPI tail, community-reported) | 1.5–2 ms | **REPORTED, not measured on our hardware** — `design-pi-image-stage0b-reference.md` AC-9 is the pending real measurement |
| Compute (one `control-core` cycle) | not separately measured | **ASSUMED negligible** relative to the items above; no isolated benchmark exists yet |
| **Estimator cost** (measured: truth-pitch margin − estimate-pitch margin) | **≈21 ms** | **MEASURED**, this issue — `tests/test_delay_budget_stage0b.py` |
| Sum of the stated line items above | ≈3 + 1 + 0.3 + 2 + 21 ≈ **27.3 ms** | — |
| **Measured total closed-loop ceiling** (ridden/cascade, nominal impulse, estimator ON — the honest default) | **survives 38 ms, strikes 39 ms** | **MEASURED**, bit-identical on repeat |
| For comparison: truth-pitch (no estimator) ceiling | **survives 59 ms, strikes 60 ms** | **MEASURED** |

**The line items do not sum to the measured ceiling, and that gap is reported rather than
papered over.** ~27.3 ms of accounted-for line items against a measured ~38–39 ms ceiling
leaves ~11 ms unattributed — likely a mix of the current clamp's nonlinearity and coupling
terms the simple additive picture does not capture. The **measured total** is the authoritative
number; the line-item table is diagnostic, not an exhaustive accounting.

**The estimator is confirmed as the dominant term**, exactly as the issue predicted: ~21 ms
against a 1.5–2 ms SPI spike is roughly **10–14×**, in the "5–15×" range flagged before this was
measured. `tests/test_delay_budget_stage0b.py::test_the_estimator_costs_a_large_fraction_of_the_delay_budget`
pins this as a standing regression gate — a future estimator change that quietly eats more of
this margin should fail a test, not wait for a bench surprise.

## 3. Outcome (AC3)

**500 Hz is not the problem, and the honest budget says so with a wide margin, not a
knife's-edge one.** The known/assumed transport-adjacent line items (SPI 1.5–2 ms + sampling/ZOH
3 ms + CAN ~0.3 ms + current-loop lag 1 ms ≈ **6.3 ms**) sit inside the measured 38–39 ms ceiling
with roughly **6× headroom**, even before crediting the ~11 ms of unattributed slack in §2. The
originally-proposed AC-6 (`p99.9 ≤ 1 ms, max ≤ 2 ms`) was not wrong about there being a real
constraint — it was wrong about which quantity to gate on.

**Also worth recording: the textbook 63 ms "robust target" (§1) over-predicted the achievable
margin by roughly 1.6×** against the measured 38–39 ms. The point-mass linearisation ignores the
current clamp (`max_current_a = 40 A`) and the estimator, both real. This is the concrete
argument for gating on the measured, full-closed-loop number rather than the analytic bound —
the analytic bound is a sanity check on the measurement's order of magnitude, not a substitute
for it.

**Recommendation — keep the 500 Hz schedule** (already independently justified by IMU
anti-aliasing, `crates/control-core/src/lib.rs`, `crates/board-types/src/lib.rs`; lowering it
was a FORBIDDEN outcome per the issue). **Replace AC-6** with a plant-informed threshold: total
loop delay (sampling + transport + current-loop lag, everything upstream of `control-core`'s own
compute) should stay **≤ 20 ms** — roughly half the measured 38–39 ms ceiling, leaving a real
safety factor over both the known line items (~6.3 ms) and today's estimator cost (~21 ms
already spent). See §4 in `design-pi-image-stage0b-reference.md` for the amended AC-6 row.

### What this simulator cannot yet answer

- **No tire/ground compliance and a rigid ballast** (`tests/test_closed_loop.py`'s own module
  doc already says so) — a real rider's ankles and a real tyre both add compliance this model
  does not have, in directions that could move the measured ceiling either way.
- **The delay here is a pure, fixed transport delay** (`ImperfectionProfile.actuation_delay_s`,
  interpolated to fractional cycles), not a jittery tail-latency distribution with occasional
  spikes. AC-6's original framing conflated an isolated tail spike with a sustained shift —
  this analysis only speaks to the latter. A burst of consecutive misses correlated with
  something (not modelled here) could behave differently from an equivalent constant delay.
- **The SPI tail figure (1.5–2 ms) is a community report, not a measurement on our hardware.**
  AC-9 in the reference doc is the real measurement, still pending a Pi 5 + HAT + loopback.
- **The VESC CAN frame size is unconfirmed** — the ~0.3 ms CAN line item is a generic protocol
  bound, not derived from the real `SET_CURRENT` byte layout (still an honest stub, issue #1).
- **One disturbance magnitude, one disturbance shape.** All numbers above use
  `NOMINAL_IMPULSE_NS`; the disturbance-rejection envelope (issue #24 AC2) maps a range of
  impulse magnitudes but this delay-budget analysis was not re-run across that whole range.
- **If any of these assumptions turn out to dominate the conclusion, that supersedes this
  document** — per the issue's own instruction, this analysis is not asking to be trusted past
  what it actually checked.
