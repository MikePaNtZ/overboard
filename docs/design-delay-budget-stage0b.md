# Delay budget — replacing AC-6's plant-ignorant threshold

<!--
covers:
  - scripts/analyse_control.py
  - scripts/analyse_delay_budget.py
  - scripts/reference_disturbance.py
  - scripts/analyse_deadline_bursts.py
  - tests/test_delay_budget_stage0b.py
  - tests/test_reference_disturbance.py
reconciled: 2e41fc7
-->

- **Status:** Analysis. **AC-6 is DEMOTED, not amended — it currently gates nothing** (§3).
  The 20 ms figure is a measured capacity at a design point whose gains, `kt` and reference
  disturbance are all scheduled to change. No implementation change.
  **Addresses [#133](https://github.com/MikePaNtZ/overboard/issues/133) (AC1's time-domain half,
  AC3, AC4) — not closed: AC1's frequency-domain re-sweep is left open, see §2's correction.**
- **Owner:** Senior Controls.
- **Closes:** [#113](https://github.com/MikePaNtZ/overboard/issues/113),
  [#138](https://github.com/MikePaNtZ/overboard/issues/138),
  [#142](https://github.com/MikePaNtZ/overboard/issues/142).
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
| `p = sqrt(mgl/I)`, fixed pivot — **superseded** | 3.191 rad/s | original derivation |
| **Unstable pole `p`, identified from the model** | **5.238 rad/s** | `scripts/analyse_delay_budget.py` (#134) |

**Corrected in #134 — the fixed-pivot form understates this pole by 64%.** `sqrt(mgl/I)` is the
pendulum on a *fixed* pivot. This axle is not fixed: it rides on a wheel free to roll, so as the
board pitches the support runs **out from under** the mass instead of reacting against it. That
makes the plant *faster*, not slower.

Linearizing the actual model with `mjd_transitionFD` about the settled upright trim (contact
included) gives **5.238 rad/s**. The closed form for a pendulum on a translating support of
effective mass `M` (wheel mass plus spin inertia referred to the contact patch, `I_w/r²` =
7.31 kg) —

```
p² = m·g·l·(M + m) / ( I·(M + m) − (m·l)² )
```

— gives 5.564 rad/s, agreeing to **5.9%**, the residual between a rigid two-body idealisation
and a model with a compliant contact.

The original note said the rolling coupling is "the outer loop's job, not this pole's." That is
right about the **ride-away** mode. It is not right about the **pole**: support translation
enters the tipping mode's own characteristic equation through the `(m·l)²` term above, whether
or not any outer loop is closed.

**Validity range:** small-angle, upright, flat ground, rigid ballast — and with a hard ceiling
that is *not* the usual small-angle argument. Above **3.06°** of lean the plant saturates: the
40 A clamp is 28 N·m and gravity alone asks `m·g·l·θ = 28 N·m` there. The impulse runs peak at
5.95°, so they are deep in saturation, which is why §3 gates on the measured number.

Two standard bounds follow from `p` alone, independent of gain choice:

- **Fundamental delay-margin ceiling, `1/p` = 191 ms** (was quoted as 313 ms).
- **Textbook robust-design target, `0.2/p` = 38.2 ms** (was quoted as 63 ms).

**The second was reported as accurate to 2% against the measured 38–39 ms ceiling — that
comparison does not survive the #133 tau correction** (§3): the ceiling it was checked against
moved to 51–52 ms once the estimator was measured at the production time constant, and `0.2/p`
is now ~26% low against it. See §3 for the corrected comparison and why the original "1.6×
over-prediction" reading was still right to reject.

**Why this doesn't route through `kt`.** `KT_NM_PER_A = 0.7` (`sim/scenarios/plant.py`) is
explicitly flagged in-repo as an **unfitted guess**. A crossover-frequency derivation from the
torque-denominated gains (`kp_nm_per_rad / kt`, issue #137) would inherit that fragility
directly. `p` above needs no `kt` at all — it is pure mass/geometry/gravity — which is why it
is trustworthy where a gain-based crossover estimate would not be.

## 2. The delay budget, line items (AC2)

| Line item | Value | Status |
|---|---|---|
| Sampling + ZOH (500 Hz, `T = 2 ms`, ≈`1.5T`) | ~3 ms | **DETERMINISTIC** — follows from the ICD-fixed loop rate, not measured |
| Current-loop lag (`ImperfectionProfile.current_loop_tau_s`) | 1 ms | **PROVISIONAL placeholder** (`STAGE0_PLACEHOLDER`) — no Stage-0A bench data yet (`docs/runbook-stage0b-bench.md`) |
| CAN transport bit time (extended frame, 8-byte payload, 500 kbit/s per `docs/runbook-stage0a-bench.md`) | ~0.26–0.32 ms | **COMPUTED** from the CAN 2.0B protocol + documented bitrate. The real VESC `SET_CURRENT` frame size is **unconfirmed** — `vesc-wire`/`vesc-tx` are honest stubs (issue #1, issue #52) — so this is a generic protocol bound, not a measurement of our actual frame |
| Sensor transport (Pi 5 SPI tail, community-reported) | 1.5–2 ms | **REPORTED, not measured on our hardware** — `design-pi-image-stage0b-reference.md` AC-9 is the pending real measurement |
| Compute (one `control-core` cycle) | not separately measured | **ASSUMED negligible** relative to the items above; no isolated benchmark exists yet |
| **Estimator cost** (measured: truth-pitch margin − estimate-pitch margin) | **≈9 ms** | **MEASURED**, this issue — `tests/test_delay_budget_stage0b.py` |
| Sum of the stated line items above | ≈3 + 1 + 0.3 + 2 + 9 ≈ **15.3 ms** | — |
| **Measured total closed-loop ceiling** (ridden/cascade, nominal impulse, estimator ON — the honest default) | **survives 51 ms, strikes 52 ms** | **MEASURED**, bit-identical on repeat |
| For comparison: truth-pitch (no estimator) ceiling | **survives 59 ms, strikes 60 ms** | **MEASURED** |

**Correction (#133): the estimator cost above was measured at the wrong time constant.**
Every number in this section previously used `estimator_tau_s`'s FFI default of 1.0 s, not the
`tau = 2 s` `sim/scenarios/hill.py:140` documents as the recommended production config — nothing
had ever asked the estimator for the config that actually ships. The correction runs the
opposite direction from what was assumed going in: a longer tau trusts the delay-free gyro more
and the phase-corrupted accelerometer less (see `WheelAccelEstimator`'s comment in
`crates/control-core/src/lib.rs`), so it **costs less** delay margin, not more — the estimator
cost drops from ~21 ms to ~9 ms, and the closed-loop ceiling rises from 38–39 ms to 51–52 ms.
This holds consistently across every disturbance amplitude in §3's sweep, not just the reference
one. The truth-pitch ceiling (59–60 ms) is untouched — it does not depend on the estimator.

**The line items do not sum to the measured ceiling, and that gap is reported rather than
papered over.** ~15.3 ms of accounted-for line items against a measured ~51–52 ms ceiling
leaves **~36 ms unattributed** — a bigger unattributed share than before the #133 correction,
not a smaller one. The named line items shrank (the estimator's did); the ceiling grew by more
than they shrank. This is likely still a mix of the current clamp's nonlinearity and coupling
terms the simple additive picture does not capture, but that is now the majority of the ceiling
rather than a minority, and is worth its own measurement rather than being carried as
"unattributed" indefinitely. The **measured total** is the authoritative number regardless; the
line-item table remains diagnostic, not an exhaustive accounting.

**The estimator is a real cost, but no longer the dominant term.** ~9 ms against a 1.5–2 ms SPI
spike is roughly **4.5–6×** — still the single largest *named* line item, but well under the
previously-reported 10–14× and the "5–15×" range flagged before either measurement.
`tests/test_delay_budget_stage0b.py::test_the_estimator_costs_a_large_fraction_of_the_delay_budget`
still pins this as a standing regression gate (threshold lowered from 1.4× to 1.15×, matching the
smaller-but-real measured ratio) — a future estimator change that quietly eats more of this
margin should fail a test, not wait for a bench surprise.

## 3. Outcome (AC3)

**500 Hz is not the problem, and the honest budget says so with a wide margin, not a
knife's-edge one.** The known/assumed transport-adjacent line items (SPI 1.5–2 ms + sampling/ZOH
3 ms + CAN ~0.3 ms + current-loop lag 1 ms ≈ **6.3 ms**) sit inside the measured 51–52 ms ceiling
with roughly **8× headroom**, even before crediting the ~36 ms of unattributed slack in §2. The
originally-proposed AC-6 (`p99.9 ≤ 1 ms, max ≤ 2 ms`) was not wrong about there being a real
constraint — it was wrong about which quantity to gate on.

**Correction (#134): the textbook target did not over-predict — the pole was wrong.** This
section previously recorded the 63 ms robust target as over-predicting by ~1.6×, and blamed the
current clamp and the estimator. With §1's corrected pole, `0.2/p` = **38.2 ms**, which at the
time was compared against a measured **39 ms** ceiling — an apparent agreement of **2%**.

**Correction (#133) reopens that agreement, and it does not survive.** `0.2/p` is a property of
the plant and control law alone; it does not depend on the estimator. The 39 ms figure it was
checked against did — it was the estimator-in-loop ceiling, measured (like everything else in
§2) at the wrong tau. At the corrected, production-tau ceiling of 51–52 ms, `0.2/p` = 38.2 ms is
**~26% low**, not a 2% match. Checked instead against the estimator-independent truth-pitch
ceiling (59–60 ms, unchanged by either correction) it is **~36% low**. Neither is a close
agreement; the earlier "2%" was this document coincidentally comparing a plant-only bound
against an estimator-dependent number that happened, at the wrong tau, to land nearby. **This is
reported rather than re-explained** — a new hypothesis for the analytic/measured gap is real
analysis work, not a byproduct of a tau fix, and is left open rather than guessed at here.

The conclusion of #134 is otherwise unchanged: **gate on the measured number, not the linear
one.** What #133 removes is the specific claim that the analytic bound was validated to 2% — it
was not; that arithmetic itself was correct, but what it was being checked against was not the
right comparison. `0.2/p` remains a useful order-of-magnitude cross-check (§1's pole correction
stands on its own, independent of the estimator), just not the tight one previously reported.

**Recommendation — keep the 500 Hz schedule** (already independently justified by IMU
anti-aliasing, `crates/control-core/src/lib.rs`, `crates/board-types/src/lib.rs`; lowering it
was a FORBIDDEN outcome per the issue).

### The 20 ms figure is a measured capacity at a design point, NOT a threshold (#138)

**It is deliberately not written as a requirement, and it must not be cited as one.**

> **Measured delay capacity at the stated design point is 51–52 ms; 20 ms is well under half of
> it.** The binding hardware threshold is **re-derived after the `kt` fit (#132) and the
> reference disturbance is settled (#142)** — it does not exist yet.

All five conditions are part of the number, not context for it — the estimator's τ is added
here (#133) having previously been named in the re-open trigger below but never actually
listed as one of "the" conditions, which is how it went unmeasured at its production value for
as long as it did:

| Condition | Value | Status |
|---|---|---|
| Inner gain `kp` | 200 A/rad *(now N·m/rad, #137)* | **about to change** — #132 retune |
| Inner gain `kd` | 30 A/(rad/s) | **about to change** — #132 retune |
| `kt` | 0.7 N·m/A | **UNFITTED placeholder** |
| Reference disturbance | 20 N·s | **INHERITED, underived** — see §4 |
| Estimator `estimator_tau_s` | 2.0 s | **PRODUCTION value** (#133) — was measured at the FFI default of 1.0 s until this correction |

**Re-open trigger, in the criterion itself:** *any* change to `kp`, `kd`, `kt`, the estimator's
τ, or the reference disturbance **voids this number** and requires re-running
`scripts/analyse_delay_budget.py`. Three of those five are already scheduled to change.

**The 2× safety factor has moved from the delay axis to the disturbance axis.** §4's table is
the reason: a 1.5× change in disturbance (20 → 30 N·s) collapses the ceiling from 51 ms to
28 ms, which outweighs a 2× factor on delay entirely. **A safety factor on the wrong axis is
worse than none, because it reads as covered.** The margin that matters is the one on how hard
the board gets hit.

#### Proposed AC-6 row — handoff to the COO

`docs/design-pi-image-stage0b-reference.md` is COO turf (`CODEOWNERS`), so this is proposed,
not applied. Replace AC-6's threshold sentence with:

> **Not yet gateable.** Measured delay capacity is 51–52 ms at the design point
> (`kp`=200, `kd`=30, `kt`=0.7 *unfitted*, reference disturbance 20 N·s *inherited*, estimator
> τ=2 s *production*). A binding p99.9 threshold is re-derived once #132 (gain retune, `kt` fit)
> and #142 (reference disturbance) land. Until then AC-6 gates **nothing**, and the 20 ms figure
> may not be cited on its own to pass or fail a hardware decision.

**The test for whether this demotion actually worked** (#138's own acceptance criterion): can
anyone still cite "20 ms" alone to pass or fail a hardware decision? With the above, no — the
sentence that contains the number also contains the four conditions and the words "gates
nothing".

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
- **~~One disturbance magnitude.~~ Now swept (#134), and it is the sharpest limit here.** The
  ceiling is saturation-driven, so it is a function of disturbance amplitude, not a plant
  constant. Bisected to 1 ms, same harness as §2:

  | Impulse | Truth ceiling | Estimator ceiling | Peak pitch at zero delay |
  |---|---|---|---|
  | 10 N·s | 71 ms | 62 ms | 3.21° |
  | **20 N·s (`NOMINAL_IMPULSE_NS`)** | **60 ms** | **51 ms** | 6.36° |
  | 30 N·s | 37 ms | **28 ms** | 9.71° |
  | 40 N·s | — | — | **inverts at zero delay** |

  **Correction (#133):** the Estimator-ceiling and peak-pitch columns above are re-measured at
  the production `estimator_tau_s = 2.0` s; the Truth-ceiling column is unchanged (it does not
  depend on the estimator). At 30 N·s the estimator-in-loop ceiling is now **28 ms, above the
  20 ms figure** — the opposite of the pre-correction reading (15 ms, below it) that this
  section previously used to argue AC-6's legacy number was already unsafe at 30 N·s. It is not,
  at the production estimator config; the margin is thin (28 ms against 20 ms, an 8 ms cushion)
  but positive. This does not reopen AC-6 as a threshold — that demotion stands on the "the
  number is a design-point artefact, not a requirement" argument above, independent of which
  side of it the measurement lands on. The ceiling still falls faster than linearly with
  amplitude (~0.9 ms/N·s from 10→20, ~2.3 ms/N·s from 20→30), and 40 N·s is still fatal at zero
  delay regardless of tau. (The estimator's own cost is ~9 ms at *every* amplitude — 9/9/9 — down
  from ~22 ms (23/22/22) at the previously-used tau; still what justifies §2 charging it as a
  fixed line item.)
- **If any of these assumptions turn out to dominate the conclusion, that supersedes this
  document** — per the issue's own instruction, this analysis is not asking to be trusted past
  what it actually checked.

---

## 4. The reference disturbance (#142)

Everything above is quoted at `NOMINAL_IMPULSE_NS = 20 N·s`. **That figure was inherited from a
scenario nominal, not derived.** Its own docstring gives the game away — *"On 12.5 kg it is a
1.6 m/s delta-v — a firm shove."* 12.5 kg is the **driverless** board. On the 82.5 kg ridden
vehicle every gate actually runs against, the same 20 N·s is **0.24 m/s**, and nothing
establishes that a firm shove is the worst thing that happens to a board.

The #132 retune needs a target, and "survive a firm shove" is not one.

### The derivation — now the canonical one, not a second model

`scripts/reference_disturbance.py` used to re-derive kerb impulse itself, from a rigid-wheel
model with no angular-inertia coupling (`Δv = v·h/r`). **That has been replaced** with Sr.
Mechanical & Systems' landed answer to #142 AC2 — `kerb_strike_impulse` /
`kerb_strike_vs_com_impulse` in `sim/scenarios/plant.py`, the same model
`crates/sim-host/src/host.rs` already names as *"the kerb derivation this repo trusts... one
place"* and `tests/test_cmd_envelope_reserve.py` already drives the closed loop with. The two
models disagreed — at 20 mm/4 m/s the retired one said 45.4 N·s where the canonical bracket is
**[56.7, 87.9] N·s**, understated by 20–90% depending on height and speed. This was exactly the
two-models-that-could-disagree failure the `host.rs` comment warns about, just not yet fixed on
the Python side. `scripts/reference_disturbance.py` now imports rather than reimplements.

### What the current numbers mean as obstacles

At `r` = 145.4 mm and `M` = 82.5 kg, from the canonical bracket:

| Obstacle | Speed | Impulse (N·s) | Pitch rate imparted |
|---|---|---|---|
| 5 mm lip | 4 m/s | 20 (both models agree at small `h`) | 42 deg/s |
| 20 mm lip | 4 m/s | 57–88 | **119–222 deg/s** |
| 20 mm lip | 8 m/s | 113–176 | **239–443 deg/s** |

**The current 20 N·s reference is closer to a 5 mm crack than a kerb.** A 20 mm lip — brick edge,
root heave, the kind of thing a pavement has every few metres — is already 3–4× that in N·s, and
imparts real pitch rate a same-magnitude CoM impulse does not.

### Does the current design point survive it? (#142 AC4)

**No — measured, not just derived.**
`tests/test_cmd_envelope_reserve.py::test_criterion_a3_full_stick_during_a_kerb_strike_does_not_invert`
(ADR-0011 criterion (a) entry 3) takes this model's own impulse and pitch rate for a 20 mm lip at
hold speed, injects the equivalent force/torque into the closed-loop host, and the board inverts.
`xfail(strict=True)`, attributed to ballast stroke / CoM height being undersized — **not fixable
by input shaping**, and not the motor. This supersedes the earlier zero-delay-only derivation
below: the closed loop, with the estimator and delay in the loop, was actually run against it.

### Two caveats, pointing in OPPOSITE directions — one is now closed

1. **The wheel and step are rigid here; the real tyre is pneumatic.** A real tyre deforms over
   the edge and spreads the impulse over a longer window, so this **overstates `J`** — by how
   much is a tyre-compliance question this repo still cannot answer; there is no tyre model.
   **Still open. This is an upper bound on a rigid-wheel strike, and the gap is probably large.**
   `KERB_STRIKE_VALIDITY` in `plant.py` names the same gap explicitly.
2. **A kerb acts at the contact patch; the sim's impulse acts through the CoM.**
   `ImpulseParams.application_height_m` defaults to 0.0 so the disturbance is a pure *linear*
   impulse with zero initial pitch rate by construction. **Closed, not just flagged:**
   `kerb_strike_vs_com_impulse` derives the pitch rate a real strike imparts and states plainly
   that comparing the two on N·s alone — which this section used to do — "matches them on the
   channel that matters least." Pitch rate, not N·s, is the right currency, and §4's table above
   now reports it directly rather than inverting a mismatched unit.

Caveat (1) remains genuinely unquantified and is Sr. Mechanical & Systems' surface. Caveat (2) no
longer is — **the direction of the residual used to be unknown because the two caveats fought
each other; it no longer is, because (2) now has a number and a closed-loop measurement behind
it, and that measurement alone already inverts the board.**

### What this changes, and what it does not

- It does **not** justify lowering the envelope quietly. It says the envelope is currently
  described by a number nobody derived, and that credible obstacles are outside it — now
  confirmed in the closed loop, not only asserted from a zero-delay derivation.
- It **does** give #132 a target: the retune should be scoped against a defended disturbance,
  not against 20 N·s. #132 itself remains blocked on Sr. Mechanical & Systems' bench `kt` fit —
  unaffected by this update.
- **The tyre model is still on the critical path.** Caveat (1) is the single remaining
  uncertainty in this section, and it still sits on Sr. Mechanical & Systems' surface.

### AC2, landed (previously "open, owned elsewhere")

**What is a kerb strike worth, in N·s, for this vehicle?** Answered:
`roles/sr-mechanical-systems/log/2026-07-31-kerb-strike-reference.md` and
`sim/scenarios/plant.py::kerb_strike_impulse`/`kerb_strike_vs_com_impulse`, with tests in
`tests/test_plant_kerb_strike.py`. **The inherited 20 N·s nominal is not conservative** — it is
exceeded at every riding speed by even a modest lip, measured against the model's own lower
bound. `20 N·s` stays as the quoted reference in §1–§3 above with `inherited, underived` on its
face, since replacing it is #132's retune, not this section's call — but it is no longer an
undefended guess as to whether that reference is adequate: it is not.

---

## 5. Consecutive deadline misses (#130)

§2's ceiling was measured against a **pure, fixed** delay, and §3's limits said so: this
analysis "only speaks to the sustained shift". #130 asked for the other half — what a **burst**
of consecutive missed deadlines costs — because `hal::Observation` already reports
`missed_cycles` and nothing pre-registers an acceptable value.

`scripts/analyse_deadline_bursts.py`.

### Where it is modelled, and why not in `ImperfectionProfile` (AC1)

#130 proposed putting burst structure on `ImperfectionProfile`. It is modelled at the
**controller seam** instead, and not for turf convenience: `ImperfectionProfile` models the
**plant and its hardware** — sensor noise, quantisation, actuator lag. A missed control deadline
is none of those. It is a **compute/scheduling** failure — the task did not run, so no new
command was produced and the actuator holds the last one. The plant is behaving perfectly; the
computer is not. Putting it in the profile would place a scheduler property inside the physics
contract Sr. Mechanical & Systems owns and `sim-backend` must conform to. **If they would rather
it live there, that is their call** — flagged, not assumed.

**Deterministic worst case, not a distribution** (AC1 asks which and why):

1. **A distribution cannot be pre-registered honestly today.** There is no measured jitter
   distribution — no hardware, no `cyclictest` run. Inventing one and deriving a threshold from
   it would be exactly the failure #113 was filed about.
2. **The bound wanted is a worst case**, and "no more than K consecutive misses" is checkable
   per-cycle against `missed_cycles`. A distributional bound is not.

### The boundary (AC2)

Placement is swept before bisecting — a miss only matters while the loop has work to do. Worst
placement is **+25 ms after the disturbance**, and there:

| | |
|---|---|
| **Last surviving burst** | **114 cycles = 228 ms** |
| First fatal burst | 115 cycles = 230 ms |

**Monotonicity was verified, not assumed.** Bisection on a saturating nonlinear plant can return
an arbitrary crossing dressed up as a boundary, so the script also scans linearly around the
answer and reports the transition count: **exactly one transition**, so the boundary is real.

### A burst is ~6× CHEAPER than the same duration of constant delay (AC4)

This is the useful output, and it inverts the issue's expectation:

| Failure mode | Last surviving |
|---|---|
| Constant delay | **37.8 ms** |
| Held burst, worst placement | **228 ms** |
| **Ratio** | **6.03×** |

(The 37.8 ms constant-delay figure reproduces §2's independently — a free cross-check that this
harness and §2's agree.)

**So a burst of K cycles does NOT behave like K × 2 ms of fixed delay. It is far cheaper**,
because a constant delay costs phase on *every* cycle forever, while a burst is a transient the
loop recovers from once it ends. #130's premise — that consecutive misses are what threaten a
balancer — is not supported: **the plant tolerates 114 consecutive misses at the worst instant.**

### ⚠️ What this does NOT support: a pre-registered bound from these numbers (AC3, AC5)

#130's AC3 asked for a bound at "roughly half the measured ceiling", i.e. ~57 consecutive
misses. **That number should not be registered, for a reason the measurement itself surfaced.**

A repeating burst holds a **superset** of the cycles a single burst of the same length holds, so
it can never survive where the single one dies. It does anyway:

| Pattern | Last surviving |
|---|---|
| Single burst | 114 cycles |
| Repeating every 300 ms | **121 cycles** |
| Repeating every 500 ms | **121 cycles** |
| Repeating every 1000 ms | 93 cycles |

Two mechanisms were ruled out directly rather than argued away. It is **not** the bisection cap
(a first pass at 20/50/100 ms periods reported the cap, not the physics; these periods are all
longer than the single-burst limit). It is **not** the stale-state resume either — re-running
with the controller still executing each cycle and only its *output* held gives the same
~114-cycle boundary, so a frozen estimator and a `dt` jump on resume are not what kills it.

What is left is that the failure involves **what the controller does after a long hold** — which
a subsequent hold then interrupts. That is a real finding and it means **the failure mode is not
"too many consecutive misses"**, so a bound pre-registered off the single-burst number would be
describing the wrong mechanism.

**Per the honesty clause: the assumptions dominate the conclusion here, so this stops.** What is
solid: bursts are ~6× cheaper than constant delay, and 114 consecutive misses survive at the
worst instant. What is not solid enough to pre-register: any bound derived from those, until the
post-burst mechanism is understood.

**Interim position — bound it on schedulability, not on the plant.** AC-6a already requires the
loop to complete within its period. A missed deadline is a violation of *that*, and the sane
engineering bound (a small number of cycles) sits two orders of magnitude below what the plant
tolerates. **Consecutive misses are not the binding constraint on this vehicle**, and the
evidence says the effort belongs on disturbance rejection (§4) instead.
