# Delay-budget corrections — the pole, and the amplitude it is quoted at (PR #134)

**Follow-up to #122, which landed while this was in flight.** Additive: it does not restate
the budget and does not re-litigate AC-6's 20 ms.

**This is a HANDOFF.** The corrections below belong in `docs/design-delay-budget-stage0b.md`,
which is COO turf (`CODEOWNERS:52`). They are written out here, in Senior Controls' own tree,
rather than applied directly — the `policy` turf gate correctly refused the cross-turf edit,
and `TURF-OVERRIDE` would have been me authorising myself, which is the thing that gate exists
to stop. **The COO applies §A and §B, or rejects them.**

Reproduce every number with:

```
.venv/bin/python scripts/analyse_estimator_phase.py --out-dir sim/out/experiments
.venv/bin/python scripts/analyse_delay_budget.py    --out-dir sim/out/experiments
```

The second reads the first's saved sweep, so they cannot drift apart silently, and it exits
non-zero rather than reporting a number if its cross-check on the plant fails.

---

## A. The unstable pole is 64% higher than §1 states

§1 uses `p = sqrt(mgl/I)` — the pendulum on a **fixed** pivot — for **3.191 rad/s**, noting it
deliberately ignores rolling coupling as *"the outer loop's job, not this pole's."*

**That holds for the ride-away mode. It does not hold for the pole.** The axle rides on a wheel
free to roll, so as the board pitches the support runs *out from under* the mass rather than
reacting against it. Support translation therefore appears in the tipping mode's **own**
characteristic equation, through the `(m·l)²` term below, whether or not any outer loop is
closed.

Linearizing the actual model with `mjd_transitionFD` about the settled upright trim (contact
included), and cross-checking against the closed form for a translating support of effective
mass `M` = wheel mass + `I_w/r²` = 7.31 kg:

```
p² = m·g·l·(M + m) / ( I·(M + m) − (m·l)² )
```

| Form | Value |
|---|---|
| `sqrt(mgl/I)`, fixed pivot (§1 today) | 3.191 rad/s |
| **Identified from the model** | **5.238 rad/s** |
| Closed form, translating support | 5.564 rad/s — agrees to **5.9%** |

The 5.9% residual is a rigid two-body idealisation meeting a model with a compliant contact.

### The consequence, which is the reason this is worth a correction

§3 records that the textbook `0.2/p` robust target *"over-predicted the achievable margin by
roughly 1.6×"*, and attributes that to the linearisation ignoring the current clamp and the
estimator.

**The attribution is wrong.** With the corrected pole:

| | §1's pole | Identified pole | Measured |
|---|---|---|---|
| `0.2/p` robust target | 62.7 ms | **38.2 ms** | **39 ms** |
| `1/p` fundamental ceiling | 313 ms | **191 ms** | — |

`0.2/p` lands within **2%** of the measured ceiling. The rule was never loose — the pole was
64% low, and a plausible physical effect got fitted to the resulting error. The clamp and the
estimator are real and do matter (the estimator costs ~21 ms, exactly as §2 measured), but they
are not what explains that gap, because with the right pole there is no gap to explain.

**§3's actual conclusion is unchanged and still correct**: gate on the measured number. What
changes is that the analytic bound becomes a sharp cross-check rather than expected slack — a
future measurement drifting far from `0.2/p` should now read as a signal that something moved.

### Proposed edit

- §1: keep the `sqrt(mgl/I)` row, mark it superseded, add the identified pole and the closed
  form above. Replace the two bounds with **191 ms** and **38.2 ms**.
- §1 validity paragraph: the rolling-coupling sentence is right about ride-away, wrong about
  the pole — narrow it to the former.
- §3: replace the "over-predicted by 1.6×" paragraph with the table above.

---

## B. AC-6's 20 ms does not survive a 1.5× disturbance

§3's limits list *"one disturbance magnitude, one disturbance shape"* as unmeasured. It is now
measured, and it is the sharpest limit on the document. The ceiling is saturation-driven, so it
is **not a plant constant**. Bisected to 1 ms, same harness as §2:

| Impulse | Truth ceiling | Estimator ceiling | Peak pitch at zero delay |
|---|---|---|---|
| 10 N·s | 71 ms | 48 ms | 2.94° |
| **20 N·s (`NOMINAL_IMPULSE_NS`)** | **60 ms** | **38 ms** | 5.95° |
| 30 N·s | 37 ms | **15 ms** | 8.98° |
| 40 N·s | — | — | **inverts at zero delay** |

Ceiling = last surviving delay. The 20 N·s estimator figure reproduces §2's exactly; the
truth-pitch figure lands 1 ms above §2's 59 ms — bisection-boundary sensitivity, not a
disagreement.

**At 30 N·s the estimator-in-loop ceiling is 15 ms — below AC-6's own 20 ms, before a
microsecond of transport is charged.** The ceiling falls faster than linearly (~1.0 ms/N·s from
10→20, ~2.3 ms/N·s from 20→30), so the 2× safety factor over 38 ms does not survive a 1.5×
increase in disturbance.

At 40 N·s — ~0.51 m/s of velocity change on a 78 kg system, i.e. a kerb — the board inverts
with a **perfect, zero-delay** controller. Delay is not what fails there.

**So the binding constraint on this vehicle is disturbance-rejection capability, not loop
delay**, and AC-6's threshold is only meaningful with its reference disturbance attached.

**§2's estimator line item holds up well:** its cost is ~22 ms at *every* amplitude (23/22/22),
which is exactly what justifies charging it as a fixed line item rather than a fraction.
Independently corroborated at **16.6 ms** from θ̂/θ phase at the derived crossover (two
excitations agreeing to 0.13°, coherence ≈ 1.00).

### Proposed edit

Replace the "one disturbance magnitude" bullet with the table above, and attach the reference
disturbance to AC-6's threshold wherever it is quoted.

---

## C. Not proposed as doc edits — filed as issues instead

- **#132** — the loop crosses over at **4.48 rad/s**, *below* its corrected unstable pole
  (5.238), with only **2.9 dB** of peak loop gain, on an unfitted `kt` that scales loop gain
  directly. For an RHP-pole plant the low-gain failure is *instability*, not sluggishness.
  **The comfortable delay margin and that marginal gain margin are not independent**: the loop
  tolerates delay partly *because* it is weak, so the known fix (retune for `ω_c ≥ 2p`) would
  reduce the very ceiling AC-6 gates on. AC-6 is valid only for the current gains at `kt = 0.7`.
- **#133** — the estimator's share of the budget, and the reference disturbance nobody has yet
  defended. 20 N·s is the scenario nominal, inherited, not derived; Sr. Mechanical & Systems
  owns what a real kerb strike is worth in N·s.

## D. Note on the stale crossover constant

`scripts/analyse_estimator_phase.py:68` carries `OMEGA_C = 12.0`, sourced to the gain
derivation "at the ridden inertia". That derivation is at the **driverless** inertia
(`I ≈ 0.403 kg·m²`, `kp = 80`); the ridden plant is `I = 50.3` with `kp = 200`. The identified
crossover is **4.48 rad/s**.

This matters for anyone reading estimator phase: at 12 rad/s the two excitations disagree by 7°
and the phase is a **lead**, so that script's `tau_lag = tan|φ|/ω` fit reports a lead as a 22 ms
lag. At the real crossover the excitations agree to 0.13° and the lag is genuine. The 22 ms in
`estimator-phase.json` should not be carried into a budget — that it happens to sit near the
correct time-domain figure of 22 ms is a coincidence of two different quantities.

Left as a note rather than fixed: that script is Senior Controls' turf and the fix is cheap, but
it belongs with #133's re-measurement at the production `est_tau_s`, not bundled here.
