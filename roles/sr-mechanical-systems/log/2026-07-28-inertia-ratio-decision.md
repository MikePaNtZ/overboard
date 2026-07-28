# Add a $20-30 steel disc, sequenced behind the bare-run measurement, not before it

Issue #65: the confirmed goBILDA flywheels (3302 g·cm² total, manufacturer figure) give a ratio
against the estimated bare-rotor band of 1.1–1.65, against a 6.69 design point that was never
actually buildable (it assumed a heavier custom disc that was never bought). Derived the
error-amplification formula `sqrt(1+ρ²)/(ρ-1)` (ρ = alpha1/alpha2 = 1+ratio) from `identify()`'s
own two-run algebra and cross-checked it numerically (independent finite-difference derivative,
not a restatement) — `sim/scenarios/bench_inertia_ratio.py`. Result: 1.72×–2.11× amplification on
the confirmed flywheels, 1.48×–1.82× worse than the 1.159× design point — a firm number where the
issue only had "roughly doubles."

**Decision: add inertia, not accept-and-carry.** A single ~150 mm/600 g steel disc stacked on the
confirmed flywheels restores the ratio to 6.7–10.1 (amplification back to ~1.16×) for the cost the
issue itself named, against torque figures every later stage inherits. **But order it only after
the bench's own bare-rotor run measures `J_bare`** — that resolves the 16× sourcing disagreement
for free (mode 1 of `identify()` already measures it), so buying before measuring risks sizing the
disc against the wrong end of a 16×-wide guess. `recommend(j_bare_measured_kg_m2=...)` re-derives
the pick once that number exists.

PR #75, issue #65.
