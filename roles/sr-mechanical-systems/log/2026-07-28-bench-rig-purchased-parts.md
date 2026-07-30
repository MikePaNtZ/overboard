# 2026-07-28 — `bench_rig.xml` now matches the parts actually purchased (#66)

Flywheel geometry was a 75 mm/12 mm custom aluminium disc estimate; the confirmed part is two
goBILDA 3628-0032-0082 units (82 mm OD, 152 g, 1651 g·cm² each, manufacturer-published). A
same-mass solid disc at that radius computes to ~1277 g·cm² — 30% low — because the real part is
a hub-and-rim casting, not a uniform disc, so each flywheel now carries an explicit `<inertial>`
sourced directly from the datasheet rather than a density-derived guess. Plate stock corrected to
the confirmed 12″×12″ ¼″ 6061-T651 sheet (thickness 6 mm→6.35 mm exact); clamps confirmed as
IRWIN Quick-Grip mini, noted as a second candidate resonance source alongside the riser (#64).
Rotor can/hub stay geometry-derived estimates — no manufacturer figure exists for them, that's
what the bare-run measurement is for.

**Before/after, measured not assumed:** J_disc 1.6103e-3 → 3.3020e-4 kg·m² (matches 2×1651 g·cm²
exactly), J_loaded 1.8510e-3 → 5.7085e-4 kg·m², ratio J_disc/J_bare 6.69 → 1.37. That drop is
expected, not a regression — #65 already flagged 1.1–1.65 from these same confirmed figures and
owns the decision of whether to accept it or add inertia; this issue only made the model match
what's on the shaft. `sim/scenarios/bench_spinup.py`'s `known_disc_inertia_kg_m2()` now returns
the manufacturer figure too (was a radius/thickness/density formula), since real hardware would
know this from a datasheet, not a ruler. 202 tests pass, same count as master — updated in place,
not added, since the affected assertions are model-inertia values, not new coverage.

PR #72.
