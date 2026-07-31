# Issue #113 — replace AC-6's plant-ignorant threshold with a delay budget (PR TBD)

Derived the RIDDEN plant's unstable pole from the compiled model rather than a hand-copied
guess: `p = sqrt(mgl/I) = 3.191 rad/s`, using MuJoCo's own `mgl` (matches `plant_summary()`
exactly, both from the same `mj_forward` state) and a per-body `Iyy` + parallel-axis sum for
`I`. Deliberately did **not** route this through `kt` — `KT_NM_PER_A = 0.7` is an explicitly
unfitted guess (`sim/scenarios/plant.py`), and a crossover-frequency derivation from the current
gains would have inherited that fragility directly. The pole needs no `kt` at all.

**Measured, not assumed, per the issue's own mandate on the estimator line item:** bisected
`scripts/analyse_control.py`'s existing `actuation_delay_s` knob (the same one `impulse_response.run()`
calls `actuation_delay_cycles` in its docstring) on the ridden/cascade closed loop
(`kp_a_per_rad=200, kd_a_per_rad_s=30` inner + `kp_v_rad_per_m_s=0.05, ki_v_rad_per_m=0.02` outer,
`use_estimator=True`, the honest post-issue-#24 default). Result: survives 38 ms, strikes 39 ms.
With truth pitch instead of the estimate: survives 59 ms, strikes 60 ms. **The estimator alone
costs ~21 ms of delay margin** — confirmed as the dominant term, 10-14x the SPI tail's reported
1.5-2 ms, in the "5-15x" range the issue predicted before this was measured.

New doc `docs/design-delay-budget-stage0b.md` carries the full derivation, the delay-budget
table (sampling+ZOH 3ms deterministic, current-loop lag 1ms PROVISIONAL placeholder, CAN TX
~0.3ms computed from the documented 500kbit/s bitrate and generic CAN 2.0B framing — NOT the
real VESC frame size, which is still an honest unconfirmed stub per issue #1/#52 — SPI tail
1.5-2ms reported not measured, estimator ~21ms measured), and the "what this simulator cannot
yet answer" paragraph the issue's own template asks for.

**Outcome: 500 Hz is not the problem, kept as-is** (already independently justified by IMU
anti-aliasing, not stability — lowering it was a FORBIDDEN outcome per the issue). Amended
`design-pi-image-stage0b-reference.md`'s AC-6 from `p99.9<=1ms / max<=2ms` (fractions of the 2ms
period) to `p99.9<=20ms` total loop delay — roughly half the measured 38-39ms ceiling, clear of
both the known ~6.3ms transport-adjacent budget and the estimator's own ~21ms measured cost.

**Also worth recording:** the textbook `0.2/p` robust-margin heuristic (~63ms) over-predicted
the achievable margin by ~1.6x against the measured 38-39ms — the point-mass linearisation
ignores the current clamp and the estimator, both real. Concrete argument for gating on the
measured full-closed-loop number, not the analytic bound alone.

New test file `tests/test_delay_budget_stage0b.py` pins both boundaries (38/39ms with estimator,
59/60ms truth pitch), a bit-identical-repeat check at the boundary (same discipline as issue
#24 AC4), a bounded (not exact-ms) regression gate on the estimator's cost so a future estimator
change that quietly eats more margin fails a test rather than waiting for a bench surprise, and
a loose (2.5-4.0 rad/s) sanity bound on the pole read directly from the compiled model — same
discipline as `test_r_eff_matches_model.py`: a derived number is re-computed from the model, not
hand-copied, so it cannot silently drift.

**TURF-OVERRIDE used** on the three `docs/` edits (COO turf per CODEOWNERS), same precedent
issues #32 and #54 already established for this exact document family — `design-pi-image-stage0b.md`
already carries `Owner: Senior Controls` in its own header. Documented in a commit message per
`policy_check.py`'s requirement, not just here.

**Deliberately left out / flagged, not fixed:**
- Did not touch AC-5, AC-7, AC-8, AC-9, AC-10, AC-11 — the issue asked specifically to replace
  AC-6, not to re-litigate the rest of the numeric acceptance criteria table.
- Did not re-run the delay-budget sweep across the full disturbance-rejection envelope (issue
  #24 AC2's impulse-magnitude range) — only `NOMINAL_IMPULSE_NS`, stated as a validity-range
  gap in the new doc rather than silently generalised.
- Did not attempt to derive or fabricate the real VESC `SET_CURRENT` CAN frame size — `vesc-wire`/
  `vesc-tx` remain honest stubs (issue #1, #52, the known vesc-project.com 403 dead end already
  in this file). The ~0.3ms CAN line item is a generic protocol bound, stated as such.
- Did not attempt the AC-9 SPI-tail hardware measurement itself — no Pi 5 exists in this
  session's reach; the 1.5-2ms figure stays REPORTED, not measured, exactly as design-pi-image-stage0b.md
  already flagged it.

**Could not verify:** whether 500 kbit/s CAN framing overhead assumptions (extended ID, 8-byte
payload, worst-case bit-stuffing) match the eventual real VESC command frame once its byte
layout is known — flagged in the new doc, not asserted as fact.

Full suite re-run clean: 273 passed, 2 xfailed (Python), full Rust workspace test suite green,
`cargo run -p xtask -- gate` passes, `python3 .github/policy_check.py` passes (turf override
recorded, doc-length advisories only, no hard failures).
