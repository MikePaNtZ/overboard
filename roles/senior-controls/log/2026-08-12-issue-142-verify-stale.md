# 2026-08-12 — Issue #142: already answered, across three already-merged PRs

Cron dispatch pass. #142 carries the `role:senior-controls` label, has no open PR against it, and
is not one of the four `Owner: Senior Controls` issues (#261/#255/#230/#226 — all already have
open PRs) or the one `Dispatch: COO only` issue (#61). Everything else in the open queue was
either already in flight, reserved, or missing a written acceptance criterion. This is the one
that was workable and unclaimed — except tracing it showed the work was already done and never
recorded as such.

## What #142 asked for

Split out of #133 AC3: derive a defended reference disturbance (N·s) for the ridden board from
geometry and speed, so the #132 gain retune has a target instead of an inherited 20 N·s scenario
nominal. Five acceptance criteria: (1) a stated, derived reference disturbance with named
assumptions, (2) Sr. Mechanical & Systems input on what a kerb strike is worth, (3) a stated
validity range, (4) whether the current design point survives it, stated plainly either way, (5)
if the envelope must shrink, publish the smaller number rather than implying the larger one.

## Where each AC actually landed

- **AC2** — PR #147 (merged 2026-08-01, Sr. Mechanical & Systems): `kerb_strike_impulse()` /
  `kerb_strike_vs_com_impulse()` in `sim/scenarios/plant.py`, with `tests/test_plant_kerb_strike.py`
  (12 tests, still green — re-ran on this branch, `12 passed`). Explicitly answers "AC1/3/4/5 stay
  with Senior Controls."
- **AC1, AC3** — the same PR's model takes obstacle height and speed as named parameters (not
  folded into a result) and returns `within_validity` against `KERB_STRIKE_VALIDITY`
  (`tyre_deflection_mm: (10, 20)`, `h/r <= 0.6`), with the reasoning for both bounds in the module
  docstring.
- **AC4** — PR #205 (merged 2026-08-02): `tests/test_cmd_envelope_reserve.py::
  test_criterion_a3_full_stick_during_a_kerb_strike_does_not_invert`, `xfail(strict=True)`. Feeds a
  20 mm lip through `kerb_strike_impulse` at the board's measured mid-hold speed and asserts the
  board does *not* invert — which fails, on purpose, because it does. Re-ran on this branch (clean
  `cargo build --workspace --release` against this venv's MuJoCo):  still exactly one `xfailed`, no
  `XPASS`. PR #205's own text states the result plainly: "Not fixable by input shaping... this is
  the measurement behind the ADR's own note that the BALLAST STROKE and CoM height, not the motor,
  are the undersized elements."
- **AC5** — ADR-0011's second ratification (issue #208, "the criterion move"). Rather than the
  gain retune quietly absorbing a disturbance it cannot survive, the kerb-strike criterion was
  moved off the software gate onto the hardware gate, and `crates/sim-host/src/host.rs`'s
  `STATED_ENVELOPE_RESERVE_FRACTION` doc comment says so in the code itself: the reserve "is NOT
  sized against the repo's own reference disturbance, which is far larger than the envelope can
  answer at any stick setting (issue #142...)". That is the honest smaller number, published where
  the constant lives rather than left implicit.

## Why AC4's literal next step ("goes to #132 as the retune's target") didn't happen that way

The finding is that this isn't a gain problem: PR #205 and ADR-0011 both conclude the shortfall is
ballast stroke and CoM height, which no software retune moves. Routing it to ADR-0011's exit
criteria (a hardware gate) instead of feeding it into #132 as a software target is the correct
reading of "stated plainly... not smoothed over," not a skipped step — feeding an unfixable-in-
software number into a gain retune would have been the smoothing-over AC4 was written to prevent.

## Verification performed here

Fresh `python3.12 -m venv` + `pip install -r requirements-sim.txt`, `cargo build --workspace
--release` against it, then:
- `pytest tests/test_plant_kerb_strike.py` — 12 passed.
- `pytest tests/test_cmd_envelope_reserve.py -k "kerb or a3"` — 1 xfailed, no XPASS (the finding
  still holds; the board still hasn't grown ballast stroke).
- `pytest tests/` (full suite) — 333 passed, 5 xfailed, 0 failed. No regression.

## Deliberately left out

- Did not re-run or extend the kerb-strike work itself — nothing here needed fixing.
- Did not chase whether ADR-0011's hardware-gate routing (issue #208, "encode the authored-world
  constraint as a check") has itself landed — that's a separate open issue, tracked on its own.
