#!/usr/bin/env python3
"""Issue #132 -- the (kp, kd) feasibility map, entirely `kt`-independent.

**Not a retune.** This produces a feasibility map over a grid of candidate
ridden inner-loop gains; it does not choose or recommend shipping gains. See
"What must not be concluded" below.

## Where this sits in #132's revised order

The issue's own comment thread reframed the finding (`|L(0)| = K*kp/p^2`,
stable iff `K*kp > p^2`) and laid out a five-step order. Step 1, torque
re-denomination, landed as #137: the control law now takes `kp_nm_per_rad` /
`kd_nm_per_rad_s` directly (`crates/control-core/src/lib.rs`), so `kt` no
longer appears anywhere in the loop gain -- `K` above collapses to 1 in the
gains this script (and the shipped controller) actually use. Step 2, deriving
a reference disturbance, landed as #133/#142. **This script is step 3**: sweep
`(kp, kd)` as a plane against downward gain margin and linear delay margin,
producing a feasibility map rather than a tune.

**Deliberately out of scope here, left for follow-up work:**

- The nonlinear disturbance-capacity axis (step 3's third dimension). Each
  grid point would need a full closed-loop impulse-response run
  (`sim/scenarios/impulse_response.py`) to measure it, which is expensive
  over a two-parameter grid; this script only computes what a single
  linearization answers cheaply (gain margin, crossover, phase/delay
  margin). `scripts/analyse_delay_budget.py` already does the nonlinear
  amplitude sweep at ONE gain pair (the shipped one) -- extending that to
  every grid point is real, separate work.
- Step 4, the robustness sweep over `p` (rider mass/CoM range, ballast vs
  rider) -- a different independent variable, not this one.
- The bench `kt` fit itself (#132 AC1) is Sr. Mechanical & Systems' turf, not
  a repo-only computation, and this script does not need it: everything here
  is in torque-native units per #137, so `kt` never enters.

## Method

One linearization, reused across the whole grid. The trim point (`A`, `B`,
`Cp`) is a property of the PLANT, not the gains -- `settle_upright` finds
upright equilibrium under some stabilising PD (any stabilising pair works,
since the trim state is the same fixed point), so it is computed once via
`scripts/analyse_delay_budget.py`'s own machinery and re-used, rather than
re-linearizing per grid point. Only the loop transfer `L(jw) = kp*Cpos(x) +
kd*Cvel(x)` depends on the swept `(kp, kd)`, and that is a cheap linear solve
per frequency, per grid point.

    scripts/gain_margin_sweep.py --out-dir sim/out/experiments

## What must NOT be concluded from this

No shipping gain change. This is a feasibility map over the LINEAR,
delay-free, disturbance-blind plane -- it says where the loop is structurally
sound and where it is not, not where it should operate. Per the issue's own
guardrail, any retune needs the nonlinear disturbance-capacity axis (still
unmeasured here) and the robustness-over-p sweep (also unmeasured here)
before it is a shipping decision, and needs to stay inside the 40 A clamp
once denominated back through a FITTED `kt` -- still a bench measurement,
still not made here.
"""

from __future__ import annotations

import argparse
import itertools
import json
import math
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from scripts.analyse_delay_budget import (  # noqa: E402
    BALLAST_KG, BALLAST_M, CLAMP_A,
    analytic_pole, linearize, margins, pitch_output_map, settle_upright,
)
from sim.scenarios.plant import build_model  # noqa: E402

#: Shipped ridden inner-loop gains, torque-native (#137;
#: `tests/test_closed_loop.py:172`). `200 A/rad, 30 A/(rad/s)` at the placeholder
#: `kt = 0.7 N*m/A` -- carried here only as the grid's reference point, not as
#: an input to anything: the loop transfer below takes `kp`/`kd` directly.
SHIPPED_KP_NM_PER_RAD = 140.0
SHIPPED_KD_NM_PER_RAD_S = 21.0

#: Grid bounds. Lower edge below the RHP boundary at the shipped `kd` (so the
#: infeasible region is visible on the map, not just its edge); upper edge
#: comfortably past `omega_c = 2p` (~10.5 rad/s), which #132's own comment
#: found to raise crossover too far into unmodelled tyre/rider-body resonance
#: (6-19 rad/s) -- the map should show that this is a bad idea, not merely
#: avoid asking.
KP_GRID_NM_PER_RAD = np.linspace(60.0, 450.0, 14)
KD_GRID_NM_PER_RAD_S = np.linspace(7.0, 75.0, 14)


def loop_transfer(A, B, Cp, omega, kp_nm, kd_nm):
    """`L(jw)` for torque-native gains -- no `kt` conversion anywhere.

    Same construction as `analyse_delay_budget.loop_transfer`, generalised to
    take the swept gains as arguments instead of the module-level shipped
    pair, and with the `KT_NM_PER_A` factor dropped: `B` already maps the
    actuator's torque command (N*m) to state derivative (the MJCF actuator IS
    a torque source), and the gains here are already N*m/rad and
    N*m/(rad/s), so multiplying by `kt` would double-apply the actuation
    boundary's conversion. This is the concrete benefit #137 bought: this
    function needs one fewer constant than its predecessor.
    """
    nv = len(Cp)
    nx = A.shape[0]
    Cpos = np.zeros(nx)
    Cpos[:nv] = Cp
    Cvel = np.zeros(nx)
    Cvel[nv:2 * nv] = Cp

    out = np.zeros(len(omega), dtype=complex)
    I = np.eye(nx)
    for k, w in enumerate(omega):
        x = np.linalg.solve(1j * w * I - A, B[:, 0])
        out[k] = kp_nm * (Cpos @ x) + kd_nm * (Cvel @ x)
    return out


def fit_reduced_gain_constant(A, B, Cp, omega, p, kp_ref, kd_ref) -> float:
    """Calibrate the reduced 2-state pendulum's `b` (`#132`'s `K`) against the
    FULL model, once, at the shipped gains -- rather than trying to read
    `|L(0)|` off the full 14-state model directly.

    A direct DC evaluation of the full model is unusable: the free joint
    carries integrator DOFs with no restoring force at all (wheel spin, yaw,
    forward translation), so `A` has an exact zero eigenvalue, and `L(jw)`
    computed from the full model does not converge to a stable limit as
    `w -> 0` (checked: it falls monotonically toward zero below `w ~ 0.1`
    rather than settling -- the free modes dominate exactly where a DC
    reading would be taken). Full closed-loop eigenvalues have the same
    problem from the other side: state feedback on pitch alone leaves those
    same free modes at (numerically near-)zero real part regardless of gains,
    so "is the max eigenvalue negative" flags the empirically-safe shipped
    gains as marginal.

    `#132`'s own algebra models the pitch DOF alone as a 2nd-order pendulum,
    `theta'' - p^2*theta = b*tau`, which is exactly what `analyse_delay_budget`
    already validates against the nonlinear sim at ONE point (the shipped
    gains: measured break times, not just a linear prediction). PD feedback
    on that reduced model gives `L(jw) = b*(kp + j*kd*w) / -(w^2 + p^2)`, and
    solving `|L(jwc)| = 1` at the crossover the FULL model already measured
    (validated: `analyse_delay_budget.py` reports `omega_c = 4.48 rad/s`
    here, matching the historical value) pins `b`:

        b = (wc^2 + p^2) / sqrt(kp^2 + kd^2*wc^2)

    Checked against the full model's own `|L(jw)|` from 0.1 rad/s to
    crossover at the shipped gains: agreement is within ~5% throughout that
    whole range, so the free modes the full-order DC/eigenvalue reads choke
    on are a small correction to the pitch channel at these frequencies, not
    a dominant one -- the reduction is trustworthy over the range this script
    uses it for.
    """
    L = loop_transfer(A, B, Cp, omega, kp_ref, kd_ref)
    m = margins(omega, L)
    if m is None:
        raise ValueError("reference gains have no crossover -- cannot calibrate b")
    wc = m["omega_c_rad_s"]
    return (wc * wc + p * p) / math.sqrt(kp_ref * kp_ref + kd_ref * kd_ref * wc * wc)


def evaluate(A, B, Cp, omega, p, b, kp_nm, kd_nm) -> dict:
    """One grid point: downward gain margin (reduced model, see `b` above),
    crossover, phase and delay margin (full model -- `loop_transfer` is
    well-behaved away from DC, unlike the gain-margin reading)."""
    kp_min = p * p / b
    row = {
        "kp_nm_per_rad": float(kp_nm),
        "kd_nm_per_rad_s": float(kd_nm),
        "kp_min_nm_per_rad": kp_min,
        "stable": bool(kp_nm > kp_min),
    }
    if not row["stable"]:
        row.update(downward_gain_margin_db=None, omega_c_rad_s=None,
                    phase_margin_deg=None, delay_margin_s=None)
        return row

    row["downward_gain_margin_db"] = 20.0 * math.log10(kp_nm / kp_min)
    L = loop_transfer(A, B, Cp, omega, kp_nm, kd_nm)
    m = margins(omega, L)
    if m is None:
        row.update(omega_c_rad_s=None, phase_margin_deg=None, delay_margin_s=None)
    else:
        row.update(m)
    return row


def sweep(A, B, Cp, omega, p, b, kp_grid=KP_GRID_NM_PER_RAD, kd_grid=KD_GRID_NM_PER_RAD_S):
    return [evaluate(A, B, Cp, omega, p, b, kp, kd)
            for kp, kd in itertools.product(kp_grid, kd_grid)]


#: "Treat any sim result above ~8 rad/s crossover as unvalidated" -- #132's
#: comment thread, stated alongside the 1.5p recommendation itself. Named
#: here rather than folded into `recommended_band` so both halves of that
#: one sentence stay next to each other in the diff if either ever changes.
RESONANCE_CEILING_RAD_S = 8.0


def recommended_band(p: float, ceiling: float = RESONANCE_CEILING_RAD_S):
    """`(target_omega_c, band_is_empty)` for #132's recommended crossover.

    Pulled out of `main()` so the "does the band still exist" question is
    unit-testable on synthetic `p` values, independently of the model.
    """
    target = 1.5 * p
    return target, target > ceiling


def select_feasible(rows, target_omega_c: float, ceiling: float = RESONANCE_CEILING_RAD_S):
    """Grid rows inside #132's recommended band: stable, `omega_c` between the
    1.5p floor and the unvalidated-resonance ceiling, and positive phase
    margin once there."""
    return [r for r in rows if r["stable"] and r["omega_c_rad_s"] is not None
            and target_omega_c <= r["omega_c_rad_s"] <= ceiling
            and r["phase_margin_deg"] > 0.0]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", type=Path, default=REPO / "sim/out/experiments")
    args = ap.parse_args()

    model = build_model(BALLAST_KG, BALLAST_M, CLAMP_A)
    dt = float(model.opt.timestep)
    data = settle_upright(model)

    A, B = linearize(model, data)
    Cp = pitch_output_map(model, data)
    p = analytic_pole(model)["p_rolling_support_rad_s"]

    omega = np.logspace(-1, math.log10(math.pi / dt), 4000)

    b = fit_reduced_gain_constant(A, B, Cp, omega, p,
                                   SHIPPED_KP_NM_PER_RAD, SHIPPED_KD_NM_PER_RAD_S)

    rows = sweep(A, B, Cp, omega, p, b)
    shipped = evaluate(A, B, Cp, omega, p, b,
                        SHIPPED_KP_NM_PER_RAD, SHIPPED_KD_NM_PER_RAD_S)

    # #132's revised recommendation: prefer omega_c ~ 1.5p (10 dB, tolerates a
    # 68% gain shortfall) over the originally-proposed 2p, because the higher
    # target crosses into unmodelled tyre/rider-body resonance (6-19 rad/s).
    # The SAME comment gives an explicit ceiling: "treat any sim result above
    # ~8 rad/s crossover as unvalidated" -- so the recommended band is NOT
    # "1.5p and up", it is [1.5p, ~8 rad/s]. `p` has moved since that comment
    # was written (5.24 -> 5.564 rad/s here, an intervening model change --
    # see the module docstring), so 1.5p now sits at ~8.35 rad/s: ABOVE the
    # ceiling. That is a finding in its own right, reported explicitly below
    # rather than silently widening the ceiling to make the band non-empty.
    target_omega_c, band_is_empty = recommended_band(p)
    feasible = select_feasible(rows, target_omega_c)
    best = max(feasible, key=lambda r: r["phase_margin_deg"]) if feasible else None

    report = {
        "p_rolling_support_rad_s": p,
        "fitted_reduced_gain_constant_b": b,
        "target_omega_c_rad_s_at_1p5p": target_omega_c,
        "resonance_ceiling_rad_s": RESONANCE_CEILING_RAD_S,
        "recommended_band_is_empty": band_is_empty,
        "shipped_gains": shipped,
        "grid_shape": [len(KP_GRID_NM_PER_RAD), len(KD_GRID_NM_PER_RAD_S)],
        "feasible_in_recommended_band_count": len(feasible),
        "best_phase_margin_in_recommended_band": best,
        "grid": rows,
    }

    print(json.dumps(report, indent=2))

    args.out_dir.mkdir(parents=True, exist_ok=True)
    out = args.out_dir / "gain_margin_sweep.json"
    out.write_text(json.dumps(report, indent=2) + "\n")
    print(f"\nwrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
