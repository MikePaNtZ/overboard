#!/usr/bin/env python3
"""What size disturbance must the board actually survive?

Issue #142. Every delay and margin number in `docs/design-delay-budget-stage0b.md`
is quoted at a reference disturbance of `NOMINAL_IMPULSE_NS = 20 N*s`, and that
figure is **inherited from a scenario nominal, not derived from anything**. Its
own docstring gives the game away: *"On 12.5 kg it is a 1.6 m/s delta-v -- a
firm shove."* 12.5 kg is the DRIVERLESS board. On the 82.5 kg ridden vehicle
every gate actually runs against, the same 20 N*s is 0.24 m/s -- and nothing
says a firm shove is the worst thing that happens to a board.

The #132 retune needs a target, and "survive a firm shove" is not one.

## The model -- and why this file no longer derives its own

This script used to re-derive kerb impulse from `dv = v*h/r`, a simplified
rigid-wheel model written before Sr. Mechanical & Systems answered #142 AC2.
That was a mistake this repo has made before with a different constant (see
`tests/test_r_eff_matches_model.py`'s history) and `crates/sim-host/src/host.rs`
says so explicitly for this exact case: *"the kerb derivation this repo
trusts lives in ONE place -- `sim/scenarios/plant.py::kerb_strike_impulse`
... re-implementing that geometry in Rust would give this repo two kerb
models that could disagree."* This file WAS that second, disagreeing model --
at a 20 mm lip and 4 m/s it said 45.4 N*s where the canonical model brackets
[56.7, 87.9] N*s, a 20-90% understatement depending on speed and height. It
now imports `kerb_strike_impulse`/`kerb_strike_vs_com_impulse` instead of
reimplementing them. See `tests/test_plant_kerb_strike.py` for the model
itself and why its first version was wrong.

## Two caveats that pointed in OPPOSITE directions -- one is now answered

1.  **The wheel and step are rigid here; the real tyre is pneumatic.** A real
    tyre deforms over the edge and spreads the impulse across a longer
    window, so this OVERSTATES `J` -- by how much is a tyre-compliance
    question this repo cannot answer (no tyre model). **Still open: this is
    an upper bound on a rigid-wheel strike, and the gap is probably large.**
    `kerb_strike_impulse`'s `KERB_STRIKE_VALIDITY` names the same gap.
2.  **A kerb acts at the contact patch; the sim's impulse acts through the
    CoM.** `ImpulseParams.application_height_m` defaults to 0.0 precisely so
    the disturbance is a pure LINEAR impulse with zero initial pitch rate.
    **Now answered, not just flagged:** `kerb_strike_vs_com_impulse` derives
    the pitch rate a real kerb strike imparts (100-450+ deg/s across this
    grid) against the ZERO deg/s a same-magnitude CoM impulse imparts by
    construction. N*s was always the wrong currency for comparing the two;
    pitch rate is what the pitch-rate control channel actually feels, and
    that comparison is what this report now leads with.
    `tests/test_cmd_envelope_reserve.py::test_criterion_a3_full_stick_during_a_kerb_strike_does_not_invert`
    (ADR-0011 criterion (a) entry 3) has since measured the consequence in
    the closed loop, not just derived it: a realistic lip at hold speed
    inverts the board. xfail, not a bug to fix here -- ADR-0011 attributes it
    to ballast stroke / CoM height, not the motor.

Caveat (1) is still open and is Sr. Mechanical & Systems' surface, same as
before. Caveat (2) is closed by their landed `kerb_strike_vs_com_impulse`.

    .venv/bin/python scripts/reference_disturbance.py
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

import mujoco  # noqa: E402

from sim.scenarios.impulse_response import NOMINAL_IMPULSE_NS  # noqa: E402
from sim.scenarios.plant import (  # noqa: E402
    build_model,
    kerb_strike_vs_com_impulse,
)

#: The ridden plant every closed-loop gate runs against.
BALLAST_KG, BALLAST_M, CLAMP_A = 70.0, 0.75, 40.0

#: Obstacle heights, metres. Spans "paving lip" to "UK kerb".
#: A standard UK kerb face is 100-125 mm; 25 mm is a raised paving slab or a
#: pothole edge; 5-10 mm is a tired footpath joint.
KERB_HEIGHTS_M = (0.005, 0.010, 0.015, 0.020, 0.025, 0.050, 0.100, 0.125)

#: Approach speeds, m/s. 4 m/s (~14 km/h) is the cutback ONSET speed in
#: `STAGE0_CUTBACK` -- i.e. the speed the drive is expected to work at, not a
#: stunt. 8 m/s (~29 km/h) is that profile's "plausible top speed".
SPEEDS_M_S = (2.0, 4.0, 6.0, 8.0)

CLOSED_LOOP_VERDICT = (
    "Measured, not just derived: "
    "tests/test_cmd_envelope_reserve.py::"
    "test_criterion_a3_full_stick_during_a_kerb_strike_does_not_invert "
    "(ADR-0011 criterion (a) entry 3) injects this model's own force/torque "
    "for a 20 mm lip at hold speed into the closed-loop host and the board "
    "inverts. xfail(strict=True) -- attributed to ballast stroke / CoM "
    "height (undersized), not the motor, and not fixable by input shaping."
)


def build_report() -> dict:
    model = build_model(BALLAST_KG, BALLAST_M, CLAMP_A)

    # Read geometry and mass from the compiled model, not from a doc.
    geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "wheel_geom")
    r = float(model.geom_size[geom][0])
    total_mass = float(sum(model.body_mass[b] for b in range(model.nbody)))

    rows = []
    for h in KERB_HEIGHTS_M:
        if h >= r:
            rows.append({"kerb_m": h, "note": "h >= r: the wheel cannot mount "
                                              "this at all; it is a wall"})
            continue
        for v in SPEEDS_M_S:
            strike = kerb_strike_vs_com_impulse(h, v)
            rows.append({
                "kerb_m": h, "speed_m_s": v,
                "impulse_ns": strike["impulse_ns"],
                "pitch_rate_deg_s": strike["pitch_rate_deg_s"],
                "com_impulse_pitch_rate_deg_s": strike["com_impulse_pitch_rate_deg_s"],
                "pivot_reachable": strike["pivot_reachable"],
                "within_validity": strike["within_validity"],
                "note": strike["note"],
            })

    report = {
        "plant": {"wheel_radius_m": r, "total_mass_kg": total_mass},
        "canonical_model": "sim.scenarios.plant.kerb_strike_vs_com_impulse "
                            "(Sr. Mechanical & Systems, issue #142 AC2)",
        "current_reference_ns": NOMINAL_IMPULSE_NS,
        "current_reference_delta_v_m_s": NOMINAL_IMPULSE_NS / total_mass,
        "current_reference_com_impulse_pitch_rate_deg_s": 0.0,
        "grid": rows,
        "closed_loop_verdict": CLOSED_LOOP_VERDICT,
    }
    return report


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out-dir", type=Path, default=REPO / "sim/out/experiments")
    args = ap.parse_args()

    report = build_report()

    print(json.dumps(report, indent=2))
    args.out_dir.mkdir(parents=True, exist_ok=True)
    out = args.out_dir / "reference_disturbance.json"
    out.write_text(json.dumps(report, indent=2) + "\n")
    print(f"\nwrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
