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

## The model, and why this one

A wheel of radius `r` rolling at `v` into a step of height `h < r`. At impact
the contact point jumps to the step edge and the wheel must begin rotating
about it, so the velocity component **along** the edge-to-centre line is
destroyed and only the perpendicular component survives.

The edge-to-centre vector has length `r` and vertical component `(r - h)`, so
the surviving speed is `v*(r - h)/r` and

    dv = v * h / r          J = M * dv = M * v * h / r

That is the whole derivation. It needs no tuning constant, no fitted
coefficient and no `kt` -- only geometry, speed and mass, which is exactly what
#142 asked for. Note `h` and `v` enter identically: a 10 mm lip at 8 m/s is the
same impulse as a 20 mm lip at 4 m/s.

## Two caveats that point in OPPOSITE directions

Stated together because quoting either alone would be misleading:

1.  **The wheel and step are rigid here; the real tyre is pneumatic.** A real
    tyre deforms over the edge and spreads the impulse across a longer window,
    so this OVERSTATES `J` -- by how much is a tyre-compliance question this
    repo cannot answer (no tyre model, `docs/design-delay-budget-stage0b.md`
    says so). **This is an upper bound on a rigid-wheel strike.**
2.  **A kerb acts at the contact patch; the sim's impulse acts through the
    CoM.** `ImpulseParams.application_height_m` defaults to 0.0 precisely so the
    disturbance is a pure LINEAR impulse. A real kerb force lands ~0.83 m BELOW
    the CoM, which adds an angular impulse AND decelerates the base -- and
    decelerating the base of an inverted pendulum pitches it further forward,
    the opposite of the recovery input. **So feeding a kerb-derived `J` in as a
    CoM impulse UNDERSTATES its severity.**

Neither is quantified here. The honest output is a magnitude with its
assumptions attached, not a single blessed number -- and per #142's AC2 the
kerb geometry itself is Sr. Mechanical & Systems' call, not Controls'.

    .venv/bin/python scripts/reference_disturbance.py
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

import mujoco  # noqa: E402

from sim.scenarios.impulse_response import NOMINAL_IMPULSE_NS  # noqa: E402
from sim.scenarios.plant import build_model  # noqa: E402

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

#: What the closed loop was measured to do, from `analyse_delay_budget.py`
#: and #122. Used to classify each derived impulse rather than to derive it.
MEASURED = {
    20.0: "solvent, barely (38 ms estimator-in-loop ceiling)",
    30.0: "INSOLVENT -- 15 ms ceiling, below AC-6b's own threshold",
    40.0: "INVERTS AT ZERO DELAY -- beyond disturbance rejection entirely",
}


def classify(j_ns: float) -> str:
    if j_ns >= 40.0:
        return "inverts at zero delay"
    if j_ns >= 30.0:
        return "insolvent"
    if j_ns >= 20.0:
        return "at/over the current reference"
    return "inside the current reference"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", type=Path, default=REPO / "sim/out/experiments")
    args = ap.parse_args()

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
            dv = v * h / r
            j = total_mass * dv
            rows.append({
                "kerb_m": h, "speed_m_s": v, "delta_v_m_s": dv,
                "impulse_ns": j, "verdict": classify(j),
            })

    # Invert: what obstacle does each measured threshold correspond to?
    inverted = {}
    for j_crit, meaning in MEASURED.items():
        inverted[f"{j_crit:g} N*s"] = {
            "meaning": meaning,
            "equivalent_kerb_mm_at_speed": {
                f"{v:g} m/s": 1000.0 * j_crit * r / (total_mass * v)
                for v in SPEEDS_M_S
            },
        }

    report = {
        "plant": {"wheel_radius_m": r, "total_mass_kg": total_mass},
        "current_reference_ns": NOMINAL_IMPULSE_NS,
        "current_reference_delta_v_m_s": NOMINAL_IMPULSE_NS / total_mass,
        "model": "dv = v*h/r  (rigid wheel, rigid step, angular momentum "
                 "about the step edge)",
        "grid": rows,
        "what_each_threshold_means_as_an_obstacle": inverted,
    }

    print(json.dumps(report, indent=2))
    args.out_dir.mkdir(parents=True, exist_ok=True)
    out = args.out_dir / "reference_disturbance.json"
    out.write_text(json.dumps(report, indent=2) + "\n")
    print(f"\nwrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
