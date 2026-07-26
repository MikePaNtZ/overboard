#!/usr/bin/env python3
"""Interactive viewer for the Overboard onewheel model.

Runs the same impulse disturbance-response scenario the CI gate runs, in the
live viewer, so what you watch is what CI measures.

macOS note: MuJoCo's viewer needs its own run loop on the main thread, so it
must be launched with `mjpython`, not plain `python`:

    .venv/bin/mjpython scripts/view_sim.py
    .venv/bin/mjpython scripts/view_sim.py --impulse 6     # sub-threshold: survives
    .venv/bin/mjpython scripts/view_sim.py --impulse 0     # undisturbed: just sits there

The board is DRIVERLESS and STABLE at rest — it will not fall over on its own,
and a build where it does is a bug (that was the old rider-mass-on-a-mast
model). Press the impulse at t0 and it noses in at ~18.6 deg. Hit `r` in the
viewer to reset and watch it again.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from sim.scenarios.impulse_response import (  # noqa: E402
    MODEL_PATH,
    NOMINAL_IMPULSE_NS,
    ImpulseParams,
    load_model,
    nose_strike_angle_deg,
)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--impulse", type=float, default=NOMINAL_IMPULSE_NS, help="N*s")
    ap.add_argument("--realtime", action="store_true", default=True)
    args = ap.parse_args()

    params = ImpulseParams(magnitude_ns=args.impulse)
    model = load_model()
    data = mujoco.MjData(model)
    frame = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "frame")
    force = np.array(params.direction) * (params.magnitude_ns / params.duration_s)

    print(f"model            {MODEL_PATH}")
    print(f"impulse          {params.magnitude_ns:.1f} N*s forward at t={params.t0_s}s")
    print(f"nose strike at   {nose_strike_angle_deg(model):.2f} deg pitch")
    print("press `r` in the viewer to reset and re-run the kick\n")

    announced = False
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.perf_counter()

            data.xfrc_applied[frame] = 0.0
            if params.t0_s <= data.time < params.t0_s + params.duration_s:
                data.xfrc_applied[frame][:3] = force
                if not announced:
                    print(f"t={data.time:.2f}s  IMPULSE {params.magnitude_ns:.0f} N*s")
                    announced = True
            if data.time < params.t0_s:
                announced = False

            mujoco.mj_step(model, data)
            viewer.sync()

            if args.realtime:
                lag = model.opt.timestep - (time.perf_counter() - step_start)
                if lag > 0:
                    time.sleep(lag)


if __name__ == "__main__":
    main()
