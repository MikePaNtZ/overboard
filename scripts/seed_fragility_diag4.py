#!/usr/bin/env python3
"""DIAGNOSTIC ONLY. Isolates whether terrain.py's near-instant collapse is a
CONTROL-LOOP phenomenon (mechanism a: closed-loop marginal stability) or a
GEOMETRIC INITIAL-CONDITION defect (mechanism b: the analytically-placed
frame does not sit exactly on the discretised/interpolated heightfield at
the crest, producing a free-fall/impact transient at t=0 independent of any
controller).

Method: build the terrain model exactly as terrain.py does, then step with
ZERO commanded current (no controller at all) for the first ~50 ms and watch
pitch rate / contact state. If pitch rate departs from ~0 immediately with
zero control torque, the defect is geometric, not a control-law artifact.
Also reports the analytic crest height vs MuJoCo's own interpolated height at
x=0 to quantify any air-gap/penetration.

Changes nothing on disk.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from sim.scenarios.impulse_response import frame_pitch_rad  # noqa: E402
from sim.scenarios.terrain import (  # noqa: E402
    TerrainParams,
    amplitude_for_grade,
    build_terrain_model,
    surface_z,
)

params = TerrainParams()  # default: 8% grade
model, amp = build_terrain_model(params)
print(f"amplitude (analytic crest height): {amp:.8f} m")

data = mujoco.MjData(model)
mujoco.mj_forward(model, data)

frame = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "frame")
ground = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "ground")

print(f"frame z at t=0 (from XML placement, r+amp): {float(data.xpos[frame][2]):.8f}")

# MuJoCo's own reading of the interpolated surface height under x=0, via a ray
# straight down from well above the crest.
pnt = np.array([0.0, 0.0, amp + 1.0])
vec = np.array([0.0, 0.0, -1.0])
geomid_arr = np.zeros(1, dtype=np.int32)
dist = mujoco.mj_ray(model, data, pnt, vec, None, 1, -1, geomid_arr)
hit_z = pnt[2] - dist if dist >= 0 else None
print(f"mj_ray hit geom {geomid_arr[0]}, dist={dist}, hit_z = {hit_z}")
print(f"analytic surface_z(0) = {surface_z(0.0, amp, params.crest_to_crest_m):.8f}")

print(f"\nncon at t=0 (post mj_forward): {data.ncon}")
for i in range(data.ncon):
    c = data.contact[i]
    print(f"  contact {i}: geom1={c.geom1} geom2={c.geom2} pos={c.pos} dist={c.dist:.6f}")

print(f"\nqvel at t=0: {data.qvel[:8]}")
print(f"qacc at t=0 (post mj_forward, BEFORE any step): {data.qacc[:8]}")

# Now step with ZERO control torque -- no controller in the loop at all.
data.ctrl[0] = 0.0
print("\n--- stepping with data.ctrl[0] = 0.0 (no controller) ---")
print(f"{'step':>5} {'t_ms':>7} {'pitch_deg':>10} {'pitch_rate':>11} {'z_frame':>9} {'ncon':>5}")
for i in range(30):
    mujoco.mj_step(model, data)
    pitch_deg = math.degrees(frame_pitch_rad(model, data))
    print(f"{i:5d} {1000*data.time:7.2f} {pitch_deg:10.5f} {data.qvel[4]:11.5f} "
          f"{float(data.xpos[frame][2]):9.5f} {data.ncon:5d}")
