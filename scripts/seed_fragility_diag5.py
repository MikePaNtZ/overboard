#!/usr/bin/env python3
"""DIAGNOSTIC ONLY. Traces terrain.py's first ~0.3 s under NOISELESS_CUTBACK
(deterministic, RNG never sampled) with the REAL RustController in the loop,
to find exactly where/how the divergence starts. Companion to
seed_fragility_diag4.py, which showed the crest spawn itself is benign with
zero control torque -- so if this traces a divergence, it originates in the
closed loop, not the initial geometry.

Changes nothing on disk.
"""
from __future__ import annotations

import dataclasses
import math
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

import mujoco  # noqa: E402

from sim.scenarios.imperfections import STAGE0_CUTBACK, ImperfectionState  # noqa: E402
from sim.scenarios.impulse_response import KT_NM_PER_A, frame_pitch_rad  # noqa: E402
from sim.scenarios.rust_controller import RustController  # noqa: E402
from sim.scenarios.terrain import TerrainParams, build_terrain_model  # noqa: E402

NOISELESS_CUTBACK = dataclasses.replace(
    STAGE0_CUTBACK, gyro_noise_rad_s=0.0, gyro_bias_rad_s=0.0, accel_noise_m_s2=0.0
)

params = TerrainParams()
model, amp = build_terrain_model(params)
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)

dt = float(model.opt.timestep)
imp = ImperfectionState(profile=NOISELESS_CUTBACK, dt_s=dt)


def v_ref_at(t):
    return 0.0  # well inside settle_s=2.0 for the whole trace


from sim.scenarios.plant import imu_readings  # noqa: E402

flowing_a = 0.0
with RustController(
    kp_nm_per_rad=140.0, kd_nm_per_rad_s=40.0, max_current_a=params.max_current_a,
    kp_v_rad_per_m_s=0.05, ki_v_rad_per_m=0.02, com_above_axle=True,
    v_ref_fn=v_ref_at, use_estimator=params.use_estimator,
    estimator_tau_s=params.estimator_tau_s,
    accel_ff_current_source=params.accel_ff_current_source,
) as ctl:
    print(f"{'step':>5} {'t_ms':>7} {'pitch':>9} {'pitch_est':>10} {'gyro_y':>9} "
          f"{'accel':>24} {'proposed_A':>11} {'applied_A':>10} {'pref_deg':>9}")
    for i in range(150):
        t = float(data.time)
        pitch_rad = frame_pitch_rad(model, data)
        wheel_true = float(data.qvel[6])
        true_gyro, true_accel = imu_readings(model, data)
        gyro_v = imp.gyro_vec(true_gyro)
        accel_v = imp.accel_vec(true_accel)

        proposed = float(ctl(
            t, pitch_rad, imp.gyro(float(data.qvel[4])),
            imp.wheel_rate(wheel_true, t),
            gyro_rad_s=gyro_v, accel_m_s2=accel_v, motor_current_a=flowing_a,
        ))
        current = imp.apply_current(proposed, wheel_true)
        flowing_a = current
        data.ctrl[0] = current * KT_NM_PER_A
        mujoco.mj_step(model, data)

        if i < 40 or i % 10 == 0:
            print(f"{i:5d} {1000*t:7.2f} {math.degrees(pitch_rad):9.4f} "
                  f"{math.degrees(float(ctl.pitch_used_rad)):10.4f} {gyro_v[1]:9.5f} "
                  f"[{accel_v[0]:7.3f},{accel_v[1]:7.3f},{accel_v[2]:7.3f}] "
                  f"{proposed:11.4f} {current:10.4f} "
                  f"{math.degrees(ctl.pitch_ref_rad):9.4f}")
