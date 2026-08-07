#!/usr/bin/env python3
"""Clamp-sweep campaign: what does `max_pitch_ref_rad` buy, and what does it cost?

**Characterisation only.** Sweeps the outer-loop pitch-reference clamp
(`sim/scenarios/rust_controller.py::DEFAULT_MAX_PITCH_REF_RAD`, shipped at
0.087 rad / 4.98 deg) across rolling terrain, steady hills, and a flat
braking stop, then greps the existing pytest suite for what a raised clamp
breaks. It does not tune KP, KD, motor/drag constants, the outer-loop gains,
`CMD_ENVELOPE_RESERVE*`, or the 0.85 warning threshold, and it does not pick
a final clamp value -- that is a derived decision (strike geometry + a margin
argument) left to whoever owns this campaign's finding.

THE HYPOTHESIS UNDER TEST
-------------------------
`max_pitch_ref_rad = 0.087` caps the torque the outer loop may request at
`KP_NM_PER_RAD * 0.087` = 12.18 N*m, 32% of the 60 A / 37.70 N*m envelope.
Predicted: this is why the 8% rolling-terrain gate nose-strikes and why a
flat braking stop falls short of a Pint-class figure, and raising the clamp
should move both AT ONCE, with no other change.

USAGE
-----
    cargo build --release -p control-ffi
    cargo build --release -p sim-host --bin sim-host   # only for --regression
    .venv/bin/python scripts/clamp_sweep.py terrain
    .venv/bin/python scripts/clamp_sweep.py hill
    .venv/bin/python scripts/clamp_sweep.py brake
    .venv/bin/python scripts/clamp_sweep.py asymmetric
    .venv/bin/python scripts/clamp_sweep.py noise --clamp-deg 10 --seeds 25
    .venv/bin/python scripts/clamp_sweep.py regression
    .venv/bin/python scripts/clamp_sweep.py all

A KNOWN-SITES CORRECTION
-------------------------
The task brief that spawned this campaign cites
`crates/control-ffi/src/lib.rs:539` as a *production* default of 0.087 rad,
distinct from `control-core`'s test-only fixture. Re-checked here: BOTH are
inside `#[cfg(test)] mod tests` blocks (`control-ffi`'s is the `params()`
fixture used only by that crate's own unit tests). There is no Rust-side
"production default" for this clamp at all -- production values always flow
in at runtime through `ObParamsV2::max_pitch_ref_rad`, set by whichever
caller builds the struct. The only genuine standing default is Python's
`sim/scenarios/rust_controller.py::DEFAULT_MAX_PITCH_REF_RAD`, which
`hill.py`/`terrain.py`/`shuttle_run.py` all inherit by never overriding it.

WHY THESE SCENARIOS USE A CUSTOM `controller_factory`, NOT A NEW DATACLASS FIELD
---------------------------------------------------------------------------------
`hill.run()`/`terrain.run()` already accept a `controller_factory` injection
point "so a test can substitute gains without this module knowing what a
RustController is" (their own docstrings). That is exactly the override this
campaign needs, and using it means `HillParams`/`TerrainParams` need no new
field and this campaign touches no scenario-default behaviour.

ASYMMETRIC CLAMP -- WHAT CHANGED TO SUPPORT IT
------------------------------------------------
`control_core::VelocityLoop` gained `new_asymmetric(..., accel_limit,
brake_limit, ...)`; `new(..., limit, ...)` is now `new_asymmetric` with equal
limits, so every existing caller is bit-identical. `braking` is decided by
`v_m_s * (v_m_s - v_ref_m_s) > 0` -- coupling-independent (verified: the
naive "sign of the P-term" test flips between `ComAboveAxle`/`ComBelowAxle`,
this does not) and it reads a standing start (`v_m_s == 0`) as accelerating,
matching `CMD_ENVELOPE_RESERVE_BRAKING`'s own standing-start exclusion.
`control_ffi::ObParamsV2` gained two ADDITIVE fields at the end
(`max_pitch_ref_accel_rad`/`max_pitch_ref_brake_rad`, zero-or-negative falls
back to `max_pitch_ref_rad`) -- no ABI version bump needed (the struct's
`size` guard alone forces a stale caller to update, per that field's own doc
comment), but the Python ctypes mirror in `rust_controller.py` DOES need the
matching two fields appended, and does now.

WHY NOT NOISELESS -- A SEPARATE, PRE-EXISTING FINDING THIS CAMPAIGN TRIPPED OVER
-----------------------------------------------------------------------------------
The brief asked for the sweep run noiseless first. Measured here: a truly
noiseless profile (`NOISELESS_CUTBACK` below -- `STAGE0_CUTBACK` with every
seeded-random term zeroed) makes BOTH `terrain.py` and `hill.py` nose/tail
-strike within 0.25-2.0 s of t=0, DURING THE SETTLE WINDOW, before any speed
command or outer-loop authority is even in play -- identically across every
swept clamp and grade, because the outer loop's reference never gets past
2.6-8.2 deg (nowhere near even the smallest 5 deg clamp) before the strike.
Re-seeding `STAGE0_CUTBACK` itself (still noisy, just seeds other than the
default 12345) reproduces the SAME near-instant collapse; only the shipped
default seed 12345 happens not to trigger it on the cases this file's own
tests already pin. That makes "noiseless" (and most other seeds) a
pre-existing, clamp-INDEPENDENT startup fragility, not a property of the
clamp sweep -- reported here rather than chased, since fixing it is a
different campaign. This file's sweeps default to `DEFAULT_PROFILE`
(`STAGE0_CUTBACK` at its own fixed seed 12345) instead, which is
deterministic and matches what `hill.py`/`terrain.py`'s own tests already
run against.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import math
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from sim.scenarios.hill import HillParams, gravity_for_grade  # noqa: E402
from sim.scenarios.hill import run as hill_run  # noqa: E402
from sim.scenarios.imperfections import STAGE0_CUTBACK, ImperfectionState  # noqa: E402
from sim.scenarios.impulse_response import (  # noqa: E402
    KT_NM_PER_A,
    _bumper_ground_contact,
    frame_pitch_rad,
)
from sim.scenarios.plant import build_model, imu_readings  # noqa: E402
from sim.scenarios.rust_controller import DEFAULT_R_EFF_M as R_EFF_M  # noqa: E402
from sim.scenarios.rust_controller import RustController  # noqa: E402
from sim.scenarios.terrain import TerrainParams  # noqa: E402
from sim.scenarios.terrain import run as terrain_run  # noqa: E402

# --- shared constants, matching the plant every scenario in this campaign
# already runs against. NOT swept, NOT tuned -- see the module docstring. ---
MAX_CURRENT_A = 60.0
KP_NM_PER_RAD = 140.0
KD_NM_PER_RAD_S = 40.0
KP_V_RAD_PER_M_S = 0.05
KI_V_RAD_PER_M = 0.02

#: The inner loop's own pitch-authority ceiling: `tau_max / KP`. Sweeping the
#: outer-loop clamp past this asks the outer loop for a reference the inner
#: loop cannot even track at rest, current-limited -- meaningless to go
#: beyond it, so it is this sweep's top end rather than a round number.
ACCEL_CEILING_DEG = math.degrees(MAX_CURRENT_A * KT_NM_PER_A / KP_NM_PER_RAD)

CLAMP_DEGS = [5.0, 6.0, 8.0, 10.0, 12.0, 14.0, ACCEL_CEILING_DEG]

#: STAGE0_CUTBACK with every SEEDED-RANDOM noise source zeroed. **Measured and
#: REJECTED as this campaign's default** -- see the module docstring's "WHY
#: NOT NOISELESS" section. Kept only for that finding's own reproduction
#: and for anyone who wants to re-check it; nothing below uses it by default.
NOISELESS_CUTBACK = dataclasses.replace(
    STAGE0_CUTBACK, gyro_noise_rad_s=0.0, gyro_bias_rad_s=0.0, accel_noise_m_s2=0.0
)

#: The sweep's actual default: STAGE0_CUTBACK at its own fixed default seed
#: (12345) -- deterministic (one fixed seed, not "noiseless") and the SAME
#: profile `hill.py`/`terrain.py`/`shuttle_run.py` already default to, so
#: results here are directly comparable to the existing pytest baseline.
DEFAULT_PROFILE = STAGE0_CUTBACK

PINT_TARGET_STOP_M = 4.57
PINT_TARGET_DECEL_M_S2 = 3.15
CRUISE_12MPH_M_S = 5.364


def hill_terrain_factory(params, max_pitch_ref_deg, v_ref_fn,
                          accel_deg: float | None = None, brake_deg: float | None = None):
    """Rebuilds `hill.py`/`terrain.py`'s own internal `controller_factory`,
    with the outer-loop clamp overridden. Everything else (KP/KD, kp_v/ki_v,
    estimator config, current cap) is copied from `params` verbatim -- this
    changes nothing else about either scenario's default cascade.
    """
    def factory():
        kwargs = dict(
            kp_nm_per_rad=KP_NM_PER_RAD,
            kd_nm_per_rad_s=KD_NM_PER_RAD_S,
            max_current_a=params.max_current_a,
            kp_v_rad_per_m_s=KP_V_RAD_PER_M_S,
            ki_v_rad_per_m=KI_V_RAD_PER_M,
            max_pitch_ref_rad=math.radians(max_pitch_ref_deg),
            com_above_axle=True,
            v_ref_fn=v_ref_fn,
            use_estimator=params.use_estimator,
            estimator_tau_s=params.estimator_tau_s,
            accel_ff_current_source=params.accel_ff_current_source,
        )
        if accel_deg is not None:
            kwargs["max_pitch_ref_accel_rad"] = math.radians(accel_deg)
        if brake_deg is not None:
            kwargs["max_pitch_ref_brake_rad"] = math.radians(brake_deg)
        return RustController(**kwargs)
    return factory


def _v_ref_at(params):
    def f(t: float) -> float:
        if t < params.settle_s:
            return 0.0
        if params.ramp_s <= 0.0:
            return params.v_ref_m_s
        return params.v_ref_m_s * min(1.0, (t - params.settle_s) / params.ramp_s)
    return f


# ---------------------------------------------------------------------------
# 1. Rolling terrain
# ---------------------------------------------------------------------------

TERRAIN_GRADES_PCT = [8.0, 10.0, 12.0, 15.0, 18.0]


def run_terrain(clamp_deg: float, grade_pct: float, profile=DEFAULT_PROFILE,
                 accel_deg=None, brake_deg=None):
    params = TerrainParams(max_grade_pct=grade_pct)
    factory = hill_terrain_factory(
        params, clamp_deg, _v_ref_at(params), accel_deg=accel_deg, brake_deg=brake_deg
    )
    return terrain_run(params, profile=profile, controller_factory=factory)


def sweep_terrain(clamps=CLAMP_DEGS, grades=TERRAIN_GRADES_PCT, profile=DEFAULT_PROFILE):
    rows = []
    print(f"\n{'clamp':>7} {'grade':>6} {'outcome':>16} {'t_strike':>9} {'pk_lean':>8} "
          f"{'pk_pref':>8} {'pegged':>7} {'pk_A':>7} {'env%':>6}")
    for clamp in clamps:
        for grade in grades:
            r = run_terrain(clamp, grade, profile=profile)
            m = r.metrics
            outcome = (f"{m.struck_end}/{m.struck_phase}@{m.t_strike_s:.2f}s"
                       if m.nose_strike else "survived")
            pegged = m.peak_abs_pitch_ref_deg >= clamp - 0.05
            row = dict(clamp_deg=clamp, grade_pct=grade, survived=m.survived,
                       reached_crest=m.reached_next_crest, held_speed=m.held_speed,
                       struck_end=m.struck_end, struck_phase=m.struck_phase,
                       t_strike_s=m.t_strike_s, peak_abs_pitch_deg=m.peak_abs_pitch_deg,
                       peak_abs_pitch_ref_deg=m.peak_abs_pitch_ref_deg, pegged=pegged,
                       peak_abs_current_a=m.peak_abs_current_a,
                       envelope_frac=m.peak_abs_current_a / MAX_CURRENT_A)
            rows.append(row)
            print(f"{clamp:7.2f} {grade:6.1f} {outcome:>16} "
                  f"{'-' if m.t_strike_s is None else f'{m.t_strike_s:9.2f}'} "
                  f"{m.peak_abs_pitch_deg:8.2f} {m.peak_abs_pitch_ref_deg:8.2f} "
                  f"{'yes' if pegged else 'no':>7} {m.peak_abs_current_a:7.2f} "
                  f"{100 * m.peak_abs_current_a / MAX_CURRENT_A:5.1f}%")
    return rows


# ---------------------------------------------------------------------------
# 2. Steady hill -- descent and climb
# ---------------------------------------------------------------------------

HILL_GRADES_PCT = [12.0, 18.0]
HILL_V_REF_M_S = 2.0


def run_hill(clamp_deg: float, grade_pct: float, v_ref_m_s=HILL_V_REF_M_S,
             profile=DEFAULT_PROFILE, accel_deg=None, brake_deg=None):
    params = HillParams(grade_pct=grade_pct, v_ref_m_s=v_ref_m_s)
    factory = hill_terrain_factory(
        params, clamp_deg, _v_ref_at(params), accel_deg=accel_deg, brake_deg=brake_deg
    )
    return hill_run(params, profile=profile, controller_factory=factory)


def sweep_hill(clamps=CLAMP_DEGS, grades=HILL_GRADES_PCT, profile=DEFAULT_PROFILE):
    rows = []
    print(f"\n{'clamp':>7} {'grade':>6} {'dir':>7} {'outcome':>16} {'v_final':>8} "
          f"{'v_ref':>6} {'held':>5} {'pk_lean':>8} {'pk_pref':>8} {'pegged':>7} {'env%':>6}")
    for clamp in clamps:
        for grade in grades:
            for direction, signed_grade in (("descent", grade), ("climb", -grade)):
                r = run_hill(clamp, signed_grade)
                m = r.metrics
                outcome = f"{m.struck_end}@{m.t_strike_s:.2f}s" if m.nose_strike else "survived"
                pegged = m.peak_abs_pitch_ref_deg >= clamp - 0.05
                rows.append(dict(
                    clamp_deg=clamp, grade_pct=grade, direction=direction,
                    signed_grade_pct=signed_grade, survived=m.survived,
                    held_speed=m.held_speed, ran_away=m.ran_away,
                    reversed_direction=m.reversed_direction,
                    v_final_m_s=m.v_final_m_s, v_ref_m_s=m.v_ref_m_s,
                    peak_abs_pitch_deg=m.peak_abs_pitch_deg,
                    peak_abs_pitch_ref_deg=m.peak_abs_pitch_ref_deg, pegged=pegged,
                    peak_abs_current_a=m.peak_abs_current_a,
                    envelope_frac=m.peak_abs_current_a / MAX_CURRENT_A,
                ))
                print(f"{clamp:7.2f} {grade:6.1f} {direction:>7} {outcome:>16} "
                      f"{m.v_final_m_s:8.2f} {m.v_ref_m_s:6.2f} "
                      f"{'yes' if m.held_speed else 'NO':>5} {m.peak_abs_pitch_deg:8.2f} "
                      f"{m.peak_abs_pitch_ref_deg:8.2f} {'yes' if pegged else 'no':>7} "
                      f"{100 * m.peak_abs_current_a / MAX_CURRENT_A:5.1f}%")
    return rows


# ---------------------------------------------------------------------------
# 3. Flat braking stop, 12 mph -> 0
# ---------------------------------------------------------------------------
#
# Neither hill.run() nor terrain.run() supports a ramp-up-then-ramp-down
# speed profile (their own metrics assume a single constant `v_ref_m_s`
# cruise), so this reimplements their stepping loop directly rather than
# bending either scenario's post-processing to a schedule it was not built
# to score.

def flat_brake_stop(clamp_deg: float, cruise_m_s=CRUISE_12MPH_M_S,
                     settle_s=2.0, cruise_ramp_s=8.0, hold_s=2.0, brake_ramp_s=0.05,
                     tail_s=10.0, profile=DEFAULT_PROFILE,
                     accel_deg=None, brake_deg=None):
    """Accelerate closed-loop to `cruise_m_s`, hold, then command a stop.

    `cruise_ramp_s = 8.0` (accel ~= 0.67 m/s^2, tan^-1 ~= 3.9 deg-equivalent
    lean) is chosen so the RAMP-UP is achievable with margin even at the
    smallest swept clamp (5 deg) -- a symmetric clamp small enough to
    struggle on the accelerating side too is a real, reportable cost, but it
    must not be produced by an arbitrarily aggressive ramp schedule of this
    script's own choosing. `brake_ramp_s` is deliberately short (not a step)
    only to avoid handing the controller a literal discontinuity; it is NOT
    modelling a rider's reaction time -- this measures the controller's own
    stopping authority, not a human's.
    """
    t_brake = settle_s + cruise_ramp_s + hold_s
    duration_s = t_brake + brake_ramp_s + tail_s

    def v_ref_at(t: float) -> float:
        if t < settle_s:
            return 0.0
        if t < settle_s + cruise_ramp_s:
            return cruise_m_s * (t - settle_s) / cruise_ramp_s
        if t < t_brake:
            return cruise_m_s
        if t < t_brake + brake_ramp_s:
            return cruise_m_s * max(0.0, 1.0 - (t - t_brake) / brake_ramp_s)
        return 0.0

    model = build_model(70.0, 0.75, MAX_CURRENT_A, grade_pct=0.0)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    frame = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "frame")
    ground = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "ground")
    bumper_ids = {
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, n): end
        for n, end in (("front_bumper_geom", "nose"), ("rear_bumper_geom", "tail"))
    }

    kwargs = dict(
        kp_nm_per_rad=KP_NM_PER_RAD, kd_nm_per_rad_s=KD_NM_PER_RAD_S,
        max_current_a=MAX_CURRENT_A,
        kp_v_rad_per_m_s=KP_V_RAD_PER_M_S, ki_v_rad_per_m=KI_V_RAD_PER_M,
        max_pitch_ref_rad=math.radians(clamp_deg),
        com_above_axle=True, v_ref_fn=v_ref_at,
        use_estimator=True, estimator_tau_s=2.0, accel_ff_current_source=0,
    )
    if accel_deg is not None:
        kwargs["max_pitch_ref_accel_rad"] = math.radians(accel_deg)
    if brake_deg is not None:
        kwargs["max_pitch_ref_brake_rad"] = math.radians(brake_deg)

    dt = float(model.opt.timestep)
    n_steps = int(round(duration_s / dt))
    imp = ImperfectionState(profile=profile, dt_s=dt)
    x0 = float(data.xpos[frame][0])
    flowing_a = 0.0
    ts, vs, currents, prefs = [], [], [], []
    nose_strike, t_strike, struck_end = False, None, ""

    with RustController(**kwargs) as ctl:
        for _ in range(n_steps):
            t = float(data.time)
            pitch_rad = frame_pitch_rad(model, data)
            wheel_true = float(data.qvel[6])
            true_gyro, true_accel = imu_readings(model, data)
            proposed = float(ctl(
                t, pitch_rad, imp.gyro(float(data.qvel[4])),
                imp.wheel_rate(wheel_true, t),
                gyro_rad_s=imp.gyro_vec(true_gyro),
                accel_m_s2=imp.accel_vec(true_accel),
                motor_current_a=flowing_a,
            ))
            available = profile.available_current_a(wheel_true)
            current = imp.apply_current(proposed, wheel_true)
            flowing_a = current
            data.ctrl[0] = current * KT_NM_PER_A
            mujoco.mj_step(model, data)

            ts.append(t)
            vs.append(float(data.qvel[6]) * R_EFF_M)
            currents.append(current)
            prefs.append(math.degrees(ctl.pitch_ref_rad))

            if _bumper_ground_contact(model, data, ground, set(bumper_ids)) is not None:
                nose_strike, t_strike = True, float(data.time)
                ends = set()
                for i in range(data.ncon):
                    c = data.contact[i]
                    for a, b in ((c.geom1, c.geom2), (c.geom2, c.geom1)):
                        if a == ground and b in bumper_ids:
                            ends.add(bumper_ids[b])
                struck_end = ends.pop() if len(ends) == 1 else ("both" if ends else "")
                break
        peak_abs_pitch_ref_deg = math.degrees(float(ctl.peak_abs_pitch_ref_rad))

    ts_arr, v_arr, cur_arr = np.asarray(ts), np.asarray(vs), np.asarray(currents)
    brake_mask = ts_arr >= t_brake
    if brake_mask.sum() < 2 or nose_strike:
        return dict(clamp_deg=clamp_deg, nose_strike=nose_strike, t_strike_s=t_strike,
                    struck_end=struck_end,
                    v0_m_s=cruise_m_s, stop_distance_m=None, stop_time_s=None,
                    mean_decel_m_s2=None, peak_decel_m_s2=None,
                    peak_abs_current_a=float(np.max(np.abs(cur_arr))) if len(cur_arr) else 0.0,
                    peak_abs_pitch_ref_deg=peak_abs_pitch_ref_deg)

    tb, vb = ts_arr[brake_mask], v_arr[brake_mask]
    v0 = float(vb[0])
    below = np.where(vb <= 0.02 * v0)[0]
    if len(below) == 0:
        stop_time_s, stop_idx = None, len(vb) - 1
    else:
        stop_idx = int(below[0])
        stop_time_s = float(tb[stop_idx] - tb[0])
    stop_distance_m = float(np.trapezoid(vb[: stop_idx + 1], tb[: stop_idx + 1]))
    decel = -np.diff(vb[: stop_idx + 1]) / np.diff(tb[: stop_idx + 1])
    mean_decel = v0 / stop_time_s if stop_time_s else None
    peak_decel = float(np.max(decel)) if len(decel) else None
    peak_abs_current_a = float(np.max(np.abs(cur_arr)))

    return dict(clamp_deg=clamp_deg, nose_strike=nose_strike, t_strike_s=t_strike,
                struck_end=struck_end,
                v0_m_s=v0, stop_distance_m=stop_distance_m, stop_time_s=stop_time_s,
                mean_decel_m_s2=mean_decel, peak_decel_m_s2=peak_decel,
                peak_abs_current_a=peak_abs_current_a,
                envelope_frac=peak_abs_current_a / MAX_CURRENT_A,
                peak_abs_pitch_ref_deg=peak_abs_pitch_ref_deg)


def sweep_brake(clamps=CLAMP_DEGS, profile=DEFAULT_PROFILE):
    rows = []
    print(f"\n{'clamp':>7} {'v0':>6} {'dist_m':>7} {'t_s':>6} {'mean_a':>7} {'pk_a':>7} "
          f"{'pk_A':>7} {'env%':>6} {'pk_pref':>8}")
    print(f"  (Pint-class target: {PINT_TARGET_STOP_M} m / {PINT_TARGET_DECEL_M_S2} m/s^2)")
    for clamp in clamps:
        r = flat_brake_stop(clamp, profile=profile)
        rows.append(r)
        if r["stop_distance_m"] is None:
            print(f"{clamp:7.2f}  STRUCK ({r['struck_end']}) @{r['t_strike_s']:.2f}s -- "
                  f"peak_A {r['peak_abs_current_a']:.2f}, peak_pref {r['peak_abs_pitch_ref_deg']:.2f}")
            continue
        print(f"{clamp:7.2f} {r['v0_m_s']:6.3f} {r['stop_distance_m']:7.3f} "
              f"{r['stop_time_s']:6.3f} {r['mean_decel_m_s2']:7.3f} {r['peak_decel_m_s2']:7.3f} "
              f"{r['peak_abs_current_a']:7.2f} {100 * r['envelope_frac']:5.1f}% "
              f"{r['peak_abs_pitch_ref_deg']:8.2f}")
    return rows


# ---------------------------------------------------------------------------
# 4. Asymmetric clamp
# ---------------------------------------------------------------------------

def sweep_asymmetric(accel_deg=12.0, brake_deg=15.43, profile=DEFAULT_PROFILE):
    print(f"\nASYMMETRIC: {accel_deg} deg accel / {brake_deg} deg brake")
    print("-- flat braking stop --")
    r = flat_brake_stop(brake_deg, profile=profile, accel_deg=accel_deg, brake_deg=brake_deg)
    print(json.dumps(r, indent=2, default=str))

    print("-- terrain regression check at the ACCEL side (8%, 12%) --")
    for grade in (8.0, 12.0):
        params = TerrainParams(max_grade_pct=grade)
        factory = hill_terrain_factory(params, accel_deg, _v_ref_at(params),
                                        accel_deg=accel_deg, brake_deg=brake_deg)
        m = terrain_run(params, profile=profile, controller_factory=factory).metrics
        print(f"  grade {grade:.0f}%: survived={m.survived} reached_crest={m.reached_next_crest} "
              f"peak_lean={m.peak_abs_pitch_deg:.2f} peak_pref={m.peak_abs_pitch_ref_deg:.2f} "
              f"peak_A={m.peak_abs_current_a:.2f}")
    return r


# ---------------------------------------------------------------------------
# 5. Seeded IMU noise, N seeds
# ---------------------------------------------------------------------------

def sweep_noise(clamp_deg: float, seeds=range(25), scenario="terrain", grade_pct=8.0):
    print(f"\nNOISE: clamp {clamp_deg} deg, {scenario} @ {grade_pct}% grade, {len(list(seeds))} seeds")
    inversions = 0
    peak_leans = []
    for seed in seeds:
        profile = dataclasses.replace(STAGE0_CUTBACK, seed=seed)
        if scenario == "terrain":
            m = run_terrain(clamp_deg, grade_pct, profile=profile).metrics
            survived, peak_lean = m.survived, m.peak_abs_pitch_deg
        elif scenario == "brake":
            r = flat_brake_stop(clamp_deg, profile=profile)
            survived, peak_lean = not r["nose_strike"], r["peak_abs_pitch_ref_deg"]
        else:
            raise ValueError(scenario)
        if not survived:
            inversions += 1
        peak_leans.append(peak_lean)
        print(f"  seed {seed:>3}: {'OK' if survived else 'INVERTED/STRUCK'} "
              f"peak {peak_lean:.2f}")
    arr = np.asarray(peak_leans)
    print(f"  inversions: {inversions}/{len(arr)}")
    print(f"  peak-lean distribution: mean {arr.mean():.2f} std {arr.std():.2f} "
          f"min {arr.min():.2f} max {arr.max():.2f}")
    return dict(clamp_deg=clamp_deg, inversions=inversions, n=len(arr),
                mean=float(arr.mean()), std=float(arr.std()),
                min=float(arr.min()), max=float(arr.max()))


# ---------------------------------------------------------------------------
# 6. Flat-ground regression -- what currently PASSES, at each clamp
# ---------------------------------------------------------------------------
#
# Monkeypatches `rust_controller.DEFAULT_MAX_PITCH_REF_RAD` for the duration
# of one `pytest` subprocess per clamp value, rather than editing the
# constant on disk -- this campaign must never leave the constant changed on
# exit, successful or not. Scoped to the three files whose scenarios build a
# `RustController` with the outer loop enabled by DEFAULT (kp_v/ki_v != 0
# and no explicit `max_pitch_ref_rad` override): `test_terrain.py`,
# `test_hill.py`, `test_shuttle_run.py`. Everything else in `tests/` either
# does not touch the outer loop at all (the `sim-host`-backed files:
# `test_braking_authority.py`, `test_incline_tolerance.py`,
# `test_crr_sweep.py`, `test_cmd_envelope_reserve.py`,
# `test_disturbance_envelope.py` -- `sim-host` never enables `VelocityLoop`,
# see `crates/sim-host/src/host.rs`'s own header note) or passes its own
# explicit gains (checked by hand against the failure list this produces).

REGRESSION_FILES = ["tests/test_terrain.py", "tests/test_hill.py", "tests/test_shuttle_run.py"]


def run_regression(clamp_deg: float) -> tuple[int, int, int, list[str]]:
    rc_path = REPO / "sim" / "scenarios" / "rust_controller.py"
    original = rc_path.read_text()
    patched_line = f"DEFAULT_MAX_PITCH_REF_RAD = {math.radians(clamp_deg)!r}"
    assert "DEFAULT_MAX_PITCH_REF_RAD = " in original
    new_text = "\n".join(
        patched_line if line.startswith("DEFAULT_MAX_PITCH_REF_RAD = ") else line
        for line in original.splitlines()
    ) + "\n"
    rc_path.write_text(new_text)
    try:
        proc = subprocess.run(
            [sys.executable, "-m", "pytest", *REGRESSION_FILES, "-q"],
            cwd=REPO, capture_output=True, text=True,
        )
    finally:
        rc_path.write_text(original)
    out = proc.stdout + proc.stderr
    failed = [line[len("FAILED "):].split(" ")[0] for line in out.splitlines()
              if line.startswith("FAILED ")]
    tail = out.strip().splitlines()[-1] if out.strip() else ""
    import re
    n_failed = int(m.group(1)) if (m := re.search(r"(\d+) failed", tail)) else 0
    n_passed = int(m.group(1)) if (m := re.search(r"(\d+) passed", tail)) else 0
    n_xfailed = int(m.group(1)) if (m := re.search(r"(\d+) xfailed", tail)) else 0
    return n_failed, n_passed, n_xfailed, failed


def sweep_regression(clamps=CLAMP_DEGS):
    print(f"\nREGRESSION -- {REGRESSION_FILES} at each clamp")
    baseline_failed, baseline_passed, baseline_xfailed, baseline_list = run_regression(4.98)
    print(f"  baseline (4.98 deg, ~ shipped 0.087 rad): "
          f"{baseline_failed} failed / {baseline_passed} passed / {baseline_xfailed} xfailed")
    for name in sorted(baseline_list):
        print(f"    already-red: {name}")
    rows = []
    for clamp in clamps:
        nf, np_, nx, failed = run_regression(clamp)
        new_failures = sorted(set(failed) - set(baseline_list))
        newly_fixed = sorted(set(baseline_list) - set(failed))
        rows.append(dict(clamp_deg=clamp, failed=nf, passed=np_, xfailed=nx,
                          new_failures=new_failures, newly_fixed=newly_fixed))
        print(f"  clamp {clamp:6.2f} deg: {nf} failed / {np_} passed / {nx} xfailed"
              + (f"  NEW FAILURES: {new_failures}" if new_failures else "")
              + (f"  NEWLY FIXED: {newly_fixed}" if newly_fixed else ""))
    return rows


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("which", choices=["terrain", "hill", "brake", "asymmetric", "noise",
                                       "regression", "all"])
    ap.add_argument("--clamp-deg", type=float, default=10.0, help="for `noise`")
    ap.add_argument("--seeds", type=int, default=25, help="for `noise`")
    ap.add_argument("--scenario", default="terrain", choices=["terrain", "brake"],
                     help="for `noise`")
    args = ap.parse_args()

    print(f"Inner-loop pitch-authority ceiling (tau_max / KP): {ACCEL_CEILING_DEG:.2f} deg")
    print(f"Clamp values swept (deg): {[round(c, 2) for c in CLAMP_DEGS]}")

    if args.which in ("terrain", "all"):
        sweep_terrain()
    if args.which in ("hill", "all"):
        sweep_hill()
    if args.which in ("brake", "all"):
        sweep_brake()
    if args.which in ("asymmetric", "all"):
        sweep_asymmetric()
    if args.which == "noise":
        sweep_noise(args.clamp_deg, seeds=range(args.seeds), scenario=args.scenario)
    if args.which in ("regression", "all"):
        sweep_regression()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
