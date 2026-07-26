#!/usr/bin/env python3
"""Generate and archive the plant and control-loop datasets.

The estimator work produced `.npz` files as it went; the plant and control-loop
sweeps did not — they ran in throwaway scripts and only their conclusions were
written down. This regenerates them **as data**.

That is legitimate rather than a reconstruction: every scenario here is
deterministic and seeded, so re-running a configuration reproduces it
bit-for-bit. The numbers below are the same numbers the decisions were made on,
archived after the fact rather than during. The notebooks say so too.

    scripts/analyse_control.py --out-dir sim/out/experiments
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

import mujoco  # noqa: E402

from sim.scenarios.imperfections import (  # noqa: E402
    STAGE0_PLACEHOLDER,
    ImperfectionProfile,
)
from sim.scenarios.impulse_response import (  # noqa: E402
    NOMINAL_IMPULSE_NS,
    ImpulseParams,
    frame_pitch_rad,
    run,
)
from sim.scenarios.plant import KT_NM_PER_A, build_model, plant_summary  # noqa: E402
from sim.scenarios.rust_controller import RustController  # noqa: E402

G = 9.81


def ctl(**kw):
    base = dict(kp_a_per_rad=200.0, kd_a_per_rad_s=30.0, max_current_a=40.0,
                com_above_axle=True)
    base.update(kw)
    return RustController(**base)


def impulse(model, secs=8.0, ns=NOMINAL_IMPULSE_NS, profile=STAGE0_PLACEHOLDER, **kw):
    with ctl(**kw) as c:
        r = run(ImpulseParams(magnitude_ns=ns, sim_seconds=secs),
                model=model, controller=c, profile=profile)
        return r, c


# ---------------------------------------------------------------------------
# 1. The plant: how mgl and authority change with rider mass
# ---------------------------------------------------------------------------

def plant_sweep() -> dict:
    rows = []
    for mass, height in ((0.0, 0.0), (10.0, 0.5), (20.0, 0.4), (40.0, 0.6), (70.0, 0.75)):
        m = build_model(mass, height if mass else 0.75, 40.0)
        s = plant_summary(m)
        l = s["com_above_axle_mm"] / 1000.0
        rows.append(dict(
            ballast_kg=mass, height_m=height,
            total_kg=s["total_mass_kg"], com_mm=s["com_above_axle_mm"],
            mgl=s["mgl_n_m_per_rad"], inverted=s["inverted_pendulum"],
            # Torque to hold a 5 deg lean -- the authority the outer loop has.
            tau_5deg_nm=abs(s["mgl_n_m_per_rad"] * np.sin(np.radians(5.0))),
        ))
    return {"rows": rows}


def freeswing() -> dict:
    """The free-swing period, three ways -- the number that got corrected twice.

    Analytic (frame only), analytic (whole board, the original mistake), and
    MEASURED off the model with the frame released from an offset.
    """
    m = build_model(0.0, 0.0, 40.0)
    d = mujoco.MjData(m)
    frame = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "frame")

    m_frame, l = 8.0, 0.030
    I_cm, I_wheel = 0.400, 0.0595
    I_p = I_cm + m_frame * l**2
    analytic_frame = 2 * np.pi * np.sqrt(I_p / (m_frame * G * l))
    analytic_whole = 2 * np.pi * np.sqrt(I_p / (12.5 * G * l))   # the original error
    analytic_locked = 2 * np.pi * np.sqrt((I_p + I_wheel) / (m_frame * G * l))

    # Measured: pin the axle, release from 6 deg, time the zero crossings.
    d.qpos[3:7] = [np.cos(0.0523), 0, np.sin(0.0523), 0]
    d.qpos[0:3] = [0, 0, 0.1454]
    mujoco.mj_forward(m, d)
    t, th = [], []
    for _ in range(8000):
        d.qvel[0:3] = 0.0            # hold the axle: this is the pinned config
        d.qpos[0:3] = [0, 0, 0.1454]
        mujoco.mj_step(m, d)
        t.append(float(d.time)); th.append(frame_pitch_rad(m, d))
    t, th = np.asarray(t), np.asarray(th)
    crossings = t[np.where(np.diff(np.signbit(th)))[0]]
    measured = float(2 * np.mean(np.diff(crossings))) if len(crossings) > 2 else float("nan")

    return {
        "analytic_frame_only_s": round(float(analytic_frame), 4),
        "analytic_whole_board_s_THE_ORIGINAL_ERROR": round(float(analytic_whole), 4),
        "analytic_wheel_locked_s": round(float(analytic_locked), 4),
        "measured_s": round(measured, 4),
        "I_pivot": round(I_p, 5), "mgl": round(m_frame * G * l, 4),
        "t": t, "pitch_rad": th,
    }


# ---------------------------------------------------------------------------
# 2. Control: the gain floor, the coupling sign, the delay margin
# ---------------------------------------------------------------------------

def gain_floor() -> dict:
    """Why the FAILING gains demand more torque than the succeeding ones."""
    m = build_model(70.0, 0.75, 40.0)
    rows = []
    for kp, kd in ((80, 11), (200, 30), (400, 60), (800, 110), (1600, 220)):
        r, c = impulse(m, kp_a_per_rad=kp, kd_a_per_rad_s=kd,
                       kp_v_rad_per_m_s=0.05, ki_v_rad_per_m=0.02)
        tau = np.abs(r.motor_current_a) * KT_NM_PER_A
        rows.append(dict(kp=kp, kd=kd, strike=bool(r.metrics.nose_strike),
                         peak_pitch_deg=round(r.metrics.peak_abs_pitch_deg, 3),
                         peak_torque_nm=round(float(tau.max()), 3),
                         rms_torque_nm=round(float(np.sqrt((tau**2).mean())), 3)))
    return {"rows": rows}


def coupling_sign() -> dict:
    """The measurement that made PlantCoupling an enum.

    Hold a CONSTANT pitch reference on each plant and see which way it travels.
    Only rows where the mean pitch actually tracks the reference are valid.
    """
    out = []
    for label, mass, kp, kd, above in (("driverless", 0.0, 80, 11, False),
                                       ("ridden 70kg", 70.0, 800, 110, True)):
        model = build_model(mass, 0.75, 400.0)
        for ref_deg in (-1.0, +1.0):
            class Hold:
                def __init__(s, ref): s.ref = ref
                def __enter__(s): return s
                def __exit__(s, *a): pass
                def __call__(s, t, p, r, w, **_):
                    return max(-400.0, min(400.0, -(kp * (p - s.ref) + kd * r)))
            r = run(ImpulseParams(magnitude_ns=0.0, sim_seconds=5.0),
                    model=model, controller=Hold(np.radians(ref_deg)))
            held = float(np.mean(r.pitch_deg[-100:]))
            out.append(dict(plant=label, com_above_axle=above, ref_deg=ref_deg,
                            mean_pitch_deg=round(held, 3),
                            tracked=bool(abs(held - ref_deg) < 0.6),
                            travel_m=round(r.metrics.travel_m, 3)))
    return {"rows": out}


def delay_margin() -> dict:
    """How much actuation delay before the loop breaks. ICD estimate: 0.4-1.0 ms."""
    m = build_model(70.0, 0.75, 40.0)
    rows = []
    for ms in (1, 5, 10, 20, 40, 60, 80, 100):
        p = ImperfectionProfile(
            f"delay-{ms}ms", actuation_delay_s=ms / 1000.0,
            current_loop_tau_s=STAGE0_PLACEHOLDER.current_loop_tau_s,
            gyro_noise_rad_s=STAGE0_PLACEHOLDER.gyro_noise_rad_s,
            gyro_bias_rad_s=STAGE0_PLACEHOLDER.gyro_bias_rad_s,
            accel_noise_m_s2=STAGE0_PLACEHOLDER.accel_noise_m_s2,
            wheel_rate_quantum_rad_s=STAGE0_PLACEHOLDER.wheel_rate_quantum_rad_s,
            wheel_rate_update_hz=STAGE0_PLACEHOLDER.wheel_rate_update_hz)
        r, _ = impulse(m, profile=p, kp_v_rad_per_m_s=0.05, ki_v_rad_per_m=0.02)
        rows.append(dict(delay_ms=ms, strike=bool(r.metrics.nose_strike),
                         peak_pitch_deg=round(r.metrics.peak_abs_pitch_deg, 3),
                         peak_current_a=round(float(np.abs(r.motor_current_a).max()), 2)))
    return {"rows": rows}


def outer_loop_traces() -> dict:
    """Inner-only vs cascade, as time series -- the plot the notebook wants."""
    m = build_model(70.0, 0.75, 40.0)
    out = {}
    for label, kw in (("inner_only", {}),
                      ("cascade", dict(kp_v_rad_per_m_s=0.05, ki_v_rad_per_m=0.02))):
        r, _ = impulse(m, secs=12.0, **kw)
        out[label] = dict(t=r.t, pitch_deg=r.pitch_deg, travel_m=r.travel_m,
                          wheel_rate=r.wheel_rate_rads, current_a=r.motor_current_a)
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out-dir", type=Path, default=REPO / "sim" / "out" / "experiments")
    args = ap.parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)

    fs = freeswing()
    fs_arrays = {"freeswing_t": fs.pop("t"), "freeswing_pitch_rad": fs.pop("pitch_rad")}
    traces = outer_loop_traces()

    summary = {
        "note": "Regenerated deterministically. Same configurations, therefore "
                "bit-identical to the runs the decisions were made on.",
        "kt_nm_per_a_UNFITTED": KT_NM_PER_A,
        "plant_sweep": plant_sweep(),
        "freeswing": fs,
        "gain_floor": gain_floor(),
        "coupling_sign": coupling_sign(),
        "delay_margin": delay_margin(),
    }
    (args.out_dir / "control-analysis.json").write_text(json.dumps(summary, indent=2) + "\n")

    np.savez_compressed(
        args.out_dir / "control-analysis.npz", **fs_arrays,
        **{f"{k}_{f}": v for k, d in traces.items() for f, v in d.items()},
    )

    fsr = summary["freeswing"]
    print(f"free-swing  analytic(frame) {fsr['analytic_frame_only_s']:.3f}s  "
          f"analytic(WHOLE BOARD, the error) {fsr['analytic_whole_board_s_THE_ORIGINAL_ERROR']:.3f}s  "
          f"wheel-locked {fsr['analytic_wheel_locked_s']:.3f}s  MEASURED {fsr['measured_s']:.3f}s")
    print("\ngain floor:")
    for r in summary["gain_floor"]["rows"]:
        print(f"  Kp {r['kp']:>5}  strike={str(r['strike']):<5} "
              f"peak {r['peak_pitch_deg']:6.2f} deg  torque {r['peak_torque_nm']:6.2f} Nm")
    print("\ncoupling sign (valid rows only):")
    for r in summary["coupling_sign"]["rows"]:
        if r["tracked"]:
            print(f"  {r['plant']:<12} ref {r['ref_deg']:+.1f} deg -> travel {r['travel_m']:+.2f} m")
    print("\ndelay margin:")
    for r in summary["delay_margin"]["rows"]:
        print(f"  {r['delay_ms']:>4} ms  strike={r['strike']}")
    print(f"\nwrote {args.out_dir/'control-analysis.json'} and .npz")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
