#!/usr/bin/env python3
"""Estimator analysis: produce the dataset and the plots behind the write-up.

Kept as a script rather than a notebook *for now* on purpose — it runs in CI's
environment, it is diffable, and it emits a plain `.npz` that a notebook can
load later without re-running any physics. When the notebooks land they should
read this output rather than duplicate it.

    scripts/analyse_estimator.py --out-dir sim/out/experiments
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

from sim.scenarios.imperfections import STAGE0_PLACEHOLDER, ImperfectionState  # noqa: E402
from sim.scenarios.impulse_response import frame_pitch_rad  # noqa: E402
from sim.scenarios.plant import build_model, imu_readings  # noqa: E402
from sim.scenarios.rust_controller import RustController  # noqa: E402
from sim.scenarios.shuttle_run import ShuttleParams, VelocityProfile  # noqa: E402

INK, AMBER, MINT, MUTED = "#16232E", "#F2A24A", "#2AAE97", "#96A8B0"


def collect(kind: str) -> dict:
    """Run one scenario on TRUTH and record everything an estimator would see.

    Truth-driven on purpose: the trajectory is then the known-good one, so
    estimator variants can be compared against each other rather than each
    against the different trajectory it caused.
    """
    model = build_model(70.0, 0.75, 40.0)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    dt = float(model.opt.timestep)
    frame = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "frame")
    imp = ImperfectionState(profile=STAGE0_PLACEHOLDER, dt_s=dt)

    vref_fn = None
    if kind == "shuttle":
        vref_fn = VelocityProfile(ShuttleParams()).v_ref
        n = int(VelocityProfile(ShuttleParams()).duration_s / dt)
    else:
        n = int(8.0 / dt)

    t, th, ax, az, gy, v = [], [], [], [], [], []
    # TRUTH PITCH, DELIBERATELY, AND STATED SO RATHER THAN INHERITED.
    #
    # `harvest` flies the trajectory on truth and records the raw IMU; the
    # filter is then run OFFLINE over those arrays by `run_filter` below. That
    # is a legitimate open-loop analysis and it is what this function is for.
    #
    # It is written out because leaving it to the default cost real time. The
    # numbers this produces (<1 deg RMS) were compared against the scenario
    # harness's closed-loop numbers (7.4 deg RMS) as though the two measured the
    # same thing, and the 10x gap was carried in the handoff as an unexplained
    # defect. They are not the same measurement: open-loop replay over a
    # truth-flown trajectory versus closed-loop error that moves the trajectory
    # it is measured on. Nothing was broken; the comparison was.
    #
    # Do NOT "fix" this to use_estimator=1. That would make `run_filter`'s
    # offline pass meaningless, because the trajectory would already have been
    # flown on the estimate.
    with RustController(
        kp_a_per_rad=200.0, kd_a_per_rad_s=30.0, max_current_a=40.0,
        kp_v_rad_per_m_s=0.05, ki_v_rad_per_m=0.02, com_above_axle=True,
        v_ref_fn=vref_fn,
        use_estimator=False,
    ) as c:
        for _ in range(n):
            now = float(data.time)
            data.xfrc_applied[frame] = 0.0
            if kind == "impulse" and 0.5 <= now < 0.55:
                data.xfrc_applied[frame][:3] = [-400.0, 0, 0]
            p = frame_pitch_rad(model, data)
            cur = float(c(now, p, float(data.qvel[4]), float(data.qvel[6])))
            data.ctrl[0] = imp.apply_current(cur) * 0.7
            mujoco.mj_step(model, data)
            g, a = imu_readings(model, data)
            t.append(float(data.time))
            th.append(frame_pitch_rad(model, data))
            ax.append(a[0]); az.append(a[2]); gy.append(g[1])
            v.append(float(data.qvel[6]) * 0.14605)
    return {k: np.asarray(x) for k, x in
            dict(t=t, truth=th, ax=ax, az=az, gyro=gy, v=v, dt=dt).items()}


def low_pass(x: np.ndarray, tau: float, dt: float) -> np.ndarray:
    y = np.zeros_like(x)
    a = dt / (tau + dt)
    for i in range(1, len(x)):
        y[i] = y[i - 1] + a * (x[i] - y[i - 1])
    return y


def filter_pitch(d: dict, tau: float, aiding: bool) -> np.ndarray:
    """Replay the complementary filter offline, exactly as the Rust one runs."""
    dt = float(d["dt"])
    a_fwd = low_pass(np.gradient(d["v"], dt), 0.05, dt) if aiding else np.zeros_like(d["v"])
    theta = np.arctan2(d["ax"][0] - a_fwd[0], -d["az"][0])
    out = np.empty_like(d["truth"])
    alpha = tau / (tau + dt)
    for i in range(len(out)):
        th_acc = np.arctan2(d["ax"][i] - a_fwd[i], -d["az"][i])
        theta = alpha * (theta + d["gyro"][i] * dt) + (1 - alpha) * th_acc if i else th_acc
        out[i] = theta
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out-dir", type=Path, default=REPO / "sim" / "out" / "experiments")
    args = ap.parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    summary = {}
    fig, axes = plt.subplots(2, 2, figsize=(13, 7.5))

    for col, kind in enumerate(("impulse", "shuttle")):
        d = collect(kind)
        t, truth = d["t"], np.degrees(d["truth"])
        raw = np.degrees(np.arctan2(d["ax"], -d["az"]))
        no_aid = np.degrees(filter_pitch(d, 1.0, aiding=False))
        aided = np.degrees(filter_pitch(d, 1.0, aiding=True))

        def rms(x):
            return float(np.sqrt(((x - truth) ** 2).mean()))

        summary[kind] = {
            "raw_accel_rms_deg": round(rms(raw), 3),
            "filter_rms_deg": round(rms(no_aid), 3),
            "filter_aided_rms_deg": round(rms(aided), 3),
            "improvement_pct": round(100 * (1 - rms(aided) / max(rms(no_aid), 1e-9)), 1),
        }

        a = axes[0][col]
        a.plot(t, truth, color=INK, lw=2.0, label="truth")
        a.plot(t, raw, color=MUTED, lw=0.8, alpha=0.75, label="raw accelerometer")
        a.plot(t, aided, color=MINT, lw=1.4, label="filter + accel aiding")
        a.set_title(f"{kind}: attitude", color=INK)
        a.set_ylabel("pitch (deg)")
        a.set_ylim(min(truth.min(), -8) - 4, max(truth.max(), 8) + 4)
        a.legend(fontsize=8, loc="upper right")
        a.grid(alpha=0.25)

        b = axes[1][col]
        b.plot(t, np.abs(no_aid - truth), color=AMBER, lw=1.2, label="filter, no aiding")
        b.plot(t, np.abs(aided - truth), color=MINT, lw=1.2, label="filter + aiding")
        b.axhline(0.5, ls="--", lw=1.2, color=INK, label="AC: 0.5 deg RMS")
        b.set_title(f"{kind}: estimator error   "
                    f"(RMS {summary[kind]['filter_rms_deg']:.2f} -> "
                    f"{summary[kind]['filter_aided_rms_deg']:.2f} deg)", color=INK)
        b.set_xlabel("time (s)"); b.set_ylabel("|error| (deg)")
        # Floor the log axis: the first few samples are exactly on truth and
        # would otherwise stretch the scale to 1e-18 and flatten everything
        # that matters into a single band at the top.
        b.set_yscale("log"); b.set_ylim(1e-3, 30)
        b.legend(fontsize=8); b.grid(alpha=0.25)

        np.savez_compressed(args.out_dir / f"estimator-{kind}.npz",
                            t=t, truth=truth, raw=raw, no_aid=no_aid, aided=aided,
                            ax=d["ax"], az=d["az"], gyro=d["gyro"], v=d["v"])

    fig.suptitle("Attitude estimator: the accelerometer is corrupted by the board's own motion",
                 color=INK, fontsize=13)
    fig.tight_layout()
    fig.savefig(args.out_dir / "estimator-analysis.png", dpi=130)
    (args.out_dir / "estimator-analysis.json").write_text(json.dumps(summary, indent=2) + "\n")

    for k, v in summary.items():
        print(f"{k:>9}: raw {v['raw_accel_rms_deg']:6.2f}  filter {v['filter_rms_deg']:6.2f}"
              f"  aided {v['filter_aided_rms_deg']:6.2f} deg RMS"
              f"   ({v['improvement_pct']:+.0f}% from aiding)")
    print(f"wrote {args.out_dir / 'estimator-analysis.png'} and two .npz datasets")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
