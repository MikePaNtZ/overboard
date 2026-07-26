#!/usr/bin/env python3
"""Run, film and archive a controls experiment.

The problem this solves: the most valuable results in this project so far came
from throwaway diagnostic runs -- the ballast torque sweep, the outer-loop sign
measurement -- and they existed only in a terminal scrollback. A learning that
is not archived with the configuration that produced it is not a result, it is
an anecdote.

Each run writes a MANIFEST alongside the video: the exact plant and gains, the
measured outcome, and the stated learning. The manifest is the thing that makes
a clip re-derivable a month later; the clip on its own is just a board moving.

    scripts/experiment.py --id ballast-kp80 \
        --ballast-mass 70 --ballast-height 0.75 --kp 80 --kd 11 \
        --learning "Soft gains fail on the unstable plant..."

    # A/B two configurations in one two-pane clip:
    scripts/experiment.py --id ballast-gain-sweep --ballast-mass 70 \
        --kp 80 --kd 11 --vs-kp 200 --vs-kd 30 --render

Outputs land in sim/out/experiments/.

Determinism: no wall clock is read. Runs are identified by the git SHA of the
tree that produced them, so the same commit re-runs to the same numbers.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from sim.scenarios.impulse_response import (  # noqa: E402
    KT_NM_PER_A,
    NOMINAL_IMPULSE_NS,
    ImpulseParams,
    load_model,
    run,
)
from sim.scenarios.rust_controller import RustController  # noqa: E402

OUT_DIR = REPO / "sim" / "out" / "experiments"
MODEL_PATH = REPO / "sim" / "models" / "overboard_onewheel.xml"


def git_sha() -> str:
    try:
        out = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=REPO,
                             capture_output=True, text=True, check=True)
        dirty = subprocess.run(["git", "status", "--porcelain"], cwd=REPO,
                               capture_output=True, text=True, check=True)
        return out.stdout.strip() + ("-dirty" if dirty.stdout.strip() else "")
    except Exception:
        return "unknown"


def build_model(ballast_mass: float, ballast_height: float, clamp_a: float):
    """The stock model, optionally with a rigid rider-proxy mass above the axle.

    A rigid ballast is NOT a rider: a real one is compliant at the ankles and
    knees, which changes the dynamics substantially. It is the honest
    order-of-magnitude stand-in for "what happens when the centre of mass moves
    above the axle", which is the property that matters here.
    """
    import mujoco

    xml = MODEL_PATH.read_text()
    if clamp_a is not None:
        lim = clamp_a * KT_NM_PER_A
        xml = xml.replace('ctrlrange="-28 28"', f'ctrlrange="{-lim:g} {lim:g}"')
    if ballast_mass > 0:
        # The geom is VISUAL ONLY -- contype/conaffinity 0 so it collides with
        # nothing, and the explicit <inertial> means it contributes no mass of
        # its own. Without it the ballast is invisible in the render, and an
        # archived clip would show a bare board while 70 kg sits above the
        # axle, which is exactly the kind of quietly misleading artifact this
        # archive exists to avoid.
        body = (
            f'\n      <body name="ballast" pos="0 0 {ballast_height}">'
            f'<inertial pos="0 0 0" mass="{ballast_mass}" '
            f'diaginertia="{ballast_mass * 0.15:.4f} {ballast_mass * 0.15:.4f} '
            f'{ballast_mass * 0.08:.4f}"/>'
            f'<geom name="ballast_viz" type="sphere" size="0.12" '
            f'rgba="0.949 0.635 0.290 0.45" contype="0" conaffinity="0" group="1"/>'
            f'</body>\n'
        )
        xml = xml.replace('      <site name="imu"', body + '      <site name="imu"', 1)
        # Widen the field of view. The cameras are framed for a 0.3 m-tall
        # board; a ballast 0.75 m above the axle falls outside them, and a clip
        # that crops out the mass whose whole point is being there is worse
        # than no clip.
        xml = xml.replace('fovy="26"', 'fovy="52"').replace('fovy="24"', 'fovy="50"')

    # Load from a string with the meshes supplied in-memory, rather than
    # writing a temporary MJCF next to the real one. A temp file in the model
    # directory has to be cleaned up on every exit path, and a stray one is
    # indistinguishable from a real model to the next person who looks.
    mesh_dir = MODEL_PATH.parent / "meshes" / "openwheel"
    assets = {p.name: p.read_bytes() for p in mesh_dir.glob("*.stl")}
    return mujoco.MjModel.from_xml_string(xml, assets)


def plant_summary(model) -> dict:
    """The two numbers that decide what kind of plant this is."""
    import mujoco

    d = mujoco.MjData(model)
    mujoco.mj_forward(model, d)
    axle_z = float(d.xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "frame")][2])
    total = float(sum(model.body_mass))
    com_z = float(sum(model.body_mass[i] * d.xipos[i][2] for i in range(model.nbody)) / total)
    l = com_z - axle_z
    return {
        "total_mass_kg": round(total, 2),
        "com_above_axle_mm": round(l * 1000.0, 1),
        # Positive mgl means gravity DESTABILISES -- an inverted pendulum.
        # Negative means it restores. The sign is the plant's character.
        "mgl_n_m_per_rad": round(total * 9.81 * l, 2),
        "inverted_pendulum": l > 0,
    }


def one_run(model, kp, kd, clamp_a, impulse, seconds):
    with RustController(kp_a_per_rad=kp, kd_a_per_rad_s=kd, max_current_a=clamp_a) as ctl:
        r = run(ImpulseParams(magnitude_ns=impulse, sim_seconds=seconds),
                model=model, controller=ctl, capture_state=True)
        sat = ctl.saturated_cycles
    tau = np.abs(r.motor_current_a) * KT_NM_PER_A
    m = r.metrics
    return r, {
        "kp_a_per_rad": kp,
        "kd_a_per_rad_s": kd,
        "clamp_a": clamp_a,
        "nose_strike": bool(m.nose_strike),
        "peak_abs_pitch_deg": round(m.peak_abs_pitch_deg, 3),
        "settle_time_s": m.settle_time_s,
        "travel_m": round(m.travel_m, 3),
        "final_wheel_rate_rad_s": round(float(r.wheel_rate_rads[-1]), 3),
        "peak_current_a": round(float(np.abs(r.motor_current_a).max()), 2),
        "peak_torque_nm": round(float(tau.max()), 2),
        "rms_torque_nm": round(float(np.sqrt((tau**2).mean())), 2),
        "saturated_cycles": sat,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--id", required=True, help="slug; names the artifacts")
    ap.add_argument("--title", default="", help="one-line human title")
    ap.add_argument("--learning", default="", help="what this run taught us")
    ap.add_argument("--ballast-mass", type=float, default=0.0)
    ap.add_argument("--ballast-height", type=float, default=0.75, help="m above axle")
    ap.add_argument("--kp", type=float, default=80.0)
    ap.add_argument("--kd", type=float, default=11.0)
    ap.add_argument("--vs-kp", type=float, help="second config, for an A/B clip")
    ap.add_argument("--vs-kd", type=float)
    ap.add_argument("--clamp-a", type=float, default=40.0)
    ap.add_argument("--impulse", type=float, default=NOMINAL_IMPULSE_NS)
    ap.add_argument("--seconds", type=float, default=6.0)
    ap.add_argument("--camera", default="beauty", choices=("beauty", "side"))
    ap.add_argument("--render", action="store_true", help="film it")
    ap.add_argument("--out-dir", type=Path, default=OUT_DIR)
    args = ap.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    model = build_model(args.ballast_mass, args.ballast_height, args.clamp_a)
    plant = plant_summary(model)

    ra, a = one_run(model, args.kp, args.kd, args.clamp_a, args.impulse, args.seconds)
    runs = {"a": a}
    rb = None
    if args.vs_kp is not None:
        rb, b = one_run(model, args.vs_kp, args.vs_kd if args.vs_kd is not None else args.kd,
                        args.clamp_a, args.impulse, args.seconds)
        runs["b"] = b

    manifest = {
        "schema": 1,
        "id": args.id,
        "title": args.title or args.id,
        "git_sha": git_sha(),
        "plant": plant,
        "stimulus": {"impulse_ns": args.impulse, "seconds": args.seconds},
        "kt_nm_per_a_UNFITTED": KT_NM_PER_A,
        "runs": runs,
        "learning": args.learning,
    }

    print(f"=== {manifest['title']}  [{args.id} @ {manifest['git_sha']}]")
    print(f"plant: {plant['total_mass_kg']} kg, CoM {plant['com_above_axle_mm']:+} mm vs axle, "
          f"mgl {plant['mgl_n_m_per_rad']:+} "
          f"({'INVERTED pendulum' if plant['inverted_pendulum'] else 'stable pendulum'})")
    for k, v in runs.items():
        print(f"  [{k}] Kp={v['kp_a_per_rad']:<7g} strike={str(v['nose_strike']):<5} "
              f"peak|pitch|={v['peak_abs_pitch_deg']:7.2f}deg  peak_tau={v['peak_torque_nm']:6.2f}Nm  "
              f"travel={v['travel_m']:+7.2f}m")

    mpath = args.out_dir / f"{args.id}.json"
    mpath.write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"wrote {mpath}")

    if args.render:
        from render_scenario import render_comparison, render_frames, write_video

        def tag(cfg):
            return f"Kp={cfg['kp_a_per_rad']:g} Kd={cfg['kd_a_per_rad_s']:g}"

        try:
            if rb is not None:
                frames = render_comparison(ra, rb, args.camera,
                                           top_label=tag(a), bottom_label=tag(b), model=model)
            else:
                frames = render_frames(ra, args.camera)
        except Exception as exc:
            print(f"render unavailable ({type(exc).__name__}: {exc}); manifest still written")
            return 0
        vpath = args.out_dir / f"{args.id}.mp4"
        write_video(frames, vpath, "libx264", 6)
        print(f"wrote {vpath} ({len(frames)} frames)")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
