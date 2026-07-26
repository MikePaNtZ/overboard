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
from sim.scenarios.plant import build_model, plant_summary, rider_geoms  # noqa: E402,F401
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


def one_run(model, kp, kd, clamp_a, impulse, seconds,
            kp_v=0.0, ki_v=0.0, v_ref=0.0, com_above_axle=None):
    """One configuration against one plant.

    `com_above_axle` defaults to being READ OFF THE PLANT rather than passed
    in. The outer loop's sign is a property of where the centre of mass sits,
    not a tuning choice, and deriving it here means an experiment cannot
    accidentally run the ridden coupling on the driverless board.
    """
    if com_above_axle is None:
        com_above_axle = plant_summary(model)["inverted_pendulum"]
    with RustController(kp_a_per_rad=kp, kd_a_per_rad_s=kd, max_current_a=clamp_a,
                        kp_v_rad_per_m_s=kp_v, ki_v_rad_per_m=ki_v, v_ref_m_s=v_ref,
                        com_above_axle=com_above_axle) as ctl:
        r = run(ImpulseParams(magnitude_ns=impulse, sim_seconds=seconds),
                model=model, controller=ctl, capture_state=True)
        sat = ctl.saturated_cycles
        peak_ref = ctl.peak_abs_pitch_ref_rad
    tau = np.abs(r.motor_current_a) * KT_NM_PER_A
    m = r.metrics
    return r, {
        "kp_a_per_rad": kp,
        "kd_a_per_rad_s": kd,
        "kp_v_rad_per_m_s": kp_v,
        "ki_v_rad_per_m": ki_v,
        "v_ref_m_s": v_ref,
        "com_above_axle": bool(com_above_axle),
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
        "peak_abs_pitch_ref_deg": round(float(np.degrees(peak_ref)), 3),
    }


def run_shuttle(args) -> int:
    """The Shuttle Run: a commanded route rather than an A/B of two gain sets.

    Single pane -- there is no second configuration to compare against, and the
    interest is in whether the board follows the command, which a viewer reads
    from the motion itself.
    """
    from render_scenario import FPS, HEIGHT, WIDTH, write_video

    from sim.scenarios.shuttle_run import ShuttleParams, run as run_shuttle_scenario

    # The shuttle is ridden-plant-only, so a zero --ballast-mass means "use the
    # scenario's own default" rather than "run it driverless", which the
    # scenario would refuse anyway.
    defaults = ShuttleParams()
    params = ShuttleParams(
        ballast_mass_kg=args.ballast_mass or defaults.ballast_mass_kg,
        ballast_height_m=args.ballast_height,
        max_current_a=args.clamp_a,
    )
    r = run_shuttle_scenario(params, capture_state=args.render)
    m = r.metrics

    print(f"=== Shuttle Run  [{args.id} @ {git_sha()}]")
    print(f"plant: {r.plant['total_mass_kg']} kg, CoM {r.plant['com_above_axle_mm']:+} mm vs axle")
    print(f"  duration        {m.duration_s:6.1f} s")
    print(f"  RETURN ERROR    {m.return_error_m * 100:6.1f} cm")
    print(f"  reached         {m.min_forward_m:+.2f} .. {m.max_forward_m:+.2f} m")
    print(f"  max lag         {m.max_tracking_error_m * 100:6.1f} cm")
    print(f"  creep in hold   {m.max_hold_drift_m * 100:6.1f} cm")
    print(f"  peak pitch      {m.peak_abs_pitch_deg:6.2f} deg (ref peak {m.peak_abs_pitch_ref_deg:.2f})")
    print(f"  peak current    {m.peak_current_a:6.2f} A, rms torque {m.rms_torque_nm:.2f} Nm")

    manifest = {
        "schema": 1,
        "id": args.id,
        "title": args.title or "Shuttle Run",
        "git_sha": git_sha(),
        "scenario": "shuttle_run",
        "plant": r.plant,
        "kt_nm_per_a_UNFITTED": KT_NM_PER_A,
        **r.to_json_dict(),
        "learning": args.learning,
    }
    mpath = args.out_dir / f"{args.id}.json"
    mpath.write_text(json.dumps(manifest, indent=2, default=str) + "\n")
    print(f"wrote {mpath}")

    if args.render:
        import mujoco
        from PIL import Image, ImageDraw
        from render_scenario import AMBER, CLOUD, INK, MINT, MUTED, _font

        # Rebuild from the SCENARIO's params, not the CLI args. They differ:
        # a zero --ballast-mass is resolved to the scenario default above, and
        # rendering from the raw args produced a clip of a bare board while the
        # physics ran with a 70 kg rider aboard.
        model = build_model(params.ballast_mass_kg, params.ballast_height_m,
                            params.max_current_a, args.rider_style)
        data = mujoco.MjData(model)
        dt = float(model.opt.timestep)
        stride = max(1, int(round((1.0 / FPS) / dt)))
        renderer = mujoco.Renderer(model, height=HEIGHT, width=WIDTH)
        frames = []
        try:
            opt = mujoco.MjvOption()
            for i in range(0, len(r.t), stride):
                data.qpos[:] = r.qpos[i]
                mujoco.mj_forward(model, data)
                # `wide` unless overridden: this scenario is about travel, and
                # a tracking camera hides exactly that.
                cam = "wide" if args.camera == "beauty" else args.camera
                renderer.update_scene(data, camera=cam, scene_option=opt)
                img = Image.fromarray(renderer.render().copy())
                d = ImageDraw.Draw(img, "RGBA")
                f_t, f_b, f_s = _font(26), _font(19), _font(15)

                d.rectangle([0, 0, WIDTH, 52], fill=(*INK, 210))
                d.text((22, 12), "SHUTTLE RUN", font=f_t, fill=AMBER)
                d.text((215, 17), "commanded velocity, reversal, return to home",
                       font=f_s, fill=MUTED)

                # Position against command is the whole story, so it is the
                # readout -- pitch is a supporting number here, not the point.
                d.rectangle([22, 68, 330, 176], fill=(*INK, 190))
                d.text((38, 80), "position", font=f_s, fill=MUTED)
                d.text((38, 98), f"{float(r.pos_m[i]):+6.2f} m", font=f_b, fill=CLOUD)
                d.text((186, 80), "commanded", font=f_s, fill=MUTED)
                d.text((186, 98), f"{float(r.pos_cmd_m[i]):+6.2f} m", font=f_b, fill=MINT)
                d.text((38, 128), "speed", font=f_s, fill=MUTED)
                d.text((38, 146), f"{float(r.v_m_s[i]):+5.2f} m/s", font=f_b, fill=CLOUD)
                d.text((186, 128), "pitch", font=f_s, fill=MUTED)
                d.text((186, 146), f"{float(r.pitch_deg[i]):+5.1f}°", font=f_b, fill=CLOUD)

                # Home marker: the number the run is judged on, on its own
                # panel so it stays readable over a pale floor.
                off = abs(float(r.pos_m[i]))
                d.rectangle([WIDTH - 288, 68, WIDTH - 22, 132], fill=(*INK, 190))
                d.text((WIDTH - 270, 80), "distance from home", font=f_s, fill=MUTED)
                d.text((WIDTH - 270, 100), f"{off * 100:.0f} cm", font=f_b,
                       fill=MINT if off < 0.3 else AMBER)
                frames.append(np.asarray(img))
        finally:
            renderer.close()

        vpath = args.out_dir / f"{args.id}.mp4"
        write_video(frames, vpath, "libx264", 6)
        print(f"wrote {vpath} ({len(frames)} frames)")
    return 0


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
    ap.add_argument("--kp-v", type=float, default=0.0,
                    help="outer velocity loop; 0 disables it")
    ap.add_argument("--ki-v", type=float, default=0.0)
    ap.add_argument("--vs-kp-v", type=float)
    ap.add_argument("--vs-ki-v", type=float)
    ap.add_argument("--v-ref", type=float, default=0.0, help="speed setpoint, m/s")
    ap.add_argument("--clamp-a", type=float, default=40.0)
    ap.add_argument("--impulse", type=float, default=NOMINAL_IMPULSE_NS)
    ap.add_argument("--seconds", type=float, default=6.0)
    ap.add_argument("--camera", default="beauty", choices=("beauty", "side", "wide"))
    ap.add_argument("--rider-style", default="figure", choices=("figure", "sphere"),
                    help="visual proxy for the ballast; changes no physics")
    ap.add_argument("--shuttle", action="store_true",
                    help="run the Shuttle Run scenario instead of an impulse A/B")
    ap.add_argument("--render", action="store_true", help="film it")
    ap.add_argument("--out-dir", type=Path, default=OUT_DIR)
    args = ap.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)

    if args.shuttle:
        return run_shuttle(args)

    model = build_model(args.ballast_mass, args.ballast_height, args.clamp_a,
                        args.rider_style)
    plant = plant_summary(model)

    ra, a = one_run(model, args.kp, args.kd, args.clamp_a, args.impulse, args.seconds,
                    args.kp_v, args.ki_v, args.v_ref)
    runs = {"a": a}
    rb = None
    if args.vs_kp is not None or args.vs_kp_v is not None:
        rb, b = one_run(
            model,
            args.vs_kp if args.vs_kp is not None else args.kp,
            args.vs_kd if args.vs_kd is not None else args.kd,
            args.clamp_a, args.impulse, args.seconds,
            args.vs_kp_v if args.vs_kp_v is not None else args.kp_v,
            args.vs_ki_v if args.vs_ki_v is not None else args.ki_v,
            args.v_ref,
        )
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
        outer = (f"Kpv={v['kp_v_rad_per_m_s']:g}/{v['ki_v_rad_per_m']:g}"
                 if (v["kp_v_rad_per_m_s"] or v["ki_v_rad_per_m"]) else "no outer loop")
        print(f"  [{k}] Kp={v['kp_a_per_rad']:<6g} {outer:<18} "
              f"strike={str(v['nose_strike']):<5} peak|pitch|={v['peak_abs_pitch_deg']:7.2f}deg  "
              f"travel={v['travel_m']:+7.2f}m  final_wheel={v['final_wheel_rate_rad_s']:+7.2f}")

    mpath = args.out_dir / f"{args.id}.json"
    mpath.write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"wrote {mpath}")

    if args.render:
        from render_scenario import render_comparison, render_frames, write_video

        def tag(cfg):
            if cfg["kp_v_rad_per_m_s"] or cfg["ki_v_rad_per_m"]:
                return f"CASCADE  Kp={cfg['kp_a_per_rad']:g}  Kpv={cfg['kp_v_rad_per_m_s']:g}"
            return f"INNER ONLY  Kp={cfg['kp_a_per_rad']:g}"

        try:
            if rb is not None:
                # Side-by-side once a rider is aboard: the subject is then
                # taller than it is wide, and full-height panes frame it.
                frames = render_comparison(
                    ra, rb, args.camera, top_label=tag(a), bottom_label=tag(b),
                    model=model, layout="h" if args.ballast_mass > 0 else "v")
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
