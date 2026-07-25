#!/usr/bin/env python3
"""Headless proof that the Overboard onewheel checkpoint model falls over.

Steps the model (no controller -- see sim/models/overboard_onewheel.xml) for
a few seconds and renders it. Two outputs are attempted:

  1. sim/out/fall.gif  -- an animated GIF of the fall, rendered offscreen
     via mujoco.Renderer (best-effort: needs a working GL context; on
     macOS this generally works out of the box via MuJoCo's CGL backend).
  2. sim/out/fall_pitch.png -- a plot of mast tilt-from-vertical vs. time,
     which always works (no GL needed) and is the guaranteed fallback proof
     if offscreen rendering is unavailable in a given environment.

Either way, the final tilt angle is printed to stdout as numeric proof the
rig toppled.

Run from the project venv:
    source .venv/bin/activate
    python scripts/render_fall.py
"""

from pathlib import Path

import mujoco
import numpy as np

MODEL_PATH = Path(__file__).resolve().parent.parent / "sim" / "models" / "overboard_onewheel.xml"
OUT_DIR = Path(__file__).resolve().parent.parent / "sim" / "out"
SIM_SECONDS = 5.0
FALL_THRESHOLD_DEG = 45.0


def mast_tilt_deg(model, data, mast_body_id) -> float:
    """Tilt of the mast body's local z-axis from world-vertical, in degrees."""
    xmat = data.xmat[mast_body_id].reshape(3, 3)
    z_axis = xmat[:, 2]
    tilt_rad = np.arccos(np.clip(np.dot(z_axis, [0.0, 0.0, 1.0]), -1.0, 1.0))
    return float(np.degrees(tilt_rad))


def render_gif(model, data, n_steps, mast_body_id) -> tuple[list, list]:
    """Step + render offscreen. Returns (frames, tilts_deg). Raises on GL failure."""
    renderer = mujoco.Renderer(model, height=480, width=640)
    frames = []
    tilts = []
    render_every = max(1, int((1.0 / 30.0) / model.opt.timestep))  # ~30 fps

    for i in range(n_steps):
        mujoco.mj_step(model, data)
        tilts.append(mast_tilt_deg(model, data, mast_body_id))
        if i % render_every == 0:
            renderer.update_scene(data)
            frames.append(renderer.render().copy())

    renderer.close()
    return frames, tilts


def step_only(model, data, n_steps, mast_body_id) -> list:
    """Step without rendering (used for the plot-only fallback path)."""
    tilts = []
    for _ in range(n_steps):
        mujoco.mj_step(model, data)
        tilts.append(mast_tilt_deg(model, data, mast_body_id))
    return tilts


def save_gif(frames, path: Path) -> None:
    from PIL import Image

    images = [Image.fromarray(f) for f in frames]
    images[0].save(
        path,
        save_all=True,
        append_images=images[1:],
        duration=1000 // 30,
        loop=0,
    )


def save_pitch_plot(tilts, dt, path: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    times = np.arange(len(tilts)) * dt
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(times, tilts, linewidth=2)
    ax.axhline(FALL_THRESHOLD_DEG, color="red", linestyle="--", label=f"{FALL_THRESHOLD_DEG:.0f} deg fall threshold")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("mast tilt from vertical (deg)")
    ax.set_title("Overboard onewheel checkpoint: uncontrolled fall")
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    mast_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "mast")

    n_steps = int(SIM_SECONDS / model.opt.timestep)

    gif_path = OUT_DIR / "fall.gif"
    plot_path = OUT_DIR / "fall_pitch.png"

    tilts = None
    try:
        frames, tilts = render_gif(model, data, n_steps, mast_body_id)
        save_gif(frames, gif_path)
        print(f"Wrote {len(frames)} frames to {gif_path}")
    except Exception as exc:  # best-effort: headless GL not guaranteed everywhere
        print(f"Offscreen rendering unavailable ({type(exc).__name__}: {exc}).")
        print("Falling back to pitch-angle-vs-time plot (no GL required).")
        # data/model may have partially stepped above; reset and redo cleanly.
        mujoco.mj_resetData(model, data)
        tilts = step_only(model, data, n_steps, mast_body_id)

    if not gif_path.exists():
        save_pitch_plot(tilts, model.opt.timestep, plot_path)
        print(f"Wrote pitch-vs-time plot to {plot_path}")
    else:
        # Also always save the plot alongside the gif -- cheap and useful.
        save_pitch_plot(tilts, model.opt.timestep, plot_path)
        print(f"Also wrote pitch-vs-time plot to {plot_path}")

    final_tilt = tilts[-1]
    print(f"Final mast tilt from vertical after {SIM_SECONDS:.1f}s: {final_tilt:.2f} deg")
    if final_tilt > FALL_THRESHOLD_DEG:
        print(f"PASS: onewheel toppled (> {FALL_THRESHOLD_DEG:.0f} deg).")
    else:
        print(f"WARNING: did not exceed {FALL_THRESHOLD_DEG:.0f} deg fall threshold.")


if __name__ == "__main__":
    main()
