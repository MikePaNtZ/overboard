#!/usr/bin/env python3
"""Film the impulse disturbance-response scenario.

Produces the artifact set that CI publishes and the landing page eventually
shows. Replaces the old scripts/render_fall.py, which filmed a model that
toppled on its own.

    .venv/bin/python scripts/render_scenario.py                # nominal, all formats
    .venv/bin/python scripts/render_scenario.py --impulse 6    # sub-threshold
    .venv/bin/python scripts/render_scenario.py --no-video     # plot + metrics only

Outputs land in sim/out/:
    impulse_open_loop.mp4    H.264 -- what the landing page embeds
    impulse_open_loop.webm   VP9 fallback
    impulse_open_loop.gif    short loop for the README
    impulse_poster.jpg       video poster frame
    impulse_pitch.png        pitch-vs-time plot (needs no GL)
    impulse_metrics.json     the numbers, for the CI gate and the page caption

HEADLESS RENDERING: set MUJOCO_GL=osmesa (software, what CI uses) or =egl
(GPU). The physics never touches GL -- the scenario is run first and this
script replays the captured trajectory -- so if rendering is unavailable the
plot and metrics are still produced and the exit status is unchanged.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from sim.scenarios.impulse_response import (  # noqa: E402
    NOMINAL_IMPULSE_NS,
    ImpulseParams,
    ImpulseResult,
    load_model,
    nose_strike_angle_deg,
    run,
)

OUT_DIR = REPO / "sim" / "out"
STEM = "impulse_open_loop"

WIDTH, HEIGHT, FPS = 1280, 720, 30

# Brand palette, from overboard-web index.html :root tokens.
INK = (22, 35, 46)
AMBER = (242, 162, 74)
MINT = (42, 174, 151)
CLOUD = (244, 248, 247)
MUTED = (150, 168, 176)


# ---------------------------------------------------------------------------
# HUD
# ---------------------------------------------------------------------------

def _font(size: int):
    """DejaVu Sans, sourced from matplotlib so the HUD looks identical on a
    developer's Mac and on a bare CI container. PIL's built-in bitmap font is
    unreadable at 720p and there is no system font we can rely on in CI."""
    from matplotlib import font_manager
    from PIL import ImageFont

    try:
        path = font_manager.findfont("DejaVu Sans", fallback_to_default=True)
        return ImageFont.truetype(path, size)
    except Exception:
        return ImageFont.load_default()


def _draw_hud(frame: np.ndarray, result: ImpulseResult, idx: int, strike_deg: float,
              events: bool = True) -> np.ndarray:
    """Overlay the engineering readout. This is the difference between a clip
    of a toy falling over and a clip that shows what was measured."""
    from PIL import Image, ImageDraw

    img = Image.fromarray(frame).convert("RGB")
    d = ImageDraw.Draw(img, "RGBA")
    f_title, f_body, f_small, f_big = _font(23), _font(17), _font(14), _font(40)

    t = float(result.t[idx])
    pitch = float(result.pitch_deg[idx])
    p = result.params
    m = result.metrics

    # --- title block -------------------------------------------------------
    d.rectangle([0, 0, WIDTH, 64], fill=(*INK, 214))
    d.text((26, 13), "OVERBOARD", font=f_title, fill=AMBER)
    # Lay the subtitle out from the measured title width -- hardcoding an x
    # offset collides the moment the font or size changes.
    sub_x = 26 + d.textlength("OVERBOARD", font=f_title) + 22
    d.line([(sub_x - 11, 16), (sub_x - 11, 48)], fill=(*MUTED, 120), width=1)
    d.text((sub_x, 19), "impulse disturbance response", font=f_body, fill=CLOUD)
    label = "OPEN LOOP · no controller" if m.control_effort_a_s == 0 else "CLOSED LOOP"
    tw = d.textlength(label, font=f_small)
    d.text((WIDTH - tw - 26, 23), label, font=f_small, fill=MUTED)

    # --- live readout ------------------------------------------------------
    d.rectangle([26, 88, 278, 214], fill=(*INK, 190))
    d.text((44, 102), "t", font=f_small, fill=MUTED)
    d.text((44, 118), f"{t:5.2f} s", font=f_body, fill=CLOUD)
    d.text((160, 102), "pitch", font=f_small, fill=MUTED)
    over = pitch >= strike_deg - 0.05
    d.text((160, 118), f"{pitch:+6.1f}°", font=f_body, fill=AMBER if over else CLOUD)
    d.text((44, 152), "nose strike at", font=f_small, fill=MUTED)
    d.text((44, 168), f"{strike_deg:.1f}°", font=f_body, fill=AMBER)
    d.text((160, 152), "travel", font=f_small, fill=MUTED)
    d.text((160, 168), f"{float(result.travel_m[idx]):5.2f} m", font=f_body, fill=CLOUD)

    # --- pitch trace strip -------------------------------------------------
    x0, y0, x1, y1 = 26, HEIGHT - 150, WIDTH - 26, HEIGHT - 46
    d.rectangle([x0, y0, x1, y1], fill=(*INK, 190))
    # Pitch is nose-up-positive (ICD 10.1), so a nose strike is a NEGATIVE
    # excursion and the interesting half of the axis is below zero.
    lo = min(-strike_deg - 6.0, float(result.pitch_deg.min()) - 3.0)
    hi = 8.0

    def px(i):
        return x0 + 10 + (x1 - x0 - 20) * (result.t[i] / result.t[-1])

    def py(v):
        return y1 - 14 - (y1 - y0 - 28) * ((v - lo) / (hi - lo))

    d.line([(x0 + 10, py(0)), (x1 - 10, py(0))], fill=(*MUTED, 110), width=1)
    d.line([(x0 + 10, py(-strike_deg)), (x1 - 10, py(-strike_deg))], fill=(*AMBER, 150), width=2)
    # Right-aligned: the impulse marker lives at the left of the strip and the
    # two labels sat on top of each other.
    sl = f"nose strike  {strike_deg:.1f}°"
    d.text((x1 - 14 - d.textlength(sl, font=f_small), py(-strike_deg) - 18), sl,
           font=f_small, fill=AMBER)

    if idx > 1:
        step = max(1, idx // 600)
        pts = [(px(i), py(float(result.pitch_deg[i]))) for i in range(0, idx + 1, step)]
        if len(pts) > 1:
            d.line(pts, fill=MINT, width=3)
        d.ellipse([pts[-1][0] - 4, pts[-1][1] - 4, pts[-1][0] + 4, pts[-1][1] + 4], fill=CLOUD)

    # impulse marker
    xi = px(int(p.t0_s / (result.t[-1] / len(result.t))))
    d.line([(xi, y0 + 8), (xi, y1 - 8)], fill=(*AMBER, 170), width=2)
    d.text((xi + 6, y0 + 10), f"{p.magnitude_ns:.0f} N·s", font=f_small, fill=AMBER)

    # --- event callouts ----------------------------------------------------
    # Placed in the empty band between the title bar and the board, not over
    # it. Suppressed entirely for the poster frame, which needs a clean shot.
    CALLOUT_Y = 108
    if events and p.t0_s <= t < p.t0_s + 0.9:
        fade = int(255 * max(0.0, 1.0 - (t - p.t0_s) / 0.9))
        txt = f"IMPULSE  {p.magnitude_ns:.0f} N·s"
        tw = d.textlength(txt, font=f_big)
        d.text(((WIDTH - tw) / 2, CALLOUT_Y), txt, font=f_big, fill=(*AMBER, fade))

    if events and m.t_strike_s is not None and m.t_strike_s <= t < m.t_strike_s + 1.6:
        fade = int(255 * max(0.0, 1.0 - (t - m.t_strike_s) / 1.6))
        txt = "NOSE STRIKE"
        tw = d.textlength(txt, font=f_big)
        d.text(((WIDTH - tw) / 2, CALLOUT_Y), txt, font=f_big, fill=(*AMBER, fade))
        sub = f"{m.speed_at_strike_ms:.2f} m/s   {m.pitch_rate_at_strike_dps:.0f}°/s"
        sw = d.textlength(sub, font=f_body)
        d.text(((WIDTH - sw) / 2, CALLOUT_Y + 50), sub, font=f_body, fill=(*CLOUD, fade))

    return np.asarray(img)


# ---------------------------------------------------------------------------
# Render
# ---------------------------------------------------------------------------

def render_frames(result: ImpulseResult, camera: str) -> list[np.ndarray]:
    """Replay the captured trajectory through the offscreen renderer."""
    import mujoco

    model = load_model()
    data = mujoco.MjData(model)
    strike = nose_strike_angle_deg(model)

    dt = float(model.opt.timestep)
    stride = max(1, int(round((1.0 / FPS) / dt)))
    renderer = mujoco.Renderer(model, height=HEIGHT, width=WIDTH)
    try:
        scene_opt = mujoco.MjvOption()
        scene_opt.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT] = False
        frames = []
        for i in range(0, len(result.t), stride):
            data.qpos[:] = result.qpos[i]
            mujoco.mj_forward(model, data)
            renderer.update_scene(data, camera=camera, scene_option=scene_opt)
            frames.append(_draw_hud(renderer.render().copy(), result, i, strike))
        return frames
    finally:
        renderer.close()


def render_poster(result: ImpulseResult, camera: str, idx: int) -> np.ndarray:
    """One clean frame for the video poster — no transient callouts.

    A second single-frame pass rather than keeping un-overlaid copies of every
    frame around: at 720p that would be another ~650 MB of RAM to save one
    render.
    """
    import mujoco

    model = load_model()
    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, height=HEIGHT, width=WIDTH)
    try:
        data.qpos[:] = result.qpos[idx]
        mujoco.mj_forward(model, data)
        renderer.update_scene(data, camera=camera)
        return _draw_hud(renderer.render().copy(), result, idx,
                         nose_strike_angle_deg(model), events=False)
    finally:
        renderer.close()


def write_video(frames: list[np.ndarray], path: Path, codec: str, quality: int) -> None:
    import imageio.v2 as imageio

    writer = imageio.get_writer(
        path, fps=FPS, codec=codec, quality=quality,
        macro_block_size=1, ffmpeg_log_level="error",
    )
    try:
        for f in frames:
            writer.append_data(f)
    finally:
        writer.close()


def write_gif(frames: list[np.ndarray], path: Path, result: ImpulseResult) -> None:
    """Short loop for the README — the interesting seconds only.

    The mp4 is the artifact that matters; this exists so GitHub renders
    something inline. Filming all 8 s at half rate produced a 4.9 MB GIF, which
    is a bad thing to put at the top of a README, so this cuts to the window
    around the kick and the strike and drops to 1/3 rate.
    """
    from PIL import Image

    m, p = result.metrics, result.params
    end_s = (m.t_strike_s + 1.5) if m.t_strike_s is not None else float(result.t[-1])
    lo = max(0, int((p.t0_s - 0.3) * FPS))
    hi = min(len(frames), int(end_s * FPS))
    clip = frames[lo:hi:3] or frames[::3]

    imgs = [
        Image.fromarray(f)
        .resize((WIDTH // 2, HEIGHT // 2), Image.LANCZOS)
        .quantize(colors=64, method=Image.MEDIANCUT, dither=Image.NONE)
        for f in clip
    ]
    imgs[0].save(path, save_all=True, append_images=imgs[1:],
                 duration=int(3000 / FPS), loop=0, optimize=True)


def save_pitch_plot(result: ImpulseResult, strike_deg: float, path: Path) -> None:
    """The GL-free proof. Always produced, even when rendering is unavailable."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    m, p = result.metrics, result.params
    fig, (ax, ax2) = plt.subplots(2, 1, figsize=(9, 6), sharex=True,
                                  gridspec_kw={"height_ratios": [3, 1]})
    ax.plot(result.t, result.pitch_deg, lw=2.2, color="#2AAE97", label="frame pitch")
    ax.axhline(-strike_deg, ls="--", lw=1.8, color="#C4650F",
               label=f"nose strike ({strike_deg:.1f}°, from geometry)")
    ax.axvspan(p.t0_s, p.t0_s + p.duration_s, color="#F2A24A", alpha=0.35,
               label=f"impulse {p.magnitude_ns:.0f} N·s")
    if m.t_strike_s is not None:
        ax.plot([m.t_strike_s], [result.pitch_deg[np.searchsorted(result.t, m.t_strike_s)]],
                "o", ms=9, color="#C4650F", zorder=5)
        ax.annotate(f"strike @ {m.t_strike_s:.2f}s\n{m.speed_at_strike_ms:.2f} m/s",
                    (m.t_strike_s, -strike_deg), textcoords="offset points",
                    xytext=(16, -46), color="#16232E", fontsize=9,
                    arrowprops=dict(arrowstyle="-", lw=0.8, color="#8a99a0"))
    ax.set_ylabel("pitch, nose-down positive (deg)")
    ax.set_title("Overboard — impulse disturbance response, open loop (no controller)")
    # Lower right: the action is all in the upper left (the kick, the first
    # swing to the strike line) and an upper-left legend buried it.
    ax.legend(loc="lower right", fontsize=9, framealpha=0.92)
    ax.margins(y=0.12)
    ax.grid(alpha=0.25)

    ax2.plot(result.t, result.travel_m, lw=1.8, color="#16232E")
    ax2.set_ylabel("travel (m)")
    ax2.set_xlabel("time (s)")
    ax2.grid(alpha=0.25)

    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--impulse", type=float, default=NOMINAL_IMPULSE_NS, help="N*s")
    ap.add_argument("--seconds", type=float, default=ImpulseParams().sim_seconds)
    ap.add_argument("--camera", default="beauty", choices=("beauty", "side"))
    ap.add_argument("--no-video", action="store_true", help="skip GL; plot + metrics only")
    ap.add_argument("--no-gif", action="store_true")
    ap.add_argument("--out-dir", type=Path, default=OUT_DIR)
    args = ap.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    params = ImpulseParams(magnitude_ns=args.impulse, sim_seconds=args.seconds)

    model = load_model()
    strike = nose_strike_angle_deg(model)
    result = run(params, model=model, capture_state=True)
    m = result.metrics

    print(f"impulse            {params.magnitude_ns:.1f} N*s at t={params.t0_s}s")
    print(f"nose strike angle  {strike:.2f} deg (from collision hull)")
    print(f"peak |pitch|       {m.peak_abs_pitch_deg:.2f} deg at t={m.t_peak_s:.2f}s")
    print(f"nose strike        {m.nose_strike}"
          + (f" at t={m.t_strike_s:.3f}s, {m.speed_at_strike_ms:.2f} m/s, "
             f"{m.pitch_rate_at_strike_dps:.0f} deg/s" if m.nose_strike else ""))
    print(f"travel             {m.travel_m:.2f} m")

    save_pitch_plot(result, strike, args.out_dir / "impulse_pitch.png")
    print(f"wrote {args.out_dir / 'impulse_pitch.png'}")

    (args.out_dir / "impulse_metrics.json").write_text(
        json.dumps(result.to_json_dict(), indent=2) + "\n"
    )
    print(f"wrote {args.out_dir / 'impulse_metrics.json'}")

    if not args.no_video:
        try:
            frames = render_frames(result, args.camera)
        except Exception as exc:
            # Rendering is a publishing concern, not a correctness one. The
            # physics gate lives in tests/ and has already run without GL.
            print(f"\noffscreen rendering unavailable ({type(exc).__name__}: {exc})")
            print("plot + metrics were still written; set MUJOCO_GL=osmesa or =egl for video.")
            return 0

        from PIL import Image

        write_video(frames, args.out_dir / f"{STEM}.mp4", "libx264", 6)
        print(f"wrote {args.out_dir / f'{STEM}.mp4'} ({len(frames)} frames @ {FPS}fps)")
        write_video(frames, args.out_dir / f"{STEM}.webm", "libvpx-vp9", 7)
        print(f"wrote {args.out_dir / f'{STEM}.webm'}")

        # Poster: the strike itself — peak pitch, most expressive pose.
        poster_at = m.t_strike_s if m.t_strike_s is not None else m.t_peak_s
        pi = min(int(round(poster_at / float(model.opt.timestep))), len(result.qpos) - 1)
        Image.fromarray(render_poster(result, args.camera, pi)).save(
            args.out_dir / "impulse_poster.jpg", quality=92
        )
        print(f"wrote {args.out_dir / 'impulse_poster.jpg'}")

        if not args.no_gif:
            write_gif(frames, args.out_dir / f"{STEM}.gif", result)
            print(f"wrote {args.out_dir / f'{STEM}.gif'}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
