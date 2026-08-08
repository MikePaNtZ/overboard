#!/usr/bin/env python3
"""Film a sim scenario and produce the publishable artifact set.

Two scenarios are filmed today. Both produce **Sim Replay** output -- generated
from a recorded simulator run, so the clips may carry engineering numbers and
they carry a source tag. The source is a PARAMETER (`--source`), not a literal,
so the day a bench run drives this same pipeline `--source hardware` is the
whole change on this side. Category names are defined in exactly one place, the
shared vocabulary, and are deliberately not restated here.

    .venv/bin/python scripts/render_scenario.py                      # impulse, all formats
    .venv/bin/python scripts/render_scenario.py --impulse 6          # sub-threshold
    .venv/bin/python scripts/render_scenario.py --no-video           # plot + metrics only
    .venv/bin/python scripts/render_scenario.py --scenario terrain   # the rolling-terrain ride
    .venv/bin/python scripts/render_scenario.py --scenario terrain --compare

Outputs land in sim/out/:

  impulse (`--scenario impulse`, the default)
    impulse_open_loop.mp4    H.264 -- what the landing page embeds
    impulse_open_loop.webm   VP9 fallback
    impulse_open_loop.gif    short loop for the README
    impulse_poster.jpg       video poster frame
    impulse_pitch.png        pitch-vs-time plot (needs no GL)
    impulse_metrics.json     the numbers, for the CI gate and the page caption
    impulse_compare.mp4      (--compare) open loop against closed loop
    impulse_render_manifest.json

  rolling terrain (`--scenario terrain`)
    terrain_ride.mp4/.webm   the default ride: crest, descent, dip, ascent, crest
    terrain_poster.jpg       video poster frame
    terrain_ride.png         truth/estimate/grade plot (needs no GL)
    terrain_metrics.json     the numbers behind the ride
    terrain_compare.mp4      (--compare) truth pitch against the attitude estimate
    terrain_compare.png      (--compare) the same comparison as a plot
    terrain_truth_metrics.json, terrain_estimate_metrics.json
    terrain_render_manifest.json

Every invocation writes a **render manifest** naming the source commit, the
scenario parameters, the headline metrics and a sha256 of every file it
produced, so "this clip came from that run" is checkable by someone who was not
in the room.

HEADLESS RENDERING: set MUJOCO_GL=osmesa (software, what CI uses) or =egl
(GPU). The physics never touches GL -- the scenario is run first and this
script replays the captured trajectory -- so if rendering is unavailable the
plot and metrics are still produced and the exit status is unchanged.
"""

from __future__ import annotations

import argparse
import contextlib
import datetime as _dt
import hashlib
import json
import math
import os
import subprocess
import sys
from dataclasses import asdict, dataclass
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
ALARM = (214, 90, 66)


# ---------------------------------------------------------------------------
# Categorisation and provenance
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class Source:
    """Where the pixels came from, carried as a value rather than a literal.

    `tag` is the badge burned into the frame; `category` is the vocabulary
    category the clip is filed under. Both are parameters so that when real
    telemetry runs this same path, selecting a different source is the entire
    change on this side -- no new render path, no edited string constants, and
    no chance of a hardware clip inheriting a sim badge because a literal was
    missed.

    The categories are defined in exactly one place -- the shared vocabulary --
    and are deliberately NOT restated here:
    https://app.notion.com/p/3aa472a5fb6981ebaaa7cf2e996f1e8b
    """

    tag: str
    category: str

    @property
    def badge(self) -> str:
        """What is burned into the frame. A Replay always names its source."""
        return self.category.upper()


SOURCES: dict[str, Source] = {
    "sim": Source(tag="SIM", category="Sim Replay"),
    "hardware": Source(tag="HARDWARE", category="Hardware Replay"),
}

MANIFEST_SCHEMA = 1


def source_commit() -> dict:
    """The commit the rendered data came from.

    Preference order matters. On a `pull_request` build `github.sha` is the
    ephemeral merge commit, which exists in no branch and cannot be checked
    out; stamping it makes the artifact untraceable, which is the opposite of
    the point. CI therefore passes the head sha in `ARTIFACT_SHA` and that wins
    over anything discovered locally.
    """
    sha = os.environ.get("ARTIFACT_SHA") or os.environ.get("GITHUB_SHA")
    dirty = None
    if not sha:
        try:
            sha = subprocess.run(
                ["git", "rev-parse", "HEAD"], cwd=REPO, capture_output=True,
                text=True, check=True,
            ).stdout.strip()
            dirty = bool(subprocess.run(
                ["git", "status", "--porcelain"], cwd=REPO, capture_output=True,
                text=True, check=True,
            ).stdout.strip())
        except Exception:
            sha = None
    out: dict = {"commit": sha, "commit_short": sha[:7] if sha else None}
    # Only reported when it is known. A `false` here on a CI build -- where the
    # working tree is a fresh checkout and the question is meaningless -- would
    # read as a checked claim rather than an absent one.
    if dirty is not None:
        out["working_tree_dirty"] = dirty
    server = os.environ.get("GITHUB_SERVER_URL")
    repo = os.environ.get("GITHUB_REPOSITORY")
    if sha and server and repo:
        out["commit_url"] = f"{server}/{repo}/commit/{sha}"
    if server and repo and os.environ.get("GITHUB_RUN_ID"):
        out["run_url"] = f"{server}/{repo}/actions/runs/{os.environ['GITHUB_RUN_ID']}"
    return out


def write_manifest(
    path: Path,
    *,
    scenario: str,
    source: Source,
    runs: dict[str, dict],
    outputs: list[Path],
    renderer: dict,
) -> None:
    """Record what was filmed, from which commit, with which parameters.

    Separate from the scenario's own metrics.json on purpose: that file is the
    scenario's schema and belongs to whoever owns the scenario. This one is the
    PUBLISHING record -- provenance, categorisation, and a digest of every file
    that left the building -- and downstream consumers read it without having
    to reach into a scenario's internals.
    """
    files = []
    for p in sorted(set(outputs)):
        if not p.exists():
            continue
        files.append({
            "name": p.name,
            "bytes": p.stat().st_size,
            "sha256": hashlib.sha256(p.read_bytes()).hexdigest(),
        })

    import mujoco

    doc = {
        "schema": MANIFEST_SCHEMA,
        "generated_at_utc": _dt.datetime.now(_dt.UTC).replace(microsecond=0).isoformat(),
        "category": source.category,
        "source_tag": source.tag,
        "vocabulary": "https://app.notion.com/p/3aa472a5fb6981ebaaa7cf2e996f1e8b",
        "scenario": scenario,
        "source": source_commit(),
        "runs": runs,
        "renderer": {
            "script": "scripts/render_scenario.py",
            "mujoco": mujoco.__version__,
            "mujoco_gl": os.environ.get("MUJOCO_GL", "<default>"),
            **renderer,
        },
        "outputs": files,
    }
    path.write_text(json.dumps(doc, indent=2) + "\n")
    print(f"wrote {path}")


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


PANE_H = HEIGHT // 2


def _pane_label(frame: np.ndarray, title: str, subtitle: str, colour,
                width: int = WIDTH) -> np.ndarray:
    """Label one pane of the comparison. Deliberately not the full HUD.

    The HUD is laid out for a 720-high frame; shrinking it to fit a pane makes
    it unreadable, and the comparison's job is to show the two behaviours side
    by side, not to restate every metric twice.
    """
    from PIL import Image, ImageDraw

    img = Image.fromarray(frame)
    d = ImageDraw.Draw(img, "RGBA")
    f_title, f_sub = _font(26), _font(17)

    d.rectangle([0, 0, width, 44], fill=(*INK, 205))
    d.text((18, 9), title, font=f_title, fill=colour)
    # Subtitle drops to a second line if the pane is too narrow to hold both.
    tw = d.textlength(title, font=f_title)
    if 18 + tw + 14 + d.textlength(subtitle, font=f_sub) < width - 10:
        d.text((18 + tw + 14, 16), subtitle, font=f_sub, fill=CLOUD)
    else:
        d.rectangle([0, 44, width, 72], fill=(*INK, 175))
        d.text((18, 49), subtitle, font=f_sub, fill=CLOUD)
    return np.asarray(img)


def render_comparison(
    open_loop: ImpulseResult,
    closed: ImpulseResult,
    camera: str,
    top_label: str = "OPEN LOOP",
    bottom_label: str = "CLOSED LOOP",
    model=None,
    layout: str = "v",
) -> list[np.ndarray]:
    """Two panes, same disturbance: uncontrolled above, controlled below.

    Labels and model are parameters so the same renderer serves any A/B
    experiment -- failing gains against working gains, driverless against
    ballasted -- rather than only the open/closed case it was written for.

    Both panes are rendered natively at half height rather than rendered full
    size and downscaled — downscaling a 720p frame to 360 turns the board into
    mush exactly where the pitch angle is the thing being read.

    The two runs are separate trajectories of differing length; each pane
    replays its own and holds its final frame if it is the shorter, so the
    comparison stays time-aligned rather than one pane running ahead.
    """
    import mujoco

    model = model if model is not None else load_model()
    dt = float(model.opt.timestep)
    stride = max(1, int(round((1.0 / FPS) / dt)))
    n = max(len(open_loop.t), len(closed.t))

    # A tall subject (a board with a rider on it) does not fit a 360-high
    # pane: fovy is the VERTICAL field of view, so a short pane can only frame
    # a tall thing by zooming out until it is too small to read. Side-by-side
    # panes are full height, which is the right layout the moment the subject
    # is taller than it is wide.
    pane_w, pane_h = ((WIDTH, PANE_H) if layout == "v" else (WIDTH // 2, HEIGHT))
    renderer = mujoco.Renderer(model, height=pane_h, width=pane_w)
    data = mujoco.MjData(model)
    try:
        scene_opt = mujoco.MjvOption()
        scene_opt.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT] = False

        def pane(result: ImpulseResult, i: int, title: str, colour) -> np.ndarray:
            j = min(i, len(result.qpos) - 1)
            data.qpos[:] = result.qpos[j]
            mujoco.mj_forward(model, data)
            renderer.update_scene(data, camera=camera, scene_option=scene_opt)
            sub = (
                f"pitch {float(result.pitch_deg[j]):+6.2f}°   "
                f"travel {float(result.travel_m[j]):+5.2f} m"
            )
            return _pane_label(renderer.render().copy(), title, sub, colour, width=pane_w)

        frames = []
        for i in range(0, n, stride):
            top = pane(open_loop, i, top_label, AMBER)
            bottom = pane(closed, i, bottom_label, MINT)
            frames.append(np.vstack([top, bottom]) if layout == "v"
                          else np.hstack([top, bottom]))
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


# ===========================================================================
# ROLLING TERRAIN
#
# The one scenario in this repo that can actually be filmed. `hill.py` models a
# slope by rotating gravity on flat ground, which is exact for an infinite
# uniform plane and invisible in a render -- the board appears on level ground,
# accelerating and tipping for no reason a viewer can see. `terrain.py` tilts
# the real ground, so there is a hill in frame. It was built to be filmed and
# then nothing filmed it; this section is that.
# ===========================================================================

TERRAIN_STEM = "terrain_ride"

#: Two panes of 306 plus a 108-high shared footer fills the same 720 as
#: everything else, and leaves the footer room for the ground profile -- which
#: is what carries the grade when the panes are too short to hold both crests.
TERRAIN_PANE_H = 306
TERRAIN_FOOTER_H = HEIGHT - 2 * TERRAIN_PANE_H


@contextlib.contextmanager
def _pose_recorder():
    """Record `qpos` from a scenario run that does not offer to record it.

    `impulse_response.run(capture_state=True)` appends a pose history.
    `terrain.run()` has no such switch: `TerrainResult` carries every channel a
    plot needs and no pose history at all, so the ride cannot be replayed
    through the renderer as it stands. `sim/scenarios/terrain.py` belongs to
    Senior Controls under CODEOWNERS -- this role consumes it and does not edit
    it -- so the fix on their side is one keyword argument, and it is requested
    rather than taken.

    Re-running the ride inside this script to reconstruct poses was the other
    option and it is the worse one: the controller, the STAGE0_CUTBACK
    imperfection profile and the heightfield build all live inside that loop,
    and a second copy of them here is exactly how a clip ends up being of a
    different run than the metrics printed beside it.

    So the real run is RECORDED rather than reproduced -- `mujoco.mj_step` is
    wrapped for the duration of the call and the post-step pose kept. The
    caller then asserts one pose per recorded sample, so a scenario that
    changes how it integrates fails loudly here instead of quietly filming a
    trajectory that is not the one it measured.

    Delete this the moment `terrain.run` accepts `capture_state=` -- the
    signature check in `run_terrain` already prefers the real thing.
    """
    import mujoco

    poses: list[np.ndarray] = []
    real_step = mujoco.mj_step

    def recording_step(model, data, *args, **kwargs):
        real_step(model, data, *args, **kwargs)
        poses.append(data.qpos.copy())

    mujoco.mj_step = recording_step
    try:
        yield poses
    finally:
        mujoco.mj_step = real_step


def run_terrain(params) -> tuple[object, np.ndarray]:
    """Run the ride and come back with a pose per recorded sample."""
    import inspect

    from sim.scenarios import terrain as terrain_mod

    if "capture_state" in inspect.signature(terrain_mod.run).parameters:
        result = terrain_mod.run(params, capture_state=True)  # type: ignore[call-arg]
        return result, np.asarray(result.qpos)

    with _pose_recorder() as poses:
        result = terrain_mod.run(params)
    if len(poses) != len(result.t):
        raise RuntimeError(
            f"recorded {len(poses)} poses for {len(result.t)} samples. The "
            "terrain loop no longer steps once per sample, so the recorded "
            "trajectory would not line up with the metrics beside it. Fix the "
            "recorder, or -- better -- add capture_state= to terrain.run()"
        )
    return result, np.asarray(poses)


#: How far ahead of the board the camera looks, metres. Enough that the board
#: sits behind centre and the ground it is about to ride is the larger half of
#: the frame -- a descent reads as somewhere to go rather than as a wall.
CAM_LEAD_M = 1.6
#: Aim point above the ground, metres. Roughly the rider's waist: it keeps the
#: board off the bottom edge without pointing the lens at empty sky.
CAM_AIM_HEIGHT_M = 0.55


def terrain_camera(model, distance_m: float, fovy_deg: float,
                   elevation_deg: float = -9.0):
    """Content's shot, defined here rather than in the scenario.

    `terrain.build_terrain_model` ships a world-fixed `terrain` camera whose own
    comment calls it a compromise and says the real shot is this role's. It is
    right that it is a compromise: holding both crests needs distance, and at
    8% a 24 m roller is only 0.6 m of relief, so from far enough back to frame
    the whole profile the hill flattens toward a texture.

    A side-on camera that GLIDES ALONG THE RIDE resolves it the other way. The
    board stays large enough to read its pitch, and because the ground is
    genuinely tilted -- not a flat plane under rotated gravity -- the ground
    line sweeps through frame at the local slope. That, plus the profile strip
    in the HUD, is what makes the grade legible without pretending the hill is
    steeper than it is.

    Free rather than `mjCAMERA_TRACKING` because tracking aims at the tracked
    body's centre of mass, which on a ridden board is most of a metre above the
    wheel and drifts with the rider's lean. `aim_terrain_camera` points it at
    the GROUND instead, so the horizon stays put and the only thing that tilts
    in frame is the terrain.
    """
    import mujoco

    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.distance = distance_m
    # Looking from -y, so forward travel (toward -x) crosses the frame left to
    # right. The alternative reads as the board riding backwards.
    cam.azimuth = -90.0
    cam.elevation = elevation_deg
    # Free and tracking cameras take their field of view from the model's
    # global visual settings rather than from the camera object.
    model.vis.global_.fovy = fovy_deg
    return cam


def aim_terrain_camera(cam, board_x: float, amp: float, length: float) -> None:
    """Point the camera at the ground just ahead of the board."""
    lead_x = board_x - CAM_LEAD_M
    cam.lookat[0] = lead_x
    cam.lookat[1] = 0.0
    cam.lookat[2] = amp * math.cos(2.0 * math.pi * lead_x / length) + CAM_AIM_HEIGHT_M


def _chip(d, xy, text, font, fg, bg) -> float:
    """A bordered badge. Returns its width."""
    x, y = xy
    tw = d.textlength(text, font=font)
    d.rounded_rectangle([x, y, x + tw + 20, y + 26], radius=5, fill=(*bg, 235),
                        outline=(*fg, 255), width=1)
    d.text((x + 10, y + 5), text, font=font, fill=fg)
    return tw + 20


def _draw_profile_strip(d, box, params, amp, markers, fonts, *, compact=False,
                        caption=True) -> None:
    """The ground itself, crest to crest, with a marker where the board is.

    This is the element that makes the grade readable. The camera can show a
    slope but not WHERE on the profile the board is; this shows both, and says
    on its face how much the vertical is exaggerated so nobody reads a 0.6 m
    roller as a mountain.
    """
    f_small, f_tiny = fonts
    x0, y0, x1, y1 = box
    d.rectangle(box, fill=(*INK, 205))

    length = params.crest_to_crest_m
    pad = 16
    s_lo, s_hi = -2.5, length + 2.5
    inner_w = (x1 - x0) - 2 * pad
    top = y0 + (10 if compact else 26)
    bot = y1 - (20 if compact else 26)

    def sx(s: float) -> float:
        return x0 + pad + inner_w * (s - s_lo) / (s_hi - s_lo)

    def sz(z: float) -> float:
        return bot - (bot - top) * (z + amp) / (2.0 * amp)

    # Forward travel is toward -x and cos is even, so ground height at travel s
    # is the same expression as ground height at world x = s.
    ss = np.linspace(s_lo, s_hi, 220)
    zs = amp * np.cos(2.0 * np.pi * ss / length)
    line = [(sx(s), sz(z)) for s, z in zip(ss, zs)]
    d.polygon(line + [(sx(s_hi), y1 - 2), (sx(s_lo), y1 - 2)], fill=(*MUTED, 55))
    d.line(line, fill=(*MUTED, 210), width=2)

    for s, label in ((0.0, "CREST"), (length / 2.0, "DIP"), (length, "CREST")):
        z = amp * math.cos(2.0 * math.pi * s / length)
        d.line([(sx(s), sz(z)), (sx(s), y1 - 4)], fill=(*MUTED, 70), width=1)
        tw = d.textlength(label, font=f_tiny)
        d.text((sx(s) - tw / 2, y1 - 18), label, font=f_tiny, fill=(*MUTED, 220))

    for travel, colour, tag in markers:
        s = float(np.clip(travel, s_lo, s_hi))
        z = amp * math.cos(2.0 * math.pi * s / length)
        # Clamped into the strip: at a crest the marker sits on the top of the
        # curve, and an unclamped offset puts half of it outside the panel.
        cx = sx(s)
        cy = float(np.clip(sz(z) - 7, top + 7, bot))
        d.ellipse([cx - 6, cy - 6, cx + 6, cy + 6], fill=colour, outline=CLOUD)
        if tag:
            d.text((cx + 10, cy - 7), tag, font=f_tiny, fill=colour)

    if caption:
        exag = ((s_hi - s_lo) / inner_w) / ((2.0 * amp) / (bot - top))
        cap = (f"{length:.0f} m crest to crest · {params.max_grade_pct:.0f}% peak grade "
               f"· vertical ×{exag:.0f}")
        d.text((x0 + pad, y0 + 5), cap, font=f_small if not compact else f_tiny,
               fill=(*MUTED, 235))


def _draw_grade_panel(d, box, grade_pct: float, peak_pct: float, fonts) -> None:
    """Local grade: the number, the direction as a word, and a bar against peak.

    A wedge drawn at the true angle was the first attempt and it is unreadable:
    8% is 4.6°, which at panel scale is a line indistinguishable from flat. The
    honest fix is not to exaggerate the wedge but to drop it -- the number and
    the word carry the reading, and the bar says how much of THIS profile's
    peak grade the board is on right now.
    """
    f_small, f_body, f_big = fonts
    x0, y0, x1, y1 = box
    d.rectangle(box, fill=(*INK, 200))
    d.text((x0 + 18, y0 + 10), "LOCAL GRADE", font=f_small, fill=MUTED)

    # terrain.surface_grade_pct is forward-signed: positive means downhill is
    # forward. Showing a magnitude plus the word removes any chance of a viewer
    # inverting the sign convention in their head.
    word = ("DESCENDING" if grade_pct > 0.3 else
            "CLIMBING" if grade_pct < -0.3 else "LEVEL")
    colour = AMBER if abs(grade_pct) > 0.3 else CLOUD
    d.text((x0 + 18, y0 + 30), f"{abs(grade_pct):.1f}%", font=f_big, fill=colour)
    d.text((x0 + 18, y0 + 78), word, font=f_body, fill=CLOUD)

    bx0, bx1 = x1 - 116, x1 - 20
    by = y0 + 48
    d.rectangle([bx0, by, bx1, by + 14], fill=(*MUTED, 55), outline=(*MUTED, 110))
    frac = min(1.0, abs(grade_pct) / max(peak_pct, 1e-6))
    if frac > 0.01:
        d.rectangle([bx0, by, bx0 + (bx1 - bx0) * frac, by + 14], fill=colour)
    d.text((bx0, by - 18), "0", font=f_small, fill=MUTED)
    lbl = f"{peak_pct:.0f}% peak"
    d.text((bx1 - d.textlength(lbl, font=f_small), by - 18), lbl, font=f_small,
           fill=MUTED)


def _draw_terrain_hud(frame: np.ndarray, result, idx: int, source: Source,
                      amp: float, strike_deg: float, events: bool = True) -> np.ndarray:
    """The engineering readout for the ride.

    Carries what the camera cannot: where on the profile the board is, what the
    grade is there, and how far the attitude ESTIMATE has drifted from the
    truth -- which is the whole point of filming this scenario rather than a
    prettier one.
    """
    from PIL import Image, ImageDraw

    img = Image.fromarray(frame).convert("RGB")
    d = ImageDraw.Draw(img, "RGBA")
    f_title, f_body, f_small, f_big = _font(23), _font(17), _font(14), _font(40)
    f_tiny, f_huge = _font(12), _font(46)

    p, m = result.params, result.metrics
    t = float(result.t[idx])
    grade = float(result.grade_pct[idx])
    travel = float(result.travel_m[idx])
    pitch = float(result.pitch_deg[idx])
    est = float(result.pitch_est_deg[idx])

    # --- title block -------------------------------------------------------
    d.rectangle([0, 0, WIDTH, 64], fill=(*INK, 214))
    d.text((26, 13), "OVERBOARD", font=f_title, fill=AMBER)
    sub_x = 26 + d.textlength("OVERBOARD", font=f_title) + 22
    d.line([(sub_x - 11, 16), (sub_x - 11, 48)], fill=(*MUTED, 120), width=1)
    d.text((sub_x, 19), "rolling terrain — crest · dip · crest", font=f_body, fill=CLOUD)

    regime = ("attitude ESTIMATE" if p.use_estimator else "TRUTH pitch")
    chip_w = d.textlength(source.badge, font=f_small) + 20
    d.text((WIDTH - chip_w - 40 - d.textlength(regime, font=f_small), 23), regime,
           font=f_small, fill=MUTED)
    _chip(d, (WIDTH - chip_w - 26, 19), source.badge, f_small, AMBER, INK)

    # --- live readout ------------------------------------------------------
    d.rectangle([26, 88, 322, 244], fill=(*INK, 195))
    cols = (44, 196)
    rows = (
        (("t", f"{t:5.2f} s", CLOUD), ("travel", f"{travel:5.1f} m", CLOUD)),
        (("speed", f"{float(result.v_m_s[idx]):+5.2f} m/s", CLOUD),
         ("current", f"{float(result.motor_current_a[idx]):+5.1f} A", CLOUD)),
        (("pitch (truth)", f"{pitch:+6.2f}°", MINT),
         ("estimate", f"{est:+6.2f}°", AMBER)),
    )
    for r_i, row in enumerate(rows):
        y = 102 + r_i * 48
        for c_i, (label, value, colour) in enumerate(row):
            d.text((cols[c_i], y), label, font=f_small, fill=MUTED)
            d.text((cols[c_i], y + 17), value, font=f_body, fill=colour)

    _draw_grade_panel(d, (26, 256, 322, 366), grade, p.max_grade_pct,
                      (f_small, f_body, f_big))

    # --- bottom strips -----------------------------------------------------
    _draw_profile_strip(d, (26, HEIGHT - 172, 640, HEIGHT - 26), p, amp,
                        [(travel, MINT, "")], (f_small, f_tiny))

    # pitch: truth against the estimate, with the geometric strike line
    bx0, by0, bx1, by1 = 656, HEIGHT - 172, WIDTH - 26, HEIGHT - 26
    d.rectangle([bx0, by0, bx1, by1], fill=(*INK, 205))
    d.text((bx0 + 16, by0 + 5), "pitch — truth vs estimate (deg)", font=f_small,
           fill=(*MUTED, 235))
    lo = min(-strike_deg - 3.0, float(result.pitch_deg.min()) - 2.0,
             float(result.pitch_est_deg.min()) - 2.0)
    hi = max(strike_deg + 3.0, float(result.pitch_deg.max()) + 2.0,
             float(result.pitch_est_deg.max()) + 2.0)
    span_t = max(float(result.t[-1]), 1e-6)

    def px(i):
        return bx0 + 16 + (bx1 - bx0 - 32) * (result.t[i] / span_t)

    def py(v):
        return by1 - 20 - (by1 - by0 - 46) * ((v - lo) / (hi - lo))

    d.line([(bx0 + 16, py(0)), (bx1 - 16, py(0))], fill=(*MUTED, 110), width=1)
    for sign in (-1, 1):
        d.line([(bx0 + 16, py(sign * strike_deg)), (bx1 - 16, py(sign * strike_deg))],
               fill=(*AMBER, 120), width=1)
    lbl = f"bumper down at ±{strike_deg:.1f}°"
    d.text((bx1 - 16 - d.textlength(lbl, font=f_tiny), py(-strike_deg) + 3), lbl,
           font=f_tiny, fill=(*AMBER, 200))

    if idx > 1:
        step = max(1, idx // 500)
        for series, colour in ((result.pitch_est_deg, AMBER), (result.pitch_deg, MINT)):
            pts = [(px(i), py(float(series[i]))) for i in range(0, idx + 1, step)]
            if len(pts) > 1:
                d.line(pts, fill=colour, width=2)
        d.ellipse([px(idx) - 4, py(pitch) - 4, px(idx) + 4, py(pitch) + 4], fill=CLOUD)

    # --- event callouts ----------------------------------------------------
    CALLOUT_Y = 96

    def _callout(text: str, sub: str, colour, fade: int) -> None:
        # Backed by a panel, not floated over the render: these sit against a
        # pale sky, and amber-on-sky at 46 px is the one thing on the frame a
        # viewer is guaranteed to have to squint at.
        tw = d.textlength(text, font=f_huge)
        sw = d.textlength(sub, font=f_body)
        half = max(tw, sw) / 2 + 26
        d.rounded_rectangle([WIDTH / 2 - half, CALLOUT_Y - 14,
                             WIDTH / 2 + half, CALLOUT_Y + 88], radius=8,
                            fill=(*INK, int(fade * 0.78)))
        d.text(((WIDTH - tw) / 2, CALLOUT_Y), text, font=f_huge, fill=(*colour, fade))
        d.text(((WIDTH - sw) / 2, CALLOUT_Y + 56), sub, font=f_body, fill=(*CLOUD, fade))

    if events:
        if m.nose_strike and m.t_strike_s is not None and t >= m.t_strike_s:
            _callout(f"{m.struck_end.upper() or 'BUMPER'} DOWN",
                     f"t = {m.t_strike_s:.2f} s   {travel:+.1f} m of "
                     f"{p.crest_to_crest_m:.0f} m   grade {abs(grade):.1f}%",
                     ALARM, 255)
        elif m.t_reached_dip_s is not None and m.t_reached_dip_s <= t < m.t_reached_dip_s + 1.4:
            _callout("THROUGH THE DIP", "the grade reverses here", AMBER,
                     int(255 * max(0.0, 1.0 - (t - m.t_reached_dip_s) / 1.4)))
        elif m.t_reached_crest_s is not None and t >= m.t_reached_crest_s - 0.02:
            _callout("NEXT CREST", f"{p.crest_to_crest_m:.0f} m in "
                     f"{m.t_reached_crest_s:.1f} s", MINT, 255)

    return np.asarray(img)


def render_terrain_ride(result, qpos: np.ndarray, model, source: Source, amp: float,
                        strike_deg: float, *, distance_m: float, fovy_deg: float,
                        events: bool = True, only_index: int | None = None):
    """Replay the recorded ride through the offscreen renderer."""
    import mujoco

    cam = terrain_camera(model, distance_m, fovy_deg)
    data = mujoco.MjData(model)
    dt = float(model.opt.timestep)
    stride = max(1, int(round((1.0 / FPS) / dt)))

    renderer = mujoco.Renderer(model, height=HEIGHT, width=WIDTH)
    try:
        opt = mujoco.MjvOption()
        opt.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT] = False
        indices = ([only_index] if only_index is not None
                   else range(0, len(result.t), stride))
        frames = []
        length = result.params.crest_to_crest_m
        for i in indices:
            data.qpos[:] = qpos[i]
            mujoco.mj_forward(model, data)
            aim_terrain_camera(cam, float(qpos[i][0]), amp, length)
            renderer.update_scene(data, camera=cam, scene_option=opt)
            frames.append(_draw_terrain_hud(renderer.render().copy(), result, i,
                                            source, amp, strike_deg, events=events))
        return frames
    finally:
        renderer.close()


def _terrain_pane(frame, result, title: str, subtitle: str, colour,
                  width: int, verdict: bool) -> np.ndarray:
    """Label one pane of the comparison, and say plainly how it ended.

    `verdict` goes true once the pane is holding its final frame. Both outcomes
    get a band, not just the failure: a red banner with nothing opposite it
    reads as an accusation rather than a comparison, and the point is that the
    only difference between the two runs is where attitude came from.
    """
    from PIL import Image, ImageDraw

    img = Image.fromarray(frame)
    d = ImageDraw.Draw(img, "RGBA")
    f_title, f_sub, f_small, f_big = _font(24), _font(16), _font(13), _font(34)

    d.rectangle([0, 0, width, 40], fill=(*INK, 210))
    d.text((18, 8), title, font=f_title, fill=colour)
    tw = d.textlength(title, font=f_title)
    d.text((18 + tw + 16, 13), subtitle, font=f_sub, fill=CLOUD)

    m = result.metrics
    if not verdict or not (m.nose_strike or m.reached_next_crest):
        return np.asarray(img)

    # Persistent, not a fade. This pane has stopped; a viewer arriving at any
    # later second has to be able to see that it stopped and why.
    band_y = TERRAIN_PANE_H - 74
    if m.nose_strike:
        head = f"{(m.struck_end or 'bumper').upper()} DOWN"
        # Travel in metres, not a percentage of the ride. This run goes
        # backwards before it goes forwards, so `fraction_completed` is
        # negative and "-3% of 24 m" reads as a bug rather than as a board that
        # slid back over the crest it started on.
        travel = float(m.travel_m)
        where = (f"{abs(travel):.1f} m back over the start crest" if travel < 0
                 else f"{travel:.1f} m into a {m.crest_to_crest_m:.0f} m ride")
        detail = f"t = {m.t_strike_s:.2f} s   {where}"
        band = ALARM
    else:
        head = "NEXT CREST"
        detail = (f"{m.crest_to_crest_m:.0f} m crest to crest in "
                  f"{m.t_reached_crest_s:.1f} s")
        band = (26, 110, 98)

    d.rectangle([0, band_y, width, TERRAIN_PANE_H], fill=(*band, 215))
    d.text((22, band_y + 8), head, font=f_big, fill=CLOUD)
    hw = d.textlength(head, font=f_big)
    d.text((22 + hw + 18, band_y + 20), detail, font=f_sub, fill=CLOUD)
    if m.nose_strike:
        # Once, on the pane that failed. On both it is just noise.
        d.text((22, band_y + 48),
               "same terrain, same controller — only the attitude source differs",
               font=f_small, fill=(*CLOUD, 230))
    return np.asarray(img)


def render_terrain_comparison(truth, truth_q: np.ndarray, est, est_q: np.ndarray,
                              model, source: Source, amp: float, *,
                              distance_m: float, fovy_deg: float):
    """Truth pitch above, the attitude estimate below, one shared footer.

    The two runs are different lengths -- that IS the result -- so each pane
    replays its own trajectory and holds its final frame if it is the shorter.
    The estimate pane therefore sits on its own failure for the rest of the
    clip, under a banner that says so, while the truth pane rides on.
    """
    import mujoco
    from PIL import Image, ImageDraw

    cam = terrain_camera(model, distance_m, fovy_deg)
    data = mujoco.MjData(model)
    dt = float(model.opt.timestep)
    stride = max(1, int(round((1.0 / FPS) / dt)))
    n = max(len(truth.t), len(est.t))

    renderer = mujoco.Renderer(model, height=TERRAIN_PANE_H, width=WIDTH)
    try:
        opt = mujoco.MjvOption()
        opt.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT] = False

        length = truth.params.crest_to_crest_m

        def pane(result, qpos, i, title, colour, is_last):
            j = min(i, len(qpos) - 1)
            data.qpos[:] = qpos[j]
            mujoco.mj_forward(model, data)
            aim_terrain_camera(cam, float(qpos[j][0]), amp, length)
            renderer.update_scene(data, camera=cam, scene_option=opt)
            sub = (f"t {float(result.t[j]):5.2f} s   "
                   f"travel {float(result.travel_m[j]):+5.1f} m   "
                   f"grade {abs(float(result.grade_pct[j])):4.1f}%   "
                   f"pitch {float(result.pitch_deg[j]):+6.2f}°")
            # A pane has delivered its verdict once it has nothing left to
            # advance to. `is_last` covers the longer run, which the stride
            # generally overshoots rather than landing exactly on.
            verdict = is_last or i >= len(qpos) - 1
            return _terrain_pane(renderer.render().copy(), result, title, sub,
                                 colour, WIDTH, verdict), j

        f_small, f_tiny, f_body = _font(14), _font(12), _font(16)
        frames = []
        steps = list(range(0, n, stride))
        for i in steps:
            is_last = i == steps[-1]
            top, jt = pane(truth, truth_q, i, "TRUTH PITCH", MINT, is_last)
            bottom, je = pane(est, est_q, i, "ATTITUDE ESTIMATE", AMBER, is_last)

            # Footer: one header row, then the shared ground profile with a
            # marker per pane. Two rows rather than one because the markers and
            # their labels ride on the curve and collide with anything else put
            # in the same band.
            footer = Image.new("RGB", (WIDTH, TERRAIN_FOOTER_H), INK)
            fd = ImageDraw.Draw(footer, "RGBA")
            HEAD_H = 30
            _draw_profile_strip(fd, (0, HEAD_H, WIDTH, TERRAIN_FOOTER_H),
                                truth.params, amp,
                                [(float(truth.travel_m[jt]), MINT, "truth"),
                                 (float(est.travel_m[je]), AMBER, "estimate")],
                                (f_small, f_tiny), compact=True, caption=False)
            fd.rectangle([0, 0, WIDTH, HEAD_H], fill=INK)
            fd.text((18, 7), f"{truth.params.crest_to_crest_m:.0f} m crest to crest · "
                    f"{truth.params.max_grade_pct:.0f}% peak grade · "
                    f"v_ref {truth.params.v_ref_m_s:.1f} m/s",
                    font=f_body, fill=MUTED)
            chip_w = fd.textlength(source.badge, font=f_small) + 20
            _chip(fd, (WIDTH - chip_w - 20, 2), source.badge, f_small, AMBER, INK)
            frames.append(np.vstack([top, bottom, np.asarray(footer)]))
        return frames
    finally:
        renderer.close()


def save_terrain_plot(result, strike_deg: float, path: Path) -> None:
    """The GL-free proof for the ride. Always produced, even without a GPU."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    m, p = result.metrics, result.params
    fig, (ax, ax2, ax3) = plt.subplots(3, 1, figsize=(9, 8), sharex=True,
                                       gridspec_kw={"height_ratios": [3, 1.4, 1.4]})
    ax.plot(result.travel_m, result.pitch_deg, lw=2.0, color="#2AAE97",
            label="frame pitch (truth)")
    ax.plot(result.travel_m, result.pitch_est_deg, lw=1.5, color="#F2A24A",
            ls="--", label="attitude estimate")
    for sign in (-1, 1):
        ax.axhline(sign * strike_deg, ls=":", lw=1.4, color="#C4650F")
    ax.text(0.995, 0.045, f"bumper down at ±{strike_deg:.1f}° (from geometry)",
            transform=ax.transAxes, ha="right", va="bottom", fontsize=8,
            color="#C4650F")
    if m.t_strike_s is not None:
        ax.axvline(result.travel_m[-1], color="#D65A42", lw=1.6)
        ax.annotate(f"{m.struck_end or 'bumper'} down @ {m.t_strike_s:.2f}s",
                    (result.travel_m[-1], result.pitch_deg[-1]),
                    textcoords="offset points", xytext=(-12, 12), ha="right",
                    fontsize=9, color="#D65A42")
    ax.set_ylabel("pitch (deg)")
    ax.set_title(f"Overboard — rolling terrain, {p.max_grade_pct:.0f}% peak grade over "
                 f"{p.crest_to_crest_m:.0f} m, {'estimate' if p.use_estimator else 'truth'}-driven")
    ax.legend(loc="upper left", fontsize=9, framealpha=0.92)
    ax.grid(alpha=0.25)

    ax2.plot(result.travel_m, result.pitch_est_deg - result.pitch_deg, lw=1.6,
             color="#16232E")
    ax2.axhline(0, lw=0.8, color="#8a99a0")
    ax2.set_ylabel("estimate − truth\n(deg)")
    ax2.grid(alpha=0.25)

    ax3.plot(result.travel_m, result.grade_pct, lw=1.8, color="#8a99a0")
    ax3.axhline(0, lw=0.8, color="#8a99a0")
    ax3.set_ylabel("grade (%)\n+ is downhill")
    ax3.set_xlabel(f"travel (m) — dip at {p.crest_to_crest_m / 2:.0f} m, "
                   f"next crest at {p.crest_to_crest_m:.0f} m")
    ax3.grid(alpha=0.25)

    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def save_terrain_compare_plot(truth, est, strike_deg: float, path: Path) -> None:
    """Both runs on one pair of axes: the same ride, two attitude sources."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    p = truth.params
    fig, (ax, ax2) = plt.subplots(2, 1, figsize=(9, 6.5), sharex=True,
                                  gridspec_kw={"height_ratios": [2.4, 1]})
    ax.plot(truth.t, truth.pitch_deg, lw=2.0, color="#2AAE97", label="truth pitch")
    ax.plot(est.t, est.pitch_deg, lw=2.0, color="#F2A24A", label="attitude estimate")
    for sign in (-1, 1):
        ax.axhline(sign * strike_deg, ls=":", lw=1.4, color="#C4650F")
    if est.metrics.t_strike_s is not None:
        ax.plot([est.metrics.t_strike_s], [est.pitch_deg[-1]], "o", ms=9,
                color="#D65A42", zorder=5)
        ax.annotate(f"{est.metrics.struck_end or 'bumper'} down @ "
                    f"{est.metrics.t_strike_s:.2f}s",
                    (est.metrics.t_strike_s, est.pitch_deg[-1]),
                    textcoords="offset points", xytext=(14, 10), fontsize=9,
                    color="#D65A42")
    ax.set_ylabel("pitch (deg)")
    ax.set_title(f"Overboard — {p.max_grade_pct:.0f}% rolling terrain: truth pitch "
                 "completes the ride, the estimate does not")
    ax.legend(loc="lower left", fontsize=9, framealpha=0.92)
    ax.grid(alpha=0.25)

    ax2.plot(truth.t, truth.travel_m, lw=1.8, color="#2AAE97")
    ax2.plot(est.t, est.travel_m, lw=1.8, color="#F2A24A")
    ax2.axhline(p.crest_to_crest_m, ls="--", lw=1.2, color="#8a99a0")
    ax2.text(0.01, p.crest_to_crest_m, " next crest", va="top", fontsize=8,
             color="#8a99a0")
    ax2.set_ylabel("travel (m)")
    ax2.set_xlabel("time (s)")
    ax2.grid(alpha=0.25)

    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def render_terrain(args, source: Source) -> int:
    """Film the rolling-terrain ride, and optionally the truth/estimate pair."""
    from sim.scenarios.terrain import TerrainParams, amplitude_for_grade, summary_line

    out = args.out_dir
    written: list[Path] = []
    runs: dict[str, dict] = {}

    ride_params = TerrainParams(max_grade_pct=args.terrain_grade,
                                v_ref_m_s=args.terrain_speed)
    ride, ride_q = run_terrain(ride_params)
    amp = amplitude_for_grade(ride_params.max_grade_pct, ride_params.crest_to_crest_m)
    print(summary_line(ride))

    strike_deg = nose_strike_angle_deg(load_model())

    save_terrain_plot(ride, strike_deg, out / f"{TERRAIN_STEM}.png")
    print(f"wrote {out / f'{TERRAIN_STEM}.png'}")
    written.append(out / f"{TERRAIN_STEM}.png")

    (out / "terrain_metrics.json").write_text(
        json.dumps(ride.to_json_dict(), indent=2) + "\n")
    print(f"wrote {out / 'terrain_metrics.json'}")
    written.append(out / "terrain_metrics.json")
    runs["ride"] = {"params": asdict(ride_params), "metrics": asdict(ride.metrics)}

    renderer_info = {
        "width": WIDTH, "height": HEIGHT, "fps": FPS,
        "camera": "tracking, side-on, defined in render_scenario.terrain_camera",
        "camera_distance_m": args.terrain_distance,
        "camera_fovy_deg": args.terrain_fovy,
        "nose_strike_angle_deg": round(strike_deg, 3),
    }

    def finish(status: str) -> int:
        renderer_info["status"] = status
        write_manifest(out / "terrain_render_manifest.json", scenario="rolling terrain",
                       source=source, runs=runs, outputs=written,
                       renderer=renderer_info)
        return 0

    if args.no_video:
        return finish("plot + metrics only (--no-video)")

    from PIL import Image

    # The comparison is built first when asked for, because it is the artifact
    # that matters most here: it shows a limitation the project is working on,
    # rather than a clean success. If GL dies partway, that is the one to have.
    if args.compare:
        truth_params = TerrainParams(max_grade_pct=args.terrain_compare_grade,
                                     v_ref_m_s=args.terrain_speed, use_estimator=False)
        est_params = TerrainParams(max_grade_pct=args.terrain_compare_grade,
                                   v_ref_m_s=args.terrain_speed, use_estimator=True)
        truth, truth_q = run_terrain(truth_params)
        est, est_q = run_terrain(est_params)
        print(f"truth    {summary_line(truth)}")
        print(f"estimate {summary_line(est)}")

        # No try/except and no fallback: an "truth vs estimate" clip whose two
        # panes did not actually differ would be claiming a result it does not
        # have, in the most visible artifact this project publishes.
        if truth.metrics.nose_strike or not truth.metrics.reached_next_crest:
            raise RuntimeError(
                f"truth pitch did not complete the {args.terrain_compare_grade:.0f}% "
                "ride, so this is not the comparison it is captioned as. Re-derive "
                "the envelope with Senior Controls before publishing anything")

        for label, res in (("compare_truth", truth), ("compare_estimate", est)):
            runs[label] = {"params": asdict(res.params), "metrics": asdict(res.metrics)}
        (out / "terrain_truth_metrics.json").write_text(
            json.dumps(truth.to_json_dict(), indent=2) + "\n")
        (out / "terrain_estimate_metrics.json").write_text(
            json.dumps(est.to_json_dict(), indent=2) + "\n")
        written += [out / "terrain_truth_metrics.json",
                    out / "terrain_estimate_metrics.json"]
        save_terrain_compare_plot(truth, est, strike_deg, out / "terrain_compare.png")
        written.append(out / "terrain_compare.png")
        print(f"wrote {out / 'terrain_compare.png'}")

        try:
            frames = render_terrain_comparison(
                truth, truth_q, est, est_q, load_terrain_model(truth_params), source,
                amp, distance_m=args.terrain_distance,
                fovy_deg=args.terrain_compare_fovy)
        except Exception as exc:
            print(f"\ncomparison render unavailable ({type(exc).__name__}: {exc})")
            print("plots + metrics were still written; set MUJOCO_GL=osmesa or =egl.")
            return finish(f"plots + metrics only ({type(exc).__name__})")
        # Both verdict bands only appear on the final frames; hold them so the
        # clip ends on the comparison rather than cutting away from it.
        frames += [frames[-1]] * int(round(1.6 * FPS))
        write_video(frames, out / "terrain_compare.mp4", "libx264", 6)
        written.append(out / "terrain_compare.mp4")
        print(f"wrote {out / 'terrain_compare.mp4'} ({len(frames)} frames @ {FPS}fps)")

    try:
        model = load_terrain_model(ride_params)
        frames = render_terrain_ride(ride, ride_q, model, source, amp, strike_deg,
                                     distance_m=args.terrain_distance,
                                     fovy_deg=args.terrain_fovy)
    except Exception as exc:
        print(f"\noffscreen rendering unavailable ({type(exc).__name__}: {exc})")
        print("plots + metrics were still written; set MUJOCO_GL=osmesa or =egl.")
        return finish(f"plots + metrics only ({type(exc).__name__})")

    # Hold the last frame. The run stops the instant the far crest is reached,
    # so without this the "NEXT CREST" callout is on screen for one frame and
    # the clip ends on what looks like a cut rather than an arrival.
    frames += [frames[-1]] * int(round(1.2 * FPS))

    write_video(frames, out / f"{TERRAIN_STEM}.mp4", "libx264", 6)
    written.append(out / f"{TERRAIN_STEM}.mp4")
    print(f"wrote {out / f'{TERRAIN_STEM}.mp4'} ({len(frames)} frames @ {FPS}fps)")
    write_video(frames, out / f"{TERRAIN_STEM}.webm", "libvpx-vp9", 7)
    written.append(out / f"{TERRAIN_STEM}.webm")
    print(f"wrote {out / f'{TERRAIN_STEM}.webm'}")

    # Poster: the steepest point of the descent, which is the most expressive
    # pose and the one where the hill reads best.
    poster_i = int(np.argmax(ride.grade_pct))
    poster = render_terrain_ride(ride, ride_q, model, source, amp, strike_deg,
                                 distance_m=args.terrain_distance,
                                 fovy_deg=args.terrain_fovy, events=False,
                                 only_index=poster_i)[0]
    Image.fromarray(poster).save(out / "terrain_poster.jpg", quality=92)
    written.append(out / "terrain_poster.jpg")
    print(f"wrote {out / 'terrain_poster.jpg'}")

    return finish("ok")


def load_terrain_model(params):
    """A fresh model for rendering.

    Built rather than reused because `terrain.run` does not hand its model
    back, and because `terrain_camera` writes the field of view into
    `model.vis`, so the ride render and the comparison render must not share
    one instance.
    """
    from sim.scenarios.terrain import build_terrain_model

    model, _amp = build_terrain_model(params)
    return model


def render_impulse(args, source: Source) -> int:
    params = ImpulseParams(magnitude_ns=args.impulse, sim_seconds=args.seconds)
    written: list[Path] = []
    runs: dict[str, dict] = {}
    renderer_info = {"width": WIDTH, "height": HEIGHT, "fps": FPS,
                     "camera": args.camera}

    def finish(status: str) -> int:
        renderer_info["status"] = status
        write_manifest(args.out_dir / "impulse_render_manifest.json",
                       scenario="impulse disturbance response", source=source,
                       runs=runs, outputs=written, renderer=renderer_info)
        return 0

    model = load_model()
    strike = nose_strike_angle_deg(model)
    result = run(params, model=model, capture_state=True)
    m = result.metrics
    runs["open_loop"] = {"params": asdict(params), "metrics": asdict(m)}
    renderer_info["nose_strike_angle_deg"] = round(strike, 3)

    print(f"impulse            {params.magnitude_ns:.1f} N*s at t={params.t0_s}s")
    print(f"nose strike angle  {strike:.2f} deg (from collision hull)")
    print(f"peak |pitch|       {m.peak_abs_pitch_deg:.2f} deg at t={m.t_peak_s:.2f}s")
    print(f"nose strike        {m.nose_strike}"
          + (f" at t={m.t_strike_s:.3f}s, {m.speed_at_strike_ms:.2f} m/s, "
             f"{m.pitch_rate_at_strike_dps:.0f} deg/s" if m.nose_strike else ""))
    print(f"travel             {m.travel_m:.2f} m")

    save_pitch_plot(result, strike, args.out_dir / "impulse_pitch.png")
    print(f"wrote {args.out_dir / 'impulse_pitch.png'}")
    written.append(args.out_dir / "impulse_pitch.png")

    (args.out_dir / "impulse_metrics.json").write_text(
        json.dumps(result.to_json_dict(), indent=2) + "\n"
    )
    print(f"wrote {args.out_dir / 'impulse_metrics.json'}")
    written.append(args.out_dir / "impulse_metrics.json")

    if not args.no_video:
        try:
            frames = render_frames(result, args.camera)
        except Exception as exc:
            # Rendering is a publishing concern, not a correctness one. The
            # physics gate lives in tests/ and has already run without GL.
            print(f"\noffscreen rendering unavailable ({type(exc).__name__}: {exc})")
            print("plot + metrics were still written; set MUJOCO_GL=osmesa or =egl for video.")
            return finish(f"plot + metrics only ({type(exc).__name__})")

        from PIL import Image

        write_video(frames, args.out_dir / f"{STEM}.mp4", "libx264", 6)
        print(f"wrote {args.out_dir / f'{STEM}.mp4'} ({len(frames)} frames @ {FPS}fps)")
        write_video(frames, args.out_dir / f"{STEM}.webm", "libvpx-vp9", 7)
        print(f"wrote {args.out_dir / f'{STEM}.webm'}")
        written += [args.out_dir / f"{STEM}.mp4", args.out_dir / f"{STEM}.webm"]

        # Poster: the strike itself — peak pitch, most expressive pose.
        poster_at = m.t_strike_s if m.t_strike_s is not None else m.t_peak_s
        pi = min(int(round(poster_at / float(model.opt.timestep))), len(result.qpos) - 1)
        Image.fromarray(render_poster(result, args.camera, pi)).save(
            args.out_dir / "impulse_poster.jpg", quality=92
        )
        print(f"wrote {args.out_dir / 'impulse_poster.jpg'}")
        written.append(args.out_dir / "impulse_poster.jpg")

        if not args.no_gif:
            write_gif(frames, args.out_dir / f"{STEM}.gif", result)
            print(f"wrote {args.out_dir / f'{STEM}.gif'}")
            written.append(args.out_dir / f"{STEM}.gif")

    if args.compare:
        # Imported here, not at module scope: the open-loop path is the CI
        # gate's artifact and must not start depending on a built cdylib.
        from sim.scenarios.rust_controller import RustController

        # No try/except. If the library is missing this must fail loudly --
        # silently publishing an "open vs closed" clip whose closed pane was
        # actually uncontrolled would be a lie in the most visible artifact
        # the project produces.
        with RustController() as ctl:
            closed = run(params, model=model, controller=ctl, capture_state=True)
            saturated = ctl.saturated_cycles
        cm = closed.metrics

        print()
        print(f"closed-loop peak   {cm.peak_abs_pitch_deg:.2f} deg "
              f"(open loop {m.peak_abs_pitch_deg:.2f})")
        print(f"closed-loop strike {cm.nose_strike}")
        print(f"closed-loop travel {cm.travel_m:.2f} m")
        print(f"peak current       {float(abs(closed.motor_current_a).max()):.2f} A "
              f"({saturated} saturated cycles)")

        (args.out_dir / "impulse_closed_loop_metrics.json").write_text(
            json.dumps(closed.to_json_dict(), indent=2) + "\n"
        )
        print(f"wrote {args.out_dir / 'impulse_closed_loop_metrics.json'}")
        written.append(args.out_dir / "impulse_closed_loop_metrics.json")
        runs["closed_loop"] = {"params": asdict(params), "metrics": asdict(cm)}

        if not args.no_video:
            try:
                cmp_frames = render_comparison(result, closed, args.camera)
            except Exception as exc:
                print(f"\ncomparison render unavailable ({type(exc).__name__}: {exc})")
                return finish(f"comparison unavailable ({type(exc).__name__})")
            write_video(cmp_frames, args.out_dir / "impulse_compare.mp4", "libx264", 6)
            print(f"wrote {args.out_dir / 'impulse_compare.mp4'} ({len(cmp_frames)} frames)")
            written.append(args.out_dir / "impulse_compare.mp4")

    return finish("ok")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", default="impulse", choices=("impulse", "terrain"),
                    help="which scenario to film (default: impulse)")
    ap.add_argument("--source", default="sim", choices=sorted(SOURCES),
                    help="where the data came from. Sets the on-frame badge and the "
                         "manifest's category; the categories themselves are defined "
                         "in the shared vocabulary, not here")
    ap.add_argument("--impulse", type=float, default=NOMINAL_IMPULSE_NS, help="N*s")
    ap.add_argument("--seconds", type=float, default=ImpulseParams().sim_seconds)
    ap.add_argument("--camera", default="beauty", choices=("beauty", "side"))
    ap.add_argument("--no-video", action="store_true", help="skip GL; plot + metrics only")
    ap.add_argument("--no-gif", action="store_true")
    ap.add_argument("--out-dir", type=Path, default=OUT_DIR)
    ap.add_argument(
        "--compare",
        action="store_true",
        help="also film a two-pane comparison: for --scenario impulse, the same "
        "disturbance closed-loop; for --scenario terrain, truth pitch against the "
        "attitude estimate (requires: cargo build --release -p control-ffi)",
    )

    terrain = ap.add_argument_group("rolling terrain")
    terrain.add_argument("--terrain-grade", type=float, default=8.0,
                         help="peak grade of the filmed ride, percent (default 8, the "
                              "steepest profile the board completes on the estimate)")
    terrain.add_argument("--terrain-compare-grade", type=float, default=10.0,
                         help="peak grade for the truth-vs-estimate comparison "
                              "(default 10, where the two outcomes diverge)")
    terrain.add_argument("--terrain-speed", type=float, default=2.0, help="v_ref, m/s")
    terrain.add_argument("--terrain-distance", type=float, default=9.0,
                         help="camera distance, m. Closer than this and the board "
                              "drifts left into the HUD's readout panel")
    terrain.add_argument("--terrain-fovy", type=float, default=28.0,
                         help="vertical field of view for the full-height ride, deg")
    terrain.add_argument("--terrain-compare-fovy", type=float, default=20.0,
                         help="vertical field of view for a comparison pane, deg. "
                              "Tighter than the ride's because a pane is 306 px tall "
                              "and fovy is VERTICAL -- the ride's number would leave "
                              "the board too small to read its pitch")
    args = ap.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    source = SOURCES[args.source]
    print(f"category           {source.category}  (source tag {source.tag})")

    if args.scenario == "terrain":
        return render_terrain(args, source)
    return render_impulse(args, source)


if __name__ == "__main__":
    raise SystemExit(main())
