#!/usr/bin/env python3
"""Build the compact per-run plot data for the Overboard Lab page.

The page draws its charts in vanilla JS from a single pre-computed data file.
This script reads the raw wire tracks, controller traces and the terrain grid,
downsamples everything to about 40 Hz, derives the carve phases and the lane
cross-track, renders one grey hillshade per run, and computes every card
metric from the data (no hand-typed numbers). It writes `data/lab_data.js`.

The page loads that file with a `<script>` tag, so it also works from file://
(a fetch() call does not). The global is `window.LAB_DATA`.

Run:
  /Users/mike/projects/overboard/.venv/bin/python \
      sim/carve/build_lab_data.py

Output:
  <lab>/data/lab_data.js
"""

from __future__ import annotations

import base64
import io
import json
import os

import numpy as np

# --- Paths -----------------------------------------------------------------
LAB = "/Users/mike/projects/overboard-viz/out/carve-lab"
DATA = os.path.join(LAB, "data")
TERRAIN = os.path.join(DATA, "terrain")
OUT = os.path.join(DATA, "lab_data.js")

# Each run: the clip window (sim time, s) the video covers, and an optional
# hard cut (tmax). a01 flips at 5 s; the post-crash flailing is noise, so the
# plots stop just after the clip ends.
RUNS = {
    "a01": {"clip": [0.0, 7.0], "tmax": 7.0},
    "a02": {"clip": [3.0, 21.0], "tmax": None},
    "a03": {"clip": [3.0, 21.0], "tmax": None},
    "a04": {"clip": [3.0, 21.0], "tmax": None},
    "a05": {"clip": [3.0, 23.0], "tmax": 25.0},
}

RATE_HZ = 40.0          # output sample rate
ENVELOPE_A = 40.0       # motor current envelope (+/- A)
SPEED_MAX = 7.0         # fixed speed colour scale, shared across runs (m/s)

# Paint strip that tripped a01: the 2 cm centre-line dash, MuJoCo frame (m).
PAINT_STRIP = {"x0": -16.1, "x1": -15.4, "y0": 0.30, "y1": 0.45}

# --- Terrain ---------------------------------------------------------------
HALF_EXTENT = 75.0
SPACING = 0.05
CENTER = 1500


def load_terrain():
    h = np.load(os.path.join(TERRAIN, "carve_height.npy"))  # [row=y, col=x]

    def sample(x, y):
        col = np.clip(np.round(x / SPACING + CENTER).astype(int), 0, h.shape[1] - 1)
        row = np.clip(np.round(y / SPACING + CENTER).astype(int), 0, h.shape[0] - 1)
        return h[row, col]

    return h, sample


def load_csv(path):
    with open(path) as f:
        header = f.readline().strip().split(",")
    raw = np.loadtxt(path, delimiter=",", skiprows=1)
    return {name: raw[:, i] for i, name in enumerate(header)}


def resample(t_src, y_src, t_dst):
    return np.interp(t_dst, t_src, y_src)


def carve_phases(t, signal, thresh, min_dur=1.2):
    """Segment the run into left (+) / right (-) / straight carves."""
    sign = np.where(signal > thresh, 1, np.where(signal < -thresh, -1, 0))
    phases = []
    start = 0
    for i in range(1, len(sign)):
        if sign[i] != sign[start]:
            phases.append([t[start], t[i], int(sign[start])])
            start = i
    phases.append([t[start], t[-1], int(sign[start])])
    changed = True
    while changed and len(phases) > 1:
        changed = False
        out = [phases[0]]
        for p in phases[1:]:
            if (p[1] - p[0]) < min_dur or p[2] == out[-1][2]:
                out[-1][1] = p[1]
                changed = True
            else:
                out.append(p)
        phases = out
    return [{"t0": round(p[0], 2), "t1": round(p[1], 2), "dir": p[2]} for p in phases]


def hillshade_png(sub, dx, dy):
    """Render a small neutral-grey hillshade PNG and return a data URI."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import LightSource, LinearSegmentedColormap

    cmap = LinearSegmentedColormap.from_list("grey", ["#242b31", "#3a444c", "#59646d"])
    ls = LightSource(azdeg=315, altdeg=42)
    rgb = ls.shade(sub, cmap=cmap, vert_exag=5.0, dx=dx, dy=dy, blend_mode="soft")
    h, w = sub.shape
    fig = plt.figure(figsize=(w / 100, h / 100), dpi=100)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.imshow(rgb, origin="lower", interpolation="bilinear")
    ax.set_axis_off()
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=100)
    plt.close(fig)
    return "data:image/png;base64," + base64.b64encode(buf.getvalue()).decode()


def lane_interp(lane):
    """Return sorted lane x and the min/max edge arrays for np.interp."""
    arr = np.array(sorted(lane, key=lambda p: p[0]))
    return arr[:, 0], arr[:, 1], arr[:, 2]


def build_run(run, cfg, terrain, sampler, lane):
    npz = np.load(os.path.join(DATA, f"{run}_track.npz"))
    csv = load_csv(os.path.join(DATA, f"{run}_trace.csv"))

    t_raw = npz["t"]
    t_csv = csv["sim_time_s"]
    t_hi = float(min(t_raw[-1], t_csv[-1]))
    if cfg.get("tmax"):
        t_hi = min(t_hi, cfg["tmax"])
    n = int(t_hi * RATE_HZ) + 1
    t = np.linspace(0.0, t_hi, n)

    px = resample(t_raw, npz["px"], t)
    py = resample(t_raw, npz["py"], t)
    speed = resample(t_raw, np.sqrt(npz["vx"] ** 2 + npz["vy"] ** 2), t)
    yaw = resample(t_raw, np.degrees(npz["yaw"]), t)
    rider_fa = resample(t_raw, npz["rider_fa"], t)
    rider_lat = resample(t_raw, npz["rider_lat"], t)

    proposed = resample(t_csv, csv["proposed_amps"], t)
    applied = resample(t_csv, csv["applied_amps"], t)
    util = resample(t_csv, csv["utilisation_filtered"], t)
    saturated = resample(t_csv, csv["saturated"], t)

    # Fall: bit 2 of flags. Keep the index on the output time base.
    fb = ((npz["flags"].astype(int) >> 2) & 1)
    fallen_t = float(t_raw[np.argmax(fb)]) if fb.any() else None
    fall_idx = int(np.searchsorted(t, fallen_t)) if fallen_t is not None else None

    # Lane cross-track: board y against the clean lane centre and edges.
    lx, lymin, lymax = lane_interp(lane)
    ymin = np.interp(px, lx, lymin)
    ymax = np.interp(px, lx, lymax)
    centre = (ymin + ymax) / 2
    half = (ymax - ymin) / 2
    cross = py - centre                      # + = left of centre
    margin = np.minimum(py - ymin, ymax - py)  # clearance to the nearer edge

    # Distance and terrain elevation / grade along the path.
    seg = np.hypot(np.diff(px), np.diff(py))
    dist = np.concatenate([[0.0], np.cumsum(seg)])
    elev = sampler(px, py)
    win = max(3, int(RATE_HZ * 0.4))
    de = np.gradient(elev)
    dd = np.gradient(dist)
    # Grade = rise / run along the path. Where the board nearly stops, the run
    # per sample collapses and the ratio explodes; carry the last good value
    # there (interpolate over the gaps) so the ends do not spike.
    med = np.median(dd[dd > 0]) if np.any(dd > 0) else 1.0
    good = dd > 0.3 * med
    raw = np.full_like(elev, np.nan)
    raw[good] = -de[good] / dd[good] * 100.0
    gi = np.arange(len(raw))
    if good.any():
        raw = np.interp(gi, gi[good], raw[good])
    else:
        raw = np.zeros_like(elev)
    grade = np.convolve(raw, np.ones(win) / win, mode="same")
    grade = np.clip(grade, -12.0, 12.0)
    # The convolution and the stop at the run end leave small endpoint spikes;
    # hold the nearest stable sample at each end.
    if len(grade) > 2 * win:
        grade[:win] = grade[win]
        grade[-win:] = grade[-win - 1]

    # Carve phases from a heavily smoothed lateral lean (~1.5 s window).
    cwin = max(3, int(RATE_HZ * 1.5))
    lat_smooth = np.convolve(rider_lat, np.ones(cwin) / cwin, mode="same")
    amp = np.percentile(np.abs(lat_smooth), 90)
    phases = carve_phases(t, lat_smooth, thresh=0.3 * max(amp, 1e-4))

    # --- Metrics, all from the data ---------------------------------------
    # Valid window: the clip, cut at the fall.
    c0, c1 = cfg["clip"]
    if fallen_t is not None:
        c1 = min(c1, fallen_t)
    valid = (t >= c0) & (t <= c1)
    live = slice(0, fall_idx) if fall_idx is not None else slice(None)
    path_len = float(dist[live][-1] if fall_idx else dist[-1])
    peak_speed = float(np.max(speed[live]))
    margin_min = float(np.min(margin[valid])) if valid.any() else float(np.min(margin))
    grade_lo = float(np.min(grade[live]))
    grade_hi = float(np.max(grade[live]))

    if fallen_t is not None:
        verdict = f"Fell at {fallen_t:.1f} s"
        metrics = [
            ["Result", f"Fell at {fallen_t:.1f} s"],
            ["Distance to fall", f"{path_len:.0f} m"],
            ["Peak speed", f"{peak_speed:.1f} m/s"],
            ["Cause", "2 cm paint mesh"],
        ]
    else:
        verdict = "Upright"
        metrics = [
            ["Result", "Upright the full run"],
            ["Distance", f"{path_len:.0f} m"],
            ["Peak speed", f"{peak_speed:.1f} m/s"],
            ["Grade met", f"{grade_lo:.0f} to {grade_hi:.0f} %"],
            ["Lane clearance", f"{margin_min:.2f} m min (clip)"],
        ]
    # One-line verdict headline for the card.
    if fallen_t is not None:
        head = f"Fell at {fallen_t:.1f} s · {path_len:.0f} m · {peak_speed:.1f} m/s peak"
    else:
        head = (f"Upright · {path_len:.0f} m · {peak_speed:.1f} m/s peak "
                f"· {margin_min:.1f} m clearance")

    # --- Hillshade: path + lane bbox, exported at ~6 px/m ------------------
    margin_m = 3.0
    ylo = min(float(np.min(py)), float(np.min(ymin)))
    yhi = max(float(np.max(py)), float(np.max(ymax)))
    x0, x1 = float(np.min(px)) - margin_m, float(np.max(px)) + margin_m
    y0, y1 = ylo - margin_m, yhi + margin_m
    col0 = int(np.clip(x0 / SPACING + CENTER, 0, terrain.shape[1] - 1))
    col1 = int(np.clip(x1 / SPACING + CENTER, 0, terrain.shape[1] - 1))
    row0 = int(np.clip(y0 / SPACING + CENTER, 0, terrain.shape[0] - 1))
    row1 = int(np.clip(y1 / SPACING + CENTER, 0, terrain.shape[0] - 1))
    step = max(1, int(round((1 / 6.0) / SPACING)))   # ~6 px / m
    sub = terrain[row0:row1:step, col0:col1:step]
    img = hillshade_png(sub, dx=SPACING * step, dy=SPACING * step)
    extent = [round((col0 - CENTER) * SPACING, 2), round((col1 - CENTER) * SPACING, 2),
              round((row0 - CENTER) * SPACING, 2), round((row1 - CENTER) * SPACING, 2)]

    def a(x, nd):
        return [round(float(v), nd) for v in x]

    out = {
        "clip": cfg["clip"],
        "t": a(t, 2),
        "px": a(px, 2), "py": a(py, 2),
        "speed": a(speed, 2),
        "dist": a(dist, 1), "elev": a(elev, 2), "grade": a(grade, 1),
        "cross": a(cross, 2), "laneHalf": a(half, 2),
        "yaw": a(yaw, 1),
        "riderFA": a(rider_fa * 1000.0, 1), "riderLat": a(rider_lat * 1000.0, 1),
        "proposedA": a(proposed, 1), "appliedA": a(applied, 1),
        "util": a(np.clip(util, 0, 2), 3),
        "saturated": [int(round(v)) for v in saturated],
        "phases": phases,
        "fallenT": None if fallen_t is None else round(fallen_t, 2),
        "fallIdx": fall_idx,
        "envelope": ENVELOPE_A, "speedMax": SPEED_MAX,
        "metrics": metrics, "verdict": verdict, "head": head,
        "map": {"img": img, "extent": extent,
                "paint": PAINT_STRIP if run == "a01" else None},
    }
    print(f"  {run}: {verdict}; {path_len:.0f} m; peak {peak_speed:.2f} m/s; "
          f"clearance {margin_min:.2f} m; grade {grade_lo:.0f}..{grade_hi:.0f}%")
    return out


def main():
    terrain, sampler = load_terrain()
    with open(os.path.join(DATA, "lane.json")) as f:
        lane = json.load(f)

    out = {"lane": lane, "speedMax": SPEED_MAX}
    for run, cfg in RUNS.items():
        print(f"building {run} ...")
        out[run] = build_run(run, cfg, terrain, sampler, lane)

    with open(OUT, "w") as f:
        f.write("// Generated by sim/carve/build_lab_data.py. Do not edit by hand.\n")
        f.write("window.LAB_DATA = ")
        json.dump(out, f, separators=(",", ":"))
        f.write(";\n")
    print(f"wrote {OUT} ({os.path.getsize(OUT) / 1024:.0f} kB)")


if __name__ == "__main__":
    main()
