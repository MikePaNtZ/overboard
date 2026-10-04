"""Author a test course for MuJoCo from a list of grade segments.

The course is the single source of truth: MuJoCo rides it (heightfield for
`sim-host --terrain`), the rider program follows its lane, and Unreal builds
its landscape and path from the same file.

The path runs along MuJoCo -X (board forward), centred on y = 0, starting at
x = +X0. The ridden surface is the path: flat across its width with a small
crown. Beside it is a short verge, then valley walls with smooth noise. The
walls are scenery; nothing is ridden there.

    python course.py OUT_DIR [--preset valley]

Writes OUT_DIR/course_hfield.bin (+ course_height.npy, metadata.json in the
format sim-host reads), OUT_DIR/lane.json (x, y_min, y_max of the clean lane),
OUT_DIR/course.json (segments, profile, path), and OUT_DIR/profile.png.
"""
import argparse, json, struct
from pathlib import Path
import numpy as np

# Segments: (name, length m, grade %). Grade is DOWNHILL-positive along travel.
# Grade changes are joined by vertical curves of radius R_SAG / R_CREST.
PRESETS = {
    "valley": [
        ("run-in", 12.0, 2.0),
        ("descent", 60.0, 12.0),
        ("valley floor", 25.0, 0.0),
        ("climb", 40.0, -8.0),
        ("crest", 18.0, 0.0),
    ],
}
R_SAG = 30.0      # m, concave transitions (grade decreases along travel)
R_CREST = 40.0    # m, convex transitions
PATH_WIDTH = 5.0  # m (two-way bike path)
CROWN = 0.015     # m, centre higher than the edges
VERGE = 1.5       # m of flat verge each side
WALL_GRADE = 0.25 # valley walls rise at 25 % beyond the verge
SPACING = 0.05    # m between heightfield posts
HALF = 100.0      # m half-extent (square grid, origin at the centre)
X0 = 90.0         # path start, MuJoCo x


def grade_profile(segs, ds=0.05):
    """Grade (fraction, downhill +) and height along the path, with vertical
    curves centred on each segment boundary."""
    total = sum(L for _, L, _ in segs)
    s = np.arange(0.0, total + ds, ds)
    g = np.zeros_like(s)
    bounds, acc = [], 0.0
    for name, L, gr in segs:
        bounds.append((acc, acc + L, gr / 100.0))
        acc += L
    # piecewise-constant grade
    for a, b, gr in bounds:
        g[(s >= a) & (s < b)] = gr
    g[s >= bounds[-1][1]] = bounds[-1][2]
    # vertical curves: linear grade change over L = R * |dg| around each boundary
    for i in range(1, len(bounds)):
        sb = bounds[i][0]
        g0, g1 = bounds[i - 1][2], bounds[i][2]
        dg = g1 - g0
        if dg == 0:
            continue
        # along travel, z' = -g; z'' = -dg/ds. Grade increasing downhill = convex (crest).
        R = R_CREST if dg > 0 else R_SAG
        L = R * abs(dg)
        m = (s >= sb - L / 2) & (s <= sb + L / 2)
        g[m] = g0 + dg * (s[m] - (sb - L / 2)) / L
    z = -np.concatenate([[0.0], np.cumsum(0.5 * (g[1:] + g[:-1]) * ds)])
    return s, g, z, bounds


def smooth_noise(shape, scale_px, amp, seed=7):
    """Cheap smooth noise: random grid, bicubic-ish upsample by repeated box blur."""
    from scipy import ndimage as nd
    rng = np.random.default_rng(seed)
    n = rng.standard_normal(shape)
    n = nd.gaussian_filter(n, scale_px)
    return amp * n / (n.std() + 1e-9)


def build(segs, out: Path):
    out.mkdir(parents=True, exist_ok=True)
    s, g, zs, bounds = grade_profile(segs)
    n = int(round(2 * HALF / SPACING)) + 1
    ax = (np.arange(n) - (n - 1) / 2) * SPACING          # same axis for x (cols) and y (rows)
    X, Y = np.meshgrid(ax, ax)                            # H[row=y, col=x]
    # path coordinate s = X0 - x along the path; clamp beyond the ends
    S = np.clip(X0 - X, 0.0, s[-1])
    zpath = np.interp(S, s, zs)
    ay = np.abs(Y)
    half_w = PATH_WIDTH / 2
    crown = CROWN * (1 - np.clip(ay / half_w, 0, 1) ** 2)
    wall = np.clip(ay - half_w - VERGE, 0, None)
    noise = smooth_noise((n, n), scale_px=int(6 / SPACING), amp=0.6) * np.clip(wall / 6.0, 0, 1)
    H = zpath + crown + WALL_GRADE * wall + noise
    # beyond the path ends, let the ground carry on level so nothing is a cliff
    H = H.astype(np.float32)
    c = (n - 1) // 2
    # metadata in the format sim-host / model.py read
    zmin, zmax = float(H.min()), float(H.max())
    (out / "course_hfield.bin").write_bytes(struct.pack("<ii", n, n) + H.astype("<f4").tobytes())
    np.save(out / "course_height.npy", H)
    meta = {"nrow": n, "ncol": n, "half_extent_m": HALF, "spacing_m": SPACING,
            "z_min_m": zmin, "z_max_m": zmax, "center_post_index": c,
            "row_col_convention": "height[row, col]; row increases with MuJoCo +Y, col with +X",
            "source": "sim/carve/course.py (authored course, not measured)"}
    (out / "metadata.json").write_text(json.dumps(meta, indent=1))
    # lane: the clean path, 0.25 m kept off each edge
    xs = np.arange(X0, X0 - s[-1] - 0.01, -2.0)
    lane = [[float(x), -half_w + 0.25, half_w - 0.25] for x in xs]
    (out / "lane.json").write_text(json.dumps(lane))
    course = {
        "segments": [{"name": nm, "length_m": L, "grade_pct": gr} for nm, L, gr in segs],
        "vertical_curve_radius_m": {"sag": R_SAG, "crest": R_CREST},
        "path": {"start_x_m": X0, "axis": "-X", "width_m": PATH_WIDTH, "crown_m": CROWN},
        "profile": {"s_m": s[::40].round(3).tolist(), "z_m": zs[::40].round(4).tolist(),
                    "grade_pct": (100 * g[::40]).round(2).tolist()},
        "spawn_x_m": X0 - 2.0,
    }
    (out / "course.json").write_text(json.dumps(course, indent=1))
    try:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        fig, a = plt.subplots(2, 1, figsize=(12, 6), dpi=100, sharex=True)
        a[0].plot(s, zs); a[0].set_ylabel("height m"); a[0].grid(alpha=.3)
        for (a0, b0, gr), (nm, _, _) in zip(bounds, segs):
            a[0].axvspan(a0, b0, alpha=0.06 if nm != "valley floor" else 0.12)
            a[0].text((a0 + b0) / 2, zs.max(), nm, ha="center", va="top", fontsize=8)
        a[1].plot(s, 100 * g); a[1].set_ylabel("grade % (downhill +)"); a[1].set_xlabel("distance along path m"); a[1].grid(alpha=.3)
        fig.tight_layout(); fig.savefig(out / "profile.png")
    except Exception as e:  # plotting is a convenience
        print("profile plot skipped:", e)
    print(f"course: {s[-1]:.0f} m, drop {zs.max()-zs.min():.2f} m, grid {n}x{n} @ {SPACING} m, z [{zmin:.2f}, {zmax:.2f}]")
    print(f"spawn: --spawn-x {course['spawn_x_m']}  (path start x = {X0})")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--preset", default="valley", choices=sorted(PRESETS))
    ap.add_argument("--r-sag", type=float, default=R_SAG, help="sag vertical-curve radius, m")
    ap.add_argument("--r-crest", type=float, default=R_CREST, help="crest vertical-curve radius, m")
    a = ap.parse_args()
    R_SAG, R_CREST = a.r_sag, a.r_crest
    build(PRESETS[a.preset], Path(a.out))
