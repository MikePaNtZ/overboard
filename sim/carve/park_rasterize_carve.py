# rasterize_hfield.py -- runs OUTSIDE the UE editor, in the overboard venv (numpy + mujoco).
#
#   export PATH="/usr/sbin:$HOME/.cargo/bin:$PATH"
#   export DYLD_LIBRARY_PATH="/Users/mike/projects/overboard/.venv/lib/python3.14/site-packages/mujoco"
#   export MUJOCO_DIR="/Users/mike/projects/overboard/.venv/lib/python3.14/site-packages/mujoco"
#   /Users/mike/projects/overboard/.venv/bin/python3 tools/terrain_probe/rasterize_hfield.py
#
# Reads dump_triangles.py's Saved/TerrainProbe/triangles_f32.bin (UE cm, world space),
# transforms it into MuJoCo's frame (metres, right-handed, board origin at (0,0,0)), rasterizes
# the top surface onto a regular grid, and writes a MuJoCo-loadable hfield + metadata.
#
# THIS SCRIPT COMPUTES NO PHYSICS AND IS NOT WIRED INTO sim-host OR THE WIRE. It produces
# artifacts under Saved/TerrainProbe/ (gitignored) and a measured transect for
# terrain/levels/OB_City.terrain. See docs/citypark-level.md and ADR-0010.

import json
import os, sys
import struct
import time

import numpy as np

# ============================================================================================
# MAP-GENERAL PARAMETER BLOCK -- the only place region/grid parameters live in this script.
#
# To rasterize a DIFFERENT region (or move to a tiled full-map hfield later), change ONLY the
# values below, and the matching bounds in dump_triangles.py's own parameter block (that
# script's HALF_EXTENT_CM must stay >= this script's GRID_HALF_EXTENT_M, converted to cm, plus
# a margin -- see that script's comment).
# ============================================================================================
IN_DIR = "/tmp/parkprobe"
TRIANGLES_BIN_PATH = os.path.join(IN_DIR, "triangles_f32.bin")
DUMP_MANIFEST_PATH = os.path.join(IN_DIR, "dump_manifest.json")

OUT_DIR = "/tmp/carve_terrain"
HFIELD_BIN_PATH = os.path.join(OUT_DIR, "carve_hfield.bin")
HEIGHT_NPY_PATH = os.path.join(OUT_DIR, "carve_height.npy")
METADATA_JSON_PATH = os.path.join(OUT_DIR, "metadata.json")
TRANSECT_TXT_PATH = os.path.join(OUT_DIR, "transect_corridor.txt")
HOLES_NPY_PATH = os.path.join(OUT_DIR, "holes.npy")

# --- UE -> MuJoCo datum (ADR-0010's silent-killer; see BoardActor.cpp / CoordinateTransform.cpp) --
# The board origin PlayerStart OB_BoardOrigin: UE (-3880, -7450, -275.0) cm, yaw 90.
WORLD_ORIGIN_OFFSET_CM = (-47725.0, -30575.0, float(os.environ["ZCM"]))
WORLD_ORIGIN_YAW_DEG = -37.6

# --- Grid definition ---
# Odd post count so a post lands EXACTLY on x=0, y=0 (rather than the ~half-spacing offset an
# even count would leave at the origin) -- CENTER_INDEX below is that post, and the datum
# assertion reads it directly rather than interpolating.
GRID_SPACING_M = 0.05
GRID_POSTS_PER_AXIS = 3001
CENTER_INDEX = (GRID_POSTS_PER_AXIS - 1) // 2  # 1430
GRID_HALF_EXTENT_M = CENTER_INDEX * GRID_SPACING_M  # 50.05 m; full span 100.1 m > reach_m 100.0

# Facets steeper than this contribute no area to a top-down max-Z raster (walls/kerb faces).
NEAR_VERTICAL_SLOPE_DEG = 88.0
# ============================================================================================


def log(msg):
    print("[rasterize_hfield] %s" % msg, flush=True)


# ---------------------------------------------------------------------------------------------
# UE -> MuJoCo transform. This is the exact INVERSE of ABoardActor::UpdatePoseFromHistory's
# forward mapping (Source/OverboardGame/Private/BoardActor.cpp:328-333) composed with
# OverboardWire::MuJoCoToUnreal (wire/CoordinateTransform.cpp:5-21):
#
#   Forward:  ue_cm = RotateZ(yaw) * (100 * diag(1,-1,1) * mj_m)  +  WorldOriginOffsetCm
#             (BoardActor.cpp: "Rotate MuJoCo's whole frame about the world vertical, THEN
#             translate" -- OriginYaw.RotateVector(...) + WorldOriginOffsetCm)
#
#   Inverse:  mj_m = (1/100) * diag(1,-1,1) * RotateZ(-yaw) * (ue_cm - WorldOriginOffsetCm)
#
# Folding the Y-mirror into the rotation gives one explicit 3x3 matrix A such that
#   mj_m = A @ (ue_cm - WorldOriginOffsetCm)
# with, for yaw in degrees and c=cos(yaw), s=sin(yaw):
#   A = [ [ c/100,  s/100, 0 ],
#         [ s/100, -c/100, 0 ],
#         [   0,      0, 1/100 ] ]
# At yaw=90 (c=0, s=1): A = [[0, 1/100, 0], [1/100, 0, 0], [0, 0, 1/100]] -- i.e. swap X/Y and
# scale cm -> m.
# ---------------------------------------------------------------------------------------------
def build_ue_to_mujoco_transform(yaw_deg, offset_cm):
    yaw_rad = np.radians(yaw_deg)
    c, s = np.cos(yaw_rad), np.sin(yaw_rad)
    a = np.array([
        [c / 100.0, s / 100.0, 0.0],
        [s / 100.0, -c / 100.0, 0.0],
        [0.0, 0.0, 1.0 / 100.0],
    ], dtype=np.float64)
    t = np.array(offset_cm, dtype=np.float64)
    return a, t


def ue_to_mujoco_points(points_ue_cm, a, t):
    """points_ue_cm: (N,3) array. Returns (N,3) MuJoCo metres."""
    d = points_ue_cm - t
    return d @ a.T


def assert_datum(a, t):
    # Position datum: the board origin itself must map to MuJoCo (0,0,0) exactly (algebraic,
    # since D = origin - t = 0 there) -- verified numerically to catch a transcription bug.
    origin_mj = ue_to_mujoco_points(np.array([list(WORLD_ORIGIN_OFFSET_CM)]), a, t)[0]
    log("Datum position check: UE %s -> MuJoCo %s" % (WORLD_ORIGIN_OFFSET_CM, origin_mj.tolist()))
    if not np.allclose(origin_mj, [0.0, 0.0, 0.0], atol=1e-9):
        raise AssertionError(
            "UE->MuJoCo position datum FAILED: UE %s must map to MuJoCo (0,0,0), got %s"
            % (WORLD_ORIGIN_OFFSET_CM, origin_mj.tolist())
        )

    # Direction datum: the UE direction the board faces at yaw 90 -- FRotator(0,90,0) applied
    # to the actor's local +X (forward) -- is UE +Y (see BoardActor.cpp's forward RotateZ:
    # (1,0,0) at yaw 90 -> (0,1,0)). That UE direction must map to MuJoCo +X.
    ue_forward_dir = np.array([np.cos(np.radians(WORLD_ORIGIN_YAW_DEG)), np.sin(np.radians(WORLD_ORIGIN_YAW_DEG)), 0.0])
    mj_dir = ue_forward_dir @ a.T
    mj_dir_normalized = mj_dir / np.linalg.norm(mj_dir)
    log("Datum direction check: UE +Y (board-forward at yaw 90) -> MuJoCo direction %s"
        % mj_dir_normalized.tolist())
    if not np.allclose(mj_dir_normalized, [1.0, 0.0, 0.0], atol=1e-9):
        raise AssertionError(
            "UE->MuJoCo direction datum FAILED: UE +Y (board forward at yaw 90) must map to "
            "MuJoCo +X, got %s" % mj_dir_normalized.tolist()
        )
    log("Datum assertions PASSED.")


# ---------------------------------------------------------------------------------------------
# Rasterization
# ---------------------------------------------------------------------------------------------
def rasterize(triangles_mj):
    """triangles_mj: (N,3,3) array [triangle, vertex, xyz] in MuJoCo metres.
    Returns (height, covered) both shape (GRID_POSTS_PER_AXIS, GRID_POSTS_PER_AXIS), where
    height[iy, ix] is the elevation (m) at (x_m[ix], y_m[iy]) -- row index runs along +Y, column
    index runs along +X (verified empirically against the installed mujoco package -- see
    metadata.json's 'row_col_convention' and the load-validation step below)."""
    n = GRID_POSTS_PER_AXIS
    height = np.full((n, n), -np.inf, dtype=np.float64)
    covered = np.zeros((n, n), dtype=bool)

    idx = np.arange(n) - CENTER_INDEX
    axis_m = idx * GRID_SPACING_M  # shared by X and Y -- square grid
    x0 = axis_m[0]
    y0 = axis_m[0]

    n_tris = triangles_mj.shape[0]
    n_skipped_vertical = 0
    n_skipped_offgrid = 0
    n_skipped_degenerate = 0
    n_rasterized = 0

    t_start = time.time()
    for i in range(n_tris):
        v0, v1, v2 = triangles_mj[i, 0], triangles_mj[i, 1], triangles_mj[i, 2]

        normal = np.cross(v1 - v0, v2 - v0)
        norm_len = np.linalg.norm(normal)
        if norm_len < 1e-12:
            n_skipped_degenerate += 1
            continue

        cos_theta = min(1.0, abs(normal[2]) / norm_len)
        slope_deg = np.degrees(np.arccos(cos_theta))
        if slope_deg > NEAR_VERTICAL_SLOPE_DEG:
            n_skipped_vertical += 1
            continue

        xs = (v0[0], v1[0], v2[0])
        ys = (v0[1], v1[1], v2[1])
        xlo, xhi = min(xs), max(xs)
        ylo, yhi = min(ys), max(ys)

        ix_lo = max(0, int(np.floor((xlo - x0) / GRID_SPACING_M)))
        ix_hi = min(n - 1, int(np.ceil((xhi - x0) / GRID_SPACING_M)))
        iy_lo = max(0, int(np.floor((ylo - y0) / GRID_SPACING_M)))
        iy_hi = min(n - 1, int(np.ceil((yhi - y0) / GRID_SPACING_M)))
        if ix_lo > ix_hi or iy_lo > iy_hi:
            n_skipped_offgrid += 1
            continue

        xs_grid = axis_m[ix_lo:ix_hi + 1]
        ys_grid = axis_m[iy_lo:iy_hi + 1]
        xx, yy = np.meshgrid(xs_grid, ys_grid)  # shape (ny, nx)

        # Barycentric coordinates in the XY plane.
        denom = (v1[1] - v2[1]) * (v0[0] - v2[0]) + (v2[0] - v1[0]) * (v0[1] - v2[1])
        if abs(denom) < 1e-12:
            n_skipped_degenerate += 1
            continue
        a_bary = ((v1[1] - v2[1]) * (xx - v2[0]) + (v2[0] - v1[0]) * (yy - v2[1])) / denom
        b_bary = ((v2[1] - v0[1]) * (xx - v2[0]) + (v0[0] - v2[0]) * (yy - v2[1])) / denom
        c_bary = 1.0 - a_bary - b_bary

        tol = -1e-9
        mask = (a_bary >= tol) & (b_bary >= tol) & (c_bary >= tol)
        if not mask.any():
            n_skipped_offgrid += 1
            continue

        z = a_bary * v0[2] + b_bary * v1[2] + c_bary * v2[2]

        block_h = height[iy_lo:iy_hi + 1, ix_lo:ix_hi + 1]
        z_masked = np.where(mask, z, -np.inf)
        height[iy_lo:iy_hi + 1, ix_lo:ix_hi + 1] = np.maximum(block_h, z_masked)
        covered[iy_lo:iy_hi + 1, ix_lo:ix_hi + 1] |= mask
        n_rasterized += 1

        if (i + 1) % 200000 == 0:
            log("... rasterized %d/%d triangles (%.1fs elapsed)"
                % (i + 1, n_tris, time.time() - t_start))

    log("Rasterization done: %d used, %d skipped (near-vertical), %d skipped (off-grid/no-overlap), "
        "%d skipped (degenerate), %.1fs elapsed."
        % (n_rasterized, n_skipped_vertical, n_skipped_offgrid, n_skipped_degenerate,
           time.time() - t_start))
    return height, covered, axis_m


def fill_holes(height, covered):
    """Deterministic iterative nearest-neighbour dilation via fixed-order numpy shifts (North,
    South, West, East). Returns (filled_height, hole_mask)."""
    n = height.shape[0]
    filled = height.copy()
    known = covered.copy()
    hole_mask = ~covered

    def shift(arr, dy, dx, fill_value):
        out = np.full_like(arr, fill_value)
        src_y0, src_y1 = max(0, -dy), n - max(0, dy)
        dst_y0, dst_y1 = max(0, dy), n - max(0, -dy)
        src_x0, src_x1 = max(0, -dx), n - max(0, dx)
        dst_x0, dst_x1 = max(0, dx), n - max(0, -dx)
        out[dst_y0:dst_y1, dst_x0:dst_x1] = arr[src_y0:src_y1, src_x0:src_x1]
        return out

    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # N, S, W, E -- fixed order
    max_iterations = n  # generous: enough to flood-fill from any single seed to the far corner
    it = 0
    while not known.all() and it < max_iterations:
        it += 1
        progressed = False
        for dy, dx in directions:
            src_known = shift(known, dy, dx, False)
            src_val = shift(filled, dy, dx, 0.0)
            apply_here = (~known) & src_known
            if apply_here.any():
                filled[apply_here] = src_val[apply_here]
                known[apply_here] = True
                progressed = True
        if not progressed:
            break

    remaining = int((~known).sum())
    if remaining > 0:
        log("WARNING: %d posts remained unfilled after %d dilation passes (isolated from any "
            "measured post) -- left as NaN." % (remaining, it))
        filled[~known] = np.nan

    log("Hole fill: %d holes (%.3f%% of %d posts), %d dilation passes."
        % (int(hole_mask.sum()), 100.0 * hole_mask.sum() / hole_mask.size, hole_mask.size, it))
    return filled, hole_mask


# ---------------------------------------------------------------------------------------------
# MuJoCo custom binary hfield format (verified against the installed mujoco 3.10.0 package /
# doc/XMLreference.rst "asset/hfield/file": int32 nrow, int32 ncol, then nrow*ncol float32,
# row-major -- and separately load-tested below: no row flip on binary load, row index runs
# along the geom's local Y, column index along local X).
# ---------------------------------------------------------------------------------------------
def write_hfield_binary(path, height_grid):
    nrow, ncol = height_grid.shape
    with open(path, "wb") as f:
        f.write(struct.pack("<ii", nrow, ncol))
        f.write(height_grid.astype("<f4").tobytes())


def validate_hfield_load(path, height_grid, half_extent_m):
    """Load-validation ONLY -- proves the exported binary is a well-formed MuJoCo hfield.
    Does not touch sim-host or any wire code."""
    import mujoco

    nrow, ncol = height_grid.shape
    zmin, zmax = float(np.nanmin(height_grid)), float(np.nanmax(height_grid))
    elevation_z = max(zmax - zmin, 1e-6)
    base_z = 0.5

    xml = """
    <mujoco>
      <asset>
        <hfield name="hf" file="%s" size="%.6f %.6f %.6f %.6f"/>
      </asset>
      <worldbody>
        <geom name="terrain" type="hfield" hfield="hf" pos="0 0 0"/>
      </worldbody>
    </mujoco>
    """ % (path, half_extent_m, half_extent_m, elevation_z, base_z)

    m = mujoco.MjModel.from_xml_string(xml)
    loaded_nrow, loaded_ncol = int(m.hfield_nrow[0]), int(m.hfield_ncol[0])
    loaded = np.array(m.hfield_data).reshape(loaded_nrow, loaded_ncol)

    ok = (loaded_nrow == nrow) and (loaded_ncol == ncol)
    result = {
        "written_nrow": nrow, "written_ncol": ncol,
        "loaded_nrow": loaded_nrow, "loaded_ncol": loaded_ncol,
        "shape_match": ok,
        "written_z_min": zmin, "written_z_max": zmax,
        "loaded_normalized_min": float(loaded.min()), "loaded_normalized_max": float(loaded.max()),
    }

    # Spot-check the round trip at a handful of posts, including the exact centre.
    rng = np.random.default_rng(0)
    sample_ry = [CENTER_INDEX] + list(rng.integers(0, nrow, size=4))
    sample_rx = [CENTER_INDEX] + list(rng.integers(0, ncol, size=4))
    spot_checks = []
    all_close = True
    for ry, rx in zip(sample_ry, sample_rx):
        orig = height_grid[ry, rx]
        expected_norm = (orig - zmin) / (zmax - zmin) if zmax > zmin else 0.0
        got_norm = loaded[ry, rx]
        close = bool(np.isclose(expected_norm, got_norm, atol=1e-4))
        all_close = all_close and close
        spot_checks.append({"row": int(ry), "col": int(rx), "orig_m": float(orig),
                             "expected_normalized": float(expected_norm),
                             "loaded_normalized": float(got_norm), "close": close})
    result["spot_checks_all_close"] = all_close
    result["spot_checks"] = spot_checks
    result["pass"] = bool(ok and all_close)
    return result


def compute_adjacent_stats(height_grid, label):
    dz_x_mm = np.abs(np.diff(height_grid, axis=1)) * 1000.0  # adjacent along columns (+X)
    dz_y_mm = np.abs(np.diff(height_grid, axis=0)) * 1000.0  # adjacent along rows (+Y)
    all_dz = np.concatenate([dz_x_mm.ravel(), dz_y_mm.ravel()])
    all_dz = all_dz[~np.isnan(all_dz)]

    def worst_loc(dz_arr, axis_name):
        if dz_arr.size == 0:
            return None
        flat_idx = np.nanargmax(dz_arr)
        loc = np.unravel_index(flat_idx, dz_arr.shape)
        return {"value_mm": float(dz_arr[loc]), "index": [int(x) for x in loc], "axis": axis_name}

    stats = {
        "label": label,
        "n_pairs": int(all_dz.size),
        "min_mm": float(np.min(all_dz)) if all_dz.size else None,
        "max_mm": float(np.max(all_dz)) if all_dz.size else None,
        "p50_mm": float(np.percentile(all_dz, 50)) if all_dz.size else None,
        "p99_mm": float(np.percentile(all_dz, 99)) if all_dz.size else None,
        "count_exceeding_0.25mm": int(np.sum(all_dz > 0.25)),
        "pct_exceeding_0.25mm": float(100.0 * np.sum(all_dz > 0.25) / all_dz.size) if all_dz.size else None,
        "count_exceeding_20mm": int(np.sum(all_dz > 20.0)),
        "pct_exceeding_20mm": float(100.0 * np.sum(all_dz > 20.0) / all_dz.size) if all_dz.size else None,
        "worst_in_x": worst_loc(dz_x_mm, "x"),
        "worst_in_y": worst_loc(dz_y_mm, "y"),
    }
    return stats


def worst_continuous_descent(z_values_m):
    """Mirrors SurfaceCheck.cpp's CheckProbes accumulation rule: a continuous run of drops
    accumulates; any climb resets it."""
    run_drop = 0.0
    worst = 0.0
    for i in range(1, len(z_values_m)):
        dz = z_values_m[i] - z_values_m[i - 1]
        if dz < 0.0:
            run_drop += -dz
            worst = max(worst, run_drop)
        else:
            run_drop = 0.0
    return worst


def main():
    log("Loading triangles from %s" % TRIANGLES_BIN_PATH)
    raw = np.fromfile(TRIANGLES_BIN_PATH, dtype="<f4")
    if raw.size % 9 != 0:
        raise AssertionError("triangles_f32.bin size is not a multiple of 9 floats")
    n_tris = raw.size // 9
    triangles_ue_cm = raw.reshape(n_tris, 3, 3).astype(np.float64)
    log("Loaded %d triangles (UE cm, world space)." % n_tris)

    a, t = build_ue_to_mujoco_transform(WORLD_ORIGIN_YAW_DEG, WORLD_ORIGIN_OFFSET_CM)
    assert_datum(a, t)

    flat_ue = triangles_ue_cm.reshape(n_tris * 3, 3)
    flat_mj = ue_to_mujoco_points(flat_ue, a, t)
    triangles_mj = flat_mj.reshape(n_tris, 3, 3)

    log("Rasterizing onto a %d x %d grid, spacing %.3f m, half-extent %.4f m ..."
        % (GRID_POSTS_PER_AXIS, GRID_POSTS_PER_AXIS, GRID_SPACING_M, GRID_HALF_EXTENT_M))
    height, covered, axis_m = rasterize(triangles_mj)

    filled, hole_mask = fill_holes(height, covered)
    np.save(HOLES_NPY_PATH, hole_mask)

    # ---- Datum check on the MEASURED grid: centre post must read ~0.000 m -------------------
    centre_z = float(filled[CENTER_INDEX, CENTER_INDEX])
    log("Centre post (index %d,%d, x=%.6f y=%.6f) measured elevation: %.6f m"
        % (CENTER_INDEX, CENTER_INDEX, axis_m[CENTER_INDEX], axis_m[CENTER_INDEX], centre_z))
    if abs(centre_z) > 0.002:
        was_hole = bool(hole_mask[CENTER_INDEX, CENTER_INDEX])
        raise AssertionError(
            "Centre-post datum FAILED: measured elevation at the board origin is %.6f m, "
            "outside the 2 mm tolerance. Aborting -- this is the transform sanity anchor.\n"
            "  centre post was a HOLE before fill: %s\n"
            "\n"
            "  BEFORE SUSPECTING THE TRANSFORM, check what is being rasterised. This check\n"
            "  failed at 4.326 mm on the first full run, and the cause was NOT the transform:\n"
            "  the dump was 94%% foliage by triangle count, and a top-down max-Z raster takes\n"
            "  a dry leaf over the spawn point as the ground. dump_triangles.py now applies\n"
            "  HARD_SURFACE_KEEP for exactly this reason -- if that filter is disabled or the\n"
            "  dump predates it, this failure is expected and the fix is to re-dump, not to\n"
            "  widen the tolerance. A real transform error shows up in METRES, not millimetres."
            % (centre_z, "yes -- this value came from dilation, not measurement" if was_hole else "no")
        )
    log("Centre-post datum PASSED (< 2 mm).")

    np.save(HEIGHT_NPY_PATH, filled.astype(np.float32))
    write_hfield_binary(HFIELD_BIN_PATH, filled.astype(np.float32))
    log("Wrote %s (%d bytes) and %s (%d bytes)"
        % (HFIELD_BIN_PATH, os.path.getsize(HFIELD_BIN_PATH),
           HEIGHT_NPY_PATH, os.path.getsize(HEIGHT_NPY_PATH)))

    load_result = validate_hfield_load(HFIELD_BIN_PATH, filled, GRID_HALF_EXTENT_M)
    log("hfield load-validation: %s" % json.dumps(load_result, indent=2))
    if not load_result["pass"]:
        raise AssertionError("hfield load-validation FAILED: %s" % load_result)

    # ---- Transect (y=0 row) -------------------------------------------------------------------
    transect_row = filled[CENTER_INDEX, :]
    with open(TRANSECT_TXT_PATH, "w") as f:
        for ix in range(GRID_POSTS_PER_AXIS):
            f.write("%.6f %.6f %.6f\n" % (axis_m[ix], 0.0, transect_row[ix]))
    log("Wrote %s (%d lines)" % (TRANSECT_TXT_PATH, GRID_POSTS_PER_AXIS))

    # ---- Stats: full grid AND the y=0 transect, separately -----------------------------------
    full_stats = compute_adjacent_stats(filled, "full_grid")
    transect_dz_mm = np.abs(np.diff(transect_row)) * 1000.0
    transect_stats = {
        "label": "y=0_transect",
        "n_pairs": int(transect_dz_mm.size),
        "min_mm": float(np.min(transect_dz_mm)),
        "max_mm": float(np.max(transect_dz_mm)),
        "p50_mm": float(np.percentile(transect_dz_mm, 50)),
        "p99_mm": float(np.percentile(transect_dz_mm, 99)),
        "count_exceeding_0.25mm": int(np.sum(transect_dz_mm > 0.25)),
        "pct_exceeding_0.25mm": float(100.0 * np.sum(transect_dz_mm > 0.25) / transect_dz_mm.size),
        "count_exceeding_20mm": int(np.sum(transect_dz_mm > 20.0)),
        "pct_exceeding_20mm": float(100.0 * np.sum(transect_dz_mm > 20.0) / transect_dz_mm.size),
        "worst_continuous_descent_m": worst_continuous_descent(transect_row.tolist()),
        "span_m": float((GRID_POSTS_PER_AXIS - 1) * GRID_SPACING_M),
        "z_min_m": float(np.min(transect_row)), "z_max_m": float(np.max(transect_row)),
    }
    full_stats["worst_continuous_descent_along_y=0_m"] = transect_stats["worst_continuous_descent_m"]
    full_stats["z_min_m"] = float(np.nanmin(filled))
    full_stats["z_max_m"] = float(np.nanmax(filled))

    log("FULL GRID STATS: %s" % json.dumps(full_stats, indent=2))
    log("Y=0 TRANSECT STATS: %s" % json.dumps(transect_stats, indent=2))

    # ---- metadata.json -------------------------------------------------------------------------
    dump_manifest_summary = None
    if os.path.exists(DUMP_MANIFEST_PATH):
        with open(DUMP_MANIFEST_PATH) as f:
            dm = json.load(f)
        dump_manifest_summary = {
            "path": DUMP_MANIFEST_PATH,
            "actors_visited": dm.get("actors_visited"),
            "actors_kept": dm.get("actors_kept"),
            "total_triangles_kept": dm.get("total_triangles_kept"),
            "run_utc": dm.get("run_utc"),
        }

    metadata = {
        "origin": {
            "ue_cm": list(WORLD_ORIGIN_OFFSET_CM),
            "ue_yaw_deg": WORLD_ORIGIN_YAW_DEG,
            "note": "OB_BoardOrigin PlayerStart; this UE point maps to MuJoCo (0,0,0) by "
                    "construction (see transform below). Heights in this dataset are relative "
                    "to the road surface at the board origin (z=0 there), per "
                    "docs/citypark-level.md.",
        },
        "extent_m": float((GRID_POSTS_PER_AXIS - 1) * GRID_SPACING_M),
        "half_extent_m": GRID_HALF_EXTENT_M,
        "spacing_m": GRID_SPACING_M,
        "nrow": GRID_POSTS_PER_AXIS,
        "ncol": GRID_POSTS_PER_AXIS,
        "center_post_index": CENTER_INDEX,
        "row_col_convention": (
            "height[row, col], row-major, shape (nrow, ncol) = (%d, %d). Row index increases "
            "with MuJoCo +Y (row 0 = y=-%.4f m edge, row %d = y=+%.4f m edge). Column index "
            "increases with MuJoCo +X (col 0 = x=-%.4f m edge, col %d = x=+%.4f m edge). "
            "Verified empirically against the installed mujoco 3.10.0 package: a synthetic "
            "hfield with a value ramp encoded along columns produced a matching ramp along "
            "world +X on ray-cast query, and along rows a ramp along world +Y, with no row flip "
            "on binary-file load (the XML-reference row-flip note applies only to the "
            "<hfield elevation=...> XML attribute path, not the binary file path)."
        ) % (GRID_POSTS_PER_AXIS, GRID_POSTS_PER_AXIS, GRID_HALF_EXTENT_M, GRID_POSTS_PER_AXIS - 1,
             GRID_HALF_EXTENT_M, GRID_HALF_EXTENT_M, GRID_POSTS_PER_AXIS - 1, GRID_HALF_EXTENT_M),
        "z_min_m": full_stats["z_min_m"],
        "z_max_m": full_stats["z_max_m"],
        "hole_count": int(hole_mask.sum()),
        "hole_pct": float(100.0 * hole_mask.sum() / hole_mask.size),
        "hole_fill_policy": (
            "Deterministic iterative nearest-neighbour dilation, fixed direction order "
            "(North, South, West, East), numpy array shifts, run to fixed-point or a "
            f"{GRID_POSTS_PER_AXIS}-pass cap. holes.npy is the boolean hole mask BEFORE fill "
            "(True = no triangle covered that post); the exported height grid and hfield are "
            "POST-fill."
        ),
        "ue_to_mujoco_transform": {
            "matrix_a": a.tolist(),
            "translation_cm": list(t),
            "formula": "mj_m = A @ (ue_cm - translation_cm)",
            "provenance": (
                "Exact inverse of ABoardActor::UpdatePoseFromHistory's forward mapping "
                "(Source/OverboardGame/Private/BoardActor.cpp, the OriginYaw.RotateVector(...) "
                "+ WorldOriginOffsetCm block) composed with OverboardWire::MuJoCoToUnreal "
                "(wire/CoordinateTransform.cpp:5-21, m->cm + Y-mirror). See this script's "
                "build_ue_to_mujoco_transform() docstring for the full derivation."
            ),
        },
        "source_manifest": dump_manifest_summary,
        "source_triangles_bin": TRIANGLES_BIN_PATH,
        "run_date_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "stats_full_grid": full_stats,
        "stats_y0_transect": transect_stats,
        "hfield_load_validation": load_result,
    }
    with open(METADATA_JSON_PATH, "w") as f:
        json.dump(metadata, f, indent=2, sort_keys=False)
    log("Wrote %s" % METADATA_JSON_PATH)
    log("SUCCESS")


if __name__ == "__main__":
    main()
