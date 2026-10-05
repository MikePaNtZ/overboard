# dump_triangles.py -- UE editor python (run via -run=pythonscript, see README below).
#
# Extracts every triangle of every static-mesh surface (StaticMeshComponent AND
# InstancedStaticMeshComponent / HierarchicalInstancedStaticMeshComponent) that overlaps a
# probe region of OB_City, transformed to WORLD space, and writes them to a flat binary file
# for rasterize_hfield.py (which runs OUTSIDE the editor) to turn into a MuJoCo hfield.
#
# WHY GEOMETRY EXTRACTION, NOT LINE TRACES: see docs/citypark-level.md -- a commandlet/editor
# world has no physics scene, so every line trace misses. get_section_from_static_mesh is the
# proven method (used for the board-origin measurement in that same doc).
#
# NEVER SAVE ANYTHING. This script only reads the level. Calling save_current_level,
# save_map, save_dirty_packages etc. is how the sublevel got corrupted once before -- see
# docs/citypark-level.md, "never make the sublevel current / never save".
#
# Run:
#   "/Users/Shared/Epic Games/UE_5.7/Engine/Binaries/Mac/UnrealEditor-Cmd" \
#     "/Users/mike/projects/overboard-game/OverboardGame.uproject" \
#     -run=pythonscript -script="/Users/mike/projects/overboard-game/tools/terrain_probe/dump_triangles.py" \
#     -stdout -unattended -nosplash

import json
import os
import sys
import time

import unreal

# ============================================================================================
# MAP-GENERAL PARAMETER BLOCK -- the only place region bounds live in this script.
#
# To export a DIFFERENT region (or the whole map, by iterating tiled windows), change ONLY the
# values below. Nothing else in this script -- and nothing in rasterize_hfield.py downstream,
# which has its own equivalent block -- encodes region bounds.
# ============================================================================================
MAP_PACKAGE_PATH = "/Game/Maps/OB_City"

# ============================================================================================
# HARD-SURFACE FILTER -- the single most important parameter in this file.
#
# WHY IT EXISTS. The first full dump of OB_City was 12,631,439 triangles and **94% of them
# were foliage**:
#
#     Flora (grass/bushes/trees)   11,871,165   94.0%
#     Ground (dry leaves, sticks)     626,526    5.0%
#     MergeMeshes (the road)           65,142    0.5%
#     Road / ParkSquare / Props       ~68,000    0.5%
#
# One grass asset, SM_grass01_3, is 8,465,926 triangles. The entire road, SM_MergedRoad02, is
# 16,076. rasterize_hfield.py does a top-down MAX-Z raster, so with foliage in the dump it
# rasterises grass blades and leaf litter AS THE GROUND -- the road and its kerbs end up buried
# under vegetation. Three symptoms all trace back to this one cause:
#
#   * the centre-post datum check failing at 4.326 mm -- that is a LEAF over the spawn point,
#     not a transform error, which is what it was first assumed to be;
#   * 39.041% of posts coming out as holes -- the gaps between foliage clumps;
#   * a cross-section of the road that reads as noise, in which the real kerbs (measured at
#     y = +-4.5 m, ~0.21 m high) are invisible.
#
# A heightmap is a model of the surface you RIDE ON. Grass is not that surface -- a board does
# not ride on top of a blade of grass -- so this is a correctness filter, not an optimisation,
# though it is also a 100x one: ~130k triangles instead of 12.6M.
#
# HOW TO CHANGE IT. Keep is by substring against the mesh's asset path. Prefer adding to KEEP
# over removing from DROP: a surface wrongly kept produces a bump you can see and argue about,
# while a surface wrongly dropped produces a hole that hole-filling silently paves over.
# ============================================================================================
# Only meshes whose asset path contains one of these is extracted. Empty list = keep everything
# (the original behaviour, kept reachable so the filter's effect can be measured rather than
# assumed).
HARD_SURFACE_KEEP = [
    "/Meshes/MergeMeshes/",  # SM_MergedRoad02/04 -- the road itself, and the park square
    "/Meshes/Road/",         # bridges and road pieces
    "/Meshes/ParkSquare/",   # the paved plaza
]
# Belt and braces: even inside a kept path, anything matching these is dropped. Decals are
# zero-thickness overlays that would otherwise win a max-Z raster over the road they sit on --
# exactly the "thin thing on top of the real surface" failure this filter exists to prevent.
HARD_SURFACE_DROP = [
    "Decal",
    "SM_Leak",
]

# Centre of the probe region, UE world space, centimetres. This is OB_BoardOrigin's XY
# (docs/citypark-level.md: PlayerStart at (-3880, -7450, -275.0) cm, yaw 90).
CENTER_UE_X_CM = -3880.0
CENTER_UE_Y_CM = -7450.0

# Half-extent of the square XY probe region, centimetres. Chosen to comfortably cover
# rasterize_hfield.py's MuJoCo grid (2861 posts x 0.035 m spacing => 50.05 m / 5005.0 cm
# half-extent, see that script's own parameter block) plus a margin so no post along the
# raster's edge is starved of triangle coverage.
HALF_EXTENT_CM = 100000.0

# Z window, UE world space, centimetres (absolute, not component-relative). Keeps any
# triangle with at least one vertex between the road (-275 cm at the board origin) minus
# headroom for dips, and well above the tallest kerb/planter/step expected near the road.
Z_MIN_CM = -100000.0
Z_MAX_CM = 100000.0

# Sanity floor for "did the park sublevel actually stream in". docs/citypark-level.md reports
# 458 actors in a clean load; a persistent-level-only load (sublevel failed to stream) has a
# handful (GameMode, PlayerStart, level-streaming actor, ...). This threshold sits well below
# the real number and well above the failure-mode number.
MIN_EXPECTED_ACTORS = 50

OUT_DIR = "/tmp/parkprobe"
TRIANGLES_BIN_PATH = os.path.join(OUT_DIR, "triangles_f32.bin")
MANIFEST_PATH = os.path.join(OUT_DIR, "dump_manifest.json")
# ============================================================================================


def log(msg):
    print("[terrain_probe] %s" % msg)
    sys.stdout.flush()


def fail(msg):
    log("ABORT: %s" % msg)
    sys.exit(1)


def get_bbox_corners(box):
    """8 corners of an unreal.Box, as unreal.Vector."""
    mn, mx = box.min, box.max
    corners = []
    for x in (mn.x, mx.x):
        for y in (mn.y, mx.y):
            for z in (mn.z, mx.z):
                corners.append(unreal.Vector(x, y, z))
    return corners


def quat_to_matrix(q):
    """3x3 rotation matrix (row vectors are the rotated basis) from an unreal.Quat, so that
    world_vec = local_vec @ R (row-vector convention, matching how it is applied below)."""
    x, y, z, w = q.x, q.y, q.z, q.w
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    # Standard quaternion -> rotation matrix, ROW-major so that (row vector) @ R rotates it.
    return [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy + wz), 2.0 * (xz - wy)],
        [2.0 * (xy - wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz + wx)],
        [2.0 * (xz + wy), 2.0 * (yz - wx), 1.0 - 2.0 * (xx + yy)],
    ]


def transform_point(transform, p):
    """Local unreal.Vector -> world unreal.Vector via an unreal.Transform (scale, then
    rotate, then translate -- FTransform's own convention)."""
    s = transform.scale3d
    sx, sy, sz = p.x * s.x, p.y * s.y, p.z * s.z
    r = quat_to_matrix(transform.rotation)
    rx = sx * r[0][0] + sy * r[1][0] + sz * r[2][0]
    ry = sx * r[0][1] + sy * r[1][1] + sz * r[2][1]
    rz = sx * r[0][2] + sy * r[1][2] + sz * r[2][2]
    t = transform.translation
    return (rx + t.x, ry + t.y, rz + t.z)


class MeshCache:
    """Extracts each unique StaticMesh's LOD0 geometry (local space) exactly once."""

    def __init__(self):
        self.by_path = {}  # mesh path -> list of (v0,v1,v2) local tuples, or None on failure
        self.extraction_failures = []  # list of {path, nanite, error}

    def get_local_triangles(self, mesh):
        if mesh is None:
            return None
        path = mesh.get_path_name()
        if path in self.by_path:
            return self.by_path[path]

        tris = []
        error = None
        try:
            num_sections = mesh.get_num_sections(0)
        except Exception as e:  # noqa: BLE001 -- report, never silently skip
            num_sections = 0
            error = "get_num_sections(0) raised: %s" % e

        for section_index in range(num_sections):
            try:
                vertices, triangles, _normals, _uvs, _tangents = (
                    unreal.ProceduralMeshLibrary.get_section_from_static_mesh(
                        mesh, 0, section_index
                    )
                )
            except Exception as e:  # noqa: BLE001
                error = "get_section_from_static_mesh(lod0, section %d) raised: %s" % (
                    section_index, e,
                )
                continue
            if not vertices or not triangles:
                continue
            for i in range(0, len(triangles), 3):
                i0, i1, i2 = triangles[i], triangles[i + 1], triangles[i + 2]
                v0, v1, v2 = vertices[i0], vertices[i1], vertices[i2]
                tris.append(((v0.x, v0.y, v0.z), (v1.x, v1.y, v1.z), (v2.x, v2.y, v2.z)))

        if not tris:
            nanite = None
            try:
                nanite = bool(mesh.nanite_settings.enabled)
            except Exception:  # noqa: BLE001
                pass
            self.extraction_failures.append({
                "mesh_path": path,
                "num_sections_lod0": num_sections,
                "nanite_enabled": nanite,
                "error": error,
            })
            self.by_path[path] = None
            return None

        self.by_path[path] = tris
        return tris

    def get_local_bounds(self, mesh):
        try:
            return mesh.get_bounding_box()
        except Exception:  # noqa: BLE001
            return None


def aabb_overlaps_probe(world_corners, xmin, xmax, ymin, ymax, zmin, zmax):
    xs = [c[0] for c in world_corners]
    ys = [c[1] for c in world_corners]
    zs = [c[2] for c in world_corners]
    return (
        max(xs) >= xmin and min(xs) <= xmax
        and max(ys) >= ymin and min(ys) <= ymax
        and max(zs) >= zmin and min(zs) <= zmax
    )


def is_hard_surface(mesh_path):
    """True if this mesh is a surface the board can ride on.

    See HARD_SURFACE_KEEP above for why this is a correctness filter and not a speed one.
    """
    for drop in HARD_SURFACE_DROP:
        if drop.lower() in mesh_path.lower():
            return False
    if not HARD_SURFACE_KEEP:
        return True  # filter disabled -- original behaviour
    for keep in HARD_SURFACE_KEEP:
        if keep.lower() in mesh_path.lower():
            return True
    return False


def main():
    t_start = time.time()
    os.makedirs(OUT_DIR, exist_ok=True)

    xmin = CENTER_UE_X_CM - HALF_EXTENT_CM
    xmax = CENTER_UE_X_CM + HALF_EXTENT_CM
    ymin = CENTER_UE_Y_CM - HALF_EXTENT_CM
    ymax = CENTER_UE_Y_CM + HALF_EXTENT_CM
    zmin = Z_MIN_CM
    zmax = Z_MAX_CM

    log("Probe bounds (UE cm): x [%.2f, %.2f] y [%.2f, %.2f] z [%.2f, %.2f]"
        % (xmin, xmax, ymin, ymax, zmin, zmax))

    log("Loading map %s ..." % MAP_PACKAGE_PATH)
    les = unreal.LevelEditorSubsystem()
    ok = les.load_level(MAP_PACKAGE_PATH)
    if not ok:
        fail("LevelEditorSubsystem.load_level(%s) returned False" % MAP_PACKAGE_PATH)

    actor_subsystem = unreal.EditorActorSubsystem()
    all_actors = actor_subsystem.get_all_level_actors()
    log("Loaded. %d actors visible across persistent level + streamed sublevels." % len(all_actors))
    if len(all_actors) < MIN_EXPECTED_ACTORS:
        fail(
            "Only %d actors found (expected >= %d, ~458 when the City Park sublevel streams in "
            "correctly per docs/citypark-level.md). The always-loaded sublevel most likely did "
            "not stream in -- do not proceed on a park that isn't actually there."
            % (len(all_actors), MIN_EXPECTED_ACTORS)
        )

    cache = MeshCache()
    per_mesh_kept = {}
    actors_visited = 0
    meshes_skipped_soft = 0
    actors_kept = 0
    total_kept = 0
    components_visited = 0
    ism_instances_visited = 0

    out_f = open(TRIANGLES_BIN_PATH, "wb")
    try:
        for actor in all_actors:
            actors_visited += 1
            actor_contributed = False

            try:
                sm_components = actor.get_components_by_class(unreal.StaticMeshComponent)
            except Exception as e:  # noqa: BLE001
                log("WARNING: get_components_by_class failed on actor '%s': %s"
                    % (actor.get_name(), e))
                continue

            for comp in sm_components:
                components_visited += 1
                mesh = None
                try:
                    mesh = comp.static_mesh
                except Exception:  # noqa: BLE001
                    pass
                if mesh is None:
                    continue
                if not is_hard_surface(mesh.get_path_name()):
                    meshes_skipped_soft += 1
                    continue

                local_bounds = cache.get_local_bounds(mesh)

                if isinstance(comp, unreal.InstancedStaticMeshComponent):
                    try:
                        instance_count = comp.get_instance_count()
                    except Exception:  # noqa: BLE001
                        instance_count = 0
                    for inst_idx in range(instance_count):
                        ism_instances_visited += 1
                        inst_transform = comp.get_instance_transform(inst_idx, True)
                        if inst_transform is None:
                            continue
                        if local_bounds is not None and local_bounds.is_valid:
                            corners = [
                                transform_point(inst_transform, c)
                                for c in get_bbox_corners(local_bounds)
                            ]
                            if not aabb_overlaps_probe(corners, xmin, xmax, ymin, ymax, zmin, zmax):
                                continue
                        local_tris = cache.get_local_triangles(mesh)
                        if not local_tris:
                            continue
                        kept = write_kept_triangles(
                            out_f, local_tris, inst_transform, xmin, xmax, ymin, ymax, zmin, zmax
                        )
                        if kept:
                            total_kept += kept
                            actor_contributed = True
                            per_mesh_kept[mesh.get_path_name()] = (
                                per_mesh_kept.get(mesh.get_path_name(), 0) + kept
                            )
                else:
                    try:
                        world_transform = comp.get_world_transform()
                    except Exception:  # noqa: BLE001
                        continue
                    if local_bounds is not None and local_bounds.is_valid:
                        corners = [
                            transform_point(world_transform, c)
                            for c in get_bbox_corners(local_bounds)
                        ]
                        if not aabb_overlaps_probe(corners, xmin, xmax, ymin, ymax, zmin, zmax):
                            continue
                    local_tris = cache.get_local_triangles(mesh)
                    if not local_tris:
                        continue
                    kept = write_kept_triangles(
                        out_f, local_tris, world_transform, xmin, xmax, ymin, ymax, zmin, zmax
                    )
                    if kept:
                        total_kept += kept
                        actor_contributed = True
                        per_mesh_kept[mesh.get_path_name()] = (
                            per_mesh_kept.get(mesh.get_path_name(), 0) + kept
                        )

            if actor_contributed:
                actors_kept += 1

            if actors_visited % 2000 == 0:
                log("... %d/%d actors visited, %d triangles kept so far"
                    % (actors_visited, len(all_actors), total_kept))
    finally:
        out_f.close()

    elapsed = time.time() - t_start
    log("Done: %d actors visited, %d kept, %d components visited, %d ISM instances visited, "
        "%d triangles kept, %.1fs elapsed." % (
            actors_visited, actors_kept, components_visited, ism_instances_visited,
            total_kept, elapsed,
        ))
    if cache.extraction_failures:
        log("WARNING: %d mesh assets produced zero triangles on extraction (see manifest "
            "'extraction_failures' -- possible Nanite-only render data or missing LOD0)."
            % len(cache.extraction_failures))
        for f in cache.extraction_failures[:20]:
            log("  extraction failure: %s" % f)

    manifest = {
        "map_package_path": MAP_PACKAGE_PATH,
        "bounds_used_ue_cm": {
            "center_x": CENTER_UE_X_CM, "center_y": CENTER_UE_Y_CM,
            "half_extent": HALF_EXTENT_CM,
            "x_min": xmin, "x_max": xmax, "y_min": ymin, "y_max": ymax,
            "z_min": zmin, "z_max": zmax,
        },
        "actors_total_in_world": len(all_actors),
        "actors_visited": actors_visited,
        "actors_kept": actors_kept,
        "components_visited": components_visited,
        "ism_instances_visited": ism_instances_visited,
        "total_triangles_kept": total_kept,
        "per_mesh_asset_triangles_kept": per_mesh_kept,
        "mesh_extraction_failures": cache.extraction_failures,
        "elapsed_seconds": elapsed,
        "run_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "triangles_bin_path": TRIANGLES_BIN_PATH,
        "triangles_bin_format": "flat float32, 9 values per triangle (3 verts x XYZ), UE cm, world space",
    }
    with open(MANIFEST_PATH, "w") as f:
        json.dump(manifest, f, indent=2, sort_keys=True)
    log("Wrote %s (%d bytes)" % (TRIANGLES_BIN_PATH, os.path.getsize(TRIANGLES_BIN_PATH)))
    log("Wrote %s" % MANIFEST_PATH)
    log("SUCCESS")


def write_kept_triangles(out_f, local_tris, transform, xmin, xmax, ymin, ymax, zmin, zmax):
    """Transforms local_tris to world space with `transform`, keeps triangles with any vertex
    inside the XY bounds AND the Z window, writes them (9 float32 each) to out_f. Returns the
    count kept."""
    import struct

    kept = 0
    for v0, v1, v2 in local_tris:
        w0 = transform_point(transform, unreal.Vector(v0[0], v0[1], v0[2]))
        w1 = transform_point(transform, unreal.Vector(v1[0], v1[1], v1[2]))
        w2 = transform_point(transform, unreal.Vector(v2[0], v2[1], v2[2]))
        any_ok = False
        for w in (w0, w1, w2):
            if xmin <= w[0] <= xmax and ymin <= w[1] <= ymax and zmin <= w[2] <= zmax:
                any_ok = True
                break
        if not any_ok:
            continue
        out_f.write(struct.pack(
            "<9f",
            w0[0], w0[1], w0[2],
            w1[0], w1[1], w1[2],
            w2[0], w2[1], w2[2],
        ))
        kept += 1
    return kept


if __name__ == "__main__":
    main()
