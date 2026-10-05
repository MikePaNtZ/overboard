"""Write a small RECTANGULAR test level for sim-host's `--terrain` loader.

    python3 sim/carve/test_level.py [OUT_DIR]

The level is 60 m along X and 40 m along Y (a rectangle), posts 0.05 m apart,
flat at z = 0 with a 0.15 m step up for y >= 5 m. The metadata uses the
rectangular `half_extent_x_m` / `half_extent_y_m` keys, a `spawn` object at
(-10, -5) heading 90 deg, and a `bounds` rectangle. It exercises the new
loader paths without the City Park field. No board physics here: sim-host
makes the model and runs the sim.

The output directory gets `metadata.json` and `course_hfield.bin`; the script
prints the `--terrain` path.
"""
import json
import struct
import sys
from pathlib import Path

SPACING_M = 0.05
HALF_X_M = 30.0            # X spans 60 m (columns)
HALF_Y_M = 20.0            # Y spans 40 m (rows)
STEP_Y_M = 5.0            # a 0.15 m step up for y >= 5 m
STEP_HEIGHT_M = 0.15


def main():
    out_dir = Path(sys.argv[1] if len(sys.argv) > 1 else "/tmp/carve_test_level")
    out_dir.mkdir(parents=True, exist_ok=True)

    # Odd post counts so a post lands exactly on the origin.
    ncol = round(2 * HALF_X_M / SPACING_M) + 1
    nrow = round(2 * HALF_Y_M / SPACING_M) + 1
    assert ncol % 2 == 1 and nrow % 2 == 1

    heights = []
    z_min = 0.0
    z_max = STEP_HEIGHT_M
    for row in range(nrow):
        y = -HALF_Y_M + row * SPACING_M
        z = STEP_HEIGHT_M if y >= STEP_Y_M else 0.0
        heights.extend([z] * ncol)

    bin_path = out_dir / "course_hfield.bin"
    with open(bin_path, "wb") as f:
        f.write(struct.pack("<ii", nrow, ncol))
        f.write(struct.pack("<%df" % len(heights), *heights))

    meta = {
        "nrow": nrow,
        "ncol": ncol,
        "half_extent_m": HALF_X_M,
        "half_extent_x_m": HALF_X_M,
        "half_extent_y_m": HALF_Y_M,
        "z_min_m": z_min,
        "z_max_m": z_max,
        "row_col_convention": "height[row, col]; row increases with MuJoCo +Y, col with +X",
        "spawn": {"x": -10.0, "y": -5.0, "yaw_deg": 90.0},
        "bounds": {"xmin": -28.0, "xmax": 28.0, "ymin": -18.0, "ymax": 18.0},
        "source": "sim/carve/test_level.py (synthetic rectangular test level)",
    }
    with open(out_dir / "metadata.json", "w") as f:
        json.dump(meta, f, indent=1)

    print(f"level: {nrow}x{ncol} rect, spawn (-10,-5) yaw 90 deg, step 0.15 m at y>=5")
    print(f"--terrain {bin_path}")


if __name__ == "__main__":
    main()
