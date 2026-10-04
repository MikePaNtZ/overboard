# Overboard — project notes

A DIY self-balancing onewheel-style board. Current focus: **sim fidelity and renders** for the
moore-mike.com blog. Hardware is paused until Mike buys a DIY kit; hardware safety rules come then.

Earlier process rules, ADRs and role files were archived on 2026-10-03 (git tag
`archive/pre-reset-2026-10-03`). They are historical and do not bind any work. Comments in the
code that cite an ADR or a role describe history only.

## Repos
- `overboard` — Rust control code + MuJoCo sim (`crates/`, `sim/`, `tests/`).
- `overboard-game` — Unreal 5.7 client (City Park level, rider, wire to `sim-host`).
- `overboard-viz` — offline renders (Blender), and the Carve Lab page (`out/carve-lab`,
  served by `ops/serve-carve-lab.sh`).
- `overboard-web` — landing page.

## Working
- Use a feature branch and a PR; CI (`rust`, `sim`) must be green to merge.
- Never check in renders or binaries. `sim/out/` and `overboard-viz/out/` are gitignored.

## Build and run notes (macOS)
- `cargo`, `sysctl` can be missing from PATH: `export PATH="$HOME/.cargo/bin:/usr/sbin:/sbin:$PATH"`.
- Python with mujoco: `.venv/bin/python`. The Rust MuJoCo plant needs `MUJOCO_DIR` set to the
  venv's `mujoco` package directory to build, and `DYLD_LIBRARY_PATH` set to
  `target/release/build/plant-mujoco-*/out` to run.
- `sim-host --terrain HFIELD.bin` rides a heightmap from `overboard-game/tools/terrain_probe`;
  `--schedule-csv` plays a scripted stick schedule.
- Real-time `sim-host` runs on this Mac report many missed 2 ms deadlines even on flat ground;
  the tick count still tracks wall time.
