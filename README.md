# Overboard

Overboard is a DIY self-balancing **onewheel** (single hub motor, rideable
inverted pendulum). The long-term goal is a Rust real-time control loop
running on PREEMPT_RT Linux (Raspberry Pi), driving a VESC-controlled hub
motor. Development is sim-first, using MuJoCo as the shared physics model.

See the root `CLAUDE.md` for the full project context and working
conventions, and `docs/README.md` for where the design docs live.

## Status

This is the initial workspace scaffold plus the first visual checkpoint: a
MuJoCo onewheel model with **no controller** that visibly topples over under
gravity. The Rust control loop does not step MuJoCo yet (that's a later
milestone, tracked as a TODO in `crates/sim-backend`).

## Build the Rust workspace

```sh
cargo build --workspace
cargo test --workspace
cargo clippy --workspace --all-targets -- -D warnings
```

Run the (currently stubbed) control loop:

```sh
cargo run -p board-app -- --backend sim --cycles 100
```

## Watch the onewheel fall over (the checkpoint)

Set up the Python environment once:

```sh
python3 -m venv .venv
source .venv/bin/activate
pip install mujoco matplotlib
```

Then, to watch it live in an interactive window -- **on macOS this must run
under `mjpython`, not plain `python`**:

```sh
mjpython scripts/view_sim.py
```

Or to generate a headless proof (an animated GIF plus a pitch-angle-vs-time
plot in `sim/out/`, and the final tilt angle printed to stdout):

```sh
source .venv/bin/activate
python scripts/render_fall.py
```

## Layout

- `crates/board-types` -- shared types (`Command`, `Observation`, `Faults`, `Params`), `no_std`-friendly.
- `crates/hal` -- the `BoardIo` seam between control-core and a backend (sim or hardware).
- `crates/control-core` -- pure/deterministic control logic (stub: zero command for now).
- `crates/safety` -- pure safety envelope (stub: arm/disarm passthrough).
- `crates/sim-backend` -- `hal::BoardIo` backend (stub today; MuJoCo FFI is the next milestone).
- `crates/board-app` -- the binary that wires it all together.
- `sim/models` -- MuJoCo MJCF models (the shared physics asset).
- `scripts/` -- Python viewer/render scripts for the MuJoCo sim.
- `docs/` -- Markdown+Mermaid mirrors of the design docs (Notion is the source of truth).
