# Overboard

Overboard is a DIY self-balancing **onewheel** (single hub motor, rideable
inverted pendulum). The long-term goal is a Rust real-time control loop
running on PREEMPT_RT Linux (Raspberry Pi), driving a VESC-controlled hub
motor. Development is sim-first, using MuJoCo as the shared physics model.

See the root `CLAUDE.md` for the full project context and working
conventions, and `docs/README.md` for where the design docs live.

## Status

Workspace scaffold plus the first real sim-in-the-loop test: an **impulse
disturbance-response scenario** that runs in CI on every push. The Rust control
loop does not step MuJoCo yet (a later milestone, tracked as a TODO in
`crates/sim-backend`), so the scenario is open-loop — which makes it the
baseline the controller has to beat.

<!-- Served live from the rolling `sim-latest` release, which CI republishes on
     every green mainline push — so this is always the current mainline's result,
     never a stale checked-in copy. `sim/out/` stays gitignored: the artifacts are
     regenerated every build and committing 2 MB per push would bloat history. -->
[![Impulse disturbance response](https://github.com/MikePaNtZ/overboard/releases/download/sim-latest/impulse_open_loop.gif)](https://github.com/MikePaNtZ/overboard/releases/download/sim-latest/impulse_open_loop.mp4)

<sub>▶ [Full 1280×720 clip](https://github.com/MikePaNtZ/overboard/releases/download/sim-latest/impulse_open_loop.mp4) ·
[response plot](https://github.com/MikePaNtZ/overboard/releases/download/sim-latest/impulse_pitch.png) ·
[metrics](https://github.com/MikePaNtZ/overboard/releases/download/sim-latest/impulse_metrics.json) ·
[all artifacts](https://github.com/MikePaNtZ/overboard/releases/tag/sim-latest)</sub>

A driverless onewheel at rest is **stable** — the battery and hub motor put the
centre of mass below the axle. So it is kicked with a 20 N·s impulse, and with
no controller it rolls 4 m, pitches over and noses into the ground at 1.0 m/s.

The board physically cannot exceed **18.6° of pitch** while upright: the bumper
is on the ground before that. That angle is computed from the collision hulls
rather than assumed, and it is the margin the balance controller has to hold.
See [`docs/sim-impulse-response.md`](docs/sim-impulse-response.md).

### What the sim results here do and do not claim

A simulation is only worth publishing if it is honest about its own reach, so
this repo states the boundary rather than leaving it to be inferred:

- Results are from a **model whose constants are hand-specified**, not measured.
  No hardware exists yet. Every absolute figure — torque constant, friction,
  latency — is a placeholder awaiting the Stage-0 bench campaign.
- A **bench-stand** result says nothing about balancing. Pinning the axle
  removes translation, which is the mechanism that makes balancing hard.
- Nothing here has been ridden, and nothing has carried a rider's mass. The
  genuinely unstable equilibrium only appears once the centre of mass sits
  above the axle.

The strongest claim the sim-only phase can make is that *the control path is
correct, timed, signed and instrumented, and the identify → design → verify
method closes against a plant whose truth is known*. It cannot claim that the
balance controller works. Public status is held to the same line (UR-13).

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

## Run the sim

Set up the Python environment once. The pins are exact on purpose — the
scenario is a CI gate whose pass/fail depends on reproducible physics, so the
MuJoCo version is part of the test fixture:

```sh
python3 -m venv .venv
.venv/bin/pip install -r requirements-sim.txt
```

**The acceptance gate** (this is what CI runs on every push; ~2 s, no GL):

```sh
.venv/bin/python -m pytest tests/ -v
```

**Film it** — writes the mp4/webm/gif, poster, plot and metrics to `sim/out/`:

```sh
.venv/bin/python scripts/render_scenario.py
.venv/bin/python scripts/render_scenario.py --impulse 6   # sub-threshold: survives
```

Headless environments need `MUJOCO_GL=osmesa` (software) or `=egl` (GPU). If
offscreen rendering is unavailable the plot and metrics are still written and
the exit status is unchanged — rendering is a publishing concern, not a
correctness one.

**Watch it live** — on macOS this must run under `mjpython`, not plain
`python`:

```sh
.venv/bin/mjpython scripts/view_sim.py
.venv/bin/mjpython scripts/view_sim.py --impulse 0   # undisturbed: it just sits there
```

## Layout

- `crates/board-types` -- shared types (`Command`, `Observation`, `Faults`, `Params`), `no_std`-friendly.
- `crates/hal` -- the `BoardObserve` / `BoardActuate` seam between control-core and a backend (sim or hardware).
- `crates/control-core` -- pure/deterministic control logic (stub: zero command for now).
- `crates/safety` -- pure safety envelope (stub: arm/disarm passthrough).
- `crates/sim-backend` -- `hal` backend (stub today; MuJoCo FFI is the next increment).
- `crates/board-app` -- the binary that wires it all together.
- `sim/models` -- MuJoCo MJCF models (the shared physics asset). The onewheel's
  visual shells are imported from [Openwheel](https://github.com/bytesizedengineering/Openwheel)
  by Byte Sized Engineering (MIT license, see `sim/models/meshes/openwheel/NOTICE.md`).
- `sim/scenarios` -- scripted, deterministic sim experiments. `impulse_response`
  is the first: the disturbance-response test that gates CI.
- `scripts/` -- Python viewer/render entry points for the MuJoCo sim.
- `tests/` -- the sim-in-the-loop acceptance gate (pytest).
- `docs/` -- Markdown+Mermaid mirrors of the design docs (Notion is the source of truth).
- `notebooks/` -- the maths, with the debugging stories. Load archived datasets; see `notebooks/README.md`.

## Licence

MIT — see [`LICENSE`](LICENSE). The vendored Openwheel meshes are MIT under
their own terms; attribution and a documented upstream README/LICENSE
discrepancy are in `sim/models/meshes/openwheel/NOTICE.md`.

**This controls a powered, rideable vehicle.** The warranty disclaimer is not
boilerplate here: the control software is experimental, has never been
validated against hardware, and is published as an engineering record rather
than as something fit to ride.
