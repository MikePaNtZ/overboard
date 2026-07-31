"""I1b (issue #106): Rust-hosted and Python-hosted MuJoCo must be
BIT-IDENTICAL on an open-loop `ctrl` replay -- no controller, no `hal`, no
`sim-backend`.

WHY THIS GATE IS SEPARATE FROM ANY CLOSED-LOOP COMPARISON. An inverted
pendulum with contact is chaotic: a single-tick offset in when `ctrl` is
written relative to `mj_step`, or reading state before rather than after the
step, produces trajectories that agree for about half a second and then
separate without bound. If the only gate were a later closed-loop, tolerance-
based comparison (I1c, #107), a failure there could not distinguish "the
plant wrapper has a one-tick ordering bug" from "the physics legitimately
diverged". This project has already lost days to exactly that confound once
(the estimator 10x discrepancy -- open-loop replay vs closed-loop error, not
two loops disagreeing). So: if THIS test fails, the bug is in
`crates/plant-mujoco`, full stop -- no controls debugging required.

Six seams this pins down, mirrored on both hosts (see
`crates/plant-mujoco/README.md`'s "Ordering contract" and the SAFETY/doc
comments in `crates/plant-mujoco/src/{lib.rs,shim.c,bin/replay.rs}`):

  1. `mj_step` (never the `mj_step1`/`mj_step2` split) on both hosts.
  2. `ctrl` written BEFORE `mj_step`, never within/after it, on both hosts.
  3. State (`qpos`/`qvel`/`time`) read AFTER `mj_step` returns, on both hosts.
  4. Fresh `mjData` (Python's `mujoco.MjData(model)`, Rust's `mj_makeData` via
     `Plant::open`) rather than `mj_resetData` on a reused one -- MuJoCo's own
     `mj_makeData` already sets the initial configuration, so the two are
     equivalent starting points; neither host calls `mj_resetData` here.
  5. `qacc_warmstart` carry-over: neither host calls `mj_forward` (or
     anything else) before the first `mj_step`, so both start that first real
     step from an identical, freshly-zeroed warmstart -- not primed by a
     sensor-populating call the way the CONTROLLED scenarios prime it for
     their controller's first cycle (this replay has no controller to prime).
  6. No substepping/frame-skip on either host: one recorded `ctrl` sample,
     one `mj_step` call.

THE REPLAY BINARY MUST BE BUILT. If it is missing this FAILS, not skips --
see `test_closed_loop.py`'s header for why a green "gate" that quietly did
nothing is worse than no gate.
"""

from __future__ import annotations

import subprocess
import tempfile
from pathlib import Path

import mujoco
import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[1]

MODELS = [
    REPO / "sim" / "models" / "overboard_onewheel.xml",
    REPO / "sim" / "models" / "bench_rig.xml",
]

#: Steps replayed per model. At bench_rig's dt=0.0005s that is 1.0s of sim
#: time; at overboard_onewheel's dt=0.002s that is 4.0s -- both several times
#: past the ~0.5s window issue #106 names as where a one-tick ordering bug's
#: trajectories visibly separate, and both far more than "ten steps".
N_STEPS = 2000

#: Fixed seed so the ctrl sequence is reproducible run to run. It is
#: generated exactly ONCE, here, in Python, and handed to the Rust binary as
#: raw little-endian f64 bytes -- never recomputed on the Rust side -- so a
#: mismatch downstream can only be the two hosts' `mj_step`, never two
#: languages evaluating the same formula slightly differently.
SEED = 106


def _replay_bin_path() -> Path:
    return REPO / "target" / "release" / "plant-replay"


def test_the_replay_binary_is_actually_built():
    """Fails loudly rather than letting the rest of the file skip -- see
    `test_closed_loop.py`'s `test_the_control_library_is_actually_built`."""
    assert _replay_bin_path().exists(), (
        "plant-mujoco's I1b replay binary is not built, so the equivalence "
        "gate would be vacuous. Run:\n"
        "    cargo build --release -p plant-mujoco --bin plant-replay"
    )


def _recorded_ctrl(model: mujoco.MjModel, n_steps: int) -> np.ndarray:
    """A deterministic pseudo-random ctrl sequence spanning each actuator's
    own `ctrlrange` -- "recorded" in the sense the acceptance criterion
    means it: captured once, then replayed identically on both hosts. Large
    excursions are deliberate: this is meant to exercise contact and the
    inverted-pendulum's genuinely nonlinear, chaotic regime, not stay in a
    tame linear one where an ordering bug would be least visible.
    """
    rng = np.random.default_rng(SEED)
    lo = model.actuator_ctrlrange[:, 0]
    hi = model.actuator_ctrlrange[:, 1]
    return rng.uniform(lo, hi, size=(n_steps, model.nu)).astype(np.float64)


def _python_hosted_replay(
    model: mujoco.MjModel, ctrl: np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """The Python-hosted plant, replaying `ctrl` open-loop.

    Mirrors `sim/scenarios/impulse_response.py`'s per-step ordering exactly:
    ctrl written immediately before `mj_step`, state read only once it
    returns. Deliberately does NOT call `mj_forward` before the loop the way
    those scenarios do -- there it exists to prime `sensordata` for a
    CONTROLLER's first cycle, and this replay has no controller. Calling it
    here would perturb `qacc_warmstart` ahead of the first real `mj_step`,
    which is precisely the seam this test exists to pin down, not paper over.
    """
    data = mujoco.MjData(model)  # fresh mjData -- no mj_resetData call
    n_steps = ctrl.shape[0]
    times = np.empty(n_steps, dtype=np.float64)
    qpos = np.empty((n_steps, model.nq), dtype=np.float64)
    qvel = np.empty((n_steps, model.nv), dtype=np.float64)

    for i in range(n_steps):
        data.ctrl[:] = ctrl[i]
        mujoco.mj_step(model, data)  # the whole step -- never mj_step1/mj_step2
        times[i] = data.time
        qpos[i] = data.qpos
        qvel[i] = data.qvel

    return times, qpos, qvel


def _rust_hosted_replay(
    model_path: Path, model: mujoco.MjModel, ctrl: np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """The Rust-hosted plant (`crates/plant-mujoco/src/bin/replay.rs`),
    replaying the identical `ctrl` bytes open-loop.

    Invoked via `cargo run`, not the raw binary path: Cargo adds the native
    library's `OUT_DIR` to `DYLD_LIBRARY_PATH`/`LD_LIBRARY_PATH` for every
    `cargo test`/`cargo run` in the same invocation (see
    `crates/plant-mujoco/build.rs`), which is what makes the linked libmujoco
    resolve at runtime without hand-set env vars. `_replay_bin_path()` above
    already asserts the binary is built, so this `cargo run` only ever
    re-links, never recompiles from scratch.
    """
    with tempfile.TemporaryDirectory() as tmp:
        ctrl_path = Path(tmp) / "ctrl.f64le"
        out_path = Path(tmp) / "out.f64le"
        ctrl_path.write_bytes(ctrl.astype("<f8").tobytes())

        subprocess.run(
            [
                "cargo", "run", "--release", "-q",
                "-p", "plant-mujoco", "--bin", "plant-replay", "--",
                str(model_path), str(ctrl_path), str(out_path),
            ],
            cwd=REPO,
            check=True,
        )

        raw = np.frombuffer(out_path.read_bytes(), dtype="<f8")

    n_steps = ctrl.shape[0]
    stride = 1 + model.nq + model.nv
    assert raw.size == n_steps * stride, (
        f"plant-replay wrote {raw.size} f64s, expected {n_steps * stride} "
        f"({n_steps} steps x (1 time + {model.nq} qpos + {model.nv} qvel))"
    )
    rows = raw.reshape(n_steps, stride)
    times = rows[:, 0].copy()
    qpos = rows[:, 1 : 1 + model.nq].copy()
    qvel = rows[:, 1 + model.nq :].copy()
    return times, qpos, qvel


@pytest.mark.parametrize("model_path", MODELS, ids=lambda p: p.stem)
def test_open_loop_replay_is_bit_identical_across_hosts(model_path: Path):
    model = mujoco.MjModel.from_xml_path(str(model_path))
    ctrl = _recorded_ctrl(model, N_STEPS)

    py_time, py_qpos, py_qvel = _python_hosted_replay(model, ctrl)
    rs_time, rs_qpos, rs_qvel = _rust_hosted_replay(model_path, model, ctrl)

    # Sanity: this must not be a vacuous zero-vs-zero match. Random ctrl
    # driving an actuator against gravity for N_STEPS steps has to move
    # something, on the Python side that is truth for this whole test.
    assert not np.array_equal(py_qpos[0], py_qpos[-1]), (
        "the trajectory never moved -- this run proves nothing"
    )

    # BIT-IDENTICAL, not tolerance-bounded -- the whole point of I1b.
    assert np.array_equal(rs_time, py_time), (
        f"time diverged, first mismatch at step "
        f"{int(np.argmax(rs_time != py_time))}"
    )
    assert np.array_equal(rs_qpos, py_qpos), (
        "qpos diverged at step "
        f"{int(np.argmax(np.any(rs_qpos != py_qpos, axis=1)))} of {N_STEPS}"
    )
    assert np.array_equal(rs_qvel, py_qvel), (
        "qvel diverged at step "
        f"{int(np.argmax(np.any(rs_qvel != py_qvel, axis=1)))} of {N_STEPS}"
    )
