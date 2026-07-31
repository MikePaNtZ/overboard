# plant-mujoco

I1a (issue #91): proves Rust can call `mj_step` at all. I1b (issue #106)
extends it with a `ctrl`-in / `qpos`+`qvel`-out surface and proves the
Rust-hosted and Python-hosted plants are bit-identical on an open-loop
replay -- see "Ordering contract" below.

A small C shim over MuJoCo's C API (`src/shim.c`), compiled by `build.rs`
with the `cc` crate against MuJoCo's own headers. `Plant`'s `extern "C"`
surface binds only to the shim's opaque-handle functions (`*mut c_void` for
both `mjModel*` and `mjData*`) -- there are no hand-written `#[repr(C)]`
mirrors of either struct, on purpose: those layouts drift between MuJoCo
minor versions, and a mismatch would be silent memory corruption rather than
a build error.

## Linking

`build.rs` discovers libmujoco via a single `MUJOCO_DIR` env var, with a
wheel-probe fallback if it is unset. The probe asks each of these, in order,
and takes the first that can `import mujoco`:

1. `python3`, then `python`, as found on `PATH` — this is what CI hits, since
   it `pip install`s the wheel into the runner's system Python.
2. `<repo>/.venv/bin/python3`, then `<repo>/.venv/bin/python` — **this is what
   a developer machine hits**, because the wheel goes into the repo `.venv`
   that every other tool here uses, while `python3` on `PATH` is a Homebrew or
   system interpreter that has never heard of MuJoCo.

`PATH` is probed first, so the `.venv` entries are strictly additive and
cannot change which libmujoco CI links against.

Either way it links the **same** `libmujoco.so.3.10.0` (or, on macOS,
`libmujoco.3.10.0.dylib`) the pip-installed `mujoco` wheel ships and the `sim`
CI job already installs from `requirements-sim.txt` -- not a system package,
not a vendored copy. That is what makes the Rust-vs-Python plant equivalence
check below meaningful: both sides load the identical shared object.

The build fails loudly, not silently, if no MuJoCo install is found, and the
panic lists every interpreter it probed.

### macOS: what issue #112 actually was

**Resolved. `cargo test -p plant-mujoco` passes on macOS from a clean checkout
with only `pip install -r requirements-sim.txt` -- no `MUJOCO_DIR`, no
`DYLD_*`, no hand-created symlinks.**

#112 filed this as a *runtime* defect: the wheel's `.dylib` carries a
framework-style install name that does not match its own filename, so the
loader could not resolve it. The install name mismatch is real -- `otool -D`
reports `@rpath/mujoco.framework/Versions/A/libmujoco.3.10.0.dylib` for a file
actually named `libmujoco.3.10.0.dylib` -- but it is **already handled**, and
was before #112 was written. `build.rs` symlinks the library into `OUT_DIR`
under **both** its unversioned and its real versioned name; the versioned one
is what `DYLD_FALLBACK_LIBRARY_PATH` matches on, by leaf filename, when the
`@rpath` lookup comes up short. That is why the versioned symlink exists and
why removing it would look like an unrelated cleanup.

What actually failed on macOS was one step earlier and had nothing to do with
dylibs: **the build never got far enough to link.** The wheel-probe only asked
the `PATH` interpreters, so on any machine using the repo `.venv` it panicked
with "no MuJoCo install found" *despite* `requirements-sim.txt` having been
installed exactly as documented -- which reads as a broken crate rather than
an unactivated virtualenv. The `.venv` probe entries above are the fix.

## Ordering contract (I1b, issue #106)

This is the canonical repo-local record of the seams a Rust-hosted vs
Python-hosted MuJoCo comparison is sensitive to. **The canonical ICD entry
lives in Notion** and needs this same list mirrored into it by the COO --
this crate cannot reach Notion, so it is not attempted here. Both hosts
(`src/bin/replay.rs` here; `tests/test_rust_python_plant_replay_equivalence.py`'s
`_python_hosted_replay`, mirroring `sim/scenarios/impulse_response.py` and
its siblings) make the SAME choice on every one of these:

| Seam | Choice both hosts make |
|---|---|
| `mj_step` vs `mj_step1`/`mj_step2` split | Always the single full `mj_step` call. Neither host ever splits it. |
| `ctrl` written before vs within the step | Always written immediately BEFORE `mj_step`, never mid-step. |
| State read before vs after the step | `qpos`/`qvel`/`time` are always read AFTER `mj_step` returns, never before. |
| `mj_resetData` vs fresh `mj_makeData` | Both hosts use a freshly allocated `mjData` (`Plant::open`'s `mj_makeData`; Python's `mujoco.MjData(model)`) and never call `mj_resetData`. `mj_makeData`'s own header comment ("If the model buffer is unallocated the initial configuration will not be set") implies it already sets the initial configuration when the model buffer IS allocated -- i.e. it is already equivalent to a reset, so there is nothing to reconcile between the two mechanisms here. |
| `qacc_warmstart` carry-over | Neither host calls `mj_forward` (or anything else) before the first `mj_step`, so both start it from an identical, freshly-zeroed warmstart. The CONTROLLED scenarios (`impulse_response.py` etc.) call `mj_forward` once before their loop to prime `sensordata` for a controller's first cycle -- this open-loop replay has no controller and deliberately skips that call rather than adding an extra state-perturbing step neither host's "real" usage agrees on. |
| Python-side substepping / frame-skip | None. One recorded `ctrl` sample maps to exactly one `mj_step` call, on both hosts. |

`tests/test_rust_python_plant_replay_equivalence.py` is the test that holds all six of these to
account: it replays a recorded `ctrl` sequence (2000 steps, generated once in
Python with a fixed seed, handed to the Rust binary as raw bytes so no
transcendental function is ever computed twice by two languages) through
both hosts on every scenario model (`sim/models/overboard_onewheel.xml` and
`sim/models/bench_rig.xml`) and asserts `qpos`, `qvel` and `time` are
bit-identical (`numpy.array_equal`, not a tolerance) at every step.

## What this crate does NOT do

No `hal` implementation, no controller, no control-law change -- `sim-backend`
implements `hal` against this crate's `Plant` (I1c, #107); see that crate's
own header for the seams that carry over from here (AC8 in particular: the
`qacc_warmstart` row above applies unmodified to an open-loop replay, but a
Rust host driving a real controller must make the pre-loop `mj_forward` call
the CONTROLLED scenarios make, which this crate does not do on its own). See
issues #91, #106 and #107 for the full scope split.

## Version check

`Plant::open` asserts the linked `mj_versionString()` equals
`plant_mujoco::REQUIRED_VERSION` ("3.10.0") -- the string, not
`mj_version()`'s packed integer, which is ambiguous between e.g. `3.1.0` and
`3.10.0`.
