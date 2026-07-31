# plant-mujoco

I1a (issue #91): proves Rust can call `mj_step` at all.

A ~60-line C shim over MuJoCo's C API (`src/shim.c`), compiled by `build.rs`
with the `cc` crate against MuJoCo's own headers. `Plant`'s `extern "C"`
surface binds only to the shim's opaque-handle functions (`*mut c_void` for
both `mjModel*` and `mjData*`) -- there are no hand-written `#[repr(C)]`
mirrors of either struct, on purpose: those layouts drift between MuJoCo
minor versions, and a mismatch would be silent memory corruption rather than
a build error.

## Linking

`build.rs` discovers libmujoco via a single `MUJOCO_DIR` env var, with a
wheel-probe fallback (`python3 -c 'import mujoco'`) if it is unset. Either
way it links the **same** `libmujoco.so.3.10.0` the pip-installed `mujoco`
wheel ships and the `sim` CI job already installs from
`requirements-sim.txt` -- not a system package, not a vendored copy. That is
what makes the later Rust-vs-Python plant equivalence check (I1b, #106)
meaningful: both sides load the identical shared object.

The build fails loudly, not silently, if no MuJoCo install is found.

## What this crate does NOT do

No `hal` implementation, no comparison against the Python-hosted plant, no
control-law change. `sim-backend` stays a stub until I1c (#107). See issue
#91 for the full scope split.

## Version check

`Plant::open` asserts the linked `mj_versionString()` equals
`plant_mujoco::REQUIRED_VERSION` ("3.10.0") -- the string, not
`mj_version()`'s packed integer, which is ambiguous between e.g. `3.1.0` and
`3.10.0`.
