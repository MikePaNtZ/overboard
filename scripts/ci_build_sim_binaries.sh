#!/usr/bin/env bash
#
# Build every Rust artifact `pytest tests/` needs. THE one place to add a new
# one (issue #124).
#
# Why this file exists rather than the steps living in the workflow: two CI
# jobs run the suite -- `sim` directly, and `publish-sim-artifact` indirectly,
# because `scripts/emit_claims.py` shells out to pytest to build the claims
# manifest. Both therefore need every binary the suite loads, and both used to
# maintain that list by hand.
#
# They drifted three times in one day, always the same way -- `sim` got the new
# build step and `publish-sim-artifact` did not:
#
#   #117 (I1b)  plant-replay             -> publish-sim-artifact failed
#   #120 (I1c)  impulse-response-rust    -> failed again, on master, three
#                                           consecutive pushes, unnoticed
#   #123        (the fix for the above)  -> patched the symptom
#
# It went unnoticed because `publish-sim-artifact` is deliberately NOT a
# required status check -- correctly, since it renders through an offscreen GL
# stack that must never block a merge whose physics passed. But a job that can
# fail indefinitely without interrupting anyone is a job whose failures are
# free. Patching the fourth instance is not the fix; having one list is.
#
# Every binary below is loaded or shelled out to by a test that FAILS rather
# than skips when it is missing. That rule is deliberate: a closed-loop test
# passing because the controller was absent is worse than no test at all.

set -euo pipefail

# --release throughout: the tests look for artifacts in target/release, and a
# debug plant is slow enough to change the suite's wall time materially.

# The real control law over the C ABI. Loaded by the closed-loop gates via
# ctypes (`sim/scenarios/rust_controller.py`), and by the render's --compare
# pane, which would otherwise film an "open vs closed" clip whose closed pane
# was quietly uncontrolled.
cargo build --release -p control-ffi

# I1b (#106): tests/test_rust_python_plant_replay_equivalence.py shells out to
# this to compare the Rust-hosted plant against the Python-hosted one.
cargo build --release -p plant-mujoco --bin plant-replay

# I1c (#107, AC6): tests/test_rust_hosted_impulse_response.py shells out to
# this to compare the Rust-hosted closed loop (hal + sim-backend's real plant)
# against the Python-hosted one.
cargo build --release -p board-app-driverless --bin impulse-response-rust

# Fail HERE, with a readable message, rather than 40 minutes later as an
# obscure pytest error about a missing file. A renamed binary is the other way
# this list goes stale, and `cargo build` succeeding says nothing about the
# artifact landing under the name the tests look for.
missing=()
for bin in libcontrol_ffi plant-replay impulse-response-rust; do
  case "$bin" in
    libcontrol_ffi)
      # cdylib extension is platform-dependent; the tests resolve it the same
      # way (`sim/scenarios/rust_controller.py:library_path`).
      compgen -G "target/release/${bin}.*" > /dev/null || missing+=("$bin")
      ;;
    *)
      [[ -x "target/release/${bin}" ]] || missing+=("$bin")
      ;;
  esac
done

if (( ${#missing[@]} )); then
  echo "ci_build_sim_binaries.sh: built without error, but these artifacts are" >&2
  echo "not where the tests look for them: ${missing[*]}" >&2
  echo "A binary was probably renamed. Update this script AND the test that" >&2
  echo "shells out to it -- they are the same list in two places by necessity." >&2
  exit 1
fi

echo "ci_build_sim_binaries.sh: all sim test artifacts present in target/release"
