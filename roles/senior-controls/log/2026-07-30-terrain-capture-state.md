# Give `terrain.run()` a `capture_state` pose history, mirroring `impulse_response`

Issue #81, filed by Digital Content Production from #69: `impulse_response.run()` takes
`capture_state: bool = False` and appends `data.qpos.copy()` per step into `ImpulseResult.qpos`, so
`render_scenario.py` can replay the exact trajectory the metrics describe rather than re-stepping the
physics inside the renderer. `terrain.run()` had neither the flag nor the field, so a terrain ride
could not be filmed the same way.

**Mirrored the impulse signature exactly**, per the ask: `run(..., capture_state: bool = False)`
appending to a new `TerrainResult.qpos`, same field name and shape (`np.asarray(states) if states
else np.empty(0)`). Off by default — the append only happens inside `if capture_state:`, at the same
point in the per-step loop where the other trajectory channels are recorded, so a pose is captured for
every sample the metrics use and never for any other step (the loop can exit early on a strike or on
reaching the crest; the capture point is on that same path, so the count can never drift from `len(t)`).
`to_json_dict()` was already untouched — like impulse's, it only serialises `params`/`plant`/`metrics`,
so the pose array was never at risk of landing in metrics.json.

Added `test_capture_state_records_one_pose_per_sample` to `tests/test_terrain.py`: asserts `qpos` is
empty when `capture_state=False` and that its row count equals `len(t)` when `True`. Existing
`test_repeat_runs_are_bit_identical` (which never passes `capture_state`) is unchanged and still
passes, pinning that the default path is untouched.

`scripts/render_scenario.py` already inspects `terrain.run`'s signature and prefers `capture_state=`
the moment it exists, falling back to its own `mj_step`-wrapping shim otherwise (that shim is marked
for deletion in its own docstring). Per the issue, landing this closes that gap automatically —
nothing in the renderer needed to change, and per the issue text ("nothing else in the renderer
changes") I left `scripts/render_scenario.py` untouched, including the now-dead shim itself; deleting
it is a separate, renderer-owned cleanup outside this issue's ask.

Verified: `PATH="/usr/sbin:$PATH" .venv/bin/python3 -m pytest tests/ -q` → 258 passed, 2 xfailed (no
regressions). `cargo build --release -p control-ffi` built clean first, per the closed-loop test
dependency. `GITHUB_ACTIONS=1 POLICY_BRANCH=feat/controls/terrain-capture-state
POLICY_BASE_REF=origin/master python3 .github/policy_check.py` passes (8 roles, 38 ownership rules;
only pre-existing doc-drift advisories, unrelated to this change).

PR #TBD, closes #81.
