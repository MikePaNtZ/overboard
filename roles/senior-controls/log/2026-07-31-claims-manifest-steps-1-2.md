# Claims manifest, steps 1-2: the marker/fixture, the generator, one real claim

Issue #61, design `docs/design-claims-manifest.md` (COO, ratified). Implementation half is
steps 1-2: the pytest plumbing that lets a test declare a public claim, and
`scripts/emit_claims.py` that turns a green suite run into `claims.json`. Step 3 (the
`overboard-web` site-side check and `data-claim` attributes) is explicitly not this — it
needs the Senior Digital Marketer's countersign and lives in a different repo.

**`tests/conftest.py`**: `@pytest.mark.claim(id, requirement=None, unit=None, gate=None)`
plus a `record_claim` fixture. Both are required, hard-enforced two ways — a test marked
`claim` that never even requests `record_claim` fails in `pytest_runtest_setup` before the
body runs (no value coming either way); a test that requests it but never calls it fails in
the fixture's teardown (`pytest.fail`, `pytrace=False`, once the test body has finished).
Either way the suite goes red, not a soft skip — the AC is a marked-but-silent claim must
fail, and both paths were verified to actually fail (see below), not just written to.
`pytest_runtest_logreport`/`pytest_sessionfinish` write only claims whose test's "call" phase
reported `passed` to `CLAIMS_RAW_PATH` (default `.claims_raw.json` at repo root, gitignored)
at session end — a failing test's claim is absent from that file, not stale.

**`scripts/emit_claims.py`**: runs `pytest` in a subprocess with `CLAIMS_RAW_PATH` pointed at
a temp file, then folds whatever conftest wrote into the published schema (`schema`, `sha`,
`generated_at`, `claims: {id: {value, unit, requirement, test, gate}}`), writing
`sim/out/claims.json` by default. `sha` resolution mirrors `render_scenario.py`'s
`source_commit()`: prefers `ARTIFACT_SHA`/`GITHUB_SHA` (a PR's `github.sha` is the ephemeral
merge commit, not an ancestor of `master`), falls back to local `git rev-parse`. Exit code
mirrors pytest's, so a chained `&&` step still fails on a red run even though the script
itself always writes a (possibly smaller) manifest rather than crashing.

**One real claim**: `recovery.peak-pitch` on `test_closed_loop_prevents_the_nose_strike`
(`tests/test_closed_loop.py`) — `requirement="SR-SIM-5"`, `gate="peak_abs_pitch_deg < 3.0"`,
recording `r.metrics.peak_abs_pitch_deg` via `record_claim`.

**Assumption flagged, not silently made:** the issue text and design doc both cite this claim
as "the README's 0.2331 m closed-loop peak pitch." That figure does not exist anywhere in
this repo — not in `README.md` (which states no closed-loop peak-pitch number at all today),
not in any test, not in `git log -S` history. It traces to the design doc's own illustrative
backstory (`docs/design-claims-manifest.md`'s "problem" section), whose worked JSON example
also labels the field `unit: "m"` while its own code sample records `peak_abs_pitch_deg` — a
degrees value, so the example was never internally consistent as a literal number to copy.
Fabricating `0.2331`/`"m"` into a real claim would have hardcoded exactly the failure mode
this whole mechanism exists to prevent: a claim value not actually produced by a green test.
Recorded the real, currently-measured value instead (`0.878 deg` at last run), with the
honest unit (`deg`), off the exact test/metric the design doc's own example names
(`test_closed_loop_prevents_the_nose_strike` / `peak_abs_pitch_deg`). `gate` is not in the
issue's literal 3-arg marker spec (`id, requirement=, unit=`) but is in the published schema
(the design doc's JSON example carries it); added it as a fourth optional marker kwarg rather
than leaving the field permanently null.

**CI**: `.github/workflows/ci.yml`'s `publish-sim-artifact` job (already Senior Controls'
turf) gets one new step, "Emit claims manifest" (`python scripts/emit_claims.py`), placed
after the control-ffi build the closed-loop tests need and before the render steps; and
`sim/out/claims.json` was added to the existing `gh release upload sim-latest --clobber`
file list. No second publish mechanism — same job, same rolling release, same pattern the
sim artifact already uses, per the design doc's own instruction not to invent one.

**Deliberate violation, shown failing, not asserted:** temporarily changed the gate's
assertion to an impossible threshold (`< 0.0`) and re-ran `scripts/emit_claims.py`. Full
before/after transcript in the PR body. Confirmed both: (1) the specific claim disappears
from `claims.json` (`"claims": {}`) while 257 other tests still pass, and (2) the run's exit
code goes non-zero, so a real CI step would go red. Also smoke-tested the
"marked-but-never-requests-`record_claim`" path with a throwaway test file outside `tests/`
— fails at setup with the expected message. Reverted the violation; re-ran clean (258 passed,
2 xfailed, same as before touching anything).

**Left out, flagged rather than fixed:** the requirement-register mirror (design doc's
option 1, "follow-up not precondition") — untouched, per the design doc's own sequencing and
this issue's scope (steps 1-2 only). `overboard-web`/`check_page.py`/`data-claim` (step 3) —
untouched, different repo, needs the Senior Digital Marketer.

Verified: `PATH="$HOME/.cargo/bin:/usr/sbin:/sbin:$PATH" .venv/bin/python -m pytest tests/ -q`
→ 258 passed, 2 xfailed (no regressions). `cargo build --release -p control-ffi` built clean
first. `cargo fmt --all -- --check` clean (no Rust files touched). `python3
.github/policy_check.py --who` confirms every edited path (`tests/conftest.py`,
`tests/test_closed_loop.py`, `scripts/emit_claims.py`, `.github/workflows/ci.yml`) is Senior
Controls' turf; `docs/design-claims-manifest.md` (COO's) was read, not edited.

PR #TBD, references #61 (does not close it — step 3 remains open, and #61 is not fully closed
until that lands with the Digital Marketer's countersign).
