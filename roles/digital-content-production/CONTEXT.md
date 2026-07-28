# Digital Content Production — working context

- **Worktree:** `~/projects/overboard-viz` (ADR-0006: one worktree per role, never share a working directory)
- **Registry:** [`docs/decisions/ROLES.md`](../../docs/decisions/ROLES.md)
- **Read first:** [`docs/decisions/INDEX.md`](../../docs/decisions/INDEX.md)

## Current sub-goals
- **Blocking the first public announcement:** produce a clip showing the *weighted* board. The
  whole argument is that mass is the variable; a clip of an empty board does not show it.
  Must be a **Sim Replay** (generated from a real run), not **Concept**.
- Building the sim-run → video pipeline.

## ⚠️ Standing risk for this role
Work was reported uncommitted, unbacked-up, and **on another role's branch**. That is the exact
failure ADR-0006 exists to stop. Get onto your own worktree and branch prefix, and end every
session with a commit.

## Turf notes
- Owns `scripts/render_scenario.py` and `docs/web-artifact-pipeline.md` in this repo.
- Does **not** own the landing page markup or CSS — that is the Senior Digital Marketer's.

## Decisions made (append as you go)

- **The ride the weighted-board sub-goal needs already exists: `sim/scenarios/terrain.py`.**
  It runs a 70 kg ballast with a rider figure by default, so a terrain clip *is* a clip of the
  weighted board, and it is a Sim Replay. Filming it does not need a new scenario.
- **The source tag is a renderer parameter (`--source`), never a literal.** Real telemetry runs
  the same code path with no change to it. Categories stay defined only in the shared vocabulary
  and are not restated in code or in `docs/web-artifact-pipeline.md`.
- **Every render writes a manifest** (`<scenario>_render_manifest.json`) carrying the source
  commit, the scenario parameters, the headline metrics and a sha256 per output file. It is
  deliberately separate from any scenario's `metrics.json`, which belongs to whoever owns that
  scenario. CI fails the job if a manifest names no commit.
- **This role defines its own camera**, in `render_scenario.py`, not in the scenario. The
  `terrain` camera in `build_terrain_model` is a compromise and says so in its own comment.
  A side-on camera that glides along the ride, aimed at the GROUND ahead of the board, reads far
  better than a fixed wide shot: the ground line sweeps through frame at the local slope.
- **`.github/workflows/ci.yml` is not this role's** (it is Senior Controls' under CODEOWNERS).
  Wiring a render into `publish-sim-artifact` needs `TURF-OVERRIDE: <reason>` in a **commit
  message** — CI reads commit messages, not the PR body.
- **`docs/web-artifact-pipeline.md` covers `ci.yml` and `render_scenario.py`** in its
  `<!-- covers: -->` manifest, so touching either without updating that doc fails the `policy`
  gate. Stamp it with `python3 .github/policy_check.py --reconcile docs/web-artifact-pipeline.md`.

## Known dead ends

- **Drawing the grade as a wedge at the true slope angle is unreadable.** 8% is 4.6°, which at
  HUD-panel scale is indistinguishable from flat. Do not exaggerate it to compensate — show the
  number, the direction as a word, a bar against the profile's peak, and a ground-profile strip
  with the board's position on it. The grade then reads without anyone being lied to.
- **A fixed wide shot of the whole crest-to-crest profile does not work**, and it is physics
  rather than framing: the usable grade is single digits, so a 24 m roller is well under a metre
  of relief and flattens toward a texture from far enough back to hold both crests.
- **`terrain.run()` cannot be filmed as it ships** — no `capture_state=`, and `TerrainResult`
  carries no pose history. Do NOT re-run the ride inside the renderer to reconstruct poses: the
  controller, the imperfection profile and the heightfield build all live in that loop and a
  second copy produces a clip of a different run than the metrics beside it. `run_terrain()`
  prefers `capture_state=` when the signature offers it and otherwise records the real run.
  Asked for as a one-line change on Senior Controls' side; delete the recorder when it lands.
- **The 10% comparison does not fail "on the descent"**, whatever the issue text says. Both runs
  drift backwards off the start crest during the 2 s settle; truth pitch recovers and rides on,
  the estimate does not and puts the nose in at 3.29 s, 0.8 m *behind* the start. The scenario's
  own `struck_phase` reports "descent" because its classifier buckets negative travel there.
  Caption it as what it is.

## Vocabulary

Categories are **Footage · Sim Replay · Hardware Replay · Concept**, defined once in
[Shared Vocabulary (canonical)](https://app.notion.com/p/3aa472a5fb6981ebaaa7cf2e996f1e8b). A Replay always names its source — there is no bare
"Replay". **"Lane A / Lane B" is retired**; if you see it anywhere, it is stale.
