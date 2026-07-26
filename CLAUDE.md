# Balance Board — project rules

Extends the global `~/.claude/CLAUDE.md`. Project: a DIY **lean-to-steer self-balancing board** (rideable inverted pendulum) — Rust real-time control on PREEMPT_RT Linux, off-the-shelf smart drive (torque mode) + hoverboard hub motors, sim-first. A deferred companion **PX4 drone** lives in its own Notion doc. Full context: memory `[[balance-board-project]]`.

## Repo boundary
- **This repo is controls only** — Rust, sim, hardware, design docs. The public landing page and all
  brand/marketing assets live in the sibling repo **`overboard-web`** (`~/projects/overboard-web`).
  Keep them separate: no HTML/marketing here, no control code there.
- The two are coupled by **facts, not code**. When a capability ships or a phase turns over, the
  `overboard-web` page status must be updated in the same pass (Requirements `SR-WEB-4`, and the
  lock-step rule in [M0](https://app.notion.com/p/3a8472a5fb6981ffbf73ee8297e62f07)).
- Local multi-repo work: open `~/projects/overboard.code-workspace` to get both folders at once.

## Git workflow — feature branches + PR, CI is the gate (HARD)
- **Never commit to `master`.** All work goes on a feature branch (`feat/…`, `fix/…`, `docs/…`)
  and lands via PR. `master` is protected: linear history, no force-push, no deletion.
- **CI success is the merge gate.** `rust` and `sim` are required status checks and the branch
  must be up to date with `master` before merging. A red build cannot be merged.
- Required approvals are set to **0 deliberately** — GitHub forbids approving your own PR, so on a
  solo repo any non-zero count would permanently block every merge. Mike still reviews; CI is the
  hard gate. Add approvals if a second contributor ever joins.
- `publish-sim-artifact` is deliberately **not** a required check: it only runs on push to `master`,
  so requiring it would hang every PR forever waiting for a check that never reports.
- `enforce_admins` is off, so Mike can break glass in an emergency. Claude must not.
- Open the PR with a body that states what changed and *why*, and call out any acceptance criteria
  that moved. Wait for CI, then merge (squash) — do not merge red or bypass protection.

## Public artifacts
- Both repos are **public**. CI publishes the sim render + metrics to the rolling `sim-latest`
  release on every green `master` push; that URL is the single source for the README embed, the
  Notion design doc video, and (later) the landing page. Never check binaries into git —
  `sim/out/` is gitignored and the artifacts are regenerated every build.

## Docs & source of truth
- **Notion is the PRIMARY home** for vision, design, and project/roadmap docs. The repo `docs/` holds **Markdown+Mermaid mirrors that track the implementation**; Notion may drift during heavy dev. Run a periodic cleanup pass to reconcile Notion vision/early-design with what shipped.
- **No production code** until the design-doc set passes the ready-to-code gate (numeric acceptance criteria — see D0).

## Notion design-doc iteration loop
1. Draft/update the doc in Notion (Notion-flavored Markdown; Mermaid for diagrams).
2. Mike reviews, leaves inline comments.
3. Claude reads comments (`notion-get-comments`), addresses each (revise the doc; reply/resolve via `notion-create-comment`), re-publishes.
4. Repeat until Mike approves → mirror to `docs/` → tag.
Keep docs clean: prune resolved threads; keep the roadmap synced to reality.

## Model routing (project override of global)
- **Oracle = `opus5-oracle` (Opus 5), NOT fable-oracle.** Escalate judgment here (epic scoping/sequencing, acceptance criteria, "right problem?" gate, thorny architecture/root-cause/adjudication, adversarial pre-commit review). Distill first; one call; adjudicate-not-author; read-only.
- **Opus 4.8 drives; Sonnet executes** well-scoped work (`sonnet-executor`; `general-purpose`/`Explore` at `model: sonnet`). Opus reviews every hand-back.
- **Effort is a lever:** `low`/`medium` for Sonnet + simple driver tasks, `high`/`xhigh` for oracle-grade judgment; prefer lowering effort over adding scaffolding. Opus-5 self-verifies — don't add verify passes; cap delegation and constrain scope on narrow tasks. (See global CLAUDE.md → Opus-5-era refinements, and `[[context-engineering-anthropic]]`.)

## Agent skills (Addy Osmani suite)
Use the `agent-skills:*` suite. Judgment steps (`idea-refine`/`spec`, `plan`, `doubt-driven-development`, `code-review`, `debugging-and-error-recovery`) escalate to `opus5-oracle`; execution steps (`build`, `incremental-implementation`, `test`, `code-simplify`) delegate to Sonnet; Opus orchestrates.

## Engineering conventions
- **Sim-in-the-loop from day one, TDD-style:** every change runs against a sim; build meaningful automated integration tests + Rust unit tests from the start. A rough visual 3D sim POC comes early (full UE5 later).
- **One log schema, common standard** (MCAP + Foxglove) streamable to AWS for real-time monitoring + post-processing/tuning.
- **Safety:** hardware deadman in series with motor power; rider-in-loop only after sim + ballasted-dummy bench gates pass; AI never in a real-time / ridden loop.
- **Sizing:** bench rig sized to the rideable board from the start.
- **Steering from day one:** riderless build is differential-steerable via a gamepad (Xbox/PS5) → desired velocity + turn-rate setpoints.
