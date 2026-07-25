# Balance Board — project rules

Extends the global `~/.claude/CLAUDE.md`. Project: a DIY **lean-to-steer self-balancing board** (rideable inverted pendulum) — Rust real-time control on PREEMPT_RT Linux, off-the-shelf smart drive (torque mode) + hoverboard hub motors, sim-first. A deferred companion **PX4 drone** lives in its own Notion doc. Full context: memory `[[balance-board-project]]`.

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
