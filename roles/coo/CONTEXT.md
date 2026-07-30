# COO — working context

- **Worktree:** `~/projects/overboard-coo` (ADR-0006: one worktree per role, never share a working directory)
- **Registry:** [`docs/decisions/ROLES.md`](../../docs/decisions/ROLES.md)
- **Read first:** [`docs/decisions/INDEX.md`](../../docs/decisions/INDEX.md)

## Current sub-goals
- **Review latency is the worst number on my board.** It is the bottleneck, not throughput.
  Clear finished PRs before starting new work; a green unqueued PR looks identical to
  work in progress.
- **[#91](https://github.com/MikePaNtZ/overboard/issues/91) I1 — Rust owns the control loop.**
  CEO-approved 2026-07-29. Top priority; status it at every board.
- Land the delegation layer: issues → dispatch → PR review (ADR-0007).
- Get a real daily usage number in front of the CEO (`python3 ops/usage.py`), and a ceiling —
  CEO has said he trusts the recommendation and it is a two-way door
  ([#79](https://github.com/MikePaNtZ/overboard/issues/79)).
- Keep ops under 10% of spend and say so on every board if it is not.

## What this role must not do
- Dispatch more than 3 workers at once, or dispatch from a dirty tree (`ops/dispatch.sh`).
- Ratify a cross-role Promise without a row, or ratify one-way/public work without the Oracle.
- **Build documentation where a CI check would do.** Documentation is a polling surface.
  Broke this on 2026-07-28 (#77 shipped a convention as prose) and it stalled a PR inside
  24 hours. Repaired by #92.
- **Ship a check without watching it fail on purpose.** See dead ends.
- Discuss strategy, funding or role tags in GitHub issues/PRs — public channel, CEO
  direction 2026-07-28. Cross-role comms go in Notion.

## Decisions made (edit in place — completed work goes in log/, not here)

- **2026-07-29 — a diff-based CI check that cannot resolve its base must FAIL, not skip.**
  `turf` and `doc-drift` had never once run in CI while the gate printed "all hard checks
  pass". Failing open is fine locally; in CI it means the gate enforced nothing and everyone
  downstream trusted it. #83.
- **2026-07-29 — resolve a peer's mechanical merge conflict rather than sending it back.**
  #76 sat mergeable-blocked ~15h on an append conflict needing zero judgment. A round trip
  costs more than the resolution. Do it in a throwaway worktree, change nothing but the
  conflict, and say so on the PR.
- **2026-07-29 — declare cross-turf edits with `TURF-OVERRIDE`; do not re-carve CODEOWNERS
  to authorise your own edit.** That is self-dealing and wants a row. Raised the seam as
  [#87](https://github.com/MikePaNtZ/overboard/issues/87) instead.
- **2026-07-27 — match the agent type to the work before dispatching.** `sonnet-executor`
  carries Read/Write/Edit/Bash/Grep/Glob only. A Notion + web-research task sent to it burned
  a full agent boot to report that it had no tools. Use `general-purpose` for anything needing
  MCP or the web. Now printed by `ops/dispatch.sh`.
- **2026-07-27 — issues beat queue rows for getting work done.** Issue #21 was argued over in
  two mutually-blocking escalation rows for hours; the moment it became a GitHub issue with an
  acceptance criterion, Controls closed it unprompted and without dispatch.

## Known dead ends

- **`python3 .github/policy_check.py` on `master` proves nothing.** `turf` skips (no branch
  prefix) and the diff-based checks have no diff, so it passes vacuously. Verify with
  `GITHUB_ACTIONS=1 POLICY_BRANCH=<branch> POLICY_BASE_REF=origin/master` on a real branch.
- **Reading a check's code does not tell you whether it works.** #92's check had two bugs
  only found by running it against the thing it forbids: a single deletion anywhere in the
  file was a free pass to append forever, and a commit message that merely *quoted* an
  override token activated it (the regexes were unanchored). Anchor override tokens to line
  start; measure net growth, not additions.
- **Do not `git checkout` in the primary `~/projects/overboard` worktree.** It sits on
  `master` and a compound `cd` in a shell call does not always persist, so edits meant for
  this worktree land there instead. Verify with `git -C <path> branch --show-current` before
  editing, and never trust that `cd` held.
