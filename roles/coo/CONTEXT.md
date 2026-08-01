# COO — working context

- **Worktree:** `~/projects/overboard-coo` (ADR-0006: one worktree per role, never share a working directory)
- **Registry:** [`docs/decisions/ROLES.md`](../../docs/decisions/ROLES.md)
- **Read first:** [`docs/decisions/INDEX.md`](../../docs/decisions/INDEX.md)

## Current sub-goals

**Everything the previous list named is closed.** SR-SIM-5 satisfied (#91→#107), the delegation
layer landed and proved (#38), and the usage gate is live and verified to refuse. Deleting them
here in the same pass, per Mechanical's rule — a stale priority list is read as the current one
by the next session.

- **[#33](https://github.com/MikePaNtZ/overboard/issues/33) sim-fidelity session.** Prep is
  WRITTEN (`docs/sim-fidelity-session-prep.md`, #143) — this is now genuinely waiting on the
  CEO's time, which it was not before. Mechanical asked to attend (#128) and should.
- **[#151](https://github.com/MikePaNtZ/overboard/issues/151) cron decision.** Deliberately NOT
  taken as a consequence of the gate going live. The number is a floor; cron runs exactly the
  agents it cannot see. CEO's call.
- **Two doc manifests are becoming decorative.** `docs/design-claims-manifest.md` and
  `docs/web-artifact-pipeline.md` both declare `covers: .github/workflows/ci.yml`, so every CI
  change trips both and gets a routine `DOC-OK`. Four times today. Narrow them to what they
  actually describe, or drop them. Both are mine.
- Keep ops under 10% of spend and say so on every board if it is not. **Now measurable:** COO is
  21.3% of attributed spend, which is not the same statistic and should not be reported as if
  it were.

## What this role must not do
- Dispatch more than 3 workers at once, or dispatch from a dirty tree (`ops/dispatch.sh`).
- Ratify a cross-role Promise without a row, or ratify one-way/public work without the Oracle.
- **Build documentation where a CI check would do.** Documentation is a polling surface.
  Broke this on 2026-07-28 (#77 shipped a convention as prose) and it stalled a PR inside
  24 hours. Repaired by #92.
- **Ship a check without watching it fail on purpose.** See dead ends.
- **Add a gate without asking what it costs the person who did nothing wrong.** Four of mine
  landed on other roles before they landed on me — the role-log heuristic blocked DCP and
  Mechanical with a 0% true-positive rate, and a `covers:` manifest on a session-prep doc
  red-built Mechanical within the hour. "Is this check correct?" is the wrong first question.
- **Assert someone broke a convention without reading what they wrote.** Did this to DCP on
  #105. They had not. A gate asserting fault is bad; its owner repeating the assertion
  unchecked is how people stop trusting the tooling.
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

- **2026-07-31 — a diff-based check that cannot resolve its base must FAIL in CI, not skip.**
  `turf` and `doc-drift` had never once executed. A skip that reports green is a lie the whole
  org builds on. #83.
- **2026-07-31 — calibrate from the DELTA of two readings, never one absolute reading.** The
  trailing-7-day window and the weekly meter are different windows; a single reading taken after
  a reset implies a ceiling ~14x too large. #127.
- **2026-07-31 — when two TURF-OVERRIDEs land on the same file in a row, the map is wrong, not
  the author.** Fixed by moving the file to its real owner (#136), not by more paperwork.

## Known dead ends

- **`python3 .github/policy_check.py` on `master` proves nothing.** `turf` skips (no branch
  prefix) and the diff-based checks have no diff, so it passes vacuously. Verify with
  `GITHUB_ACTIONS=1 POLICY_BRANCH=<branch> POLICY_BASE_REF=origin/master` on a real branch.
- **Reading a check's code does not tell you whether it works.** #92's check had two bugs
  only found by running it against the thing it forbids: a single deletion anywhere in the
  file was a free pass to append forever, and a commit message that merely *quoted* an
  override token activated it (the regexes were unanchored). Anchor override tokens to line
  start; measure net growth, not additions.
- **`git reset --hard` discards uncommitted work in the SAME worktree.** Lost a fix mid-test on
  2026-07-31 and briefly misread the result as a code failure.
- **The Overboard guard denies the WHOLE bash call.** A blocked `cp` chained after a real patch
  meant the patch never ran — and a later syntax check passing was misread as confirmation.
  Never chain a possibly-blocked command after real work.
- **`gh pr create --body "..."` mangles backticked shell metacharacters.** `kt`, `0.2/p`,
  `p99.9`, `docs/` all got evaluated; four PR bodies needed repair. Write the body to a file and
  use `--body-file`.
- **Do not `git checkout` in the primary `~/projects/overboard` worktree.** It sits on
  `master` and a compound `cd` in a shell call does not always persist, so edits meant for
  this worktree land there instead. Verify with `git -C <path> branch --show-current` before
  editing, and never trust that `cd` held.
