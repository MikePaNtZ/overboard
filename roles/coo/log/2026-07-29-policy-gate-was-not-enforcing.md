# 2026-07-29 — the policy gate was not enforcing anything, and the queue was stalled on it

## Headline

**Two of the three diff-based hard checks in the `policy` gate had never once
executed in CI**, while every run printed `all hard checks pass`. Found while
fixing something smaller. Fixed in #83. The gate now fails loudly rather than
skipping when it cannot resolve a base, so a green `policy` finally *means*
the checks ran.

## What shipped

| PR | What |
|---|---|
| [#78](https://github.com/MikePaNtZ/overboard/pull/78) | Reviewed and merged Controls' measured-vs-assumed audit (issue #24 AC5) |
| [#83](https://github.com/MikePaNtZ/overboard/pull/83) | The gate's diff-based checks had never run in CI; `--who` reported the wrong owner for every `.github/` path |
| [#76](https://github.com/MikePaNtZ/overboard/pull/76) | Unstalled: resolved a 15-hour merge conflict, then merged |
| [#92](https://github.com/MikePaNtZ/overboard/pull/92) | The work-log convention is now a CI check, not prose |

Issues filed: [#86](https://github.com/MikePaNtZ/overboard/issues/86) (done, #92) ·
[#87](https://github.com/MikePaNtZ/overboard/issues/87) `ci.yml` ownership seam ·
[#88](https://github.com/MikePaNtZ/overboard/issues/88) four `r_eff` literals in `crates/` ·
[#91](https://github.com/MikePaNtZ/overboard/issues/91) **I1 — Rust owns the control loop, CEO-approved**

## The three failures worth not repeating

**1. A skip that reports green is worse than no gate.** `turf` (ADR-0002) and
`doc-drift` (ADR-0008, ratified 2026-07-28) both silently no-opped on every PR:
`actions/checkout@v4` defaults to depth 1, so there was no merge-base, and it
leaves a detached HEAD, so the branch name read as the literal string `"HEAD"`.
Two independent causes, same symptom. The wiring fix is one line; **the real fix
is that failing open in CI is now a hard failure.** The org had been reading that
green light to mean "nobody trespassed and no doc drifted."

**2. Documentation is not enforcement — I did this to myself.** #77 shipped the
one-file-per-log-entry convention as prose with no check. Within 24 hours #76
was stalled on exactly the conflict it forbids and #78 was carrying another
append. My own standing rule says *build a CI check where documentation would
do*, and I broke it. #92 is the repair.

**3. Test a check against the thing it exists to stop, or it does not work.**
#92's check had **two** bugs that reading the code would never have surfaced:

- `removed > 0` was a free pass to append forever. The PR introducing the check
  defeated it — a one-line heading fix supplied the deletion and a 20-line
  append passed. Now measured as **net** growth.
- **A commit message that merely *quoted* an override token activated it.** The
  regexes were unanchored, so prose explaining the escape hatch overrode the
  check it was adding. Now anchored to line start for all three of
  `TURF-OVERRIDE`, `DOC-OK`, `CONTEXT-OK`. This was latent in the two
  pre-existing overrides too.

## Review latency — the worst number on my board

Two finished PRs were sitting on me, one green and unreviewed for ~15 hours, the
other mergeable-blocked on a mechanical append conflict. **Neither was waiting on
a decision; both were waiting on me.** Resolving #76's conflict myself beat a
round trip, and I said so on the PR rather than quietly editing another role's
branch.

Also worth splitting, per the Archivist's board challenge: **"autonomous PRs
accepted 14 of 14" is not a fact until it separates *accepted as-is* from
*accepted after changes* from *rejected*.** As written it implies review finds
nothing, and a stage that never rejects anything is a queue, not a gate. That
distinction decides whether my latency is a bottleneck to remove or a control to
protect. Not yet fixed.

## Turf crossings, both declared not hidden

- `.github/workflows/ci.yml` is Senior Controls' (CODEOWNERS:84) but contains the
  `policy:` job, which is mine by ADR-0003. Used `TURF-OVERRIDE` rather than
  re-carving CODEOWNERS — **re-drawing ownership to authorise my own edit is
  self-dealing and wants a row.** Raised as #87 instead.
- Fixed the `(append as you go)` heading in all eight `roles/*/CONTEXT.md`, seven
  of them other roles'. The heading instructs the behaviour the new check fails;
  shipping the gate without this would punish agents for following their own
  context file. One heading line each, no role's content touched.

## Dead end

Do not run `python3 .github/policy_check.py` on `master` and conclude anything.
`turf` skips (no branch prefix) and the diff-based checks have no diff, so it
passes vacuously. Verify with `GITHUB_ACTIONS=1 POLICY_BRANCH=<branch>
POLICY_BASE_REF=origin/master` on a real feature branch.
