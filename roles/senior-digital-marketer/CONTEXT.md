# Senior Digital Marketer — working context

- **Worktree:** `~/projects/overboard-web-sdm` (ADR-0006: one worktree per role, never share a working directory)
- **Registry:** [`docs/decisions/ROLES.md`](../../docs/decisions/ROLES.md)
- **Read first:** [`docs/decisions/INDEX.md`](../../docs/decisions/INDEX.md)

## Status: Ratified (ROLES.md, 2026-07-31)
Registered 2026-07-26 after filing under the CMO seat with a title prefix — every row filed
before that was misattributed and has been repaired. Ratified 2026-07-31 once `overboard-web`
had a CODEOWNERS to point at (overboard-web#35); prefix `feat/web/`, escalates to the CMO.
Owns nothing in the `overboard` repo except `roles/senior-digital-marketer/**`; the surface is
`overboard-web`.

> ## 🛑 THE LAUNCH IS HELD — [ADR-0011](../../docs/decisions/ADR-0011-hold-the-launch-until-the-board-stops-flipping.md)
>
> - **2026-08-03 is dead and there is no new date.** The hold is gated on a control fix, not a
>   calendar. Do not set a date, do not let one be inferred, do not publish.
> - **"The board never became unstable at any aggression level tested" is WITHDRAWN** — false,
>   and not to be reinstated in any softened form. The measurements behind it were taken through
>   a harness delivering stick at 7–13 Hz against a 100 ms staleness cutoff, so the board was
>   commanded at ~0.62 of the lean the tests believed. **They may not be cited.**
> - **On this surface I am the safety net.** The controls repo's `policy` gate registers the
>   claim but does not gate `overboard-web`. `check_page.py` rule 6c does — see the log entry
>   for 2026-08-02. Narrow it when the hold lifts; do not delete it.

## Current sub-goals
- The L1 announcement is drafted and deliberately held on two engineering conditions, plus the
  ADR-0011 hold. It does not go out on a date — it goes out on the exit criteria.
- **The `#now` heading is settled — do not reopen it.** See "Decisions made" below. It moves when
  the frontier moves, not because of the hold.

## Rules
- Lead with the measured surprise, not the ambition. Tag every link so we can tell which
  community sent people. End the post with an ask — the GitHub click-through is the conversion.
- **I never clear my own page copy.** CMO reviews SDM copy (CEO ruling 2026-07-31); never queue
  own prose with `--auto`. CI scripts and this file are not copy.

## Decisions made (edit in place — completed work goes in log/, not here)

- **2026-08-02 — the `#now` heading stays as written. CEO ruling, "audit is fine."** I raised
  that "Right now: it stays up, and it goes where it's told" might fall under ADR-0011's broader
  bar on asserting balance-controller stability, even though it is not the withdrawn claim and
  the 08-02 audit cleared it. **The CEO ruled the audit's scope is correct.** The heading is not
  a hold problem. It still moves under its own standing rule — it names the *current* failure
  mode and gets rewritten when the frontier moves — so it will be re-based when the control fix
  lands, as ordinary lock-step, not as claim remediation. Do not re-litigate this against the
  hold; it was asked and answered.
- **2026-08-02 — did not touch `feat/marketing/board-0801-relaunch`.** ADR-0011 assigned it to
  me, but it is the CMO's branch (`feat/marketing/`, `roles/cmo/**`, checked out in
  `~/projects/overboard-cmo`), it **merged as #164 on 08-01**, and the dead date it carried was
  already defused on `master` by the COO's `TURF-OVERRIDE` banner. Nothing to strip; editing it
  would have been Turf on a file another role had handled.
- **2026-08-02 — gated the claim in CI rather than trusting the ADR to be read** (overboard-web
  #55). ADR-0011 itself says the date "binds only sessions that read it"; that is precisely the
  failure a check does not have.

## Known dead ends

- **`git log master..<branch>` does not tell you whether a branch merged.** Squash-merge rewrites
  the SHA, so a fully-merged branch still lists its commits as absent from master and looks live.
  Check `gh pr list --head <branch> --state all`, or diff against `origin/master`, before
  concluding there is work outstanding. This nearly sent me editing another role's merged branch.
