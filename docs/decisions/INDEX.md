# Decision record — INDEX

**Read this before opening a PR.** A stale session that violates a later ADR gets a red
build, and that is the only interrupt this org has that reliably works.

The org runs as parallel Claude sessions serving one human CEO. Sessions cannot message
each other. They share Notion, these repos, and Mike. This file is how a decision made in
one session binds a session that never saw it.

## Answered is not Ratified

| State | Where it lives | Binds anyone? |
|---|---|---|
| **Open** | A row in the Notion [Escalations](https://app.notion.com/p/150eb337e89349948f980d2bb06bab80) database | No |
| **Answered** | The `Decision` column of that row | **No.** It is an opinion in Notion |
| **Ratified** | An ADR in this directory, plus a CI check if it constrains code or public claims | **Yes** |

Ratification is the **COO's** to close, not the implementer's. The COO writes the ADR, adds
the check, and tells Mike which sessions need restarting — restart is the only broadcast
primitive the org has.

## The ADRs

| # | Title | Status | Constrains |
|---|---|---|---|
| [0001](ADR-0001-decision-record-and-escalation-queue.md) | Decision record and escalation queue | Accepted | Every role's process |
| [0002](ADR-0002-path-ownership-map.md) | Path ownership map | Accepted ⚠️ *unopposed* | Who may edit which directory |
| [0003](ADR-0003-policy-ci-gate.md) | `policy` CI gate | Accepted | Public claims in this repo |
| [0004](ADR-0004-decisions-handoffs-and-work-requests.md) | Three lanes: decisions, handoffs, work requests | Accepted | How every role routes traffic |
| [0005](ADR-0005-role-registry.md) | One home for the role list | Accepted | Who exists, and who owes registration |
| [0006](ADR-0006-one-worktree-per-role.md) | One git worktree per role | Accepted | Where every session may work |
| [0007](ADR-0007-delegation-dispatch-and-utilization.md) | Delegation: board → issues → dispatch | Accepted | How work flows downward, and the concurrency cap |
| [0008](ADR-0008-documentation-drift.md) | Two doc tiers; drift caught at PR time | Accepted | Anyone changing code a doc describes |
| [0009](ADR-0009-fourth-repo-and-game-engineer-seat.md) | Fourth repo for the Unreal client, and a Game Engineer seat | Accepted | Where game/renderer code may live; the repo-boundary rule |
| [0010](ADR-0010-game-wire-v1-fixed-for-the-launch-weekend.md) | Game wire v1, fixed by the COO for the launch weekend | Accepted ⏳ *expires Tue 2026-08-04* | `sim-host` and the UE client; the MuJoCo↔Unreal frame transform |
| [0011](ADR-0011-hold-the-launch-until-the-board-stops-flipping.md) | Hold the launch until the board stops flipping at full stick | Accepted 🛑 *launch held; exit criteria provisional pending diagnosis* | Every role with Monday-dated work; all public stability claims |

**Roles:** [`ROLES.md`](ROLES.md) is the single home for the role list — `policy` parses it.
`python3 .github/policy_check.py --who <path>` answers "who owns this?".
`--reconcile <doc>` stamps a doc's manifest after you have brought it back in line with the code.

## Conventions

- Filename `ADR-NNNN-kebab-title.md`, numbered in order of ratification, never reused.
- Status is one of `Proposed`, `Accepted`, `Superseded by ADR-NNNN`, `Rejected`.
- An ADR is never edited to reverse it. Write a new one and mark the old one superseded.
- Every ADR names the Escalations row it closes, so the reasoning stays reachable.
- `Proposed` ADRs are visible on purpose: they are the current working assumption, and a
  role that disagrees files a row rather than editing around it.

## Never wait on your own PR

Branch protection is **strict** — a PR must be up to date with master before it merges, and
master moves several times a day. Polling for that is wasted time and wasted tokens.

```sh
gh pr merge <n> --squash --auto      # queues it; GitHub updates the branch and merges when green
```

Auto-merge and auto-delete-on-merge are enabled on all three repos. **Queue the merge and move
on to the next thing.** Come back only if a check actually fails.

## Recording what you did — write a NEW file, never append to a shared list

**`roles/<role>/CONTEXT.md` is standing context** — sub-goals, turf notes, known dead ends. It
changes rarely and is edited deliberately.

**To record work you completed, add a new file:** `roles/<role>/log/YYYY-MM-DD-<slug>.md`.
One entry, one file. Do **not** append to a shared list.

The reason is mechanical, not stylistic. Every dispatched agent appending to one list means
**every PR touches the same file, so every merge conflicts every other open PR.** On 2026-07-28
that produced five simultaneous conflicts across eight PRs and serialised a queue that was
otherwise entirely green. Separate files never collide.

## Escalate when any one is true — Promise, Door, or Turf

- **Promise** — it changes something outside your role relies on: a public claim, a
  requirement, an acceptance criterion, an interface/ICD, or a doc another role owns.
- **Door** — it is one-way or expensive to reverse: money, anything published, a schema or
  API, a claim already live on the site.
- **Turf** — you are about to do work another role owns. See [CODEOWNERS](../../.github/CODEOWNERS).

All three false → decide it yourself and log it. **Difficulty is not a trigger.**
Hard-but-reversible-and-yours is what your Oracle is for, and consulting the Oracle is not
an escalation.

Every row carries a **default action** and a **deadline**. Nobody is ever parked waiting on
an answer; if the deadline passes, execute the default and record that it went unopposed
rather than agreed. The record must distinguish those two.
