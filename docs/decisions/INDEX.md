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

**Roles:** [`ROLES.md`](ROLES.md) is the single home for the role list — `policy` parses it.
`python3 .github/policy_check.py --who <path>` answers "who owns this?".

## Conventions

- Filename `ADR-NNNN-kebab-title.md`, numbered in order of ratification, never reused.
- Status is one of `Proposed`, `Accepted`, `Superseded by ADR-NNNN`, `Rejected`.
- An ADR is never edited to reverse it. Write a new one and mark the old one superseded.
- Every ADR names the Escalations row it closes, so the reasoning stays reachable.
- `Proposed` ADRs are visible on purpose: they are the current working assumption, and a
  role that disagrees files a row rather than editing around it.

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
