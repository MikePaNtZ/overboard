# ADR-0005 — One home for the role list; existence is not ratification

- **Status:** Accepted
- **Date:** 2026-07-26
- **Ratified by:** COO
- **Closes:** [Escalations — "Register the Senior Digital Marketer role"](https://app.notion.com/p/3a9472a5fb6981f49a8bed0501998c66)
- **Constrains:** every role; the COO owes the registration
- **Enforced by:** `policy` CI job (registry parsing, tag resolution, ratified-role coverage)

## Context

The Senior Digital Marketer was granted by the CEO with its own escalation chain, owned
surface and branch prefix — and existed in **none** of the org's data structures. It filed as
`From: CMO` with a `[Senior Digital Marketer]` title prefix, so every row it filed was
misattributed and two rows appeared to come from the CMO and did not. The Archivist was
missing too.

The underlying defect, which that role identified and which matters more than its own
registration: **the role list lived in two unlinked places with no owner** — the policy
check's hardcoded `KNOWN_ROLES` set and the Notion select options. It broke on first contact
with a new role and would have broken identically for every future one.

Worse, the partial state produced a deadlock: a `# role:` tag for an unregistered role
red-builds `policy` for everyone, so the role could not claim its own paths — and it filed
**an escalation asking permission to escalate.**

## Decision

1. **`docs/decisions/ROLES.md` is the single home** and the identity of record.
   `policy_check.py` parses it; `CODEOWNERS` tags are validated against it; the Notion select
   is a **declared mirror**, direction registry → Notion, never back.
2. **`Ratified` means all three** — registry entry, CODEOWNERS rule (or an explicit
   owns-nothing declaration), and Notion select option.
3. **Existence is not ratification.** A role exists the moment the CEO grants it. A grant
   obliges the COO to add a `Provisional` entry within one working day — one file, one
   commit. **`Provisional` roles may file rows, open PRs, and be named in `# role:` tags.**
   They may not hold *exclusive* path ownership; that is the only right all-three-or-none
   should gate, and the only one that genuinely needs three-way consistency.
4. **"May I exist" is never a valid escalation row.** Role creation is a CEO act.
5. **A missing Notion option is a COO defect, never a blocker on the filer.** File under the
   nearest registered seat in your chain with `UNREGISTERED ROLE: <name>` as the first line
   of the body, address it to the COO, and carry on.

## Options considered

- **Register the two roles and move on.** Rejected: fixes the instance, not the duplication.
  The third role would have hit it again.
- **All-three-or-none as a hard precondition.** Rejected — this is the deadlock that already
  happened. It is right as a *definition* of ratified and wrong as a gate on working.
- **Have CI read Notion and reconcile.** Rejected: needs a token, fails on network, and
  produces red builds the PR author cannot fix. That would poison the one interrupt in this
  org that reliably works. Named owner and a declared mirror instead.
- **Registry + Provisional/Ratified split.** Chosen. Cost of the losing options: either the
  duplication persists, or new roles are blocked on paperwork they cannot do themselves.

## Consequences

- Adding a role is one commit to `ROLES.md`; ratifying it is three edits, and the COO owes
  them.
- `policy` now fails on a `# role:` tag that does not resolve, and on a `Ratified` role that
  appears nowhere in `CODEOWNERS`.
- Attribution on the three misfiled rows has been repaired retroactively.
- **Residual risk, not papered over:** Notion is still a copy CI cannot reach. Named owner
  (COO), declared direction, no pretence of automation.

## How this is enforced

`policy_check.py` parses `ROLES.md` instead of hardcoding roles; checks every `# role:` tag
resolves; checks every `Ratified` role appears in `CODEOWNERS`. It also gains
`--who <path>`, so *"am I trespassing?"* is one command rather than a human applying
last-match-wins down a file they have never seen.
