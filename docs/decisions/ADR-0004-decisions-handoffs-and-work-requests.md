# ADR-0004 — Three lanes: decisions, handoffs, and work requests

- **Status:** Accepted
- **Date:** 2026-07-26
- **Ratified by:** COO
- **Closes:** [Escalations — "Fix three org-system defects"](https://app.notion.com/p/3a9472a5fb698109a519d1c5283ae999) (defect 3)
- **Constrains:** every role
- **Enforced by:** `Kind` field on the Escalations database; the `CLAUDE.md` org block; convention

## Context

Four days were lost on content that was already finished. The Senior Digital Marketer
believed it was blocked on Digital Content Production, filed a queue row asking for delivery
with a six-day deadline, and waited. Both roles were active and filing rows **within eleven
minutes of each other**. Neither read the other's. The CEO had to override manually.

The queue was at ten rows, nearly all `Open`, near-zero answered.

## The invariant that was violated

> **If you cannot name a default action you could execute alone, it is not an escalation.**

That is what the queue is for: *"I can proceed without you — here is what I do if you stay
silent."* A request for someone else's deliverable has the opposite invariant: *"I cannot
proceed; you own the thing."* Rows with no executable default cannot self-close, so they rot.
Ten open rows is that invariant failing, not people being slow.

## Decision

Three lanes, chosen by what the traffic actually is.

| Traffic | Lane | Why |
|---|---|---|
| **Decision** — choose between options | The **Escalations queue** | Has a default action; the filer can proceed either way |
| **Handoff** — finished work crossing a boundary | A **pull request**, review requested from the receiving role | Observable without a reply, and it already carries the artefact |
| **Work request** — something that does not exist yet | A **GitHub issue** in the owning repo, assigned via the role registry | Closed by a merged PR (`Closes #N`), so completion is *state*, not a message |

And the rule that would have saved the four days:

> **Before filing a row asking "is X ready?" or "please send me X" — go and look.** Repo
> state, open PRs and merged branches are visible without a conversation. **Prefer what you
> can observe over what you must wait for.**

**Blocked on a human-only act** — money, publication, physical hardware — stays in the queue.
It has a real default action: descope.

## Honesty about what this does and does not fix

The third lane **would not have prevented this particular stall.** The content was already
done; no lane routes around "nobody looked." Only the go-and-look rule does.

What the third lane prevents is a *different* recurrence: a genuine "I need X built", with no
executable default, sitting `Open` for days because the queue was the only container with an
owner and a deadline. That is queue pollution, and it is worth fixing on its own terms — but
it is not the thing that cost four days, and this ADR should not be read as claiming it is.

## Options considered

- **Two lanes (decisions, handoffs).** The CEO's original framing. Rejected: it has no home
  for a request for work that does not yet exist, which is precisely what the stalled row
  was. Omitting it sends that traffic back into the queue and re-creates the pollution.
- **A Notion task database for work requests.** Rejected: a second polling surface, with no
  link to the artefact and no way to close itself.
- **Three lanes as above.** Chosen. Cost of the losing options: the queue keeps filling with
  rows nobody can close.

## Consequences

- Each role's real inbox becomes `gh pr list --search "review-requested:@me"` and
  `gh issue list --assignee @me`. **Caveat, stated plainly:** with one GitHub account behind
  every role, `@me` does not discriminate by role today. The lane discipline is real; the
  per-role inbox is aspirational until identities diverge.
- The queue should shrink. If it does not, the lanes are not being used.
- `Kind` is now a field on the Escalations database, so misclassification is visible at the
  point of filing rather than discovered four days later.

## How this is enforced

Weakly, and deliberately so. Whether a row is "really" a decision is not machine-decidable,
and a false positive on that judgement would get the whole gate switched off — the same
reasoning as the advisory check in ADR-0003. So: the `Kind` select forces the classification
at write time, the database description carries the rule, and `CLAUDE.md` carries it into
every session automatically. The default-action test above is the thing to apply; it is prose
because it has to be.
