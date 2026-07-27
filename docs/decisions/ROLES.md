# Role registry — the single home

**This file is the identity of record for what roles exist.** `policy_check.py` parses the
table below; `.github/CODEOWNERS` `# role:` tags are validated against it; the Notion
Escalations `From`/`To` selects are a **declared mirror** of it.

Before this file existed the role list lived in two unlinked places — the policy check's
hardcoded set and the Notion select — with no owner. It broke on first contact with a new
role: the Senior Digital Marketer was granted by the CEO with its own escalation chain,
owned surface and branch prefix, and appeared in none of the org's data structures. It filed
under the CMO seat with a title prefix, so **every row it filed was misattributed**, and two
rows appeared to come from the CMO and did not. See ADR-0005.

## Existence is not ratification

**A role exists the moment the CEO grants it.** Ratification is the COO's paperwork, not the
role's permission slip. Conflating the two produced a real escalation row *asking permission
to escalate*.

| Status | Means | May file rows? | May be a `# role:` tag? | Exclusive path ownership? |
|---|---|---|---|---|
| `Provisional` | CEO has granted it; COO paperwork incomplete | **Yes** | **Yes** | No — shared/additive only |
| `Ratified` | Registry entry **+** CODEOWNERS rule **+** Notion select option | Yes | Yes | Yes |

All-three-or-none is the definition of `Ratified`. It is **not** a precondition for working.

- A CEO grant obliges the COO to add a `Provisional` row here within one working day. One
  file, one commit — no Notion, no CODEOWNERS needed.
- **"May I exist" is never a valid escalation row.** Role creation is a CEO act.
- If your role is missing from the Notion select, that is a **COO defect, never a blocker on
  you.** File under the nearest registered seat in your chain, put
  `UNREGISTERED ROLE: <name>` as the first line of the row body, address it to the COO, and
  carry on. The COO repairs attribution when registering.

## The roles

| Role | Status | Escalates to | Branch prefix | Owns |
|---|---|---|---|---|
| `CEO` | Ratified | — | — | Direction, public claims, licence, money. Final arbiter |
| `COO` | Ratified | `CEO` | `feat/ops/` | Cross-role operations, the decision record, the escalation queue for the engineering line |
| `CMO` | Ratified | `CEO` | — | The marketing line. Peer of COO — neither escalates to the other. Owns nothing in this repo by design; brand lives in `overboard-web` |
| `Senior Digital Marketer` | Provisional | `CMO` | `feat/web/` | Landing page, brand and visual identity, page copy, analytics — in `overboard-web` |
| `Digital Content Production` | Ratified | `Senior Digital Marketer` | `feat/content/` | Renders and published media. Explicitly **not** page markup or CSS |
| `Sr. Mechanical & Systems` | Ratified | `COO` | — | BoM, platform selection, the bench rig, the sim-to-hardware fidelity contract |
| `Senior Controls` | Ratified | `COO` | — | The control law and its harness |
| `Archivist` | Provisional | `CEO` | — | *Surface not yet declared* |

⚠️ **Unverified entries.** Escalation targets for `Sr. Mechanical & Systems` and
`Senior Controls`, the branch prefixes for `Digital Content Production` and
`Senior Digital Marketer`, and everything about the `Archivist` are **inferred** — from the
COO's own charter ("receives from Sr. Mechanical and Systems Engineering"), from branch
conventions observed in sibling repos, and from a CEO remark that the Archivist reports
directly. Inferring turf from a branch prefix is guessing, which is the thing this registry
exists to stop. Each is flagged rather than asserted, and each wants one line of confirmation
from the role itself or the CEO. `Provisional` roles stay provisional until that lands.

## Residual risk, stated rather than papered over

**Notion is a copy that CI cannot reach.** There is no automated link between this file and
the Escalations select options — a token-bearing CI job that queries Notion would produce red
builds the PR author cannot fix, which would poison the one interrupt that works. So the
mirror has a named owner (**COO**), a declared direction (registry → Notion, never back), and
no pretence of automation.

## Repo scope

This registry covers all three repos — `overboard`, `overboard-web`, `overboard-viz`. Only
`overboard` has a `CODEOWNERS` today, so "do not edit paths another role owns" is still
unfalsifiable in two thirds of the estate. Open item in ADR-0002.
