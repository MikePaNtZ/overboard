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
| `Senior Digital Marketer` | Ratified | `CMO` | `feat/web/` | Landing page, brand and visual identity, page copy, analytics — in `overboard-web` |
| `Content Designer` | Provisional | `Senior Digital Marketer` | `feat/copy/` | Public copy craft to the Style Guide, and UX fit-and-finish. **No exclusive path by design** — works in files the SDM owns, gated by SDM review |
| `Digital Content Production` | Ratified | `Senior Digital Marketer` | `feat/content/` | Renders and published media. Explicitly **not** page markup or CSS |
| `Sr. Mechanical & Systems` | Ratified | `COO` | `feat/mech/` | BoM, platform selection, the bench rig, the sim-to-hardware fidelity contract |
| `Senior Controls` | Ratified | `COO` | `feat/controls/` | The control law and its harness |
| `Archivist` | Ratified | `CEO` | `feat/archive/` | Strategy-tier Notion, shared vocabulary across all three repos, drift tooling. Owns `docs/vocabulary/` and no other repo path |
| `Game Engineer` | Provisional | `COO` | `feat/game/` | The Unreal client in `overboard-game` — rendering, gamepad input, world authoring, avatar. **Peer of Senior Controls, not subordinate to it.** Owns nothing in this repo by design; the wire schema is the seam between them. See ADR-0009 |

✅ **Branch prefixes for `Sr. Mechanical & Systems` (`feat/mech/`) and `Senior Controls`
(`feat/controls/`) are now OBSERVED**, not inferred — both appear repeatedly in merged branch
history. They were blank until 2026-07-27, which meant the `policy` turf check **silently
skipped** every branch those two roles pushed: it fails open for roles with no declared prefix,
so the two most active engineering roles had no turf enforcement at all. Caught by a dispatched
agent, not by me.

✅ **`Senior Digital Marketer` is RATIFIED as of 2026-07-31**, and its `feat/web/` prefix is
observed rather than inferred. It could not be ratified before now for a concrete reason:
`overboard-web` had **no CODEOWNERS at all**, so the second of the three legs did not exist to
point at. Ported in overboard-web#35; the flag follows the artefact, not the other way round.

⚠️ **Still unverified.** Escalation targets for `Sr. Mechanical & Systems` and
`Senior Controls`, and the branch prefix for `Digital Content Production`, are **inferred** — from the
COO's own charter ("receives from Sr. Mechanical and Systems Engineering"), from branch
conventions observed in sibling repos. The `Archivist` is no longer among them — the CEO
declared its surface on 2026-07-27 and it was ratified the same day. Inferring turf from a branch prefix is guessing, which is the thing this registry
exists to stop. Each is flagged rather than asserted, and each wants one line of confirmation
from the role itself or the CEO. `Provisional` roles stay provisional until that lands.

## Residual risk, stated rather than papered over

**Notion is a copy that CI cannot reach.** There is no automated link between this file and
the Escalations select options — a token-bearing CI job that queries Notion would produce red
builds the PR author cannot fix, which would poison the one interrupt that works. So the
mirror has a named owner (**COO**), a declared direction (registry → Notion, never back), and
no pretence of automation.

## Repo scope

This registry covers all four repos — `overboard`, `overboard-web`, `overboard-viz`, and
`overboard-game` (added by ADR-0009). **Two of the four have a `CODEOWNERS`:** `overboard`, and
`overboard-web` since overboard-web#35. So "do not edit paths another role owns" remains
unfalsifiable in `overboard-viz` and `overboard-game`. Open item in ADR-0002.

This is also the concrete reason `Game Engineer` is `Provisional` rather than `Ratified`: the
ownership leg of the three cannot exist until `overboard-game` has a `CODEOWNERS`. The flag
follows the artefact. It does not block the role from working — see "Existence is not
ratification" above.
