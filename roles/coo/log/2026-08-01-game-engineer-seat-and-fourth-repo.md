# 2026-08-01 — a Game Engineer seat, a fourth repo, and the M3 planning set

## Headline

The CEO scoped **M3 — a rider-driven Unreal client with post-ride reconstruction** — in session,
granted a **Game Engineer** seat, and directed that the Unreal work live in its own repo. This
entry covers the paperwork that makes those decisions bind: ADR-0009, the registry row, and the
repo-boundary amendment.

## What landed

| Thread | |
|---|---|
| **ADR-0009** | Fourth repo `overboard-game` for the Unreal client; `Game Engineer` seat owns it |
| **Registry** | `Game Engineer` added as `Provisional` — escalates to COO, `feat/game/`, peer of Senior Controls |
| **`CLAUDE.md`** | Repo boundary rewritten from two named repos to four, with the "nothing outside this repo computes board physics" rule stated once for both renderers |
| **Notion** | M3 Scope, M3 Implementation Plan, and design docs D6–D9, all under Program Plan & Roadmap |
| **#156** | Sensor procurement raised to Sr. Mechanical & Systems — 9-axis IMU + GNSS into the BoM |

## The thing worth carrying forward

**The boundary M3 needs was already written down, in a repo nobody thought to look at.**

`overboard-viz`'s README has stated since 2026-07-26 that the renderer never computes physics and
that exactly one data contract crosses. That is precisely the architecture the Unreal client needs,
and it was rediscovered from first principles in conversation before anyone checked whether it
already existed. It did.

So ADR-0009 is deliberately *not* a new boundary — it reuses viz's and says so. The only genuine
question was whether the Unreal client belonged **in** viz, and that turned on lifecycle (offline
Blender vs. a real-time engine with multi-gigabyte assets) and on turf (viz sits under Digital
Content Production, on the marketing line; the game is an engineering instrument Senior Controls
depends on). Those two, not the boundary, are what produced a fourth repo.

The generalisation: **go and look at the estate before designing something the estate already has.**
This org's repo READMEs are load-bearing documents and they are cheaper to read than to re-derive.

## Registry accuracy repaired in passing

`ROLES.md`'s "Repo scope" claimed only `overboard` had a `CODEOWNERS`, "unfalsifiable in two thirds
of the estate". Stale — `overboard-web` got one in overboard-web#35, which is the same PR the SDM's
ratification note in this very file already cites. Corrected to two of four, with `overboard-viz`
and `overboard-game` named as the gaps.

That is the second time a claim in this registry drifted from an artefact the registry itself
mentions. Worth a check that reads the repos rather than trusting the prose, if it happens again.

## Open, and owed by me

- **`overboard-game` has no `CODEOWNERS`**, which is why the seat is `Provisional` and not
  `Ratified`. The flag follows the artefact.
- **Notion Escalations select** needs a `Game Engineer` option — registry → Notion, never back.
- **Session start/teardown prompt** for the new role.
- The CEO has not yet said whether M3 sits inside the **D0 ready-to-code gate**. D6–D9 carry numeric
  acceptance criteria either way, so the answer changes the process, not the documents.
