# AI Workflow Log

Raw capture of **how this project is actually being built with AI** — the collaboration
itself, not the engineering results. Engineering outcomes live in the other `docs/`
mirrors and in the commit history; this file holds the process material that is free to
record while the work is happening and impossible to reconstruct afterwards.

Required by [P0 — Portfolio Strategy](https://app.notion.com/p/3a9472a5fb69815587c7ca82e22ac781)
§7 and §10: *"capture the process material — it is cheap now and unrecoverable later."*
P0 §5 argues the AI workflow is the single most transferable artefact this project
produces, and the one an eventual programme would actually teach.

## Capture discipline

- **Append-only, dated, rough.** This is a notebook, not a document. Do not polish it;
  polish happens later when an entry is drawn on for the public build log.
- **Record the misses, not just the hits.** An entry where the model was confidently
  wrong is worth more than three where it was right — those are the entries that make
  the honest case, and they are exactly the ones nobody remembers to write down.
- **Capture the decision, the reasoning, and who made it.** The interesting unit is not
  "AI wrote some code"; it is where judgment sat, what was delegated, what was escalated,
  and what a human overruled.
- **No cadence promise.** Entries land when something happens.
- ⚠️ **Nothing identifying about family or any child** goes in this file, per P0 §8.1,
  until that policy exists.

## Entries

### 2026-07-25 — The design doc caught a bug the code was hiding

**What happened:** reconciling the Notion design docs against the implementation turned up
an inverted motor sign in the MuJoCo model (`e696c11`). BoardIo ICD §7.3 mandates
`amps > 0 => forward wheel acceleration => nose pitches up`; the model did the exact
opposite, and **the model's own comment claimed the opposite of what the model did.**

**Why it matters as process material:** nothing depended on it yet — the open-loop
scenario never actuates — so no test was failing and nothing looked broken. It would have
inverted the balance law the instant a controller was attached. The ICD names a sign error
across this seam the most dangerous bug in the system and says in as many words: *do not
implement from memory*. It was found by a **document-to-code reconciliation pass**, not by
running anything.

**The transferable bit:** the design doc earned its keep here as an *executable check*
rather than as documentation. Worth watching whether that repeats.

**Left deliberately unfixed:** the scenario reports pitch nose-down-positive while the ICD
is nose-up-positive, so the same control law is written `+K*pitch` in one place and
`-K*pitch` in the other. Resolving it means deciding *which document moves* — a human
call about the ICD, not a unilateral code change. Pinned by tests so it cannot silently
invert again.

### 2026-07-25 — Two model errors that a plausible-looking answer would have buried

Both from the impulse-response scenario work, both cases where the first confident answer
was wrong in a way that reads fine:

1. **An unfalsifiable acceptance criterion.** The design doc set the topple gate at "pitch
   exceeds ~45°." The vehicle physically cannot reach 45° — a bumper contacts the ground at
   **18.57°**. The gate could never have been satisfied, no matter how hard the board was
   hit. Caught by computing the contact angle from the collision hulls instead of accepting
   the number in the doc.
2. **A wrong derivation that looks right.** Re-deriving that angle from the STL bounding box
   gives 14.9°, by assuming the extreme −X and extreme −Z coordinates meet at one corner.
   They don't — the bumper sweeps upward, so the vertex that lands first is the underside
   heel ~90 mm inboard of the tip. This was wrong in an earlier pass of the model header
   before it was caught.

**The transferable bit:** both errors produce a plausible number. Neither would have been
caught by reading the output. The thing that caught them was insisting the value be
*computed from the geometry at runtime* rather than written down.

### 2026-07-26 — The agent worked from a stale picture of the repo and said so late

**What happened:** a session opened with a snapshot of the repo taken hours earlier. The
agent reported project status confidently from it — including "the `web/` directory is
uncommitted" — when `web/` had by then been split into its own public repo and four more
commits had landed. The error surfaced only when a file read failed on a path that the
snapshot said existed.

**The transferable bit:** the agent's confidence was identical before and after the
correction. There was no internal signal distinguishing the stale report from the accurate
one; it took a **failed read against the real filesystem** to expose it. Verification
against live state, not self-assessment, is what caught it.

### 2026-07-26 — Escalation overruled the driver on the website audience

**What happened:** the owner asked to re-aim the public page at non-specialists (high-school
age), and the question was escalated to a higher-tier model for adjudication rather than
executed directly. Three outcomes worth recording:

- **The driver's plan was partly the failure it was trying to avoid.** The proposal was
  "teen-readable above the fold, engineer depth below it." The adjudication called this
  vertical segregation — if a reader can point at a band and say *that part is for the
  kids*, both audiences are lost at that seam. Replaced with an artifact-first cut:
  images, plots and numbers are legible to a 13-year-old and respected by a specialist,
  where **text** is what forces the choice.
- **A one-way door got caught before it opened.** Writing at a teen-readable register is
  cheap and reversible; putting *education / STEM / nonprofit* framing on the public page
  is neither, and would commit a later programme stage publicly before the current one has
  any evidence. Rule adopted: write so a 15-year-old can read it; never say it's for them.
- **Both the owner and the driver had missed the actual top-priority defect.** The public
  page described a Rust control stack that does not exist — a `BoardIo` seam presented as
  working when both backends are stubs, a control core credited with an estimator and
  safety envelope when it returns zero, and a Rust binary claimed to drive the sim when the
  working harness is Python. Fixed the same day, ahead of any redesign work.

**The transferable bit:** the escalation's value was not a better version of the plan
submitted to it. It was **reordering the work** — finding that the page needed to be made
*true* before it was made more compelling.
