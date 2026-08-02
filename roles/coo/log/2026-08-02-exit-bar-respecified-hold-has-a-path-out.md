# 2026-08-02 (second session) — the fix landed, two criteria didn't, and the hold now has a path out

**Role:** COO · **Session outcome:** ADR-0011's fix built, measured and merged. Exit bar
respecified on an Oracle adjudication. The hold has a finite, named path out for the first
time. It is **not** cleared.

## The one thing to know

**The specified fix works and does not clear the hold.** It meets (b), (c), (a)-1 and (a)-2 —
including the **full reverse-to-forward reversal at speed**, which ADR-0011 records as never
tested and names the worst case. It provably cannot meet (f) or (a)-3 at any value of the
constant. Both criteria have now been respecified rather than softened, and the reasoning is
in ADR-0011's second ratification.

## The finding that settles criterion (f) — check the arithmetic, don't take it on trust

- ADR-0011 lean sensitivity: **5.84° per m/s²**, fixed by geometry.
- One radian per g: **57.2958 / 9.80665 = 5.8425° per m/s²**.
- Measured `est − truth` residual slope: **~5.8° per m/s²**.

**Same number, same physics.** The lean needed to sustain acceleration `a` is
`atan(a/g) ≈ a/g` — exactly the apparent-vertical tilt a complementary filter reads.

So the estimator is **not** carrying an error that flatters the controller. It is
inadvertently implementing the textbook balance-vehicle solution, the lean-into-acceleration a
Segway performs deliberately. `--pitch-source truth` does not de-bias the loop — it **deletes
the acceleration reference** and leaves a pitch-only regulator with 4 cm of ballast trying to
buy 17 cm of CoM offset. Inverting at every stick fraction down to 0.05 is what that physics
predicts. Criterion (f) was measuring a **different, broken controller**.

⚠️ **This supersedes the previous session's finding #2**, which said "fixing the estimator
makes the board worse — accidental model error, not a designed property." The measurement was
right; the interpretation was half right. It is not an error at all. It is the correct
behaviour, arrived at accidentally and — until #207 lands — **not pinned by anything**.

## What did NOT follow, and why it matters

**±0.25° did not become the new criterion**, even though it is the measured static-error
tolerance. That would derive the acceptance number from whatever happened to pass, which is
the precise move (f) exists to forbid. It is recorded as a characterisation. If a future
session sees ±0.25° quoted as a threshold anywhere, that is a regression.

## Measured numbers worth carrying forward

| | |
|---|---|
| peak-demand slope | **41.97 A/unit** (re-measured; 42.03 documented) |
| worst point (reversal at speed) | **35.40 A of 40 A** → 4.60 A headroom (11.5%) |
| | **9.74° of 11.46°** → 1.72° pitch headroom (15.0%) |
| warning lead over `FALLEN` | **+2.868 s** (`FALLEN` trails saturation by −0.948 s) |
| static pitch error | inverts at **±0.5°**, survives **±0.25°** |
| kerb | ~**1 mm** at a calm point, nothing at the worst |
| 20 mm lip | **201 °/s** imparted vs ~**76 °/s** of KD authority |

**The 0.80 reserve is trim-derived, not geometry-derived.** The slope was measured at the
current operating trim. Anyone describing it as geometry-derived is rationalising.

## Harness validity — the check that separated finding from bug

My first instinct was that "inverts at 0.05 stick" smelled like a wiring bug. It isn't: the
harness reproduces ADR-0011's own independently-established figures — saturation **4.920 s**
vs 4.92 s, `FALLEN` **5.868 s** vs 5.87 s. Run that check before believing any alarming
result out of this harness, and before disbelieving one.

## Process: two agents died on usage limits, mid-task

Both left substantial work **uncommitted**. The first left ~1,277 lines one `git clean` from
gone. Snapshotted under a commit message that states plainly it is unvalidated, then verified
before anything was propagated.

**The lesson is not "agents die."** It is that the previous session produced exactly this
artefact — `feat/controls/turn-radius-and-reset` — and it sat for a day looking plausible
because **its CI was green**. Green CI on a branch that adds no tests of its own says nothing.
Snapshot immediately, label honestly, verify before quoting.

I also ran the acceptance suite myself and got `11 passed, 4 xfailed` **in 0.36 s** — far too
fast to have simulated anything. `cargo` was missing from `PATH`, and the xfail strings are
static decorator text, so it looked like a result and was not one. With `PATH` fixed: 7.31 s,
same verdict, real. **An xfail reason is not a measurement.**

## What cleared, and what is queued

| | |
|---|---|
| #188 | merged — turf override; oracle is now Fable 5 |
| #198 | merged — demo manifest rescued off volatile `/tmp` |
| #199 | merged — restart briefs, **six** roles |
| #200 | merged — ADR-0011 amended, CMO added to "who moves" |
| #205 | queued — the reserve + warning, validated |
| #206 | queued — **ADR-0011 second ratification, exit bar respecified** |

Full gate on #205, first-hand: fmt clean, clippy clean, `cargo test` **194 passed 0 failed**,
`pytest tests/` **318 passed 6 xfailed 0 failed**, acceptance file **11 passed 4 xfailed, no
XPASS**. The four are `xfail(strict=True)`, so an XPASS would mean a known failure had
silently stopped being one.

## What now clears the hold — finite, for the first time

1. **[#207]** (f1) pin the estimator residual so a retune that moves the trim breaks CI;
   (f2) assert the 41.97 A/unit slope so the constant goes stale loudly; measure the authored
   incline tolerance (predicted <0.5°).
2. **[#208]** encode the authored-world constraint as a **check**, not prose.
3. The loss-of-authority warning must actually **surface to the player** — it is implemented,
   not yet shown.

All three conditions are required. Drop one and the criterion move becomes the softening
manoeuvre the ADR forbids.

**The headroom-based fix is NOT required now** — but it is a **named blocking prerequisite on
the next boundary**: world expansion, any retune moving the pinned trim band, and the hardware
gate. It is not a deferred aspiration any more.

## Still open from the previous session

- **[#201]** turn radius — **6.95 m** measured, target ~3.5 m. Never implemented; the constant
  is byte-identical to master.
- **[#202]** reset works (verified by measurement) but has **zero tests**, and `host.rs` ~847
  claims measurements that do not exist. The comment becomes true or it goes.
- **[#204]** braking/coasting. **Sequence last** — braking loads the board the way gravity does
  during a forward stop, which is the reversal case. It may create a *new* way to invert.
- **[#203]** a role context file can contradict a ratified ADR and nothing notices.

⚠️ **`host.rs` `run()` ~773–873 is the sequencing constraint.** #201, #202, #204 and #207 all
land there. One at a time.

## Owed to the CEO

**Four roles still need restarting** — briefs are written and merged at
`roles/coo/restart-briefs-2026-08-02-launch-hold.md`. **CMO and Senior Digital Marketer
first.** The CMO brief was added this session because ADR-0011's own consequences list missed
it, and its `CONTEXT.md` on `master` still issued Monday launch orders; a banner defuses it,
but the sub-goals are CMO's to re-base.

**Senior Controls can now be restarted** — #205 is queued, so the collision that blocked it is
gone. Point it at #207 first.

**Usage is the live constraint.** Weekly all-models was ~85% at session end with a hard cliff
near. Fable had headroom, which is why the adjudication went there — one distilled call, no
fan-out. Prefer running checks in the driver over spawning agents: an agent re-reads the
codebase, and most of this session's spend was that.
