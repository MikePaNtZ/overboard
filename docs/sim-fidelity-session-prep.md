# Sim fidelity — prep for the working session (#33)

<!--
NO `covers:` MANIFEST, deliberately. ADR-0008 has two tiers: implementation-tier
docs carry a manifest so doc-drift can force them to move when the code they
describe moves. This is NOT one of those -- it is a dated snapshot prepared for
one session, and it cites code the way a meeting agenda cites a report.

It briefly had a manifest over plant.py and imperfections.py, and within the
hour that blocked Sr. Mechanical & Systems' PR #128: their change to
imperfections.py dragged this prep doc along and red-built work that had
nothing to do with it. A manifest on a snapshot taxes everyone who touches the
covered files, forever, to keep a document current that is meant to go stale
the day after the session.
-->

**Requirement: `SR-SIM-3`** (no ideal-only mode in CI) and **`SR-SIM-5`** (one control loop, two
backends). This document is the COO's prep for the #33 session, not a decision. It exists so the
session argues about the right things instead of rediscovering the inventory live.

**Owner:** COO (prep) · **Participants:** CEO, Senior Controls, **Sr. Mechanical & Systems**
— the fidelity contract is Mechanical's surface and they have asked to be in the room (#128).

**Status: prep only. Nothing here is a decision, and nothing here changes a threshold.**

---

## 1. What exists today — six scenarios, 89 tests

The distinction that matters for this session is **assert** versus **demonstrate**. A scenario that
*asserts* something is a gate: it fails the build when the property breaks. A scenario that
*demonstrates* is evidence for a human, and its number can drift without anything going red.

| Scenario | Tests | What it ASSERTS (gates) | What it only DEMONSTRATES |
|---|---|---|---|
| `impulse_response` | 16 | Model integrity — no rider proxy bodies, CoM at or below the axle, strike angle derived from the collision hull rather than hardcoded | The open-loop topple itself |
| `closed_loop` | 15 | Nose strike prevented, closed beats open loop, cascade returns to rest, gains leave current headroom, ABI version matches | That the balance controller "works" — **explicitly disclaimed in the file header** |
| `hill` | 22 | Grade sign conventions, cutback symmetry and monotonicity, estimator absorbs a stated fraction of slope | Crash statistics beyond the 25° sanity ceiling |
| `terrain` | 17 | Crest-dip-crest profile shape, steady vs rolling comparison, estimator costs the envelope | Ride "quality" |
| `shuttle_run` | 11 | Route integrates to zero net displacement, returns past home | Return error, station-keeping — **the file says outright these bounds are loose and expected to move** |
| `disturbance_envelope` | 8 | Boundary-finding logic itself — one grid step below first failure, raises if the sweep starts too high | Where the boundary actually is |

**The pattern worth noticing:** most of what is *asserted* is **internal consistency** — sign
conventions, geometry, boundary-search logic, model integrity. Most of what is *physical* is
demonstrated rather than gated. That is not a criticism; it is the honest state, and it is exactly
what "representative enough" has to be defined against.

## 2. Known model defects, stated by the code itself

These are not discoveries. Each is documented at the point it applies — which is why they are
credible and why the session can trust the list.

| Gap | Where it is stated | Why it matters for fidelity |
|---|---|---|
| **No rider dynamics at all** | `plant.py:81` — *"a rigid lump bolted to the frame — it does not articulate, shift weight, or absorb anything at the ankles and knees"* | Rider body modes sit at ~1–3 Hz ≈ **6–19 rad/s**, directly on top of any candidate crossover above ~8 rad/s |
| **No tyre model** | `test_closed_loop.py` header | No contact-patch compliance; unmodelled lag exactly where the loop is fastest |
| **`kt = 0.7` unfitted** | `plant.py:57` | Was load-bearing on loop gain until #137; now a headroom term, but still unmeasured |
| **Pitch is MuJoCo truth in the headline gate** | `test_closed_loop.py:11` | The estimator is in the loop elsewhere, but the flagship number is ideal-sensor |
| **No IMU noise spectral model** | absent — *that is the gap* | Bias instability, random walk and motor vibration coupling all land in the band a balance loop cares about |
| **Fixed delay, not a jitter distribution** | `imperfections.py` | Speaks to a sustained shift, not to a burst of consecutive misses (#130) |
| **No imperfection profile on the Rust host** | #129 | `SR-SIM-3`'s "no ideal-only mode in CI" **does not hold** on the Rust path today |
| **Reference disturbance inherited, not derived** | #142 | 20 N·s is a scenario nominal; the verdict flips by 30 N·s |

## 3. What the session has to settle

Framed as decisions, because that is what a session is for.

1. **What does "representative enough" mean, numerically?** The stopping rule matters more than
   the list — sim fidelity is unbounded, and without a rule the answer is always "more".
2. **Which gaps change a control decision, and which are realism for its own sake?** The rider
   compliance and tyre gaps sit where the loop is fastest, so they are candidates for the first
   group. Cosmetic realism is not.
3. **Which scenarios exist to PROVE something versus to SHOW something** — and which do both.
   Those are the valuable ones, because one run serves both engineering and the content roadmap.
4. **What we deliberately will NOT model, and why.** Written down, so it stops being relitigated.
5. **How it pairs with the content roadmap** so a scenario produces footage without a second run.

## 4. What this document deliberately does not do

It does not propose a fidelity target, rank the gaps, or recommend what to model next. Those are
the session's output, and pre-empting them in the prep is how a working session becomes a
rubber stamp on the preparer's opinion.

It also states no new numbers. Every figure above is cited from code or from an existing issue.
