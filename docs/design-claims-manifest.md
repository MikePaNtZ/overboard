# Claims Manifest — let CI verify page copy against engineering state

<!--
covers:
  - tests/conftest.py
  - scripts/emit_claims.py
  - .github/workflows/ci.yml
-->

**Requirement: `SR-WEB-4`** — a capability claim may not appear on the site unless a
requirement backs it, and a claim that becomes false comes off in the same pass. This
document designs the enforcement `SR-WEB-4` has never had.

**Status: design only. Nothing here is implemented.** Closes the design half of
[#61](https://github.com/MikePaNtZ/overboard/issues/61); implementation is Senior Controls'.

**Owner of this document:** COO. **Site-side half needs the Senior Digital Marketer's
countersign** — it changes when their deploy is allowed to proceed, which is a Promise, not
a decision I can take alone.

## The problem

The public site carries capability claims and numbers. Today, checking that a claim still
matches engineering reality is **a human reading two things and remembering**. `SR-WEB-4`
says a claim may not appear unless a requirement backs it, and that a claim which becomes
false comes off in the same pass. **That rule has no enforcement on the site side.**

It has already failed once in the direction that matters. A published figure — the
closed-loop peak pitch on the README — was generated from a simulation that knew its own
tilt perfectly. The honest number is **0.2331 m**, worse than the 0.0648 m that was public.
Nothing detected that; a person noticed.

## What already exists, and what it does not do

`check_claims()` in `.github/policy_check.py` is **text-level only**. It bans absolute
language and requires a requirement ID near any section headed "claim", across this repo's
public markdown. It never reads a number, never runs a test, and never reaches
`overboard-web`. It is a style gate, not a truth gate.

## The five design questions, answered

### 1. Where does the manifest live?

**Generated here, published as `claims.json` to the rolling `sim-latest` GitHub release on
every green `master` push** — the same job and the same pattern as the sim artifact, which
already works and already has a stable URL the site consumes.

Both repos are public, so the site's CI fetches it over plain HTTPS with no token. That
matters: ADR-0003 refuses to let a required check depend on a credential, because a red
build the author cannot fix poisons the one interrupt this org has.

**A fetch failure fails the site deploy.** It does not fall back to the last known manifest
and it does not skip. A deploy that cannot verify its claims is exactly the deploy that
should not go out, and this week already produced two checks that reported green while
enforcing nothing (#83). The blast radius is one re-runnable deploy, not a blocked queue.

### 2. What is a claim, mechanically?

```json
{
  "schema": 1,
  "sha": "3bb36f4",
  "generated_at": "2026-07-30T04:00:00Z",
  "claims": {
    "recovery.peak-pitch": {
      "value": 0.2331,
      "unit": "m",
      "requirement": "SR-SIM-5",
      "test": "tests/test_closed_loop.py::test_closed_loop_prevents_the_nose_strike",
      "gate": "peak_abs_pitch_deg < 3.0"
    }
  }
}
```

**A claim with no green test is not in the file.** That absence is the whole mechanism —
the site cannot state something engineering is not currently proving.

### 3. Generated or hand-maintained?

**Generated, or it drifts.** Which means a test must be able to declare what it pins:

```python
@pytest.mark.claim("recovery.peak-pitch", requirement="SR-SIM-5", unit="m")
def test_closed_loop_prevents_the_nose_strike(record_claim):
    ...
    record_claim(metrics.peak_abs_pitch_deg)
```

The marker carries identity; the `record_claim` fixture carries the measured value. **Both
are required** — a marked test that records nothing is a hard error, not a claim with a
missing value. Otherwise the manifest quietly grows entries that assert nothing, which is
the `reconciled:` failure in [#85](https://github.com/MikePaNtZ/overboard/issues/85) all
over again: a field that looks checkable and is not.

Only tests that **passed** contribute. A failing test's claim is absent, and absent means
the site will not deploy copy that depends on it.

### 4. What happens on failure?

The site's existing `checks` job — `python3 .github/scripts/check_page.py`, which already
gates every deploy — gains a claims step. It fails, and therefore blocks the deploy, when:

| Condition | Why |
|---|---|
| Page references a claim id absent from the manifest | The test is failing, was deleted, or never existed |
| Page's stated number disagrees with the manifest value | The number went stale — the 0.0648 case |
| Manifest cannot be fetched | Cannot verify, so do not publish |
| Manifest `sha` is not an ancestor of the controls repo's `master` | A forged or stale artifact |

### 5. Numbers, not just booleans

The manifest carries the **current value and unit**, so a stale figure is caught rather than
just an unbacked claim.

The page has **no build step and no dependencies** — vanilla HTML that must open from
`file://` — so the binding is a data attribute, invisible to rendering and readable by CI:

```html
<span data-claim="recovery.peak-pitch">0.23 m</span>
```

The checker extracts the number from the element's **text content** and verifies it matches
the manifest value *at the precision the page states*. `0.23` is valid for a manifest value
of `0.2331`; `0.24` is not; `0.0648` is not.

**The value is deliberately not duplicated into an attribute.** Two copies of a number in
one tag drift apart, and then the check passes while the reader sees the wrong figure.

## The limit this design does NOT close, stated rather than buried

**CI cannot verify that a `requirement` ID resolves to anything.** The requirement register
lives in Notion, and ADR-0003 forbids the gate from querying it. So `"requirement":
"SR-SIM-5"` is a **label, not a checked assertion** — format-validated at most.

Do not let it read as verified. The honest options:

1. **Mirror the register into the repo** as a checked-in list the manifest validates against,
   with a named owner and a declared direction — Notion → repo, never back. This is exactly
   the pattern `ROLES.md` already uses for the Notion Escalations select.
2. **Accept the gap and mark the field unverified** in the schema.

**Recommendation: option 1**, owner COO, as a follow-up rather than a precondition. The
manifest is worth having before the register is mirrored; it is not worth pretending the
field is checked.

## Sequence

1. **Marker + fixture + `scripts/emit_claims.py`, and one real claim end-to-end** — the
   README's 0.2331 m figure. One claim proves the mechanism; ten prove nothing more.
2. **Publish `claims.json`** alongside the sim artifact on green `master`.
3. **Site-side check** in `check_page.py`, plus `data-claim` on the page's existing numbers.
   Lands as a PR into `overboard-web` with review from the Senior Digital Marketer. **I do
   not merge that half myself** — it changes their deploy gate.
4. **Requirement-register mirror**, if option 1 is taken.

Steps 1–2 are this repo's and can start now. Step 3 needs the countersign.

## Prove it rejects before trusting it

Every step lands with a **deliberate violation shown failing** — a claim whose number is
edited on the page, and a claim whose test is made to fail — not merely a green run.
`turf` and `doc-drift` both reported green for their entire existence while executing
nothing (#83). A gate is not a gate until it has been watched to reject.
