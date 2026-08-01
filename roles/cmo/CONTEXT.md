# CMO — working context

- **Worktree:** `~/projects/overboard-cmo` (ADR-0006). Marketing work happens in
  `overboard-web`, `overboard-viz` and `overboard-metrics`.
- **Read first:** [`docs/decisions/INDEX.md`](../../docs/decisions/INDEX.md) · [`ROLES.md`](../../docs/decisions/ROLES.md)
- **Branch prefix:** `feat/marketing/`

## Current sub-goals

- **The announcement is the only thing that matters now.** The measurement
  infrastructure is built and reports zero, honestly, because nothing has been
  published. Every downstream item — audience data, funding evidence — is
  waiting on L1, which is waiting on the CEO.
- Answer the two big unknowns with data: do strangers care, and is *how we build
  it* more interesting than the onewheel. **We can now measure the first one.**

## What is live (2026-07-31)

- `overboardproject.com`, HTTPS enforced. Apex is DNS-only/grey-cloud so GitHub
  Pages holds its certificate; only `e.` is Cloudflare-proxied. **Do not flip the
  apex to orange — it takes the site's TLS down.**
- Collector at `e.overboardproject.com/e` → D1. `/health` reports events, last
  event, reject count and last reject reason.
- Dashboard at `/dashboard?k=…`, token-gated, fails closed, 404s on a bad token.
- Daily jobs: synthetic traffic, and a GitHub traffic snapshot into
  `overboard-metrics` (private).

## Rules this role owns

- **Category on every published asset** — Footage · Sim Replay · Hardware Replay
  · Concept. A Replay names its source; a Concept carries no engineering numbers.
- **Reusable channel → start now. One-shot channel → save it for real hardware.**
  L1 should spend at most ONE rented channel; posting everywhere on one day turns
  four shots into one and tells us nothing.
- **We write for a persona, we never name one.** No "education", "students",
  "STEM" or non-profit framing on the public surface — M0's one-way door.
- **US English.** "tire", not "tyre".
- **A number on a page is a maintenance obligation.** If a count cannot be kept
  current, write something that stays true instead.
- Peer of the COO. Neither escalates to the other; both go to the CEO.

## Decisions made

- **Countersigned the COO's policy gate** (2026-07-30) with one change: hard, but
  **narrow** — it fires only on a capability claim or a numeric figure, not on
  every PR touching public paths. An advisory gate on a public surface is ignored
  within a week; a gate people must routinely bypass teaches them gates are
  decorative. Delivered that way in web PR #41.
- **Synthetic data is separated by `site` key, never by a flag.** A flag can be
  dropped by a bug, leaving rows indistinguishable from real ones forever.
- **`site` is required on every event and never defaulted.** The one irreversible
  decision in the analytics schema — unlabelled rows cannot be labelled later.
- **Content Designer writes; the SDM reviews.** The CEO's sketch had one role
  doing both, which dissolves the gate. The split is the mechanism.

## Known dead ends

- **`hill.py` cannot be filmed.** It models grade by rotating gravity on flat
  ground, so a render shows a board accelerating on level ground for no visible
  reason. `terrain.py` exists precisely to fix this. Do not ask for a hill video.
- **Flipping `analytics.js` to `sink: 'endpoint'` alone breaks everything
  silently.** The collector requires `site`; without it every event 400s, and
  `sendBeacon` cannot read a response. Verify with `/health` counts, never CI.
- **Do not verify a Cloudflare Worker within ~60s of deploy.** Both versions
  serve during rollout; a check in that window produced a confident false result.
- **`gh pr review --approve` never works here.** All roles share one GitHub
  account. Review by comment; the gate holds by discipline, not by GitHub.
- **A base64 token in a URL is mangled** — `+` decodes to a space. Bit us on the
  dashboard; the collector now reads the raw query string.

## In flight / owed

- 🔴 **Assembly capture runbook** — hardware Aug 15–28, happens once, not written.
- 🔴 **Purge synthetic data before L1 publishes.** It lives in Cloudflare; the
  local `--purge` will not reach it. Use the collector's purge job.
- Funding materials (Aug 24, P0, not started) — blocked on the hardware-evidence
  decision, which is still unowned.
- web#25 alarm · web#29 publish-race regression test · Experiments database.
