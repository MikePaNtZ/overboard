# Landing Page — Hosting & Automatic Sim-Artifact Pipeline

**Status: design only. Nothing here is implemented.** Phases A (sim scenario)
and B (CI renders + publishes) have shipped; this is the deferred Phase C.

It lives in the controls repo because it is a *contract*: what
`MikePaNtZ/overboard` publishes and what `MikePaNtZ/overboard-web` may consume.
Implementation belongs in the web repo.

Governing Notion docs: [M0 Product & Marketing](https://app.notion.com/p/3a8472a5fb6981ffbf73ee8297e62f07),
[M2 Landing Page & Instrumentation](https://app.notion.com/p/3a8472a5fb6981bb9574d7fa6caa1304),
[Requirements](https://app.notion.com/p/3a8472a5fb69817f98ebc9e52e1fb2d8) (`SR-WEB-*`).

---

## 1. The decision that has to be made first

**The demo section of the landing page currently reads "First balance, in
simulation."** The artifact this pipeline publishes shows the board *falling
over*. Wiring the two together as-is publishes a video that contradicts its own
headline — a straight `SR-WEB-4` lock-step violation, and on a page whose entire
credibility rests on not letting marketing outrun the code.

There is no way to automate around this. Either the copy changes or the clip
does not go on the page.

**Recommendation — change the copy, and make it the stronger story:**

> ### Disturbance response, in simulation.
> Kick a driverless onewheel with a 20 N·s impulse and, with no controller, it
> rolls four metres and noses into the ground at 1.0 m/s. It physically cannot
> tilt past 18.6° — the bumper is on the ground first. That number is the
> margin the balance controller has to hold, and this clip is the baseline it
> has to beat.

That is honest, specific, and more interesting to the target audience than a
balance claim that is not yet true. When closed-loop lands, the same pipeline
swaps in a recovery clip and the copy earns its upgrade.

**Second rule, structural:** the page states the *regime*, the artifact states
the *run*. All run-specific numbers come from `sim-run.json` at deploy time, so
no automated update can ever change what the page claims — only what it
reports. This is what makes an auto-updating page safe.

---

## 2. Hosting

**Cloudflare Pages, deployed by GitHub Actions.** Already the top recommendation
in the web repo's README, and it keeps Workers + D1 available on the same origin
for the analytics sink and the email form — no CORS, no third party.

Actions-driven rather than Cloudflare's native Git integration, because the
deploy needs a build step that fetches the sim artifact. With the Git
integration, Cloudflare builds from the repo alone and there is no hook to pull
an external asset.

**Domain:** buy through Cloudflare Registrar — at-cost pricing, free WHOIS
privacy, and the zone is configured automatically. Add it as a Pages custom
domain; TLS is automatic. Apex + `www` redirect to one canonical host.

Do the domain purchase **early and in parallel** — DNS and certificate issuance
are wall-clock, not work.

---

## 3. Artifact flow

```
overboard (controls)                          overboard-web
─────────────────────                         ─────────────
push to master
  └─ sim gate (physics, every push)
  └─ publish-sim-artifact  [green only]
       ├─ render mp4/webm/gif/poster
       ├─ write sim-run.json
       ├─ upload → rolling release `sim-latest`
       └─ repository_dispatch ─────────────►  deploy workflow
                                                ├─ checkout
                                                ├─ curl sim-latest assets → assets/
                                                ├─ wrangler pages deploy .
                                                └─ live
```

**The artifact never enters the web repo's git history.** It is fetched at
deploy time. The web repo stays two files with no build step; the coupling is a
URL plus a JSON of facts, which is consistent with the "coupled by facts, not
code" boundary rule in `CLAUDE.md`.

Triggers for the deploy workflow: `push`, `repository_dispatch: sim-artifact`,
and `workflow_dispatch` (manual).

### Already built (Phase B)

- Rolling release `sim-latest` with stable per-asset URLs:
  `https://github.com/MikePaNtZ/overboard/releases/download/sim-latest/<name>`
- `sim-run.json` — flat, versioned (`schema: 1`), deliberately separate from
  `impulse_metrics.json` so page copy never reaches into the scenario's internal
  schema:

```json
{
  "schema": 1,
  "commit": "abc1234",
  "commit_url": "…", "run_url": "…",
  "scenario": "impulse disturbance response",
  "regime": "open-loop (no controller)",
  "impulse_ns": 20.0,
  "nose_strike_angle_deg": 18.57,
  "peak_pitch_deg": 18.64,
  "t_strike_s": 1.646,
  "speed_at_strike_ms": 1.0,
  "travel_m": 4.03,
  "mujoco_version": "3.10.0"
}
```

- The dispatch step, currently a no-op until `WEB_DISPATCH_TOKEN` exists.

### Remaining work

| # | Task | Repo |
|---|---|---|
| C1 | Cloudflare Pages project + Actions deploy workflow (fetch assets → `wrangler pages deploy`) | web |
| C2 | Domain via Cloudflare Registrar; custom domain, apex/www redirect | — |
| C3 | Replace `.media-empty` with the `<video>` already sitting commented out at `index.html:401`; rewrite the demo copy per §1 | web |
| C4 | Caption rendered from `sim-run.json`, with graceful fallback when absent | web |
| C5 | `_headers` (cache-control, security headers), `404.html`, robots/sitemap, real canonical + OG URLs | web |
| C6 | Update M2 + Requirements in the same pass; add a requirement that the clip is a CI artifact of current mainline | Notion |

### Credentials needed

- Cloudflare account + API token (Pages:Edit) → `CLOUDFLARE_API_TOKEN`, `CLOUDFLARE_ACCOUNT_ID`
- Fine-grained GitHub PAT, `contents:write` on `overboard-web` → `WEB_DISPATCH_TOKEN` on `overboard`
- The domain purchase

---

## 4. Constraints the implementation must respect

- **`file://` must keep working.** The web repo's rule is no build step, no
  framework, no dependencies. The caption reads `assets/sim-run.json` and
  degrades to the existing `.media-empty` block when it is missing, so opening
  `index.html` from disk still works.
- **Cache headers matter.** `sim-run.json` and the video share a URL across
  deploys; without an explicit `Cache-Control`, visitors get a stale clip
  against fresh copy. Set a short max-age on `assets/*`, or fingerprint the
  filenames at deploy time.
- **Never publish behind a red gate.** Already enforced in Phase B — the
  publish job `needs: [sim, rust]` and is mainline-push only.
- **A page claim needs a requirement behind it** (`SR-WEB-4`). Auto-updating
  the *evidence* is fine; auto-updating a *claim* is not.

---

## 5. Deliberately not doing

- **Committing artifacts into the web repo.** Binary churn on every mainline
  push, for no gain over a deploy-time fetch.
- **Fetching the clip from GitHub at page load.** Third-party request on the
  critical path, breaks the no-CDN rule, and couples page availability to the
  GitHub releases CDN.
- **Cloudflare R2 for artifact storage.** A GitHub Release is free, already
  authenticated, and needs no new account. Revisit only if artifacts outgrow
  release limits.
