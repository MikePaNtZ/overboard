# Landing Page — Hosting & Automatic Sim-Artifact Pipeline

<!--
covers:
  - .github/workflows/ci.yml
  - scripts/render_scenario.py
reconciled: e4f40d5
-->

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
  "peak_abs_pitch_deg": 18.64,
  "t_strike_s": 1.646,
  "speed_at_strike_ms": 1.0,
  "travel_m": 4.03,
  "mujoco_version": "3.10.0"
}
```

- The dispatch step, currently a no-op until `WEB_DISPATCH_TOKEN` exists.

### What `publish-sim-artifact` films

Two scenarios, in two separate steps so a failure in one is attributable on
sight and cannot suppress the other:

| Step | Command | Headline outputs |
|---|---|---|
| Render impulse scenario | `render_scenario.py --compare` | `impulse_open_loop.{mp4,webm,gif}`, `impulse_compare.mp4`, `impulse_poster.jpg`, `impulse_pitch.png`, `impulse_metrics.json`, `impulse_closed_loop_metrics.json` |
| Render rolling-terrain ride | `render_scenario.py --scenario terrain --compare --source sim` | `terrain_ride.{mp4,webm}`, `terrain_compare.mp4`, `terrain_poster.jpg`, `terrain_ride.png`, `terrain_compare.png`, `terrain_metrics.json`, `terrain_truth_metrics.json`, `terrain_estimate_metrics.json` |

The terrain pair is the rolling ride at 8% peak grade over 24 m, and a two-pane
comparison at 10% in which truth pitch completes the ride and the attitude
estimate does not. `sim/scenarios/terrain.py` exists in order to be filmable —
`hill.py` models a slope by rotating gravity on flat ground, which is exact for
a uniform plane and shows nothing in a render.

### Categorisation and provenance

Every artifact this pipeline produces is generated from a recorded simulator
run. It is categorised accordingly and carries a source tag burned into the
frame. **The categories are defined in exactly one place — the [shared
vocabulary](https://app.notion.com/p/3aa472a5fb6981ebaaa7cf2e996f1e8b) — and are
not restated here or in the renderer.** Two rules from it bear directly on this
pipeline:

- A Replay always names its source. There is no bare "Replay".
- The source is a **parameter** of the renderer (`--source`), not a literal, so
  the same code path can carry real telemetry without a change to it.

Each render writes a **manifest** — `impulse_render_manifest.json`,
`terrain_render_manifest.json` — alongside the clips, and both are published to
`sim-latest`. The manifest is the publishing record, kept separate from any
scenario's `metrics.json` so downstream consumers never reach into a scenario's
internal schema:

```json
{
  "schema": 1,
  "generated_at_utc": "…",
  "category": "…", "source_tag": "…",
  "vocabulary": "…",
  "scenario": "rolling terrain",
  "source": { "commit": "…", "commit_short": "…", "commit_url": "…", "run_url": "…" },
  "runs": { "ride": { "params": {…}, "metrics": {…} }, "compare_truth": {…}, "compare_estimate": {…} },
  "renderer": { "script": "scripts/render_scenario.py", "mujoco": "…", "camera": "…", "status": "ok" },
  "outputs": [ { "name": "terrain_ride.mp4", "bytes": 0, "sha256": "…" } ]
}
```

That is what makes "this clip came from that run" checkable by someone who was
not in the room: the commit, the exact scenario parameters, and a digest of
every file that left the building. The CI verification step fails the job if a
manifest names no source commit.

`sim-run.json` is unchanged and remains impulse-specific — it is the flat
caption feed for the page, not the provenance record.

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

## 3.5 Repo visibility — settled

**Both `overboard` and `overboard-web` are now public**, which is what
`CLAUDE.md` records and what the README embed relies on. The constraints below
applied while they were private and are kept for the reasoning, not as current
state; the private-repo workarounds are no longer needed:

- The deploy workflow must fetch the release assets **authenticated** (`gh
  release download -R MikePaNtZ/overboard` with a PAT), not with a plain
  `curl`. Fine — it already needs a token for the dispatch, so it is the same
  credential. Design unchanged, one extra flag.
- **Anything that renders a release asset by URL will 404 for anyone**,
  including GitHub's own image proxy. That is why the clip is not inlined in
  the repo README, and why the Notion doc embeds an interactive replay of the
  trajectory rather than the mp4 — Notion cannot fetch a private asset either.
- Once the site is public, the *clip itself* becomes public regardless of repo
  visibility, since Pages serves it from the deploy bundle. Worth being
  deliberate about: the first public artifact is a video of the board failing.

With both repos public, several things are simpler at once: README inline,
direct Notion video embed, unauthenticated deploy fetch.

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
