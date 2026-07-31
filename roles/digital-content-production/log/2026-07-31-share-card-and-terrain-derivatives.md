# 2026-07-31 — the share card, the terrain derivatives, and a caption I got wrong

## Headline

Two of this role's four open issues shipped. The announcement's share card had
been a **broken image since the page was written** — `index.html` declared an
`og:image` that never existed — and the rolling-terrain clips the page wants
were still sitting in a CI release rather than in `assets/`.

Both deliveries were decided by constraints rather than taste, which is the part
worth keeping.

## What shipped

| PR | What |
|---|---|
| [overboard-viz#12](https://github.com/MikePaNtZ/overboard-viz/pull/12) | `og_card.py` — builds the share card from a published, hashed Sim Replay frame |
| [overboard-web#20](https://github.com/MikePaNtZ/overboard-web/pull/20) | `assets/og.png` delivered, with provenance sidecar |
| [overboard-viz#13](https://github.com/MikePaNtZ/overboard-viz/pull/13) | `web_derivatives.py` — page-ready encodes of the terrain ride and the compare |
| [overboard-web#21](https://github.com/MikePaNtZ/overboard-web/pull/21) | both clips delivered, own poster and sidecar each |
| [overboard-viz#14](https://github.com/MikePaNtZ/overboard-viz/pull/14) | caption correction — see below |
| [overboard-web#22](https://github.com/MikePaNtZ/overboard-web/pull/22) | corrected sidecar re-delivered, media byte-identical |

## The three things that decided the work

**1. The source was forced, not chosen.** Every board-riding pose track in
`overboard-viz` predates the IMU frame-map correction — all land 2026-07-26
before 15:08 −0700, and the fix is `5c1d11c` at 15:08:24. Anything rendered from
them puts a pre-fix trajectory on published assets. The `sim-latest` terrain
artifacts are the only post-fix board-riding imagery that exists. That is now in
CONTEXT.md as a dead end because it will constrain the next delivery too.

**2. The category rule beat the better-looking option.** The V1.3 dusk waterfront
masters are far stronger thumbnails than any MuJoCo frame. They are **Concept**,
the standing quota says Concept never carries a milestone, and the announcement
is the milestone. Decided in about a minute once the rule was applied, which is
the argument for having the rule.

**3. The share card's framing had an invisible failure mode.** The obvious crop
is "past the HUD column" for clean terrain. The HUD sits to the board's *left*,
so that framing pushes the board to the frame edge — exactly where the centre
square crop that several social surfaces apply discards it. Nothing in the
1200×630 file reveals the problem; only the square rendering is empty ground.
The board is centred instead and the HUD comes along, which is a fair trade
because `LOCAL GRADE 8.0% DESCENDING` is the one figure that reads at thumbnail
size and it is backed by the run. Pinned by a test rather than a comment.

## The caption I got wrong

I described the compare clip as *"the same 10% descent twice"* and shipped that
wording into a delivered provenance sidecar.

It is wrong. The manifest's `struck_phase: "descent"` is a **classifier
artefact** — the scenario buckets negative travel into that phase. The run never
reaches the descent. Both runs drift backwards off the start crest during the
2 s settle; truth pitch recovers and rides on to the next crest at 24 m, and the
estimate puts the nose in at 3.28 s while still 0.87 m *behind* the start.

**This was already written down.** It was an entry in this role's own CONTEXT.md,
added by an earlier session, and I shipped before reading it. Corrected in
viz#14 / web#22 within the hour.

The durable fix is not "read the file next time". The sidecar now carries a
`caption_warning` field naming the trap, because **the role that writes the
caption is not the role that watched the run** — the Senior Digital Marketer
reads the sidecar, not this log. Guidance that only exists in a context file
protects only the people who read that context file.

## What I got wrong about process, too

I queued `overboard#105` with `--squash --auto` and reported it as done. It was
**red** — the `policy` gate rejected this session's CONTEXT.md edit as an
appended work-log (+54/−6, net 48, against a limit of 12). The gate was right;
this file is the fix.

"Never poll your own PR" means do not sit watching a green build land. It does
not mean never look at whether the build went green at all. A queued red PR
merges never, and reporting it as queued reads identical to reporting it as fine.

## Constraints found, worth not rediscovering

- **A VP9 companion to the terrain clips encodes *larger* than the H.264**, because
  the published `.webm` is already VP9 and re-encoding is a second lossy
  generation. `assets/` is mp4-only for the same reason.
- **CRF is not a useful size lever on this footage.** 26/28/30 land at
  2.3/2.0/2.0 MB against 2.3 MB at 23 — the hatched ground shading is
  high-frequency detail the encoder cannot cheaply discard. With
  `preload="none"` nothing downloads until play, so quality wins.
- **The compare's poster is its closing frame**, where both panes carry their
  outcome at once: truth at the next crest, estimate nose-down at 3.28 s. The
  argument of the clip is legible before anyone presses play.

## Left open

- **overboard-viz#11 is not closed by the asset landing.** `index.html:15` still
  points `og:image` at `https://overboard.example/og.png` — placeholder domain,
  root path. Until the Senior Digital Marketer updates it the card still does not
  render. It must be an absolute URL, so it wants to land with
  `feat/web/custom-domain`.
- **overboard-viz#6** (post-frame-map-fix shuttle re-render) is also the only
  route to this repo owning *any* post-fix board-riding track. Worth doing before
  #4 for that reason alone.
