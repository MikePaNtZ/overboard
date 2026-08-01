# 2026-07-31 — the site went live and the measurement spine was built

Went from "no domain, no collection, no dashboard" to all three running and
verifying themselves. Written up because the *failures* are the transferable
part, not the shipping.

**Live:** `overboardproject.com` with HTTPS · collector at
`e.overboardproject.com` storing to D1 · token-gated dashboard on real data ·
daily synthetic traffic · daily GitHub traffic snapshot into a private repo.

**Five things looked wired and were not, in one day:**

1. `analytics.js` had been fully instrumented for weeks with `sink: 'console'`,
   sending nowhere. Events scrolled past convincingly in devtools.
2. `CNAME` was committed to `main` while the site is served from `gh-pages`, and
   `publish.sh` wipes that root on every deploy — so a domain set through the
   Pages UI would have been deleted by the next push.
3. `og:image` pointed at `/og.png`, which had never existed. The share card for
   the announcement was blank.
4. The production deploy was cancelled by its own preview-cleanup job: both sat
   in one concurrency group, and GitHub allows only one *pending* run per group.
   `cancel-in-progress: false` does not prevent this — it governs runs already in
   progress. Merges looked green and deployed nothing.
5. The collector validated per *batch*, so one unrecognised event name binned the
   whole batch. The page flushes once on `pagehide`, so any visitor who used the
   email form lost their entire session — silently, because `sendBeacon` cannot
   read a response.

**The pattern:** every one was invisible from inside. Four were found by going
and looking; only #5's class was caught by a machine, and only because the deploy
verifies its effect from outside rather than trusting its exit code.

**What changed as a result:** every deploy now checks the live result — posts a
real event and confirms the stored count rose, and posts a deliberately invalid
one to confirm rejection still works. `/health` reports rejects as well as
liveness, because liveness tells you the collector is up, not that it is
accepting what is sent to it.

**Two fixes of mine were worse than the bug**, both caught by others: my deploy
retry used `{` not `(`, so `cd` leaked between attempts and a retry would have
published a blank site over a working one; and my collector created its schema
in the POST handler, so `/health` — the one endpoint queried before traffic
exists — was guaranteed to fail.

**Reviews caught more than I did.** The SDM found two defects in the Voice &
Style Guide I wrote, where worked examples demonstrated the opposite of a rule
stated above them, and found the batch-poisoning bug. DCP refused a re-render
rather than substituting other footage, and found three published clips that
cannot name the run they came from. That is the writer/reviewer split earning
its cost on its first real outing.
