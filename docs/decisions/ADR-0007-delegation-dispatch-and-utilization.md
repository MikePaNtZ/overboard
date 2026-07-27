# ADR-0007 — Delegation downward: board → issues → dispatch

- **Status:** Accepted
- **Date:** 2026-07-26
- **Ratified by:** COO
- **Closes:** none — CEO direction, 2026-07-26
- **Constrains:** COO, CMO, Archivist and everyone reporting to them
- **Enforced by:** `ops/dispatch.sh` (concurrency cap, usage check, tree hygiene); `policy` feedstock check

## Context

The org had escalation *up* and nothing going *down*. The CEO's direction: interact directly
only with COO, CMO and Archivist — through session prompts and board-doc comments — and have
those three drive their reports, keeping engineering maximally utilized while marketing and
content are throttled when there is not enough engineering output to feed them.

The substrate constrains the answer hard. Nothing can interrupt a running session. A session
that is not running cannot be woken by a document. So "wake up the workers" cannot mean
"write something and hope."

## Decision

**The chain.** CEO comments on the board doc → COO/CMO/Archivist convert their comments into
**GitHub issues** with a written acceptance criterion and an owning role → issues are
dispatched → work returns as a **PR** → the dispatcher reviews. Completion is `Closes #N` on a
merged PR, so it is *state*, not a message.

**Dispatch is the only actuator this org has.** A running session spawning a subagent — in its
own worktree, with the issue body as its work order — is the only mechanism by which a
manager can *cause* work to happen rather than hope a session is running.

**A role is a contract, not a process.** A session's context window is a depreciating,
unreadable, unforkable asset: it compacts away, nobody else can read it, it cannot be woken,
and it dies with the window. An org cannot be built on that. So each role's durable state
lives in `roles/<role>/CONTEXT.md` — current sub-goals, decisions already made and why, known
dead ends, worktree path. Every dispatched agent reads it first and may append to it in its
PR. **This is what makes dispatch safe:** it moves standing context from a session property
to a repo property.

**Dispatchable means:** written acceptance criterion + paths inside the owning role's turf +
no open design question. If you cannot write the acceptance criterion, it is not dispatchable
work — it is a decision or a handoff, and ADR-0004 already says where those go.

**Utilization is a property of the backlog, not of token burn.** "Continually utilized" means
*the issue queue is never empty and never blocked*. It does **not** mean sessions are always
running. This distinction is load-bearing: read the other way, the directive becomes
"maximize spend", and the shared quota is the company's operating cost.

**Throttling needs no mechanism; un-throttling does.** A session that is not running is
already throttled to zero — the substrate has no ambient work. So the marketing throttle is
simply the COO not dispatching marketing issues, plus:

- **Dispatch-side (convenience):** a content/marketing issue is dispatchable only if it names
  satisfied feedstock — `Feedstock: #<merged PR>` or a delivered artefact.
- **Merge-side (law):** a PR touching public-facing paths must carry a `Feedstock:` reference.
  The dispatch gate is porous by construction — the CEO prompts the CMO session directly, which
  routes around the dispatcher entirely — so the binding gate is on the merge, where it holds
  regardless of who started the work.

**Hard limits, in the script and not in this document:**

- **At most 3 concurrent dispatches.**
- `ops/usage.py --check` runs before every dispatch and can halt it.
- Refuses a dirty or foreign worktree (ADR-0006).

## Options considered

- **Cron-scheduled agents per role.** Kept as the option for unattended runs, not the default:
  a cron agent that wakes with nothing dispatchable burns quota to discover that.
- **A self-paced event loop watching usage.** **Rejected** — see ADR-0007a note below. It is a
  control loop with no actuator, and a long-lived session pays a cache read on its entire
  window every tick. Measured: cache reads already dominate this org's spend by ~100:1 against
  output tokens. The sensor moved next to the actuator instead.
- **Dispatch to fresh subagents with no context files.** Rejected: that genuinely does destroy
  role knowledge. `roles/*/CONTEXT.md` is the fix and is a precondition of this ADR.

## Consequences

- The COO becomes the throughput bottleneck for engineering dispatch, deliberately — review
  capacity is the real constraint, and an unreviewed PR is not delivery.
- Issues become the backlog of record. An empty issue queue is now a *reportable condition*,
  not a quiet state.
- Marketing genuinely can be starved by design when engineering has not shipped. That is the
  intent, stated so nobody mistakes it for neglect.

## The failure mode this is built to avoid

**Quota exhaustion by dispatch fan-out.** "Keep everyone continually utilized" plus the
ability to spawn arbitrary parallel subagents plus one shared quota — *the same quota the CEO
uses* — is a chain already pointed at our foot. A subagent costs meaningful tokens just to
boot. Ten parallel dispatches iterating on PRs could hard-stop the company, including its CEO,
by lunchtime, having merged a pile of process documents and ordered zero hardware. Hence the
cap of 3, the pre-dispatch usage check, and the backlog definition of utilization.

## How this is enforced

`ops/dispatch.sh` for the cap, the usage check and tree hygiene. The `policy` job for the
feedstock rule on public-facing paths. The rest is `ops/dispatch.md`, which is documentation
*of* the script rather than a substitute for it.
