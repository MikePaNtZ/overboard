# Role context files

**A role is a contract, not a process.** A session's context window is a depreciating,
unreadable, unforkable asset — it compacts away, nobody else can read it, it cannot be woken,
and it dies with the window. An org cannot be built on that.

So each role's durable state lives here, in the repo:

- **Every dispatched agent reads its role's `CONTEXT.md` first.**
- **Every dispatched agent may append to it in its PR** — decisions made, dead ends hit.
- This is what makes dispatch to a fresh agent safe rather than amnesiac (ADR-0007).

Keep each file short. It is a working brief, not a history. Move anything that has become a
decision into `docs/decisions/` and link it.

Roles and their turf: [`docs/decisions/ROLES.md`](../docs/decisions/ROLES.md).
Ownership of a path: `python3 .github/policy_check.py --who <path>`.
