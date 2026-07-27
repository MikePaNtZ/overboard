# ADR-0006 — One git worktree per role; never share a working directory

- **Status:** Accepted
- **Date:** 2026-07-26
- **Ratified by:** COO
- **Closes:** none — two live corruption incidents in one day; CEO asked the COO to recommend the mechanism
- **Constrains:** every role, and every dispatched subagent
- **Enforced by:** `ops/dispatch.sh` refuses a dirty or foreign tree; `policy` turf check; convention for the rest

## Context

Two incidents, same day, same root cause:

1. While the COO was mid-edit, another role's session **switched the git branch out from under it.** Uncommitted COO files were left sitting on the other role's branch, one `git add -A` away from being committed by the wrong role onto the wrong branch.
2. Digital Content Production was reported working **on top of another role's branch**, with in-progress work neither committed nor backed up.

This is the only place in this org so far where real work was actually destroyed or nearly destroyed. Everything else has been process friction.

A shared working directory makes branch state a **global mutable variable with no lock**, in a system whose defining constraint is that sessions cannot see or interrupt each other.

## Decision

1. **One `git worktree` per role, permanently.** `git worktree add ../overboard-<role> <branch>`. The main checkout at `~/projects/overboard` is not a workspace — treat it as the shared master checkout that nobody edits in.
2. **One branch prefix per role**, from `docs/decisions/ROLES.md`. Already enforced by the `policy` turf check for roles that have declared one.
3. **Every session ends with a commit.** A WIP commit on your own role branch is always correct; uncommitted work is *invisible to everyone including you next session*, and worktrees do not fix that — incident 2 was loss of uncommitted work, not a branch collision.
4. **Never two agents in one working directory.** A dispatched subagent gets its own worktree (`isolation: "worktree"`), always.
5. **`ops/dispatch.sh` refuses to run** in a tree that is dirty, or whose branch prefix does not match the dispatching role.

## Options considered

- **Convention only — "please don't switch branches."** Rejected: this *was* the convention, implicitly, and it failed twice in one day. A rule that only holds when everyone remembers it is not a mechanism.
- **A lock file naming the active session.** Rejected: another polling surface, and it is advisory — a session that does not check it is unaffected, which is exactly the session that causes the problem.
- **Worktree per role.** Chosen. Costs a little disk and one setup command per role. Cost of the losing options: silent work loss, which is the most expensive failure available to us.

## Consequences

- Each role's checkout is independent; branch switching cannot cross roles.
- Sessions must know their own worktree path. It goes in each role's `roles/<role>/CONTEXT.md`.
- Disk: one full checkout per role. Acceptable — this repo is small and the alternative is losing work.
- **Not fixed by this ADR:** a role that never commits still loses its own work. That is what rule 3 is for, and it is convention, honestly labelled.

## How this is enforced

`ops/dispatch.sh` hard-refuses a dirty or foreign tree before spawning anything. The `policy` turf check catches the *result* of a cross-role edit at PR time. Rule 3 (end-of-session commit) is convention — there is no way to check a session that never ran.
