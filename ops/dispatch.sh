#!/usr/bin/env bash
# Pre-dispatch gate. Run this BEFORE spawning any worker agent.
#
# This is the only actuator the org has, so it is also the only place worth
# putting a limit. See docs/decisions/ADR-0007.
#
#   ops/dispatch.sh <role> [issue-number ...]   pre-flight gate, then the queue
#   ops/dispatch.sh --audit                     routing integrity only, no role
#
# Exits non-zero and explains itself if dispatching now would be unsafe.
set -uo pipefail

# --- 0. Routing integrity -------------------------------------------------
# Every open issue must carry EXACTLY ONE role: label. Zero labels means the
# issue is invisible to every cron -- nobody polls it and it is not "backlog",
# it is lost. Two labels means two routines both pick it up, which is the bug
# the old --assignee design had (all eight roles share one GitHub account, so
# --assignee could never discriminate) wearing a new costume.
#
# A hard error, never a silent skip. This week produced two checks that
# reported green while enforcing nothing; this is not going to be the third.
audit_routing() {
  local bad=0 line num labels count
  while IFS=$'\t' read -r num labels; do
    [ -n "$num" ] || continue
    if [ -z "$labels" ]; then count=0; else count="$(tr ',' '\n' <<<"$labels" | grep -c '^role:')"; fi
    if [ "$count" -ne 1 ]; then
      echo "  ROUTING ERROR: issue #${num} has ${count} role: labels (need exactly 1) [${labels}]"
      bad=$((bad + 1))
    fi
  done < <(gh issue list --state open --limit 200 \
             --json number,labels \
             --jq '.[] | "\(.number)\t\([.labels[].name] | join(","))"')
  if [ "$bad" -gt 0 ]; then
    echo "REFUSED: ${bad} open issue(s) are unroutable."
    echo "         An unlabelled issue is invisible to every poll. Label it, or"
    echo "         close it -- but do not leave it looking like backlog."
    return 1
  fi
  echo "routing: every open issue carries exactly one role: label"
  return 0
}

if [ "${1:-}" = "--audit" ]; then
  audit_routing; exit $?
fi

ROLE="${1:-}"
shift || true
REPO_ROOT="$(git rev-parse --show-toplevel 2>/dev/null)" || { echo "not a git repo"; exit 2; }
cd "$REPO_ROOT" || exit 2

MAX_CONCURRENT="${OPS_MAX_CONCURRENT:-3}"
fail() { echo "REFUSED: $*"; exit 1; }

[ -n "$ROLE" ] || { echo "usage: ops/dispatch.sh <role> [issue ...]"; exit 2; }

# --- 1. The role must exist in the registry -------------------------------
grep -q "^| \`${ROLE}\` |" docs/decisions/ROLES.md \
  || fail "role '${ROLE}' is not in docs/decisions/ROLES.md (ADR-0005)"

# --- 2. Tree hygiene (ADR-0006) -------------------------------------------
# Uncommitted work is invisible to everyone including you next session, and a
# dispatched agent in a dirty tree can commit someone else's work by accident.
if [ -n "$(git status --porcelain --untracked-files=no)" ]; then
  fail "working tree is dirty. Commit your WIP first -- ADR-0006 rule 3.
         Uncommitted work is invisible to every other session, and a dispatched
         agent here could commit it onto the wrong branch."
fi

BRANCH="$(git rev-parse --abbrev-ref HEAD)"
PREFIX="$(awk -F'|' -v r="\`${ROLE}\`" '$2 ~ r {gsub(/[` ]/,"",$5); print $5}' docs/decisions/ROLES.md | head -1)"
if [ -n "$PREFIX" ] && [ "$PREFIX" != "—" ]; then
  case "$BRANCH" in
    "$PREFIX"*) : ;;
    *) fail "branch '${BRANCH}' does not carry ${ROLE}'s prefix '${PREFIX}'.
         Dispatching from another role's branch is how work gets lost (ADR-0006)." ;;
  esac
fi

# --- 3. Concurrency cap ---------------------------------------------------
# Quota exhaustion by fan-out is the failure mode most likely to stop this org:
# one shared quota, and the CEO draws on it too.
ACTIVE="$(git worktree list --porcelain | grep -c '^worktree ' || echo 0)"
echo "worktrees checked out: ${ACTIVE}  (cap on concurrent dispatches: ${MAX_CONCURRENT})"
if [ "$#" -gt "$MAX_CONCURRENT" ]; then
  fail "asked to dispatch $# issues at once; cap is ${MAX_CONCURRENT}.
         Dispatch in waves and review between them -- an unreviewed PR is not delivery."
fi

# --- 4. Usage check -------------------------------------------------------
python3 ops/usage.py --check || fail "daily usage ceiling reached -- do not dispatch.
         Raise OPS_USAGE_CEILING_M deliberately, or wait."

# --- 5. Agent-type fit ----------------------------------------------------
# Learned the hard way: sonnet-executor has Read/Write/Edit/Bash/Grep/Glob and
# NOTHING ELSE -- no ToolSearch, no WebSearch, no MCP. A research or Notion
# task handed to it fails instantly, having spent ~22k tokens discovering that
# its own toolbox is empty. Match the agent type to the work BEFORE spawning.
cat <<'FIT'

  AGENT-TYPE FIT -- check before you spawn:
    repo edits, tests, builds, refactors ....... sonnet-executor  (no web, no MCP)
    web research, Notion, anything needing MCP .. general-purpose  (all tools)
    read-only search across many files ......... Explore
    a bounded judgement call ................... opus5-oracle     (read-only)
  Wrong type = a guaranteed no-op that still costs a full agent boot.
FIT

# --- 6. Dispatchability ---------------------------------------------------
# A work request without an acceptance criterion is a decision or a handoff in
# disguise (ADR-0004). Checked by eye against the issue body; reminded here.
for issue in "$@"; do
  echo "  issue #${issue}: confirm it has an acceptance criterion, paths inside"
  echo "                  ${ROLE}'s turf, and no open design question."
done

# --- 7. The queue, from the label router ----------------------------------
# `gh issue list --label` is the poll a cron routine runs. Prove it routes
# BEFORE trusting it: audit first, then show exactly what this role would get.
audit_routing || exit 1

ROLE_SLUG="$(printf '%s' "$ROLE" \
  | tr '[:upper:]' '[:lower:]' \
  | sed -e 's/[^a-z0-9]\+/-/g' -e 's/^-//' -e 's/-$//' \
        -e 's/^sr-mechanical-and-systems$/sr-mechanical-systems/')"
LABEL="role:${ROLE_SLUG}"

if ! gh label list --limit 100 --json name --jq '.[].name' | grep -qx "$LABEL"; then
  fail "no '${LABEL}' label exists. The router cannot address ${ROLE}.
         Labels are the routing field -- see #38. Create it before dispatching."
fi

echo
echo "  QUEUE for ${ROLE}  (poll: gh issue list --label \"${LABEL}\" --state open)"
QUEUE="$(gh issue list --state open --label "$LABEL" --limit 50 \
           --json number,title --jq '.[] | "    #\(.number)  \(.title)"')"
if [ -z "$QUEUE" ]; then
  echo "    (empty -- nothing routed to this role)"
else
  echo "$QUEUE"
fi

echo
echo "OK to dispatch as ${ROLE}. Give each agent its own worktree (isolation: worktree)."
echo "Every worker reads roles/<role>/CONTEXT.md first."
echo "Completed work goes in roles/<role>/log/YYYY-MM-DD-<slug>.md -- ONE FILE PER ENTRY."
echo "Appending a work-log entry to CONTEXT.md is a CI failure since #92."
echo
echo "AFTER the hand-back: run ops/inbox.sh. A finished PR that is not queued is"
echo "indistinguishable from work in progress -- #94 sat green and unqueued for 10 hours."
