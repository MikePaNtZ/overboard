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

# --- The estate ------------------------------------------------------------
# ALL THREE REPOS, not just the one this script happens to sit in (#103).
#
# This used to run a bare `gh issue list` after cd-ing to the repo root, so it
# polled exactly one repo of three. Work filed in overboard-web or
# overboard-viz was labelled, routable in principle, and in no queue anything
# could see. Worse, --audit asserted "every open issue carries exactly one
# role: label" and reported green while auditing a third of the estate -- the
# same reports-green-while-enforcing-nothing failure this script's own comments
# were written to disown, committed by the script itself.
#
# ROLES.md already declares its registry covers all three. The actuator now
# matches the registry.
REPOS=(${OPS_REPOS:-MikePaNtZ/overboard MikePaNtZ/overboard-web MikePaNtZ/overboard-viz})

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
  local bad=0 seen=0 repo num labels count out
  for repo in "${REPOS[@]}"; do
    # A repo we cannot read is a repo whose issues are invisible. That is the
    # exact condition this audit exists to detect, so it is a hard error rather
    # than a skipped line -- otherwise a typo'd repo name reads as "all clear".
    if ! out="$(gh issue list --repo "$repo" --state open --limit 200 \
                  --json number,labels \
                  --jq '.[] | "\(.number)\t\([.labels[].name] | join(","))"' 2>&1)"; then
      echo "  ROUTING ERROR: cannot read issues in ${repo} -- ${out%%$'\n'*}"
      bad=$((bad + 1))
      continue
    fi
    while IFS=$'\t' read -r num labels; do
      [ -n "$num" ] || continue
      seen=$((seen + 1))
      if [ -z "$labels" ]; then count=0; else count="$(tr ',' '\n' <<<"$labels" | grep -c '^role:')"; fi
      if [ "$count" -ne 1 ]; then
        echo "  ROUTING ERROR: ${repo}#${num} has ${count} role: labels (need exactly 1) [${labels}]"
        bad=$((bad + 1))
      fi
    done <<<"$out"
  done
  if [ "$bad" -gt 0 ]; then
    echo "REFUSED: ${bad} routing problem(s)."
    echo "         An unlabelled issue is invisible to every poll. Label it, or"
    echo "         close it -- but do not leave it looking like backlog."
    return 1
  fi
  echo "routing: ${seen} open issue(s) across ${#REPOS[@]} repo(s), each with exactly one role: label"
  return 0
}

if [ "${1:-}" = "--audit" ]; then
  audit_routing; exit $?
fi

ISOLATED=0
ARGS=()
for a in "$@"; do
  case "$a" in
    --isolated) ISOLATED=1 ;;
    *) ARGS+=("$a") ;;
  esac
done
set -- "${ARGS[@]+"${ARGS[@]}"}"

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

# CROSS-ROLE dispatch is the normal case, not the exception -- the COO
# dispatching Senior Controls is literally what #38 asks for. This check used to
# compare the DISPATCHER's branch against the DISPATCHED role's prefix and
# refuse, which blocked the primary use case while allowing the genuinely
# dangerous one.
#
# What actually matters is whether the worker can reach this branch. Under
# `isolation: worktree` it gets its own worktree and its own branch and cannot
# touch this one, so the dispatcher's branch is irrelevant. Without isolation
# the worker commits HERE, onto a branch belonging to another role -- which is
# the real "work gets lost" failure ADR-0006 describes.
#
# So: gate on isolation, not on the dispatcher's branch.
if [ -n "$PREFIX" ] && [ "$PREFIX" != "—" ]; then
  case "$BRANCH" in
    "$PREFIX"*)
      echo "branch '${BRANCH}' carries ${ROLE}'s prefix -- same-role dispatch" ;;
    *)
      if [ "$ISOLATED" -eq 1 ]; then
        echo "cross-role dispatch: ${ROLE} (prefix '${PREFIX}') from '${BRANCH}'"
        echo "  --isolated given: worker gets its OWN worktree and branch, so it"
        echo "  cannot commit onto this one."
      else
        fail "cross-role dispatch: you are on '${BRANCH}' and dispatching ${ROLE},
         whose prefix is '${PREFIX}'.

         That is fine -- it is the normal case -- but ONLY if the worker runs in
         its own git worktree (ADR-0006). Without isolation it commits onto THIS
         branch, which belongs to another role, and the work is lost.

         Re-run with --isolated once you are spawning the agent with
         isolation: worktree, and a branch starting '${PREFIX}'."
      fi ;;
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
python3 ops/usage.py --check || fail "usage threshold reached -- do not dispatch.
         Raise OPS_USAGE_THRESHOLD_PCT deliberately, or wait."
echo "  (spend above is a FLOOR: cloud/scheduled agents are absent from it -- #79)"

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

# -E, not basic regex: BSD sed (macOS, where this is actually run) does not
# support \+, so 's/[^a-z0-9]\+/-/g' silently matched nothing and "Senior
# Controls" resolved to the label "role:senior controls" -- which cannot exist.
# The refusal was correct and the slug was wrong.
ROLE_SLUG="$(printf '%s' "$ROLE" \
  | tr '[:upper:]' '[:lower:]' \
  | sed -E -e 's/[^a-z0-9]+/-/g' -e 's/^-+//' -e 's/-+$//')"
LABEL="role:${ROLE_SLUG}"

# The label has to exist in EVERY repo, not just this one. A role addressable
# in overboard and not in overboard-web is a role whose marketing work can
# never be routed -- which is the state #103 found: neither marketing repo had
# role: labels at all until the CMO mirrored them.
MISSING=()
for repo in "${REPOS[@]}"; do
  gh label list --repo "$repo" --limit 100 --json name --jq '.[].name' 2>/dev/null \
    | grep -qx "$LABEL" || MISSING+=("$repo")
done
if [ "${#MISSING[@]}" -gt 0 ]; then
  fail "no '${LABEL}' label in: ${MISSING[*]}
         The router cannot address ${ROLE} there, so any issue filed in those
         repos is unroutable no matter how it is labelled. Labels are the
         routing field -- see #38, #103. Create it before dispatching."
fi

echo
echo "  QUEUE for ${ROLE}  (poll: gh issue list -R <repo> --label \"${LABEL}\" --state open)"
# No `2>/dev/null` here, deliberately. The first version had one, plus a
# `gh ... --jq --arg` that gh does not support -- gh has no --arg flag. The
# error went to the suppressed stderr and every marketing queue printed
# "(empty)" while two labelled issues sat in overboard-web. A silenced error
# reported as an empty queue is indistinguishable from no work, which is the
# whole failure #103 is about. The repo name is prefixed in bash instead.
FOUND=0
for repo in "${REPOS[@]}"; do
  if ! Q="$(gh issue list --repo "$repo" --state open --label "$LABEL" --limit 50 \
              --json number,title --jq '.[] | "#\(.number)  \(.title)"')"; then
    fail "cannot read ${repo}'s queue. Refusing to report an empty queue for a
         repo that failed to answer -- that is indistinguishable from no work."
  fi
  while IFS= read -r ln; do
    [ -n "$ln" ] || continue
    echo "    ${repo#*/}${ln}"
    FOUND=$((FOUND + 1))
  done <<<"$Q"
done
[ "$FOUND" -eq 0 ] && echo "    (empty -- nothing routed to this role in any of the ${#REPOS[@]} repos)"

echo
echo "OK to dispatch as ${ROLE}. Give each agent its own worktree (isolation: worktree)."
echo "Every worker reads roles/<role>/CONTEXT.md first."
echo "Completed work goes in roles/<role>/log/YYYY-MM-DD-<slug>.md -- ONE FILE PER ENTRY."
echo "Appending a work-log entry to CONTEXT.md is a CI failure since #92."
echo
echo "AFTER the hand-back: run ops/inbox.sh. A finished PR that is not queued is"
echo "indistinguishable from work in progress -- #94 sat green and unqueued for 10 hours."
