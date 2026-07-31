#!/usr/bin/env bash
# What is waiting on ME right now.
#
#   ops/inbox.sh            report only
#   ops/inbox.sh --queue N  review is done: queue PR N for auto-merge
#
# Dispatch solved the wrong half. #38 automated STARTING work; the failure that
# actually cost this org throughput was at the END -- #94 sat finished, green
# and CLEAN for ten hours because nobody ran `gh pr merge --auto` on it, and an
# unqueued PR looks exactly like work in progress. Three times in one week.
#
# This deliberately does NOT auto-queue everything. The COO reviews every
# hand-back before it lands (ADR-0007), so blanket-queueing would trade one
# failure for a worse one. It makes finished work IMPOSSIBLE TO MISS, and
# queueing stays one explicit command.
set -uo pipefail

REPO_ROOT="$(git rev-parse --show-toplevel 2>/dev/null)" || { echo "not a git repo"; exit 2; }
cd "$REPO_ROOT" || exit 2

if [ "${1:-}" = "--queue" ]; then
  PR="${2:-}"
  [ -n "$PR" ] || { echo "usage: ops/inbox.sh --queue <pr-number>"; exit 2; }
  gh pr merge "$PR" --squash --auto && echo "queued #${PR} for auto-merge on green"
  exit $?
fi

echo "== PRs waiting on you =="
ROWS="$(gh pr list --state open --limit 50 \
          --json number,title,mergeStateStatus,autoMergeRequest,statusCheckRollup,isDraft \
          --jq '.[] | select(.isDraft | not)
                | [ .number,
                    (if .autoMergeRequest == null then "UNQUEUED" else "queued" end),
                    # Branch on .status, NOT on .conclusion being null. A check
                    # that is still running reports conclusion "" (an empty
                    # STRING, not null), so filtering on `!= null` kept it and
                    # then `"" != "SUCCESS"` classified every in-flight PR as
                    # RED. Caught within minutes of shipping, by this tool
                    # calling a green PR red -- a report nobody can trust is
                    # worse than no report.
                    ( [ .statusCheckRollup[]? ] as $all
                      | [ $all[] | select(.status == "COMPLETED") ] as $done
                      | [ $done[] | select(.conclusion != "SUCCESS" and .conclusion != "NEUTRAL" and .conclusion != "SKIPPED") ] as $bad
                      | if ($all | length) == 0 then "no-checks"
                        elif ($bad | length) > 0 then "RED"
                        elif ($done | length) < ($all | length) then "running"
                        else "green" end ),
                    .mergeStateStatus,
                    .title ]
                | @tsv')"

if [ -z "$ROWS" ]; then
  echo "  (nothing open)"
else
  ACTION=0
  while IFS=$'\t' read -r num queued checks mergestate title; do
    [ -n "$num" ] || continue
    if [ "$queued" = "UNQUEUED" ] && [ "$checks" = "green" ] && [ "$mergestate" != "DIRTY" ]; then
      echo "  >> #${num}  FINISHED AND NOT QUEUED -- ${title}"
      echo "       review it, then: ops/inbox.sh --queue ${num}"
      ACTION=$((ACTION + 1))
    elif [ "$checks" = "RED" ]; then
      echo "     #${num}  red        -- ${title}"
    elif [ "$mergestate" = "DIRTY" ]; then
      echo "     #${num}  CONFLICTS  -- ${title}"
      ACTION=$((ACTION + 1))
    else
      echo "     #${num}  ${queued}/${checks} -- ${title}"
    fi
  done <<<"$ROWS"
  [ "$ACTION" -gt 0 ] && echo "  ${ACTION} item(s) need you."
fi

echo
echo "== Issues routed to COO =="
gh issue list --state open --label "role:coo" --limit 50 \
  --json number,title --jq '.[] | "     #\(.number)  \(.title)"' \
  || echo "  (none)"
