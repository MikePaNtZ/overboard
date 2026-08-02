#!/usr/bin/env bash
# AC-1: every pinned package resolves to the exact version named.
#
# Runs against the live archive with no Pi and no image builder, so it catches
# a fabricated or aged-out pin on every PR rather than at flash time. The
# design's own no-Pi table rates this HIGH confidence, and it has already
# earned it once: this is the check that caught the `rpi-2712-rt` assumption
# before it became a pin.
#
# `--dry-run` deliberately: resolving the pin proves the version exists and
# its dependencies are satisfiable, which is the whole claim. Actually
# unpacking a kernel to assert that would cost minutes per PR for no extra
# information -- verify_kernel.sh unpacks the one package where the CONTENTS
# are also load-bearing.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=pins.env
source "$HERE/pins.env"

fail=0

check_pin() {
  local spec="$1"
  echo "--- $spec"
  # `-s` is the modern spelling of --dry-run. Output goes to the log on
  # failure so an unsatisfiable pin says WHICH dependency could not be met --
  # "it failed" would send someone back to the archive by hand.
  if apt-get install -s -y "$spec" >/tmp/apt-dry.log 2>&1; then
    echo "    ok"
  else
    echo "    UNSATISFIABLE"
    sed 's/^/      /' /tmp/apt-dry.log
    fail=1
  fi
}

echo "=== AC-1: pinned packages must resolve ==="
check_pin "${KERNEL_PACKAGE}=${KERNEL_VERSION}"

for spec in $PINNED_PACKAGES; do
  [ -n "$spec" ] || continue
  check_pin "$spec"
done

# The pin that must NOT be used, asserted as a pin rather than as a comment.
# The metapackage resolving to something other than our held version is the
# NORMAL state -- it floats, that is what it is for. This step exists to make
# the gap visible in the log every run, so "why are we not just using the
# metapackage" has a standing, dated answer.
echo
echo "=== AC-2: what the floating metapackage would have given us instead ==="
floating="$(apt-cache policy linux-image-rpi-v8-rt 2>/dev/null | awk '/Candidate:/{print $2}')"
echo "    linux-image-rpi-v8-rt candidate: ${floating:-<not found>}"
echo "    held pin:                        ${KERNEL_VERSION}"
if [ "${floating:-}" = "${KERNEL_VERSION}" ]; then
  echo "    (they agree today; the pin is still what ships -- agreement is a"
  echo "     coincidence of timing, not a reason to unpin)"
fi

if [ "$fail" -ne 0 ]; then
  echo
  echo "FAIL: at least one pin is unsatisfiable."
  echo "Correct response: re-verify against the archive and MOVE the pin."
  echo "Never unpin to make this green -- an unpinned kernel invalidates every"
  echo "latency number Stage 0B produces (design section 2)."
  exit 1
fi

echo
echo "PASS: every pin resolves."
