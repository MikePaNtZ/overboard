#!/usr/bin/env bash
# AC-2 evidence: the pinned kernel really ships what the design assumes.
#
# This automates, on every PR, exactly what was done by hand on 2026-07-27 and
# recorded in docs/design-pi-image-stage0b-verification.md. The design says so
# in as many words -- "this is exactly what the implementation must automate".
#
# A resolving pin (verify_pins.sh) proves the version EXISTS. It says nothing
# about the contents, and the contents are where the design's load-bearing
# claims live: that a generic 4K-page v8-rt kernel is still a complete Pi 5
# kernel, and that the CAN stack we depend on is actually in it. Those were
# established by unpacking the .deb, so that is what gets repeated here.
#
# Downloads and unpacks a ~100 MB kernel. Runs only when the pins or these
# scripts change -- see the workflow's `paths:` filter.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=pins.env
source "$HERE/pins.env"

WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT
cd "$WORK"

echo "=== downloading ${KERNEL_PACKAGE}=${KERNEL_VERSION} ==="
# `apt-get download` fetches AND verifies the SHA256 against the signed
# Packages index. Doing the checksum by hand afterwards would re-check what
# apt already checked, against a value read from the same index -- proving
# only that we can read a file twice.
apt-get download "${KERNEL_PACKAGE}=${KERNEL_VERSION}"
deb="$(ls -1 ./*.deb)"
echo "    $deb"
echo "    sha256: $(sha256sum "$deb" | cut -d' ' -f1)"

echo
echo "=== unpacking ==="
dpkg-deb -x "$deb" ./root
dpkg-deb -e "$deb" ./root/DEBIAN

fail=0

echo
echo "=== required modules and device tree ==="
for f in $KERNEL_REQUIRED_FILES; do
  # -name rather than an exact path: the module tree is versioned, and
  # hardcoding lib/modules/<version>/... would make every kernel bump a
  # two-place edit with one of them easy to miss.
  #
  # The suffix glob matters and its absence produced a FALSE ALARM on run 2:
  # this kernel ships modules COMPRESSED (mcp251xfd.ko.xz), so an exact
  # `-name mcp251xfd.ko` matched nothing and the script announced a
  # "DESIGN-INVALIDATING result" about a kernel that ships exactly what the
  # design says it does. The config flags were green in the same run, which
  # is what gave the lie away -- CONFIG_CAN_MCP251XFD=m and no module file is
  # not a state a real kernel package can be in.
  if found="$(find ./root \( -name "$f" -o -name "$f.xz" -o -name "$f.zst" \
                             -o -name "$f.gz" \) -print -quit)" && [ -n "$found" ]; then
    echo "    ok       $f  ->  ${found#./root}"
  else
    echo "    MISSING  $f"
    fail=1
  fi
done

echo
echo "=== shipped kernel config ==="
config="$(find ./root/boot -name 'config-*' -print -quit || true)"
if [ -z "$config" ]; then
  echo "    MISSING: the .deb ships no /boot/config-*, so none of the flags below"
  echo "             can be checked. Treat every config claim as unverified."
  fail=1
else
  echo "    $(basename "$config")"
  for flag in $KERNEL_REQUIRED_CONFIG; do
    if grep -qx "$flag" "$config"; then
      echo "    ok       $flag"
    else
      echo "    MISSING  $flag"
      # What the config actually says, so a changed default is diagnosable
      # from the log instead of needing a local re-download.
      key="${flag%%=*}"
      echo "             actual: $(grep -E "^($key=|# $key )" "$config" || echo "<absent>")"
      fail=1
    fi
  done

  # Recorded, deliberately NOT asserted. Design section 4 accepts both of
  # these as the known cost of there being no rpi-2712-rt: 4K pages instead
  # of 16K, and the isolated core still taking the 250 Hz tick. They are
  # stated in advance and included in the measurement rather than discovered
  # mid-debug -- so the build must not fail on them, but the log must show
  # them every time.
  echo
  echo "=== known costs (recorded, not asserted -- design section 4) ==="
  echo "    $(grep -E '^CONFIG_ARM64_(4K|16K|64K)_PAGES=' "$config" || echo '<no page size flag>')"
  echo "    $(grep -E '^CONFIG_HZ=' "$config" || echo '<no CONFIG_HZ>')"
  echo "    NO_HZ_FULL: $(grep -E '^(CONFIG_NO_HZ_FULL=|# CONFIG_NO_HZ_FULL )' "$config" || echo '<absent, as expected>')"
fi

echo
if [ "$fail" -ne 0 ]; then
  echo "FAIL: the pinned kernel does not ship what the design assumes."
  echo "This is a DESIGN-INVALIDATING result, not a flaky build: docs/design-pi-image-stage0b.md"
  echo "section 4 and its verification log are wrong, or the archive has changed under us."
  echo "Do not adjust the assertions to match. Re-verify and amend the design."
  exit 1
fi

echo "PASS: the pinned kernel ships the CAN stack, the Pi 5 device tree, and CONFIG_PREEMPT_RT."
