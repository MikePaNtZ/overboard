#!/usr/bin/env bash
# I0 -- the D3 spike. Does `rpi-image-gen` build AT ALL in our CI shape?
#
# Design section 3 chose `rpi-image-gen` over `pi-gen`, and gated that choice
# on one question it refused to settle by argument: the tool documents
# "Debian Bookworm and Trixie arm64 as the supported native hosts", needs
# CAP_SYS_ADMIN, and says containers are "not formally supported". Our shape
# is a privileged debian:trixie container on a hosted ubuntu-24.04-arm runner
# -- close to supported, not identical, and the only unsupported axis is "in
# a container". That is a real risk, and half a day of CI settles it.
#
# GREEN  -> proceed with rpi-image-gen.
# RED    -> take the derive-from-stock fallback (reference doc section 2).
#           Explicitly NOT pi-gen: a lateral move, same class of uncertainty,
#           plus rebuilding a whole distribution we do not need.
#
# THIS SCRIPT'S JOB IS TO PRODUCE INFORMATION, NOT TO PASS.
#
# A spike that fails having printed nothing has cost the half day and bought
# nothing. So the tool's actual interface -- its README, its layout, its
# example configs -- is dumped BEFORE the build is attempted, unconditionally.
# If the build then fails because this script guessed the entrypoint wrong,
# the log still contains what the right entrypoint is, and the next attempt
# is a one-line edit rather than another half day.
set -uo pipefail

REPO="https://github.com/raspberrypi/rpi-image-gen"
WORK="/tmp/spike"

section() { echo; echo "=============== $* ==============="; echo; }

section "host and container"
echo "uname:    $(uname -a)"
echo "id:       $(id)"
echo "os:       $(grep PRETTY_NAME /etc/os-release | cut -d= -f2-)"
# The three capabilities the tool's host requirements actually turn on. If
# loop devices are missing, nothing below can work and the failure is
# attributable to the runner rather than to the tool.
echo "loop dev: $(ls -l /dev/loop-control 2>&1 | head -1)"
echo "cap:      $(grep CapEff /proc/self/status 2>/dev/null || echo '<unknown>')"

section "install prerequisites"
export DEBIAN_FRONTEND=noninteractive
apt-get update -qq
# Deliberately unpinned: this is throwaway spike scaffolding, not the image.
# AC-1's pinning rule governs what lands ON THE CARD -- pinning the spike's
# own git client would be ceremony, and would make the spike fail for reasons
# unrelated to the question it is asking.
apt-get install -y --no-install-recommends \
  git ca-certificates sudo kmod parted fdisk dosfstools e2fsprogs \
  xz-utils zstd bc python3 >/dev/null 2>&1
echo "ok"

section "clone $REPO"
rm -rf "$WORK"
git clone --depth 1 "$REPO" "$WORK" || { echo "CLONE FAILED - spike cannot proceed"; exit 1; }
cd "$WORK"
echo "commit: $(git rev-parse --short HEAD)"

# ---------------------------------------------------------------------------
# Everything from here is the information the spike exists to produce. It runs
# whether or not the build later succeeds.
# ---------------------------------------------------------------------------
section "repository layout"
ls -la

section "README (first 120 lines)"
head -120 README.md 2>/dev/null || echo "<no README.md>"

section "candidate entrypoints"
find . -maxdepth 2 -type f \( -name '*.sh' -o -name 'Makefile' -o -name 'meson.build' \) \
  -not -path './.git/*' | sort | head -40

section "example / stock configurations"
find . -maxdepth 3 \( -path ./.git -prune \) -o \
  \( -type d -name '*config*' -o -type d -name '*example*' -o -type f -name '*.cfg' \) -print \
  2>/dev/null | grep -v '^./.git' | sort | head -40

section "documented dependencies"
for f in install_deps.sh depends requirements.txt; do
  [ -e "$f" ] && { echo "--- $f"; head -40 "$f"; }
done

# ---------------------------------------------------------------------------
# The attempt. Guessed from the tool's own conventions; if the guess is wrong
# the sections above are what makes the next attempt cheap.
# ---------------------------------------------------------------------------
section "dependency install (tool's own script, if it ships one)"
if [ -x ./install_deps.sh ]; then
  ./install_deps.sh 2>&1 | tail -30
  echo "install_deps.sh exit: $?"
else
  echo "<no install_deps.sh - skipping>"
fi

section "BUILD ATTEMPT"
build_rc=1
if [ -x ./build.sh ]; then
  echo "$ ./build.sh"
  ./build.sh 2>&1 | tail -120
  build_rc="${PIPESTATUS[0]}"
else
  echo "no ./build.sh found - the entrypoint guess was wrong."
  echo "See 'candidate entrypoints' above for what this repo actually ships."
fi
echo "build exit: $build_rc"

section "did an image appear?"
# The spike's actual acceptance criterion, from design section 3: "produces
# any .img at all". Not a correct image, not our image -- any image. Anything
# stronger would be testing the tool instead of testing our host shape.
mapfile -t imgs < <(find . -name '*.img' -o -name '*.img.xz' -o -name '*.img.zst' 2>/dev/null | head -10)
if [ "${#imgs[@]}" -gt 0 ]; then
  for i in "${imgs[@]}"; do echo "    $(ls -lh "$i" | awk '{print $5, $9}')"; done
  echo
  echo "SPIKE RESULT: GREEN - rpi-image-gen produced an image in our CI shape."
  echo "Design section 3's primary choice stands; proceed to I1."
  exit 0
fi

echo "    <none>"
echo
echo "SPIKE RESULT: RED - no image produced in our CI shape."
echo
echo "This is a RESULT, not a broken build. Read the sections above before"
echo "concluding anything: if the failure is a wrong entrypoint guess, fix the"
echo "guess and re-run -- that is not evidence against the tool. If the tool"
echo "genuinely cannot build in a privileged container on an arm64 runner,"
echo "take the derive-from-stock fallback (reference doc section 2), NOT pi-gen."
exit 1
