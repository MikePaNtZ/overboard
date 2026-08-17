#!/usr/bin/env bash
# I1 -- build the Overboard Stage-0B image.
#
# Runs inside a privileged debian:trixie container on an ubuntu-24.04-arm
# runner: native arm64, no QEMU. The D3 spike (issue #182 I0) proved that host
# shape works before any of this was written, which is the whole reason this
# script can be short.
#
# Produces, under /tmp/overboard-image:
#   overboard-stage0b.img.*        the image, as rpi-image-gen compressed it
#   *.sha256                       checksum, so flash_pi.sh can verify
#   overboard-image.json           AC-4 provenance, lifted out of the rootfs
#   overboard-packages.txt         AC-3 package manifest, for diffing builds
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
SRC="$HERE/image"
OUT="${OUT_DIR:-/tmp/overboard-image}"
WORK="${WORK_DIR:-/tmp/rpi-image-gen}"
GEN_REPO="https://github.com/raspberrypi/rpi-image-gen"

section() { echo; echo "=============== $* ==============="; echo; }

section "host"
echo "uname:  $(uname -m) $(uname -r)"
echo "os:     $(grep PRETTY_NAME /etc/os-release | cut -d= -f2-)"
echo "commit: ${OVERBOARD_GIT_SHA:-<unset>}"

section "prerequisites"
export DEBIAN_FRONTEND=noninteractive
apt-get update -qq
apt-get install -y --no-install-recommends git ca-certificates sudo xz-utils zstd >/dev/null
echo ok

section "fetch rpi-image-gen"
# Shallow clone of the default branch. NOT pinned to a commit, and that is a
# real gap rather than an oversight: design §2 pins what goes ON the card,
# and the builder is not on the card. But a builder change can still alter
# the output, so this should become a pinned ref once the image stabilises.
# Recorded here rather than left implicit -- the commit is echoed below so
# any two builds are at least attributable.
rm -rf "$WORK"
git clone --depth 1 "$GEN_REPO" "$WORK"
echo "rpi-image-gen commit: $(git -C "$WORK" rev-parse --short HEAD)"

section "install rpi-image-gen dependencies"
( cd "$WORK" && ./install_deps.sh ) 2>&1 | tail -20

section "the Overboard image tree"
find "$SRC" -type f | sed "s|$SRC|scripts/pi/image|" | sort

# ---------------------------------------------------------------------------
# The Raspberry Pi archive will not verify on Trixie without this -- the key
# carries a SHA-1 self-certification and Trixie's sqv rejects SHA-1 from
# 2026-02-01. add_rpi_archive.sh applies the narrow hash-policy relaxation
# (signature verification stays ON) and only when the default policy actually
# refuses, so it retires itself when the key is re-signed.
#
# Needed HERE as well as in the verify jobs because the image build resolves
# the RT kernel from that same archive, inside mmdebstrap.
# ---------------------------------------------------------------------------
section "raspberry pi archive"
bash "$HERE/add_rpi_archive.sh"

section "BUILD"
# `-c` is relative to `-S`'s config directory -- the form rpi-image-gen's own
# examples/slim README documents. Established the hard way: the spike spent
# two rounds passing paths that were nearly right.
cd "$WORK"
./rpi-image-gen build -S "$SRC" -c overboard.yaml

section "kernel selection matches what was built"

# config.txt names a kernel FILE, and firmware loads that exact name. If the
# file is not in the boot partition the card is a brick -- and NOTHING else in
# this build would notice. The image is well-formed, every pin resolves, the
# manifest is accurate, the checksum matches. It fails only at 3am on a Pi
# with a dark screen.
#
# This is not hypothetical. The line said kernel8.img while the RT package
# installs kernel8_rt.img, on the strength of a spike that built a DIFFERENT
# kernel. It reached a real card and was caught by hand minutes before the
# first AC-11 attempt. The assertion below is what makes that a build failure
# instead of a bench evening.
KCFG="$SRC/device/overboard-pi5/config.txt"
[ -f "$KCFG" ] || { echo "FATAL: $KCFG not found"; exit 1; }
kname="$(awk -F= '/^kernel=/{print $2}' "$KCFG" | tail -1 | tr -d '[:space:]')"
[ -n "$kname" ] || { echo "FATAL: $KCFG has no kernel= line to verify"; exit 1; }

if [ -n "$(find "$WORK/work" -name "$kname" -print -quit 2>/dev/null)" ]; then
  echo "OK: config.txt names kernel=$kname, and that file is in the build"
else
  echo "FATAL: config.txt names kernel=$kname, but no such file was built."
  echo "       The card would not boot. Kernel images actually present:"
  find "$WORK/work" -name 'kernel*.img' 2>/dev/null \
    | sed 's|.*/|         |' | sort -u
  echo "       Point kernel= at one of those, or work out why the expected"
  echo "       kernel package did not install."
  exit 1
fi

section "no baked-in ssh host keys (#284)"

# openssh-server's postinst generates host keys at package-install time,
# inside THIS build container -- every image built from a given container
# state would otherwise ship the identical private key, published as a
# public CI artefact anyone can download and use to impersonate the board.
# overboard-base.yaml's customize-hooks strip them from the rootfs; this is
# the second line of defence in case that hook is ever weakened without
# anyone noticing until a card ships.
leaked_keys="$(find "$WORK/work" -path '*/etc/ssh/ssh_host_*' 2>/dev/null)"
if [ -n "$leaked_keys" ]; then
  echo "FATAL: /etc/ssh/ssh_host_* found in the built rootfs -- every card"
  echo "       flashed from this image would share the same SSH identity."
  echo "$leaked_keys" | sed 's/^/         /'
  exit 1
else
  echo "OK: no ssh_host_* files in the built rootfs"
fi

section "collect artefacts"
mkdir -p "$OUT"
rm -f "$OUT"/*

# Prefer the deploy directory's compressed artefact; fall back to the raw
# image. Globbed rather than named: the compression format is rpi-image-gen's
# choice (it deploys .zst today), and hardcoding an extension here is how the
# publish step ends up looking for a file that was never written.
found=0
for f in $(find "$WORK/work" -name '*.img' -o -name '*.img.xz' -o -name '*.img.zst' 2>/dev/null | sort); do
  cp "$f" "$OUT/"
  found=1
done
[ "$found" -eq 1 ] || { echo "BUILD PRODUCED NO IMAGE"; exit 1; }

# Provenance and package manifest, lifted out of the built rootfs so what
# ships beside the image is what is actually ON it.
for f in etc/overboard-image.json etc/overboard-packages.txt; do
  src="$(find "$WORK/work" -path "*/$f" -print -quit 2>/dev/null || true)"
  if [ -n "$src" ]; then
    cp "$src" "$OUT/$(basename "$f")"
  else
    echo "WARNING: $f not found in the rootfs - AC-4/AC-3 artefact missing" >&2
  fi
done

section "checksums"
cd "$OUT"
for f in *.img*; do
  [ -e "$f" ] || continue
  sha256sum "$f" > "$f.sha256"
  echo "$(cat "$f.sha256")  ($(du -h "$f" | cut -f1))"
done

section "provenance"
cat "$OUT/overboard-image.json" 2>/dev/null || echo "<missing>"
echo
echo "packages: $(wc -l < "$OUT/overboard-packages.txt" 2>/dev/null || echo 0)"

echo
echo "IMAGE BUILD COMPLETE - artefacts in $OUT"
ls -la "$OUT"
