#!/usr/bin/env bash
# Add the Raspberry Pi archive to a Debian container's apt sources.
#
# Shared by every job that needs to resolve a Pi package, so the archive is
# configured exactly one way. Both verification jobs and the image build read
# from the same place; three copies of this would drift, and a drifted
# ARCHIVE URL is a pin that verifies against a different mirror than the one
# the card is built from.
#
# `signed-by` rather than the deprecated apt-key: a key in trusted.gpg.d is
# trusted for EVERY source, so a compromised Pi mirror could sign a Debian
# package. Scoping the key to its own source is the whole point.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=pins.env
source "$HERE/pins.env"

KEYRING="/usr/share/keyrings/raspberrypi-archive.gpg"
export DEBIAN_FRONTEND=noninteractive

# ---------------------------------------------------------------------------
# FINDING, 2026-08-02: Debian Trixie will not verify the Raspberry Pi archive.
#
# Trixie replaced apt's OpenPGP verifier with Sequoia's `sqv`, which enforces a
# crypto policy rejecting SHA-1 as of 2026-02-01. The Raspberry Pi archive key
# (CF8A1AF502A2AA2D763BAE7E82B129927FA3303E) carries a SHA-1 self-certification,
# so `sqv` refuses to bind the signing subkey and apt reports:
#
#     E: The repository '.../debian trixie InRelease' is not signed.
#
# This is not our bug and not a flaky network. It blocks EVERY Trixie-hosted
# build against archive.raspberrypi.com, which is the exact shape design
# section 3 chose, so it is on the critical path for I0/I1 (issue #182).
#
# WHAT THIS DOES, AND THE LINE IT DOES NOT CROSS
#
# It relaxes the HASH POLICY so a SHA-1 binding signature is accepted. It does
# NOT disable verification. Every package is still cryptographically verified
# against the pinned key fetched over TLS from the Pi archive.
#
# The tempting one-line "fix" is `[trusted=yes]` on the sources line, and that
# is categorically different: it turns signature checking OFF, so anything
# reaching the mirror or the connection lands on the card unchallenged. For an
# RT kernel that will command a motor, that is not a trade worth making to
# save a config file. If this exception ever stops working, the answer is to
# escalate -- never to reach for trusted=yes.
#
# NARROW, AND WITH AN EXPIRY CONDITION: revisit when Raspberry Pi re-sign the
# key with SHA-256. Then delete this block and let the default policy stand.
# Tracked on issue #182.
# ---------------------------------------------------------------------------
relax_sha1_policy() {
  # Written to sqv's DEFAULT system path, not just exported. Each `run:` step
  # in a GitHub job is a separate shell, so a SEQUOIA_CRYPTO_POLICY exported
  # here would vanish before verify_pins.sh runs in the next step -- and the
  # symptom would be this script reporting success followed by an
  # inexplicable "not signed" one step later.
  local policy=/etc/crypto-policies/back-ends/sequoia.config
  mkdir -p "$(dirname "$policy")"
  cat > "$policy" <<'POLICY'
# Overboard: accept SHA-1 binding signatures so apt can verify the Raspberry
# Pi archive on Debian Trixie. Signature verification stays ON -- only the
# hash policy is relaxed. See scripts/pi/add_rpi_archive.sh for why.
[hash_algorithms]
sha1.collision_resistance = "always"
sha1.second_preimage_resistance = "always"
POLICY
  export SEQUOIA_CRYPTO_POLICY="$policy"
  # Carry it to later steps too, belt and braces, when running under Actions.
  [ -n "${GITHUB_ENV:-}" ] && echo "SEQUOIA_CRYPTO_POLICY=$policy" >> "$GITHUB_ENV"
  echo "sequoia crypto policy relaxed for SHA-1 binding signatures (verification still enforced)"
}

apt-get update -qq
apt-get install -y --no-install-recommends curl ca-certificates gnupg >/dev/null

curl -fsSL "https://archive.raspberrypi.com/debian/raspberrypi.gpg.key" \
  | gpg --dearmor -o "$KEYRING"

echo "deb [signed-by=$KEYRING arch=$DEBIAN_ARCH] $RPI_ARCHIVE_URL $RPI_ARCHIVE_SUITE $RPI_ARCHIVE_COMPONENT" \
  > /etc/apt/sources.list.d/raspberrypi.list

# Try the distribution's own policy first, and only relax if it actually
# rejects the archive. If Raspberry Pi re-sign the key with SHA-256, this
# quietly stops relaxing anything and the workaround retires itself instead
# of lingering as a permanent weakening nobody rechecks.
if apt-get update -qq 2>/tmp/apt-update.err; then
  echo "archive verified under the distribution's default crypto policy"
else
  if grep -qi "not signed\|signature verification failed" /tmp/apt-update.err; then
    echo "--- default policy rejected the archive:"
    sed 's/^/    /' /tmp/apt-update.err
    relax_sha1_policy
    apt-get update -qq
  else
    echo "apt-get update failed for a reason unrelated to signing:" >&2
    cat /tmp/apt-update.err >&2
    exit 1
  fi
fi

echo "raspberrypi archive added: $RPI_ARCHIVE_URL $RPI_ARCHIVE_SUITE $RPI_ARCHIVE_COMPONENT ($DEBIAN_ARCH)"
