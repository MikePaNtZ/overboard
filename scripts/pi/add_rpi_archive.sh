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

apt-get update -qq
apt-get install -y --no-install-recommends curl ca-certificates gnupg >/dev/null

curl -fsSL "https://archive.raspberrypi.com/debian/raspberrypi.gpg.key" \
  | gpg --dearmor -o "$KEYRING"

echo "deb [signed-by=$KEYRING arch=$DEBIAN_ARCH] $RPI_ARCHIVE_URL $RPI_ARCHIVE_SUITE $RPI_ARCHIVE_COMPONENT" \
  > /etc/apt/sources.list.d/raspberrypi.list

apt-get update -qq

echo "raspberrypi archive added: $RPI_ARCHIVE_URL $RPI_ARCHIVE_SUITE $RPI_ARCHIVE_COMPONENT ($DEBIAN_ARCH)"
