#!/usr/bin/env bash
# Runs ON THE PI, once, at first boot. Shipped inside the image (I1).
#
# Moves the credentials the flash script staged on the FAT32 boot partition
# into their real homes on the ext4 rootfs, then deletes them from the boot
# partition -- because FAT32 has no permissions, so anything left there is
# world-readable to any machine that sees the card.
#
# WHY THIS EXISTS RATHER THAN RASPBERRY PI IMAGER'S MECHANISM
#
# Imager hooks a `firstrun.sh` from `cmdline.txt` and has that script edit
# `cmdline.txt` back out again at the end -- a boot-critical file being
# rewritten by a script running during that same boot, where an interrupted
# first boot can leave a card that does not boot at all. We build the image,
# so we can do better: a plain systemd oneshot that reads a directory.
# `cmdline.txt` is never touched.
#
# Idempotent, because "ran but the network came up too late to matter" is a
# real first-boot outcome and re-running must be safe.
set -euo pipefail

BOOT="${BOOT_DIR:-/boot/firmware}"
NET_SRC="$BOOT/overboard-net.d"
NM_DIR="/etc/NetworkManager/system-connections"
USERCONF="$BOOT/overboard-userconf"
AUTHKEYS="$BOOT/overboard-authorized_keys"

log() { echo "overboard-firstboot: $*"; }

# Nothing staged is the normal state on every boot after the first.
if [ ! -d "$NET_SRC" ] && [ ! -f "$USERCONF" ]; then
  log "no staged configuration - nothing to do"
  exit 0
fi

# ---------------------------------------------------------------------------
# Identity
# ---------------------------------------------------------------------------
username=""
if [ -f "$USERCONF" ]; then
  # Parsed, not sourced: this file came off a FAT32 partition that anyone
  # could have written to. Executing it would hand that person root on first
  # boot, which is a strange way to configure a hostname.
  while IFS='=' read -r key value; do
    case "$key" in
      hostname)         hostname="$value" ;;
      username)         username="$value" ;;
      wifi_country)     wifi_country="$value" ;;
      ssh_password_auth) ssh_password_auth="$value" ;;
      password_hash)    password_hash="$value" ;;
    esac
  done < "$USERCONF"

  if [ -n "${hostname:-}" ]; then
    log "hostname -> $hostname"
    hostnamectl set-hostname "$hostname" || log "WARNING: could not set hostname"
    sed -i "s/^127.0.1.1.*/127.0.1.1\t$hostname/" /etc/hosts || true
  fi

  if [ -n "$username" ] && ! id -u "$username" >/dev/null 2>&1; then
    log "creating user $username"
    useradd -m -s /bin/bash -G sudo,dialout,gpio,i2c,spi,netdev "$username" \
      || useradd -m -s /bin/bash -G sudo,dialout "$username"
    if [ -n "${password_hash:-}" ]; then
      usermod -p "$password_hash" "$username"
    else
      # Locked, not blank. A blank password on an account in the sudo group
      # is a different and much worse thing than no password login.
      passwd -l "$username" >/dev/null
    fi
  fi

  if [ -n "${wifi_country:-}" ]; then
    log "regulatory domain -> $wifi_country"
    raspi-config nonint do_wifi_country "$wifi_country" 2>/dev/null \
      || iw reg set "$wifi_country" 2>/dev/null \
      || log "WARNING: could not set regulatory domain; 5 GHz may be unavailable"
  fi
fi

# ---------------------------------------------------------------------------
# SSH
# ---------------------------------------------------------------------------
if [ -f "$AUTHKEYS" ] && [ -n "$username" ]; then
  home="$(getent passwd "$username" | cut -d: -f6)"
  install -d -m 0700 -o "$username" -g "$username" "$home/.ssh"
  install -m 0600 -o "$username" -g "$username" "$AUTHKEYS" "$home/.ssh/authorized_keys"
  log "installed authorized_keys for $username"
fi

if [ "${ssh_password_auth:-false}" = "false" ]; then
  install -d -m 0755 /etc/ssh/sshd_config.d
  printf 'PasswordAuthentication no\nChallengeResponseAuthentication no\n' \
    > /etc/ssh/sshd_config.d/10-overboard.conf
  log "SSH password authentication disabled (key-only)"
fi
systemctl enable --now ssh >/dev/null 2>&1 || log "WARNING: could not enable ssh"

# ---------------------------------------------------------------------------
# Wi-Fi
# ---------------------------------------------------------------------------
if [ -d "$NET_SRC" ]; then
  install -d -m 0700 "$NM_DIR"
  count=0
  for f in "$NET_SRC"/*.nmconnection; do
    [ -e "$f" ] || continue
    # 0600 or NetworkManager refuses to load the profile -- and says so only
    # in its own log, which is a miserable thing to debug on a headless board.
    install -m 0600 -o root -g root "$f" "$NM_DIR/$(basename "$f")"
    count=$((count + 1))
  done
  log "installed $count wifi profile(s)"
  nmcli connection reload 2>/dev/null || systemctl reload NetworkManager 2>/dev/null || true
fi

# ---------------------------------------------------------------------------
# Shred the staged copies.
#
# FAT32 carries no permissions, so until this runs the PSKs are readable by
# anything that can see the card. `shred` is best-effort here and worth saying
# so honestly: on a wear-levelling SD controller, overwriting a logical block
# does not reliably overwrite the physical one. It raises the cost of casual
# recovery; it is not an erasure guarantee.
# ---------------------------------------------------------------------------
if [ -d "$NET_SRC" ]; then
  find "$NET_SRC" -type f -exec shred -u {} \; 2>/dev/null || rm -f "$NET_SRC"/*
  rmdir "$NET_SRC" 2>/dev/null || true
fi
[ -f "$AUTHKEYS" ] && { shred -u "$AUTHKEYS" 2>/dev/null || rm -f "$AUTHKEYS"; }
[ -f "$USERCONF" ] && { shred -u "$USERCONF" 2>/dev/null || rm -f "$USERCONF"; }

log "complete - staged credentials removed from $BOOT"
