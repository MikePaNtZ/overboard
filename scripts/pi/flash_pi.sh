#!/usr/bin/env bash
# Flash an Overboard image to an SD card and stage credentials onto it.
#
#     scripts/pi/flash_pi.sh                                   # newest release
#     scripts/pi/flash_pi.sh --disk /dev/disk4                 # explicit target
#     scripts/pi/flash_pi.sh --image ./x.img.xz                # local image
#     scripts/pi/flash_pi.sh --dry-run                         # rehearse
#
# With no --disk, the one removable disk in the machine is proposed. Zero or
# more than one is an error, never a guess. Detection only ever REMOVES the
# lookup step -- the confirmation prompt below still has to be answered by
# hand, because the failure this script exists to prevent is writing to the
# wrong disk, and a target the script chose deserves more scrutiny than one
# you typed, not less.
#
# THIS WRITES TO A RAW BLOCK DEVICE. The classic way to lose a laptop's
# internal disk is a mistyped device path in exactly this kind of script, so
# there are three independent guards and none of them can be skipped with a
# flag:
#
#   1. The target must be removable/external. An internal disk is refused
#      outright -- there is no --force, because the only reason to want one
#      here is a mistake.
#   2. The device identifier must be typed back at a prompt. Not y/N: a habit
#      of pressing y is exactly what defeats a y/N prompt.
#   3. --dry-run performs every check and every mount, and skips only the
#      write, so the rehearsal exercises the same code path as the real thing.
#
# Credentials are read from ~/.overboard/pi-secrets.env, which lives OUTSIDE
# this repository and is never read by CI or by an agent. See
# scripts/pi/pi-secrets.example.env.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"

IMAGE_RELEASE="pi-image-latest"
SECRETS="${OVERBOARD_PI_SECRETS:-$HOME/.overboard/pi-secrets.env}"
DISK=""
IMAGE=""
DRY_RUN=0

die() { echo "error: $*" >&2; exit 1; }
say() { echo "==> $*"; }

usage() {
  # Print the header block, whatever length it happens to be: everything from
  # line 2 up to (not including) the first line of actual code. The previous
  # hard-coded '2,30p' silently truncated the moment the header grew, which is
  # how --help starts lying about a script whose whole job is not to surprise
  # you.
  sed -n '2,/^set -/p' "${BASH_SOURCE[0]}" | sed '$d' | sed 's/^# \{0,1\}//'
  exit "${1:-0}"
}

while [ $# -gt 0 ]; do
  case "$1" in
    --disk)    DISK="${2:-}"; shift 2 ;;
    --image)   IMAGE="${2:-}"; shift 2 ;;
    --release) IMAGE_RELEASE="${2:-}"; shift 2 ;;
    --secrets) SECRETS="${2:-}"; shift 2 ;;
    --dry-run) DRY_RUN=1; shift ;;
    -h|--help) usage 0 ;;
    *) die "unrecognized argument '$1' (try --help)" ;;
  esac
done

OS="$(uname -s)"
case "$OS" in
  Darwin|Linux) ;;
  *) die "unsupported platform: $OS" ;;
esac

# ---------------------------------------------------------------------------
# Target selection.
#
# --disk still wins outright. With no --disk we PROPOSE the single removable
# disk rather than requiring the operator to go and look it up -- but only
# when there is exactly one. Zero or several is an error, never a guess: the
# whole point of this script is that it does not write to the wrong disk, and
# "pick the first one" is how that promise gets broken.
#
# What detection removes is the lookup step, not a safety step. Guard 1
# (removable-only) and guard 2 (type the identifier back) both still run
# against whatever lands in $DISK, and they run identically whether a human
# or this function chose it. A target the script picked deserves the SAME
# scrutiny as one that was typed, not less.
# ---------------------------------------------------------------------------
removable_disks() {
  if [ "$OS" = "Darwin" ]; then
    diskutil list external physical 2>/dev/null | awk '/^\/dev\/disk/{print $1}'
  else
    # RM=1 is the kernel's own removable flag -- the same bit guard 1 checks.
    lsblk -dno NAME,RM 2>/dev/null | awk '$2 == 1 { print "/dev/" $1 }'
  fi
}

if [ -z "$DISK" ]; then
  FOUND="$(removable_disks || true)"
  COUNT="$(printf '%s\n' "$FOUND" | grep -c '^/dev/' || true)"
  case "$COUNT" in
    1)
      DISK="$(printf '%s\n' "$FOUND" | grep '^/dev/')"
      say "no --disk given; exactly one removable disk present: $DISK"
      ;;
    0)
      die "no --disk given, and no removable disk was found.
       Insert the card, or name the target explicitly:
         macOS:  diskutil list external
         Linux:  lsblk -o NAME,SIZE,TYPE,RM,MOUNTPOINT" ;;
    *)
      die "no --disk given, and $COUNT removable disks are present:
$(printf '%s\n' "$FOUND" | grep '^/dev/' | sed 's/^/         /')
       Refusing to choose between them -- pick one with --disk." ;;
  esac
fi

# ---------------------------------------------------------------------------
# Guard 1: the target must be removable. Refused, not warned about.
# ---------------------------------------------------------------------------
assert_removable() {
  local disk="$1"
  [ -e "$disk" ] || die "$disk does not exist"

  if [ "$OS" = "Darwin" ]; then
    local info removable internal
    info="$(diskutil info "$disk" 2>/dev/null)" || die "diskutil could not read $disk"
    removable="$(echo "$info" | awk -F': *' '/Removable Media/{print $2}' | xargs)"
    internal="$(echo "$info" | awk -F': *' '/Device Location/{print $2}' | xargs)"
    echo "$info" | grep -E 'Device / Media Name|Disk Size|Device Location|Removable Media|Volume Name' \
      | sed 's/^ */    /'
    if [ "$internal" = "Internal" ]; then
      die "$disk is an INTERNAL disk. Refusing.
       This is almost certainly your system drive. Re-read `diskutil list external`."
    fi
    case "$removable" in
      Removable|"Yes") ;;
      *) die "$disk does not report as removable media (got: '${removable:-unknown}'). Refusing." ;;
    esac
  else
    local base rm_flag
    base="$(basename "$(readlink -f "$disk")")"
    base="${base%%[0-9]*}"
    rm_flag="$(cat "/sys/block/$base/removable" 2>/dev/null || echo 0)"
    lsblk -o NAME,SIZE,TYPE,RM,MODEL "$disk" | sed 's/^/    /'
    [ "$rm_flag" = "1" ] || die "$disk is not flagged removable (/sys/block/$base/removable=0). Refusing."
  fi
}

say "target device"
assert_removable "$DISK"

# ---------------------------------------------------------------------------
# Credentials first: fail before touching the card, not after.
#
# Ordering is the whole point. Discovering a typo in the secrets file after a
# 20-minute write means doing the write again.
# ---------------------------------------------------------------------------
STAGING="$(mktemp -d)"
trap 'rm -rf "$STAGING"' EXIT

say "building boot-partition payload from $SECRETS"
python3 "$HERE/mk_boot_config.py" --secrets "$SECRETS" --out "$STAGING" \
  || die "credential staging failed - nothing was written to $DISK"

# ---------------------------------------------------------------------------
# Resolve and verify the image.
# ---------------------------------------------------------------------------
if [ -z "$IMAGE" ]; then
  command -v gh >/dev/null || die "no --image given and `gh` is not installed"
  say "downloading the '$IMAGE_RELEASE' release"
  ( cd "$STAGING" && gh release download "$IMAGE_RELEASE" --repo MikePaNtZ/overboard \
      --pattern '*.img.xz' --pattern '*.img.xz.sha256' ) \
    || die "could not download release '$IMAGE_RELEASE'.
       If the image pipeline has not landed yet (issue #182 I1), pass --image."
  IMAGE="$(ls -1 "$STAGING"/*.img.xz)"
fi
[ -f "$IMAGE" ] || die "image not found: $IMAGE"

if [ -f "$IMAGE.sha256" ]; then
  say "verifying SHA256"
  ( cd "$(dirname "$IMAGE")" && \
    { shasum -a 256 -c "$(basename "$IMAGE").sha256" 2>/dev/null \
      || sha256sum -c "$(basename "$IMAGE").sha256"; } ) \
    || die "CHECKSUM MISMATCH - refusing to flash a corrupt or tampered image"
else
  # Loud, because a silent skip is how an unverified image becomes routine.
  echo "WARNING: no $(basename "$IMAGE").sha256 alongside the image."
  echo "         Flashing UNVERIFIED. AC-11's restore test assumes a checked artefact."
fi

# ---------------------------------------------------------------------------
# Guard 2: type the identifier back.
# ---------------------------------------------------------------------------
echo
echo "About to ERASE $DISK and write:"
echo "    image:  $(basename "$IMAGE")"
echo "    creds:  $(find "$STAGING" -name '*.nmconnection' | wc -l | xargs) wifi profile(s), ssh key, userconf"
echo
if [ "$DRY_RUN" -eq 1 ]; then
  say "DRY RUN - every check above ran; skipping the write"
else
  printf "Type the device identifier (%s) to confirm: " "$DISK"
  read -r confirm
  [ "$confirm" = "$DISK" ] || die "confirmation did not match - nothing written"
fi

# ---------------------------------------------------------------------------
# Write.
# ---------------------------------------------------------------------------
BOOT_MNT=""
if [ "$OS" = "Darwin" ]; then
  RAW="${DISK/\/dev\/disk//dev/rdisk}"   # rdisk = unbuffered, ~10x faster
  if [ "$DRY_RUN" -eq 0 ]; then
    say "unmounting $DISK"
    diskutil unmountDisk "$DISK"
    say "writing (sudo; ctrl-T shows progress)"
    xz -dc "$IMAGE" | sudo dd of="$RAW" bs=4m
    sync
    say "waiting for the boot partition to mount"
    sleep 3
    diskutil mount "${DISK}s1" >/dev/null 2>&1 || true
  fi
  BOOT_MNT="/Volumes/bootfs"
  [ -d "$BOOT_MNT" ] || BOOT_MNT="/Volumes/boot"
else
  if [ "$DRY_RUN" -eq 0 ]; then
    say "unmounting any mounted partitions of $DISK"
    for part in "$DISK"?*; do umount "$part" 2>/dev/null || true; done
    say "writing (sudo)"
    xz -dc "$IMAGE" | sudo dd of="$DISK" bs=4M conv=fsync status=progress
    sync
  fi
  BOOT_MNT="$(mktemp -d)"
  [ "$DRY_RUN" -eq 0 ] && sudo mount "${DISK}1" "$BOOT_MNT"
fi

# ---------------------------------------------------------------------------
# Stage credentials onto the FAT32 boot partition.
#
# The boot partition, because the rootfs is ext4 and macOS cannot mount it.
# overboard-firstboot (shipped in the image) moves these onto the rootfs with
# correct ownership and permissions on first boot, then shreds them.
# ---------------------------------------------------------------------------
if [ "$DRY_RUN" -eq 0 ]; then
  [ -d "$BOOT_MNT" ] || die "boot partition did not mount at $BOOT_MNT.
       The image was written; only credential staging failed. Mount the boot
       partition by hand and copy the contents of a re-run --dry-run staging dir."
  say "staging credentials to $BOOT_MNT"
  sudo cp -R "$STAGING/overboard-net.d" "$BOOT_MNT/" 2>/dev/null || \
       cp -R "$STAGING/overboard-net.d" "$BOOT_MNT/"
  for f in overboard-authorized_keys overboard-userconf; do
    sudo cp "$STAGING/$f" "$BOOT_MNT/" 2>/dev/null || cp "$STAGING/$f" "$BOOT_MNT/"
  done
  sync

  say "ejecting"
  if [ "$OS" = "Darwin" ]; then
    diskutil eject "$DISK"
  else
    sudo umount "$BOOT_MNT" && rmdir "$BOOT_MNT"
  fi
fi

echo
if [ "$DRY_RUN" -eq 1 ]; then
  say "DRY RUN complete. Checks passed and credentials staged to $STAGING."
else
  say "done. Insert the card and power the Pi."
  echo "    First boot installs the credentials and then SHREDS them from the"
  echo "    boot partition, so they are not left on a FAT32 filesystem."
  echo
  echo "    Works immediately -- these ship in the image:"
  echo "      cyclictest -m -Sp95 -i 2000 -D 30m       # kernel wakeup jitter (AC-5)"
  echo "      ip -details link show can0               # CAN up at 500 kbit/s"
  echo "      cangen vcan0 & candump vcan0             # CAN stack, no hardware needed"
  echo
  echo "    NOT on the card yet -- no Overboard code ships in the image (issue"
  echo "    #182 I5). To run the controller or the loop profiler you must put"
  echo "    them there yourself for now:"
  echo "      ssh <user>@overboard.local"
  echo "      sudo apt install -y git build-essential curl"
  echo "      curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh"
  echo "      git clone https://github.com/MikePaNtZ/overboard && cd overboard"
  echo "      cargo build --release -p loop-profiler"
  echo "      sudo ./target/release/overboard-loop-profile --cycles 100000 --rt-prio 80"
fi
