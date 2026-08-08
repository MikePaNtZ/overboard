#!/usr/bin/env bash
# Issue #261 -- CI can prove the built image is well-formed but never boots
# it. Every one of the six defects #249 found on real hardware (2026-08-06)
# was invisible to `verify-pins`/`verify-kernel`/`check-pin`/`build-image`:
# the build was green on the exact commit that produced a card that hung
# forever on first boot. This closes that gap in software, before the next
# flash.
#
# Boots the built image under QEMU's *generic* `virt` machine -- not a
# hardware-accurate Pi 5. No QEMU machine type models the Pi 5 (BCM2712) at
# all; `raspi4b` emulates the Pi 4 (BCM2711) and is a different SoC, so using
# it would silently substitute one unverified assumption for another. `virt`
# with the extracted kernel and the image's own root partition attached as a
# virtio disk is the standard, widely-documented way to boot a Raspberry Pi
# OS-family aarch64 image generically (root=/dev/vda2 in place of the
# firmware/mmcblk chain); see the PR description for the sources this was
# checked against.
#
# WHAT THIS DOES NOT PROVE, stated up front so a green run is not
# over-read: RP1, real SPI, the CAN HAT, thermals and every latency number
# stay hardware-only (AC-5, AC-6). This is a *functional* boot gate --
# does the built kernel/rootfs pair reach multi-user.target and configure
# userspace the way overboard-firstboot promises to -- not a performance or
# hardware-fidelity one. There is also no virtual network device attached
# (see below), so mDNS actually resolving is out of scope here too; only
# that the avahi stack starts is checked.
#
# NEVER calls mk_boot_config.py. That script's own docstring says it must
# never run in CI or be run by an agent -- it is the real-credential path,
# and its input must never enter the repository. Everything staged onto the
# boot partition here is synthetic, hand-written below, generated fresh in a
# scratch directory, and never touches the CEO's actual secrets file.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

IMAGE=""
BOOT_TIMEOUT="${BOOT_TIMEOUT:-240}"
QEMU_MEM="${QEMU_MEM:-1024}"

usage() {
  echo "usage: $0 --image <path-to-.img|.img.xz|.img.zst>" >&2
  exit 2
}

while [ $# -gt 0 ]; do
  case "$1" in
    --image) IMAGE="$2"; shift 2 ;;
    -h|--help) usage ;;
    *) echo "unknown argument: $1" >&2; usage ;;
  esac
done
[ -n "$IMAGE" ] || usage
[ -f "$IMAGE" ] || { echo "FATAL: no such file: $IMAGE" >&2; exit 1; }

command -v qemu-system-aarch64 >/dev/null || {
  echo "FATAL: qemu-system-aarch64 not found (apt install qemu-system-arm)" >&2
  exit 1
}

WORK="$(mktemp -d /tmp/overboard-qemu-boot-test.XXXXXX)"
LOOP=""
BOOT_MNT="$WORK/mnt-boot"
ROOT_MNT="$WORK/mnt-root"

cleanup() {
  local rc=$?
  # Best-effort, in reverse dependency order. A failed unmount must not mask
  # the real exit status of the run.
  mountpoint -q "$BOOT_MNT" 2>/dev/null && sudo umount "$BOOT_MNT" || true
  mountpoint -q "$ROOT_MNT" 2>/dev/null && sudo umount "$ROOT_MNT" || true
  [ -n "$LOOP" ] && sudo losetup -d "$LOOP" 2>/dev/null || true
  rm -rf "$WORK"
  exit "$rc"
}
trap cleanup EXIT

section() { echo; echo "=============== $* ==============="; echo; }

# ---------------------------------------------------------------------------
# 1. Decompress into a working raw image. The build pipeline globs rather
#    than names the extension (build_image.sh's own comment explains why:
#    rpi-image-gen's compression choice is not ours to pin), so this must
#    handle whichever of the three shows up.
# ---------------------------------------------------------------------------
section "stage image"
RAW="$WORK/overboard.img"
case "$IMAGE" in
  *.img.zst) zstd -d -q -o "$RAW" "$IMAGE" ;;
  *.img.xz)  xz -dc "$IMAGE" > "$RAW" ;;
  *.img)     cp "$IMAGE" "$RAW" ;;
  *) echo "FATAL: unrecognised image extension: $IMAGE" >&2; exit 1 ;;
esac
echo "raw image: $(du -h "$RAW" | cut -f1)"

# ---------------------------------------------------------------------------
# 2. Attach as a loop device with partition scanning, and stage synthetic
#    boot-partition config + a CI-only assertion unit on the rootfs. This
#    mutates the WORKING COPY only -- the artefact this was downloaded from
#    is untouched, and nothing built here is ever uploaded anywhere.
# ---------------------------------------------------------------------------
section "attach loop device"
LOOP="$(sudo losetup -Pf --show "$RAW")"
echo "loop: $LOOP"
sudo udevadm settle 2>/dev/null || true
for _ in $(seq 1 20); do
  [ -b "${LOOP}p1" ] && [ -b "${LOOP}p2" ] && break
  sleep 0.5
done
[ -b "${LOOP}p1" ] && [ -b "${LOOP}p2" ] || {
  echo "FATAL: ${LOOP}p1/p2 never appeared" >&2
  exit 1
}

mkdir -p "$BOOT_MNT" "$ROOT_MNT"
sudo mount "${LOOP}p1" "$BOOT_MNT"
sudo mount "${LOOP}p2" "$ROOT_MNT"

section "kernel"
KCFG="$BOOT_MNT/config.txt"
[ -f "$KCFG" ] || { echo "FATAL: no config.txt on the boot partition" >&2; exit 1; }
KNAME="$(awk -F= '/^kernel=/{print $2}' "$KCFG" | tail -1 | tr -d '[:space:]')"
[ -n "$KNAME" ] || { echo "FATAL: config.txt has no kernel= line" >&2; exit 1; }
[ -f "$BOOT_MNT/$KNAME" ] || {
  echo "FATAL: config.txt names kernel=$KNAME but it is not on the boot partition" >&2
  exit 1
}
KERNEL="$WORK/kernel.img"
cp "$BOOT_MNT/$KNAME" "$KERNEL"
echo "kernel: $KNAME ($(du -h "$KERNEL" | cut -f1))"

# `auto_initramfs=1` (config.txt) means the FIRMWARE resolves and loads the
# matching initramfs unassisted -- design Q12's own answer, and exactly the
# mechanism this script bypasses by booting the kernel directly. QEMU gets
# no firmware, so the pairing has to be reproduced by hand: config.txt's own
# comment records the transform (kernel8_rt.img -> initramfs8_rt, "kernel"
# swapped for "initramfs", .img dropped) -- apply the same transform rather
# than hardcoding the RT suffix, so this does not silently stop covering the
# initramfs the moment the kernel package name changes.
#
# First CI run against just -kernel got no further than "Waiting for root
# device /dev/vda2..." -- this project's kernels ship virtio_blk as a
# module, not built in, and modules only load from the initramfs. Recorded
# here rather than only in the commit message: the failure mode is a silent
# hang, not an error, and worth the next reader not rediscovering it.
INITRD_NAME="$(echo "$KNAME" | sed -E 's/^kernel/initramfs/; s/\.img$//')"
[ -f "$BOOT_MNT/$INITRD_NAME" ] || {
  echo "FATAL: expected initramfs '$INITRD_NAME' (derived from kernel=$KNAME) is not on the boot partition" >&2
  exit 1
}
INITRD="$WORK/initrd.img"
cp "$BOOT_MNT/$INITRD_NAME" "$INITRD"
echo "initramfs: $INITRD_NAME ($(du -h "$INITRD" | cut -f1))"

section "stage synthetic boot-partition credentials"
# Deliberately not mk_boot_config.py's output format verbatim -- hand-written
# so it is obviously synthetic on inspection, and so this script carries no
# dependency on a script that must never run in CI. Shape matches what
# overboard-firstboot actually parses (scripts/pi/image/layer/
# overboard-base.rootfs-overlay/usr/local/sbin/overboard-firstboot).
sudo tee "$BOOT_MNT/overboard-userconf" >/dev/null <<'EOF'
hostname=overboard-qemu-ci
username=citest
wifi_country=US
ssh_password_auth=false
EOF

sudo mkdir -p "$BOOT_MNT/overboard-net.d"
sudo tee "$BOOT_MNT/overboard-net.d/overboard-wifi-1.nmconnection" >/dev/null <<'EOF'
# Synthetic, CI-only. Never associates with anything -- there is no AP under
# QEMU. Exercises the profile-install code path only.
[connection]
id=overboard-wifi-1
type=wifi
autoconnect=true
autoconnect-priority=100

[wifi]
mode=infrastructure
ssid=overboard-qemu-ci-test

[wifi-security]
key-mgmt=wpa-psk
psk=placeholder

[ipv4]
method=auto

[ipv6]
method=auto
addr-gen-mode=default
EOF

# A syntactically plausible but obviously-fake public key -- exercises
# authorized_keys install without being anyone's real credential.
sudo tee "$BOOT_MNT/overboard-authorized_keys" >/dev/null <<'EOF'
ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAINOTAREALKEYQEMUCIONLYPLACEHOLDER00 overboard-qemu-ci
EOF

section "inject CI-only boot assertions onto the rootfs"
# Runs once first-boot has had a chance to settle, prints PASS/FAIL markers
# to the console, then drives the two-phase reboot check itself (item 7 of
# #261's proposal). Never part of the published image: written straight onto
# this throwaway loop-mounted copy, not onto anything build_image.sh
# produces or uploads.
sudo install -d -m 0755 "$ROOT_MNT/usr/local/sbin"
sudo tee "$ROOT_MNT/usr/local/sbin/overboard-qemu-ci-assert" >/dev/null <<'ASSERT_EOF'
#!/usr/bin/env bash
# Injected by scripts/pi/qemu_boot_test.sh. Not part of the published image.
set -uo pipefail

MARKER=/var/lib/overboard-qemu-ci-phase2
RESULT=0

assert() {
  local name="$1"; shift
  local out
  if out="$("$@" 2>&1)"; then
    echo "QEMU-CI-ASSERT: $name=PASS"
  else
    echo "QEMU-CI-ASSERT: $name=FAIL ${out//$'\n'/ }"
    RESULT=1
  fi
}

if [ ! -f "$MARKER" ]; then
  echo "QEMU-CI-ASSERT: phase=1"
  assert firstboot_done      test -f /boot/firmware/overboard-firstboot.done
  assert user_exists         id citest
  assert user_in_video       sh -c 'id -nG citest | tr " " "\n" | grep -qx video'
  assert ssh_active          systemctl is-active --quiet ssh
  assert avahi_active        systemctl is-active --quiet avahi-daemon
  assert iw_present          sh -c 'command -v iw'
  assert rfkill_present      sh -c 'command -v rfkill'
  assert not_degraded        sh -c '[ "$(systemctl is-system-running 2>/dev/null)" != degraded ]'

  echo "QEMU-CI-ASSERT-DONE: phase=1 result=$RESULT"
  mkdir -p /var/lib
  touch "$MARKER"
  sync
  systemctl reboot
else
  echo "QEMU-CI-ASSERT: phase=2"
  assert firstboot_skipped   sh -c '[ "$(systemctl show overboard-firstboot.service -p ConditionResult --value)" = no ]'
  assert reached_again       sh -c 'true'

  echo "QEMU-CI-ASSERT-DONE: phase=2 result=$RESULT"
  sync
  systemctl poweroff
fi
ASSERT_EOF
sudo chmod 0755 "$ROOT_MNT/usr/local/sbin/overboard-qemu-ci-assert"

sudo tee "$ROOT_MNT/etc/systemd/system/overboard-qemu-ci-assert.service" >/dev/null <<'EOF'
[Unit]
Description=Overboard QEMU CI boot assertions (issue #261) -- injected, never on the published image
After=overboard-firstboot.service ssh.service avahi-daemon.service network.target
Wants=overboard-firstboot.service ssh.service avahi-daemon.service

[Service]
Type=oneshot
ExecStart=/usr/local/sbin/overboard-qemu-ci-assert
StandardOutput=journal+console
StandardError=journal+console

[Install]
WantedBy=multi-user.target
EOF
sudo mkdir -p "$ROOT_MNT/etc/systemd/system/multi-user.target.wants"
sudo ln -sf ../overboard-qemu-ci-assert.service \
  "$ROOT_MNT/etc/systemd/system/multi-user.target.wants/overboard-qemu-ci-assert.service"

sync
sudo umount "$BOOT_MNT"
sudo umount "$ROOT_MNT"
sudo losetup -d "$LOOP"
LOOP=""

# ---------------------------------------------------------------------------
# 3. Boot. Two invocations against the same raw image file: the guest's own
#    `systemctl reboot` at the end of phase 1 exits QEMU (thanks to
#    -no-reboot) rather than resetting it, so the phase transition is a
#    fresh process, not an in-VM ACPI reset this script has to detect.
#
# `-nic none` is load-bearing, not tidiness. Give QEMU no -nic/-net at all
# and it still adds a default user-mode NIC, and on this machine that
# default is a PCI device whose "romfile" property points at
# efi-virtio.rom -- a file `apt-get install --no-install-recommends
# qemu-system-arm` does not pull in (it ships in the ipxe-qemu recommend,
# which is skipped on purpose to keep the runner install fast). Without
# `-nic none`, qemu-system-aarch64 fails the romfile lookup and exits
# before the kernel ever loads, for a machine that was never asked to have
# a network device in the first place. First real CI run caught this.
#
# `virtio-blk-pci`, not `virtio-blk-device` (the MMIO transport). The `virt`
# machine's default NIC being a PCI device (the finding directly above)
# already proves PCI/GPEX is live here, and this kernel evidently carries
# virtio_blk as a PCI-attached module -- the MMIO form left the initramfs
# waiting on a /dev/vda2 that never appeared and never would, no matter how
# long the timeout, because nothing was probing that bus at all. Second
# real CI run caught this, one layer past the first fix.
# ---------------------------------------------------------------------------
boot_phase() {
  local label="$1" log="$2"
  section "boot: $label"
  timeout "${BOOT_TIMEOUT}s" qemu-system-aarch64 \
    -machine virt \
    -cpu cortex-a72 \
    -smp 2 \
    -m "$QEMU_MEM" \
    -kernel "$KERNEL" \
    -initrd "$INITRD" \
    -append "root=/dev/vda2 rw rootwait console=ttyAMA0 systemd.log_target=console systemd.log_level=info systemd.journald.forward_to_console=1" \
    -drive file="$RAW",format=raw,if=none,id=hd0 \
    -device virtio-blk-pci,drive=hd0 \
    -nic none \
    -nographic -no-reboot \
    -serial "file:$log" \
    || true  # qemu's exit status on a guest-initiated shutdown is not load-bearing here; the log is
  echo "--- tail of $label console log ---"
  tail -n 40 "$log" || true
}

LOG1="$WORK/console-phase1.log"
LOG2="$WORK/console-phase2.log"
boot_phase "phase 1 (first boot)" "$LOG1"
boot_phase "phase 2 (reboot, first-boot must be skipped)" "$LOG2"

# ---------------------------------------------------------------------------
# 4. Grade it. Every FAIL and every missing PASS is reported by name, not
#    just a pass/fail count -- a check that says only "something is wrong"
#    is the next round's guessing game.
# ---------------------------------------------------------------------------
section "verdict"

EXPECTED_PHASE1="firstboot_done user_exists user_in_video ssh_active avahi_active iw_present rfkill_present not_degraded"
EXPECTED_PHASE2="firstboot_skipped reached_again"

grade() {
  local log="$1" expected="$2" ok=0
  if ! grep -q "Reached target Multi-User System\|Startup finished" "$log"; then
    echo "  FAIL: multi-user.target was not reached within ${BOOT_TIMEOUT}s"
    ok=1
  fi
  for name in $expected; do
    if grep -q "QEMU-CI-ASSERT: ${name}=PASS" "$log"; then
      echo "  PASS: $name"
    elif grep -q "QEMU-CI-ASSERT: ${name}=FAIL" "$log"; then
      grep "QEMU-CI-ASSERT: ${name}=FAIL" "$log" | sed 's/^/  FAIL: /'
      ok=1
    else
      echo "  FAIL: $name never reported (boot did not reach the assertion unit)"
      ok=1
    fi
  done
  return "$ok"
}

overall=0
echo "-- phase 1 --"
grade "$LOG1" "$EXPECTED_PHASE1" || overall=1
echo "-- phase 2 --"
grade "$LOG2" "$EXPECTED_PHASE2" || overall=1

if [ "$overall" -eq 0 ]; then
  echo
  echo "QEMU BOOT TEST: PASS"
else
  echo
  echo "QEMU BOOT TEST: FAIL"
fi
exit "$overall"
