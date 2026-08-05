# Runbook — Pi 5 first boot: prove the board, pin the bootloader, close Q12

<!--
covers:
  - scripts/pi/flash_pi.sh
  - scripts/pi/pins.env
  - crates/loop-profiler/src/lib.rs
reconciled: 725c00c
-->

**Owned by: Senior Controls.** **Executed by: the CEO**, at a desk, with a monitor and a
keyboard. This is the session that happens the day the Pi 5 comes out of the box and before
the Stage-0B image exists.

TURF-OVERRIDE: docs/ is COO turf; issue #182 assigns the Stage-0B image path to Senior Controls.

---

## Why this runbook exists separately from Stage 0B

[`runbook-stage0b-bench.md`](runbook-stage0b-bench.md) is the procedure **the Pi executes**
once it has our image. This is the procedure **a human executes** to get from a sealed box to a
Pi that can be trusted to hold a measurement — and it is deliberately written for the state the
project is actually in:

> **The Stage-0B image does not exist yet.** PR #197 (`ops(pi): I1 — the real Overboard image`)
> has a red `build-image` job on every run to date, and no `pi-image-latest` release has been
> published. `scripts/pi/flash_pi.sh` invoked without `--image` resolves against that release and
> will fail at the download step. Do not start there.

Waiting for the image would waste the hardware. Three things on the critical path can be
established today, on stock Raspberry Pi OS, with no image and no drive:

1. **The board is not DOA**, and its thermals are known.
2. **The bootloader EEPROM is current, and its version is recorded** — see §4 for why this is a
   controls problem and not an IT chore.
3. **Q12 is closed empirically** — reference doc §11 defers "how does Pi 5 firmware select the
   v8-rt kernel, given it defaults to `kernel_2712.img`?" to first boot, and calls it "the
   difference between a card that boots and one that does not." We can answer it *before* the
   image is built rather than discovering it as a dead card afterwards.

## ⚠️ Safety envelope

**Nothing connects to the drive in this session.** No CAN HAT, no CANable 2.0, no Little FOCer,
no motor, no bench supply. This session stays inside the
[reference doc §6](design-pi-image-stage0b-reference.md) category of runs that do **not** require
the CEO stood over the deadman — and it stays there by the actuator being physically absent, not
by anyone's care. If a drive gets connected, this runbook no longer applies and Stage 0A's kill
path gate does.

---

## 0. Parts gate — check before unboxing further

| Item | Why it bites |
|---|---|
| **27 W USB-C PD supply (5 V / 5 A)** | A 5 V/3 A supply boots a Pi 5 but firmware caps total USB current at 600 mA. Adequate for today; required before the CAN HAT and drive electronics land. |
| **micro-HDMI → HDMI cable** | The Pi 5 has two **micro**-HDMI ports, not full size. The single most common first-boot blocker. Use the port nearest the USB-C jack (HDMI0). |
| **USB keyboard** | A monitor without a keyboard gets you a login prompt you cannot answer. |
| **SD card reader on the host laptop** | For writing the card and for recovering a bad `config.txt` in §5. |
| **Active cooler or heatsink** | Not cosmetic for this project. A throttling Pi is a **jitter source**, and jitter is the entire deliverable of Stage 0B. Without one, §6 and §7 numbers are provisional and must be labelled so. |

---

## 1. Write the card — stock Raspberry Pi OS, not our image

Raspberry Pi Imager on the host laptop: **Raspberry Pi 5** → **Raspberry Pi OS (64-bit)** → the
card. The card is scratch; the Stage-0B image overwrites it later, so nothing written here is
precious.

In **OS customisation**, set:

- **Hostname: `overboard-scratch`.** Deliberately **not** `overboard`.
  `scripts/pi/pi-secrets.example.env` reserves `overboard` for the real card, on the grounds that
  `overboard.local` resolving to the wrong machine while a motor is armed is a bad afternoon. Do
  not create that collision on day one.
- **Username: `overboard`** — matching the real image, so nothing about the login changes later.
- **SSH: enabled, public-key only.** The same key that will go on the real card
  (`~/.ssh/id_ed25519.pub`). No agent generates or handles a keypair — reference doc §7.
- **Wi-Fi** with the correct **two-letter country code**. An unset country disables 5 GHz and
  presents as "the network is not there" rather than as a configuration error.

## 2. First power-on

Card in, monitor to HDMI0, keyboard in, **power last**. The Pi 5 boots on power apply; the button
is for shutdown and wake.

**Expect:** splash → boot text → desktop or login, inside a minute.

**Falsifies:** no green LED activity and a dark screen across two attempts with a re-written card.
That is a DOA claim worth making — escalate for RMA rather than debugging it.

## 3. Confirm the hardware is what the pins assume

```sh
cat /proc/device-tree/model; echo
uname -a
vcgencmd measure_temp
```

**Pass:** `Raspberry Pi 5 Model B Rev 1.x`, a `-rpi-2712` kernel, idle temp below ~60 °C. The
2712 flavour here is **correct** — it is the firmware default, and it is precisely what Q12 is
about.

## 4. Update the bootloader EEPROM, and record the version

```sh
sudo rpi-eeprom-update -a
sudo reboot
# after reboot
vcgencmd bootloader_version
rpi-eeprom-update
```

**Purpose:** the bootloader lives in EEPROM **on the Pi, not on the SD card**. It is therefore an
input to every latency number Stage 0B produces that no image pin can capture, and
`scripts/pi/pins.env` does not currently name it. Update it while the card is disposable, then
record the version so the number is attributable.

**Follow-up, not done here:** add the bootloader version to `pins.env` alongside the kernel pin,
for the same reason the kernel is pinned — two numbers taken across a bootloader change are not
obviously comparable, and nothing in the data says so.

---

## 5. Close Q12 — which kernel the firmware actually selects

The highest-value step available before the image exists.

```sh
ls -la /boot/firmware/*.img
grep -v '^#' /boot/firmware/config.txt | grep -v '^$'
uname -r
```

**Expect:** both `kernel8.img` (generic v8, 4K pages) and `kernel_2712.img` present; no `kernel=`
line in `config.txt`; `uname -r` reporting the 2712 flavour — i.e. firmware auto-selected 2712.

Now exercise the override the image requires, since the pin is
`linux-image-6.12.75+rpt-rpi-v8-rt` and **there is no `rpi-2712-rt` build in the archive**
(`pins.env`; verification doc):

```sh
sudo cp /boot/firmware/config.txt /boot/firmware/config.txt.bak
echo 'kernel=kernel8.img' | sudo tee -a /boot/firmware/config.txt
sudo reboot
# after reboot
uname -r
```

**Pass:** the Pi boots and `uname -r` reports the v8 flavour rather than 2712. Q12 is then closed
by observation, and the image build knows exactly what `config.txt` must contain.

**Fail:** the Pi does not boot — no login prompt, no SSH. **This is a significant finding, not a
mishap:** it means the image as designed would ship a card that does not boot, and it must reach
Senior Controls before I1 goes green. Recover with §5.1 and report it.

Record the outcome either way. A confirmed pass is as load-bearing as a fail.

### 5.1 Recovery — only if the Pi did NOT boot

**On a pass, skip this section entirely.** It is a repair procedure for a failure, not a further
step of the test.

**If the Pi still boots, you do not need the card reader.** Undo the change over a normal shell:

```sh
ls -la /boot/firmware/config.txt*          # confirm the .bak is there
sudo cp /boot/firmware/config.txt.bak /boot/firmware/config.txt
sudo reboot
```

**Only if there is no way in at all** — no console, no SSH — go via the card:

1. Power down. `sudo shutdown -h now` if a shell exists; otherwise pull power once the activity
   LED settles.
2. Move the microSD to the host laptop's reader.
3. macOS mounts the FAT boot partition as **`/Volumes/bootfs`**. It is the *only* partition that
   appears — the rootfs is ext4 and macOS cannot read it. Expected, not a fault.
4. Restore the file. No `sudo`: FAT32 carries no Unix permissions.
   ```sh
   ls /Volumes/bootfs/config.txt*
   cp /Volumes/bootfs/config.txt.bak /Volumes/bootfs/config.txt
   ```
5. **No `.bak`?** Edit in place and delete the offending line with `nano /Volumes/bootfs/config.txt`
   — **not** TextEdit, which can save rich text and corrupt a file the firmware then cannot parse.
6. Eject cleanly: `diskutil eject /Volumes/bootfs`. Pulling the card mid-flush turns a config
   problem into a re-flash.
7. Card back in the Pi, power on.

**Restoring after a pass is optional.** The card is scratch and the Stage-0B image overwrites it.
The only reason to go back to 2712 is to capture the second §6/§7 baseline — and since **v8 is the
flavour the image ships**, a single baseline is better taken on v8.

---

## 6. Non-RT baseline — `loop-profiler`

`crates/loop-profiler` was built for this hour (PR #181). It **cannot turn a motor** —
structurally, not by convention: the crate does not depend on `hal-actuate`, so it has no
`apply()` to call and no backend to arm.

```sh
sudo apt update && sudo apt install -y git build-essential
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source "$HOME/.cargo/env"
git clone https://github.com/MikePaNtZ/overboard.git && cd overboard
cargo build --release -p loop-profiler
sudo ./target/release/overboard-loop-profile \
  --rate-hz 500 --cycles 100000 --rt-prio 80 --json ~/baseline-stock.json
```

## 7. Non-RT baseline — `cyclictest`

```sh
sudo apt install -y rt-tests stress-ng
sudo cyclictest -m -Sp95 -i 2000 -D 5m
```

Run both §6 and §7. `cyclictest` measures the kernel; the profiler measures **our loop on** that
kernel. Them disagreeing is informative: `cyclictest` clean and the profiler dirty means the
problem is ours.

### How to read these numbers — two caveats, stated in advance

AC-5 asks for p99.9 wakeup latency ≤ 150 µs and max ≤ 500 µs. A stock non-RT kernel will very
likely miss that, **and that is the expected result rather than a bad sign.** But:

1. **This is not a clean control.** The delta from here to the image changes **two** variables at
   once — non-RT → RT, *and* 2712/16K-page → v8/4K-page. Treat it as a sanity floor, not as an
   isolated measurement of what PREEMPT_RT bought.
2. **If stock already meets AC-5, that is the interesting outcome.** It would materially weaken
   the case for the RT-kernel path and the 16K-page cost the design accepts in §4, and it should
   be escalated rather than filed.

---

## 8. Optional — rehearse the flash path against a real secrets file

There is nothing of ours to flash yet, but the credential-staging path can be validated now so a
typo does not surface after a 20-minute write later:

```sh
mkdir -p ~/.overboard && cp scripts/pi/pi-secrets.example.env ~/.overboard/pi-secrets.env
chmod 600 ~/.overboard/pi-secrets.env
$EDITOR ~/.overboard/pi-secrets.env       # real SSID/passphrase/pubkey; keep PI_HOSTNAME=overboard
scripts/pi/flash_pi.sh --disk /dev/diskN --image ~/Downloads/<stock-raspios>.img.xz --dry-run
```

`--dry-run` runs every guard and generates the boot payload, skipping only the write. Find the
disk with `diskutil list external` (macOS) or `lsblk -o NAME,SIZE,TYPE,RM,MOUNTPOINT` (Linux)
first.

**What this does and does not do.** `--image` points at the stock image purely to satisfy the
resolver; the exercise validates the removable-media guard, the confirmation prompt and
`mk_boot_config.py`'s output against a real secrets file. It is **not** a way to produce a
usable card: the staged credentials assume `firstboot_install.sh`, which stock Raspberry Pi OS
does not ship. Use Imager's own customisation (§1) for the card you actually boot.

---

## Do not

- **Do not** connect the CANable, a CAN HAT, the Little FOCer, or the motor. This is a
  compute-platform session.
- **Do not** run `rpi-update`. The design rejects it outright — unversioned bleeding-edge
  kernels, incompatible with anything reproduced from a manifest.
- **Do not** `apt full-upgrade` and then treat §6/§7 as a baseline. Capture the numbers first, or
  record exactly what was upgraded.

## What this runbook does NOT establish

No claim about command→actuation latency (AC-6), the CAN transport, the SPI tail-latency risk, or
the RT kernel's real behaviour. Every one of those needs the image, the bus and the drive. What
transfers forward is a known-good board, a recorded bootloader version, a resolved Q12, and a
non-RT floor to measure the image against.

**Next:** the Stage-0B image (issue #182, PR #197), then
[`runbook-stage0b-bench.md`](runbook-stage0b-bench.md).
