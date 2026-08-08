# Reference — Stage-0B Pi image: schemas, thresholds, verification and open questions

<!--
covers:
  - crates/hal/src/lib.rs
  - crates/hal-actuate/src/lib.rs
  - crates/safety/src/lib.rs
  - crates/xtask/src/main.rs
  - crates/vesc-tx/src/lib.rs
  - crates/vesc-wire/src/lib.rs
reconciled: 25012f4
-->

- **Parent design:** [`design-pi-image-stage0b.md`](./design-pi-image-stage0b.md) — the decisions
  (D1–D6), their rationale, and the rejected alternatives live there. This document is the
  companion split called for by [issue #54](https://github.com/MikePaNtZ/overboard/issues/54):
  the operational detail a decision doc shouldn't have to carry to stay under ADR-0008's cap —
  schemas, exact version pins, thresholds, and the open-question ledger. **Nothing here overrides
  a decision in the parent doc; this is where its numbers and field lists live.**
- **See also:** [`design-pi-image-stage0b-verification.md`](./design-pi-image-stage0b-verification.md)
  — a narrower, older split of just §4's (kernel) archive-verification method. That document is
  unchanged by this one; both are companions to the same parent.
- **Status:** operational reference. §7 (acceptance criteria) is normative — those numbers gate
  ratification exactly as they did when inline in the parent. Everything else here is schema,
  evidence, or a parked question, not a decision in its own right.

---

## 1. Provenance manifest — full field schema

### Ownership handoff mechanics (parent design §1)

`check_ownership()` fails on any new top-level directory with no explicit `CODEOWNERS` rule
(ADR-0002 calls this tax "the point"), and `CODEOWNERS` is COO turf. To make this a copy-paste
approval, the exact lines requested are `# role: Senior Controls` / `/pi/  @MikePaNtZ`.
**Fallback if unmerged at implementation time:** land under `scripts/pi/`, already Senior
Controls turf, and move later — a top-priority deliverable shouldn't be parked on one line of
another role's file.

### Provenance manifest field list (parent design §2)

Published beside the image *and* written into it at
`/etc/overboard-image.json`, so a running Pi can state its own identity. Fields: `git_sha`;
`built_at` + `workflow_run_url`; `base_image` (upstream release identifier + SHA256);
`packages[]` (**every** installed package as `name=version=arch`, from `dpkg-query`); `kernel`
(package name + exact version — parent design §4); `config_txt_sha256`, `cmdline_txt_sha256` and
`overlays[]`; and `rust_binary_sha256` + `rust_toolchain` for the control binary that shipped in
it.

GitHub Release assets are capped at 2 GB per file, so the artefact is published as **`.img.xz`**
(what Raspberry Pi Imager consumes anyway).

---

## 2. Image-builder fallback — derive-from-stock, in detail

*(Parent design §3 — image builder.)* If the `rpi-image-gen` spike fails its half-day timebox,
the fallback is: take the official **Raspberry Pi OS Lite arm64** release image, loop-mount,
`chroot` (native arm64, no QEMU), `apt install` the pinned RT kernel, apply the
`config.txt`/overlay/isolation/systemd deltas, repack, compress, hash. Small delta from stock, no
support caveat, fast, identical deliverable shape to the parent design's §2 — weakness is that
it's our own bash, not declarative configuration, hence fallback not primary.

---

## 3. Kernel — verification detail and the exact pin

*(Parent design §4 — kernel decision.)* The decision, its rationale, and the fallback ladder live
in the parent doc. This section is the supporting detail that made the decision safe to write down
rather than guess.

**Config diff, `rpi-2712` (Pi 5 default) vs `rpi-v8-rt` (the RT flavour), same version:**

| | `rpi-2712` — Pi 5 default | `rpi-v8-rt` — the RT one |
|---|---|---|
| Preemption | `CONFIG_PREEMPT_BUILD=y`, **not** RT | **`CONFIG_PREEMPT_RT=y`** |
| Page size | `CONFIG_ARM64_16K_PAGES=y` | **`CONFIG_ARM64_4K_PAGES=y`** |
| `CONFIG_HZ` | 250 | 250 |
| `CONFIG_NO_HZ_FULL` | — | **not set** |

**Two things that could have quietly invalidated the parent design's kernel choice, both
checked:**

1. **RP1 support in the generic v8 flavour.** Shipping `bcm2712-rpi-5-b.dtb` is necessary but not
   sufficient — a Pi 5 without RP1 has no usable SPI, GPIO or Ethernet, and the answer would
   have been "Pi 4 / CM4", rewriting the parent document. The RP1 stack is **identical in both
   flavours** (`CONFIG_MFD_RP1`, `CONFIG_PINCTRL_RP1`, `CONFIG_COMMON_CLK_RP1`,
   `CONFIG_PCIE_BRCMSTB`, `CONFIG_PWM_RP1`, `CONFIG_RP1_PIO`, `CONFIG_MACB`, all `=y`/`=m`).
   **The v8-rt kernel is a complete Pi 5 kernel.**
2. **The CAN path survives the flavour switch:** `mcp251xfd.dtbo`, `mcp251xfd.ko`, `vcan.ko`,
   `CONFIG_SPI_DESIGNWARE=m` (the RP1 SPI driver) and the full SocketCAN module set all present.

Full archive-verification method (how these were checked, package-by-package) is in the older,
narrower split: [`design-pi-image-stage0b-verification.md`](./design-pi-image-stage0b-verification.md).

**Exact candidate pin:** `linux-image-6.12.75+rpt-rpi-v8-rt` `1:6.12.75-1+rpt1` is the current
candidate for the parent design's "≥ 6.12, never the floating metapackage" decision; **≥ 6.12 is
the hard floor**, not 6.12.75 specifically.

---

## 4. Bench-tooling interface reference

*(Parent design §5 — bench tooling.)* The decision — one entry point, `--backend sim|hardware`,
resolved across the existing `BoardObserve` / `BoardActuate` seam — is in the parent doc. The
seam it resolves across, as it exists in code today:

- `hal::BoardObserve` — `open`, `close`, `wait_observe` (the sole time-advancing call),
  `run_metadata`.
- `hal_actuate::BoardActuate` — `arm() -> Disarm`, `apply(&Command)` (enqueues, never advances
  time), plus the `CallSequence` guard enforcing zero-or-one `apply()` per `wait_observe()`.
- `xtask gate` proves, over the `cargo metadata` graph under `--all-features`, that
  `board-app-ridden` cannot reach `hal-actuate` — with `canary-ridden` as the positive control.

---

## 5. CAN transport — supporting evidence, and cheapest sequencing

*(Parent design §6 — CAN transport decision.)* The decision (SPI HAT primary, USB-CAN as a
second transport on day one, PCIe-CAN as a named but unbought escalation tier) and its core
rationale are in the parent doc. This is the supporting material for *how seriously to weight*
the community SPI-tail-latency report that motivates the second transport.

**The report itself.** A Pi 5 report under PREEMPT_RT (kernel 6.1.70-rt21) measured **SPI
transaction times spiking to 1.5–2 ms under combined CPU and network load**, while scheduler
wakeup latency stayed under 100–150 µs. Raising RT priority on the SPI and DMA interrupts did
not fix it. No Raspberry Pi engineer replied; no root cause was established; the reporter
believed it to be RP1-specific, having seen a CM4 be "rock steady".

**Why the report is suggestive but not dispositive**, so nobody over- or under-reacts:

1. It measured **userspace `spidev`** transactions of 2 KB. `mcp251xfd` is a *kernel* driver with
   a much smaller-transfer, GPIO-IRQ-driven path. If the cause is RP1/PCIe or DMA interrupt
   latency, the kernel path eats it identically; if it's `spidev` ioctl/worker scheduling, the
   kernel path may be much better. **The report can't distinguish these** — argues for
   measuring, not abandoning.
2. It was on **6.1.70-rt21**, the out-of-tree-RT era with immature RP1 support; we will be on
   6.12.x with mainlined PREEMPT_RT. May simply be fixed.
3. Single, unreplicated, no engineer response.

### Cheapest possible sequencing, given nothing is purchased

**Buy the Pi 5 now. Run AC-9's SPI reproduction on the pinned RT kernel *before* buying the CAN
HAT** — zero incremental hardware, one evening, clears or kills the HAT before money moves.
**This information will never be cheaper than it is now.**

If both transports show tails beyond one control period, that is a genuine architecture finding
and it escalates — surfacing it is a success of the stage, not a failure. Numeric triggers:
AC-6, AC-8, AC-9.

---

## 6. Safety — the bounded envelope, abort path, who must be present

*(Parent design §7.1 — the Gate S1 invariant and its rationale stay in the parent doc. This
section is the operationalized detail: the layer table, the provisional numbers, the abort
mapping, and who must be in the room.)*

### The bounded envelope — four layers, only two of which survive our process dying

| # | Layer | Where enforced | Survives |
|---|---|---|---|
| 0 | **Deadman contactor**, in series with pack positive | physical, within arm's reach | everything |
| 1 | **Controller current ceiling** — Little FOCer motor/battery current + ERPM limits, set in its own configuration | firmware on a separate device | any Pi-side failure |
| 2 | **`safety::Envelope` clamp** — `max_current_a`, symmetric | in-process, every cycle | logic errors above it |
| 3 | **Controller command timeout** — output released if no command arrives within the timeout | firmware on a separate device | process death, hang |

Layers 0 and 1 are the ones that matter, because **they are the only two that keep working when
our process does not.** Layer 2 is defence in depth and Layer 3 depends on a firmware behaviour
this project has not yet verified (Q3, §11).

**Numbers.** These are `PROVISIONAL` and must be re-ratified against Stage-0A §6c data before
the first powered agent-driven run. What is *not* provisional is that each is enforced at
Layer 1 **and** Layer 2, and that the build refuses to run if they disagree:

| Parameter | Provisional value | Set at |
|---|---|---|
| Motor current, agent-driven bench runs | **≤ 5 A**, and never above the maximum already demonstrated by hand in Stage-0A §6c | L1 + L2 |
| Absolute ceiling, any configuration | **≤ 15 A** | L1 |
| Maximum continuous non-zero command | **≤ 3 s**, then forced zero | L2 |
| Speed ceiling | **≤ 3000 ERPM** | L1 |
| Controller command timeout | **≤ 100 ms** (50 control periods) | L3 |

### Abort path, and the three failure modes named in the brief

- **Deliberate abort:** open the deadman. Always available, needs no software, the only abort
  anyone is asked to remember under stress.
- **Comms loss** (CAN silent, cable pulled): Layer 3 releases output within the timeout; Pi side
  disarms on the first `wait_observe()` error.
- **Process death** (`SIGKILL`, panic, OOM): **destructors do not reliably run**, so the RAII
  `Disarm` guard can't be trusted here — the architectural point of the whole section: **the
  abort covering process death must live in a device that keeps running when our process dies.**
  Layers 1 and 3 do; Layer 2 does not.
- **Hung loop, still sending a stale non-zero command** — the nastiest case, since Layer 3's
  timeout never fires. Only Layers 0 and 1 cover it — why the current ceiling is set low enough
  that a stuck-on command is a nuisance, not an injury.

### Who must be present

**The first powered run of any agent-authored code requires the CEO physically present, hand on
the deadman** — also the first run after any change to envelope parameters, the `hardware`
backend, or the kernel pin.

**Not required for:** unpowered bring-up, boot/device-tree verification, `cyclictest`, `vcan`
work, image builds, any dry run on `--backend sim`. Distinguishing these keeps the CEO's
attention available for the runs that need it.

No agent transmits a command to a motor without a human in the room. The control loop contains
no AI; the parent design's §5 seam is what keeps it that way.

---

## 7. Numeric acceptance criteria (D0 ready-to-code gate)

Adjectives fail this gate. Each criterion below is measurable and has a stated instrument.

> **The go/no-go threshold is pre-registered — AC-6 is fixed before anything is measured.** A
> number with no threshold attached is not a decision; it is a number a threshold gets fitted to
> afterwards. AC-6 exists so that cannot happen.
>
> **All latency criteria are tail percentiles — p99, p99.9, max — never means.** A 2 ms spike at
> the 99.9th percentile is invisible in a mean, and a mean is exactly what an under-specified
> measurement protocol will produce.

| # | Criterion | Threshold | Instrument |
|---|---|---|---|
| **AC-1** | **Package pinning.** Every directly-installed package pinned to an explicit version | **100%**; build fails if any pin is unsatisfiable | build log + `dpkg-query` manifest |
| **AC-2** | **Kernel pin held.** The RT kernel is an explicit version, `apt-mark hold`-ed, never the floating metapackage | exact version recorded in the Release manifest; a floated kernel fails the build | parent design §2 |
| **AC-3** | **Manifest diffability.** Two CI builds of the same git SHA produce diffable manifests and identical `config.txt` / `cmdline.txt` / overlay set | **0-line diff** on boot configuration; package manifest diff **reported, not required to be empty** (no snapshot mirror exists — parent design §2) | manifest diff in CI |
| **AC-4** | **Provenance completeness.** Every field in §1's table present and non-empty | **100%** | schema check on `/etc/overboard-image.json` |
| **AC-5** ✅ **PASS 2026-08-07** | **RT scheduling jitter.** `cyclictest -m -Sp95 -i 2000 -D 30m` under `stress-ng` CPU + network load | p99.9 wakeup latency **≤ 150 µs**; max **≤ 500 µs** (25% of the 2 ms period). **Measured: p99.9 = 72 µs, max = 113 µs**, n = 2,699,993, `throttled=0x0`. See Q1 for the two recorded caveats (isolated core not measured; loopback-only network load). | `rt-tests` 2.6-1.1 |
| **AC-6** | **Command→actuation latency — the pre-registered go/no-go, amended per [issue #113](https://github.com/MikePaNtZ/overboard/issues/113)** | **Go** if total loop delay (sampling + transport + current-loop lag) p99.9 **≤ 20 ms**, over **≥ 10⁵ cycles** under representative load. Derived, not a fraction of the control period: [`design-delay-budget-stage0b.md`](./design-delay-budget-stage0b.md) measures the RIDDEN/cascade closed loop's actual delay-margin ceiling at 38–39 ms (estimator ON, the honest default) and sets 20 ms at roughly half that, clear of both the ~6.3 ms known transport-adjacent budget and the estimator's own ~21 ms measured cost. Any p99.9 > 20 ms escalates as an architecture finding rather than being tuned away. **Superseded: the original `p99.9 ≤ 1 ms / max ≤ 2 ms` (halves and wholes of the 2 ms period) — plant-ignorant, see the linked doc for why** | MCAP log, offline analysis |
| **AC-7** | **Measurement resolution.** Timestamp resolution on the AC-6 measurement | **≤ 10 µs**, via `SO_TIMESTAMPING` on the CAN socket; hardware timestamps preferred, source recorded per run | harness self-report |
| **AC-8** | **Transport attribution.** AC-6 repeated over both `mcp251xfd` and USB-CAN in one session, same rig, same harness | both reported as p99 / p99.9 / max. A **>2×** difference in p99.9 attributes the tail to the transport rather than the architecture | parent design §6 |
| **AC-9** | **SPI tail reproduction** (runs *before* the HAT is purchased). **Precondition: a >64 B `spidev` transfer must complete without a `-110` DMA timeout (§11 Q13) before the timed run counts** — a run that can't tell "our config is slow" from "this kernel's `spidev` is broken above 64 B" measures nothing | `spidev` loopback, `stress-ng`+`iperf3`, >64 B transfers: (1) confirm no `-110`; then (2) p99/p99.9/max. **Any `-110` blocks the HAT purchase and reopens Q13 as live on our hardware. Max > 2 ms once completing cleanly also blocks it**, per the parent design §6's escalation tier | parent design §6, §11 Q13 |
| **AC-10** | **CAN integrity** over the AC-6 run | **0** dropped frames; **0** bus-off events | `ip -s -d link show` |
| **AC-11** | **Restore test** — the real acceptance gate for the image. Flash the *published* artefact to a blank card and boot it | pinned `uname -r` matches; `can0` up at the intended bitrate; AC-5 tail met. **10/10** consecutive cold boots, ready in **≤ 60 s** | first-boot log |
| **AC-12** | **No-Pi coverage.** Fraction of §8's "verifiable" column running in CI with no hardware | **100%**, green on every PR touching the image tree | CI |
| **AC-13** | **Safety gate S1.** `xtask gate` proves no crate reaches `hal-actuate` without `safety`, with a canary proving the rule fires | build fails otherwise | `cargo run -p xtask -- gate` |
| **AC-14** | **Envelope agreement.** Layer-1 controller limits and Layer-2 `max_current_a` read back and compared at arm time | mismatch → refuse to arm | `hardware` backend |

---

## 8. What is verifiable **without** a Pi — and what is not

No hardware has been purchased. This section is why that is not blocking.

### Verifiable in CI today, no Pi

| Thing | How | Confidence |
|---|---|---|
| **Package names and versions resolve** | `apt-get install --dry-run` with the pinned set, in a `debian:trixie` arm64 container with the Raspberry Pi archive added | **High** — catches a fabricated pin; the verification log shows it already caught the `-2712-rt` assumption |
| **Kernel ships what we need** | unpack the `.deb`; assert `mcp251xfd.ko`, `mcp251xfd.dtbo`, `vcan.ko`, `bcm2712-rpi-5-b.dtb` and `CONFIG_PREEMPT_RT=y` | **High** — already done by hand (see the verification log) |
| **Overlay compiles** | `dtc` on the overlay source | **High** — already established |
| **The whole CAN stack, end to end** | `vcan0` + a **simulated VESC responder**; `hardware` backend talks to `vcan0` exactly as `can0`. Exercises framing, socket setup, timeouts, error paths, MCAP capture, arm/disarm | **High for logic, zero for timing** |
| **Bench harness logic** | `--backend sim` against `bench_spinup.py`; `canplayer` replay of recorded frames | **High** |
| **`cyclictest` harness** | builds/self-tests on any Linux; on non-RT it produces *bad* numbers, still enough to prove parsing, thresholds, capture, upload work | **High for the harness, not the numbers** |
| **Image builds at all** | full builder run on the arm64 runner; artefact produced and hashed | **High** |
| **Safety gate S1** | `xtask gate` — pure `cargo metadata` | **High** |

### Requires the hardware. No substitute exists.

- **That the image boots**, and that the 4K-page v8-rt kernel boots on *this* Pi 5.
- **Device-tree load at runtime** — that `can0` appears, at the intended bitrate, on the intended
  SPI controller, not sharing with the IMU.
- **All real timing**: AC-5 through AC-11, and the parent design §6 SPI tail-latency question.
  **Every number Stage 0B exists to produce is in this list** — the definition of the stage, not
  a defect of it.
- **Thermals**, and whether sustained 500 Hz operation throttles.
- **Controller command-timeout behaviour** (Layer 3) — needs a Little FOCer to observe.
- **Real `vesc-wire` / `vesc-tx` byte layouts.** Deliberately honest stubs returning
  `NotYetImplemented` — fabricating VESC constants from memory into a crate that gates actuation
  was already judged the worst available artefact, and this design doesn't reverse it.

---

## 9. Credentials

**Placed by the CEO. Never generated, held, transmitted or logged by an agent.**

The image ships **generic and secret-free**: no user password, no WiFi credentials, no SSH keys,
no cloud tokens. It is published to a **public** GitHub Release and must be harmless to anyone
who downloads it.

Mechanism, using stock Raspberry Pi OS first-boot customisation:

1. The CEO flashes the image, then either uses **Raspberry Pi Imager's OS customisation**
   (hostname, user, password hashing, WiFi, SSH), or writes the `bootfs` files by hand: an empty
   **`ssh`** file, and **`userconf.txt`** holding `<username>:<hash>` from `openssl passwd -6`.
   Both are documented Raspberry Pi mechanisms.
2. **SSH: public key only, password authentication disabled** in the shipped `sshd_config`. The
   CEO copies his own public key to the card; no agent ever generates a keypair — closing the
   obvious free shot a reviewer would take on plaintext credentials on a FAT boot partition.
3. **Upload credentials** (§10) go in a file at a documented path, mode `0600`, read at runtime,
   and **excluded from MCAP logs and from the provenance manifest by name**.
4. The repo ships a `credentials.example` with **placeholder values only**; CI fails if a file
   matching the real credential path is ever tracked.

A community-reported alternative — `custom.toml` on the boot partition, Bookworm+ — may be more
convenient, but was **not** confirmed against official docs today; verify before it's written
into a runbook.

---

## 10. Data path: capture on the Pi, upload, analyse offline

One schema, and it already has a home in the code: **MCAP**, with `board_types::RunMetadata` as
the header payload (ICD §6.2 — the doc comments in `crates/hal/src/lib.rs` and
`crates/board-types/src/lib.rs` already say so). Foxglove for viewing, per the project
convention. No MCAP writer exists yet; writing one is implementation work, not design.

`Pi: run → MCAP to local disk → upload → upload completes = analysis trigger`, analysed offline
by Senior Controls against sim predictions (`bench_spinup.py`'s `replay` mode already compares
sim to a measured CSV).

Three rules, in priority order:

1. **Capture never blocks the loop.** The writer is on a non-real-time thread behind a bounded
   queue. **A full queue drops samples and increments a counter that is logged** — never
   back-pressure on a 2 ms loop. A dropped-sample count is recoverable; a blocked loop is a
   safety event.
2. **No analysis on the Pi.** The Pi captures and uploads, nothing else — mirrors runbook §8:
   not done at the bench, not by the person holding the deadman.
3. **The upload is the trigger.** Analysis begins when the artefact lands, not when someone
   remembers to ask.

---

## 11. Open questions — named, not papered over

Each carries a default action so nobody is parked.

| # | Question | Default if unanswered |
|---|---|---|
| **Q1** ✅ **CLOSED 2026-08-07 — YES** | Is the 4K-page, no-`NO_HZ_FULL` packaged RT kernel good enough for a 2 ms loop on Pi 5? **Unknowable without hardware.** | **Answered by measurement. AC-5 passes with margin on the packaged kernel:** p99.9 **72 µs** (limit 150) and max **113 µs** (limit 500), over 2,699,993 samples in a 30-minute `cyclictest -m -S -p95 -i 2000` run under `stress-ng --cpu 4 --sock 2`, `throttled=0x0`. Max is 5.7% of the 2 ms period. **The parent design §4 fallback ladder is not needed** — no `isolcpus` retune, no reduced loop rate, no custom kernel build, no moving the inner loop off Linux. The 4K-page and no-`NO_HZ_FULL` costs the design accepted are real but not binding at 500 Hz. Two caveats recorded rather than buried: the run measured CPUs 0–2, **not** the `isolcpus=3` core the loop will use (conservative, since those are the noisy cores), and the network stressor ran on loopback only because WiFi never came up. |
| **Q2** | **Is the RP1 SPI tail-latency report real on ≥6.12 with the `mcp251xfd` kernel driver?** The largest risk here. | The parent design §6's dual-transport measurement answers it with our own data |
| **Q3** | Little FOCer command-timeout semantics and default (Layer 3). Unverified — no hardware, and no VESC constants are recorded in this repo. | Do not rely on Layer 3. Layers 0 and 1 carry the safety case until measured |
| **Q4** | Do `rpi-image-gen` / `pi-gen` officially support Pi 5? Neither README confirmed it. Is `debian:trixie` on an Ubuntu arm64 runner an acceptable host? | The parent design §3's half-day spike answers both before implementation starts; derive-from-stock is the fallback |
| **Q5** | **SR-SIM-5** — Rust or Python hosts the loop? Open elsewhere; the parent design §5 survives either. | Gate S1 (parent design §7.1) binds regardless; this design supplies evidence for "Rust owns the loop" |
| **Q6** | Are the §6 provisional current limits right? Placeholders with no measured basis. | Re-ratify against Stage-0A §6c before the first powered run. **Blocking** |
| **Q7** | Which SPI controller for CAN, and which for the IMU (ICD §4.3)? A wiring decision. | Mechanical's call; record before the HAT is ordered |
| **Q8** | Does `interrupt=25` in the overlay conflict with anything else on the HAT stack? | Verify when the HAT is selected |
| **Q9** | Upload destination and retention. AWS is the convention; nothing is provisioned. | Local capture + manual upload for Stage 0B. Do not block on cloud plumbing |
| **Q10** | 6.12 or 6.18 kernel line? Both packaged with RT. | 6.12 (parent design §4); revisit if a needed fix is 6.18-only |
| **Q11** ✅ **CLOSED 2026-08-07 — NO** | Does the packaged RT kernel carry a Pi-5-specific regression the 2712 flavour does not? v8 is the less-travelled path on this board. | **No regression found.** The image boots on `6.12.75+rpt-rpi-v8-rt` (`uname -a` confirms `PREEMPT_RT`), and the kernel then met AC-5 with margin under load for 30 minutes. Both detectors named here fired clean. Not exhaustive — this exercises boot and scheduler latency, not the RP1/SPI path, which AC-9 still owns. |
| **Q12** ✅ **CLOSED 2026-08-07** | How does Pi 5 firmware select the v8-rt kernel, given it defaults to `kernel_2712.img`? **Operational, not architectural** — but it is the difference between a card that boots and one that does not. | **Two-part answer, both parts measured.** (1) *Mechanism*, 2026-08-05: a `kernel=` line in `config.txt` overrides firmware auto-selection on a Pi 5. On stock Raspberry Pi OS, `uname -r` moved from `6.18.34+rpt-rpi-2712` to `6.18.34+rpt-rpi-v8` across one reboot, `auto_initramfs=1` resolving the matching initramfs unassisted. (2) *Filename*, 2026-08-06 — **and this is where the first answer was wrong**: the pinned RT package installs as **`kernel8_rt.img`**, not `kernel8.img`, pairing with `initramfs8_rt`. `config.txt` named a file that was not on the card, and the first real card would not have booted. Caught by inspecting the boot partition before power-on. Fixed in #247, which also makes `build_image.sh` fail the build when `config.txt` names a kernel absent from the built image — so this class of error is now a red build rather than a dark screen. Confirmed end to end 2026-08-07: the image boots on `6.12.75+rpt-rpi-v8-rt` with `PREEMPT_RT`. **The lesson worth keeping:** the original value rested on "confirmed present in the spike's build output", but the spike built rpi-image-gen's *stock* config, not our RT kernel — a confirmation that never covered the thing it was cited for. |
| **Q13** | **A second, distinct RP1-SPI defect: `raspberrypi/linux` #6020 / #5696 — `spidev` transfers over 64 bytes failing outright with a DMA timeout (`-110`), not just slow.** Different from the parent design §6's tail-latency report. Researched below, not assumed. | AC-9's precondition (§7) checks it against the pinned kernel before the timed run counts |

**Q13, researched directly against GitHub, not taken on either reviewer's word (2026-07-27):**
#5696 (2023) was the same symptom, closed as fixed. #6020 (opened 2024-03-08) is a
**recurrence** — `pelwell` (RPi kernel maintainer) traced it to a non-atomic read-modify-write
of the DMA controller's `INT_EN` bit and shipped **PR #6044**, merged 2024-03-15 to `rpi-6.6.y`;
the reporter confirmed the original >64-byte timeout was fixed. **I fetched
`dw-axi-dmac-platform.c` from the pinned `rpi-6.12.y` branch and confirmed by inspection that the
fix is present** — `axi_dma_enable()` is called only from `axi_dma_resume()`, not from the
per-block path that caused the race. **What I could not confirm:** the same thread then surfaced
a second, unresolved problem — SPI clocking broke on a second SPI interface once both were
driven concurrently. No follow-up confirmation exists; #6020 **is still open**, no activity
since 2024-03-21. Relevant because ICD §4.3 runs CAN and the IMU on separate, concurrently-used
SPI controllers — the same shape. Not asserting we will hit it, only that "the >64-byte timeout
is fixed" and "dual concurrent SPI is clean" are different claims and only the first is
verified. AC-9 is amended above to check both on our own hardware.

---

## 12. See also

- [`design-pi-image-stage0b.md`](./design-pi-image-stage0b.md) — the parent: decisions D1–D6,
  their rationale, rejected alternatives, safety Gate S1, and scope.
- [`design-pi-image-stage0b-verification.md`](./design-pi-image-stage0b-verification.md) — how
  §4's (kernel) claims were checked against the archive directly, method and evidence.
