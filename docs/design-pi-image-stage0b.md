# Design — Stage-0B Pi image, provisioning, and bench tooling

<!--
covers:
  - crates/hal/src/lib.rs
  - crates/hal-actuate/src/lib.rs
  - crates/safety/src/lib.rs
  - crates/board-app-driverless/src/main.rs
  - crates/xtask/src/main.rs
  - crates/vesc-tx/src/lib.rs
  - crates/vesc-wire/src/lib.rs
reconciled: 1458f61
-->

- **Status:** Proposed — design only. No implementation is authorised by this document.
- **Owner:** Senior Controls · **Adversarial review:** Sr. Mechanical & Systems · **Approval:** COO
- **Closes (design half of):** [#32](https://github.com/MikePaNtZ/overboard/issues/32)
- **Implementation:** a separate issue, opened only after COO approval is recorded.
- **Size:** §13 (verification log) lives in a separate doc under ADR-0008's cap. **Prune when
  adding** — replace superseded reasoning rather than appending.

## 0. Scope, and the one number this exists to produce

Stage 0A (assembly, kill path, `kt`, friction, step response) needs **no Raspberry Pi** —
motor + Little FOCer Rev4 + CANable 2.0 + laptop. See
[`runbook-stage0a-bench.md`](runbook-stage0a-bench.md).

**Stage 0B needs the Pi, and produces exactly one deliverable that matters: the
command→actuation latency and jitter distribution** — the architecture go/no-go number.
Everything here is subordinate to measuring it honestly.

The control loop is **500 Hz — a 2 ms period** (ICD §11.2; the constant is already in
`crates/board-app-ridden/src/main.rs:25` and `crates/sim-backend/src/lib.rs:37`). Hold that
number in mind; §6 explains why it turns one community bug report into the largest risk here.

Out of scope: the ridden configuration, the hub motor, any balancing claim, and the
`overboard-web` status update (that fires when a capability ships, not when a design lands).

---

## 1. D1 — Repo boundary: a directory inside `overboard`

**Decision: `pi/` at the top level of this repo. Not a new repository.**

The controls-repo rule reads "controls only — Rust, sim, hardware, design docs". An OS image
isn't obviously any of those, so this needs an argument, not an assertion — and the argument is
the **runtime contract**, not file adjacency.

The obvious case for one repo — "the image contains the binary, so their versions must move
together" — is refutable in one line: a well-built image *shouldn't* contain the binary; deploy
it separately and that coupling evaporates. So that's not the reason.

The reason that survives is that the image and the control code share assumptions neither one
states alone: **kernel flavour and page size, `isolcpus`/cpuset layout, RT priority budget, CAN
interface naming and bitrate, systemd unit expectations.** The Rust code encodes them, the image
satisfies them, **nothing detects a mismatch except running the two together** — not a build
failure but a wrong, unattributable latency number at the bench. This repo's CI is the only
place positioned to exercise control code, sim, and image definition against one commit;
splitting the contract across repos with no shared CI is how it drifts silently.

Supporting: the `rust`/`sim` jobs already test the other half of this contract here on every
commit, and "controls only" excludes marketing, not deployment — the same sentence admits
"hardware" and "design docs".

**Cost of the alternative (new repo):** the runtime contract loses its only end-to-end test; a
third ownership map and policy gate to maintain; a third repo for the CEO to track. **Cost of
the choice made:** this repo grows a non-Rust build tree and a slow CI job. Both accepted.

**The ownership handoff — and why it does not block.** `check_ownership()` fails on any new
top-level directory with no explicit `CODEOWNERS` rule (ADR-0002 calls this tax "the point"),
and `CODEOWNERS` is COO turf. To make this a copy-paste approval, the exact lines requested are
`# role: Senior Controls` / `/pi/  @MikePaNtZ`. **Fallback if unmerged at implementation time:**
land under `scripts/pi/`, already Senior Controls turf, and move later — a top-priority
deliverable shouldn't be parked on one line of another role's file.

---

## 2. D2 — Deliverable shape: **(a)**, an image, produced only by **(b)**

**Decision: the deliverable is (a) — a downloadable image with a SHA256 checksum and a
provenance manifest — and it is produced exclusively by (b), a scripted flow in CI. Never by
hand, and never by running a provisioning script against a live mirror on the CEO's card.**

A flash-time script makes **the state of the network at the moment the CEO runs it** part of
the result — the "local setup archaeology" he asked to eliminate, just relocated; two cards
flashed a fortnight apart would differ, surfacing as an unattributable latency anomaly. Building
once, in CI, from pinned inputs confines that nondeterminism to one logged place.

Calling this "(a)" rather than "both" is deliberate: **the artefact the CEO flashes is the
image; the artefact reviewed and rebuilt is the script tree.** If they disagree, the script tree
wins and the image is rebuilt.

### What "reproducible" honestly means here — and what it does not

**`archive.raspberrypi.com` has no snapshot service** — no equivalent of `snapshot.debian.org`.
A rebuild from the same commit next month resolves different packages and produces a different
image; no image-builder's reproducibility claim rescues us from that. Claiming bit-for-bit
determinism would be exactly the confident-and-wrong statement this document is supposed to
avoid. What is actually promised:

1. **Built only by CI, from a tagged commit. Never by hand.**
2. **The RT kernel package version is explicitly pinned and `apt-mark hold`-ed** — never the
   floating `linux-image-rpi-v8-rt` metapackage (resolved to 6.18.34 today, something else
   tomorrow). The pin that matters most: **the kernel is the single variable that invalidates
   every latency number Stage 0B produces.**
3. **A package manifest ships beside the image**, so any two builds diff reviewably even when
   not identical.
4. **A restore test is the real acceptance gate**, not a hash comparison: flash the published
   image to a blank card, boot it, and assert the pinned `uname -r`, `can0` up at the intended
   bitrate, and the cyclictest tail under threshold (AC-11).

GitHub Release assets are capped at 2 GB per file, so the artefact is published as **`.img.xz`**
(what Raspberry Pi Imager consumes anyway).

**Provenance manifest** — published beside the image *and* written into it at
`/etc/overboard-image.json`, so a running Pi can state its own identity. Fields: `git_sha`;
`built_at` + `workflow_run_url`; `base_image` (upstream release identifier + SHA256);
`packages[]` (**every** installed package as `name=version=arch`, from `dpkg-query`); `kernel`
(package name + exact version, §4); `config_txt_sha256`, `cmdline_txt_sha256` and `overlays[]`;
and `rust_binary_sha256` + `rust_toolchain` for the control binary that shipped in it.

Images are **never committed to git** (existing repo rule) — they go to a GitHub Release, the
same mechanism `publish-sim-artifact` already uses for sim renders.

---

## 3. D3 — Image builder: `rpi-image-gen`, gated on a spike

Two candidates, both examined today:

| | `rpi-image-gen` | `pi-gen` (arm64 branch) |
|---|---|---|
| What it is | Raspberry Pi's current tool for **custom** images; assembles from packages | builds Raspberry Pi OS itself; full distro build |
| Reproducibility | README states the goal is reproducible OS artefacts, from pre-built packages rather than source | README makes no reproducibility statement |
| Host | "Debian Bookworm and Trixie arm64 as the supported native hosts"; needs `CAP_SYS_ADMIN`; containers/non-arm64 "not formally supported" | documented `build-docker.sh` container path |
| Maturity | README says "under active development" | long-established; 64-bit docs name Pi Zero 2 / 3 / 4 and **not** Pi 5 |

**Decision: `rpi-image-gen` primary** — the tool Raspberry Pi currently points at for this job,
targets reproducibility explicitly, and assembles from pinned packages instead of rebuilding a
distribution, the property §2 depends on.

The host constraint has a clean answer that didn't exist a year ago: **GitHub
`ubuntu-24.04-arm` hosted runners are generally available and free for public repositories**
(this repo is public). So the build runs **native arm64, no QEMU**, in a privileged
`debian:trixie` container on that runner — close to the stated supported configuration without
being identical (runner OS underneath is Ubuntu). The only unsupported axis is "in a container":
host is arm64, container userspace is literally `debian:trixie`, `--privileged` supplies
`CAP_SYS_ADMIN`, hosted runners are full VMs with working loop devices. Real risk, but narrow.

**Decide it with a spike, not with argument.** Before anything is built, run **one** CI job that
builds `rpi-image-gen`'s own stock example configuration in exactly that host configuration and
produces any `.img` at all. Timebox **half a day**. Green → proceed; otherwise take the
fallback — a schedule risk converted into an afternoon.

**Fallback: derive-from-stock. Explicitly not `pi-gen`** — a lateral move, not a diversified
one (same class of uncertainty, plus rebuilding a whole distribution we don't need). Instead:
take the official **Raspberry Pi OS Lite arm64** release image, loop-mount, `chroot` (native
arm64, no QEMU), `apt install` the pinned RT kernel, apply the `config.txt`/overlay/isolation/
systemd deltas, repack, compress, hash. Small delta from stock, no support caveat, fast,
identical deliverable shape to §2 — weakness is that it's our own bash, not declarative
configuration, hence fallback not primary.

**Open:** neither tool's Pi 5 support was confirmable from its README (`pi-gen`'s 64-bit
documentation names Pi Zero 2 / 3 / 4 only). Almost certainly documentation lag, but unverified
— recorded in §12.

---

## 4. D4 — Kernel: PREEMPT_RT on Pi 5, and what it actually costs

This section is the one most likely to contain a plausible-but-wrong pin, so every claim below
was checked against the archive directly rather than recalled. Method and evidence are in §13.

**The good news is better than expected.** PREEMPT_RT on a Pi is no longer "effectively a
community build". Raspberry Pi **officially package an RT kernel**: `linux-image-rpi-v8-rt`,
maintained by a raspberrypi.com address, in `archive.raspberrypi.com`, with `linux-headers-` and
`linux-base-` counterparts, across both the 6.12 and 6.18 lines.

**The bad news is specific, and it is the real cost.** There is **no RT build of the Pi 5's own
kernel flavour.** The archive carries `rpi-2712` (the Pi 5 flavour) and `rpi-v8-rt` (the RT
flavour) — and no `rpi-2712-rt`. Comparing the two shipped kernel configurations at the same
version:

| | `rpi-2712` — Pi 5 default | `rpi-v8-rt` — the RT one |
|---|---|---|
| Preemption | `CONFIG_PREEMPT_BUILD=y`, **not** RT | **`CONFIG_PREEMPT_RT=y`** |
| Page size | `CONFIG_ARM64_16K_PAGES=y` | **`CONFIG_ARM64_4K_PAGES=y`** |
| `CONFIG_HZ` | 250 | 250 |
| `CONFIG_NO_HZ_FULL` | — | **not set** |

So **choosing RT on a Pi 5 means dropping from the 16K-page, 2712-tuned kernel to the generic
4K-page v8 kernel.** Two consequences, weighted rather than both filed as "loss":

- **Page size (4K vs 16K): largely irrelevant here, possibly helpful.** A 500 Hz loop with a
  small working set is not throughput-bound, and smaller pages can *improve* determinism.
- **`CONFIG_NO_HZ_FULL` unset: the real cost, and a floor, not a knob.**
  `CONFIG_CPU_ISOLATION=y` is set, so `isolcpus=`/cpusets work, but the isolated core still takes
  the 250 Hz tick — a few microseconds of periodic perturbation, probably fine, and **stated in
  advance and included in the measurement** rather than discovered mid-debug.

**Consequently, no claim here that the platform is real-time rests on a config flag** —
`CONFIG_PREEMPT_RT=y` says the kernel was *built* RT, nothing about achieved latency under our
load. That claim is empirical and lives in AC-5.

**Two things that could have quietly invalidated this design, both checked.**

1. **RP1 support in the generic v8 flavour.** Shipping `bcm2712-rpi-5-b.dtb` is necessary but not
   sufficient — a Pi 5 without RP1 has no usable SPI, GPIO or Ethernet, and the answer would
   have been "Pi 4 / CM4", rewriting this document. The RP1 stack is **identical in both
   flavours** (`CONFIG_MFD_RP1`, `CONFIG_PINCTRL_RP1`, `CONFIG_COMMON_CLK_RP1`,
   `CONFIG_PCIE_BRCMSTB`, `CONFIG_PWM_RP1`, `CONFIG_RP1_PIO`, `CONFIG_MACB`, all `=y`/`=m`).
   **The v8-rt kernel is a complete Pi 5 kernel.**
2. **The CAN path survives the flavour switch:** `mcp251xfd.dtbo`, `mcp251xfd.ko`, `vcan.ko`,
   `CONFIG_SPI_DESIGNWARE=m` (the RP1 SPI driver) and the full SocketCAN module set all present.

**Operational unknown, not a design one:** Pi 5 firmware defaults to `kernel_2712.img`. How the
v8-rt kernel gets selected — an explicit `config.txt` `kernel=` line, the packaging's own
handling, or something else — is Q12.

**Pinning.** Pin a specific 6.12-line version, never the floating metapackage (which resolved to
6.18.34 today). 6.12 over 6.18: longer-supported, and comfortably contains the `mcp251xfd`
receive-latency fix (upstream `eb9a839`, backported November 2024, marked for stable).
`linux-image-6.12.75+rpt-rpi-v8-rt` `1:6.12.75-1+rpt1` is the current candidate; **≥ 6.12 is the
hard floor**, not 6.12.75 specifically.

**Fallback ladder if RT latency is inadequate**, cheapest first:

1. Tune what the packaged kernel allows: `isolcpus`, IRQ affinity, `SCHED_FIFO` priorities,
   `performance` governor.
2. Accept a lower loop rate — 500 Hz is an ICD number, not physics; the honest response to a
   platform that can't hold 2 ms is to say so and re-derive the rate.
3. Build our own RT kernel — 2712 flavour, `CONFIG_PREEMPT_RT=y`, `CONFIG_NO_HZ_FULL=y`, 16K
   pages. **The "community build" cost the brief asked about, now a fallback, not the
   baseline:** we'd own a kernel build, patch rebases, and module signing.
4. Move the 500 Hz inner loop off Linux entirely — an architecture change, escalated not
   decided here.

**`rpi-update` is rejected** — unversioned bleeding-edge kernels; right tool for testing, wrong
tool for anything reproduced from a manifest.

---

## 5. D5 — Bench tooling: one launcher, two backends, one seam

**Decision: one entry point, `--backend sim|hardware`, resolved across the existing
`BoardObserve` / `BoardActuate` seam** — so the bench script has been dry-run against the sim
dozens of times before it is ever pointed at a motor.

This extends an existing precedent rather than inventing one: `board-app-driverless` already
takes `--backend <sim|null>`. `hardware` is a third value, and the only one that reaches a motor.

The seam is already the right shape, and PR #49 landed the part that matters:

- `hal::BoardObserve` — `open`, `close`, `wait_observe` (the sole time-advancing call),
  `run_metadata`.
- `hal_actuate::BoardActuate` — `arm() -> Disarm`, `apply(&Command)` (enqueues, never advances
  time), plus the `CallSequence` guard enforcing zero-or-one `apply()` per `wait_observe()`.
- `xtask gate` proves, over the `cargo metadata` graph under `--all-features`, that
  `board-app-ridden` cannot reach `hal-actuate` — with `canary-ridden` as the positive control.

**Surviving SR-SIM-5 either way.** Whether Rust or Python hosts the loop is open, recommended as
"Rust owns the loop". This design doesn't depend on that landing either way — **the invariant it
relies on is the seam and the log schema, not the host language.** The `hardware` backend is a
`BoardObserve` + `BoardActuate` implementation over SocketCAN, reached identically whether the
caller is a Rust `main` or Python via `control-ffi`; if Python keeps the loop, same flag name,
same backend selection, same MCAP output. Nothing in §2, §3, §4, §6 or §7 changes.

One thing **does** change with SR-SIM-5, and it is a safety matter — §7.1.

---

## 6. D6 — CAN transport, and a second transport that is not a footnote

CAN is de-risked and is not re-litigated here: Pi 5 / RP1 + `mcp251xfd`, config line
`dtoverlay=mcp251xfd,spi0-0,interrupt=25`, **not sharing an SPI bus with the IMU** (ICD §4.3;
the Pi 5 has three SPI controllers, so this is a wiring choice).

**But there is a live, unresolved risk against that path, and at 500 Hz it is severe.** A Pi 5
report under PREEMPT_RT (kernel 6.1.70-rt21) measured **SPI transaction times spiking to
1.5–2 ms under combined CPU and network load**, while scheduler wakeup latency stayed under
100–150 µs. Raising RT priority on the SPI and DMA interrupts did not fix it. No Raspberry Pi
engineer replied; no root cause was established; the reporter believed it to be RP1-specific,
having seen a CM4 be "rock steady".

**Our control period is 2 ms.** A 2 ms SPI stall is an entire missed cycle — and it attacks the
measurement itself, because a transport with unbounded tail latency does not merely degrade the
system, it **poisons the go/no-go number**.

**Why the report is suggestive but not dispositive**, so nobody over- or under-reacts:

1. It measured **userspace `spidev`** transactions of 2 KB. `mcp251xfd` is a *kernel* driver with
   a much smaller-transfer, GPIO-IRQ-driven path. If the cause is RP1/PCIe or DMA interrupt
   latency, the kernel path eats it identically; if it's `spidev` ioctl/worker scheduling, the
   kernel path may be much better. **The report can't distinguish these** — argues for
   measuring, not abandoning.
2. It was on **6.1.70-rt21**, the out-of-tree-RT era with immature RP1 support; we will be on
   6.12.x with mainlined PREEMPT_RT. May simply be fixed.
3. Single, unreplicated, no engineer response.

### Decision: SPI HAT primary, USB-CAN as a **second transport on day one**

Not as a contingency — as a **control variable in the experiment.**

Stage 0B's entire output is a latency number that gates the architecture. If the transport can
inject unbounded spikes, a bad number is **unattributable** — we cannot separate "our
architecture is too slow" from "our CAN HAT is bad", and the go/no-go is burned on an
uninterpretable result. A second independent transport is experimental design, not product risk
mitigation.

- The **CANable 2.0** is already on the Stage-0A bill of materials and will be in the CEO's
  hands before the Pi is. Its `gs_usb` driver is present in the RT kernel (verified by
  unpacking). Marginal cost: **zero hardware, zero purchase decision.**
- **Not a clean bypass — on a Pi 5, USB also hangs off RP1.** CANable gives a different
  driver/DMA path, not an RP1-independent one, and has its own tail character (`gs_usb` bulk
  transfers, ~1 ms host-controller quantisation) — a good comparator, questionable primary.
- **The only genuinely RP1-independent path is PCIe-attached CAN** via the Pi 5's external PCIe
  connector — a separate root complex from the one feeding RP1. Named here as the escalation
  tier with a cost. **Do not buy it yet.**

> **BoM, stated directly: the CANable 2.0 changes role, not the purchase sheet.** As AC-8's
> second measurement arm, it is now **architecturally required for Stage 0B, not an optional
> bench convenience** — Stage 0B's go/no-go number isn't valid without it. Its physical presence
> on the sheet doesn't change: already listed, zero marginal cost, already in hand before the Pi
> arrives. **Nothing else on the Stage-0A sheet is newly required, obsolete, or wrong.** PCIe-CAN
> stays a named, unbought escalation tier. Order tonight exactly what the sheet already lists.

### Cheapest possible sequencing, given nothing is purchased

**Buy the Pi 5 now. Run AC-9's SPI reproduction on the pinned RT kernel *before* buying the CAN
HAT** — zero incremental hardware, one evening, clears or kills the HAT before money moves.
**This information will never be cheaper than it is now.**

If both transports show tails beyond one control period, that is a genuine architecture finding
and it escalates — surfacing it is a success of the stage, not a failure. Numeric triggers:
AC-6, AC-8, AC-9.

---

## 7. ⚠️ Safety — an agent commanding a motor

The standing rules are a **hardware deadman in series with motor power** and **no AI in a
real-time or ridden loop**. This section states how they are enforced rather than documented.

### 7.1 The invariant that must be true before a motor turns

`crates/safety` and the `hal` call-sequence rules are real and on the call path in both Rust
binaries — `board-app-driverless` runs observe → compute → **clamp** → apply every cycle, and
`Params::default()` sets `max_current_a: 0.0`, so an unconfigured envelope has zero authority.

**Correction, verified against `crates/control-ffi/src/lib.rs`: `Envelope` is traversed today;
`CallSequence` is not.** An earlier draft welded these into one false compound. `control-ffi`
constructs an `Envelope` per controller and its per-cycle entry point calls
`envelope.apply(Command::MotorCurrent { amps: proposed }, Faults::NONE)`; the **clamped** value,
not the regulator's raw proposal, is what crosses the FFI boundary to Python. So when Python
drives `control-core` through `control-ffi`, Layer 2 (§7.2) is live on that path. **The exposure
is narrower than a previous version of this document implied.**

**What is genuinely absent is `CallSequence`** — the `hal` seam's `apply`/`wait_observe`
discipline; `control-ffi`'s own doc comment says so ("Not the `hal` seam... no
`wait_observe`/`apply` here"). Python decides when the next cycle happens, and nothing enforces
one-command-per-observation. That is a **sequencing** gap, not a **clamping** gap: without
`CallSequence`, a double-`apply()` is two ordinary, individually-clamped frames rather than a
detected protocol violation. That is real, and is what Gate S1 fixes.

**Gate S1 — non-negotiable, and the one Mechanical should attack hardest:** the `hardware`
backend must be reachable **only** from a binary whose actuation path traverses
`safety::Envelope` **and** enforces `CallSequence`, proven by `xtask gate`, not convention:
*no crate may depend on `hal-actuate` without also depending on `safety`*, with a canary proving
the rule fires. **If SR-SIM-5 resolves as "Python keeps the loop", `control-ffi` must not be
reachable from the `hardware` backend** — not because the clamp is missing there, but because
`CallSequence` is, and that backend is where an undetected double-apply matters most. Still the
strongest reason to prefer "Rust owns the loop," but on narrower, sequencing-only grounds than
previously stated.

### 7.2 The bounded envelope — four layers, only two of which survive our process dying

| # | Layer | Where enforced | Survives |
|---|---|---|---|
| 0 | **Deadman contactor**, in series with pack positive | physical, within arm's reach | everything |
| 1 | **Controller current ceiling** — Little FOCer motor/battery current + ERPM limits, set in its own configuration | firmware on a separate device | any Pi-side failure |
| 2 | **`safety::Envelope` clamp** — `max_current_a`, symmetric | in-process, every cycle | logic errors above it |
| 3 | **Controller command timeout** — output released if no command arrives within the timeout | firmware on a separate device | process death, hang |

Layers 0 and 1 are the ones that matter, because **they are the only two that keep working when
our process does not.** Layer 2 is defence in depth and Layer 3 depends on a firmware behaviour
this project has not yet verified (Q3, §12).

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

### 7.3 Abort path, and the three failure modes named in the brief

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

### 7.4 Who must be present

**The first powered run of any agent-authored code requires the CEO physically present, hand on
the deadman** — also the first run after any change to envelope parameters, the `hardware`
backend, or the kernel pin.

**Not required for:** unpowered bring-up, boot/device-tree verification, `cyclictest`, `vcan`
work, image builds, any dry run on `--backend sim`. Distinguishing these keeps the CEO's
attention available for the runs that need it.

No agent transmits a command to a motor without a human in the room. The control loop contains
no AI; §5's seam is what keeps it that way.

---

## 8. Numeric acceptance criteria (D0 ready-to-code gate)

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
| **AC-2** | **Kernel pin held.** The RT kernel is an explicit version, `apt-mark hold`-ed, never the floating metapackage | exact version recorded in the Release manifest; a floated kernel fails the build | §2 |
| **AC-3** | **Manifest diffability.** Two CI builds of the same git SHA produce diffable manifests and identical `config.txt` / `cmdline.txt` / overlay set | **0-line diff** on boot configuration; package manifest diff **reported, not required to be empty** (no snapshot mirror exists — §2) | manifest diff in CI |
| **AC-4** | **Provenance completeness.** Every field in §2's table present and non-empty | **100%** | schema check on `/etc/overboard-image.json` |
| **AC-5** | **RT scheduling jitter.** `cyclictest -m -Sp95 -i 2000 -D 30m` under `stress-ng` CPU + network load | p99.9 wakeup latency **≤ 150 µs**; max **≤ 500 µs** (25% of the 2 ms period) | `rt-tests` 2.6-1.1 |
| **AC-6** | **Command→actuation latency — the pre-registered go/no-go** | **Go** if p99.9 **≤ 1 ms** (half a period) *and* max **≤ 2 ms** (one period), over **≥ 10⁵ cycles** under representative load. Any max > 2 ms escalates as an architecture finding rather than being tuned away | MCAP log, offline analysis |
| **AC-7** | **Measurement resolution.** Timestamp resolution on the AC-6 measurement | **≤ 10 µs**, via `SO_TIMESTAMPING` on the CAN socket; hardware timestamps preferred, source recorded per run | harness self-report |
| **AC-8** | **Transport attribution.** AC-6 repeated over both `mcp251xfd` and USB-CAN in one session, same rig, same harness | both reported as p99 / p99.9 / max. A **>2×** difference in p99.9 attributes the tail to the transport rather than the architecture | §6 |
| **AC-9** | **SPI tail reproduction** (runs *before* the HAT is purchased). **Precondition: a >64 B `spidev` transfer must complete without a `-110` DMA timeout (§12 Q13) before the timed run counts** — a run that can't tell "our config is slow" from "this kernel's `spidev` is broken above 64 B" measures nothing | `spidev` loopback, `stress-ng`+`iperf3`, >64 B transfers: (1) confirm no `-110`; then (2) p99/p99.9/max. **Any `-110` blocks the HAT purchase and reopens Q13 as live on our hardware. Max > 2 ms once completing cleanly also blocks it**, per §6's escalation tier | §6, §12 Q13 |
| **AC-10** | **CAN integrity** over the AC-6 run | **0** dropped frames; **0** bus-off events | `ip -s -d link show` |
| **AC-11** | **Restore test** — the real acceptance gate for the image. Flash the *published* artefact to a blank card and boot it | pinned `uname -r` matches; `can0` up at the intended bitrate; AC-5 tail met. **10/10** consecutive cold boots, ready in **≤ 60 s** | first-boot log |
| **AC-12** | **No-Pi coverage.** Fraction of §9's "verifiable" column running in CI with no hardware | **100%**, green on every PR touching the image tree | CI |
| **AC-13** | **Safety gate S1.** `xtask gate` proves no crate reaches `hal-actuate` without `safety`, with a canary proving the rule fires | build fails otherwise | `cargo run -p xtask -- gate` |
| **AC-14** | **Envelope agreement.** Layer-1 controller limits and Layer-2 `max_current_a` read back and compared at arm time | mismatch → refuse to arm | `hardware` backend |

---

## 9. What is verifiable **without** a Pi — and what is not

No hardware has been purchased. This section is why that is not blocking.

### Verifiable in CI today, no Pi

| Thing | How | Confidence |
|---|---|---|
| **Package names and versions resolve** | `apt-get install --dry-run` with the pinned set, in a `debian:trixie` arm64 container with the Raspberry Pi archive added | **High** — catches a fabricated pin; §13 shows it already caught the `-2712-rt` assumption |
| **Kernel ships what we need** | unpack the `.deb`; assert `mcp251xfd.ko`, `mcp251xfd.dtbo`, `vcan.ko`, `bcm2712-rpi-5-b.dtb` and `CONFIG_PREEMPT_RT=y` | **High** — already done by hand (§13) |
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
- **All real timing**: AC-5 through AC-11, and the §6 SPI tail-latency question. **Every number
  Stage 0B exists to produce is in this list** — the definition of the stage, not a defect of it.
- **Thermals**, and whether sustained 500 Hz operation throttles.
- **Controller command-timeout behaviour** (Layer 3) — needs a Little FOCer to observe.
- **Real `vesc-wire` / `vesc-tx` byte layouts.** Deliberately honest stubs returning
  `NotYetImplemented` — fabricating VESC constants from memory into a crate that gates actuation
  was already judged the worst available artefact, and this design doesn't reverse it.

---

## 10. Credentials

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
3. **Upload credentials** (§11) go in a file at a documented path, mode `0600`, read at runtime,
   and **excluded from MCAP logs and from the provenance manifest by name**.
4. The repo ships a `credentials.example` with **placeholder values only**; CI fails if a file
   matching the real credential path is ever tracked.

A community-reported alternative — `custom.toml` on the boot partition, Bookworm+ — may be more
convenient, but was **not** confirmed against official docs today; verify before it's written
into a runbook.

---

## 11. Data path: capture on the Pi, upload, analyse offline

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

## 12. Open questions — named, not papered over

Each carries a default action so nobody is parked.

| # | Question | Default if unanswered |
|---|---|---|
| **Q1** | Is the 4K-page, no-`NO_HZ_FULL` packaged RT kernel good enough for a 2 ms loop on Pi 5? **Unknowable without hardware.** | Proceed with the packaged kernel; §4's ladder is the response if AC-5 fails |
| **Q2** | **Is the RP1 SPI tail-latency report real on ≥6.12 with the `mcp251xfd` kernel driver?** The largest risk here. | §6's dual-transport measurement answers it with our own data |
| **Q3** | Little FOCer command-timeout semantics and default (Layer 3). Unverified — no hardware, and no VESC constants are recorded in this repo. | Do not rely on Layer 3. Layers 0 and 1 carry the safety case until measured |
| **Q4** | Do `rpi-image-gen` / `pi-gen` officially support Pi 5? Neither README confirmed it. Is `debian:trixie` on an Ubuntu arm64 runner an acceptable host? | §3's half-day spike answers both before implementation starts; derive-from-stock is the fallback |
| **Q5** | **SR-SIM-5** — Rust or Python hosts the loop? Open elsewhere; §5 survives either. | Gate S1 (§7.1) binds regardless; this design supplies evidence for "Rust owns the loop" |
| **Q6** | Are the §7.2 provisional current limits right? Placeholders with no measured basis. | Re-ratify against Stage-0A §6c before the first powered run. **Blocking** |
| **Q7** | Which SPI controller for CAN, and which for the IMU (ICD §4.3)? A wiring decision. | Mechanical's call; record before the HAT is ordered |
| **Q8** | Does `interrupt=25` in the overlay conflict with anything else on the HAT stack? | Verify when the HAT is selected |
| **Q9** | Upload destination and retention. AWS is the convention; nothing is provisioned. | Local capture + manual upload for Stage 0B. Do not block on cloud plumbing |
| **Q10** | 6.12 or 6.18 kernel line? Both packaged with RT. | 6.12 (§4); revisit if a needed fix is 6.18-only |
| **Q11** | Does the packaged RT kernel carry a Pi-5-specific regression the 2712 flavour does not? v8 is the less-travelled path on this board. | Unknowable without hardware; AC-5 and AC-11 are the detectors |
| **Q12** | How does Pi 5 firmware select the v8-rt kernel, given it defaults to `kernel_2712.img`? **Operational, not architectural** — but it is the difference between a card that boots and one that does not. | Resolve at first boot; the restore test (AC-11) catches it |
| **Q13** | **A second, distinct RP1-SPI defect: `raspberrypi/linux` #6020 / #5696 — `spidev` transfers over 64 bytes failing outright with a DMA timeout (`-110`), not just slow.** Different from §6's tail-latency report. Researched below, not assumed. | AC-9's precondition (§8) checks it against the pinned kernel before the timed run counts |

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
verified. AC-9 is amended below to check both on our own hardware.

---

## 13. Verification log — how §4 was checked

Moved to its own document, split rather than trimmed — see
[`design-pi-image-stage0b-verification.md`](./design-pi-image-stage0b-verification.md).
