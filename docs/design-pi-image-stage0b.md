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
reconciled: 7acdc2f
-->

- **Status:** Proposed — design only. No implementation is authorised by this document.
- **Owner:** Senior Controls · **Adversarial review:** Sr. Mechanical & Systems · **Approval:** COO
- **Closes (design half of):** [#32](https://github.com/MikePaNtZ/overboard/issues/32)
- **Implementation:** a separate issue, opened only after COO approval is recorded.
- **Size:** this doc runs close to ADR-0008's 40,000-char cap. **Prune when adding** — replace
  superseded reasoning rather than appending to it.

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

The controls-repo rule reads "controls only — Rust, sim, hardware, design docs". An OS image is
not obviously any of those, so this needs an argument rather than an assertion.

The argument is the **runtime contract**, not file adjacency.

The obvious case for one repo — "the image contains the binary, so their versions must move
together" — is refutable in one line: a well-built image *shouldn't* contain the binary. Deploy
the binary separately and that coupling evaporates. So that is not the reason.

The reason that survives is that the image and the control code share assumptions neither one
states alone: **kernel flavour and page size, `isolcpus` / cpuset layout, the RT priority
budget, CAN interface naming and bitrate, and systemd unit expectations.** The Rust code encodes
them; the image satisfies them; **nothing detects a mismatch except running the two together.**
A mismatch is not a build failure — it is a wrong latency number at the bench that nobody can
attribute. This repo's CI is the only place positioned to exercise the control code, the sim and
the image definition against a single commit. Splitting that contract across repos with no
shared CI is how it drifts silently.

Supporting: the `rust` and `sim` jobs already test the other half of the contract here on every
commit; and "controls only" was written to exclude marketing, not deployment — the same sentence
admits "hardware" and "design docs".

A third argument — that a new repo inherits no `CODEOWNERS` enforcement, since
`docs/decisions/ROLES.md` records turf as "unfalsifiable in two thirds of the estate" — is
**deliberately not leaned on**: it cuts both ways.

**Cost of the alternative (new repo):** the runtime contract loses its only end-to-end test; a
third ownership map and policy gate to maintain; a third repo for the CEO to track. **Cost of
the choice made:** this repo grows a non-Rust build tree and a slow CI job. Both accepted.

**The ownership handoff — and why it does not block.** `check_ownership()` fails on any new
top-level directory with no explicit `CODEOWNERS` rule (ADR-0002 calls this tax "the point"),
and `CODEOWNERS` is COO turf. To make this a copy-paste approval rather than a scheduling
dependency, the exact lines requested are `# role: Senior Controls` / `/pi/  @MikePaNtZ`.
**Fallback if that has not merged when implementation is ready:** land under `scripts/pi/`,
already Senior Controls turf, and move later. A top-priority deliverable should not be parked on
one line of another role's file.

---

## 2. D2 — Deliverable shape: **(a)**, an image, produced only by **(b)**

**Decision: the deliverable is (a) — a downloadable image with a SHA256 checksum and a
provenance manifest — and it is produced exclusively by (b), a scripted flow in CI. Never by
hand, and never by running a provisioning script against a live mirror on the CEO's card.**

The reason to prefer the image over a flash-time script is specific. A script that runs
`apt install` at flash time makes **the state of the network at the moment the CEO runs it**
part of the result — precisely the "local setup archaeology" he asked to eliminate, just
relocated. Two cards flashed a fortnight apart would differ, and the difference would surface as
a latency anomaly with no way to tell whether the plant or the platform changed. Building once,
in CI, from pinned inputs confines that nondeterminism to one logged place.

Calling this "(a)" rather than "both" is deliberate: **the artefact the CEO flashes is the
image; the artefact that is reviewed and rebuilt is the script tree.** If they disagree, the
script tree wins and the image is rebuilt.

### What "reproducible" honestly means here — and what it does not

**`archive.raspberrypi.com` has no snapshot service** — no equivalent of `snapshot.debian.org`.
A rebuild from the same commit next month will resolve different packages and produce a
different image, and no image-builder's reproducibility claim rescues us from that. Claiming
bit-for-bit determinism would be exactly the confident-and-wrong statement this document is
supposed to avoid. What is actually promised:

1. **Built only by CI, from a tagged commit. Never by hand.**
2. **The RT kernel package version is explicitly pinned and `apt-mark hold`-ed** — never the
   floating `linux-image-rpi-v8-rt` metapackage, which resolved to 6.18.34 today and will
   resolve to something else tomorrow. This is the pin that matters most: **the kernel is the
   single variable that invalidates every latency number Stage 0B produces.**
3. **A package manifest ships beside the image**, so any two builds diff reviewably even when
   they are not identical.
4. **A restore test is the real acceptance gate**, not a hash comparison: flash the published
   image to a blank card, boot it, and assert the pinned `uname -r`, `can0` up at the intended
   bitrate, and the cyclictest tail under threshold (AC-11).

GitHub Release assets are capped at 2 GB per file, so the artefact is published as **`.img.xz`**
— which is what Raspberry Pi Imager consumes anyway.

**Provenance manifest** — published beside the image *and* written into it at
`/etc/overboard-image.json`, so a running Pi can state its own identity. Fields: `git_sha`;
`built_at` + `workflow_run_url`; `base_image` (upstream release identifier + SHA256);
`packages[]` (**every** installed package as `name=version=arch`, from `dpkg-query`); `kernel`
(package name + exact version, §4); `config_txt_sha256`, `cmdline_txt_sha256` and `overlays[]`;
and `rust_binary_sha256` + `rust_toolchain` for the control binary that shipped in it.

Images are **never committed to git** (existing repo rule). They go to a GitHub Release — the
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

**Decision: `rpi-image-gen` primary.** It is the tool Raspberry Pi currently points at for this
job, it targets reproducibility explicitly, and it assembles from pinned packages instead of
rebuilding a distribution — which is the property §2 depends on.

The host constraint is the interesting part, and it has a clean answer that did not exist a
year ago: **GitHub `ubuntu-24.04-arm` hosted runners are generally available and free for public
repositories**, and this repo is public. So the build runs **native arm64, no QEMU**, in a
privileged `debian:trixie` container on that runner. That is close to the stated supported
configuration (Debian Trixie arm64, native) without being identical to it — the runner OS
underneath is Ubuntu.

The only unsupported axis is "in a container": the host is arm64, the container userspace is
literally `debian:trixie`, `--privileged` supplies `CAP_SYS_ADMIN`, and hosted runners are full
VMs with working loop devices. The risk is real but narrow.

**Decide it with a spike, not with argument.** Before anything is built, run **one** CI job that
builds `rpi-image-gen`'s own stock example configuration in exactly that host configuration and
produces any `.img` at all. Timebox **half a day**. Green → proceed; otherwise take the
fallback. That converts a schedule risk carried through implementation into an afternoon.

**Fallback: derive-from-stock. Explicitly not `pi-gen`.** `pi-gen` is a lateral move, not a
diversified one — same class of uncertainty (documented container path, 64-bit docs that never
mention Pi 5) plus rebuilding an entire distribution we do not need. Instead:

> Take the official **Raspberry Pi OS Lite arm64** release image, loop-mount, `chroot` (native
> arm64, no QEMU), `apt install` the pinned RT kernel, apply the `config.txt` / overlay /
> CPU-isolation / systemd deltas, repack, compress, hash.

Small delta from stock, no support caveat, fast, identical deliverable shape to §2. Its weakness
is that it is our own bash rather than a declarative configuration — hence fallback, not primary.

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

- **Page size (4K vs 16K): largely irrelevant here, possibly helpful.** 16K pages buy throughput
  via reduced TLB pressure. A 500 Hz loop with a small working set is not throughput-bound, and
  smaller pages can *improve* determinism by shortening worst-case memory-management work.
- **`CONFIG_NO_HZ_FULL` unset: the real cost, and it is a floor, not a knob.**
  `CONFIG_CPU_ISOLATION=y` is set, so `isolcpus=` and cpusets work — but **full tickless
  operation on the control core is unavailable**; the isolated core still takes the 250 Hz tick.
  Practically a few microseconds of periodic perturbation, and probably fine. What matters is
  that it is a **known perturbation floor stated in advance and included in the measurement**,
  not something discovered while debugging an anomaly at the bench.

**Consequently, no claim here that the platform is real-time rests on a config flag.**
`CONFIG_PREEMPT_RT=y` says the kernel was *built* RT; it says nothing about achieved latency on
this board under our load. That claim is empirical and lives in AC-5.

**Two things that could have quietly invalidated this design, both checked.**

1. **RP1 support in the generic v8 flavour.** Shipping `bcm2712-rpi-5-b.dtb` is necessary but not
   sufficient — a Pi 5 without RP1 has no usable SPI, GPIO or Ethernet, and the answer would
   have been "Pi 4 / CM4", rewriting this document. The RP1 stack is **identical in both
   flavours**: `CONFIG_MFD_RP1=y`, `CONFIG_PINCTRL_RP1=y`, `CONFIG_COMMON_CLK_RP1=y`,
   `CONFIG_PCIE_BRCMSTB=y`, `CONFIG_PWM_RP1=y`, `CONFIG_RP1_PIO=m`, `CONFIG_MACB=y`. **The
   v8-rt kernel is a complete Pi 5 kernel.**
2. **The CAN path survives the flavour switch:** `mcp251xfd.dtbo`, `mcp251xfd.ko`, `vcan.ko`,
   `CONFIG_SPI_DESIGNWARE=m` (the RP1 SPI driver) and the full SocketCAN module set all present.

**Operational unknown, not a design one:** Pi 5 firmware defaults to `kernel_2712.img`. How the
v8-rt kernel gets selected — an explicit `config.txt` `kernel=` line, the packaging's own
handling, or something else — is Q12.

**Pinning.** Pin a specific 6.12-line version, never the floating metapackage (which resolved to
6.18.34 today). 6.12 over 6.18 because it is the longer-supported branch and comfortably
contains the `mcp251xfd` receive-latency fix that made the CAN path credible (upstream
`eb9a839`, backported November 2024, marked for stable). `linux-image-6.12.75+rpt-rpi-v8-rt`
`1:6.12.75-1+rpt1` is the current candidate; the patch version is set at implementation time and
**≥ 6.12 is the hard floor**, not 6.12.75 specifically.

**Fallback ladder if RT latency is inadequate**, cheapest first:

1. Tune what the packaged kernel allows: `isolcpus`, IRQ affinity, `SCHED_FIFO` priorities,
   `performance` governor.
2. Accept a lower loop rate. 500 Hz is an ICD number, not a law of physics; the honest response
   to a platform that cannot hold 2 ms is to say so and re-derive the rate.
3. Build our own RT kernel — 2712 flavour with `CONFIG_PREEMPT_RT=y`, `CONFIG_NO_HZ_FULL=y`,
   16K pages. **This is the "community build" cost the brief asked about, and it is now a
   fallback rather than the baseline:** we would own a kernel build, its patch rebases and its
   module signing.
4. Move the 500 Hz inner loop off Linux entirely. An architecture change; escalated, not decided
   here.

**`rpi-update` is explicitly rejected** — it fetches unversioned bleeding-edge kernels. Right
tool for testing, wrong tool for anything reproduced from a manifest.

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

**Surviving SR-SIM-5 either way.** Whether Rust or Python hosts the loop is open, and
recommended as "Rust owns the loop". This design does not depend on that landing either way,
because **the invariant it relies on is the seam and the log schema, not the host language.**
The `hardware` backend is a `BoardObserve` + `BoardActuate` implementation over SocketCAN,
reached identically whether the caller is a Rust `main` or Python via `control-ffi`. If Python
keeps the loop, the launcher invokes the Python scenario with the same flag name, the same
backend selection and the same MCAP output. Nothing in §2, §3, §4, §6 or §7 changes.

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
   a GPIO-IRQ → threaded-handler → FIFO-drain path and much smaller transfers. If the cause is
   RP1/PCIe completion or DMA interrupt latency, the kernel path eats it identically; if it is
   `spidev` ioctl synchronisation plus worker scheduling, the kernel path may be much better.
   **The report cannot distinguish these** — which argues for measuring, not abandoning.
2. It was on **6.1.70-rt21**, the out-of-tree-RT era with immature RP1 support. We will be on
   6.12.x with mainlined PREEMPT_RT. It may simply be fixed.
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
- **It is not a clean bypass, and the doc should not imply otherwise: on a Pi 5, USB also hangs
  off RP1.** CANable gives a different driver and DMA path, not an RP1-independent one. It also
  has its own tail character (`gs_usb` bulk transfers, roughly 1 ms host-controller
  quantisation), which makes it a good comparator and a questionable production primary.
- **The only genuinely RP1-independent path is PCIe-attached CAN** via the Pi 5's external PCIe
  connector — a separate root complex from the one feeding RP1. Named here as the escalation
  tier with a cost. **Do not buy it yet.**

### Cheapest possible sequencing, given nothing is purchased

**Buy the Pi 5 now. Run the SPI tail-latency reproduction on the pinned RT kernel *before*
buying the CAN HAT** — a `spidev` loopback under combined `stress-ng` + `iperf3` load, reporting
transaction-time p99 / p99.9 / max, reproduces the reported conditions directly. Zero
incremental hardware, one evening, and it clears or kills the HAT before money moves. **This
information will never be cheaper than it is now.**

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

**The gap is the live path, not the crate.** Today the sim loop is hosted in Python, calling the
control law through `control-ffi` — which reaches `control-core` directly and therefore **does
not traverse `safety::Envelope` or `CallSequence` at all.** The invariants are not bypassed in
the Rust binaries; they are bypassed by the host that is actually running.

**Gate S1 — non-negotiable, and the one Mechanical should attack hardest:** the `hardware`
backend must be reachable **only** from a binary whose actuation path traverses
`safety::Envelope`, and this must be proven by `xtask gate`, not by convention. Concretely, add
to the gate: *no crate may depend on `hal-actuate` without also depending on `safety`*, with a
canary proving the rule fires. **If SR-SIM-5 resolves as "Python keeps the loop", then
`control-ffi` must not be reachable from the `hardware` backend**, or the clamp is bypassed on
the exact run where it matters. This is the single strongest reason to prefer "Rust owns the
loop", and it is offered as evidence to that decision rather than as a pre-emption of it.

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

- **Deliberate abort:** open the deadman. Always available, needs no software, and is the only
  abort anyone is asked to remember under stress.
- **Comms loss** (CAN silent, cable pulled): Layer 3 releases output within the timeout. Pi side
  disarms on the first `wait_observe()` error.
- **Process death** (`SIGKILL`, panic, OOM): **destructors do not reliably run**, so the RAII
  `Disarm` guard cannot be trusted here. This is the architectural point of the whole section —
  **the abort that covers process death must live in a device that keeps running when our
  process dies.** Layers 1 and 3 do; Layer 2 does not.
- **Hung loop, still sending a stale non-zero command** — the nastiest case, because Layer 3's
  timeout never fires. Only Layers 0 and 1 cover it. That is the honest answer, and it is why
  the current ceiling is set low enough that a stuck-on command at the ceiling is a nuisance
  rather than an injury.

### 7.4 Who must be present

**The first powered run of any agent-authored code requires the CEO physically present with a
hand on the deadman.** Also the first run after any change to the envelope parameters, the
`hardware` backend, or the kernel pin.

**Not required for:** unpowered bring-up, boot and device-tree verification, `cyclictest`,
`vcan` work, image builds, and any dry run on `--backend sim`. Distinguishing these is the
point — it keeps the CEO's attention available for the runs that need it.

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
| **AC-9** | **SPI tail reproduction** (runs *before* the CAN HAT is purchased) | `spidev` loopback under `stress-ng` + `iperf3`: report transaction-time p99 / p99.9 / max. **Max > 2 ms blocks the HAT purchase** pending §6's escalation tier | §6 |
| **AC-10** | **CAN integrity** over the AC-6 run | **0** dropped frames; **0** bus-off events | `ip -s -d link show` |
| **AC-11** | **Restore test** — the real acceptance gate for the image. Flash the *published* artefact to a blank card and boot it | pinned `uname -r` matches; `can0` up at the intended bitrate; AC-5 tail met. **10/10** consecutive cold boots, ready in **≤ 60 s** | first-boot log |
| **AC-12** | **No-Pi coverage.** Fraction of §9's "verifiable" column running in CI with no hardware | **100%**, green on every PR touching the image tree | CI |
| **AC-13** | **Safety gate S1.** `xtask gate` proves no crate reaches `hal-actuate` without `safety`, with a canary proving the rule fires | build fails otherwise | `cargo run -p xtask -- gate` |
| **AC-14** | **Envelope agreement.** Layer-1 controller limits and Layer-2 `max_current_a` read back and compared at arm time | mismatch → refuse to arm | `hardware` backend |

---

## 9. What is verifiable **without** a Pi — and what is not

No hardware has been purchased. This section is the reason that is not blocking.

### Verifiable in CI today, no Pi

| Thing | How | Confidence |
|---|---|---|
| **Package names and versions resolve** | `apt-get install --dry-run` with the pinned set, in a `debian:trixie` arm64 container with the Raspberry Pi archive added | **High** — the check that catches a fabricated pin; §13 shows it already caught the `-2712-rt` assumption |
| **Kernel ships what we need** | unpack the `.deb`; assert `mcp251xfd.ko`, `mcp251xfd.dtbo`, `vcan.ko`, `bcm2712-rpi-5-b.dtb` and `CONFIG_PREEMPT_RT=y` | **High** — already done by hand (§13) |
| **Overlay compiles** | `dtc` on the overlay source | **High** — already established |
| **The whole CAN stack, end to end** | `vcan0` + a **simulated VESC responder**; the `hardware` backend talks to `vcan0` exactly as it would to `can0`. Exercises framing, socket setup, timeouts, error paths, MCAP capture, arm/disarm | **High for logic, zero for timing** |
| **Bench harness logic** | `--backend sim` against `bench_spinup.py`; `canplayer` replay of recorded frames | **High** |
| **`cyclictest` harness** | builds and self-tests on any Linux; on a non-RT kernel it produces *bad* numbers — still enough to prove parsing, thresholds, capture and upload work | **High for the harness, not the numbers** |
| **Image builds at all** | full builder run on the arm64 runner; artefact produced and hashed | **High** |
| **Safety gate S1** | `xtask gate` — pure `cargo metadata` | **High** |

### Requires the hardware. No substitute exists.

- **That the image boots**, and that the 4K-page v8-rt kernel boots on *this* Pi 5.
- **Device-tree load at runtime** — that `can0` appears, at the intended bitrate, on the intended
  SPI controller, not sharing with the IMU.
- **All real timing**: AC-5 through AC-11, and the §6 SPI tail-latency question. **Every number
  Stage 0B exists to produce is in this list** — not a defect of the plan, the definition of the
  stage.
- **Thermals**, and whether sustained 500 Hz operation throttles.
- **Controller command-timeout behaviour** (Layer 3) — needs a Little FOCer to observe.
- **Real `vesc-wire` / `vesc-tx` byte layouts.** Deliberately honest stubs returning
  `NotYetImplemented`; fabricating VESC protocol constants from memory into a crate that gates
  actuation was already judged the worst available artefact, and this design does not reverse it.

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
   CEO copies his own public key to the card; no agent ever generates a keypair. Plaintext
   credentials on a FAT boot partition are the obvious free shot a reviewer will take, and
   key-only auth closes it.
3. **Upload credentials** (§11) go in a file at a documented path, mode `0600`, read at runtime,
   and **excluded from MCAP logs and from the provenance manifest by name**.
4. The repo ships a `credentials.example` with **placeholder values only**; CI fails if a file
   matching the real credential path is ever tracked.

A community-reported alternative — `custom.toml` on the boot partition, Bookworm and later — may
be more convenient, but was **not** confirmed against official documentation today. It must be
verified before it is written into a runbook.

---

## 11. Data path: capture on the Pi, upload, analyse offline

One schema, and it already has a home in the code: **MCAP**, with `board_types::RunMetadata` as
the header payload (ICD §6.2 — the doc comments in `crates/hal/src/lib.rs` and
`crates/board-types/src/lib.rs` already say so). Foxglove for viewing, per the project
convention. No MCAP writer exists yet; writing one is implementation work, not design.

```
Pi: run  →  MCAP to local disk  →  upload  ═══▶  upload completes = analysis trigger
                                                        │
                                          offline: Senior Controls, against
                                          sim predictions (bench_spinup.py
                                          already has a `replay` mode that
                                          compares sim to a measured CSV)
```

Three rules, in priority order:

1. **Capture never blocks the loop.** The writer is on a non-real-time thread behind a bounded
   queue. **A full queue drops samples and increments a counter that is logged** — it never
   applies back-pressure to a 2 ms loop. A dropped-sample count is a recoverable measurement
   defect; a blocked control loop is a safety event.
2. **No analysis on the Pi.** The Pi captures and uploads. Nothing else. This mirrors runbook
   §8: analysis is not done at the bench, and not by the person holding the deadman.
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

---

## 13. Verification log — how §4 was checked

Recorded because the acceptance criteria forbid fabrication, and because this is exactly what
the implementation must automate (AC-1). On 2026-07-27, against
`archive.raspberrypi.com/debian trixie main binary-arm64`:

1. Enumerated every `linux-image-*` in the `Packages` index. **This is what established that
   `rpi-v8-rt` exists and `rpi-2712-rt` does not** — otherwise a plausible, wrong pin.
2. Downloaded `linux-image-6.12.75+rpt-rpi-v8-rt_6.12.75-1+rpt1_arm64.deb`; SHA256 matched the
   index (`649fc78a…`). Unpacked: `mcp251xfd.ko`, `mcp251xfd.dtbo`, `vcan.ko`,
   `bcm2712-rpi-5-b.dtb` and the SocketCAN module set all present.
3. Its shipped `/boot/config-…`: `CONFIG_PREEMPT_RT=y`, `CONFIG_ARM64_4K_PAGES=y`,
   `CONFIG_HZ=250`, `CONFIG_CPU_ISOLATION=y`, `CONFIG_CAN_MCP251XFD=m`, `CONFIG_CAN_VCAN=m`,
   `CONFIG_SPI_DESIGNWARE=m`, **no `CONFIG_NO_HZ_FULL`**.
4. Same for `…-rpi-2712`: `CONFIG_ARM64_16K_PAGES=y`, `CONFIG_PREEMPT_BUILD=y`, no
   `CONFIG_PREEMPT_RT` — the §4 table. RP1 stack diffed between the two: **identical**.
5. `rt-tests` 2.6-1.1 and `can-utils` 2023.03-1+b2 in Debian trixie for arm64; `rtla` in the
   Raspberry Pi archive.
6. `raspberrypi/linux` PR #6466 backports upstream `eb9a839` (the `mcp251xfd` coalescing fix
   behind the reported receive latency), merged 2024-11-14 to `rpi-6.6.y`, marked for stable —
   hence in the 6.12 line.

**Not verified, and flagged rather than assumed:** §12.
