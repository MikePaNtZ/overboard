# Design — Stage-0B Pi image, provisioning, and bench tooling

<!--
covers:
  - crates/hal/src/lib.rs
  - crates/hal-actuate/src/lib.rs
  - crates/safety/src/lib.rs
  - crates/board-app-driverless/src/main.rs
  - crates/xtask/src/main.rs
reconciled: 25012f4
-->

- **Status:** Proposed — design only. No implementation is authorised by this document.
- **Owner:** Senior Controls · **Adversarial review:** Sr. Mechanical & Systems · **Approval:** COO
- **Closes (design half of):** [#32](https://github.com/MikePaNtZ/overboard/issues/32)
- **Implementation:** a separate issue, opened only after COO approval is recorded.
- **Size:** operational detail (schemas, pins, thresholds, open questions) lives in two
  companion docs — see §8. Split per [issue #54](https://github.com/MikePaNtZ/overboard/issues/54),
  not trimmed: nothing that was here is gone, it moved. **Prune when adding** — replace
  superseded reasoning rather than appending.

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

**The ownership handoff does not block.** `check_ownership()` fails on any new top-level
directory with no explicit `CODEOWNERS` rule (ADR-0002 calls this tax "the point"), and
`CODEOWNERS` is COO turf — copy-paste approval text and the non-blocking `scripts/pi/` fallback
are in the reference doc §1.

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

Published as **`.img.xz`** to a GitHub Release — never committed to git (existing repo rule),
same mechanism `publish-sim-artifact` already uses for sim renders. Provenance manifest field
schema is in the reference doc §1.

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
one (same class of uncertainty, plus rebuilding a whole distribution we don't need). Step-by-step
detail is in the reference doc §2; weakness is that it's our own bash, not declarative
configuration, hence fallback not primary.

**Open:** neither tool's Pi 5 support was confirmable from its README (`pi-gen`'s 64-bit
documentation names Pi Zero 2 / 3 / 4 only). Almost certainly documentation lag, but unverified
— recorded in the reference doc §11.

---

## 4. D4 — Kernel: PREEMPT_RT on Pi 5, and what it actually costs

This section is the one most likely to contain a plausible-but-wrong pin, so every claim below
was checked against the archive directly rather than recalled. Method and evidence are in
[`design-pi-image-stage0b-verification.md`](./design-pi-image-stage0b-verification.md); the
config-diff table and the two invalidation checks that survived are in the reference doc §3.

**The good news is better than expected.** PREEMPT_RT on a Pi is no longer "effectively a
community build". Raspberry Pi **officially package an RT kernel**: `linux-image-rpi-v8-rt`,
maintained by a raspberrypi.com address, in `archive.raspberrypi.com`, with `linux-headers-` and
`linux-base-` counterparts, across both the 6.12 and 6.18 lines.

**The bad news is specific, and it is the real cost.** There is **no RT build of the Pi 5's own
kernel flavour.** The archive carries `rpi-2712` (the Pi 5 flavour) and `rpi-v8-rt` (the RT
flavour) — and no `rpi-2712-rt`. So **choosing RT on a Pi 5 means dropping from the 16K-page,
2712-tuned kernel to the generic 4K-page v8 kernel.** Two consequences, weighted rather than
both filed as "loss":

- **Page size (4K vs 16K): largely irrelevant here, possibly helpful.** A 500 Hz loop with a
  small working set is not throughput-bound, and smaller pages can *improve* determinism.
- **`CONFIG_NO_HZ_FULL` unset: the real cost, and a floor, not a knob.**
  `CONFIG_CPU_ISOLATION=y` is set, so `isolcpus=`/cpusets work, but the isolated core still takes
  the 250 Hz tick — a few microseconds of periodic perturbation, probably fine, and **stated in
  advance and included in the measurement** rather than discovered mid-debug.

**Consequently, no claim here that the platform is real-time rests on a config flag** —
`CONFIG_PREEMPT_RT=y` says the kernel was *built* RT, nothing about achieved latency under our
load. That claim is empirical and lives in AC-5.

**Operational unknown, not a design one:** how the v8-rt kernel gets selected at boot, given Pi 5
firmware defaults to `kernel_2712.img` — Q12 in the reference doc §11.

**Pinning.** Pin a specific 6.12-line version, never the floating metapackage (which resolved to
6.18.34 today). 6.12 over 6.18: longer-supported, and comfortably contains the `mcp251xfd`
receive-latency fix (upstream `eb9a839`, backported November 2024, marked for stable). Exact
candidate version is in the reference doc §3; **≥ 6.12 is the hard floor**, not a specific patch
release.

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
The seam's existing shape — `hal::BoardObserve`, `hal_actuate::BoardActuate`, and the `xtask
gate` proof that `board-app-ridden` cannot reach `hal-actuate` — is detailed in the reference
doc §4; PR #49 landed the part that matters.

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

**But there is a live, unresolved risk against that path.** A community-reported Pi 5
SPI-tail-latency spike under PREEMPT_RT is the largest open risk here — the report itself, and
why it's suggestive but not dispositive, are in the reference doc §5. **Severity, not at 500 Hz
but against the plant's actual delay budget:** [`design-delay-budget-stage0b.md`](./design-delay-budget-stage0b.md)
measures the ridden closed loop's real ceiling at 38–39 ms against this spike's reported 1.5–2 ms
— comfortable, not disqualifying (issue #113).

**Our control period is 2 ms.** A 2 ms SPI stall is an entire missed cycle — and it attacks the
measurement itself, because a transport with unbounded tail latency does not merely degrade the
system, it **poisons the go/no-go number**.

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

Cheapest sequencing, given nothing is purchased yet, is in the reference doc §5. If both
transports show tails beyond one control period, that is a genuine architecture finding and it
escalates — surfacing it is a success of the stage, not a failure. Numeric triggers: AC-6, AC-8,
AC-9.

---

## 7. ⚠️ Safety — an agent commanding a motor

The standing rules are a **hardware deadman in series with motor power** and **no AI in a
real-time or ridden loop**. This section states how they are enforced rather than documented.
The bounded-envelope layer table, the provisional numbers, the abort-path mapping, and who must
be physically present are in the reference doc §6 — what follows is the invariant itself.

### 7.1 The invariant that must be true before a motor turns

`crates/safety` and the `hal` call-sequence rules are real and on the call path in both Rust
binaries — `board-app-driverless` runs observe → compute → **clamp** → apply every cycle, and
`Params::default()` sets `max_current_a: 0.0`, so an unconfigured envelope has zero authority.

**Correction, verified against `crates/control-ffi/src/lib.rs`: `Envelope` is traversed today;
`CallSequence` is not.** An earlier draft welded these into one false compound. `control-ffi`
constructs an `Envelope` per controller and its per-cycle entry point calls
`envelope.apply(Command::MotorCurrent { amps: proposed }, Faults::NONE)`; the **clamped** value,
not the regulator's raw proposal, is what crosses the FFI boundary to Python. So when Python
drives `control-core` through `control-ffi`, Layer 2 (the bounded-envelope clamp — reference doc
§6) is live on that path. **The exposure is narrower than a previous version of this document
implied.**

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

---

## 8. See also

- [`design-pi-image-stage0b-reference.md`](./design-pi-image-stage0b-reference.md) — provenance
  schema, exact version pins, bench-tooling interface, CAN supporting evidence, the safety
  envelope/abort tables, the numeric acceptance criteria, the no-Pi verifiability table,
  credentials, the data path, and the open-question ledger.
- [`design-pi-image-stage0b-verification.md`](./design-pi-image-stage0b-verification.md) — how
  §4's kernel claims were checked against the archive, method and evidence.
- [`design-delay-budget-stage0b.md`](./design-delay-budget-stage0b.md) — the plant-derived delay
  budget behind the amended AC-6 (issue #113): the ridden closed loop's measured 38–39 ms
  ceiling, and what the SPI tail and the estimator each cost against it.
