# 2026-08-02 — Stage-0B: the loop profiler, the flash path, and the image build

Issue [#182](https://github.com/MikePaNtZ/overboard/issues/182). Triggered by the CEO asking
three questions the day before the Pi arrives: is the image built, how do I flash it, and can I
profile the loop.

**The honest answer to the first was no.** The design ([#32](https://github.com/MikePaNtZ/overboard/issues/32),
PR [#51](https://github.com/MikePaNtZ/overboard/pull/51)) was approved on 2026-07-27 and the
implementation issue was *never opened*, so nothing existed: no `pi/`, no image workflow, no
release, and the D3 spike unrun. That gap is the finding of the session; everything below is
the response to it.

## Landed

| | |
|---|---|
| [#181](https://github.com/MikePaNtZ/overboard/pull/181) | `crates/loop-profiler` — AC-5's instrument |
| [#184](https://github.com/MikePaNtZ/overboard/pull/184) | `scripts/pi/` — spike, AC-1/AC-2 verification, flash + credential path, secret guard |
| [#197](https://github.com/MikePaNtZ/overboard/pull/197) | I1 — the real image tree. **Open, does not build yet.** |

## The number that reframes Stage 0B

`overboard-loop-profile` on first run: **control-law compute p99.9 = 0.7 µs against a 2000 µs
period.** ~0.03% of budget.

Loop-rate viability is therefore *entirely* a scheduler question. If the Pi cannot hold 2 ms it
will not be because of our arithmetic, and time spent optimising the control law is time wasted.
This should be stated before anyone starts tuning.

The profiler also **withholds its verdict** rather than failing when the platform cannot support
one (`ac5_verdict()` returns `None` off PREEMPT_RT, not `Some(false)`) — "could not measure" and
"platform failed" are different findings and only one is about the Pi.

## Findings worth carrying

**1. Debian Trixie will not verify `archive.raspberrypi.com`.** Trixie's Sequoia `sqv` rejects
SHA-1 from 2026-02-01; the Pi archive key carries a SHA-1 self-certification. This blocks *every*
Trixie-hosted build against that archive — the exact shape design §3 chose.
Handled in `scripts/pi/add_rpi_archive.sh` by relaxing the **hash policy only**; signature
verification stays enforced. `[trusted=yes]` was explicitly rejected: it turns verification off
entirely, and this is a kernel that will command a motor. Applied only when the default policy
actually refuses, so it retires itself when the key is re-signed.

**2. I0 is GREEN.** `rpi-image-gen` builds in a privileged `debian:trixie` container on
`ubuntu-24.04-arm`, native arm64, no QEMU (`build exit: 0`, 277 MB `.img`). The "containers not
formally supported" risk is answered in the affirmative. **Derive-from-stock is retired** as the
active plan.

**3. The stock `rpi5` device layer is unusable, and quietly so.** It requires `rpi-linux-2712` —
the 16K-page flavour with **no RT build**. Using it would put a non-RT kernel on a card whose
purpose is measuring real-time latency, and **the image would build green while being wrong**.
`scripts/pi/image/device/overboard-pi5/device.yaml` is the stock layer with the kernel
requirement swapped, and is the one place design §4's tradeoff becomes a file.

**4. `kernel=kernel8.img` is the Q12 attempt, marked UNVERIFIED in `config.txt`.** Pi 5 firmware
defaults to `kernel_2712.img`; the RT kernel ships as `kernel8.img`. If the board does not boot
tomorrow, **that line is the first thing to change.**

**5. The artefact format does not match the design.** `rpi-image-gen` deploys `.img.zst`; §2
specifies `.img.xz` and `flash_pi.sh` expects `.img.xz`. One of the three must move. Publishing
is deliberately **switched off** until it is settled — the build uploads a CI artefact instead.

## Where I1 stopped

Four build rounds, each failing further in:
provider conflict → DEB822 header → `apt-mark` absent → **`apt` absent**.

Current failure is one line, in the *stock* `image-rpios` layer's `customize05-pkgs`:

```
chroot: failed to run command 'apt': No such file or directory
```

Our base is slim and apt is not installed in the target.
**Fix: add `apt` to `packages:` in `scripts/pi/image/layer/overboard-base.yaml`.**
High confidence, untested — the next session should start here.

## Not done, and needed

- **I5 — the controller pipeline.** *No Overboard code ships in the image.* Not the controller,
  not the profiler, not the repo. `flash_pi.sh` was printing `sudo overboard-loop-profile …` as
  a next step, which would have sent someone debugging a working card; corrected to name the
  manual clone-and-build path and point at I5. Until I5 exists, getting code onto the Pi is
  `git clone` + `cargo build --release -p loop-profiler` on the board (~10 min, few deps).
- **I7 — sim-in-the-loop self-test.** Needs the workspace plus MuJoCo on arm64. Note: do **not**
  step MuJoCo inside the RT thread's deadline — physics on a non-isolated core, controller on
  the isolated one. Mixing them makes a scheduler problem indistinguishable from a physics-step
  problem, which is the ambiguity Stage 0B exists to avoid.

## ⚠️ Untested safety catch — needs a human

`flash_pi.sh` refuses non-removable disks. This session's sandbox **blocks `diskutil`**, so only
the fail-closed path was exercised (unreadable device → refuse). The discrimination logic —
correctly identifying a real internal disk and refusing it — has never run.
**Run `--dry-run` against a real SD card before trusting it with a real one.**

## Process notes

- **Three PRs ran zero CI for hours.** GitHub schedules no `pull_request` runs when it cannot
  compute a merge commit, so a `CONFLICTING` PR looks normal and simply has no checks. Bit us
  three ways: a stale base branch, a squash-merge making a different commit than the branch
  carried, and a stale async status. **Check `mergeStateStatus`, not just the checks list.**
- **The primary working directory is not safe to assume.** Mid-session
  `/Users/mike/projects/overboard` was switched to another role's branch with their uncommitted
  work. This teardown was done from a throwaway worktree. ADR-0006 exists for this reason.
