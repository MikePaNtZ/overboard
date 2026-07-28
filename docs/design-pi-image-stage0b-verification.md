# Verification log — Stage-0B Pi image, how §4 was checked

<!--
covers:
  - crates/xtask/src/main.rs
reconciled: 1458f61
-->

- **Parent design:** [`design-pi-image-stage0b.md`](./design-pi-image-stage0b.md) §4 (kernel).
  Split out under ADR-0008's size cap — this is a record of *how* §4's claims were verified, a
  different kind of artefact from the design itself, not a trim of it.
- **Status:** informational — nothing here overrides an acceptance criterion in the parent doc.

Recorded because the acceptance criteria forbid fabrication, and because this is exactly what
the implementation must automate (AC-1, parent doc §8). On 2026-07-27, against
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
   `CONFIG_PREEMPT_RT` — the parent doc's §4 table. RP1 stack diffed between the two:
   **identical**.
5. `rt-tests` 2.6-1.1 and `can-utils` 2023.03-1+b2 in Debian trixie for arm64; `rtla` in the
   Raspberry Pi archive.
6. `raspberrypi/linux` PR #6466 backports upstream `eb9a839` (the `mcp251xfd` coalescing fix
   behind the reported receive latency), merged 2024-11-14 to `rpi-6.6.y`, marked for stable —
   hence in the 6.12 line.

**Not verified, and flagged rather than assumed:** parent doc §12 (open questions), including
Q13's RP1-SPI DMA-timeout research, added the same pass as this split.
