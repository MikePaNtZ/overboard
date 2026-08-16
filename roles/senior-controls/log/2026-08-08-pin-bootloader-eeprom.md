# Issue #222 — pin the bootloader EEPROM version in pins.env (PR TBD)

Added `BOOTLOADER_VERSION`, `BOOTLOADER_TIMESTAMP`, `BOOTLOADER_CHANNEL` to
`scripts/pi/pins.env`, "held, not tracked" like the kernel pin, using the
values the issue's own comment thread already measured on the CEO's Pi 5
Rev 1.1 (`086b83e3332dfc8927c56762771d082f3077a1ae`, built 2026-05-26,
channel `default`). The channel is pinned alongside the hash, not the hash
alone — `rpi-eeprom-update` resolves "latest" differently per channel, so a
hash without its channel does not reproduce a state (the issue's own
revised proposal, made after seeing the hardware data). Not consumed by
`check_image_pin.py` or `verify_pins.sh`, deliberately: there is nothing on
the card to check the bootloader against, unlike the kernel.

**Deliberately left out:**

- **Step 2 of the issue's proposal** ("have the Stage-0B run record
  `vcgencmd bootloader_version` into the run log, alongside `git_sha`").
  The obvious home for that is `crates/loop-profiler`'s JSON schema
  (`report.rs`/`profile.rs`) — but PR #234 (issue #226) is an open PR
  actively bumping that exact schema to v2 right now. Landing a competing
  schema edit to the same files in parallel would conflict with in-flight
  work for no benefit; this is a separate, smaller increment once #234
  merges.
- **Item 3 of the issue** (whether AC-11's restore test should assert the
  bootloader pin). The issue itself frames this as "worth a considered call
  rather than a default" — a Promise-class call (it would change a public
  acceptance criterion for a hobbyist-facing image), not mine to decide
  unilaterally.
- `docs/runbook-pi-first-boot.md` §4 still reads "Follow-up, not done here:
  add the bootloader version to `pins.env`..." — now stale now that the pin
  exists. `docs/` is COO turf; flagged rather than edited.

Addresses #222 (step 1 of its proposal); left open for step 2 and item 3 as
noted above, so the issue stays open rather than auto-closing.
