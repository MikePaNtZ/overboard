# Capture plan — Stage 0A assembly

<!--
covers:
  - docs/runbook-stage0a-bench.md
reconciled: 908f341
-->

**Assembly happens exactly once.** Everything else on the content roadmap can be redone;
this cannot. This plan is capture only — no editing, no publishing, no decisions about what
gets used. That is the marketing line's job, and keeping this to capture is what keeps each
step under 30 seconds instead of a production.

**One person, alone.** Every step below either needs no hands, or is propped/tripod first,
record second. If a step needs two hands and a camera, the workaround is stated.

**Failures are the most valuable material, not a thing to hide.** A part that doesn't fit, a
wrong connector, a motor that won't detect — capture it exactly like a success. This
project's strongest published artefact so far was a bug write-up.

**Keep the camera on the hardware.** No footage of a person's face/identity — that's a
separate, explicit decision, not a default here.

**Where footage goes:** a shared Google Drive folder (`Overboard – Raw Capture`, dated
subfolders), not this repo. Large binaries don't belong in git history, and Drive's phone app
auto-uploads on wifi with zero extra step at the bench. Digital Content Production pulls raw
footage from there for editing — this repo never sees it.

---

## The checklist

| Moment (runbook §) | Capture | How | Camera |
|---|---|---|---|
| Unboxing / first look (§1) | Stills | 5–6 quick phone photos: unopened boxes, then parts laid out | Handheld, before hands get busy |
| Kill-path wiring + the 3x switch test (§2) | Video, one continuous clip | Phone **propped against a box**, framed on the switch + controller LED, before you start the three test cycles | Propped — both hands are on the switch |
| Motor bolted to plate / bracket coming together (§3) | Stills | One photo each: bare plate, motor bolted, disc mounted | Handheld, between steps |
| First power-on (§5.1–5.3) | Video | Phone propped facing the bench, **recording before** you press precharge; frame the controller LED | Propped — hands on precharge + deadman |
| First `candump`/`ip link` trace (§5.4) | Screen recording or photo | `⌘+Shift+5` (or phone photo of the laptop screen) showing frames arriving | Handheld/laptop, 10s |
| **First spin** (§5.5) — the most valuable second in the project | Video | Phone propped/tripod on the disc+motor, **start recording before** running motor detection | Propped — cannot be redone if missed |
| First `kt` run (§6b) | Screen recording or phone video of the laptop plot | Start the recorder before commanding the current step, stop after the ramp settles | Propped or handheld, whichever is faster to set up |

**Any failure along the way** (wrong hall pinout, disc doesn't clear, connector mismatch,
detection fails): same rule as above — capture what's in front of you, don't reset first.

---

See `docs/runbook-stage0a-bench.md` for the procedure these moments are attached to; the
markers there point back here.
