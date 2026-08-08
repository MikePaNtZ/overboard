# Issue #255 — pin AC-5's cyclictest to the isolated core (PR TBD)

`cyclictest -S` starts one thread per CPU **in the current cpuset**, and `isolcpus=3` removes
CPU 3 — the core the 500 Hz control loop is meant to run on — from that set. The 30-minute AC-5
run recorded 2026-08-07 (`docs/design-pi-image-stage0b-reference.md` Q1/AC-5 rows) therefore
covered CPUs 0–2 and never touched the isolated core. The pass stands (conservative, not wrong —
the noisy cores cleared the threshold with margin and the isolated core should do no worse), but
the number for the control core is, strictly, still unmeasured.

Fixed forward in the one place the repo tells the operator what to run: `scripts/pi/flash_pi.sh`'s
post-flash guidance. Replaced the bare `cyclictest -m -Sp95 -i 2000 -D 30m` line with the full
protocol from the issue — `stress-ng --cpu 4 --sock 2` load, `cyclictest ... -a 3` pinned to the
isolated core, and a `vcgencmd get_throttled` check afterward (per #230, a throttled run is
uninterpretable, not just pessimistic). Next time this runbook step is executed the repro command
itself is correct, instead of relying on the operator to remember the caveat from the issue.

**Deliberately left out:** did not touch `docs/design-pi-image-stage0b-reference.md`'s Q1/AC-5
rows — they already record the CPU 0-2 caveat honestly, and this reference doc isn't one this
role carries a standing TURF-OVERRIDE on (issue #54's override was scoped to that one split, not
a blanket claim on the file). Updating the row to reflect a re-run against CPU 3, once the CEO
actually executes the corrected command, is follow-up work for whoever reviews that data — not
invented here. Also did not decide whether AC-5's own wording should name the isolated core
explicitly (the issue raises this as a question, not an instruction); that reads as a Promise
(changes what an acceptance criterion says) and belongs in the escalation queue if anyone wants
to press it, not a default action for an unattended run.
