# control-core

Pure, deterministic balance control logic -- no I/O, no clock reads, depends
on `board-types` only (not `hal`), so it can be unit-tested and fuzzed with
no backend in the loop. Currently a stub `Controller` that always returns
`Command::ZERO`; the real cascaded (inner rate loop / outer tilt loop)
controller is a future milestone.
