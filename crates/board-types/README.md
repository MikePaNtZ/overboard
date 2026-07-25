# board-types

Shared types for the Overboard control stack: `Command`, `Observation`,
`Faults`, `Params`, and SI-unit newtypes (`Amps`, `Radians`, `RadPerSec`).
`no_std`-friendly and dependency-light so it can eventually be shared with a
bare-metal or PREEMPT_RT-hosted binary; every other crate in the workspace
depends on this one.
