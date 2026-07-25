# board-app

The binary that wires `control-core` and `safety` to a `hal::BoardIo`
backend and runs a fixed-step loop, printing a heartbeat each cycle.
`--backend sim` and `--backend null` both currently resolve to the
`sim-backend` stub. Run `board-app --help` for usage.
