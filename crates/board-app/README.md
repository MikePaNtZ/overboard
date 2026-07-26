# board-app

The binary that wires `control-core` and `safety` to a `hal` backend and runs
the ICD §5.2 control loop: **observe → compute → clamp → apply**, with
`wait_observe()` the only call that advances time. Prints a periodic heartbeat
showing the cycle, IMU batch size, measured current, and both clamp stages.

`--backend sim` and `--backend null` both currently resolve to the
`sim-backend` stub. Run `board-app --help` for usage.
