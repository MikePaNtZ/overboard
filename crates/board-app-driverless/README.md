# board-app-driverless

The binary that wires `control-core` and `safety` to a `hal` + `hal-actuate`
backend and runs the ICD §5.2 control loop: **observe → compute → clamp →
apply**, with `wait_observe()` the only call that advances time. Prints a
periodic heartbeat showing the cycle, IMU batch size, measured current, and
both clamp stages.

This is the binary with motion authority — it links `hal-actuate` and
`sim-backend`. The ridden binary is `board-app-ridden`, which links `hal`
only and cannot actuate; `crates/xtask` gates that at the dependency-graph
level.

`--backend sim` and `--backend null` both currently resolve to the
`sim-backend` stub. Run `board-app-driverless --help` for usage.
