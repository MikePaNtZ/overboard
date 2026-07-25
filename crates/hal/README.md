# hal

The seam between `control-core` and whatever actually drives the wheel: the
`BoardIo` trait exposes a single synchronous `cycle(&mut self, cmd) -> Observation`
call. The backend (sim or hardware) owns the clock and pacing; control-core
never reads one directly, and instead derives dt from the timestamps carried
on consecutive observations.
