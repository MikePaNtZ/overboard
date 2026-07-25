# safety

Pure safety envelope sitting between `control-core`'s output and whatever
`hal` backend applies it: arm/disarm gating and (eventually) command
clamping based on faults. Currently a stub `Envelope` that passes commands
through unchanged when armed and fault-free, and zeroes them otherwise; real
tilt/current limits land with the actual controller.
