# sim-backend

Implements the `hal` seam (`BoardObserve` + `BoardActuate`) against the MuJoCo
onewheel model (`sim/models/overboard_onewheel.xml`).

**Still a stub — it does not step MuJoCo yet.** It advances a synthetic 500 Hz
clock and returns synthetic observations, which exercises the call-sequence
contract end to end but is not a plant. Wiring `mj_step` through FFI is the next
increment; until it lands, SR-SIM-5 stays unmet.

What the stub already honours, because retrofitting these is expensive:

- `wait_observe()` advances time; `apply()` does not.
- Double `apply()`, and `apply()` before the first `wait_observe()`, are both
  `ProtocolViolation` — and nothing is buffered when they are refused, because
  the contract is that no frame is sent, not merely that an error is returned.
- Measured current is **not** an echo of the command. It appears a cycle later,
  because actuation delay is additive and an echoing backend is explicitly
  non-conforming (ICD §12).
- Cold start reports `Invalid` rather than presenting zeros as measurements.
