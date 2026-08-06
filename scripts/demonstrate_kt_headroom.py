#!/usr/bin/env python3
"""Issue #137 AC4 -- demonstrate that a wrong `kt` moves headroom, not loop gain.

Before this issue, gains were amps-denominated and `kt` never crossed the
control law at all: a `kt` error was a **gain** error, changing loop gain
`kp*kt/J` in a way the controller could not see or bound. After it, gains are
torque-denominated and `kt` enters exactly once, at the actuation boundary,
converting a torque command to the amps the drive accepts.

**Deliberately NOT run against the MuJoCo plant.** The plant has its own
fixed, real `kt` (`sim/scenarios/plant.py::KT_NM_PER_A`, out of scope for this
issue -- it is 0.6284, derived from the Onewheel Hypercore hub motor as of
issue: real-motor-constants; was 0.7 when this script's demonstration was
written). Sweeping the CONTROLLER's assumed `kt` against a plant
whose real `kt` does not move demonstrates a *model-mismatch* effect (a real
and separate concern), not the property this issue's law change buys. To
isolate that property cleanly, this script drives `control_core::PitchRegulator`
+ `safety::Envelope` directly, over the C ABI, exactly as `control-ffi`'s own
tests do -- fixed pitch/rate in, command out, no plant in the loop. See
`crates/control-ffi/src/lib.rs`'s `loop_gain_the_torque_actually_commanded_is_
unchanged_by_kt` and `headroom_moves_with_kt_at_a_fixed_current_limit` for the
Rust-side version of the same demonstration.

    python scripts/demonstrate_kt_headroom.py

Requires the release library: `cargo build --release -p control-ffi`.
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from sim.scenarios.rust_controller import DEFAULT_KD_NM_PER_RAD_S, DEFAULT_KP_NM_PER_RAD, RustController

MAX_CURRENT_A = 40.0


def command(kt_nm_per_a: float, pitch_rad: float):
    """One `ob_controller_update` call, no plant: pitch/rate in, amps out.

    `use_estimator=False` -- the regulator acts on the passed-in truth pitch
    directly, matching `crates/control-ffi`'s own `params()` test helper.
    Leaving the estimator on would fuse a zeroed synthetic IMU sample (no
    real plant behind it here) instead of the pitch this script actually
    wants to probe.
    """
    with RustController(
        kp_nm_per_rad=DEFAULT_KP_NM_PER_RAD,
        kd_nm_per_rad_s=DEFAULT_KD_NM_PER_RAD_S,
        kt_nm_per_a=kt_nm_per_a,
        max_current_a=MAX_CURRENT_A,
        use_estimator=False,
    ) as c:
        amps = c(t_s=0.0, pitch_rad=pitch_rad, pitch_rate_rad_s=0.0, wheel_rate_rad_s=0.0)
        saturated = c.saturated_cycles == 1
    return amps, saturated


def main() -> int:
    print(f"kp_nm_per_rad={DEFAULT_KP_NM_PER_RAD}, kd_nm_per_rad_s={DEFAULT_KD_NM_PER_RAD_S} "
          "(fixed -- identical between every run below)")
    print()

    # --- 1. Loop gain: small error, nowhere near saturation at either kt ----
    pitch = -0.05
    amps_lo, sat_lo = command(0.5, pitch)
    amps_hi, sat_hi = command(0.9, pitch)
    torque_lo, torque_hi = amps_lo * 0.5, amps_hi * 0.9
    print(f"pitch = {pitch} rad (small error, unsaturated either way)")
    print(f"{'':28}{'kt=0.5':>12}{'kt=0.9':>12}")
    print(f"{'amps commanded':28}{amps_lo:>12.4f}{amps_hi:>12.4f}   <- headroom cost DIFFERS")
    print(f"{'torque realised (amps*kt)':28}{torque_lo:>12.4f}{torque_hi:>12.4f}   <- LOOP GAIN, unchanged")
    print(f"  loop gain difference: {abs(torque_lo - torque_hi):.2e} N*m "
          "(zero up to float32 rounding)")
    print()

    # --- 2. Headroom: an error large enough to expose the ceiling moving ---
    pitch = -0.5   # -> 28 N*m intended, exactly between the two ceilings below
    tau_max_lo, tau_max_hi = 0.5 * MAX_CURRENT_A, 0.9 * MAX_CURRENT_A
    amps_lo, sat_lo = command(0.5, pitch)
    amps_hi, sat_hi = command(0.9, pitch)
    print(f"pitch = {pitch} rad -> {DEFAULT_KP_NM_PER_RAD * abs(pitch):.1f} N*m intended")
    print(f"{'':28}{'kt=0.5':>12}{'kt=0.9':>12}")
    print(f"{'tau_max = kt * I_max':28}{tau_max_lo:>12.1f}{tau_max_hi:>12.1f}   N*m  <- HEADROOM, moves")
    print(f"{'amps commanded':28}{amps_lo:>12.4f}{amps_hi:>12.4f}   A")
    print(f"{'saturated':28}{str(sat_lo):>12}{str(sat_hi):>12}")
    print()
    print("At kt=0.5 the SAME 28 N*m command exceeds a 20 N*m ceiling and saturates; "
          "at kt=0.9 it is within a 36 N*m ceiling and does not. The torque the law "
          "asked for never changed -- only how much of it the drive's 40 A can deliver.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
