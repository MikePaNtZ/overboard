# Runbook — Stage 0A: bench rig assembly, first power-on, first measurements

<!--
covers:
  - sim/scenarios/bench_spinup.py
  - tests/test_bench_spinup.py
reconciled: 2316391
-->

**Executed by: CEO. Owned by: COO.** Written to be followed by one person, alone, safely,
without reading anything else first.

**Stage 0A needs no *tuned* Pi — but it does need a LINUX host, and that host is the Pi.**

⚠️ **Correction, 2026-07-28.** An earlier version of this runbook said "motor, controller and a
laptop are enough" and then, seven lines later, told you to run `ip link show can0`. That is a
**Linux-only** command, and the CEO's laptop is macOS. SocketCAN does not exist on Darwin, and
the CANable's candleLight firmware is a `gs_usb` device that only SocketCAN consumes. The two
statements could not both be true.

What is actually true: **the physics needs no Pi; the host does.** So the Pi and CAN HAT ship in
the first order and act as the bench host from day one. That is strictly better than a macOS
workaround — every 0A measurement is then taken through the same software path Stage 0B will
use, so the CAN stack gets exercised weeks earlier than planned.

The 0A/0B split still stands, and still earns its place: **0A is what you can measure with an
untuned Pi; 0B is what needs the RT kernel, and it produces the latency and jitter go/no-go
number.** You do not need a working RT image to do anything in this runbook — a stock Raspberry
Pi OS boot is enough.

> ⚠️ **The one rule that matters.** A 6374 outrunner with a flywheel bolted to it stores real
> energy and will take a finger. **Nothing spins until the kill path is built and tested in
> §2.** No exceptions, no "just a quick check first."

---

## 1. Parts, and what has to be true before you start

Everything in the **BoM-BENCH-001** view of the
[purchase sheet](https://app.notion.com/p/5f6e09498ad047479729d9d42f56f475). You need all of
`Received` before starting:

- Sensored ~6374 outrunner, 8 mm shaft · flywheel disc + set-screw hub · ¼″ 6061 plate ·
  bench clamps + fasteners
- Little FOCer Rev4 · CANable 2.0 (USB→CAN) · bench power supply or interim pack
- **Raspberry Pi 5 + power supply + storage** — the bench host. A stock Raspberry Pi OS image is
  fine here; the RT kernel is Stage 0B's problem, not yours today.
- *(Optional)* the Waveshare CAN HAT. **Not needed for 0A** — the CANable over USB is the
  transport for this runbook. The HAT is Stage 0B, and its purchase is gated on the SPI
  reproduction test in the Stage-0B design.
- Fuse + holder · contactor · precharge resistor + momentary button
- Klein CL800 clamp meter (**DC capable** — an AC-only meter reads nothing useful here)

**Before you touch anything:** clear the bench, and stand where you are **not** in the plane of
the disc. A shed set screw leaves along that plane.

---

## 2. Build and TEST the kill path — before the motor is ever energised

This section exists because the project's own safety rule is a hardware deadman in series with
motor power, and Stage 0A is the first time that rule becomes physical.

1. Wire, in series on the **pack positive** leg: `pack (+) → fuse → contactor → controller`.
2. Precharge: resistor + momentary button **across the contactor contacts**. Press and hold
   1–2 s before closing the contactor. This limits inrush into the controller's capacitors.
3. The contactor coil is switched by a physical switch **within arm's reach of where you will
   stand.** That switch is the deadman. It is not a software button and never will be.
4. **Test it with no motor connected:** close the contactor, confirm the controller powers up,
   then open the switch and confirm the controller dies. Do this three times.

✅ **Gate: you may not proceed until opening that switch reliably kills controller power.**

---

## 3. Mechanical assembly

1. Bolt the motor to the ¼″ plate. The shaft axis must sit **~60 mm past the desk edge** so the
   disc hangs clear — and so the pendulum upgrade stays possible later.
2. Clamp the plate to the bench. Two clamps minimum, both tight; a plate that walks under
   torque ruins every measurement in §6 and is a hazard.
3. Fit the set-screw hub to the shaft, then the disc. **Set screw onto the shaft flat**, not
   onto round stock. Thread-locker.
4. Rotate by hand: free, no wobble, no contact with anything, full clearance all round.

**Record before going further** — these are inputs to the identification, not paperwork:

- Disc **mass** (kitchen scale, grams) and **outer diameter** (ruler, mm). This is exactly why a
  simple disc was specified: both numbers are trivially verifiable, which the rotor's own
  inertia is not.
- Shaft diameter, and whether the disc is a single plate or stacked.

---

## 4. Electrical

1. Phase leads motor → controller. Bullet or XT90 connectors, fully seated.
2. Hall/sensor cable motor → controller. **Do not force it** — a wrong-pinout hall harness is a
   known multi-week outage.
3. CANable → controller CAN H/L, with a **120 Ω terminator** at each end of the bus.
4. Continuity check with everything **de-energised**: no phase-to-phase short, no phase-to-frame
   short.

---

## 5. First power-on

1. Deadman switch **open**. Confirm it.
2. Connect the supply. Press and hold precharge 1–2 s.
3. Close the deadman. Controller LED should come up. **Hands clear of the disc.**
4. **On the Pi** (not your Mac — these are Linux commands, and that is the whole reason the Pi
   is here):

   ```sh
   sudo ip link set can0 up type can bitrate 500000   # bring the CANable up
   ip -details link show can0                          # confirm it exists and is UP
   candump can0                                        # watch frames before commanding anything
   ```

   `candump` showing traffic from the controller is the gate. **If nothing arrives, stop** — a
   silent bus means you are about to command a motor you cannot hear, and every measurement in
   §6 would be untrustworthy even if it looked fine.

   You reach the Pi over SSH from the Mac, so you are still working from your own keyboard. The
   Mac never talks to the CAN bus directly.
5. Run the VESC motor detection / FOC setup. It will spin the motor briefly. **Stand out of the
   disc plane and keep the deadman within reach.**

✅ **Gate: motor detection completes and reports hall sensors found.** If halls are not
detected, stop — it is a pinout or harness fault, and running sensorless would silently change
every measurement that follows.

---

## 6. First measurements

Dry-run each of these against the sim first — `sim/scenarios/bench_spinup.py` runs the same
profile — so you know the expected shape of the answer before you see the real one.

### 6a. Coast-down (friction)
Spin to a moderate speed, cut the command, log speed decay to rest. Gives bearing, seal and
cogging friction. **Do this before the disc goes on** if you can — friction of the bare motor is
the cleaner number.

### 6b. `kt` — the two-run inertia method
The measurement Stage 0 exists for, and the reason no load cell was purchased.

1. **Run 1 — bare:** commanded current step, log the acceleration ramp.
2. **Run 2 — with the disc:** *identical* commanded current, log the ramp.
3. Two equations, two unknowns:
   `kt·i = J_bare·α₁` and `kt·i = (J_bare + J_disc)·α₂`

⚠️ **Two things that will bite:**
- **Coulomb friction biases `kt` low** by roughly `τ_c / i`. `J_bare` comes out unbiased — the
  friction term cancels between the two runs. So either fit at several currents and extrapolate
  to `1/i → 0`, or subtract the §6a friction first. **Do not trust a single-current `kt`.**
- **Check the conditioning.** The slope ratio is `1 + J_disc/J_bare`. If the two ramps look
  nearly identical, the added inertia is too small and the fit is ill-posed — stack another disc
  and repeat. Report the ratio you actually achieved.

### 6c. Step response
Current steps at three amplitudes, both directions. Gives current-loop bandwidth as a
first-order lag. This is also the data Stage 0B's latency work is compared against.

---

## 7. What Stage 0A does NOT establish

State plainly, because a bench number quietly reused elsewhere is the expensive failure mode:

- **Nothing about balancing.** The motor is bolted to a bench. No inverted equilibrium exists.
- **Nothing about the tyre.** There is no wheel and no ground contact.
- **`kt` measured here is the BENCH motor's**, not the hub motor's. Writing it into the board's
  plant model is a category error that corrupts every torque figure downstream. What *does*
  transfer is the **imperfection profile** — actuation delay, current-loop lag, wheel-rate
  quantum, sensor noise — because those are properties of the shared controller + CAN + host
  path, not of the machine.

---

## 8. Hand back

Log everything raw, push it, and open an issue with the data attached. Analysis happens
**offline**, by Senior Controls, against the sim predictions — not at the bench, and not by the
person holding the deadman.

**Next:** Stage 0B (Pi image, RT kernel, CAN HAT) measures command→actuation latency and jitter
— the architecture go/no-go number.
