"""Bench-rig riser: first structural mode, against the >50 Hz Stage-0A target.

Issue #64: nobody had checked whether the 1/4" 6061 riser is stiff enough
before the plate was cut. The plate is bought and non-returnable, so this is
a verdict on the design as it stands, not a chance to pick a new thickness.

WHY THIS MATTERS, AND WHY IT IS EASY TO MISS
---------------------------------------------
Runbook Sec 3.2 requires the fixture's first resonance well above the control
bandwidth (>50 Hz). Motor TORQUE loads the riser about its own axis -- the
riser's face is the X-Z plane (see bench_rig.xml's header), so an in-plane
moment there uses the riser's large width and height as bending dimensions
and is stiff. The problem is the FLYWHEEL MASS hanging off the shaft: it
overhangs in Y, off the plate's face, and its weight bends the riser OUT of
that plane -- deflection in Y, resisted only by the plate's 6.35 mm
THICKNESS. Bending stiffness scales with the cube of the beam depth in the
bending direction, so the riser's weak axis is ~(120/6.35)^3 ~= 6800x more
compliant than its strong one. A mode that sits inside the control bandwidth
would corrupt the very torque->pitch identification the rig exists to
perform, while producing bench data that LOOKS like a plant defect rather
than a fixture one -- see `describe_bench_signature` for how to tell them
apart.

MODEL: A CANTILEVER BEAM WITH A TIP MASS
-----------------------------------------
The riser is fixed at its base (bolted/co-planar with the base plate the
clamps grip) and free at the top, where the motor stator bolts on. Treating
it as a uniform cantilever, weak-axis bending only:

    I      = b * t^3 / 12                  (b = width, t = thickness, weak
                                             axis: bending moment about X,
                                             deflection along Y)
    k      = 3 * E * I / L^3               (tip stiffness, point load at L)
    m_eff  = m_tip + 0.236 * m_beam         (Rayleigh correction: a uniform
                                             cantilever's own mass acts at
                                             the tip as roughly a quarter of
                                             itself, for a first-mode estimate)
    f1     = (1 / 2*pi) * sqrt(k / m_eff)

This is a first-mode ROM (rough order of magnitude), not an FEA: it ignores
shear deformation (fine, L/t ~= 31, a slender-beam assumption holds well),
rotary inertia of the tip mass, and any stiffness the desk clamp itself
lacks (a separate, second candidate resonance -- see bench_rig.xml's note on
the IRWIN Quick-Grip minis). `verdict()` reports a margin, not a single
pass/fail line, because a ROM this coarse deserves one.

WHAT IS MEASURED, WHAT IS CONFIRMED, WHAT IS ASSUMED
-----------------------------------------------------
    MEASURED    riser height (0.20 m) and width (0.12 m) come straight off
                the compiled `bench_rig.xml` geom -- the same rig the model
                describes, not a separate guess.

    CONFIRMED   riser thickness (6.35 mm, exact 1/4"), flywheel mass
                (152 g each, x2) and material (6061-T651) are manufacturer
                or purchase-order figures, per issues #65/#66. The thickness
                is hardcoded here rather than read off the compiled model
                because the model currently rounds it to 6 mm (#66's fix is
                a separate, unmerged PR) -- this analysis uses the true
                purchased dimension either way.

    ASSUMED     motor mass. No datasheet in this repo states it, and this
                sandbox has no web access to find one -- a 6374-class
                sensored outrunner is assumed at 500-650 g (a working range,
                not a measurement) and every result below is reported across
                that range rather than at a single point. THE FIX FOR THIS
                ASSUMPTION IS A KITCHEN SCALE, not a better guess -- weighing
                the actual motor takes under a minute and turns this from an
                estimate into a fact, same convention as the flywheel mass
                before it shipped as a bought part.
"""

from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path

import mujoco

REPO = Path(__file__).resolve().parents[2]
MODEL_PATH = REPO / "sim" / "models" / "bench_rig.xml"
OUT_DIR = REPO / "sim" / "out" / "bench"

#: Height up the riser (from the base plate) a diagonal gusset is evaluated
#: at when the unbraced design comes out marginal or inadequate. 80 mm is
#: 40% of the 200 mm riser height -- comfortably short of the motor mount at
#: the top, and, per `assess_riser`'s own numbers, enough margin on its own
#: that hunting for the minimum viable brace height is not worth doing.
DEFAULT_BRACE_HEIGHT_M = 0.08

#: 6061-T651, per bench_rig.xml's own material call-out.
ALUMINUM_E_PA = 68.9e9
ALUMINUM_DENSITY_KG_M3 = 2700.0

#: Confirmed purchased dimension (#66) -- see module docstring for why this
#: is hardcoded rather than read off the compiled model.
RISER_THICKNESS_M = 0.00635

#: goBILDA 3628-0032-0082, manufacturer-published (#65/#66). Two are bolted
#: to the shaft.
FLYWHEEL_MASS_EACH_KG = 0.152
FLYWHEEL_COUNT = 2

#: A 6374-class sensored outrunner's mass, per no datasheet this repo has
#: access to -- a stated working assumption, not a measurement. See the
#: module docstring's ASSUMED section.
MOTOR_MASS_LOW_KG = 0.500
MOTOR_MASS_HIGH_KG = 0.650

#: Runbook Sec 3.2.
TARGET_HZ = 50.0

#: `verdict()` bands. "Adequate" asks for margin above the target, not just
#: clearing it -- a mode sitting at 51 Hz is not a result anyone should be
#: comfortable calling done.
ADEQUATE_MARGIN = 1.5


@dataclass(frozen=True)
class RiserGeometry:
    length_m: float
    width_m: float
    thickness_m: float


def riser_geometry_from_model(path: Path = MODEL_PATH) -> RiserGeometry:
    """Height and width straight off the compiled rig -- see the module
    docstring's MEASURED/CONFIRMED/ASSUMED breakdown for why thickness is
    not read from here too."""
    model = mujoco.MjModel.from_xml_path(str(path))
    geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "riser")
    if geom_id < 0:
        raise ValueError("bench_rig.xml has no geom named 'riser'")
    half_x, _half_y, half_z = model.geom_size[geom_id]
    return RiserGeometry(
        length_m=2.0 * half_z,
        width_m=2.0 * half_x,
        thickness_m=RISER_THICKNESS_M,
    )


#: bench_rig.xml's own hub geom density (6061 aluminium, matching its
#: header). Compiled `MjModel` does not expose per-geom density once built,
#: so the volume is read off the compiled geometry and paired with this
#: same constant the XML declares -- not re-derived or re-guessed.
_HUB_DENSITY_KG_M3 = 2700.0


def hub_mass_from_model(path: Path = MODEL_PATH) -> float:
    """The set-screw hub's mass, exactly -- it is modelled geometry (density
    x volume), not an assumption, unlike the motor mass beside it."""
    model = mujoco.MjModel.from_xml_path(str(path))
    geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "hub")
    if geom_id < 0:
        raise ValueError("bench_rig.xml has no geom named 'hub'")
    radius, half_len, _ = model.geom_size[geom_id]
    volume = math.pi * radius**2 * (2.0 * half_len)
    return _HUB_DENSITY_KG_M3 * volume


def cantilever_first_mode_hz(
    tip_mass_kg: float,
    geometry: RiserGeometry,
    unbraced_length_m: float | None = None,
) -> float:
    """First bending-mode frequency of the riser as a cantilever with a tip
    mass, weak-axis (out-of-plane) bending only.

    `unbraced_length_m` lets a proposed brace be evaluated by shortening the
    unsupported span, rather than duplicating this whole function for the
    braced case -- see `braced_first_mode_hz`.
    """
    length = unbraced_length_m if unbraced_length_m is not None else geometry.length_m
    moment_of_inertia = geometry.width_m * geometry.thickness_m**3 / 12.0
    stiffness = 3.0 * ALUMINUM_E_PA * moment_of_inertia / length**3
    beam_mass = ALUMINUM_DENSITY_KG_M3 * geometry.width_m * geometry.thickness_m * length
    effective_mass = tip_mass_kg + 0.236 * beam_mass
    return math.sqrt(stiffness / effective_mass) / (2.0 * math.pi)


def tip_mass_bounds_kg(hub_mass_kg: float) -> tuple[float, float]:
    """(low, high) mass hanging off the riser's free end: motor (assumed
    range) + hub (exact, from the model) + both flywheels (confirmed)."""
    flywheels = FLYWHEEL_MASS_EACH_KG * FLYWHEEL_COUNT
    low = MOTOR_MASS_LOW_KG + hub_mass_kg + flywheels
    high = MOTOR_MASS_HIGH_KG + hub_mass_kg + flywheels
    return low, high


def verdict(freq_low_hz: float, freq_high_hz: float) -> str:
    """"adequate" / "marginal" / "inadequate" against `TARGET_HZ`, evaluated
    at the PESSIMISTIC (lower) end of the assumed range -- an estimate this
    coarse should not get to claim the optimistic case."""
    worst = min(freq_low_hz, freq_high_hz)
    if worst >= TARGET_HZ * ADEQUATE_MARGIN:
        return "adequate"
    if worst >= TARGET_HZ:
        return "marginal"
    return "inadequate"


def gusset_brace_first_mode_hz(
    tip_mass_kg: float, geometry: RiserGeometry, brace_height_m: float
) -> float:
    """First mode with a diagonal gusset from the base plate to
    `brace_height_m` up the riser, modelled as shortening the unbraced span
    to `geometry.length_m - brace_height_m` -- i.e. treating the brace
    attachment as a near-rigid support, which a triangulated strut in
    axial tension/compression approximates far better than the unbraced
    plate approximates a rigid mount. This is the same ROM-level
    simplification as the rest of this module, not an FEA of the brace
    itself: it is meant to show whether the fix is in the right ballpark,
    which a tap test then confirms.
    """
    unbraced = geometry.length_m - brace_height_m
    return cantilever_first_mode_hz(tip_mass_kg, geometry, unbraced_length_m=unbraced)


def describe_bench_signature(freq_hz: float) -> str:
    return (
        f"A riser mode near {freq_hz:.0f} Hz would show up in Sec 6c step-response "
        "captures as ringing at that frequency riding on top of the expected "
        "torque/pitch response -- present on every step regardless of commanded "
        "current or direction, NOT scaling with the current-loop time constant, "
        "and reproducible by tapping the rig by hand near the motor with the "
        "controller idle (a phone microphone's spectrum view is enough to read "
        "the frequency off directly). A plant defect does not do any of that: "
        "it depends on the command, and it vanishes when the motor is off."
    )


@dataclass
class RiserAssessment:
    """The full #64 verdict: as-designed, and braced if that verdict needed it.

    Every frequency is reported as a (pessimistic, optimistic) pair across
    the assumed motor-mass range -- see the module docstring's ASSUMED
    section -- rather than collapsed to one number.  `verdict` and
    `braced_verdict` are both evaluated at the pessimistic end.
    """

    geometry: RiserGeometry
    hub_mass_kg: float
    tip_mass_low_kg: float
    tip_mass_high_kg: float
    first_mode_low_hz: float
    first_mode_high_hz: float
    verdict: str
    bench_signature: str
    brace_height_m: float | None = None
    braced_first_mode_low_hz: float | None = None
    braced_first_mode_high_hz: float | None = None
    braced_verdict: str | None = None

    def to_dict(self) -> dict:
        d = asdict(self)
        d["geometry"] = asdict(self.geometry)
        return d


def assess_riser(
    path: Path = MODEL_PATH,
    brace_height_m: float = DEFAULT_BRACE_HEIGHT_M,
    geometry: RiserGeometry | None = None,
    hub_mass_kg: float | None = None,
) -> RiserAssessment:
    """The single entry point for issue #64: as-designed verdict, and a
    braced re-check if (and only if) the unbraced design does not clear
    `TARGET_HZ` with margin.

    `geometry`/`hub_mass_kg` let a test exercise the early-return path (an
    adequate-as-designed riser) without needing a second XML fixture on
    disk; production callers should leave both at their defaults, which read
    the real compiled rig.
    """
    if geometry is None:
        geometry = riser_geometry_from_model(path)
    if hub_mass_kg is None:
        hub_mass_kg = hub_mass_from_model(path)
    hub_mass = hub_mass_kg
    tip_low, tip_high = tip_mass_bounds_kg(hub_mass)

    # Heavier tip mass -> lower frequency, so pair (tip_high -> f_low).
    f_low = cantilever_first_mode_hz(tip_high, geometry)
    f_high = cantilever_first_mode_hz(tip_low, geometry)
    as_designed_verdict = verdict(f_low, f_high)

    assessment = RiserAssessment(
        geometry=geometry,
        hub_mass_kg=hub_mass,
        tip_mass_low_kg=tip_low,
        tip_mass_high_kg=tip_high,
        first_mode_low_hz=f_low,
        first_mode_high_hz=f_high,
        verdict=as_designed_verdict,
        bench_signature=describe_bench_signature(f_low),
    )

    if as_designed_verdict == "adequate":
        return assessment

    braced_low = gusset_brace_first_mode_hz(tip_high, geometry, brace_height_m)
    braced_high = gusset_brace_first_mode_hz(tip_low, geometry, brace_height_m)
    assessment.brace_height_m = brace_height_m
    assessment.braced_first_mode_low_hz = braced_low
    assessment.braced_first_mode_high_hz = braced_high
    assessment.braced_verdict = verdict(braced_low, braced_high)
    return assessment


def main() -> int:
    assessment = assess_riser()
    print(f"=== bench riser first mode  [target >= {TARGET_HZ:.0f} Hz]")
    print(f"  geometry        L={assessment.geometry.length_m * 1000:.0f} mm, "
          f"b={assessment.geometry.width_m * 1000:.0f} mm, "
          f"t={assessment.geometry.thickness_m * 1000:.2f} mm")
    print(f"  tip mass range  {assessment.tip_mass_low_kg:.3f} - "
          f"{assessment.tip_mass_high_kg:.3f} kg "
          f"(hub {assessment.hub_mass_kg * 1000:.1f} g exact, motor "
          f"{MOTOR_MASS_LOW_KG * 1000:.0f}-{MOTOR_MASS_HIGH_KG * 1000:.0f} g ASSUMED)")
    print(f"  first mode      {assessment.first_mode_low_hz:.1f} - "
          f"{assessment.first_mode_high_hz:.1f} Hz  -> {assessment.verdict.upper()}")
    if assessment.braced_verdict is not None:
        print(f"  with a gusset to {assessment.brace_height_m * 1000:.0f} mm: "
              f"{assessment.braced_first_mode_low_hz:.1f} - "
              f"{assessment.braced_first_mode_high_hz:.1f} Hz  -> "
              f"{assessment.braced_verdict.upper()}")
    print(f"  {assessment.bench_signature}")

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    out_path = OUT_DIR / "bench_riser.json"
    out_path.write_text(json.dumps(assessment.to_dict(), indent=2) + "\n")
    print(f"wrote {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
