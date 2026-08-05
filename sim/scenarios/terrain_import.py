"""Terrain import — where SCANNED ground becomes a MuJoCo heightfield.

Nothing here talks to MuJoCo. This is the step a future LiDAR/photogrammetry
scan of real ground goes through before `lip.py` (or its successors) turn a
profile into an `hfield`: raw, arbitrarily-spaced samples in, a uniform post
grid out. Pure numpy, so it can run against a scan file with no sim
dependency at all.
"""

from __future__ import annotations

import numpy as np

#: Spacing between output posts, metres.
#:
#: ASSUMPTIONS, NOT MEASUREMENTS. Derived from the 18 psi contact-patch
#: arithmetic in `overboard_onewheel.xml`'s tyre-compliance comment (loaded
#: static deflection 4.5 mm at 82.5 kg -> patch length ~0.072 m, see
#: `PATCH_LENGTH_M` below); this is roughly half that patch, chosen to give
#: the envelope filter below a few input posts per patch rather than one.
#: NO ACCEPTANCE THRESHOLD MAY BE DERIVED FROM THIS NUMBER -- ADR-0011
#: criterion (f) exists to stop exactly that. Both constants are tuned
#: against the `lip.py` gate today and get replaced by a measured
#: load-deflection curve at the hardware gate.
POST_SPACING_M = 0.035

#: EFFECTIVE enveloping length, metres — a PRODUCT TUNE, not the physical
#: contact patch.
#:
#: The 18 psi arithmetic in `overboard_onewheel.xml`'s wheel comment gives
#: 0.072 m for the tyre's own loaded patch, and 0.072 m was this constant's
#: first value. Measured against the `lip.py` gate it is not enough: a 50 mm
#: step enveloped at 0.072 m still nose-strikes at 2 m/s cruise, because the
#: filter only stands in for the TYRE, and the model has no rider — no
#: ankles, no knees, none of the compliance that actually pops a real board
#: over a crack (`plant.py:81`, "a rigid lump bolted to the frame"). Widening
#: the kernel is the one knob that stands in for all of that at zero cost to
#: actuator authority — softening the wheel spring was swept 180k→45k N/m
#: and moved the strike boundary not at all (the binding impulse is
#: horizontal; a vertical spring never sees it).
#:
#: 0.200 m is CEO-directed (2026-08-02): the product requirement is that the
#: board rides out cracks up to ~2 in (50 mm) and CRASHES on anything
#: taller — a kerb strike is a crash by design, handed to the game engine.
#: Measured at this kernel, enveloped steps: 50/60 mm CLEAR, 70+ mm STRIKE,
#: 150 mm kerb STRIKE. (0.250 m moved the boundary to 70/80 — too generous;
#: 0.150 m failed the 50 mm requirement.)
#:
#: NO ACCEPTANCE THRESHOLD MAY BE DERIVED FROM THIS NUMBER (ADR-0011
#: criterion (f)): it is an authored stand-in pending the hardware-gate
#: measurement (ride the real board over a known cleat, fit the kernel to
#: the measured response). The 50 mm requirement itself is environmental —
#: supplied by the CEO — which is the direction ADR-0011 says a tolerance
#: must come from.
PATCH_LENGTH_M = 0.200


def envelope_profile(
    xs_m: np.ndarray,
    zs_m: np.ndarray,
    post_spacing_m: float = POST_SPACING_M,
    patch_length_m: float = PATCH_LENGTH_M,
) -> tuple[np.ndarray, np.ndarray]:
    """Filter a raw ground profile the way a finite tyre patch would.

    A pneumatic tyre's contact patch has finite length, so it physically
    cannot follow ground wavelengths shorter than the patch -- it bridges
    them instead of tracing them. Applying that filter to the IMPORTED
    TERRAIN, rather than to the plant, costs zero actuator authority and zero
    simulated DOFs, and it is deterministic: no RNG, no dependency on the
    solver.

    ORDER MATTERS: smooth first, at the INPUT'S own resolution, and only then
    resample onto the uniform post grid. Decimating first would fold
    sub-post wavelengths in as aliases rather than removing them, and a 72 mm
    patch over 35 mm posts is only ~2 posts wide -- nowhere near enough
    samples to act as a filter kernel. The averaging has to happen while the
    input's finer spacing is still available.

    1. Validate the input.
    2. Moving-average `zs_m` with a centred kernel spanning `patch_length_m`,
       sized in INPUT samples, with edge-replicated padding so plateaus at
       either end stay flat instead of drooping toward zero.
    3. Resample the smoothed profile onto uniform posts `post_spacing_m`
       apart, by linear interpolation.

    Returns `(xs_uniform, zs_smoothed)`.
    """
    xs = np.asarray(xs_m, dtype=float)
    zs = np.asarray(zs_m, dtype=float)
    if xs.shape != zs.shape:
        raise ValueError(f"xs_m and zs_m must be the same length, got {xs.shape} vs {zs.shape}")
    if xs.size < 2:
        raise ValueError(f"need at least 2 samples, got {xs.size}")
    if not np.all(np.diff(xs) > 0.0):
        raise ValueError("xs_m must be strictly increasing")

    # -- 2. moving average at INPUT resolution -----------------------------
    dx = float(np.median(np.diff(xs)))
    n = 2 * int(np.floor(patch_length_m / (2.0 * dx))) + 1
    if n <= 1:
        zs_smoothed_input = zs
    else:
        half = n // 2
        padded = np.pad(zs, half, mode="edge")
        kernel = np.ones(n) / n
        zs_smoothed_input = np.convolve(padded, kernel, mode="valid")

    # -- 3. THEN resample onto uniform posts -------------------------------
    xs_uniform = np.arange(xs[0], xs[-1], post_spacing_m)
    if xs_uniform.size == 0 or not np.isclose(xs_uniform[-1], xs[-1]):
        xs_uniform = np.append(xs_uniform, xs[-1])
    zs_uniform = np.interp(xs_uniform, xs, zs_smoothed_input)

    return xs_uniform, zs_uniform
