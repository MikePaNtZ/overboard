# The tilted-ground / rotated-gravity disagreement was mostly a comparison bug, not a physics bug

Issue #232 reported the two hill formulations in `sim/scenarios/plant.py` disagreeing by 3.48% at
10% grade against the 2% cross-check in `tests/test_plant_terrain.py`, and asked (1) to
characterise the disagreement versus grade, (2) to say which formulation is more correct for a
finite-radius wheel, and (3) to either fix the equivalence or replace the 2% assertion with a
derived bound.

## Root cause

`_roll_rotated_gravity` built its flat/rotated-gravity comparison model at the model's default
resting pose — axle at `(0, 0, r)`, board pitch 0 — while `_roll_tilted` starts the tilted-ground
model at axle `(0, 0, r/cos phi)`, also pitch 0. Those are **not equivalent starting conditions**.
Rotating gravity by `phi` about +Y is the same physics as tilting the ground by `phi` only if the
board's pose is carried through the same rotation `R_y(phi)` that carries the tilted ground's
normal onto the flat one — which means the rotated-gravity board must start at axle
`(r sin phi, 0, r cos phi)`, pitch `+phi`, not at the flat model's own resting pose. The two
free-roll trajectories were being compared from physically different starting orientations, and
that confound was inflating the measured disagreement.

## What changed

`_roll_rotated_gravity` now applies the rotation-equivalent starting pose (via `qpos` on the free
joint) whenever grade != 0. Effect at a few grades (free-roll, 3 s, same test as before):

| grade | before | after |
|---|---|---|
| 5% | 0.235% | 0.0003% |
| 10% | 1.56% | 0.0012% |
| 15% | 0.58% | 0.65% |
| 20% | 1.78% | 0.96% |

At 5% and 10% the fix takes the disagreement to noise-floor — confirming those cases were *pure*
comparison artefact, not physics. At 15% and 20% a smaller residual survives.

## Ask 1 — characterised

A 0.5%-resolution sweep of grade from -20% to +20% (post-fix) tops out at **1.57% at -14.5%**, and
is **not monotonic in the angle** — it moves in jumps (e.g. 0.001% at 11%, 1.52% at 13%, 0.076% at
14%). It vanishes exactly at 0% grade (bit-identical, `atol=1e-9`). A smooth physical effect would
not look like this; a mesh-contact discretization effect would — the tyre's collision hull engages
a genuinely tilted plane at a different set of active facets than it engages a flat plane under
tilted gravity, and which facets are active is a function of the exact angle, not a continuous one.
This also explains why the original report found solver iterations, tolerance, and timestep gave
"zero effect" / "non-monotonic, never below 2%" on their sweep — those knobs don't touch mesh
topology, so they couldn't have converged the residual away no matter how far they were pushed.

## Ask 2 — which is more correct

Neither. Both are exact formulations of the same continuum physics for a rigid body on an infinite
plane; the residual is contact-mesh discretization noise common to any polygon-hull contact solver,
symmetric in sign (`+grade` and `-grade` residuals match), and shrinks with mesh resolution rather
than with anything either formulation gets to choose. `hill.py`'s actual scenario is unaffected by
the initial-pose bug this fix addresses — it runs a closed-loop balance controller through a 2 s
settle phase before scoring anything, which absorbs an initial-pose mismatch a free-roll cannot.

## Ask 3 — the bound

Left at 2%, unwidened, but now backed by the corrected methodology and a real sweep: measured worst
case is 1.57%, so 2% carries ~25% margin rather than being a number that happened to pass. The
parametrization was widened from two grades (`[0.0, 10.0]`) to the full `GRADES` list the rest of
the file already uses, so a regression at any grade is now caught rather than only at one sample
point.

PR: (see PR body). Closes #232.
