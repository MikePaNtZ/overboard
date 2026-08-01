# What a kerb strike is worth — and why N·s is the wrong currency to answer in

Issue #142 AC2 asked this role what an obstacle strike is worth, because the reference disturbance
the #132 gain retune is scoped against — 20 N·s — is inherited from a scenario nominal and derived
from nothing. Delivered as `kerb_strike_impulse()` / `kerb_strike_vs_com_impulse()` in `plant.py`
with tests in `tests/test_plant_kerb_strike.py`. PR #145.

**The finding that matters more than the number.** `impulse_response.py` applies its 20 N·s
*through the CoM*, deliberately — at `application_height_m = 0.0` the disturbance is purely linear
and the initial pitch rate is zero by construction. Its own docstring already says a kerb strike is
the follow-on case it is *not* modelling, and that the angular channel would swamp the linear one.
A kerb strike lands ~0.7 m below the CoM and imparts 100–450 deg/s immediately. **So a reference
disturbance quoted in N·s and fed to the existing scenario understates a kerb silently, in the one
channel that actually topples the board.** Controls needs pitch rate imparted, not newton-seconds.
Answering AC1 in N·s without saying this would have been technically responsive and misleading.

**The number, for what it is worth.** A 20 mm lip — brick edge, root heave, the kind of thing a
pavement has every few metres — gives 28–44 N·s at walking pace and 113–176 N·s at 8 m/s. The
inherited 20 N·s is exceeded at every speed the board is meant to ride at, measured against the
*lower* bound. The inherited nominal is not conservative.

## The first model was wrong, and it looked fine

Conserving angular momentum about the step edge and letting the body rotate rigidly about it — the
textbook step-climb model — charges **Δv = 0.25·v for a step of height ZERO**. A zero-height step
is not an obstacle. The defect: rotation about the contact point is only forced when the wheel is
*blocked*, and nothing blocks it as h→0.

Every number that model produced was plausible. What caught it was checking a limit whose answer is
known independently of the model. **Check the free limits first — they are the only assertions that
do not depend on the thing being tested.** That is now the first test in the file.

## The friction gate, which replaced a judgement call with a derivation

The pivot model does not merely get harsh at small heights, it becomes unphysical, and the
discriminator is derivable. Pivoting means the edge arrests the wheel's roll, which needs a
tangential impulse alongside the normal one; their ratio is the friction the edge must supply:

| h | 1 mm | 5 mm | 10 mm | 20 mm | 30 mm | 50 mm | 100 mm |
|---|---|---|---|---|---|---|---|
| μ required | 5.43 | 2.34 | 1.57 | **0.99** | 0.70 | 0.38 | 0.08 |

Rubber on asphalt is μ ≈ 1.0 — the model's own `<geom friction>`. So the pivot is unreachable below
about 20 mm: the edge slips and the wheel rolls through. **That is the same ~20 mm the tyre's own
deflection gives, arrived at independently**, and two unrelated criteria landing on one boundary is
the strongest reason to believe either. Below it the bracket collapses to the frictionless value
rather than quoting an upper bound that cannot happen.

## Open input this role still owes

The tyre deflection figure (10–20 mm) is an assumption, not a measurement. It wants the tyre spec
and a load-deflection check at riding pressure. Until that exists, anything below ~20 mm is a bound
rather than an answer. Note the sim's wheel is a **rigid cylinder**, so for scoping a retune
*against the sim* these numbers are exactly right; the tyre caveat applies to claims about the real
board, where they are an upper bound.
