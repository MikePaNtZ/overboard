"""The rider proxy must never change the physics.

These clips get shared, so the figure standing on the board is the most
persuasive thing this project produces. It is also purely decorative: the
ballast is a rigid point mass with an explicit `<inertial>`, and the geoms
exist so a viewer can see where 70 kg actually sits.

That only stays true if nobody later gives a rider geom a density, a contact
type, or a mass. This asserts it rather than trusting the comment.
"""

import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "scripts"))

from experiment import build_model, one_run, plant_summary  # noqa: E402

BALLAST_KG, BALLAST_M, CLAMP_A = 70.0, 0.75, 40.0


@pytest.fixture(scope="module")
def models():
    return {s: build_model(BALLAST_KG, BALLAST_M, CLAMP_A, s) for s in ("sphere", "figure")}


def test_the_rider_visual_does_not_move_the_centre_of_mass(models):
    a, b = (plant_summary(models[s]) for s in ("sphere", "figure"))
    assert a == b, f"rider style changed the plant: {a} vs {b}"
    # And the plant is the one we think it is.
    assert a["inverted_pendulum"] is True
    assert a["total_mass_kg"] == pytest.approx(82.5)


def test_the_rider_visual_does_not_change_a_single_metric(models):
    """The strong form: a full closed-loop run must be identical."""
    _, a = one_run(models["sphere"], 200.0, 30.0, CLAMP_A, 20.0, 4.0)
    _, b = one_run(models["figure"], 200.0, 30.0, CLAMP_A, 20.0, 4.0)
    assert a == b, f"rider style changed the outcome: {a} vs {b}"


def test_every_rider_geom_is_non_colliding_and_massless(models):
    """Belt and braces, straight off the compiled model.

    The run comparison above would catch a collision that actually fires, but
    not one that merely *could* -- a geom that never happens to touch anything
    in this particular scenario. Read the flags instead of inferring them.
    """
    import mujoco

    m = models["figure"]
    rider = [
        i for i in range(m.ngeom)
        if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i) or "").startswith("rider_")
    ]
    assert rider, "no rider geoms found -- has the naming changed?"
    for i in rider:
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i)
        assert m.geom_contype[i] == 0, f"{name} can collide"
        assert m.geom_conaffinity[i] == 0, f"{name} can be collided with"
