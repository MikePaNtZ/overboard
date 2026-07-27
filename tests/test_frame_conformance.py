"""The model→ICD frame map, tested as a CONTRACT rather than as a behaviour.

This is the test whose absence let a reflection ship. `plant.imu_readings`'
docstring used to cite `test_imu_frame_matches_truth` as pinning the
conversion; that test was never written, and the map it was supposed to protect
had `det = −1`.

WHY THIS FILE IS DYNAMICS-FREE, AND WHY THAT IS THE POINT
---------------------------------------------------------
The previous map was "determined EMPIRICALLY against framequat truth". That
method is what failed. Deriving a sign convention from a simulation run makes
the answer depend on the regime you sampled — and the regime sampled (quiet,
near-upright) is precisely where this class of error is invisible, because the
error vanishes at θ = 0 and grows as 2θ.

So nothing here integrates, contacts the ground, or reads a sensor. A frame
conversion is a *definition*: two frames are declared, the map between them
follows, and it is checkable with linear algebra. ICD §10.3 names the sim as
the arbiter of the *polarity gate* — a dynamical question about which way the
plant moves. It does not make the sim the arbiter of handedness, and reading it
that way is what licensed the defect.

SCOPE. This asserts the frame contract only. It deliberately says nothing about
estimator accuracy: that is Senior Controls' acceptance criterion, measured in
tests/test_estimator.py. Keeping this file purely contractual is what keeps the
seam clean — a contract test that also encodes someone else's AC becomes a
tripwire on their tuning.
"""

import math

import numpy as np
import pytest

from sim.scenarios.plant import R_MODEL_TO_ICD

# ICD §10.1: FRD, right-handed — x forward (nose), y right, z down.
# Model (overboard_onewheel.xml): z-up, forward = −X, therefore right = +Y.
MODEL_FORWARD = np.array([-1.0, 0.0, 0.0])
MODEL_RIGHT = np.array([0.0, 1.0, 0.0])
MODEL_UP = np.array([0.0, 0.0, 1.0])

ICD_FORWARD = np.array([1.0, 0.0, 0.0])
ICD_RIGHT = np.array([0.0, 1.0, 0.0])
ICD_DOWN = np.array([0.0, 0.0, 1.0])

G = 9.81


def test_the_map_is_a_proper_rotation():
    """`det = +1`. The one assertion that would have caught the original bug.

    A reflection maps a right-handed frame to a left-handed one. ICD §10.1 says
    "right-handed" in as many words, so `det = −1` is not a near miss — it is a
    direct contradiction of the spec, checkable without knowing any physics.
    """
    det = float(np.linalg.det(R_MODEL_TO_ICD()))
    assert det == pytest.approx(1.0, abs=1e-12), (
        f"det = {det:+.6f}. A frame change is a proper rotation; det = −1 is a "
        "reflection and yields a left-handed frame (ICD §10.1 requires right-handed)."
    )


def test_the_map_is_orthonormal():
    """R·Rᵀ = I. Together with det = +1 this is the definition of a rotation,
    and it also forbids a map that quietly rescales an axis."""
    R = R_MODEL_TO_ICD()
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-12)


def test_the_three_body_axes_land_where_the_icd_says():
    """The map is fixed by the two frame definitions, so check it against them
    directly rather than against a number someone recorded."""
    R = R_MODEL_TO_ICD()
    assert np.allclose(R @ MODEL_FORWARD, ICD_FORWARD), "model forward (−X) must map to ICD +x"
    assert np.allclose(R @ MODEL_RIGHT, ICD_RIGHT), "model right (+Y) must map to ICD +y"
    assert np.allclose(R @ MODEL_UP, -ICD_DOWN), "model up (+Z) must map to ICD −z (z is DOWN)"


@pytest.mark.parametrize("deg", [-40, -25, -15, -8, -4, -1, 0, 1, 4, 8, 15, 25, 40])
def test_accel_derived_pitch_recovers_the_true_angle(deg):
    """The consequence that actually bit: a supported accelerometer at pitch θ
    must yield +θ through `control-core`'s estimator formula, not −θ.

    Specific force on a supported board is `+g·ẑ_world`, rotated into the body
    frame. No contact solver, no integration — just the tilt.

    The formula is `control-core`'s `ComplementaryFilter::accel_pitch`:
    `θ = atan2(f_x − a, −f_z)`, with the linear-acceleration term zero here.
    That formula is correct for FRD and is NOT the thing under test; the frame
    map is. If this fails, the map is wrong, not the estimator.
    """
    phi = math.radians(deg)
    # Rotation of the body by +phi about +Y in the model frame (nose-up).
    c, s = math.cos(phi), math.sin(phi)
    R_body = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])

    f_model = R_body.T @ np.array([0.0, 0.0, G])   # what the sensor reads, supported
    f_icd = R_MODEL_TO_ICD() @ f_model

    theta = math.degrees(math.atan2(f_icd[0], -f_icd[2]))
    assert theta == pytest.approx(deg, abs=1e-9), (
        f"accel-derived pitch {theta:+.4f}° for a true {deg:+d}°. "
        f"A result of {-deg:+d}° means the map inverted the x channel."
    )


@pytest.mark.parametrize(
    "model_rate, axis, expected_icd",
    [
        ((0.0, 0.7, 0.0), "pitch (nose-up, +y)", (0.0, 0.7, 0.0)),
        ((0.7, 0.0, 0.0), "roll (+x, right-wing-down)", (-0.7, 0.0, 0.0)),
        ((0.0, 0.0, 0.7), "yaw (+z, down)", (0.0, 0.0, -0.7)),
    ],
)
def test_every_gyro_axis_converts(model_rate, axis, expected_icd):
    """All three axes, not just pitch.

    Pitch was always correct because +y is the one component the old reflection
    and the true rotation agree on — which is exactly why the defect survived:
    the only axis anyone consumed was the only axis that was right.

    ROLL IS THE LATENT ONE. Nothing reads `gyro[0]` today, so an inverted roll
    rate is silent until lean-to-steer is wired up, at which point it presents
    as a steering control inversion on a rideable vehicle. Pinned here so it
    cannot be reintroduced by a future "simplification" back to a sign triple.
    """
    got = R_MODEL_TO_ICD() @ np.array(model_rate)
    assert np.allclose(got, expected_icd, atol=1e-12), (
        f"{axis}: model {model_rate} -> ICD {tuple(got)}, expected {expected_icd}"
    )


def test_the_map_survives_a_round_trip():
    """Rᵀ·R = I on arbitrary vectors — the conversion loses nothing, so a
    consumer that needs to go back to model axes can."""
    R = R_MODEL_TO_ICD()
    rng = np.random.default_rng(0)
    v = rng.normal(size=(64, 3))
    assert np.allclose((R.T @ (R @ v.T)).T, v, atol=1e-12)
