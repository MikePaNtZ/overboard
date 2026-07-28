"""Disturbance-rejection ENVELOPE for the closed-loop controller.

GitHub #24, AC2: the impulse gate (`tests/test_closed_loop.py`) only proves
the controller survives one fixed magnitude, `NOMINAL_IMPULSE_NS`. That is a
single point on a curve, not the curve -- it says nothing about how much
margin is left, or where the controller actually stops coping.

WHY A GRID SWEEP, NOT A BISECTION TO THE EXACT CROSSING
---------------------------------------------------------
`nose_strike` is a real bumper-vs-ground contact (see `impulse_response.py`),
decided against a geometric collision boundary. Close to the strike angle
(18.57 deg, `nose_strike_angle_deg`) the boolean outcome is NOT monotonic in
impulse magnitude -- measured directly: at 20 N*s steps from 260 to 320 N*s
the closed-loop driverless board struck at 280 and 300 N*s but *not* at 290
or 320 (`peak_abs_pitch_deg` sits within ~1 deg of the strike angle across
that whole band, so which side of it a run lands on is sensitive to exactly
where the trajectory is at the moment of closest approach). A bisection
search would converge on one of those knife-edge crossings and report a
number that moves with any small upstream change (gains, estimator state,
even float rounding) without the controller's actual margin having changed.

A coarse grid sweep instead reports a genuine ENVELOPE: every sampled
magnitude survived or it did not, in order, and the "recovery boundary" is
the last grid point *before* the first sampled failure -- a number with a
full grid step of headroom, not a coin-flip on the exact contact frame.
"""

from __future__ import annotations

from dataclasses import dataclass

from .impulse_response import ImpulseParams, run


@dataclass(frozen=True)
class EnvelopePoint:
    """One sampled magnitude and what happened."""

    magnitude_ns: float
    nose_strike: bool
    peak_abs_pitch_deg: float


@dataclass(frozen=True)
class EnvelopeResult:
    """The swept envelope, points in ascending magnitude order."""

    points: tuple[EnvelopePoint, ...]

    @property
    def first_failure_ns(self) -> float | None:
        """Smallest swept magnitude that struck, or None if nothing did."""
        return next((p.magnitude_ns for p in self.points if p.nose_strike), None)

    @property
    def recovery_boundary_ns(self) -> float:
        """Largest swept magnitude confirmed survivable.

        One grid step below the first sampled failure, or the top of the
        sweep if nothing in it failed. Raises rather than guessing if the
        sweep cannot show a boundary at all (nothing survived, or the
        smallest magnitude sampled already struck).
        """
        failure = self.first_failure_ns
        survived = [p.magnitude_ns for p in self.points if not p.nose_strike]
        if failure is None:
            if not survived:
                raise ValueError("no sampled magnitude survived -- sweep starts too high")
            return max(survived)
        below = [m for m in survived if m < failure]
        if not below:
            raise ValueError(
                f"the smallest sampled magnitude ({self.points[0].magnitude_ns} N*s) "
                "already struck -- sweep starts too high to show a boundary"
            )
        return max(below)

    def to_json_dict(self) -> dict:
        return {
            "points": [
                {
                    "magnitude_ns": p.magnitude_ns,
                    "nose_strike": p.nose_strike,
                    "peak_abs_pitch_deg": p.peak_abs_pitch_deg,
                }
                for p in self.points
            ],
            "first_failure_ns": self.first_failure_ns,
            "recovery_boundary_ns": self.recovery_boundary_ns,
        }


def sweep_closed_loop(model, controller_factory, magnitudes_ns, sim_seconds=8.0) -> EnvelopeResult:
    """Run the closed-loop impulse scenario at each magnitude in
    `magnitudes_ns` (ascending) and report what happened at each.

    `controller_factory` is called once per magnitude and used as a context
    manager -- the same "fresh controller per run" pattern
    `tests/test_closed_loop.py` uses, so no run carries hidden state between
    magnitudes.
    """
    points = []
    for magnitude in magnitudes_ns:
        with controller_factory() as controller:
            result = run(
                ImpulseParams(magnitude_ns=magnitude, sim_seconds=sim_seconds),
                model=model,
                controller=controller,
            )
        points.append(
            EnvelopePoint(
                magnitude_ns=magnitude,
                nose_strike=result.metrics.nose_strike,
                peak_abs_pitch_deg=result.metrics.peak_abs_pitch_deg,
            )
        )
    return EnvelopeResult(tuple(points))
