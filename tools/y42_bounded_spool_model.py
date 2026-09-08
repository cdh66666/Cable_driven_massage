"""Pure, offline-safe affine payout fitting; no hardware, files or global state.

The caller owns the physical bounds and any acceptance tolerances.  In
particular, a nominal CAD radius is not automatically a measured hard bound.
The fit is NOT clipped into the allowed interval: an out-of-bounds fit is
rejected and its diagnostics retained.  Four samples are never forced through
a piecewise interpolant.  ``accepted`` only covers the explicitly supplied
checks; it is not a statement of physical pose accuracy or tension continuity.
"""

from __future__ import annotations

import math
from collections.abc import Iterable


def slope_limits_from_radius(radius_min_mm: float, radius_max_mm: float) -> tuple[float, float]:
    """Return absolute deg/mm bounds for direct-drive ideal cylindrical payout.

    Additional transmission ratios, changing wrap path and radius uncertainty
    must be accounted for by the caller; this function does not infer them.
    """
    lower, upper = float(radius_min_mm), float(radius_max_mm)
    if not all(math.isfinite(value) for value in (lower, upper)) or not 0.0 < lower <= upper:
        raise ValueError("radius bounds must be finite, positive and ordered")
    return 180.0 / (math.pi * upper), 180.0 / (math.pi * lower)


def _finite_values(values: Iterable[float], name: str) -> list[float]:
    result = [float(value) for value in values]
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} contains non-finite values")
    return result


def fit_bounded_affine(
    distances_mm: Iterable[float],
    positions_deg: Iterable[float],
    *,
    direction: int,
    abs_slope_bounds_deg_per_mm: tuple[float, float],
    min_distance_span_mm: float = 0.0,
    max_abs_residual_mm: float | None = None,
    min_r2: float | None = None,
    runtime_knot_count: int = 4,
) -> dict:
    """Fit position = intercept + slope * length with explicit safety checks.

    Samples may come from any independently specified geometric-length law.
    Duplicates remain separate observations, so conflicting readings at the
    same length cannot disappear by coalescing.  The runtime knots are synthetic
    samples of one affine law, not additional measured calibration points.
    Residuals in mm are encoder residual / fitted gain, not measured XY error.
    Invalid inputs raise ValueError; failed physical/quality checks return an
    ``accepted=False`` result with no executable runtime curve.
    """
    xs = _finite_values(distances_mm, "distances")
    ys = _finite_values(positions_deg, "positions")
    if len(xs) != len(ys) or len(xs) < 3:
        raise ValueError("at least three paired observations are required")
    if any(value < 0.0 for value in xs):
        raise ValueError("geometric lengths cannot be negative")
    if direction not in (-1, 1) or isinstance(direction, bool):
        raise ValueError("direction must be -1 or +1")
    bounds = _finite_values(abs_slope_bounds_deg_per_mm, "slope bounds")
    if len(bounds) != 2 or not 0.0 < bounds[0] <= bounds[1]:
        raise ValueError("absolute slope bounds must be positive and ordered")
    minimum_span = float(min_distance_span_mm)
    if not math.isfinite(minimum_span) or minimum_span < 0.0:
        raise ValueError("minimum distance span must be finite and nonnegative")
    if max_abs_residual_mm is not None:
        max_abs_residual_mm = float(max_abs_residual_mm)
        if not math.isfinite(max_abs_residual_mm) or max_abs_residual_mm < 0.0:
            raise ValueError("maximum residual must be finite and nonnegative")
    if min_r2 is not None:
        min_r2 = float(min_r2)
        if not math.isfinite(min_r2) or not 0.0 <= min_r2 <= 1.0:
            raise ValueError("minimum R2 must lie between zero and one")
    if not isinstance(runtime_knot_count, int) or isinstance(runtime_knot_count, bool) or runtime_knot_count < 2:
        raise ValueError("runtime knot count must be an integer >= 2")

    low, high = min(xs), max(xs)
    span = high - low
    if span <= 0.0 or span < minimum_span:
        raise ValueError("insufficient geometric length span")
    x_mean, y_mean = math.fsum(xs) / len(xs), math.fsum(ys) / len(ys)
    denominator = math.fsum((value - x_mean) ** 2 for value in xs)
    slope = math.fsum((x - x_mean) * (y - y_mean) for x, y in zip(xs, ys)) / denominator
    intercept = y_mean - slope * x_mean
    fitted = [intercept + slope * value for value in xs]
    residual_deg = [observed - expected for observed, expected in zip(ys, fitted)]
    ss_residual = math.fsum(value * value for value in residual_deg)
    ss_total = math.fsum((value - y_mean) ** 2 for value in ys)
    r2 = 1.0 - ss_residual / ss_total if ss_total > 0.0 else None
    residual_mm = [value / abs(slope) for value in residual_deg] if slope else None
    max_residual = max(abs(value) for value in residual_mm) if residual_mm is not None else None
    rms_residual = math.sqrt(math.fsum(value * value for value in residual_mm) / len(xs)) if residual_mm is not None else None
    # Small adjacent spans expose near-flat/reversed raw interpolant segments.
    # These are diagnostics, not independently identifiable local spool radii.
    ordered = sorted(zip(xs, ys))
    local_segments = [
        {"length_interval_mm": [a[0], b[0]],
         "slope_deg_per_mm": (b[1] - a[1]) / (b[0] - a[0]),
         "within_supplied_bounds": bounds[0] <= direction * (b[1] - a[1]) / (b[0] - a[0]) <= bounds[1]}
        for a, b in zip(ordered, ordered[1:]) if b[0] > a[0]
    ]
    failures = []
    if not bounds[0] <= direction * slope <= bounds[1]:
        failures.append("fitted_slope_outside_physical_bounds")
    if max_abs_residual_mm is not None and (max_residual is None or max_residual > max_abs_residual_mm):
        failures.append("affine_residual_exceeds_supplied_limit")
    if min_r2 is not None and (r2 is None or r2 < min_r2):
        failures.append("r2_below_supplied_limit")
    knots_x = [low + span * n / (runtime_knot_count - 1) for n in range(runtime_knot_count)]
    return {
        "model_kind": "bounded_affine_v1",
        "accepted": not failures,
        "rejection_reasons": failures,
        "slope_deg_per_mm": slope,
        "intercept_deg": intercept,
        "equivalent_radius_mm": 180.0 / (math.pi * abs(slope)) if slope else None,
        "abs_slope_bounds_deg_per_mm": bounds,
        "direction": direction,
        "raw_distance_samples_mm": xs,
        "raw_position_samples_deg": ys,
        "fitted_position_samples_deg": fitted,
        "sample_count": len(xs),
        "distinct_length_count": len(set(xs)),
        "distance_span_mm": span,
        "r2": r2,
        "residual_deg": residual_deg,
        "residual_mm": residual_mm,
        "max_abs_residual_mm": max_residual,
        "rms_residual_mm": rms_residual,
        "raw_local_segments": local_segments,
        "quality_limits": {"max_abs_residual_mm": max_abs_residual_mm, "min_r2": min_r2},
        "physical_pose_accuracy_verified": False,
        "runtime_knots_are_synthetic": True,
        "runtime_curve": [[x, intercept + slope * x] for x in knots_x] if not failures else None,
    }


def affine_position_deg(model: dict, distance_mm: float) -> float:
    """Forward mapping; rejected models cannot accidentally drive an axis."""
    if model.get("accepted") is not True:
        raise ValueError("affine model was not accepted")
    distance, slope, intercept = _finite_values(
        [distance_mm, model["slope_deg_per_mm"], model["intercept_deg"]], "affine values"
    )
    return intercept + slope * distance


def affine_length_mm(model: dict, position_deg: float) -> float:
    """Exact inverse of the same signed affine law, without abs/clamping."""
    if model.get("accepted") is not True:
        raise ValueError("affine model was not accepted")
    position, slope, intercept = _finite_values(
        [position_deg, model["slope_deg_per_mm"], model["intercept_deg"]], "affine values"
    )
    if slope == 0.0:
        raise ValueError("zero affine slope")
    return (position - intercept) / slope
