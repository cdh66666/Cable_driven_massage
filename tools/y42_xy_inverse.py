"""Small, dependency-free inverse solver for a fixed-plane XY forward map.

The controller has several motor-space forward maps.  This module deliberately
knows nothing about motors, calibration files, or safety state: callers supply
the forward callable and receive a numerical diagnostic report.  A central
finite-difference Gauss--Newton solve is enough for the two-dimensional fixed
plane problem and keeps the module usable in the offline drawing tools where
NumPy is not installed.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Sequence
from typing import Any


ForwardXY = Callable[[float, float], Sequence[float]]


def _coerce_finite_vector(value: object, expected_length: int | None = None) -> list[float] | None:
    """Convert one vector to finite floats without accepting text as a vector."""
    if isinstance(value, (str, bytes, bytearray)):
        return None
    try:
        values = [float(item) for item in value]  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return None
    if expected_length is not None and len(values) != expected_length:
        return None
    return values if all(math.isfinite(item) for item in values) else None


def _parse_xy(value: object, *, default: tuple[float, float] | None = None) -> tuple[float, float] | None:
    if value is None:
        return default
    if isinstance(value, dict):
        raw = (value.get("x_mm", value.get("x")), value.get("y_mm", value.get("y")))
    else:
        raw = value
    values = _coerce_finite_vector(raw, 2)
    return (values[0], values[1]) if values is not None else None


def _parse_bounds(value: object) -> tuple[tuple[float, float], tuple[float, float]] | None:
    if value is None:
        return None
    if isinstance(value, (str, bytes, bytearray)):
        return None
    try:
        rows = list(value)  # type: ignore[arg-type]
    except TypeError:
        return None
    if len(rows) != 2:
        return None
    parsed: list[tuple[float, float]] = []
    for row in rows:
        values = _coerce_finite_vector(row, 2)
        if values is None or values[0] > values[1]:
            return None
        parsed.append((values[0], values[1]))
    return parsed[0], parsed[1]


def _clamp_xy(
    x: float,
    y: float,
    bounds: tuple[tuple[float, float], tuple[float, float]] | None,
) -> tuple[float, float]:
    if bounds is None:
        return x, y
    return (
        max(bounds[0][0], min(bounds[0][1], x)),
        max(bounds[1][0], min(bounds[1][1], y)),
    )


def _empty_report(
    target: list[float] | None,
    *,
    reason: str,
    termination: str | None = None,
    x: float | None = None,
    y: float | None = None,
    predicted: list[float] | None = None,
    residuals: list[float] | None = None,
    iterations: int = 0,
    evaluations: int = 0,
    step_norm: float | None = None,
    determinant: float | None = None,
    jacobian: list[list[float]] | None = None,
) -> dict[str, Any]:
    rms = None
    maximum = None
    if residuals is not None and residuals and all(math.isfinite(value) for value in residuals):
        rms = math.sqrt(sum(value * value for value in residuals) / len(residuals))
        maximum = max(abs(value) for value in residuals)
    finite = bool(
        target is not None
        and all(math.isfinite(value) for value in target)
        and predicted is not None
        and all(math.isfinite(value) for value in predicted)
        and (residuals is None or all(math.isfinite(value) for value in residuals))
        and (x is None or math.isfinite(x))
        and (y is None or math.isfinite(y))
        and (step_norm is None or math.isfinite(step_norm))
        and (determinant is None or math.isfinite(determinant))
    )
    xy = [x, y] if x is not None and y is not None else None
    return {
        "ok": False,
        "valid": False,
        "finite": finite,
        "converged": False,
        "x": x,
        "y": y,
        "xy": xy,
        "xy_mm": xy,
        "target": list(target) if target is not None else None,
        "predicted": list(predicted) if predicted is not None else None,
        "predicted_targets": list(predicted) if predicted is not None else None,
        "residuals": list(residuals) if residuals is not None else None,
        "residual_rms": rms,
        "residual_max": maximum,
        "iterations": int(iterations),
        "evaluations": int(evaluations),
        "step_norm": step_norm,
        "normal_determinant": determinant,
        "jacobian": jacobian,
        "termination": termination or reason,
        "reason": reason,
    }


def solve_fixed_plane_xy(
    target: Sequence[float] | object,
    forward: ForwardXY | object,
    *,
    initial_xy: Sequence[float] | object | None = None,
    bounds: Sequence[Sequence[float]] | object | None = None,
    max_iterations: int = 60,
    finite_difference_step: float = 0.05,
    step_tolerance: float = 1.0e-5,
    residual_tolerance: float = 1.0e-4,
    max_step: float = 15.0,
    determinant_tolerance: float = 1.0e-12,
) -> dict[str, Any]:
    """Solve ``forward(x, y) ~= target`` on a fixed plane.

    The returned mapping is intentionally a report instead of a bare pair.  A
    result is numerically ``ok`` only when every value is finite, the solver
    reached a declared termination condition, and the final maximum residual is
    within ``residual_tolerance``.  A small step with a large residual is
    reported as stalled, never as a successful solve.

    ``forward`` may return any finite vector with the same length as ``target``.
    It is evaluated at the exact candidate XY values supplied by the caller;
    this lets a controller diagnose the same quantised or compensated forward
    map used for its motor target generation.
    """
    raw_target = _coerce_finite_vector(target)
    if raw_target is None or len(raw_target) < 2:
        return _empty_report(
            raw_target,
            reason="target must contain at least two finite values",
            termination="invalid_target",
        )
    if not callable(forward):
        return _empty_report(raw_target, reason="forward must be callable", termination="invalid_forward")

    parsed_bounds = _parse_bounds(bounds)
    if bounds is not None and parsed_bounds is None:
        return _empty_report(raw_target, reason="bounds must contain two finite ordered pairs", termination="invalid_bounds")

    try:
        iterations_limit = int(max_iterations)
        finite_step = float(finite_difference_step)
        step_limit = float(step_tolerance)
        residual_limit = float(residual_tolerance)
        maximum_step = float(max_step)
        det_limit = float(determinant_tolerance)
    except (TypeError, ValueError):
        return _empty_report(raw_target, reason="solver options are not finite numbers", termination="invalid_options")
    if (
        iterations_limit < 1
        or iterations_limit != max_iterations
        or not math.isfinite(finite_step)
        or finite_step <= 0.0
        or not math.isfinite(step_limit)
        or step_limit <= 0.0
        or not math.isfinite(residual_limit)
        or residual_limit < 0.0
        or not math.isfinite(maximum_step)
        or maximum_step <= 0.0
        or not math.isfinite(det_limit)
        or det_limit <= 0.0
    ):
        return _empty_report(raw_target, reason="solver options are outside finite positive ranges", termination="invalid_options")

    parsed_initial = _parse_xy(initial_xy, default=(0.0, 0.0))
    if parsed_initial is None:
        return _empty_report(raw_target, reason="initial_xy must contain two finite values", termination="invalid_initial_xy")
    x, y = _clamp_xy(parsed_initial[0], parsed_initial[1], parsed_bounds)
    if not math.isfinite(x) or not math.isfinite(y):
        return _empty_report(raw_target, reason="initial_xy is not finite", termination="invalid_initial_xy")

    evaluations = 0
    forward_error: str | None = None

    def evaluate(query_x: float, query_y: float) -> list[float] | None:
        nonlocal evaluations, forward_error
        if not math.isfinite(query_x) or not math.isfinite(query_y):
            forward_error = "forward query is not finite"
            return None
        try:
            output = forward(float(query_x), float(query_y))  # type: ignore[misc]
        except Exception as exc:  # Diagnostic boundary: caller must see a clean failure report.
            forward_error = f"forward raised {type(exc).__name__}: {exc}"
            return None
        values = _coerce_finite_vector(output, len(raw_target))
        evaluations += 1
        if values is None:
            forward_error = "forward returned a non-finite or wrong-length vector"
            return None
        return values

    predicted = evaluate(x, y)
    if predicted is None:
        return _empty_report(
            raw_target,
            reason=forward_error or "forward evaluation failed",
            termination="forward_error",
            x=x,
            y=y,
            evaluations=evaluations,
        )

    residuals = [value - desired for value, desired in zip(predicted, raw_target)]
    if not all(math.isfinite(value) for value in residuals):
        return _empty_report(
            raw_target,
            reason="initial residual is not finite",
            termination="nonfinite_residual",
            x=x,
            y=y,
            predicted=predicted,
            residuals=residuals,
            evaluations=evaluations,
        )

    def report(
        *,
        ok: bool,
        converged: bool,
        termination: str,
        iterations: int,
        last_step: float | None,
        determinant: float | None,
        jacobian: list[list[float]] | None,
    ) -> dict[str, Any]:
        result = _empty_report(
            raw_target,
            reason=termination,
            termination=termination,
            x=x,
            y=y,
            predicted=predicted,
            residuals=residuals,
            iterations=iterations,
            evaluations=evaluations,
            step_norm=last_step,
            determinant=determinant,
            jacobian=jacobian,
        )
        result["ok"] = bool(ok)
        result["valid"] = bool(ok)
        result["converged"] = bool(converged)
        return result

    def residual_cost(values: list[float]) -> float:
        return sum(value * value for value in values)

    initial_cost = residual_cost(residuals)
    if max(abs(value) for value in residuals) <= residual_limit:
        return report(
            ok=True,
            converged=True,
            termination="residual_tolerance",
            iterations=0,
            last_step=0.0,
            determinant=None,
            jacobian=None,
        )

    last_step: float | None = None
    last_determinant: float | None = None
    last_jacobian: list[list[float]] | None = None
    for iteration in range(1, iterations_limit + 1):
        # Central differences are preferred.  Larger probes are attempted when
        # the caller's map is angle-quantised and a small probe lands on the
        # same output on both sides.
        jacobian: list[list[float]] = [[0.0, 0.0] for _ in raw_target]
        for column in range(2):
            derivative_values: list[float] | None = None
            for multiplier in (1.0, 2.0, 4.0, 8.0, 16.0):
                probe = finite_step * multiplier
                if column == 0:
                    minus_xy = _clamp_xy(x - probe, y, parsed_bounds)
                    plus_xy = _clamp_xy(x + probe, y, parsed_bounds)
                else:
                    minus_xy = _clamp_xy(x, y - probe, parsed_bounds)
                    plus_xy = _clamp_xy(x, y + probe, parsed_bounds)
                span = (plus_xy[column] - minus_xy[column])
                if abs(span) <= 1.0e-12:
                    continue
                minus = evaluate(minus_xy[0], minus_xy[1])
                plus = evaluate(plus_xy[0], plus_xy[1])
                if minus is None or plus is None:
                    return report(
                        ok=False,
                        converged=False,
                        termination="forward_error",
                        iterations=iteration - 1,
                        last_step=last_step,
                        determinant=last_determinant,
                        jacobian=last_jacobian,
                    )
                derivative_values = [(right - left) / span for left, right in zip(minus, plus)]
                if all(math.isfinite(value) for value in derivative_values):
                    if max(abs(value) for value in derivative_values) > 1.0e-12:
                        break
            if derivative_values is None or not all(math.isfinite(value) for value in derivative_values):
                return report(
                    ok=False,
                    converged=False,
                    termination="nonfinite_jacobian",
                    iterations=iteration - 1,
                    last_step=last_step,
                    determinant=last_determinant,
                    jacobian=last_jacobian,
                )
            for row, value in enumerate(derivative_values):
                jacobian[row][column] = value

        a = sum(row[0] * row[0] for row in jacobian)
        b = sum(row[0] * row[1] for row in jacobian)
        c = sum(row[1] * row[1] for row in jacobian)
        determinant = a * c - b * b
        last_determinant = determinant
        last_jacobian = [row[:] for row in jacobian]
        normal_scale = max(1.0, a, c)
        if not math.isfinite(determinant) or determinant <= det_limit * normal_scale * normal_scale:
            return report(
                ok=False,
                converged=False,
                termination="singular_jacobian",
                iterations=iteration - 1,
                last_step=last_step,
                determinant=determinant,
                jacobian=jacobian,
            )
        gradient_x = sum(row[0] * value for row, value in zip(jacobian, residuals))
        gradient_y = sum(row[1] * value for row, value in zip(jacobian, residuals))

        # Try an undamped Gauss--Newton step, followed by scaled diagonal
        # damping.  The line search makes quantised maps and hard bounds safe.
        step_candidates: list[tuple[float, float]] = []
        for damping in (0.0, 1.0e-8, 1.0e-6, 1.0e-4, 1.0e-2, 1.0, 100.0):
            diagonal_x = a + damping * max(1.0, a)
            diagonal_y = c + damping * max(1.0, c)
            damped_determinant = diagonal_x * diagonal_y - b * b
            if damped_determinant <= det_limit * max(1.0, diagonal_x, diagonal_y) ** 2:
                continue
            step_x = (-diagonal_y * gradient_x + b * gradient_y) / damped_determinant
            step_y = (b * gradient_x - diagonal_x * gradient_y) / damped_determinant
            if math.isfinite(step_x) and math.isfinite(step_y):
                step_candidates.append((step_x, step_y))
        accepted = False
        old_cost = residual_cost(residuals)
        for step_x, step_y in step_candidates:
            norm = math.hypot(step_x, step_y)
            if norm > maximum_step:
                step_x *= maximum_step / norm
                step_y *= maximum_step / norm
                norm = maximum_step
            if norm <= step_tolerance:
                last_step = norm
                continue
            for fraction in (1.0, 0.5, 0.25, 0.125, 0.0625, 0.03125):
                candidate_x, candidate_y = _clamp_xy(
                    x + fraction * step_x,
                    y + fraction * step_y,
                    parsed_bounds,
                )
                if math.hypot(candidate_x - x, candidate_y - y) <= 1.0e-14:
                    continue
                candidate_predicted = evaluate(candidate_x, candidate_y)
                if candidate_predicted is None:
                    return report(
                        ok=False,
                        converged=False,
                        termination="forward_error",
                        iterations=iteration - 1,
                        last_step=last_step,
                        determinant=determinant,
                        jacobian=jacobian,
                    )
                candidate_residuals = [value - desired for value, desired in zip(candidate_predicted, raw_target)]
                if not all(math.isfinite(value) for value in candidate_residuals):
                    return report(
                        ok=False,
                        converged=False,
                        termination="nonfinite_residual",
                        iterations=iteration - 1,
                        last_step=last_step,
                        determinant=determinant,
                        jacobian=jacobian,
                    )
                candidate_cost = residual_cost(candidate_residuals)
                if candidate_cost < old_cost - max(1.0e-14, old_cost * 1.0e-12):
                    x, y = candidate_x, candidate_y
                    predicted = candidate_predicted
                    residuals = candidate_residuals
                    last_step = math.hypot(fraction * step_x, fraction * step_y)
                    accepted = True
                    break
            if accepted:
                break

        if max(abs(value) for value in residuals) <= residual_limit:
            return report(
                ok=True,
                converged=True,
                termination="residual_tolerance",
                iterations=iteration,
                last_step=last_step,
                determinant=determinant,
                jacobian=jacobian,
            )
        if not accepted:
            # A tiny attempted step only counts as convergence when the residual
            # is already within the declared tolerance.  Otherwise this is an
            # honest stalled/non-converged diagnostic.
            if last_step is not None and last_step <= step_tolerance:
                return report(
                    ok=False,
                    converged=False,
                    termination="stalled_residual",
                    iterations=iteration - 1,
                    last_step=last_step,
                    determinant=determinant,
                    jacobian=jacobian,
                )
            return report(
                ok=False,
                converged=False,
                termination="no_improving_step",
                iterations=iteration - 1,
                last_step=last_step,
                determinant=determinant,
                jacobian=jacobian,
            )

    return report(
        ok=False,
        converged=False,
        termination="max_iterations",
        iterations=iterations_limit,
        last_step=last_step,
        determinant=last_determinant,
        jacobian=last_jacobian,
    )


# Descriptive aliases keep the generic module easy to discover without making
# the controller depend on an implementation-specific name.
fixed_plane_xy_least_squares = solve_fixed_plane_xy
least_squares_xy_inverse = solve_fixed_plane_xy


__all__ = [
    "ForwardXY",
    "solve_fixed_plane_xy",
    "fixed_plane_xy_least_squares",
    "least_squares_xy_inverse",
]
