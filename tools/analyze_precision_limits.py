#!/usr/bin/env python3
"""Quantify geometry, observability, and winch error limits without moving hardware.

The report deliberately separates deterministic CAD quantities from measured
legacy residuals and from illustrative rope-diameter scenarios.  It imports the
same geometry/control module used by the local Y42 debugger, so regressions in
workspace size or cable endpoints are visible immediately.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import statistics

import y42_pc_direct_web as web


ROOT = Path(__file__).resolve().parent


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    index = (len(ordered) - 1) * fraction
    lower = int(math.floor(index))
    upper = int(math.ceil(index))
    if lower == upper:
        return ordered[lower]
    weight = index - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def square_inverse(matrix: list[list[float]]) -> list[list[float]] | None:
    columns: list[list[float]] = []
    for column in range(len(matrix)):
        basis = [0.0] * len(matrix)
        basis[column] = 1.0
        solved = web._solve_square_linear_system(matrix, basis)
        if solved is None:
            return None
        columns.append(solved)
    return [[columns[column][row] for column in range(len(matrix))] for row in range(len(matrix))]


def xy_noise_gain(x: float, y: float, active: tuple[int, ...]) -> float:
    lengths = web.planar_platform_cable_lengths(x, y, 0.0)
    evaluated = web._planar_pose_residual_jacobian(x, y, 0.0, lengths, web.calibration_data())
    if evaluated is None:
        return math.inf
    full = evaluated[1]
    matrix = [[full[index][0], full[index][1]] for index in active]
    if len(active) == 3:
        matrix = [[full[index][0], full[index][1], full[index][2]] for index in active]
    inverse = square_inverse(matrix)
    if inverse is None:
        return math.inf
    # With independent 1 mm RMS cable-length errors, the first two inverse rows
    # map cable noise into X/Y.  Their Frobenius norm is RMS XY amplification.
    return math.sqrt(sum(value * value for row in inverse[:2] for value in row))


def gain_summary(
    active: tuple[int, ...],
    grid: int = 41,
    bounds: tuple[float, float, float, float] | None = None,
) -> dict:
    x0, y0, x1, y1 = bounds or (0.0, 0.0, web.WORKSPACE_WIDTH_MM, web.WORKSPACE_HEIGHT_MM)
    values: list[float] = []
    for row in range(grid):
        y = y0 + (y1 - y0) * row / (grid - 1)
        for column in range(grid):
            x = x0 + (x1 - x0) * column / (grid - 1)
            gain = xy_noise_gain(x, y, active)
            if math.isfinite(gain):
                values.append(gain)
    return {
        "motors": [index + 1 for index in active],
        "median_mm_per_mm_rms": percentile(values, 0.50),
        "p95_mm_per_mm_rms": percentile(values, 0.95),
        "max_mm_per_mm_rms": max(values),
    }


def main() -> int:
    calibration = web.calibration_data()
    motor_x, motor_y = web.motor_center_coordinates(calibration)
    exit_x, exit_y = web.cable_exit_coordinates(calibration)
    attachment_x, attachment_y = web.ball_attachment_offsets(calibration)

    cable_samples = [[] for _ in range(4)]
    height_samples = [[] for _ in range(4)]
    grid = 101
    for row in range(grid):
        y = web.WORKSPACE_HEIGHT_MM * row / (grid - 1)
        for column in range(grid):
            x = web.WORKSPACE_WIDTH_MM * column / (grid - 1)
            lengths = web.planar_platform_cable_lengths(x, y, 0.0, calibration)
            ambiguity = web.spool_height_length_uncertainty_mm(x, y, calibration=calibration)
            for index in range(4):
                cable_samples[index].append(lengths[index])
                height_samples[index].append(ambiguity[index])

    position_pairs = [gain_summary((a, b)) for a in range(4) for b in range(a + 1, 4)]
    position_triples = [gain_summary(tuple(index for index in range(4) if index != passive)) for passive in range(4)]
    inner_bounds = (20.0, 20.0, web.WORKSPACE_WIDTH_MM - 20.0, web.WORKSPACE_HEIGHT_MM - 20.0)
    inner_pairs = [gain_summary((a, b), bounds=inner_bounds) for a in range(4) for b in range(a + 1, 4)]
    inner_triples = [
        gain_summary(tuple(index for index in range(4) if index != passive), bounds=inner_bounds)
        for passive in range(4)
    ]
    best_pair = min(position_pairs, key=lambda item: item["max_mm_per_mm_rms"])
    best_triple = min(position_triples, key=lambda item: item["max_mm_per_mm_rms"])

    old_samples = [
        [442.1666666667, 73.0333333333, -443.3666666667, 166.0333333333],
        [-80.9, -453.5666666667, -212.2, 430.1],
        [-355.9, 102.3, 349.1666666667, 165.3],
        [-105.3, 355.0, -179.1, -352.1666666667],
    ]
    old_fit, old_fit_reason = web.compute_calibration(
        old_samples, web.WORKSPACE_WIDTH_MM, web.WORKSPACE_HEIGHT_MM, web.DEFAULT_BALL_DIAMETER_MM
    )
    fitted_slopes = old_fit.get("slope", []) if old_fit else []
    implied_radii = [180.0 / (math.pi * abs(value)) for value in fitted_slopes if abs(value) > 1e-9]

    cable_ranges = []
    for index, values in enumerate(cable_samples):
        span = max(values) - min(values)
        cable_ranges.append(
            {
                "motor": index + 1,
                "minimum_mm": min(values),
                "maximum_mm": max(values),
                "travel_mm": span,
                "turns_at_r12": span / (2.0 * math.pi * web.SPOOL_WINDING_RADIUS_MM),
            }
        )

    center_height = web.spool_height_length_uncertainty_mm(
        web.MECHANISM_CENTER_X_MM, web.MECHANISM_CENTER_Y_MM, calibration=calibration
    )
    report = {
        "geometry": {
            "workspace_mm": [web.WORKSPACE_WIDTH_MM, web.WORKSPACE_HEIGHT_MM],
            "mechanism_center_mm": [web.MECHANISM_CENTER_X_MM, web.MECHANISM_CENTER_Y_MM],
            "spool_axes_mm": [list(pair) for pair in zip(motor_x, motor_y)],
            "nominal_drum_reference_mm": [list(pair) for pair in zip(exit_x, exit_y)],
            "ball_attachment_offsets_mm": [list(pair) for pair in zip(attachment_x, attachment_y)],
            "spool_winding_radius_mm": web.SPOOL_WINDING_RADIUS_MM,
            "spool_winding_z_range_mm": [web.SPOOL_WINDING_Z_MIN_MM, web.SPOOL_WINDING_Z_MAX_MM],
        },
        "nominal_zero_yaw_cable_ranges": cable_ranges,
        "unknown_exit_height_only": {
            "center_each_cable_mm": center_height,
            "workspace_max_each_cable_mm": [max(values) for values in height_samples],
            "workspace_max_any_cable_mm": max(max(values) for values in height_samples),
        },
        "layer_scenarios": [
            {
                "rope_diameter_mm": diameter,
                "payout_error_per_turn_mm": 2.0 * math.pi * diameter,
            }
            for diameter in (1.0, 1.5, 2.0)
        ],
        "ideal_spool": {
            "degrees_per_mm_at_r12": 180.0 / (math.pi * web.SPOOL_WINDING_RADIUS_MM),
            "helix_note": "Use sqrt((2*pi*r)^2 + pitch^2) per revolution when sizing a grooved drum.",
        },
        "legacy_four_corner_diagnostic": {
            "fit_ok": old_fit is not None,
            "reason": old_fit_reason,
            "slope_deg_per_mm": fitted_slopes,
            "implied_radius_mm": implied_radii,
        },
        "linearized_xy_noise_amplification": {
            "definition": "RMS XY error for independent 1 mm RMS errors on each position-controlled cable",
            "best_2_plus_2": best_pair,
            "best_3_plus_1": best_triple,
            "inner_20mm_best_2_plus_2": min(inner_pairs, key=lambda item: item["max_mm_per_mm_rms"]),
            "inner_20mm_best_3_plus_1": min(inner_triples, key=lambda item: item["max_mm_per_mm_rms"]),
            "all_pairs": position_pairs,
            "all_triples": position_triples,
        },
        "measured_legacy_residuals_mm": {
            "physically_bounded_self_fit_p95": 10.27,
            "raw_passive_cable_p95_range": [31.31, 37.36],
            "orientation_endpoint_diameter": 2.0 * math.hypot(web.BALL_ATTACHMENT_HALF_X_MM, web.BALL_ATTACHMENT_HALF_Y_MM),
        },
    }
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
