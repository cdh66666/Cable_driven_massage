#!/usr/bin/env python3
"""Test an encoder/current-only monotonic spool-map fit on rotated 3+1 runs."""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix

import y42_pc_direct_web as web
from analyze_motor_only_calibration import evenly_spaced, latest_completed_snake_samples, percentile


KNOT_COUNT = 9
SEGMENT_COUNT = KNOT_COUNT - 1
GLOBAL_COUNT = 4 + 4 * SEGMENT_COUNT + 4 + 4


def prepare(samples: list[dict]) -> tuple[list[dict], list[dict], list[int]]:
    groups: dict[int, list[dict]] = {}
    for item in samples:
        role = int(item.get("passive_motor", 0) or 0)
        if role in (1, 2, 3, 4):
            groups.setdefault(role, []).append(item)
    training: list[dict] = []
    validation: list[dict] = []
    for role in sorted(groups):
        chosen = [dict(item) for item in evenly_spaced(groups[role], 100)]
        q = np.asarray([item["positions_deg"] for item in chosen], dtype=float)
        derivative = np.gradient(q, axis=0)
        direction = np.where(np.abs(derivative) >= 0.15, np.sign(derivative), 0.0)
        for index, item in enumerate(chosen):
            item["_q_direction"] = direction[index].tolist()
            (validation if index % 5 == 0 else training).append(item)
    return training, validation, sorted(groups)


def fit(samples: list[dict], calibration: dict) -> dict:
    training, validation, roles = prepare(samples)
    q = np.asarray([item["positions_deg"] for item in training], dtype=float)
    direction = np.asarray([item["_q_direction"] for item in training], dtype=float)
    current = np.asarray(
        [[float(value) if isinstance(value, (int, float)) else np.nan for value in item["phase_current_ma"]] for item in training],
        dtype=float,
    )
    current_reference = np.nanmedian(np.abs(current), axis=0)
    current = np.where(np.isfinite(current), np.abs(current), current_reference[None, :])
    passive = np.asarray([int(item["passive_motor"]) - 1 for item in training], dtype=int)
    count = len(training)
    slopes = np.asarray(calibration["slope"], dtype=float)
    intercepts = np.asarray(calibration["intercept"], dtype=float)
    signed_direction = np.sign(1.0 / slopes)
    q_knots = np.asarray(
        [np.linspace(np.min(q[:, motor]) - 0.5, np.max(q[:, motor]) + 0.5, KNOT_COUNT) for motor in range(4)]
    )
    initial_segment_slope = np.abs(1.0 / slopes)
    base_initial = np.asarray([(q_knots[motor, 0] - intercepts[motor]) / slopes[motor] for motor in range(4)])
    log_slope_initial = np.log(np.repeat(initial_segment_slope[:, None], SEGMENT_COUNT, axis=1))
    pose_initial = np.asarray(
        [[float(item["target_xy"][0]), float(item["target_xy"][1]), 0.0] for item in training], dtype=float
    )
    initial = np.concatenate((base_initial, log_slope_initial.ravel(), np.zeros(4), np.zeros(4), pose_initial.ravel()))
    exit_x, exit_y = (np.asarray(values)[None, :] for values in web.cable_exit_coordinates(calibration))
    body_x, body_y = (np.asarray(values)[None, :] for values in web.ball_attachment_offsets(calibration))

    def unpack(vector: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        return (
            vector[:4],
            np.exp(vector[4:36].reshape(4, SEGMENT_COUNT)),
            vector[36:40],
            vector[40:44],
            vector[GLOBAL_COUNT:].reshape(count, 3),
        )

    def length_knots(vector: np.ndarray) -> np.ndarray:
        base, segment_slope, _, _, _ = unpack(vector)
        result = np.empty((4, KNOT_COUNT), dtype=float)
        result[:, 0] = base
        for motor in range(4):
            increments = signed_direction[motor] * segment_slope[motor] * np.diff(q_knots[motor])
            result[motor, 1:] = base[motor] + np.cumsum(increments)
        return result

    def measured_lengths(
        vector: np.ndarray,
        query_q: np.ndarray,
        query_current: np.ndarray,
        query_direction: np.ndarray,
    ) -> np.ndarray:
        knots = length_knots(vector)
        _, _, compliance, hysteresis, _ = unpack(vector)
        result = np.empty_like(query_q, dtype=float)
        for motor in range(4):
            result[:, motor] = np.interp(query_q[:, motor], q_knots[motor], knots[motor])
        result += (query_current - current_reference[None, :]) / 1000.0 * compliance[None, :]
        result += query_direction * hysteresis[None, :]
        return result

    def raw_errors(vector: np.ndarray) -> np.ndarray:
        _, _, _, _, poses = unpack(vector)
        cosine = np.cos(poses[:, 2])[:, None]
        sine = np.sin(poses[:, 2])[:, None]
        endpoint_x = poses[:, 0, None] + cosine * body_x - sine * body_y
        endpoint_y = poses[:, 1, None] + sine * body_x + cosine * body_y
        predicted = np.hypot(exit_x - endpoint_x, exit_y - endpoint_y)
        return predicted - measured_lengths(vector, q, current, direction)

    def residual(vector: np.ndarray) -> np.ndarray:
        errors = raw_errors(vector)
        weights = np.full_like(errors, 6.0)
        weights[np.arange(count), passive] = 1.0
        _, segment_slope, compliance, hysteresis, _ = unpack(vector)
        slope_prior = (segment_slope - initial_segment_slope[:, None]) * 20.0
        smoothness = np.diff(segment_slope, axis=1) * 30.0
        knots = length_knots(vector)
        boundary = np.concatenate((np.maximum(0.0, 1.0 - knots), np.maximum(0.0, knots - 220.0))).ravel() * 4.0
        return np.concatenate(
            (
                (errors * weights).ravel(),
                slope_prior.ravel(),
                smoothness.ravel(),
                compliance * 0.1,
                hysteresis * 0.1,
                boundary,
            )
        )

    prior_rows = 4 * SEGMENT_COUNT + 4 * (SEGMENT_COUNT - 1) + 4 + 4 + 2 * 4 * KNOT_COUNT
    rows = count * 4 + prior_rows
    columns = GLOBAL_COUNT + count * 3
    sparsity = lil_matrix((rows, columns), dtype=int)
    for sample in range(count):
        for motor in range(4):
            row = sample * 4 + motor
            sparsity[row, motor] = 1
            sparsity[row, 4 + motor * SEGMENT_COUNT:4 + (motor + 1) * SEGMENT_COUNT] = 1
            sparsity[row, 36 + motor] = 1
            sparsity[row, 40 + motor] = 1
            sparsity[row, GLOBAL_COUNT + sample * 3:GLOBAL_COUNT + sample * 3 + 3] = 1
    # All regularisers depend only on global parameters; an intentionally broad
    # global block keeps the sparse pattern simple and correct.
    sparsity[count * 4:, :GLOBAL_COUNT] = 1
    lower = np.concatenate(
        (
            np.maximum(1.0, base_initial - 80.0),
            np.full(4 * SEGMENT_COUNT, math.log(0.16)),
            np.zeros(4),
            np.full(4, -8.0),
            np.tile(np.asarray([-25.0, -25.0, math.radians(-45.0)]), count),
        )
    )
    upper = np.concatenate(
        (
            np.minimum(230.0, base_initial + 80.0),
            np.full(4 * SEGMENT_COUNT, math.log(0.28)),
            np.full(4, 12.0),
            np.full(4, 8.0),
            np.tile(np.asarray([web.WORKSPACE_WIDTH_MM + 25.0, web.WORKSPACE_HEIGHT_MM + 25.0, math.radians(45.0)]), count),
        )
    )
    solved = least_squares(
        residual,
        initial,
        bounds=(lower, upper),
        jac_sparsity=sparsity.tocsr(),
        x_scale="jac",
        loss="soft_l1",
        f_scale=0.35,
        max_nfev=600,
        ftol=1e-10,
        xtol=1e-10,
        gtol=1e-10,
    )
    knots = length_knots(solved.x)
    _, fitted_slopes, compliance, hysteresis, _ = unpack(solved.x)
    train_error = raw_errors(solved.x).ravel()
    validation_closure: list[float] = []
    validation_passive: list[float] = []
    validation_yaw: list[float] = []
    for item in validation:
        query_q = np.asarray([item["positions_deg"]], dtype=float)
        query_current = np.asarray(
            [[float(value) if isinstance(value, (int, float)) else current_reference[index] for index, value in enumerate(item["phase_current_ma"])]],
            dtype=float,
        )
        query_direction = np.asarray([item["_q_direction"]], dtype=float)
        lengths = measured_lengths(solved.x, query_q, np.abs(query_current), query_direction)[0].tolist()
        initial_pose = [*item["target_xy"], 0.0]
        all_pose = web.solve_planar_platform_pose(lengths, calibration, initial_pose=initial_pose)
        if all_pose is not None:
            validation_closure.append(float(all_pose["closure_rms_mm"]))
            validation_yaw.append(abs(float(all_pose["yaw_deg"])))
        passive_index = int(item["passive_motor"]) - 1
        active = tuple(index for index in range(4) if index != passive_index)
        pose = web.solve_planar_platform_pose(
            lengths,
            calibration,
            active_indices=active,
            initial_pose=all_pose if all_pose is not None else initial_pose,
        )
        if pose is not None and pose["redundant_residuals_mm"]:
            validation_passive.append(abs(float(pose["redundant_residuals_mm"][0])))
    return {
        "success": bool(solved.success),
        "message": solved.message,
        "roles": roles,
        "training_samples": len(training),
        "validation_samples": len(validation),
        "q_knots_deg": q_knots.round(4).tolist(),
        "length_knots_mm": knots.round(4).tolist(),
        "segment_mm_per_degree": (fitted_slopes * signed_direction[:, None]).round(7).tolist(),
        "compliance_mm_per_1000ma": compliance.round(5).tolist(),
        "direction_hysteresis_mm": hysteresis.round(5).tolist(),
        "train_residual_rms_mm": float(np.sqrt(np.mean(train_error * train_error))),
        "train_residual_p95_mm": percentile(np.abs(train_error).tolist(), 0.95),
        "validation_closure_rms_p95_mm": percentile(validation_closure, 0.95),
        "validation_passive_residual_p95_mm": percentile(validation_passive, 0.95),
        "validation_abs_yaw_p95_deg": percentile(validation_yaw, 0.95),
        "evaluations": int(solved.nfev),
        "optimality": float(solved.optimality),
    }


if __name__ == "__main__":
    log = Path(__file__).with_name("y42_pc_process_log.jsonl")
    print(json.dumps(fit(latest_completed_snake_samples(log), web.calibration_data()), ensure_ascii=False, indent=2))
