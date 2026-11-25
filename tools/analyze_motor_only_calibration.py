#!/usr/bin/env python3
"""Fit motor-only cable length maps from a logged rigid-platform sweep.

This is an offline identifiability/quality tool.  It uses only four encoder
positions plus the known cable exits and platform attachments.  Commanded XY is
used only as the nonlinear solver's initial guess, never as a residual or fit
target.  A production calibration sweep should collect the same data with the
3-position + 1-torque role rotated across all four motors.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix

import y42_pc_direct_web as web


def latest_completed_snake_samples(path: Path) -> list[dict]:
    current: list[dict] = []
    current_passive: int | None = None
    latest_by_passive: dict[int, list[dict]] = {}
    fallback_latest: list[dict] = []
    with path.open("r", encoding="utf-8") as handle:
        for raw in handle:
            try:
                item = json.loads(raw)
            except json.JSONDecodeError:
                continue
            event = item.get("event")
            if event == "planar_snake_test_start":
                current = []
                value = item.get("passive_motor")
                current_passive = int(value) if isinstance(value, (int, float)) else None
            elif event == "planar_snake_test_sample" and current is not None:
                if isinstance(item.get("positions_deg"), list) and len(item["positions_deg"]) == 4:
                    current.append(item)
                    if current_passive is None and isinstance(item.get("passive_motor"), (int, float)):
                        current_passive = int(item["passive_motor"])
            elif event == "planar_snake_test_complete" and current:
                fallback_latest = current
                if current_passive in (1, 2, 3, 4):
                    latest_by_passive[int(current_passive)] = current
                current = []
                current_passive = None
    if latest_by_passive:
        return [item for motor in sorted(latest_by_passive) for item in latest_by_passive[motor]]
    return fallback_latest


def evenly_spaced(items: list[dict], count: int) -> list[dict]:
    if len(items) <= count:
        return list(items)
    indices = np.linspace(0, len(items) - 1, count).round().astype(int)
    return [items[int(index)] for index in indices]


def percentile(values: list[float], fraction: float) -> float | None:
    return float(np.quantile(np.asarray(values), fraction)) if values else None


def fit(samples: list[dict], calibration: dict) -> dict:
    by_passive: dict[int, list[dict]] = {}
    for item in samples:
        passive = int(item.get("passive_motor", 0) or 0)
        by_passive.setdefault(passive, []).append(item)
    selected: list[dict] = []
    for passive in sorted(by_passive):
        selected.extend(evenly_spaced(by_passive[passive], 80))
    training = []
    validation = []
    role_offsets: dict[int, int] = {}
    for item in selected:
        passive = int(item.get("passive_motor", 0) or 0)
        index = role_offsets.get(passive, 0)
        role_offsets[passive] = index + 1
        (validation if index % 4 == 0 else training).append(item)
    q = np.asarray([item["positions_deg"] for item in training], dtype=float)
    q_reference = np.median(q, axis=0)
    phase_current = np.asarray(
        [
            [float(value) if isinstance(value, (int, float)) else np.nan for value in item.get("phase_current_ma", [None] * 4)]
            for item in training
        ],
        dtype=float,
    )
    current_reference = np.nanmedian(np.abs(phase_current), axis=0)
    phase_current = np.where(np.isfinite(phase_current), np.abs(phase_current), current_reference[None, :])
    passive_indices = np.asarray(
        [max(0, min(3, int(item.get("passive_motor", 1) or 1) - 1)) for item in training],
        dtype=int,
    )
    slopes = np.asarray(calibration["slope"], dtype=float)
    intercepts = np.asarray(calibration["intercept"], dtype=float)
    k_initial = 1.0 / slopes
    length_reference = (q_reference - intercepts) / slopes
    pose_initial = np.asarray(
        [
            [
                float(item.get("target_xy", [web.WORKSPACE_WIDTH_MM / 2.0, web.WORKSPACE_HEIGHT_MM / 2.0])[0]),
                float(item.get("target_xy", [web.WORKSPACE_WIDTH_MM / 2.0, web.WORKSPACE_HEIGHT_MM / 2.0])[1]),
                0.0,
            ]
            for item in training
        ],
        dtype=float,
    )
    initial = np.concatenate((length_reference, k_initial, np.zeros(4), np.zeros(4), pose_initial.ravel()))
    count = len(training)
    exit_x, exit_y = web.cable_exit_coordinates(calibration)
    body_x, body_y = web.ball_attachment_offsets(calibration)
    exit_x = np.asarray(exit_x)[None, :]
    exit_y = np.asarray(exit_y)[None, :]
    body_x = np.asarray(body_x)[None, :]
    body_y = np.asarray(body_y)[None, :]

    def cable_errors(vector: np.ndarray) -> np.ndarray:
        reference = vector[:4]
        k = vector[4:8]
        quadratic = vector[8:12]
        compliance = vector[12:16]
        poses = vector[16:].reshape(count, 3)
        cosine = np.cos(poses[:, 2])[:, None]
        sine = np.sin(poses[:, 2])[:, None]
        endpoint_x = poses[:, 0, None] + cosine * body_x - sine * body_y
        endpoint_y = poses[:, 1, None] + sine * body_x + cosine * body_y
        predicted = np.hypot(exit_x - endpoint_x, exit_y - endpoint_y)
        delta_q = q - q_reference[None, :]
        measured = (
            reference[None, :]
            + delta_q * k[None, :]
            + (delta_q / 100.0) ** 2 * quadratic[None, :]
            + ((phase_current - current_reference[None, :]) / 1000.0) * compliance[None, :]
        )
        return predicted - measured

    def residual(vector: np.ndarray) -> np.ndarray:
        cable_error_matrix = cable_errors(vector)
        # The three position-controlled cables define each pose.  Give their
        # equations more weight and let the actual torque cable carry the
        # independent closure residual used to identify the global maps.
        weights = np.full_like(cable_error_matrix, 4.0)
        weights[np.arange(count), passive_indices] = 1.0
        cable_error = (cable_error_matrix * weights).ravel()
        # A weak physical-radius prior prevents an unobservable numerical scale
        # collapse.  The other priors only regularise wrap-radius curvature and
        # elastic current compensation; held-out residuals decide acceptance.
        k = vector[4:8]
        quadratic = vector[8:12]
        compliance = vector[12:16]
        radius_prior = (k - k_initial) * 20.0
        quadratic_prior = quadratic * 0.5
        compliance_prior = compliance * 0.1
        return np.concatenate((cable_error, radius_prior, quadratic_prior, compliance_prior))

    rows = count * 4 + 12
    columns = 16 + count * 3
    sparsity = lil_matrix((rows, columns), dtype=int)
    for sample in range(count):
        for motor in range(4):
            row = sample * 4 + motor
            sparsity[row, motor] = 1
            sparsity[row, 4 + motor] = 1
            sparsity[row, 8 + motor] = 1
            sparsity[row, 12 + motor] = 1
            sparsity[row, 16 + sample * 3:16 + sample * 3 + 3] = 1
    for motor in range(4):
        sparsity[count * 4 + motor, 4 + motor] = 1
        sparsity[count * 4 + 4 + motor, 8 + motor] = 1
        sparsity[count * 4 + 8 + motor, 12 + motor] = 1

    lower_k = np.asarray([-0.245 if value < 0 else 0.18 for value in k_initial])
    upper_k = np.asarray([-0.18 if value < 0 else 0.245 for value in k_initial])
    lower_pose = np.tile(np.asarray([-20.0, -20.0, math.radians(-35.0)]), count)
    upper_pose = np.tile(
        np.asarray([web.WORKSPACE_WIDTH_MM + 20.0, web.WORKSPACE_HEIGHT_MM + 20.0, math.radians(35.0)]),
        count,
    )
    lower = np.concatenate((np.full(4, 1.0), lower_k, np.full(4, -0.6), np.zeros(4), lower_pose))
    upper = np.concatenate((np.full(4, 230.0), upper_k, np.full(4, 0.6), np.full(4, 10.0), upper_pose))
    solved = least_squares(
        residual,
        initial,
        bounds=(lower, upper),
        jac_sparsity=sparsity.tocsr(),
        x_scale="jac",
        loss="soft_l1",
        f_scale=0.35,
        ftol=1e-11,
        xtol=1e-11,
        gtol=1e-11,
        max_nfev=300,
        verbose=0,
    )
    reference = solved.x[:4]
    k = solved.x[4:8]
    quadratic = solved.x[8:12]
    compliance = solved.x[12:16]
    train_error = cable_errors(solved.x).ravel()
    validation_rms: list[float] = []
    validation_max: list[float] = []
    validation_passive: list[float] = []
    validation_yaw: list[float] = []
    for sample_index, item in enumerate(validation):
        sample_q = np.asarray(item["positions_deg"], dtype=float)
        sample_current = np.asarray(
            [float(value) if isinstance(value, (int, float)) else current_reference[index] for index, value in enumerate(item.get("phase_current_ma", [None] * 4))],
            dtype=float,
        )
        delta_q = sample_q - q_reference
        lengths = (
            reference
            + delta_q * k
            + (delta_q / 100.0) ** 2 * quadratic
            + ((np.abs(sample_current) - current_reference) / 1000.0) * compliance
        ).tolist()
        initial_pose = [*item.get("target_xy", [55.0, 75.0]), 0.0]
        pose = web.solve_planar_platform_pose(lengths, calibration, initial_pose=initial_pose)
        if pose is not None:
            validation_rms.append(float(pose["closure_rms_mm"]))
            validation_max.append(float(pose["closure_max_mm"]))
            validation_yaw.append(abs(float(pose["yaw_deg"])))
        passive_motor = int(item.get("passive_motor", 0) or 0)
        passive = passive_motor - 1 if passive_motor in (1, 2, 3, 4) else sample_index % 4
        active = tuple(index for index in range(4) if index != passive)
        independent = web.solve_planar_platform_pose(
            lengths,
            calibration,
            active_indices=active,
            initial_pose=initial_pose,
        )
        if independent is not None and independent["redundant_residuals_mm"]:
            validation_passive.append(abs(float(independent["redundant_residuals_mm"][0])))

    return {
        "success": bool(solved.success),
        "message": solved.message,
        "source_samples": len(samples),
        "training_samples": len(training),
        "validation_samples": len(validation),
        "passive_roles": sorted(key for key in by_passive if key in (1, 2, 3, 4)),
        "q_reference_deg": q_reference.round(6).tolist(),
        "current_reference_ma": current_reference.round(3).tolist(),
        "length_reference_mm": reference.round(6).tolist(),
        "mm_per_degree": k.round(9).tolist(),
        "quadratic_mm_per_100deg2": quadratic.round(9).tolist(),
        "compliance_mm_per_1000ma": compliance.round(6).tolist(),
        "degrees_per_mm": (1.0 / k).round(6).tolist(),
        "effective_radius_mm": (np.abs(k) * 180.0 / math.pi).round(6).tolist(),
        "train_residual_rms_mm": float(np.sqrt(np.mean(train_error * train_error))),
        "train_residual_p95_mm": percentile(np.abs(train_error).tolist(), 0.95),
        "train_residual_max_mm": float(np.max(np.abs(train_error))),
        "validation_closure_rms_p95_mm": percentile(validation_rms, 0.95),
        "validation_closure_max_mm": max(validation_max, default=None),
        "validation_passive_residual_p95_mm": percentile(validation_passive, 0.95),
        "validation_abs_yaw_p95_deg": percentile(validation_yaw, 0.95),
        "cost": float(solved.cost),
        "optimality": float(solved.optimality),
        "evaluations": int(solved.nfev),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--log",
        type=Path,
        default=Path(__file__).with_name("y42_pc_process_log.jsonl"),
    )
    args = parser.parse_args()
    samples = latest_completed_snake_samples(args.log)
    if not samples:
        raise SystemExit("没有找到已完成的平面蛇形样本")
    print(json.dumps(fit(samples, web.calibration_data()), ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
