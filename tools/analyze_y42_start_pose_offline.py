#!/usr/bin/env python3
"""Frozen-run planar geometry analysis; no server imports or hardware access.

Reads recorded encoder data and computes nominal-model hypotheses.  No result
is an independent measurement of actual ball pose, tension or absolute error.
"""
import json
import math
import itertools
from pathlib import Path

import numpy as np

from y42_spool_geometry import planar_platform_payouts

RUN = Path(__file__).resolve().parent / "manual_test_runs/20260905_103233_820_trajectory_worker_d007530c"
AXES = np.array([[-44, 148], [154, 148], [154, 0], [-44, 0]], dtype=float)
EXITS = AXES + np.array([[12.5, 0], [-12.5, 0], [-12.5, 0], [12.5, 0]])
ATTACHMENTS = np.array([[-15.267, 11.4115], [15.267, 11.4115], [15.267, -11.4115], [-15.267, -11.4115]])
YAW_SCALE = 15.267


def geometry(pose):
    x, y, yaw_distance = pose
    yaw = yaw_distance / YAW_SCALE
    c, s = math.cos(yaw), math.sin(yaw)
    rotation = np.array([[c, -s], [s, c]])
    rotated = ATTACHMENTS @ rotation.T
    delta = np.array([x, y]) + rotated - EXITS
    lengths = np.linalg.norm(delta, axis=1)
    unit = delta / lengths[:, None]
    derivative = np.column_stack((-rotated[:, 1], rotated[:, 0])) / YAW_SCALE
    jacobian = np.column_stack((unit, np.sum(unit * derivative, axis=1)))
    return lengths, jacobian


def curve_eval(xs, ys, query):
    points = sorted(zip(xs, ys))
    index = next((i for i in range(len(points) - 1) if query <= points[i + 1][0]), len(points) - 2)
    x0, y0 = points[index]
    x1, y1 = points[index + 1]
    return y0 + (query - x0) * (y1 - y0) / (x1 - x0)


def to_lengths(calibration, positions):
    return np.array([
        curve_eval(calibration["position_samples_deg"][i], calibration["distance_samples_mm"][i], q)
        for i, q in enumerate(positions)
    ])


def to_positions(calibration, lengths):
    return np.array([
        curve_eval(calibration["distance_samples_mm"][i], calibration["position_samples_deg"][i], length)
        for i, length in enumerate(lengths)
    ])


def local_solve(lengths, seed, active=(0, 1, 2, 3), geometry_fn=None):
    geometry_fn = geometry if geometry_fn is None else geometry_fn
    pose = np.asarray(seed, dtype=float).copy()
    damping = 1e-3
    indices = list(active)
    for _ in range(100):
        try:
            predicted, jacobian = geometry_fn(pose)
        except ValueError:
            return pose, float("inf"), np.full(4, float("inf"))
        residual = predicted[indices] - lengths[indices]
        j = jacobian[indices]
        step = np.linalg.solve(j.T @ j + damping * np.eye(3), -(j.T @ residual))
        norm = np.linalg.norm(step)
        if norm > 15:
            step *= 15 / norm
        candidate = pose + step
        candidate[2] = (candidate[2] / YAW_SCALE + math.pi) % (2 * math.pi) - math.pi
        candidate[2] *= YAW_SCALE
        try:
            candidate_cost = np.linalg.norm(geometry_fn(candidate)[0][indices] - lengths[indices])
        except ValueError:
            candidate_cost = float("inf")
        if candidate_cost < np.linalg.norm(residual):
            pose = candidate
            damping = max(1e-9, damping * 0.3)
            if np.linalg.norm(step) < 1e-8:
                break
        else:
            damping *= 10
            if damping > 1e10:
                break
    residual = geometry_fn(pose)[0] - lengths
    return pose, float(np.sqrt(np.mean(residual[indices] ** 2))), residual


def describe(pose, rms=0.0, residual=None, geometry_fn=None):
    geometry_fn = geometry if geometry_fn is None else geometry_fn
    lengths, jacobian = geometry_fn(pose)
    singular = np.linalg.svd(jacobian, compute_uv=False)
    _, _, vt = np.linalg.svd(jacobian.T, full_matrices=True)
    null = vt[-1]
    if np.sum(null) < 0:
        null = -null
    return {
        "pose_xy_yaw_deg": [float(pose[0]), float(pose[1]), float(pose[2] / YAW_SCALE * 180 / math.pi)],
        "active_fit_rms_mm": rms,
        "closure_rms_mm": float(np.sqrt(np.mean(residual ** 2))) if residual is not None else 0.0,
        "residuals_mm": residual.tolist() if residual is not None else [0.0] * 4,
        "scaled_jacobian_singular_values": singular.tolist(),
        "scaled_condition_number": float(singular[0] / singular[-1]),
        "zero_external_wrench_tension_null_vector": null.tolist(),
        "all_positive_pretension_possible_without_external_planar_wrench": bool(np.min(null) > 1e-7),
    }


def physical_wrench(x, y, yaw_deg, model="tangent", branches=(1, -1, 1, -1), radius=12.0):
    yaw = math.radians(yaw_deg)
    c, s = math.cos(yaw), math.sin(yaw)
    rotated = ATTACHMENTS @ np.array([[c, -s], [s, c]]).T
    endpoints = np.array([x, y]) + rotated
    if model == "axis":
        exits = AXES
    elif model == "fixed_inward":
        exits = EXITS
    else:
        delta = endpoints - AXES
        d2 = np.sum(delta ** 2, axis=1)
        tangent_offset = radius ** 2 * delta / d2[:, None]
        perpendicular = np.column_stack((-delta[:, 1], delta[:, 0]))
        tangent_offset += np.array(branches)[:, None] * radius * np.sqrt(d2 - radius ** 2)[:, None] * perpendicular / d2[:, None]
        exits = AXES + tangent_offset
    directions = exits - endpoints
    directions /= np.linalg.norm(directions, axis=1)[:, None]
    moments = (rotated[:, 0] * directions[:, 1] - rotated[:, 1] * directions[:, 0]) / YAW_SCALE
    wrench = np.vstack((directions.T, moments))
    _, singular, vt = np.linalg.svd(wrench, full_matrices=True)
    null = vt[-1]
    if np.sum(null) < 0:
        null *= -1
    return {"model": model, "xy_yaw_deg": [x, y, yaw_deg], "singular_values": singular.tolist(), "condition": float(singular[0] / singular[-1]), "null": null.tolist(), "positive_tension_zero_planar_wrench": bool(np.min(null) > 1e-8)}


def solve_branches(lengths, active=(0, 1, 2, 3), exhaustive=False, geometry_fn=None):
    seeds = []
    for x in ([22, 55, 90] if exhaustive else [35]):
        for y in ([29.6, 74, 120] if exhaustive else [45]):
            for yaw in range(-165, 180, 15):
                seeds.append([x, y, math.radians(yaw) * YAW_SCALE])
    solutions = []
    for seed in seeds:
        pose, rms, residual = local_solve(lengths, seed, active, geometry_fn)
        if not math.isfinite(rms):
            continue
        if not any(np.linalg.norm(pose - existing[0]) < 0.05 for existing in solutions):
            solutions.append((pose, rms, residual))
    return sorted(solutions, key=lambda item: item[1])


def main():
    manifest = json.loads((RUN / "manifest.json").read_text(encoding="utf-8"))
    calibration = manifest["calibration_before"]
    events = [json.loads(line) for line in (RUN / "events.jsonl").read_text(encoding="utf-8").splitlines()]
    start = next(event for event in events if event["event"] == "trajectory_start_ramp")
    feedback = [event for event in events if event["event"] == "trajectory_start_sample"][-1]
    q0 = np.array(start["current_positions"])
    q1 = np.array(start["target_positions"])
    q_stalled = np.array(feedback["positions"])
    result = {"run_id": manifest["run_id"], "evidence_scope": "nominal frozen encoder/geometry model only", "initial_positions_deg": q0.tolist(), "target_positions_deg": q1.tolist(), "last_start_feedback_deg": q_stalled.tolist()}
    groups = {}
    for label, q in [("initial", q0), ("commanded_target", q1), ("stalled_feedback", q_stalled)]:
        lengths = to_lengths(calibration, q)
        branches = solve_branches(lengths, exhaustive=True)
        groups[label] = [{**describe(*item), "lengths_mm": lengths.tolist()} for item in branches[:5]]
    result["solutions"] = groups
    target_ideal = np.array([22, 29.6, 0.0])
    result["ideal_target"] = describe(target_ideal)
    three_branches = solve_branches(to_lengths(calibration, q1), active=(0, 1, 2), exhaustive=True)
    result["first_three_at_commanded_target_branches"] = [describe(*item) for item in three_branches[:5]]
    wrong_branch = next(item[0] for item in three_branches if abs(item[0][2]) > YAW_SCALE * 0.1 and item[1] < 1e-6)
    branch_q_path = []
    branch_det = []
    for fraction in np.linspace(0, 1, 201):
        lengths, jacobian = geometry(wrong_branch + fraction * (target_ideal - wrong_branch))
        branch_q_path.append(to_positions(calibration, lengths))
        branch_det.append(float(np.linalg.det(jacobian[:3])))
    branch_q = np.array(branch_q_path)
    result["pose_path_between_three_rope_branches"] = {
        "first_three_max_departure_from_final_angle_deg": np.max(np.abs(branch_q[:, :3] - q1[:3]), axis=0).tolist(),
        "m4_positions_start_end_deg": [float(branch_q[0, 3]), float(branch_q[-1, 3])],
        "three_rope_jacobian_determinants_start_end": [branch_det[0], branch_det[-1]],
        "three_rope_determinant_min_abs_on_path": min(abs(value) for value in branch_det),
        "three_rope_determinant_changes_sign": min(branch_det) < 0 < max(branch_det),
    }
    result["calibration_curve_segment_slopes_deg_per_mm"] = [
        np.diff(calibration["position_samples_deg"][i]).__truediv__(np.diff(calibration["distance_samples_mm"][i])).tolist()
        for i in range(4)
    ]
    result["curve_implied_segment_radius_mm"] = [
        [360 / (2 * math.pi * abs(value)) for value in row]
        for row in result["calibration_curve_segment_slopes_deg_per_mm"]
    ]
    phi_refs = [-math.pi / 4, -3 * math.pi / 4, 3 * math.pi / 4, math.pi / 4]
    corner_payouts = np.array([
        planar_platform_payouts(x, y, 0, AXES, ATTACHMENTS, [12] * 4, [1, -1, 1, -1], phi_refs)["lengths_mm"]
        for x, y in [(0, 148), (110, 148), (110, 0), (0, 0)]
    ])
    frozen_samples = np.array(calibration["samples"])
    physical_affine_fits = []
    for motor in range(4):
        basis = np.column_stack((corner_payouts[:, motor], np.ones(4)))
        coefficients, *_ = np.linalg.lstsq(basis, frozen_samples[:, motor], rcond=None)
        residual = basis @ coefficients - frozen_samples[:, motor]
        score = 1 - np.sum(residual ** 2) / np.sum((frozen_samples[:, motor] - np.mean(frozen_samples[:, motor])) ** 2)
        physical_affine_fits.append({"slope_deg_per_mm": float(coefficients[0]), "intercept_deg": float(coefficients[1]), "r2": float(score), "residuals_deg": residual.tolist(), "residual_rms_deg": float(np.sqrt(np.mean(residual ** 2))), "implied_radius_mm": 360 / (2 * math.pi * abs(float(coefficients[0])))})
    result["full_tangent_wrap_affine_corner_fit"] = physical_affine_fits
    comparisons = {}
    for label, model_branches in [("legacy_global_affine", None), ("full_wrap_photo_hypothesis", [1, -1, 1, -1]), ("full_wrap_opposite_hypothesis", [-1, 1, -1, 1])]:
        if model_branches is None:
            geom_fn = geometry
            coefficients = np.array([calibration["slope"], calibration["intercept"]]).T
        else:
            def geom_fn(pose, selected=model_branches):
                record = planar_platform_payouts(float(pose[0]), float(pose[1]), float(pose[2]) / YAW_SCALE * 180 / math.pi, AXES, ATTACHMENTS, [12] * 4, selected, phi_refs)
                jacobian = np.array(record["jacobian_xy_yaw_rad"])
                jacobian[:, 2] /= YAW_SCALE
                return np.array(record["lengths_mm"]), jacobian
            model_corners = np.array([geom_fn(np.array([x, y, 0]))[0] for x, y in [(0, 148), (110, 148), (110, 0), (0, 0)]])
            coefficients = np.array([np.linalg.lstsq(np.column_stack((model_corners[:, i], np.ones(4))), frozen_samples[:, i], rcond=None)[0] for i in range(4)])
        model_corners = np.array([geom_fn(np.array([x, y, 0]))[0] for x, y in [(0, 148), (110, 148), (110, 0), (0, 0)]])
        encoder_residuals = model_corners * coefficients[:, 0] + coefficients[:, 1] - frozen_samples
        length_residuals = encoder_residuals / coefficients[:, 0]
        model_result = {"slope_deg_per_mm": coefficients[:, 0].tolist(), "implied_radius_mm": (360 / (2 * math.pi * np.abs(coefficients[:, 0]))).tolist(), "fit_rms_mm": np.sqrt(np.mean(length_residuals ** 2, axis=0)).tolist(), "fit_max_mm": np.max(np.abs(length_residuals), axis=0).tolist(), "model_target_q_deg": (geom_fn(target_ideal)[0] * coefficients[:, 0] + coefficients[:, 1]).tolist(), "fk": {}}
        for point_label, q in [("calibration_end", np.array([33.7, 208.0, -157.9, -251.6])), ("trajectory_initial", q0), ("stalled_feedback", q_stalled), ("old_commanded_target", q1)]:
            model_lengths = (q - coefficients[:, 1]) / coefficients[:, 0]
            solved = solve_branches(model_lengths, geometry_fn=geom_fn)
            model_result["fk"][point_label] = [describe(*item, geometry_fn=geom_fn) for item in solved[:3]]
        comparisons[label] = model_result
    result["affine_model_comparison"] = comparisons
    result["wrench_axis_vs_tangent"] = [
        physical_wrench(x, y, yaw, model)
        for x, y in [(55, 74), (22, 29.6)]
        for yaw in [-30, -10, -1, 0, 1, 10, 30]
        for model in ["axis", "fixed_inward", "tangent"]
    ]
    result["all_tangent_branch_options_at_target"] = [
        {"branches": list(branches), **physical_wrench(22, 29.6, 0, branches=branches)}
        for branches in itertools.product([-1, 1], repeat=4)
    ]

    trajectory = []
    previous_pose = solve_branches(to_lengths(calibration, q0))[0][0]
    initial_pose = previous_pose.copy()
    for fraction in np.linspace(0, 1, 41):
        lengths = to_lengths(calibration, q0 + fraction * (q1 - q0))
        pose, rms, residual = local_solve(lengths, previous_pose)
        previous_pose = pose
        trajectory.append({"fraction": float(fraction), **describe(pose, rms, residual)})
    result["joint_linear_path"] = {"max_closure_rms_mm": max(row["closure_rms_mm"] for row in trajectory), "samples": [trajectory[i] for i in [0, 10, 20, 30, 40]]}
    pose_path = [describe(initial_pose + fraction * (target_ideal - initial_pose)) for fraction in np.linspace(0, 1, 41)]
    result["pose_linear_path"] = {"samples": [pose_path[i] for i in [0, 10, 20, 30, 40]], "min_singular_value": min(row["scaled_jacobian_singular_values"][-1] for row in pose_path), "all_positive_pretension_everywhere_without_external_wrench": all(row["all_positive_pretension_possible_without_external_planar_wrench"] for row in pose_path)}
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
