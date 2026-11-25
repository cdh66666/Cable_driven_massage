#!/usr/bin/env python3
"""Exploratory motor-only bundle identifiability on a frozen calibration log.

Pure recorded-data analysis: no web/serial imports and no network calls.
Optimizer outputs are not accepted calibration or metrology.
"""
import json
import math
from pathlib import Path
import time

import numpy as np

from analyze_y42_start_pose_offline import AXES, ATTACHMENTS, YAW_SCALE, geometry, local_solve, to_lengths
from y42_spool_geometry import planar_platform_payouts

TOOLS = Path(__file__).resolve().parent
RUN = TOOLS / "manual_test_runs/20260905_102906_733_calibration_worker_1d71bee2"
FROZEN = TOOLS / "manual_test_runs/20260905_103233_820_trajectory_worker_d007530c/manifest.json"
PHI_REFS = [-math.pi / 4, -3 * math.pi / 4, 3 * math.pi / 4, math.pi / 4]


def parse_rounds():
    events = [json.loads(line) for line in (RUN / "events.jsonl").read_text(encoding="utf-8").splitlines()]
    rounds, buffer = [], {}
    for event in events:
        if event.get("event") != "serial_transaction":
            continue
        if event.get("tx_hex") == "01246b":
            buffer = {i: {} for i in range(1, 5)}
        packet = bytes.fromhex(event.get("rx_hex", ""))
        if len(packet) < 4 or packet[-1] != 0x6B or packet[0] not in (1, 2, 3, 4):
            continue
        motor, command = packet[:2]
        if not buffer:
            continue
        expected_length = {0x27: 5, 0x35: 6, 0x36: 8, 0x3A: 4}.get(command)
        if expected_length is not None:
            if len(packet) < expected_length or packet[expected_length - 1] != 0x6B:
                continue
            # Several reads contain a complete response followed by 0100ee6b.
            # Parse only the validated expected reply, not the unrelated tail.
            packet = packet[:expected_length]
        record = buffer[motor]
        timestamp = event["completed_monotonic_ns"] / 1e9
        if command == 0x27 and len(packet) == 5:
            record["current"] = int.from_bytes(packet[2:4], "big")
        elif command == 0x35 and len(packet) == 6:
            record["rpm"] = (-1 if packet[2] else 1) * int.from_bytes(packet[3:5], "big") / 10
            record["rpm_t"] = timestamp
        elif command == 0x36 and len(packet) == 8:
            record["q"] = (-1 if packet[2] else 1) * int.from_bytes(packet[3:7], "big") / 10
            record["q_t"] = timestamp
        elif command == 0x3A and len(packet) == 4:
            record["flags"] = packet[2]
            if motor == 4 and all(all(key in buffer[i] for key in ("q", "q_t", "rpm", "current", "flags")) for i in range(1, 5)):
                rows = [buffer[i] for i in range(1, 5)]
                common_t = float(np.mean([row["q_t"] for row in rows]))
                rounds.append({
                    "q_raw": [row["q"] for row in rows],
                    "q_aligned_constant_rpm": [row["q"] + 6 * row["rpm"] * (common_t - row["q_t"]) for row in rows],
                    "current": [row["current"] for row in rows],
                    "rpm": [row["rpm"] for row in rows],
                    "flags": [row["flags"] for row in rows],
                    "position_skew_ms": 1000 * (max(row["q_t"] for row in rows) - min(row["q_t"] for row in rows)),
                    "t": common_t,
                })
                buffer = {}
    return events, rounds


def model_geometry(pose):
    record = planar_platform_payouts(float(pose[0]), float(pose[1]), float(pose[2]) / YAW_SCALE * 180 / math.pi, AXES, ATTACHMENTS, [12] * 4, [1, -1, 1, -1], PHI_REFS)
    jacobian = np.array(record["jacobian_xy_yaw_rad"])
    jacobian[:, 2] /= YAW_SCALE
    return np.array(record["lengths_mm"]), jacobian


def evaluate(parameters, normalized_q):
    n = len(normalized_q)
    gains = np.exp(parameters[:4])
    offsets = parameters[4:8]
    poses = parameters[8:].reshape(n, 3)
    residual = np.empty(n * 4)
    jacobian = np.zeros((n * 4, 8 + n * 3))
    projected = []
    for k, pose in enumerate(poses):
        lengths, pose_j = model_geometry(pose)
        r = gains * normalized_q[k] + offsets - lengths
        start = k * 4
        residual[start:start + 4] = r
        jacobian[start:start + 4, :4] = np.diag(gains * normalized_q[k])
        jacobian[start:start + 4, 4:8] = np.eye(4)
        jacobian[start:start + 4, 8 + k * 3:11 + k * 3] = -pose_j
        _, _, vt = np.linalg.svd(pose_j.T, full_matrices=True)
        null = vt[-1]
        projected.append(null @ jacobian[start:start + 4, :8])
    return residual, jacobian, np.array(projected)


def solve_bundle(normalized_q, initial, budget_deadline, max_iterations=100):
    parameters = initial.copy()
    damping = 1e-2
    iterations = 0
    bounded_rejections = 0
    for iterations in range(1, max_iterations + 1):
        if time.perf_counter() > budget_deadline:
            break
        residual, jacobian, _ = evaluate(parameters, normalized_q)
        normal = jacobian.T @ jacobian
        step = np.linalg.solve(normal + damping * np.diag(np.maximum(np.diag(normal), 1)), -(jacobian.T @ residual))
        accepted = False
        for fraction in (1, 0.5, 0.25, 0.125):
            candidate = parameters + fraction * step
            poses = candidate[8:].reshape(-1, 3)
            # A weak, explicitly exploratory radius prior: limit scale ratio to
            # [1/1.5,1.5]. This is NOT hardware-validated bounds or a calibration.
            if np.any(np.abs(candidate[:4]) > math.log(1.5)) or np.any(poses[:, 0] < -25) or np.any(poses[:, 0] > 135) or np.any(poses[:, 1] < -25) or np.any(poses[:, 1] > 173) or np.any(np.abs(poses[:, 2]) > math.radians(60) * YAW_SCALE):
                bounded_rejections += 1
                continue
            try:
                candidate_r, _, _ = evaluate(candidate, normalized_q)
            except ValueError:
                continue
            if candidate_r @ candidate_r < residual @ residual:
                parameters = candidate
                damping = max(1e-8, damping * 0.35)
                accepted = True
                break
        if not accepted:
            damping *= 10
            if damping > 1e12:
                break
        elif np.linalg.norm(step) < 1e-7:
            break
    residual, jacobian, projected = evaluate(parameters, normalized_q)
    full_s = np.linalg.svd(jacobian, compute_uv=False)
    normalized_projected = projected / np.maximum(np.linalg.norm(projected, axis=0), 1e-12)
    _, global_s, global_vt = np.linalg.svd(normalized_projected, full_matrices=False)
    result = {
        "iterations": iterations, "rms_mm": float(np.sqrt(np.mean(residual ** 2))), "maximum_residual_mm": float(np.max(np.abs(residual))),
        "gain_ratios": np.exp(parameters[:4]).tolist(), "length_offsets_mm": parameters[4:8].tolist(),
        "global_projected_singular_values_column_normalized": global_s.tolist(),
        "global_projected_condition": float(global_s[0] / global_s[-1]),
        "global_std_per_1mm_iid_residual_noise": np.sqrt(np.maximum(np.diag(np.linalg.pinv(projected.T @ projected)), 0)).tolist(),
        "weakest_global_parameter_direction": global_vt[-1].tolist(),
        "full_jacobian_shape": list(jacobian.shape), "full_jacobian_min_singular": float(full_s[-1]),
        "bounded_candidate_rejections": bounded_rejections,
        "poses_xy_yaw_deg": [[float(p[0]), float(p[1]), float(p[2]) / YAW_SCALE * 180 / math.pi] for p in parameters[8:].reshape(-1, 3)],
    }
    return parameters, result


def main():
    started = time.perf_counter()
    events, rounds = parse_rounds()
    candidates = [row for row in rounds if min(row["current"]) >= 100 and all(flag & 1 and not flag & 12 for flag in row["flags"])]
    stationary = [row for row in candidates if max(abs(rpm) for rpm in row["rpm"]) < 1]
    summary = {"events": len(events), "joint_complete_rounds": len(rounds), "all_current_at_least100_enabled_no_fault": len(candidates), "stationary_under1rpm": len(stationary), "position_skew_ms_range": [min(row["position_skew_ms"] for row in rounds), max(row["position_skew_ms"] for row in rounds)], "evidence_limit": "Current threshold is only a proxy; samples are sequential, not synchronous. Constant-rpm alignment cannot correct unmeasured acceleration."}
    clusters = []
    for row in stationary:
        match = next((group for group in clusters if max(abs(x - y) for x, y in zip(row["q_raw"], group[0]["q_raw"])) < 1), None)
        if match is None:
            clusters.append([row])
        else:
            match.append(row)
    summary["stationary_encoder_clusters_under1degree"] = len(clusters)
    summary["stationary_cluster_sizes"] = [len(group) for group in clusters]
    summary["stationary_only_global_codimension_rank_upper_bound"] = min(8, len(clusters))
    indices = np.unique(np.linspace(0, len(candidates) - 1, min(20, len(candidates))).astype(int))
    chosen = [candidates[i] for i in indices]
    summary["selected_samples"] = chosen
    calibration = json.loads(FROZEN.read_text(encoding="utf-8"))["calibration_before"]
    corner_lengths = np.array([model_geometry(np.array([x, y, 0]))[0] for x, y in [(0, 148), (110, 148), (110, 0), (0, 0)]])
    samples = np.array(calibration["samples"])
    fits = np.array([np.linalg.lstsq(np.column_stack((corner_lengths[:, i], np.ones(4))), samples[:, i], rcond=None)[0] for i in range(4)])
    q = np.array([row["q_aligned_constant_rpm"] for row in chosen])
    qref = np.mean(q, axis=0)
    normalized_q = (q - qref) / fits[:, 0]
    offsets = (qref - fits[:, 1]) / fits[:, 0]
    base_poses = []
    for row in chosen:
        lengths = to_lengths(calibration, row["q_aligned_constant_rpm"])
        alternatives = [local_solve(lengths, (55, 74, math.radians(yaw) * YAW_SCALE)) for yaw in (-35, 0, 35)]
        base_poses.append(min(alternatives, key=lambda item: item[1])[0])
    base_poses = np.array(base_poses)
    base_poses[:, 0] = np.clip(base_poses[:, 0], 0, 110)
    base_poses[:, 1] = np.clip(base_poses[:, 1], 0, 148)
    base_poses[:, 2] = np.clip(base_poses[:, 2], -math.radians(50) * YAW_SCALE, math.radians(50) * YAW_SCALE)
    summaries = []
    for seed in range(3):
        rng = np.random.default_rng(seed)
        poses = base_poses.copy()
        if seed:
            poses[:, 2] *= -1 if seed == 1 else 0
        initial = np.concatenate((rng.normal(0, 0.04, 4), offsets + rng.normal(0, 1, 4), poses.reshape(-1)))
        parameters, result = solve_bundle(normalized_q, initial, started + 105)
        result["seed"] = seed
        result["fitted_slopes_deg_per_mm"] = (fits[:, 0] / np.exp(parameters[:4])).tolist()
        result["implied_radii_mm"] = (360 / (2 * math.pi * np.abs(fits[:, 0] / np.exp(parameters[:4])))).tolist()
        summaries.append(result)
    summary["bundle_solutions"] = summaries
    summary["elapsed_s"] = time.perf_counter() - started
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
