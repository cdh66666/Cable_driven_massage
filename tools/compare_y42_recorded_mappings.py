#!/usr/bin/env python3
"""Offline same-encoder comparison. No web-controller import or hardware IO.

Only the angle-to-length mapping changes. Both use the same frozen planar
attachment/exit geometry, independent multistart least squares and raw angles.
No target registration, smoothing, angle offsets, or calibration publication.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from analyze_y42_start_pose_offline import curve_eval, local_solve, YAW_SCALE


def reconstruct_joint_snapshots(events, maximum_span_s=0.065):
    """One newest sample per axis, each newer than the preceding emitted batch.

    Acquisition timestamps are host receives, not simultaneous device samples.
    No temporal interpolation or smoothing; skipped superseded frames are counted.
    """
    latest, consumed, snapshots = {}, {}, []
    received = superseded = rejected_span = 0
    stream_start = next(e["sequence"] for e in events if e["event"] == "trajectory_feedback_stream_start")
    stream_stop = next(e["sequence"] for e in events if e["event"] == "trajectory_feedback_stream_stop")
    for event in events:
        if not stream_start < event["sequence"] < stream_stop or event["event"] != "timed_feedback_frames":
            continue
        for sample in event["samples"]:
            packet = bytes.fromhex(sample["raw_packet_hex"])
            if len(packet) != 37 or packet[1:4] != b"\x43\x25\x0c" or packet[-1] != 0x6B:
                raise ValueError("Unexpected raw feedback packet")
            axis = packet[0]
            q = (-1 if packet[22] else 1) * int.from_bytes(packet[23:27], "big") / 10.0
            rpm = (-1 if packet[19] else 1) * int.from_bytes(packet[20:22], "big") / 10.0
            current = int.from_bytes(packet[8:10], "big")
            if q != sample["position_deg"] or rpm != sample["rpm"] or current != sample["phase_current_ma"]:
                raise ValueError("Archive field disagrees with original packet")
            received += 1
            if axis in latest and latest[axis]["serial"] > consumed.get(axis, 0):
                superseded += 1
            latest[axis] = {"serial": received, "sequence": event["sequence"], "q": q, "rpm": rpm, "current": current, "t": sample["sampled_monotonic_s"], "packet": packet.hex()}
            if not all(i in latest and latest[i]["serial"] > consumed.get(i, 0) for i in range(1, 5)):
                continue
            rows = [latest[i] for i in range(1, 5)]
            times = [r["t"] for r in rows]
            if max(times) - min(times) > maximum_span_s:
                rejected_span += 1
                continue
            snapshots.append({"index": len(snapshots), "host_receive_start_s": min(times), "host_receive_end_s": max(times), "host_batch_span_ms": 1000 * (max(times) - min(times)), "positions_deg": [r["q"] for r in rows], "rpm": [r["rpm"] for r in rows], "phase_current_ma": [r["current"] for r in rows], "source_sequences": [r["sequence"] for r in rows], "raw_packets_hex": [r["packet"] for r in rows]})
            consumed = {i: latest[i]["serial"] for i in range(1, 5)}
    return snapshots, {"raw_frames": received, "joint_snapshots": len(snapshots), "superseded_unpaired_frames": superseded, "candidate_batches_rejected_for_span": rejected_span, "maximum_allowed_host_batch_span_ms": 1000 * maximum_span_s}


def make_geometry(calibration):
    axes = np.array(calibration["motor_center_coordinates_mm"], dtype=float)
    offset = float(calibration["cable_exit_offset_x_mm"])
    exits = axes + np.array([[offset, 0], [-offset, 0], [-offset, 0], [offset, 0]])
    bx = float(calibration["ball_attachment_span_x_mm"]) / 2
    by = float(calibration["ball_attachment_span_y_mm"]) / 2
    attachments = np.array([[-bx, by], [bx, by], [bx, -by], [-bx, -by]])

    def geometry(pose):
        x, y, yaw_scaled = pose
        c, s = math.cos(yaw_scaled / YAW_SCALE), math.sin(yaw_scaled / YAW_SCALE)
        rotated = attachments @ np.array([[c, -s], [s, c]]).T
        delta = np.array([x, y]) + rotated - exits
        lengths = np.linalg.norm(delta, axis=1)
        unit = delta / lengths[:, None]
        derivative = np.column_stack((-rotated[:, 1], rotated[:, 0])) / YAW_SCALE
        return lengths, np.column_stack((unit, np.sum(unit * derivative, axis=1)))
    return geometry


def model_lengths(calibration, positions, model):
    q = np.array(positions, dtype=float)
    if model == "legacy_piecewise":
        return np.array([curve_eval(calibration["position_samples_deg"][i], calibration["distance_samples_mm"][i], value) for i, value in enumerate(q)])
    if model == "global_affine":
        return (q - np.array(calibration["intercept"])) / np.array(calibration["slope"])
    raise ValueError("Unknown model")


def solve_mapping(calibration, snapshots, model):
    geometry = make_geometry(calibration)
    width, height = calibration["workspace_width"], calibration["workspace_height"]
    seeds = [[width / 2, height / 2, math.radians(yaw) * YAW_SCALE] for yaw in range(-180, 180, 45)]
    output = []
    for snapshot in snapshots:
        lengths = model_lengths(calibration, snapshot["positions_deg"], model)
        # Every observation has the same target-independent initial seeds.
        solutions = sorted((local_solve(lengths, seed, geometry_fn=geometry) for seed in seeds), key=lambda item: item[1])
        pose, rms, residual = solutions[0]
        alternatives = [item for item in solutions[1:] if np.linalg.norm(item[0] - pose) > 0.1]
        output.append({"raw_snapshot_index": snapshot["index"], "x_mm": float(pose[0]), "y_mm": float(pose[1]), "yaw_deg": float(math.degrees(pose[2] / YAW_SCALE)), "closure_rms_mm": rms, "closure_residual_mm": residual.tolist(), "mapped_lengths_mm": lengths.tolist(), "alternative_branch_best_closure_rms_mm": min((item[1] for item in alternatives), default=None)})
    xy = np.array([[r["x_mm"], r["y_mm"]] for r in output])
    closure = np.array([r["closure_rms_mm"] for r in output])
    jumps = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    return {"points": output, "xy": xy.tolist(), "summary": {"point_count": len(output), "start_xy_yaw_deg": [output[0]["x_mm"], output[0]["y_mm"], output[0]["yaw_deg"]], "end_xy_yaw_deg": [output[-1]["x_mm"], output[-1]["y_mm"], output[-1]["yaw_deg"]], "closure_rms_over_all_cable_residuals_mm": float(np.sqrt(np.mean(closure ** 2))), "closure_rms_median_mm": float(np.median(closure)), "closure_rms_p95_mm": float(np.percentile(closure, 95)), "closure_rms_max_mm": float(np.max(closure)), "raw_sample_step_max_mm": float(np.max(jumps)) if len(jumps) else None, "bbox_xy_min_mm": np.min(xy, axis=0).tolist(), "bbox_xy_max_mm": np.max(xy, axis=0).tolist()}}


def compare(run):
    manifest = json.loads((run / "manifest.json").read_text(encoding="utf-8"))
    final_state = json.loads((run / "final_state.json").read_text(encoding="utf-8"))
    events = [json.loads(line) for line in (run / "events.jsonl").read_text(encoding="utf-8").splitlines() if line.strip()]
    calibration = manifest["calibration_before"]
    snapshots, acquisition = reconstruct_joint_snapshots(events)
    models = {name: solve_mapping(calibration, snapshots, name) for name in ("legacy_piecewise", "global_affine")}
    checkpoint = next((event for event in reversed(events) if event["event"] == "tension_checkpoint_saved"), None)
    final_snapshot = None
    if checkpoint is not None:
        # Endpoint safety reads are slower sequential 0x36 replies, kept separate
        # from the streamed trajectory so they cannot affect its shape/noise score.
        latest_positions = {}
        stream_stop = next(e["sequence"] for e in events if e["event"] == "trajectory_feedback_stream_stop")
        for event in events:
            if not stream_stop < event["sequence"] < checkpoint["sequence"] or event["event"] != "serial_transaction":
                continue
            packet = bytes.fromhex(event.get("rx_hex", ""))[:8]
            if len(packet) == 8 and packet[1] == 0x36 and packet[-1] == 0x6B:
                q = (-1 if packet[2] else 1) * int.from_bytes(packet[3:7], "big") / 10.0
                latest_positions[packet[0]] = (q, event["completed_monotonic_ns"] / 1e9, event["sequence"], packet.hex())
        if set(latest_positions) == {1, 2, 3, 4}:
            rows = [latest_positions[i] for i in range(1, 5)]
            if [row[0] for row in rows] != checkpoint["positions_deg"]:
                raise ValueError("Final checkpoint differs from last raw position replies")
            final_snapshot = {"index": len(snapshots), "positions_deg": [row[0] for row in rows], "host_receive_start_s": min(row[1] for row in rows), "host_receive_end_s": max(row[1] for row in rows), "source_sequences": [row[2] for row in rows], "raw_packets_hex": [row[3] for row in rows], "source": "sequential final-settle 0x36 replies; not a simultaneous snapshot"}
            for name in models:
                models[name]["final_settle_point"] = solve_mapping(calibration, [final_snapshot], name)["points"][0]
    xy_difference = np.linalg.norm(np.array(models["legacy_piecewise"]["xy"]) - np.array(models["global_affine"]["xy"]), axis=1)
    return {"analysis_only": True, "calibration_published": False, "absolute_accuracy_verified": False, "run_id": run.name, "events_sha256": hashlib.sha256((run / "events.jsonl").read_bytes()).hexdigest(), "mapping_parameters": {key: calibration[key] for key in ("slope", "intercept", "distance_samples_mm", "position_samples_deg")}, "method": "Identical raw encoder angles; independent multistart least squares at each snapshot; same legacy fixed-exit planar geometry; no smoothing, target fitting, temporal interpolation, pose registration, or session offsets.", "clock_semantics": "Original host receive timestamps, not device timestamps; movement within a batch can contribute closure error.", "caveats": ["Smaller geometric closure is not independently measured ball accuracy.", "Both mappings retain the old assumed-corner coordinate datum, including its potential bias.", "Yaw branch alternatives may exist; no temporal smoother is used to hide branch changes.", "Roll/pitch, winding changes, height and cable deformation remain unobserved.", "summary.end_xy_yaw_deg is the last streamed point; final_settle_point is evaluated separately from slower sequential raw replies."], "acquisition": acquisition, "raw_joint_snapshots": snapshots, "final_settle_snapshot": final_snapshot, "target_segments": final_state.get("trajectory_target_path_segments", []), "target_used_in_inverse_solver": False, "models": models, "model_to_model_xy_difference_mm": {"median": float(np.median(xy_difference)), "max": float(np.max(xy_difference)), "rms": float(np.sqrt(np.mean(xy_difference ** 2)))}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = compare(args.run)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({"output": str(args.output.resolve()), "acquisition": result["acquisition"], "models": {name: value["summary"] for name, value in result["models"].items()}, "model_to_model_xy_difference_mm": result["model_to_model_xy_difference_mm"]}, indent=2))


if __name__ == "__main__":
    main()
