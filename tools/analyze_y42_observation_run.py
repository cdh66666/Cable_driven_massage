#!/usr/bin/env python3
"""Read-only analysis of torque-observation archives, never a calibration.

Usage: python analyze_y42_observation_run.py RUN_DIRECTORY_OR_EVENTS_JSONL
captured_start/end are host acquisition timestamps, not device sample clocks.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import statistics

import numpy as np


FEEDBACK_EVENTS = {"torque_observation_feedback", "observation_snapshot"}
WINDOW_EVENTS = {"torque_observation_static_window", "stable_window"}
RAW_EVENTS = {"torque_observation_raw_batch"}


def parse_snapshot(snapshot):
    begin, end = float(snapshot["captured_start"]), float(snapshot["captured_end"])
    if not math.isfinite(begin) or not math.isfinite(end) or begin > end:
        raise ValueError("Invalid host acquisition interval")
    motors = snapshot["motors"]
    if len(motors) != 4 or [int(row["address"]) for row in motors] != [1, 2, 3, 4]:
        raise ValueError("Missing or unordered four-axis feedback")
    parsed = {"start": begin, "end": end}
    for output, source in (("q", "position"), ("rpm", "rpm"), ("current", "phase_current"), ("flags", "flags")):
        values = [float(row[source]) for row in motors]
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"Nonfinite {source}")
        parsed[output] = values
    return parsed


def window_statistics(event, window_index):
    rows = [parse_snapshot(snapshot) for snapshot in event.get("window", [])]
    if not rows:
        raise ValueError("Static window has no raw snapshots")
    if any(rows[i + 1]["start"] < rows[i]["end"] for i in range(len(rows) - 1)):
        raise ValueError("Static-window acquisition intervals overlap or go backwards")
    q = np.array([row["q"] for row in rows])
    medians = np.median(q, axis=0)
    mad_sigma = 1.4826 * np.median(np.abs(q - medians), axis=0)
    currents = np.abs(np.array([row["current"] for row in rows]))
    return {
        "window_index": window_index,
        "vector_index": event.get("vector_index"),
        "command_currents_ma": event.get("currents_ma"),
        "raw_snapshot_count": len(rows),
        "host_stability_coverage_s": max(0.0, rows[-1]["start"] - rows[0]["end"]),
        "host_acquisition_batch_ms_range": [1000 * min(row["end"] - row["start"] for row in rows), 1000 * max(row["end"] - row["start"] for row in rows)],
        "position_median_deg": medians.tolist(),
        "position_span_deg": np.ptp(q, axis=0).tolist(),
        "position_std_deg": np.std(q, axis=0, ddof=1 if len(q) > 1 else 0).tolist(),
        "position_mad_sigma_deg": mad_sigma.tolist(),
        "maximum_abs_rpm": float(np.max(np.abs([row["rpm"] for row in rows]))),
        "current_min_ma": np.min(currents, axis=0).tolist(),
        "current_max_ma": np.max(currents, axis=0).tolist(),
        "all_enabled_no_protection": all(int(flag) & 1 and not int(flag) & 0x08 for row in rows for flag in row["flags"]),
        "any_driver_stall_bit": any(int(flag) & 0x04 for row in rows for flag in row["flags"]),
        "physical_pose_known": False,
        "tautness_verified": False,
    }


def read_events(event_file):
    """Retain valid archive records and expose malformed/truncated lines."""
    events, issues = [], []
    lines = Path(event_file).read_text(encoding="utf-8").splitlines()
    for line_number, line in enumerate(lines, 1):
        if not line.strip():
            continue
        try:
            event = json.loads(line)
            if not isinstance(event, dict):
                raise ValueError("Event is not an object")
            events.append(event)
        except (ValueError, TypeError) as error:
            issues.append({"line": line_number, "last_line": line_number == len(lines), "error": str(error)})
    return events, issues


def angle_statistics(snapshots):
    q = np.array([row["q"] for row in snapshots])
    intervals = [1000 * (row["end"] - row["start"]) for row in snapshots]
    return {
        "host_acquisition_batch_ms": {"min": min(intervals), "median": statistics.median(intervals), "max": max(intervals)},
        "observed_encoder_span_deg": np.ptp(q, axis=0).tolist(),
        "observed_excursion_from_first_deg": np.max(np.abs(q - q[0]), axis=0).tolist(),
        "sampled_cumulative_angle_lower_bound_deg": np.sum(np.abs(np.diff(q, axis=0)), axis=0).tolist(),
        "all_feedback_maximum_abs_rpm": float(np.max(np.abs([row["rpm"] for row in snapshots]))),
        "maximum_abs_current_ma_by_axis": np.max(np.abs([row["current"] for row in snapshots]), axis=0).tolist(),
    }


def analyze_events(events, *, encoder_resolution_deg=0.1, cluster_separation_sigma=5.0, archive_issues=None):
    snapshots, raw_batches, windows, invalid, terminal = [], [], [], [], []
    raw_count = 0
    config = None
    requested_vectors = None
    for event in events:
        name = event.get("event", "")
        if name == "torque_observation_begin":
            config = event.get("config")
            requested_vectors = len(event["vectors"]) if isinstance(event.get("vectors"), list) else None
        if name in RAW_EVENTS:
            raw_count += 1
            try:
                raw_batches.append(parse_snapshot(event))
            except (KeyError, TypeError, ValueError) as error:
                invalid.append({"sequence": event.get("sequence"), "event": name, "error": str(error)})
        if name in FEEDBACK_EVENTS:
            try:
                snapshots.append(parse_snapshot(event))
            except (KeyError, TypeError, ValueError) as error:
                invalid.append({"sequence": event.get("sequence"), "event": name, "error": str(error)})
        elif name in WINDOW_EVENTS:
            try:
                windows.append(window_statistics(event, len(windows)))
            except (KeyError, TypeError, ValueError) as error:
                invalid.append({"sequence": event.get("sequence"), "event": name, "error": str(error)})
        if "observation" in name and name.endswith(("_complete", "_finished", "_stop", "_end")):
            terminal.append({key: value for key, value in event.items() if key not in ("snapshots", "samples", "window")})
    summary = {
        "analysis_only": True,
        "calibration_fit_performed": False,
        "absolute_ball_xy_accuracy_verified": False,
        "clock_semantics": "captured_start/end are actual host acquisition intervals, NOT device sample timestamps; batch span is NOT known per-axis device skew",
        "event_count": len(events),
        "joint_feedback_records": len(snapshots),
        "raw_batch_records": raw_count,
        "structurally_parseable_raw_batches": len(raw_batches),
        "raw_batches_not_accepted_as_feedback_count": max(0, raw_count - len(snapshots)),
        "static_windows": windows,
        "invalid_records": invalid,
        "sampler_config": config,
        "terminal_events": terminal,
        "archive_parse_issues": list(archive_issues or []),
        "limitations": ["Motor phase current is only a load proxy, not proof of rope tension.", "Encoder configuration clusters are not independently observed ball poses.", "Unknown roll/pitch invalidate the assumed three-DOF bundle closure count."],
    }
    ends = [event for event in terminal if event.get("event") == "torque_observation_end"]
    status = ends[-1].get("status") if ends else None
    summary["completion"] = {
        "end_record_present": bool(ends),
        "recorded_status": status,
        "requested_vector_count": requested_vectors,
        "recorded_static_window_count": len(windows),
        "missing_requested_static_windows": max(0, requested_vectors - len(windows)) if requested_vectors is not None else None,
        "sampling_complete_as_recorded": bool(ends and status == "observed" and requested_vectors is not None and len(windows) == requested_vectors and not archive_issues),
        "missing_end_is_not_proof_of_safety_stop": True,
    }
    if snapshots:
        summary.update(angle_statistics(snapshots))
    if raw_batches:
        summary["all_parseable_raw_batches_including_rejected"] = angle_statistics(raw_batches)
        summary["raw_batch_safety_note"] = "Raw batches precede safety validation; their values are evidence, not accepted safe or synchronized samples."
    if windows:
        centres = np.array([window["position_median_deg"] for window in windows])
        measured_sigma = np.median([window["position_mad_sigma_deg"] for window in windows], axis=0)
        whitening_floor = np.maximum(measured_sigma, encoder_resolution_deg)
        groups = []
        for index, centre in enumerate(centres):
            closest = next((group for group in groups if np.linalg.norm((centre - centres[group[0]]) / whitening_floor) < cluster_separation_sigma), None)
            if closest is None:
                groups.append([index])
            else:
                closest.append(index)
        cluster_centres = np.array([np.mean(centres[group], axis=0) for group in groups])
        centered = cluster_centres - np.mean(cluster_centres, axis=0)
        summary["static_noise_mad_sigma_deg"] = measured_sigma.tolist()
        summary["cluster_whitening_floor_deg"] = whitening_floor.tolist()
        summary["cluster_rule"] = f"Conservative encoder-only greedy clusters; whitened separation >= {cluster_separation_sigma:g}, floor {encoder_resolution_deg:g} deg. This is not a physical pose count."
        summary["encoder_configuration_clusters"] = groups
        summary["encoder_configuration_cluster_count"] = len(groups)
        summary["cluster_centre_span_deg"] = np.ptp(cluster_centres, axis=0).tolist()
        summary["cluster_pca_singular_values_deg"] = np.linalg.svd(centered, compute_uv=False).tolist()
        summary["conditional_3dof_bundle_global_rank_upper_bound"] = min(8, len(groups))
        summary["count_alone_rules_out_8global_identification"] = len(groups) < 8
        summary["encoder_repeat_visits"] = [group for group in groups if len(group) > 1]
    else:
        summary["encoder_configuration_cluster_count"] = 0
        summary["conditional_3dof_bundle_global_rank_upper_bound"] = 0
        summary["count_alone_rules_out_8global_identification"] = True
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    args = parser.parse_args()
    event_file = args.run / "events.jsonl" if args.run.is_dir() else args.run
    events, issues = read_events(event_file)
    result = analyze_events(events, archive_issues=issues)
    result["source_events"] = str(event_file.resolve())
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
