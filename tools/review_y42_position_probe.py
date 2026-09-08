"""Read-only position-probe archive summary. Default CLI prints JSON only.

No controller import, serial, HTTP, plotting, data repair or archive writes.
Host write timestamps are not wire-time or a real-time scheduler guarantee.
"""
import argparse
import json
import math
from pathlib import Path


def finite(value):
    try:
        number = float(value)
    except (ValueError, TypeError):
        return None
    return number if math.isfinite(number) else None


def timing(times):
    values = [number for value in times if (number := finite(value)) is not None]
    gaps = [right - left for left, right in zip(values, values[1:])]
    valid = sorted(gap * 1000.0 for gap in gaps if gap > 0.0)
    def percentile(fraction):
        if not valid:
            return None
        index = fraction * (len(valid) - 1)
        lower = int(index)
        return valid[lower] + (valid[min(lower + 1, len(valid) - 1)] - valid[lower]) * (index - lower)
    return {"count": len(values), "first_time_s": values[0] if values else None,
            "last_time_s": values[-1] if values else None,
            "hz": (len(values) - 1) / (values[-1] - values[0]) if len(values) >= 2 and values[-1] > values[0] and all(gap > 0 for gap in gaps) else None,
            "interval_p50_ms": percentile(0.5), "interval_p95_ms": percentile(0.95),
            "interval_max_ms": max(valid) if valid else None,
            "nonpositive_intervals": sum(gap <= 0.0 for gap in gaps)}


def packet_bytes(event):
    value = event.get("attempted_tx_hex", event.get("tx_hex", event.get("raw_packet_hex", "")))
    try:
        return bytes.fromhex(value) if isinstance(value, str) else b""
    except ValueError:
        return b""


def control_subframes(packet):
    """Only decode exact F3 and CB formats; never search arbitrary data bytes."""
    if len(packet) == 6 and packet[1:3] == b"\xf3\xab" and packet[-1] == 0x6B:
        return [packet]
    if len(packet) == 14 and packet[1] == 0xCB and packet[-1] == 0x6B:
        return [packet]
    if (len(packet) == 61 and packet[:2] == b"\x00\xaa" and packet[-1] == 0x6B
            and int.from_bytes(packet[2:4], "big") in (59, 61)):
        parts = [packet[4 + 14 * index:18 + 14 * index] for index in range(4)]
        if all(part[0] == index + 1 and part[1] == 0xCB and part[-1] == 0x6B for index, part in enumerate(parts)):
            return parts
    return []


def summarize_events(events):
    endings = [event for event in events if event.get("event") == "run_end"]
    ending = endings[-1] if endings else {}
    result = ending.get("result") if isinstance(ending.get("result"), dict) else {}
    alerts = []
    if not endings:
        alerts.append("run_end missing: archive incomplete")
    if ending.get("error"):
        alerts.append("run_end error: " + str(ending["error"]))
    if result.get("reason") or result.get("error"):
        alerts.append(str(result.get("reason") or result.get("error")))
    # The adapter wrapper is authoritative and records even failed writes.
    # Do not count its duplicate direct.transact traffic a second time.
    writes = [event for event in events if event.get("event") == "position_probe_serial_write"]
    preferred = bool(writes)
    if not writes:
        writes = [event for event in events if event.get("event") in {"serial_transaction", "high_rate_tx_frame"}]
        alerts.append("dedicated adapter write records absent; using legacy TX evidence")
    enable_attempted, enable_written, cb_attempted, cb_written = set(), set(), set(), set()
    first_targets = {}
    variation = {axis: 0.0 for axis in (1, 2, 3, 4)}
    probe_writes = []
    failed_writes = 0
    for event in writes:
        packet = packet_bytes(event)
        parts = control_subframes(packet)
        complete = (event.get("bytes_written") == len(packet) and not event.get("error")) if preferred else bool(packet and not event.get("error"))
        if event.get("error") or (preferred and not complete):
            failed_writes += 1
        for part in parts:
            axis, command = part[:2]
            if axis not in (1, 2, 3, 4):
                continue
            if command == 0xF3 and part[3] == 1:
                enable_attempted.add(axis)
                if complete:
                    enable_written.add(axis)
            elif command == 0xCB:
                cb_attempted.add(axis)
                if complete:
                    cb_written.add(axis)
                    value = (-1.0 if part[2] else 1.0) * int.from_bytes(part[5:9], "big") / 10.0
                    first_targets.setdefault(axis, value)
                    variation[axis] = max(variation[axis], abs(value - first_targets[axis]))
        if (len(parts) == 4 and complete and
                (event.get("phase") in {"probe_positions", "microprobe_command_stream"} if preferred else event.get("event") == "high_rate_tx_frame")):
            start = finite(event.get("started_monotonic_ns"))
            end = finite(event.get("completed_monotonic_ns"))
            if start is not None and end is not None:
                probe_writes.append({"started_monotonic_ns": start, "completed_monotonic_ns": end,
                                     "bytes_written": len(packet)})
    if failed_writes:
        alerts.append(f"{failed_writes} TX writes partial/failed/unknown")
    moved_command_axes = [axis for axis, delta in variation.items() if delta > 0.1000001]
    module_commands = result.get("commands")
    if not isinstance(module_commands, list):
        module_commands = [event for event in events if event.get("event") == "position_probe_command"]
    feedback = result.get("feedback")
    if not isinstance(feedback, list):
        feedback = [event for event in events if event.get("event") == "position_probe_feedback"]
    signatures = set()
    group_times = []
    fully_new_times = []
    previous_complete = None
    invalid_feedback = 0
    positions = {axis: [] for axis in (1, 2, 3, 4)}
    unique_per_axis = {axis: set() for axis in (1, 2, 3, 4)}
    observed_maxima = {axis: {"abs_rpm": None, "abs_phase_current_ma": None, "temperature_c": None} for axis in (1, 2, 3, 4)}
    raw_limit_fault = False
    for entry in feedback:
        rows = entry.get("rows", []) if isinstance(entry, dict) else []
        if (len(rows) != 4 or any(not isinstance(row, dict) for row in rows)
                or {row.get("id") for row in rows} != {1, 2, 3, 4}):
            invalid_feedback += 1
            continue
        ordered = sorted(rows, key=lambda row: row["id"])
        signature = tuple(finite(row.get("sampled_at")) for row in ordered)
        if any(stamp is None for stamp in signature):
            invalid_feedback += 1
            continue
        for row in ordered:
            axis = row["id"]
            stamp = finite(row["sampled_at"])
            position = finite(row.get("position"))
            for field, output, absolute in (("rpm", "abs_rpm", True), ("phase_current", "abs_phase_current_ma", True), ("temperature", "temperature_c", False)):
                number = finite(row.get(field))
                if number is not None:
                    number = abs(number) if absolute else number
                    previous = observed_maxima[axis][output]
                    observed_maxima[axis][output] = number if previous is None else max(previous, number)
            if stamp not in unique_per_axis[axis]:
                unique_per_axis[axis].add(stamp)
                if position is not None:
                    positions[axis].append(position)
        if signature in signatures:
            continue
        signatures.add(signature)
        group_times.append(max(signature))
        if previous_complete is None or all(stamp > previous for stamp, previous in zip(signature, previous_complete)):
            fully_new_times.append(max(signature))
            previous_complete = signature
    if invalid_feedback:
        alerts.append(f"{invalid_feedback} invalid feedback groups")
    for axis, maxima in observed_maxima.items():
        if ((maxima["abs_rpm"] is not None and maxima["abs_rpm"] > 3.0)
                or (maxima["abs_phase_current_ma"] is not None and maxima["abs_phase_current_ma"] > 350.0)
                or (maxima["temperature_c"] is not None and maxima["temperature_c"] >= 45.0)):
            raw_limit_fault = True
            alerts.append(f"M{axis} recorded feedback exceeds probe RPM/current/temperature limits")
    if len(group_times) != len(fully_new_times):
        alerts.append("Some unique snapshots reuse individual motor samples; fully-new four-axis rate is listed separately")
    axis_results = result.get("axis_results", [])
    axes = []
    for axis in (1, 2, 3, 4):
        candidates = [item for item in axis_results if isinstance(item, dict) and item.get("id") == axis] if isinstance(axis_results, list) else []
        item = candidates[-1] if candidates else {}
        response = finite(item.get("observed_signed_response_deg"))
        peak_error = finite(item.get("peak_error_deg"))
        return_error = finite(item.get("return_error_deg"))
        confirmed = bool(axis in moved_command_axes and response is not None and response > 0.1000001
                         and peak_error is not None and abs(peak_error) <= 0.25
                         and return_error is not None and abs(return_error) <= 0.25)
        axes.append({"id": axis, "following_evidence_present": confirmed,
                     "observed_signed_response_deg": response if axis in moved_command_axes else None,
                     "peak_error_deg": finite(item.get("peak_error_deg")) if axis in moved_command_axes else None,
                     "return_error_deg": finite(item.get("return_error_deg")) if axis in moved_command_axes else None,
                     "command_variation_deg": variation[axis],
                     "observed_maxima": observed_maxima[axis],
                     "unique_position_samples": len(unique_per_axis[axis]),
                     "observed_position_range_deg": max(positions[axis]) - min(positions[axis]) if positions[axis] else None})
    execution = "motion_commands_sent" if moved_command_axes else "handshake_only" if cb_attempted or enable_attempted else "not_executed"
    passed = bool(execution == "motion_commands_sent" and result.get("success") is True
                  and result.get("final_hold_verified") is True and result.get("final_disabled_confirmed") is True
                  and result.get("configuration_restored") is True and not failed_writes and not raw_limit_fault and not invalid_feedback
                  and all(axis["following_evidence_present"] for axis in axes))
    if not moved_command_axes:
        alerts.append("No nonzero probe motion evidenced: do not report zero-error success")
    if result.get("success") is True and not passed:
        alerts.append("Recorded success is not supported by all motion/response/cleanup evidence")
    if result.get("raw_guard_fault"):
        alerts.append("raw guard: " + str(result["raw_guard_fault"]))
    starts = [write["started_monotonic_ns"] / 1e9 for write in probe_writes]
    completions = [write["completed_monotonic_ns"] / 1e9 for write in probe_writes]
    return {
        "execution": execution, "encoder_probe_pass_evidenced": passed, "xyz_precision_measured": False,
        "enable_tx_attempted_axes": sorted(enable_attempted), "enable_tx_complete_write_axes": sorted(enable_written),
        "cb_tx_attempted_axes": sorted(cb_attempted), "cb_tx_complete_write_axes": sorted(cb_written),
        "note_enable": "TX bytes alone do not prove enable ACK or final motor state",
        "axes": axes,
        "raw_host_write_start_timing": timing(starts), "raw_host_write_complete_timing": timing(completions),
        "raw_probe_write_timestamps": probe_writes,
        "module_callback_start_timing": timing([item.get("scheduled_send_time") for item in module_commands]),
        "module_callback_complete_timing": timing([item.get("completed_send_time") for item in module_commands]),
        "module_callback_timestamps": [{key: item.get(key) for key in ("stage", "scheduled_send_time", "completed_send_time")} for item in module_commands],
        "timing_note": "Host write/flush boundaries and module callback times are separate; neither certifies wire timing or hard real-time 100 Hz",
        "feedback_logged_groups": len(feedback), "unique_four_timestamp_groups": timing(group_times),
        "fully_new_four_axis_groups": timing(fully_new_times),
        "cached_or_duplicate_feedback_groups": len(feedback) - invalid_feedback - len(group_times),
        "original_configurations_present": bool(result.get("original_configurations_hex")),
        "configuration_restore": result.get("configuration_restore"),
        "configuration_restored": result.get("configuration_restored") is True,
        "final_hold_verified": result.get("final_hold_verified") is True,
        "final_disabled_confirmed": result.get("final_disabled_confirmed") is True,
        "alerts": alerts,
    }


def review_archive(directory):
    directory = Path(directory).resolve()
    path = directory / "events.jsonl"
    events = []
    parse_errors = []
    with path.open("r", encoding="utf-8") as source:
        for index, line in enumerate(source, 1):
            if not line.strip():
                continue
            try:
                event = json.loads(line)
                if not isinstance(event, dict):
                    raise ValueError("event is not an object")
                events.append(event)
            except ValueError as error:
                parse_errors.append(f"line {index}: {error}")
    summary = summarize_events(events)
    summary["archive"] = str(directory)
    summary["event_count"] = len(events)
    summary["alerts"].extend(parse_errors)
    if parse_errors:
        summary["encoder_probe_pass_evidenced"] = False
    return summary


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Read an existing run; print JSON, never write to it.")
    parser.add_argument("run_directory", type=Path)
    args = parser.parse_args()
    print(json.dumps(review_archive(args.run_directory), ensure_ascii=False, indent=2, allow_nan=False))
