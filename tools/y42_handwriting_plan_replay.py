"""Offline Cartesian planning report only. No controller/server import or IO.

The AST fixture runs the production pure planner and the exact normalization /
deduplication block. Export is append-only under qa/handwriting_benchmark.
Neither the report nor its 100 Hz tick count is hardware or accuracy evidence.
"""
import ast
import hashlib
import json
import math
from datetime import datetime, timezone
from pathlib import Path
import uuid

import y42_handwriting_benchmark as benchmark
from test_y42_motion_phases import environment


SOURCE = Path(__file__).with_name("y42_pc_direct_web.py")
SPEEDS = (12.5, 25.0, 50.0)


def conversion_environment(tree):
    env = environment(tree)
    for node in tree.body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
            if node.targets[0].id in {"WORKSPACE_WIDTH_MM", "WORKSPACE_HEIGHT_MM"}:
                env[node.targets[0].id] = ast.literal_eval(node.value)
    helpers = [node for node in tree.body if isinstance(node, ast.FunctionDef)
               and node.name in {"workspace_dimensions", "drawable_workspace_dimensions", "drawable_workspace"}]
    exec(compile(ast.Module(body=helpers, type_ignores=[]), "<pure-workspace>", "exec"), env)
    handle = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "_handle_path_locked")
    start = next(index for index, node in enumerate(handle.body)
                 if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Tuple)
                 and isinstance(node.targets[0].elts[0], ast.Name) and node.targets[0].elts[0].id == "margin_x")
    end = next(index for index, node in enumerate(handle.body)
               if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name) and node.targets[0].id == "draw_total")
    wrapper = ast.parse("def convert(usable_segments, calibration):\n    pass").body[0]
    wrapper.body = handle.body[start:end] + [ast.Return(value=ast.Tuple(elts=[ast.Name(id=name, ctx=ast.Load())
        for name in ("converted_segments", "operator_input_absolute_segments", "filtered_point_count", "refined_point_count")], ctx=ast.Load()))]
    ast.fix_missing_locations(wrapper)
    exec(compile(ast.Module(body=[wrapper], type_ignores=[]), "<production-input-conversion>", "exec"), env)
    speed_rule = next(node for node in handle.body if isinstance(node, ast.If)
                      and ast.unparse(node.test) == "benchmark_identity is not None"
                      and any(isinstance(child, ast.Assign) and isinstance(child.targets[0], ast.Name)
                              and child.targets[0].id == "travel_speed" for child in node.body))
    speed_wrapper = ast.parse("def benchmark_travel_speed(travel_speed, speed, benchmark_identity):\n    pass").body[0]
    speed_wrapper.body = [speed_rule, ast.Return(value=ast.Name(id="travel_speed", ctx=ast.Load()))]
    ast.fix_missing_locations(speed_wrapper)
    exec(compile(ast.Module(body=[speed_wrapper], type_ignores=[]), "<production-benchmark-speed-rule>", "exec"), env)
    return env


def build_report(source_bytes=None):
    source_bytes = SOURCE.read_bytes() if source_bytes is None else source_bytes
    tree = ast.parse(source_bytes.decode("utf-8"))
    env = conversion_environment(tree)
    drawing = benchmark.build_benchmark()
    segments, absolute_input, filtered_count, refined_count = env["convert"](drawing["payload"]["segments"], {})
    draw_distance = sum(math.dist(a, b) for segment in segments for a, b in zip(segment, segment[1:]))
    runs = []
    for speed in SPEEDS:
        # Fixed-benchmark-only speed isolation. Ordinary hand-drawing retains
        # its own travel limit; no saved settings or runtime parameters change.
        travel_speed = env["benchmark_travel_speed"](env["CUSTOM_PATH_TRAVEL_SPEED_MM_S"], speed,
                                                     {"id": drawing["benchmark_id"]})
        phases, _, duration, travel_distance = env["plan_custom_motion_segments"](absolute_input, speed, travel_speed, exact_vertices=True)
        duration_ms = math.ceil(duration * 1000.0)
        phase_reports = [{
            "kind": phase["kind"], "stroke": phase["stroke_index"] + 1,
            "label": drawing["operator_labels"][phase["stroke_index"]],
            "start_s": phase["start_s"], "end_s": phase["end_s"],
            "duration_s": phase["end_s"] - phase["start_s"],
            "distance_mm": sum(math.dist(a, b) for a, b in zip(phase["path"], phase["path"][1:])),
            "cruise_speed_mm_s": phase["cruise_speed_mm_s"],
            "acceleration_mm_s2": phase["acceleration_mm_s2"],
            "start_speed_mm_s": phase["speeds"][0], "end_speed_mm_s": phase["speeds"][-1],
            "start_xy_mm": phase["path"][0], "end_xy_mm": phase["path"][-1],
            "planned_node_count": len(phase["path"]),
        } for phase in phases]
        runs.append({
            "requested_draw_speed_mm_s": speed, "travel_speed_mm_s": travel_speed,
            "draw_distance_mm": draw_distance, "travel_distance_mm": travel_distance,
            "draw_time_s": sum(p["duration_s"] for p in phase_reports if p["kind"] == "draw"),
            "travel_time_s": sum(p["duration_s"] for p in phase_reports if p["kind"] == "travel"),
            "total_plan_time_s": duration, "duration_ms": duration_ms,
            "expected_100hz_command_frames": (duration_ms + 9) // 10,
            "phases": phase_reports,
        })
    return {
        "schema_version": "y42_handwriting_plan_replay.v1",
        "source_path": str(SOURCE), "source_sha256": hashlib.sha256(source_bytes).hexdigest(),
        "benchmark_id": drawing["benchmark_id"], "geometry_sha256": drawing["geometry_sha256"],
        "input_point_count": sum(len(segment) for segment in drawing["payload"]["segments"]),
        "filtered_point_count": filtered_count, "refined_point_count": refined_count,
        "workspace_mm": drawing["workspace_mm"],
        "input_normalized_segments": drawing["payload"]["segments"],
        "production_converted_input_mm": absolute_input,
        "travel_policy": "fixed benchmark only: min(global travel cap, requested draw speed)",
        "hardware_executed": False, "calibration_or_motor_limit_validated": False,
        "scope": "Cartesian planning only; 100 Hz frames exclude startup, settling, IO delays, protection and skipped ticks. No XY precision or low-speed safety certification.",
        "runs": runs,
    }


def export_report():
    report = build_report()
    root = Path(__file__).resolve().parents[1] / "qa" / "handwriting_benchmark"
    if root.resolve() != root:
        raise ValueError("QA output directory must not be redirected")
    root.mkdir(parents=True, exist_ok=True)
    directory = root / (datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S_%fZ") + "_plan_replay_" + uuid.uuid4().hex)
    directory.mkdir(exist_ok=False)
    with (directory / "plan_replay.json").open("x", encoding="utf-8", newline="\n") as output:
        output.write(json.dumps(report, ensure_ascii=False, sort_keys=True, indent=2, allow_nan=False) + "\n")
    lines = ["# 固定四笔验收图：仅离线规划", "", "尚未执行设备；不代表真实位置精度或低速安全认证。", "",
             f"源码 SHA-256：`{report['source_sha256']}`", f"固定图 SHA-256：`{report['geometry_sha256']}`", "",
             "三档绘制与转场速度一致；不改变普通手绘 150 mm/s 转场策略。", "",
             "| 绘制/转场 mm/s | 绘制 s | 转场 s | 规划总计 s | 100 Hz 期望帧数 |", "| --- | --- | --- | --- | --- |"]
    for run in report["runs"]:
        lines.append(f"| {run['requested_draw_speed_mm_s']:g} | {run['draw_time_s']:.3f} | {run['travel_time_s']:.3f} | {run['total_plan_time_s']:.3f} | {run['expected_100hz_command_frames']} |")
    for run in report["runs"]:
        lines.extend(["", f"## {run['requested_draw_speed_mm_s']:g} mm/s", "", "| 笔 / 阶段 | 距离 mm | 预计 s |", "| --- | --- | --- |"])
        for phase in run["phases"]:
            lines.append(f"| {phase['stroke']} / {'绘制' if phase['kind'] == 'draw' else '转场'} | {phase['distance_mm']:.3f} | {phase['duration_s']:.3f} |")
    lines.extend(["", "帧数只按 10 ms 指令周期计算，不含起点移动、最终核验、真实串口耗时、保护或丢帧。", ""])
    with (directory / "plan_replay.md").open("x", encoding="utf-8", newline="\n") as output:
        output.write("\n".join(lines))
    return directory, report


if __name__ == "__main__":
    output_directory, result = export_report()
    print(json.dumps({"directory": str(output_directory), "source_sha256": result["source_sha256"],
                      "geometry_sha256": result["geometry_sha256"],
                      "runs": [{key: value for key, value in run.items() if key != "phases"} for run in result["runs"]]},
                     ensure_ascii=False, sort_keys=True))
