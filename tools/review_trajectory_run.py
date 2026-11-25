"""Offline review of a saved Y42 run; never imports the controller or opens serial.

Usage: python tools/review_trajectory_run.py RUN_DIRECTORY [--output DIRECTORY]
Raw run files are read only. The default output is qa/trajectory_reviews/RUN_NAME.
"""
from __future__ import annotations

import argparse
import hashlib
import html
import json
import math
import statistics
import sys
from pathlib import Path


def percentile(values, q):
    ordered = sorted(values)
    if not ordered:
        return None
    index = (len(ordered) - 1) * q
    low = math.floor(index)
    high = math.ceil(index)
    return ordered[low] + (ordered[high] - ordered[low]) * (index - low)


def distribution(values):
    values = [float(v) for v in values if isinstance(v, (float, int)) and math.isfinite(v)]
    if not values:
        return {"count": 0, "min": None, "p50": None, "p95": None, "p99": None, "max": None}
    return {"count": len(values), "min": min(values), "mean": statistics.mean(values),
            "p50": percentile(values, .50), "p95": percentile(values, .95),
            "p99": percentile(values, .99), "max": max(values),
            "stddev": statistics.pstdev(values)}


def interval_statistics(stamps):
    stamps = list(stamps)
    deltas = [(b - a) * 1000 for a, b in zip(stamps, stamps[1:])]
    result = distribution(deltas)
    result.update({"frames": len(stamps), "span_s": stamps[-1] - stamps[0] if len(stamps) > 1 else None,
                   "mean_hz": (len(stamps) - 1) / (stamps[-1] - stamps[0]) if len(stamps) > 1 and stamps[-1] > stamps[0] else None,
                   "gap_counts": {str(limit): sum(v > limit + 1e-6 for v in deltas) for limit in (10, 12, 15, 20, 50)},
                   "nonpositive_intervals": sum(v <= 0 for v in deltas)})
    result["worst_intervals"] = sorted(
        [{"after_frame": i + 1, "elapsed_s": stamps[i] - stamps[0], "gap_ms": value}
         for i, value in enumerate(deltas)], key=lambda row: row["gap_ms"], reverse=True)[:10]
    return result, deltas


def is_four_position_packet(event):
    try:
        raw = bytes.fromhex(event.get("raw_packet_hex", ""))
    except ValueError:
        return False
    return len(raw) == 61 and raw[:2] == b"\x00\xaa" and all(
        raw[4 + 14 * index] == index + 1 and raw[5 + 14 * index] == 0xCB for index in range(4))


def valid_point(value):
    return isinstance(value, (tuple, list)) and len(value) >= 2 and all(
        isinstance(n, (float, int)) and math.isfinite(n) for n in value[:2])


def segments(value):
    if not isinstance(value, list) or not value:
        return []
    if valid_point(value[0]):
        return [[tuple(p[:2]) for p in value if valid_point(p)]]
    # Keep empty stroke slots; dropping them would compare different strokes.
    return [[tuple(p[:2]) for p in stroke if valid_point(p)] if isinstance(stroke, list) else [] for stroke in value]


def length(points):
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def resample(points, step=.5):
    if len(points) < 2:
        return points
    total = length(points)
    if not total:
        return points[:1]
    targets = [index * step for index in range(int(total // step) + 1)]
    if total - targets[-1] > 1e-9:
        targets.append(total)
    cumulative = [0.0]
    for a, b in zip(points, points[1:]):
        cumulative.append(cumulative[-1] + math.dist(a, b))
    output, edge = [], 0
    for distance in targets:
        while edge + 1 < len(cumulative) - 1 and cumulative[edge + 1] < distance:
            edge += 1
        interval = cumulative[edge + 1] - cumulative[edge]
        ratio = (distance - cumulative[edge]) / interval if interval else 0.0
        a, b = points[edge:edge + 2]
        output.append((a[0] + ratio * (b[0] - a[0]), a[1] + ratio * (b[1] - a[1])))
    return output


def point_to_polyline(point, line):
    if len(line) == 1:
        return math.dist(point, line[0])
    result = math.inf
    for a, b in zip(line, line[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        squared = dx * dx + dy * dy
        t = max(0.0, min(1.0, ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) / squared)) if squared else 0.0
        result = min(result, math.dist(point, (a[0] + t * dx, a[1] + t * dy)))
    return result


def geometry_metrics(target, actual, tolerance=4.0):
    per_stroke, all_target, all_actual = [], [], []
    missing = False
    for index in range(max(len(target), len(actual))):
        a = target[index] if index < len(target) else []
        b = actual[index] if index < len(actual) else []
        result = {"stroke": index + 1, "target_points": len(a), "raw_model_points": len(b),
                  "target_length_mm": length(a), "model_length_mm": length(b)}
        if not a or not b:
            result.update({"status": "missing_stroke", "rms_mm": None, "max_mm": None})
            missing = True
        else:
            forward = [point_to_polyline(p, b) for p in resample(a)]
            reverse = [point_to_polyline(p, a) for p in resample(b)]
            all_target.extend(forward)
            all_actual.extend(reverse)
            result.update({"status": "paired_by_stroke_index",
                           "rms_mm": math.sqrt((statistics.mean(v*v for v in forward) + statistics.mean(v*v for v in reverse)) / 2),
                           "p95_target_to_model_mm": percentile(forward, .95),
                           "max_mm": max(forward + reverse),
                           "target_coverage_within_4mm": sum(v <= tolerance for v in forward) / len(forward),
                           "start_error_mm": math.dist(a[0], b[0]), "end_error_mm": math.dist(a[-1], b[-1])})
        per_stroke.append(result)
    complete = not missing and bool(all_target) and bool(all_actual)
    return {"coordinate_claim": "motor_encoder_plus_saved_geometry_internal_estimate_only",
            "external_absolute_accuracy_mm": None,
            "method": "stroke-index matched; 0.5 mm arc-length resampling; bidirectional point-to-polyline distances; travel excluded",
            "complete_stroke_pairing": complete, "stroke_metrics": per_stroke,
            "rms_mm": math.sqrt((statistics.mean(v*v for v in all_target) + statistics.mean(v*v for v in all_actual)) / 2) if complete else None,
            "max_mm": max(all_target + all_actual) if complete else None,
            "p95_target_to_model_mm": percentile(all_target, .95) if complete else None,
            "target_coverage_within_4mm": sum(v <= tolerance for v in all_target) / len(all_target) if complete else None}


def read_run(run_dir):
    events = []
    with (run_dir / "events.jsonl").open(encoding="utf-8") as stream:
        for number, row in enumerate(stream, 1):
            if row.strip():
                try:
                    events.append(json.loads(row))
                except json.JSONDecodeError as exc:
                    raise ValueError(f"events.jsonl line {number} is incomplete: {exc}") from exc
    final_file = run_dir / "final_state.json"
    final = json.loads(final_file.read_text(encoding="utf-8")) if final_file.exists() else {}
    return events, final


def create_review(run_dir):
    events, final = read_run(run_dir)
    warnings = []
    start = next((e for e in events if e.get("event") == "run_start"), {})
    end = next((e for e in reversed(events) if e.get("event") == "run_end"), {})
    arguments = start.get("arguments", [])
    spec = arguments[1] if len(arguments) > 1 and isinstance(arguments[1], dict) else {}
    completed = [e for e in events if e.get("event") == "trajectory_complete"]
    completion = completed[-1] if completed else {}
    failures = [e for e in events if e.get("event") in ("trajectory_error", "trajectory_start_timeout")]
    start_samples = [e for e in events if e.get("event") == "trajectory_start_sample"]
    last_start_sample = start_samples[-1] if start_samples else {}
    open_event = next((e for e in events if e.get("event") == "trajectory_open"), {})
    position_targets = open_event.get("target_positions", [])
    start_actual = last_start_sample.get("positions", [])
    failure_summary = {
        "message": next((e.get("error") for e in reversed(failures) if e.get("error")), None) or (final.get("message") if not completion else None),
        "drawing_started": any(e.get("event") == "trajectory_sample" for e in events),
        "start_samples": len(start_samples),
        "last_start_positions_deg": start_actual,
        "start_target_positions_deg": position_targets,
        "last_start_axis_error_deg": [float(actual) - float(target) for actual, target in zip(start_actual, position_targets)],
        "last_start_model_pose": last_start_sample.get("solved_planar_pose"),
        "final_auto": final.get("auto"),
        "run_end_result": end.get("result"),
    }
    if not completion:
        warnings.append("此运行没有绘制成功；准备/起点保持命令不属于已完成的100Hz绘制验收。")
    expected_frames = completion.get("frames")
    boundaries = [e for e in events if e.get("event") in ("trajectory_feedback_stream_start", "trajectory_feedback_stream_skipped")]
    boundary = boundaries[0] if boundaries else next((e for e in events if e.get("event") == "trajectory_start_local_rebase"), None)
    tx = [e for e in events if e.get("event") == "high_rate_tx_frame" and is_four_position_packet(e)]
    if boundary:
        tx = [e for e in tx if e.get("sequence", 0) > boundary.get("sequence", 0)]
    else:
        warnings.append("没有记录控制窗口起点；发送统计包含起点/终点阶段，不能视作纯绘制节拍。")
    if len(completed) > 1 or spec.get("repeat"):
        warnings.append("重复运行含多个周期；本工具无法仅凭最后frames精确拆分所有周期，请使用单次运行录制。")
    if boundary and isinstance(expected_frames, int) and expected_frames > 0 and len(completed) == 1 and not spec.get("repeat"):
        command_events = tx[:expected_frames]
        scope = "first reported frames after initial trajectory feedback-stream setup; ramp and final hold excluded"
        if len(command_events) != expected_frames:
            warnings.append("原始命令记录数少于控制器报告帧数；节拍证据不完整。")
    else:
        command_events = tx
        scope = "available position frames; phase boundary not independently confirmed"
    stamps = [e["started_monotonic_ns"] / 1e9 for e in command_events if isinstance(e.get("started_monotonic_ns"), (int, float))]
    cadence, gaps = interval_statistics(stamps)
    cadence["scope"] = scope
    cadence["reported_frames"] = expected_frames
    cadence["reported_mean_hz"] = completion.get("hz")
    cadence["write_duration_ms"] = distribution([(e["completed_monotonic_ns"] - e["started_monotonic_ns"]) / 1e6 for e in command_events if "started_monotonic_ns" in e and "completed_monotonic_ns" in e])
    control_duration = completion.get("control_s")
    if cadence["span_s"] is not None and isinstance(control_duration, (int, float)) and abs(cadence["span_s"] - control_duration) > max(.05, .05 * control_duration):
        warnings.append("所选原始帧窗口与报告control_s相差超过5%/50ms；请检查阶段划分，勿直接验收100Hz。")
    raw_feedback = {index: [] for index in range(1, 5)}
    for event in events:
        if event.get("event") != "timed_feedback_frames":
            continue
        for sample in event.get("samples", []):
            address, stamp = sample.get("address"), sample.get("sampled_monotonic_s")
            if address in raw_feedback and isinstance(stamp, (int, float)) and (not stamps or stamps[0] <= stamp <= stamps[-1]):
                raw_feedback[address].append(stamp)
    feedback = {str(index): interval_statistics(sorted(set(values)))[0] for index, values in raw_feedback.items()}
    target = segments(final.get("trajectory_target_path_segments") or final.get("trajectory_target_path") or spec.get("draw_segments", []))
    actual_value = final.get("trajectory_actual_feedback_path_segments")
    if actual_value is None:
        actual_value = final.get("trajectory_actual_feedback_path", [])
    actual = segments(actual_value)
    original = segments(spec.get("operator_input_absolute_segments", []))
    if not actual:
        warnings.append("没有原始模型反算轨迹；不使用插值显示线填充精度证据。")
    phases = []
    for phase in spec.get("motion_segments", []):
        a, b = phase.get("start_s"), phase.get("end_s")
        seconds = b - a if isinstance(a, (int, float)) and isinstance(b, (int, float)) else None
        points = segments(phase.get("path", []))
        distance = sum(length(p) for p in points)
        phases.append({"kind": phase.get("kind"), "stroke_index": phase.get("stroke_index"),
                       "planned_start_s": a, "planned_end_s": b, "planned_duration_s": seconds,
                       "distance_mm": distance, "planned_mean_speed_mm_s": distance / seconds if seconds else None})
    integrity_path = run_dir / "integrity.json"
    integrity = json.loads(integrity_path.read_text(encoding="utf-8")) if integrity_path.exists() else {}
    checks = {}
    for filename, expected in [("events.jsonl", integrity.get("events", {}).get("sha256")), ("final_state.json", integrity.get("final_state_sha256"))]:
        checks[filename] = hashlib.sha256((run_dir / filename).read_bytes()).hexdigest() == expected if expected and (run_dir / filename).exists() else None
    if any(value is False for value in checks.values()):
        warnings.append("原始记录哈希与integrity不一致；必须核查后再使用结果。")
    result = {"run": run_dir.name, "raw_directory": str(run_dir), "complete": bool(completion),
              "acceptance_status": "drawing_complete_internal_evidence_only" if completion else "drawing_not_completed",
              "failure_summary": failure_summary,
              "requested_speed_mm_s": spec.get("requested_speed_mm_s"), "controller": completion.get("controller", final.get("trajectory_controller")),
              "command_cadence_ms": cadence, "per_motor_raw_feedback_cadence_ms": feedback,
              "display_reconstruction_hz": completion.get("feedback_hz"), "four_motor_raw_update_hz": completion.get("feedback_raw_hz"),
              "recorded_internal_metrics": final.get("trajectory_diagnostic", {}), "recomputed_internal_metrics": geometry_metrics(target, actual),
              "start_anchor_internal_error_mm": completion.get("start_anchor_error_mm", final.get("trajectory_start_anchor_error_mm")),
              "phase_timing": {"run_wall_s": (end.get("monotonic_ns", 0) - start.get("monotonic_ns", 0)) / 1e9 if start and end else None,
                               "run_to_first_control_s": stamps[0] - start["monotonic_ns"] / 1e9 if stamps and start.get("monotonic_ns") else None,
                               "planned_control_s": spec.get("duration_ms", 0) / 1000 or None,
                               "reported_control_s": control_duration, "reported_final_settle_s": completion.get("final_settle_s"),
                               "motion_segments_are_planned_not_measured": True, "segments": phases},
              "integrity_checks": checks, "warnings": warnings,
              "limitations": ["没有相机或独立量具：所有XY残差都是电机加同一几何模型的内部指标，不能证明外部绝对精度。",
                              "发送时间戳是主机串口write调用起点，不是驱动器接收/执行时间；平均100Hz不代表每个间隔都为10ms。",
                              "轨迹数据按笔配对且不跨笔连线；真实小球仍会经过换位路径，硬件没有抬笔轴。",
                              "当前原始日志没有每tick的motion kind；逐笔换位时长是规划值，实际逐笔耗时不能由插值伪造。"]}
    return result, target, actual, original, phases, stamps, gaps


def number(value, digits=3):
    return "未知" if value is None else f"{value:.{digits}f}" if isinstance(value, (float, int)) else html.escape(str(value))


def write_svg(path, report, target, actual, original, phases, stamps, gaps):
    svg = ['<svg xmlns="http://www.w3.org/2000/svg" width="1180" height="870" viewBox="0 0 1180 870">',
           '<rect width="1180" height="870" fill="#f8fafc"/><g font-family="Microsoft YaHei,Arial,sans-serif" fill="#182b49">',
           '<text x="35" y="38" font-size="22">分笔叠图与真实发送间隔</text>',
           '<text x="35" y="64" font-size="13">紫线：原始电机模型反算；不是独立绝对位置测量。灰虚线：物理换位，不计绘制误差。</text>']
    if not report['complete']:
        message = '未绘制成功：' + (report.get('failure_summary', {}).get('message') or '运行没有完成记录')
        svg.append('<text x="35" y="89" font-size="15" font-weight="bold" fill="#b42318">'+html.escape(message)+'</text>')
    x0, y0, scale = 70, 105, 4.2
    svg.append(f'<rect x="{x0}" y="{y0}" width="{110*scale}" height="{148*scale}" rx="10" fill="white" stroke="#cbd5e1"/>')
    def plot(strokes, color, width=2, dash="", opacity=1):
        for index, points in enumerate(strokes):
            if not points:
                continue
            coordinates = " ".join(f"{x0+x*scale:.2f},{y0+(148-y)*scale:.2f}" for x, y in points)
            svg.append(f'<polyline fill="none" stroke="{color}" stroke-width="{width}" opacity="{opacity}" stroke-dasharray="{dash}" points="{coordinates}"/>')
            if color == '#b52fd5':
                px, py = points[0]
                svg.append(f'<text x="{x0+px*scale+4:.2f}" y="{y0+(148-py)*scale-5:.2f}" font-size="12" fill="{color}">{index+1}</text>')
    plot(original, '#2563eb', 4, opacity=.40)
    plot(target, '#d99221', 2.4, '6 4')
    for phase in report.get("phase_timing", {}).get("segments", []):
        if phase["kind"] != 'travel':
            continue
        index = phase.get("stroke_index", 0)
        if isinstance(index, int) and index > 0 and index < len(target) and target[index-1] and target[index]:
            plot([[target[index-1][-1], target[index][0]]], '#94a3b8', 1.4, '5 7')
    plot(actual, '#b52fd5', 2)
    labels = [('原始手绘', '#2563eb'), ('规划目标', '#d99221'), ('原始模型反算', '#b52fd5')]
    for i, (text, color) in enumerate(labels):
        svg.append(f'<text x="{70+i*160}" y="760" font-size="15" fill="{color}">{text}</text>')
    cadence = report['command_cadence_ms']
    rows = [('发送均值', number(cadence['mean_hz'])+' Hz'), ('间隔 p95 / p99', number(cadence['p95'])+' / '+number(cadence['p99'])+' ms'),
            ('最长间隔', number(cadence['max'])+' ms'), ('间隔 >15 / >20 / >50ms', ' / '.join(str(cadence['gap_counts'][str(n)]) for n in (15,20,50))),
            ('控制 / 结束阶段', number(report['phase_timing']['reported_control_s'])+' / '+number(report['phase_timing']['reported_final_settle_s'])+' s'),
            ('反算显示 / 原始四台更新', number(report['display_reconstruction_hz'])+' / '+number(report['four_motor_raw_update_hz'])+' Hz')]
    for index, (label, value) in enumerate(rows):
        svg.append(f'<text x="595" y="{115+index*45}" font-size="15">{html.escape(label)}</text><text x="595" y="{134+index*45}" font-size="16" font-weight="bold">{html.escape(value)}</text>')
    left, top, width, height = 605, 480, 520, 240
    maximum = max(22, max(gaps, default=0) * 1.08)
    svg.append(f'<rect x="{left}" y="{top}" width="{width}" height="{height}" fill="white" stroke="#cbd5e1"/>')
    for limit in (10,15,20):
        y = top+height-limit/maximum*height
        svg.append(f'<path d="M{left},{y:.1f}h{width}" stroke="'+('#e85b5b' if limit==20 else '#cbd5e1')+'" stroke-dasharray="5 4"/>')
        svg.append(f'<text x="{left-35}" y="{y+4:.1f}" font-size="12">{limit}</text>')
    if gaps:
        points=' '.join(f'{left+i/max(1,len(gaps)-1)*width:.2f},{top+height-gap/maximum*height:.2f}' for i,gap in enumerate(gaps))
        svg.append(f'<polyline fill="none" stroke="#365fa3" stroke-width="1.25" points="{points}"/>')
    svg.append('<text x="605" y="752" font-size="14">每次真实串口write起点的相邻间隔（ms）；横轴为帧序号</text>')
    svg.append('<text x="35" y="815" font-size="13">报告保留最长间隔，不将插值显示Hz冒充原始反馈Hz；最终还需现场比较抖动、换向、松绳和换笔效果。</text></g></svg>')
    path.write_text('\n'.join(svg), encoding='utf-8')


def main():
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_directory', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    run_dir = args.run_directory.resolve()
    output = (args.output or Path(__file__).resolve().parents[1] / 'qa' / 'trajectory_reviews' / run_dir.name).resolve()
    report, target, actual, original, phases, stamps, gaps = create_review(run_dir)
    output.mkdir(parents=True, exist_ok=True)
    (output / 'review.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    write_svg(output / 'review.svg', report, target, actual, original, phases, stamps, gaps)
    rows = ''.join('<tr><td>'+html.escape(label)+'</td><td>'+html.escape(value)+'</td></tr>' for label,value in [
        ('运行', report['run']), ('验收状态', '绘制完成，仅有内部模型证据' if report['complete'] else '未绘制成功：'+str(report['failure_summary']['message'])),
        ('控制方案 / 速度', str(report['controller'])+' / '+number(report['requested_speed_mm_s'])+' mm/s'),
        ('发送平均频率', number(report['command_cadence_ms']['mean_hz'])+' Hz'),
        ('相邻间隔 p95 / p99 / 最大', ' / '.join(number(report['command_cadence_ms'][key]) for key in ('p95','p99','max'))+' ms'),
        ('内部几何 RMS / 最大', number(report['recomputed_internal_metrics']['rms_mm'])+' / '+number(report['recomputed_internal_metrics']['max_mm'])+' mm'),
        ('外部绝对定位精度', '未测量，不能由本报告证明'), ('原始文件完整性', str(report['integrity_checks']))])
    notes=''.join('<li>'+html.escape(text)+'</li>' for text in report['warnings']+report['limitations'])
    document='<!doctype html><meta charset="utf-8"><title>轨迹独立离线复核</title><style>body{max-width:1180px;margin:30px auto;font:16px/1.55 Microsoft YaHei,Arial;background:#f8fafc;color:#182b49}table{border-collapse:collapse;width:100%}td{padding:10px;border-bottom:1px solid #cbd5e1}img{max-width:100%}pre{white-space:pre-wrap;font-size:13px}</style><h1>轨迹独立离线复核</h1><table>'+rows+'</table><img src="review.svg" alt="分笔叠图及发送间隔"><ul>'+notes+'</ul><details><summary>完整指标与阶段规划</summary><pre>'+html.escape(json.dumps(report,ensure_ascii=False,indent=2))+'</pre></details>'
    (output / 'review.html').write_text(document, encoding='utf-8')
    print(json.dumps({'output': str(output), 'run': report['run'], 'cadence_ms': report['command_cadence_ms'],
                      'internal_metrics': {key:value for key,value in report['recomputed_internal_metrics'].items() if key!='stroke_metrics'},
                      'warnings': report['warnings']}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
