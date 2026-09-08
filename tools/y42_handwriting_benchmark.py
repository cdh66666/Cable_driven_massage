"""Fixed operator-loadable drawing input. No serial, HTTP or execution API.

Run this file to export a NEW record under qa/handwriting_benchmark only.
Loading its payload is not permission to move: it intentionally has no
confirmation flag or action. The ordinary application controls remain in charge.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import uuid


SCHEMA_VERSION = "y42_handwriting_benchmark.v1"
BENCHMARK_ID = "four_stroke_accuracy_v1"
WORKSPACE_WIDTH_MM = 110.0
WORKSPACE_HEIGHT_MM = 148.0
DEFAULT_SPEED_MM_S = 50.0
EXPORT_ROOT = Path(__file__).resolve().parents[1] / "qa" / "handwriting_benchmark"


def _rounded(value: float) -> float:
    rounded = round(float(value), 6)
    return 0.0 if rounded == 0.0 else rounded


def _canonical_json(value: object) -> str:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False)


def geometry_sha256(strokes: list[dict], scale: float) -> str:
    """Hash public geometry, sampling, ordering and coordinate convention."""
    geometry = {
        "schema_version": SCHEMA_VERSION,
        "benchmark_id": BENCHMARK_ID,
        "workspace_mm": {"width": WORKSPACE_WIDTH_MM, "height": WORKSPACE_HEIGHT_MM},
        "scale": scale,
        "coordinate_convention": "mm: x right, y up; normalized: x right, y down",
        "strokes": [{"id": stroke["id"], "points_mm": stroke["points_mm"],
                     "points_normalized": stroke["points_normalized"]} for stroke in strokes],
    }
    return hashlib.sha256(_canonical_json(geometry).encode("utf-8")).hexdigest()


def build_benchmark(scale: float = 1.0) -> dict:
    """Return fixed four-stroke geometry and a non-executing hand-draw payload.

    Scale 0.25..1.0 is centred on the 110 x 148 mm board. The default drawing
    stays at least 18 mm inside its edges. Sampling points are published, not
    smoothed, jittered, fitted or randomized on each load.
    """
    scale = float(scale)
    if not math.isfinite(scale) or not 0.25 <= scale <= 1.0:
        raise ValueError("scale must be finite and within 0.25..1.0")
    scale = _rounded(scale)
    circle = [(35.0 + 16.0 * math.cos(2.0 * math.pi * index / 64.0),
               78.0 + 16.0 * math.sin(2.0 * math.pi * index / 64.0)) for index in range(64)]
    circle.append(circle[0])
    figure_eight = [(77.0 + 15.0 * math.sin(2.0 * math.pi * index / 64.0),
                    75.0 + 18.0 * math.sin(4.0 * math.pi * index / 64.0)) for index in range(64)]
    figure_eight.append(figure_eight[0])
    definitions = [
        ("corner_return", "第1笔 · 长直线 / 直角折返", "long_line_right_angle_return",
         [(20.0, 124.0), (90.0, 124.0), (90.0, 102.0), (55.0, 102.0)]),
        ("closed_circle", "第2笔 · 闭圆", "closed_circle", circle),
        ("figure_eight", "第3笔 · 8字交叉", "self_crossing_figure_eight", figure_eight),
        ("short_turns", "第4笔 · 短笔换段", "short_three_edge_turns",
         [(23.0, 28.0), (31.0, 28.0), (31.0, 34.0), (39.0, 34.0)]),
    ]
    strokes = []
    for stroke_id, label, feature, raw in definitions:
        points_mm = [[_rounded(55.0 + scale * (x - 55.0)),
                      _rounded(74.0 + scale * (y - 74.0))] for x, y in raw]
        points_normalized = [{"x": _rounded(x / WORKSPACE_WIDTH_MM),
                              "y": _rounded(1.0 - y / WORKSPACE_HEIGHT_MM)} for x, y in points_mm]
        strokes.append({"id": stroke_id, "label": label, "feature": feature,
                        "points_mm": points_mm, "points_normalized": points_normalized})
    payload = {
        "segments": [[dict(point) for point in stroke["points_normalized"]] for stroke in strokes],
        "speed": DEFAULT_SPEED_MM_S,
        "repeat": False,
        "controller_mode": "four_position",
    }
    return {
        "schema_version": SCHEMA_VERSION,
        "benchmark_id": BENCHMARK_ID,
        "workspace_mm": {"width": WORKSPACE_WIDTH_MM, "height": WORKSPACE_HEIGHT_MM},
        "coordinate_convention": "mm: x right, y up; normalized: x right, y down",
        "scale": scale,
        "requested_speed_mm_s": DEFAULT_SPEED_MM_S,
        "geometry_sha256": geometry_sha256(strokes, scale),
        "operator_labels": [stroke["label"] for stroke in strokes],
        "strokes": strokes,
        "payload": payload,
        "notice": "仅载入固定验收图，不启动运动；转场不是笔画。电机反算叠图不等于外部实测精度。",
    }


def build_payload(scale: float = 1.0) -> dict:
    """Convenience payload for a read-only load button; never submits it."""
    return build_benchmark(scale)["payload"]


def export_benchmark(scale: float = 1.0) -> dict:
    """Append-only JSON export to the one fixed QA location, never elsewhere."""
    benchmark = build_benchmark(scale)
    expected = Path(__file__).resolve().parents[1] / "qa" / "handwriting_benchmark"
    # Reject redirection via a monkeypatch, symlink or junction. Resolve before
    # mkdir as well as afterwards; never accept an arbitrary CLI output path.
    if EXPORT_ROOT != expected or EXPORT_ROOT.resolve() != expected:
        raise ValueError("Benchmark export must stay under the fixed qa/handwriting_benchmark path")
    EXPORT_ROOT.mkdir(parents=True, exist_ok=True)
    if EXPORT_ROOT.resolve() != expected:
        raise ValueError("Benchmark export root was redirected")
    name = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S_%fZ") + "_" + uuid.uuid4().hex
    directory = EXPORT_ROOT / name
    directory.mkdir(exist_ok=False)
    paths = {}
    for filename, value in (("benchmark.json", benchmark), ("payload.json", benchmark["payload"])):
        path = directory / filename
        with path.open("x", encoding="utf-8", newline="\n") as output:
            output.write(json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2, allow_nan=False) + "\n")
        paths[filename] = str(path)
    return {"directory": str(directory), "geometry_sha256": benchmark["geometry_sha256"], "files": paths}


def main() -> None:
    parser = argparse.ArgumentParser(description="Export a fixed drawing only; never connects to or runs a device.")
    parser.add_argument("--scale", type=float, default=1.0, help="Board-centred scale in 0.25..1.0")
    args = parser.parse_args()
    print(json.dumps(export_benchmark(args.scale), ensure_ascii=False, sort_keys=True))


if __name__ == "__main__":
    main()
