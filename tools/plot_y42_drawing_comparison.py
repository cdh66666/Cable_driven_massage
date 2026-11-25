"""Generate model-only 2-D drawing comparisons.

This report deliberately uses the existing :mod:`y42_twin_model` interfaces.
It compares the original input polyline with the model pose produced from a
coarse and a fine refinement spacing.  It never opens a serial port or calls
HTTP; every label and JSON record states that the result is a model preview,
not a measurement.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

from y42_twin_model import preset, simulate  # noqa: E402


OUT = ROOT / "qa" / "twin"
PNG_PATH = OUT / "drawing-comparison.png"
INPUT_PATH = OUT / "drawing-comparison-input.json"
OUTPUT_PATH = OUT / "drawing-comparison-output.json"

COARSE_SPACING_MM = 1.0
FINE_SPACING_MM = 0.25
COMMON_PARAMETERS = {"speed": 10.0, "acceleration": 80.0, "height": 0.0}


def handdrawn_points() -> list[list[float]]:
    """Return a deterministic, closed freehand-style input polyline.

    Integer harmonics keep the endpoints coincident while the low-amplitude
    perturbations retain a visibly hand-drawn contour.  All points remain in
    the model's documented XY preview range and on the z=25 contact plane.
    """

    t = np.linspace(0.0, 2.0 * math.pi, 97)
    x = 55.0 + 29.0 * np.cos(t) + 2.4 * np.sin(4.0 * t + 0.2) + 1.2 * np.sin(7.0 * t)
    y = 74.0 + 23.0 * np.sin(t) + 2.6 * np.sin(3.0 * t + 0.6) + 1.1 * np.cos(8.0 * t)
    z = np.full_like(t, 25.0)
    return np.column_stack((x, y, z)).round(6).tolist()


def drawing_cases() -> list[dict]:
    return [
        {"id": "circle", "label": "圆形", "source": "preset('circle')", "points": preset("circle")},
        {"id": "square", "label": "方框", "source": "preset('square')", "points": preset("square")},
        {"id": "snake", "label": "蛇形", "source": "preset('snake')", "points": preset("snake")},
        {
            "id": "handdrawn",
            "label": "手绘样式曲线",
            "source": "plot_y42_drawing_comparison.handdrawn_points()",
            "points": handdrawn_points(),
        },
    ]


def polyline_length(points: np.ndarray) -> float:
    return float(np.linalg.norm(np.diff(points[:, :2], axis=0), axis=1).sum())


def point_to_polyline_distance(points_xy: np.ndarray, polyline_xy: np.ndarray) -> np.ndarray:
    """Return each point's shortest Euclidean distance to a 2-D polyline."""

    a = polyline_xy[:-1]
    ab = polyline_xy[1:] - a
    denom = np.einsum("ij,ij->i", ab, ab)
    safe_denom = np.where(denom > 1e-12, denom, 1.0)
    # The reports are small enough for one vectorized pass, but chunking keeps
    # the function bounded if a future drawing has many more vertices.
    result = []
    for start in range(0, len(points_xy), 2048):
        p = points_xy[start : start + 2048]
        rel = p[:, None, :] - a[None, :, :]
        u = np.clip(np.einsum("nsi,si->ns", rel, ab) / safe_denom[None, :], 0.0, 1.0)
        projection = a[None, :, :] + u[:, :, None] * ab[None, :, :]
        distance_sq = np.sum((p[:, None, :] - projection) ** 2, axis=2)
        result.append(np.sqrt(np.min(distance_sq, axis=1)))
    return np.concatenate(result) if result else np.empty(0, dtype=float)


def fidelity_metrics(frames: list[dict], original: np.ndarray) -> dict:
    pose_xy = np.asarray([frame["pose"][:2] for frame in frames], dtype=float)
    distances = point_to_polyline_distance(pose_xy, original[:, :2])
    return {
        "pose_frame_count": int(len(pose_xy)),
        "pose_to_original_rms_mm": float(np.sqrt(np.mean(distances**2))),
        "pose_to_original_p95_mm": float(np.percentile(distances, 95)),
        "pose_to_original_max_mm": float(np.max(distances)),
    }


def run_case(case: dict, spacing_mm: float) -> dict:
    parameters = {**COMMON_PARAMETERS, "spacing": spacing_mm}
    result = simulate({"points": case["points"], "parameters": parameters})
    original = np.asarray(case["points"], dtype=float)
    return {
        "spacing_mm": spacing_mm,
        "point_count": int(result["point_count"]),
        "duration_s": float(result["duration"]),
        "planned_peak_speed_mm_s": float(result["planned_peak_speed_mm_s"]),
        "model_metrics": result["metrics"],
        "shape_fidelity": fidelity_metrics(result["frames"], original),
        "frames": result["frames"],
    }


def input_record(cases: list[dict]) -> dict:
    return {
        "artifact": "drawing-comparison",
        "status": "model_only_preview",
        "model_non_measured": True,
        "model_module": "tools/y42_twin_model.py",
        "interfaces_used": ["preset", "simulate"],
        "common_parameters": COMMON_PARAMETERS,
        "sampling": {
            "coarse_spacing_mm": COARSE_SPACING_MM,
            "fine_spacing_mm": FINE_SPACING_MM,
            "meaning": "Both spacings are passed to y42_twin_model.simulate(); the original polyline is retained unchanged.",
        },
        "cases": [
            {
                "id": case["id"],
                "label": case["label"],
                "source": case["source"],
                "original_vertex_count": len(case["points"]),
                "points_xyz_mm": case["points"],
            }
            for case in cases
        ],
        "hardware": {
            "serial_access": False,
            "http_access": False,
            "hardware_validated": False,
        },
    }


def plot(cases: list[dict], runs: dict[str, dict[str, dict]]) -> None:
    plt.rcParams.update(
        {
            "font.sans-serif": ["Microsoft YaHei", "SimHei", "DejaVu Sans"],
            "axes.unicode_minus": False,
            "figure.facecolor": "white",
            "axes.facecolor": "#fbfcfe",
        }
    )
    fig, axes = plt.subplots(2, 2, figsize=(14, 11), dpi=160)
    axes = axes.ravel()
    colors = {"coarse": "#d97706", "fine": "#1769aa"}
    labels = {
        "coarse": f"粗采样仿真 · {COARSE_SPACING_MM:g} mm",
        "fine": f"细采样仿真 · {FINE_SPACING_MM:g} mm",
    }

    for ax, case in zip(axes, cases):
        original = np.asarray(case["points"], dtype=float)
        ax.plot(
            original[:, 0],
            original[:, 1],
            color="#20252b",
            linewidth=2.2,
            label="绘制原线（输入）",
            zorder=3,
        )
        ax.scatter(
            original[:: max(1, len(original) // 16), 0],
            original[:: max(1, len(original) // 16), 1],
            s=10,
            color="#20252b",
            alpha=0.55,
            zorder=4,
        )
        for key in ("coarse", "fine"):
            frames = runs[case["id"]][key]["frames"]
            pose = np.asarray([frame["pose"][:2] for frame in frames], dtype=float)
            ax.plot(
                pose[:, 0],
                pose[:, 1],
                color=colors[key],
                linewidth=1.55,
                alpha=0.92,
                label=labels[key],
                zorder=2 if key == "coarse" else 1,
            )
            # The marker cadence exposes sampling density without turning the
            # plot into a point cloud.
            marker_step = max(1, len(pose) // 32)
            ax.scatter(
                pose[::marker_step, 0],
                pose[::marker_step, 1],
                color=colors[key],
                s=7,
                alpha=0.42,
                zorder=2 if key == "coarse" else 1,
            )

        coarse = runs[case["id"]]["coarse"]["shape_fidelity"]
        fine = runs[case["id"]]["fine"]["shape_fidelity"]
        ax.text(
            0.02,
            0.98,
            "模型非实测\n"
            f"粗 RMS {coarse['pose_to_original_rms_mm']:.2f} mm · "
            f"细 RMS {fine['pose_to_original_rms_mm']:.2f} mm\n"
            f"原线长 {polyline_length(original):.1f} mm",
            transform=ax.transAxes,
            ha="left",
            va="top",
            fontsize=8.5,
            color="#39434e",
            bbox={"boxstyle": "round,pad=0.35", "facecolor": "white", "alpha": 0.88, "edgecolor": "#ccd5df"},
        )
        ax.set(
            xlim=(0, 110),
            ylim=(0, 148),
            xlabel="X / mm",
            ylabel="Y / mm",
            title=f"{case['label']} · 二维轨迹",
        )
        ax.set_aspect("equal", adjustable="box")
        ax.grid(alpha=0.22, linewidth=0.7)
        ax.legend(loc="lower right", fontsize=8, framealpha=0.9)

    fig.suptitle("Y42 绘制轨迹对照 · 仅为模型仿真（非实测）", fontsize=18, fontweight="bold")
    fig.text(
        0.5,
        0.012,
        "原线来自输入点；两条仿真曲线来自 y42_twin_model.simulate() 的 pose。"
        "粗/细间距只改变模型细化采样，不能证明控制器或物理系统正确。",
        ha="center",
        va="bottom",
        fontsize=9,
        color="#4b5563",
    )
    fig.tight_layout(rect=(0, 0.035, 1, 0.95))
    OUT.mkdir(parents=True, exist_ok=True)
    fig.savefig(PNG_PATH, dpi=180, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    cases = drawing_cases()
    input_data = input_record(cases)
    runs: dict[str, dict[str, dict]] = {}
    output_cases = []
    for case in cases:
        case_runs = {
            "coarse": run_case(case, COARSE_SPACING_MM),
            "fine": run_case(case, FINE_SPACING_MM),
        }
        runs[case["id"]] = case_runs
        output_cases.append(
            {
                "id": case["id"],
                "label": case["label"],
                "source": case["source"],
                "original_vertex_count": len(case["points"]),
                "original_polyline_length_mm": polyline_length(np.asarray(case["points"], dtype=float)),
                "simulations": {
                    key: {k: v for k, v in value.items() if k != "frames"}
                    for key, value in case_runs.items()
                },
            }
        )

    OUT.mkdir(parents=True, exist_ok=True)
    INPUT_PATH.write_text(json.dumps(input_data, ensure_ascii=False, indent=2), encoding="utf-8")
    plot(cases, runs)
    output_data = {
        "artifact": "drawing-comparison",
        "status": "model_only_preview",
        "model_non_measured": True,
        "source_model": "tools/y42_twin_model.py",
        "png": str(PNG_PATH.relative_to(ROOT)).replace("\\", "/"),
        "shape_fidelity_definition": "Shortest XY distance from every simulated pose frame to the original input polyline.",
        "interpretation_limit": "These metrics quantify this reduced model's path deviation from its input polyline; they do not validate controller code, hardware, or physical parameters.",
        "cases": output_cases,
    }
    OUTPUT_PATH.write_text(json.dumps(output_data, ensure_ascii=False, indent=2), encoding="utf-8")
    print(PNG_PATH)
    print(INPUT_PATH)
    print(OUTPUT_PATH)


if __name__ == "__main__":
    main()
