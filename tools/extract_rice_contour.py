#!/usr/bin/env python3
"""
Extract a rice-ear drawing trajectory from a source photo.

The script removes the red guide mark, thresholds the foreground, extracts
external contours, rescales them into the machine's 150 x 150 mm drawing area,
and writes both a visual preview and a web snippet that can replace
riceEarStrokes() in src/main.cpp.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, help="source photo path")
    parser.add_argument("--prefix", default="assets/rice_ear_extracted")
    parser.add_argument("--canvas-mm", type=float, default=150.0)
    parser.add_argument("--points", type=int, default=10000)
    parser.add_argument("--margin-mm", type=float, default=6.0)
    parser.add_argument("--min-area", type=float, default=60.0)
    parser.add_argument("--min-length", type=float, default=30.0)
    parser.add_argument("--preview-size", type=int, default=900)
    parser.add_argument("--keep-bottom-right", action="store_true", help="do not mask out the common bottom-right watermark area")
    return parser.parse_args()


def remove_red_marks(bgr: np.ndarray) -> np.ndarray:
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    lower_red = cv2.inRange(hsv, (0, 80, 80), (12, 255, 255))
    upper_red = cv2.inRange(hsv, (168, 80, 80), (180, 255, 255))
    red = cv2.bitwise_or(lower_red, upper_red)
    red = cv2.dilate(red, np.ones((5, 5), np.uint8), iterations=2)

    cleaned = bgr.copy()
    bg = np.median(cleaned[red == 0], axis=0) if np.any(red == 0) else np.array([245, 245, 245])
    cleaned[red > 0] = bg.astype(np.uint8)
    return cleaned


def foreground_mask(bgr: np.ndarray) -> np.ndarray:
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)

    # Gold/yellow rice is much more saturated than the gray wall. Keep enough
    # low-saturation bright highlights so pale grains are not punched out.
    gold = ((h >= 12) & (h <= 48) & (s >= 22) & (v >= 60))
    saturated = (s >= 38) & (v >= 45)
    mask = np.where(gold | saturated, 255, 0).astype(np.uint8)

    # Smooth photo noise but keep leaf tips.
    mask = cv2.medianBlur(mask, 5)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8), iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations=1)
    return mask


def remove_bottom_right_watermark(mask: np.ndarray) -> np.ndarray:
    cleaned = mask.copy()
    h, w = cleaned.shape[:2]
    cleaned[int(h * 0.54):, int(w * 0.58):] = 0
    return cleaned


def contour_length(contour: np.ndarray) -> float:
    pts = contour.reshape(-1, 2).astype(np.float32)
    if len(pts) < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.roll(pts, -1, axis=0) - pts, axis=1)))


def extract_contours(mask: np.ndarray, min_area: float, min_length: float) -> list[np.ndarray]:
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    kept: list[np.ndarray] = []
    for contour in contours:
        area = abs(cv2.contourArea(contour))
        length = contour_length(contour)
        if area >= min_area and length >= min_length:
            kept.append(contour.reshape(-1, 2).astype(np.float32))
    kept.sort(key=lambda c: contour_length(c), reverse=True)
    return kept


def resample_closed(points: np.ndarray, count: int) -> np.ndarray:
    if len(points) < 2:
        return points.copy()
    closed = np.vstack([points, points[0]])
    seg = np.linalg.norm(closed[1:] - closed[:-1], axis=1)
    total = float(np.sum(seg))
    if total <= 1e-6:
        return points[:1].copy()

    targets = np.linspace(0.0, total, max(2, count), endpoint=False)
    cumulative = np.concatenate([[0.0], np.cumsum(seg)])
    out = []
    idx = 0
    for target in targets:
        while idx < len(seg) - 1 and cumulative[idx + 1] < target:
            idx += 1
        span = max(1e-6, cumulative[idx + 1] - cumulative[idx])
        t = (target - cumulative[idx]) / span
        out.append(closed[idx] + (closed[idx + 1] - closed[idx]) * t)
    out.append(out[0].copy())
    return np.asarray(out, dtype=np.float32)


def allocate_counts(contours: list[np.ndarray], target_points: int) -> list[int]:
    lengths = np.asarray([max(1.0, contour_length(c)) for c in contours], dtype=np.float64)
    raw = lengths / float(np.sum(lengths)) * target_points
    counts = np.maximum(8, np.floor(raw).astype(int))
    diff = target_points - int(np.sum(counts))
    order = np.argsort(raw - np.floor(raw))[::-1]
    cursor = 0
    while diff != 0 and len(order) > 0:
        i = int(order[cursor % len(order)])
        if diff > 0:
            counts[i] += 1
            diff -= 1
        elif counts[i] > 8:
            counts[i] -= 1
            diff += 1
        cursor += 1
        if cursor > target_points * 4:
            break
    return counts.tolist()


def image_to_mm(strokes_px: list[np.ndarray], canvas_mm: float, margin_mm: float) -> list[list[dict[str, float]]]:
    all_points = np.vstack(strokes_px)
    min_xy = all_points.min(axis=0)
    max_xy = all_points.max(axis=0)
    span = np.maximum(max_xy - min_xy, 1.0)
    drawable = canvas_mm - 2.0 * margin_mm
    scale = min(drawable / span[0], drawable / span[1])
    offset = np.array([
        margin_mm + (drawable - span[0] * scale) * 0.5,
        margin_mm + (drawable - span[1] * scale) * 0.5,
    ], dtype=np.float32)

    result = []
    for stroke in strokes_px:
        mm = (stroke - min_xy) * scale + offset
        mm[:, 1] = canvas_mm - mm[:, 1]
        result.append([
            {"x": round(float(x), 2), "y": round(float(y), 2)}
            for x, y in mm
        ])
    return result


def write_svg(strokes: list[list[dict[str, float]]], path: Path, canvas_mm: float) -> None:
    size = 900
    pad = 36
    scale = (size - pad * 2) / canvas_mm
    lines = []
    for stroke in strokes:
        points = " ".join(
            f"{pad + p['x'] * scale:.1f},{pad + (canvas_mm - p['y']) * scale:.1f}"
            for p in stroke
        )
        lines.append(
            f'  <polyline points="{points}" fill="none" stroke="#000" '
            f'stroke-width="4" stroke-linecap="round" stroke-linejoin="round"/>'
        )
    path.write_text(
        "\n".join([
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{size}" height="{size}" viewBox="0 0 {size} {size}">',
            '  <rect width="100%" height="100%" fill="#fff"/>',
            f'  <rect x="{pad}" y="{pad}" width="{size - pad * 2}" height="{size - pad * 2}" fill="none" stroke="#111" stroke-width="4"/>',
            *lines,
            "</svg>",
            "",
        ]),
        encoding="utf-8",
    )


def write_png_from_svg_style(strokes: list[list[dict[str, float]]], path: Path, canvas_mm: float, size: int) -> None:
    pad = max(24, int(size * 0.04))
    scale = (size - pad * 2) / canvas_mm
    img = np.full((size, size, 3), 255, dtype=np.uint8)
    cv2.rectangle(img, (pad, pad), (size - pad, size - pad), (17, 17, 17), 4)
    for stroke in strokes:
        pts = np.asarray([
            [pad + p["x"] * scale, pad + (canvas_mm - p["y"]) * scale]
            for p in stroke
        ], dtype=np.int32)
        if len(pts) >= 2:
            cv2.polylines(img, [pts], False, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.imwrite(str(path), img)


def write_web_snippet(strokes: list[list[dict[str, float]]], path: Path) -> None:
    payload = json.dumps(strokes, separators=(",", ":"))
    path.write_text(
        "function riceEarStrokes(){\n"
        f"  return {payload};\n"
        "}\n",
        encoding="utf-8",
    )


def main() -> None:
    args = parse_args()
    source = Path(args.input)
    prefix = Path(args.prefix)
    prefix.parent.mkdir(parents=True, exist_ok=True)

    bgr = cv2.imread(str(source), cv2.IMREAD_COLOR)
    if bgr is None:
        raise SystemExit(f"Cannot read image: {source}")

    cleaned = remove_red_marks(bgr)
    mask = foreground_mask(cleaned)
    if not args.keep_bottom_right:
        mask = remove_bottom_right_watermark(mask)
    contours = extract_contours(mask, args.min_area, args.min_length)
    if not contours:
        raise SystemExit("No contours found. Try lowering --min-area or --min-length.")

    counts = allocate_counts(contours, max(1, args.points - len(contours)))
    strokes_px = [resample_closed(contour, count) for contour, count in zip(contours, counts)]
    strokes_mm = image_to_mm(strokes_px, args.canvas_mm, args.margin_mm)

    json_path = prefix.with_suffix(".json")
    svg_path = prefix.with_suffix(".svg")
    png_path = prefix.with_suffix(".png")
    js_path = prefix.with_name(prefix.name + "_web.js")
    mask_path = prefix.with_name(prefix.name + "_mask.png")
    clean_path = prefix.with_name(prefix.name + "_cleaned.png")

    json_path.write_text(json.dumps(strokes_mm, ensure_ascii=False), encoding="utf-8")
    write_svg(strokes_mm, svg_path, args.canvas_mm)
    write_png_from_svg_style(strokes_mm, png_path, args.canvas_mm, args.preview_size)
    write_web_snippet(strokes_mm, js_path)
    cv2.imwrite(str(mask_path), mask)
    cv2.imwrite(str(clean_path), cleaned)

    point_count = sum(len(stroke) for stroke in strokes_mm)
    print(f"contours={len(strokes_mm)} points={point_count}")
    print(f"preview={png_path}")
    print(f"web_snippet={js_path}")


if __name__ == "__main__":
    main()
