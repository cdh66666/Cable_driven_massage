from __future__ import annotations

import json
import math
import struct
import sys
import tempfile
import threading
import time
import tkinter as tk
import urllib.error
import urllib.request
import webbrowser
from dataclasses import dataclass
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

import cv2
import numpy as np


CANVAS_MM = 150.0
PREVIEW_SIZE = 900
DISPLAY_SIZE = 430
BOARD_URL = "http://192.168.4.1"
WEB_EXPORT_MAX_POINTS = 0
DEFAULT_USB_PORT = "COM52"
USB_BAUD = 460800

MODE_AUTO = "自动"
MODE_SURPRISE = "惊喜复刻"
MODE_PHOTO_DITHER = "照片点描复刻"
MODE_FULL_SKETCH = "整图素描"
MODE_PHOTO_SUBJECT = "照片主体线稿"
MODE_DARK_LINE = "深色线稿"
MODE_DARK_CONTOUR = "黑色区域轮廓"
MODE_EDGE = "边缘线稿"
MODE_OUTLINE = "物体外轮廓"
MODE_CENTERLINE = "中心线(慢)"


@dataclass
class TrajectoryResult:
    strokes: list[list[dict[str, float]]]
    preview: np.ndarray
    mask: np.ndarray
    stats: str


def imread_unicode(path: str) -> np.ndarray | None:
    data = np.fromfile(path, dtype=np.uint8)
    if data.size:
        img = cv2.imdecode(data, cv2.IMREAD_UNCHANGED)
        if img is not None:
            return img
    cap = cv2.VideoCapture(path)
    ok, frame = cap.read()
    cap.release()
    return frame if ok else None


def imwrite_unicode(path: str, image: np.ndarray) -> bool:
    ext = Path(path).suffix or ".png"
    ok, encoded = cv2.imencode(ext, image)
    if not ok:
        return False
    encoded.tofile(path)
    return True


def to_bgr_on_white(img: np.ndarray) -> np.ndarray:
    if img.ndim == 2:
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    if img.shape[2] == 4:
        bgr = img[:, :, :3].astype(np.float32)
        alpha = img[:, :, 3:4].astype(np.float32) / 255.0
        white = np.full_like(bgr, 255.0)
        return (bgr * alpha + white * (1.0 - alpha)).astype(np.uint8)
    return img[:, :, :3]


def resize_for_processing(img: np.ndarray, max_side: int) -> np.ndarray:
    h, w = img.shape[:2]
    scale = min(1.0, max_side / max(h, w))
    if scale >= 1.0:
        return img.copy()
    nw = max(1, int(round(w * scale)))
    nh = max(1, int(round(h * scale)))
    return cv2.resize(img, (nw, nh), interpolation=cv2.INTER_AREA)


def remove_small_components(mask: np.ndarray, min_area: int) -> np.ndarray:
    if min_area <= 1:
        return mask
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    out = np.zeros_like(mask)
    for i in range(1, count):
        if stats[i, cv2.CC_STAT_AREA] >= min_area:
            out[labels == i] = 255
    return out


def photo_subject_edges(bgr: np.ndarray, canny_low: int, canny_high: int, min_area: int) -> np.ndarray:
    h, w = bgr.shape[:2]
    rect = (
        max(1, int(w * 0.18)),
        max(1, int(h * 0.04)),
        max(2, int(w * 0.64)),
        max(2, int(h * 0.94)),
    )
    grab_mask = np.zeros((h, w), np.uint8)
    bgd = np.zeros((1, 65), np.float64)
    fgd = np.zeros((1, 65), np.float64)
    try:
        cv2.grabCut(bgr, grab_mask, rect, bgd, fgd, 3, cv2.GC_INIT_WITH_RECT)
        subject = np.where((grab_mask == cv2.GC_FGD) | (grab_mask == cv2.GC_PR_FGD), 255, 0).astype(np.uint8)
    except cv2.error:
        subject = np.zeros((h, w), np.uint8)
        cv2.rectangle(subject, (rect[0], rect[1]), (rect[0] + rect[2], rect[1] + rect[3]), 255, -1)

    kernel = np.ones((5, 5), np.uint8)
    subject = cv2.morphologyEx(subject, cv2.MORPH_OPEN, kernel, iterations=1)
    subject = cv2.morphologyEx(subject, cv2.MORPH_CLOSE, kernel, iterations=2)
    subject = cv2.dilate(subject, kernel, iterations=1)

    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    smooth = cv2.bilateralFilter(gray, 7, 45, 45)
    edges = cv2.Canny(smooth, max(10, canny_low), max(canny_low + 10, canny_high))
    edges = cv2.bitwise_and(edges, subject)

    subject_outline = np.zeros_like(gray)
    contours, _ = cv2.findContours(subject, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    cv2.drawContours(subject_outline, contours, -1, 255, 1)
    mask = cv2.bitwise_or(edges, subject_outline)

    focus = np.zeros_like(gray)
    cv2.ellipse(
        focus,
        (int(w * 0.53), int(h * 0.55)),
        (int(w * 0.34), int(h * 0.56)),
        0,
        0,
        360,
        255,
        -1,
    )
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
    filtered = np.zeros_like(mask)
    min_keep = max(min_area, 18)
    for i in range(1, count):
        area = stats[i, cv2.CC_STAT_AREA]
        if area < min_keep:
            continue
        comp = labels == i
        overlap = cv2.countNonZero(np.where(comp, focus, 0))
        cx, cy = centroids[i]
        centered = ((cx - w * 0.53) / (w * 0.42)) ** 2 + ((cy - h * 0.55) / (h * 0.64)) ** 2 <= 1.0
        if centered or overlap >= area * 0.12:
            filtered[comp] = 255
    return filtered


def preprocess_mask(
    bgr: np.ndarray,
    mode: str,
    threshold: int,
    canny_low: int,
    canny_high: int,
    invert: bool,
    close_iter: int,
    min_area: int,
) -> np.ndarray:
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    if invert:
        gray = 255 - gray
    blur = cv2.GaussianBlur(gray, (3, 3), 0)

    if mode == MODE_PHOTO_SUBJECT:
        mask = photo_subject_edges(bgr, canny_low, canny_high, min_area)
    elif mode == MODE_DARK_LINE:
        _, mask = cv2.threshold(blur, threshold, 255, cv2.THRESH_BINARY_INV)
    elif mode == MODE_DARK_CONTOUR:
        _, mask = cv2.threshold(blur, threshold, 255, cv2.THRESH_BINARY_INV)
    elif mode == MODE_OUTLINE:
        edges = cv2.Canny(blur, canny_low, canny_high)
        kernel = np.ones((3, 3), np.uint8)
        closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=max(1, close_iter))
        contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        mask = np.zeros_like(gray)
        cv2.drawContours(mask, contours, -1, 255, 1)
    else:
        _, dark = cv2.threshold(blur, threshold, 255, cv2.THRESH_BINARY_INV)
        edges = cv2.Canny(blur, canny_low, canny_high)
        mask = cv2.bitwise_or(dark, edges)

    if close_iter > 0 and mode not in (MODE_OUTLINE, MODE_PHOTO_SUBJECT):
        kernel = np.ones((2, 2), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=close_iter)
    return remove_small_components(mask, min_area)


def zhang_suen_thinning(mask: np.ndarray) -> np.ndarray:
    img = (mask > 0).astype(np.uint8)
    changed = True
    while changed:
        changed = False
        for step in (0, 1):
            rows, cols = img.shape
            to_remove: list[tuple[int, int]] = []
            for y in range(1, rows - 1):
                for x in range(1, cols - 1):
                    p1 = img[y, x]
                    if p1 == 0:
                        continue
                    p2 = img[y - 1, x]
                    p3 = img[y - 1, x + 1]
                    p4 = img[y, x + 1]
                    p5 = img[y + 1, x + 1]
                    p6 = img[y + 1, x]
                    p7 = img[y + 1, x - 1]
                    p8 = img[y, x - 1]
                    p9 = img[y - 1, x - 1]
                    ns = [p2, p3, p4, p5, p6, p7, p8, p9]
                    b = int(sum(ns))
                    if b < 2 or b > 6:
                        continue
                    a = sum(1 for i in range(8) if ns[i] == 0 and ns[(i + 1) % 8] == 1)
                    if a != 1:
                        continue
                    if step == 0:
                        if p2 * p4 * p6 != 0 or p4 * p6 * p8 != 0:
                            continue
                    else:
                        if p2 * p4 * p8 != 0 or p2 * p6 * p8 != 0:
                            continue
                    to_remove.append((y, x))
            if to_remove:
                changed = True
                for y, x in to_remove:
                    img[y, x] = 0
    return (img * 255).astype(np.uint8)


def neighbors(point: tuple[int, int], pixels: set[tuple[int, int]]) -> list[tuple[int, int]]:
    x, y = point
    out = []
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dx == 0 and dy == 0:
                continue
            q = (x + dx, y + dy)
            if q in pixels:
                out.append(q)
    return out


def skeleton_to_paths(skeleton: np.ndarray, min_pixels: int) -> list[list[tuple[float, float]]]:
    ys, xs = np.where(skeleton > 0)
    all_pixels = {(int(x), int(y)) for x, y in zip(xs, ys)}
    paths: list[list[tuple[float, float]]] = []

    while all_pixels:
        seed = next(iter(all_pixels))
        stack = [seed]
        comp: set[tuple[int, int]] = set()
        all_pixels.remove(seed)
        while stack:
            p = stack.pop()
            comp.add(p)
            for q in neighbors(p, all_pixels):
                all_pixels.remove(q)
                stack.append(q)
        if len(comp) < min_pixels:
            continue
        paths.extend(component_to_paths(comp, min_pixels))
    return paths


def contours_to_paths(mask: np.ndarray, min_pixels: int, external_only: bool) -> list[list[tuple[float, float]]]:
    mode = cv2.RETR_EXTERNAL if external_only else cv2.RETR_LIST
    contours, _ = cv2.findContours(mask, mode, cv2.CHAIN_APPROX_NONE)
    paths: list[list[tuple[float, float]]] = []
    for contour in contours:
        if len(contour) < min_pixels:
            continue
        pts = contour.reshape(-1, 2)
        path = [(float(x), float(y)) for x, y in pts]
        if len(path) >= 3 and math.hypot(path[0][0] - path[-1][0], path[0][1] - path[-1][1]) <= 2.0:
            path.append(path[0])
        paths.append(path)
    return paths


def full_image_sketch_paths(
    bgr: np.ndarray,
    canny_low: int,
    canny_high: int,
    min_pixels: int,
    min_area: int,
) -> tuple[np.ndarray, list[list[tuple[float, float]]]]:
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=1.7, tileGridSize=(8, 8))
    tonal = clahe.apply(gray)
    smooth = cv2.bilateralFilter(tonal, 7, 36, 36)

    edges = cv2.Canny(smooth, max(8, canny_low), max(canny_low + 10, canny_high))
    adaptive = cv2.adaptiveThreshold(
        smooth,
        255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV,
        23,
        8,
    )
    line_mask = cv2.bitwise_or(edges, adaptive)
    line_mask = remove_small_components(line_mask, max(4, min_area // 2))

    paths = contours_to_paths(line_mask, min_pixels, external_only=False)
    hatch_paths = raster_tone_paths(smooth, min_pixels)
    mask_preview = cv2.bitwise_or(line_mask, draw_pixel_paths_mask(hatch_paths, smooth.shape))
    return mask_preview, paths + hatch_paths


def add_contour_level_paths(
    paths: list[list[tuple[float, float]]],
    gray: np.ndarray,
    level: int,
    min_pixels: int,
    min_area: int,
) -> None:
    mask = (gray < level).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8), iterations=1)
    contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    min_len = max(18.0, float(min_pixels) * 2.2)
    min_area = max(8, min_area)
    for contour in contours:
        if len(contour) < min_pixels:
            continue
        arc = cv2.arcLength(contour, True)
        if arc < min_len:
            continue
        area = abs(cv2.contourArea(contour))
        if area < min_area and arc < min_len * 1.8:
            continue
        pts = contour.reshape(-1, 2)
        path = [(float(x), float(y)) for x, y in pts]
        if len(path) >= 3:
            path.append(path[0])
            paths.append(path)


def photo_trajectory_paths(
    bgr: np.ndarray,
    canny_low: int,
    canny_high: int,
    min_pixels: int,
    min_area: int,
) -> tuple[np.ndarray, list[list[tuple[float, float]]]]:
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    gray = cv2.bilateralFilter(gray, 7, 34, 34)
    gray = cv2.createCLAHE(clipLimit=1.45, tileGridSize=(8, 8)).apply(gray)

    soft = cv2.GaussianBlur(gray, (0, 0), 0.85)
    edges = cv2.Canny(soft, max(6, canny_low), max(canny_low + 10, canny_high))
    lap = cv2.Laplacian(soft, cv2.CV_8U, ksize=3)
    _, lap_mask = cv2.threshold(lap, 18, 255, cv2.THRESH_BINARY)
    adaptive = cv2.adaptiveThreshold(
        soft,
        255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV,
        25,
        9,
    )
    line_mask = cv2.bitwise_or(edges, lap_mask)
    line_mask = cv2.bitwise_or(line_mask, adaptive)
    line_mask = cv2.morphologyEx(line_mask, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8), iterations=1)
    line_mask = remove_small_components(line_mask, max(5, min_area // 2))

    paths = contours_to_paths(line_mask, max(5, min_pixels), external_only=False)

    level_blur = cv2.GaussianBlur(gray, (0, 0), 1.35)
    for level in (42, 58, 74, 90, 106, 122, 138, 154, 170, 188, 206, 224):
        add_contour_level_paths(paths, level_blur, level, max(8, min_pixels), min_area)

    preview = draw_pixel_paths_mask(paths, gray.shape)
    return preview, paths


def draw_pixel_paths_mask(paths: list[list[tuple[float, float]]], shape: tuple[int, int]) -> np.ndarray:
    mask = np.zeros(shape, np.uint8)
    for path in paths:
        if len(path) < 2:
            continue
        pts = np.array([[int(round(x)), int(round(y))] for x, y in path], dtype=np.int32)
        cv2.polylines(mask, [pts], False, 255, 1, cv2.LINE_AA)
    return mask


def hatching_paths(gray: np.ndarray, min_pixels: int) -> list[list[tuple[float, float]]]:
    h, w = gray.shape[:2]
    blur = cv2.GaussianBlur(gray, (0, 0), sigmaX=3, sigmaY=3)
    configs = [
        (190, 18.0, 0.0),
        (145, 16.0, -28.0),
        (105, 13.0, 28.0),
        (68, 10.0, 0.0),
    ]
    paths: list[list[tuple[float, float]]] = []
    for threshold, spacing, angle in configs:
        dark = (blur < threshold).astype(np.uint8)
        dark = cv2.morphologyEx(dark, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations=1)
        paths.extend(hatch_mask(dark, spacing, angle, max(10, min_pixels)))
    return paths


def raster_tone_paths(gray: np.ndarray, min_pixels: int) -> list[list[tuple[float, float]]]:
    h, w = gray.shape[:2]
    small = cv2.GaussianBlur(gray, (3, 3), 0)
    bayer = np.array(
        [
            [0, 48, 12, 60, 3, 51, 15, 63],
            [32, 16, 44, 28, 35, 19, 47, 31],
            [8, 56, 4, 52, 11, 59, 7, 55],
            [40, 24, 36, 20, 43, 27, 39, 23],
            [2, 50, 14, 62, 1, 49, 13, 61],
            [34, 18, 46, 30, 33, 17, 45, 29],
            [10, 58, 6, 54, 9, 57, 5, 53],
            [42, 26, 38, 22, 41, 25, 37, 21],
        ],
        dtype=np.float32,
    )
    threshold = ((bayer + 0.5) / 64.0)
    tiled = np.tile(threshold, (h // 8 + 1, w // 8 + 1))[:h, :w]
    darkness = np.clip((245.0 - small.astype(np.float32)) / 210.0, 0.0, 1.0)
    dark = darkness > tiled
    paths: list[list[tuple[float, float]]] = []
    row_step = 3
    min_run = max(6, min_pixels)
    for y in range(0, h, row_step):
        row = dark[y]
        x = 0
        while x < w:
            while x < w and not row[x]:
                x += 1
            start = x
            while x < w and row[x]:
                x += 1
            end = x - 1
            if end - start + 1 >= min_run:
                paths.append([(float(start), float(y)), (float(end), float(y))])
    return paths


def photo_dither_paths(
    bgr: np.ndarray,
    canny_low: int,
    canny_high: int,
    max_points: int,
) -> tuple[np.ndarray, list[list[tuple[float, float]]]]:
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    gray = cv2.bilateralFilter(gray, 5, 26, 26)
    clahe = cv2.createCLAHE(clipLimit=1.25, tileGridSize=(8, 8))
    gray = clahe.apply(gray)

    bayer = np.array(
        [
            [0, 48, 12, 60, 3, 51, 15, 63],
            [32, 16, 44, 28, 35, 19, 47, 31],
            [8, 56, 4, 52, 11, 59, 7, 55],
            [40, 24, 36, 20, 43, 27, 39, 23],
            [2, 50, 14, 62, 1, 49, 13, 61],
            [34, 18, 46, 30, 33, 17, 45, 29],
            [10, 58, 6, 54, 9, 57, 5, 53],
            [42, 26, 38, 22, 41, 25, 37, 21],
        ],
        dtype=np.float32,
    )
    h, w = gray.shape[:2]
    tiled = np.tile((bayer + 0.5) / 64.0, (h // 8 + 1, w // 8 + 1))[:h, :w]
    darkness = np.clip((252.0 - gray.astype(np.float32)) / 242.0, 0.0, 1.0)
    darkness = np.power(darkness, 1.12)
    mask = darkness > tiled

    edges = cv2.Canny(gray, max(6, canny_low), max(canny_low + 10, canny_high))
    edge_mask = cv2.dilate(edges, np.ones((2, 2), np.uint8), iterations=1) > 0
    mask = np.logical_or(mask, edge_mask)
    edge_paths = contours_to_paths(edges, max(4, max_points // 40000), external_only=False)

    ys, xs = np.where(mask)
    edge_points = sum(len(path) for path in edge_paths)
    max_dots = max(1, (max_points - edge_points) // 2)
    if len(xs) > max_dots:
        idx = np.linspace(0, len(xs) - 1, max_dots).round().astype(np.int64)
        xs = xs[idx]
        ys = ys[idx]

    preview = np.zeros((h, w), np.uint8)
    preview[ys, xs] = 255
    paths: list[list[tuple[float, float]]] = edge_paths
    for x, y in zip(xs, ys):
        seed = (int(x) * 73856093) ^ (int(y) * 19349663)
        angle = (seed % 6283) / 1000.0
        length = 0.42 + ((seed >> 8) % 100) / 260.0
        dx = math.cos(angle) * length
        dy = math.sin(angle) * length
        paths.append([(float(x), float(y)), (float(x) + dx, float(y) + dy)])
    return preview, paths


def hatch_mask(mask: np.ndarray, spacing: float, angle_deg: float, min_run: int) -> list[list[tuple[float, float]]]:
    h, w = mask.shape[:2]
    theta = math.radians(angle_deg)
    d = np.array([math.cos(theta), math.sin(theta)], dtype=np.float32)
    n = np.array([-math.sin(theta), math.cos(theta)], dtype=np.float32)
    corners = np.array([[0, 0], [w - 1, 0], [0, h - 1], [w - 1, h - 1]], dtype=np.float32)
    n_proj = corners @ n
    d_proj = corners @ d
    c_min, c_max = float(n_proj.min()), float(n_proj.max())
    t_min, t_max = float(d_proj.min()) - 3.0, float(d_proj.max()) + 3.0
    paths: list[list[tuple[float, float]]] = []

    c = c_min
    while c <= c_max:
        current: list[tuple[float, float]] = []
        t = t_min
        while t <= t_max:
            p = d * t + n * c
            x = int(round(float(p[0])))
            y = int(round(float(p[1])))
            inside = 0 <= x < w and 0 <= y < h and mask[y, x] > 0
            if inside:
                if not current or math.hypot(current[-1][0] - x, current[-1][1] - y) >= 2.5:
                    current.append((float(x), float(y)))
            else:
                if len(current) >= min_run:
                    paths.append(current)
                current = []
            t += 2.0
        if len(current) >= min_run:
            paths.append(current)
        c += spacing
    return paths


def component_to_paths(comp: set[tuple[int, int]], min_pixels: int) -> list[list[tuple[float, float]]]:
    adj = {p: neighbors(p, comp) for p in comp}
    nodes = {p for p, ns in adj.items() if len(ns) != 2}
    visited_edges: set[frozenset[tuple[int, int]]] = set()
    paths: list[list[tuple[float, float]]] = []

    def mark(a: tuple[int, int], b: tuple[int, int]) -> None:
        visited_edges.add(frozenset((a, b)))

    def marked(a: tuple[int, int], b: tuple[int, int]) -> bool:
        return frozenset((a, b)) in visited_edges

    def walk(start: tuple[int, int], nxt: tuple[int, int]) -> list[tuple[int, int]]:
        path = [start, nxt]
        mark(start, nxt)
        prev, cur = start, nxt
        for _ in range(len(comp) + 4):
            if cur in nodes and cur != start:
                break
            candidates = [q for q in adj[cur] if q != prev and not marked(cur, q)]
            if not candidates:
                break
            q = candidates[0]
            mark(cur, q)
            path.append(q)
            prev, cur = cur, q
            if cur == start:
                break
        return path

    for node in sorted(nodes, key=lambda p: (p[1], p[0])):
        for nxt in adj[node]:
            if not marked(node, nxt):
                path = walk(node, nxt)
                if len(path) >= min_pixels:
                    paths.append([(float(x), float(y)) for x, y in path])

    remaining = []
    for p, ns in adj.items():
        for q in ns:
            if not marked(p, q):
                remaining.append((p, q))
    while remaining:
        start, nxt = remaining.pop()
        if marked(start, nxt):
            continue
        path = walk(start, nxt)
        if len(path) >= min_pixels:
            paths.append([(float(x), float(y)) for x, y in path])
    return paths


def rdp(points: list[tuple[float, float]], epsilon: float) -> list[tuple[float, float]]:
    if len(points) < 3 or epsilon <= 0:
        return points
    a = np.array(points[0], dtype=float)
    b = np.array(points[-1], dtype=float)
    ab = b - a
    denom = float(np.dot(ab, ab))
    max_dist = -1.0
    index = 0
    for i in range(1, len(points) - 1):
        p = np.array(points[i], dtype=float)
        if denom == 0:
            dist = float(np.linalg.norm(p - a))
        else:
            t = np.clip(float(np.dot(p - a, ab) / denom), 0.0, 1.0)
            dist = float(np.linalg.norm(p - (a + t * ab)))
        if dist > max_dist:
            max_dist = dist
            index = i
    if max_dist > epsilon:
        left = rdp(points[: index + 1], epsilon)
        right = rdp(points[index:], epsilon)
        return left[:-1] + right
    return [points[0], points[-1]]


def smooth_path(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    if len(points) < 5:
        return points
    arr = np.array(points, dtype=np.float32)
    out = arr.copy()
    out[1:-1] = arr[:-2] * 0.25 + arr[1:-1] * 0.5 + arr[2:] * 0.25
    return [(float(x), float(y)) for x, y in out]


def path_length(points: list[tuple[float, float]]) -> float:
    if len(points) < 2:
        return 0.0
    return sum(math.hypot(points[i][0] - points[i - 1][0], points[i][1] - points[i - 1][1]) for i in range(1, len(points)))


def resample_path(points: list[tuple[float, float]], spacing: float) -> list[tuple[float, float]]:
    if len(points) < 2 or spacing <= 0:
        return points
    total = path_length(points)
    if total <= spacing:
        return [points[0], points[-1]]
    target_count = max(2, int(math.ceil(total / spacing)) + 1)
    out = [points[0]]
    seg_index = 1
    seg_start = points[0]
    seg_end = points[1]
    seg_len = math.hypot(seg_end[0] - seg_start[0], seg_end[1] - seg_start[1])
    traveled = 0.0
    for k in range(1, target_count - 1):
        target = k * total / (target_count - 1)
        while traveled + seg_len < target and seg_index < len(points) - 1:
            traveled += seg_len
            seg_index += 1
            seg_start = points[seg_index - 1]
            seg_end = points[seg_index]
            seg_len = math.hypot(seg_end[0] - seg_start[0], seg_end[1] - seg_start[1])
        t = 0.0 if seg_len == 0 else (target - traveled) / seg_len
        out.append((seg_start[0] + (seg_end[0] - seg_start[0]) * t, seg_start[1] + (seg_end[1] - seg_start[1]) * t))
    out.append(points[-1])
    return out


def fit_to_hardware(
    paths: list[list[tuple[float, float]]],
    image_shape: tuple[int, int],
    margin_mm: float,
) -> list[list[tuple[float, float]]]:
    all_pts = np.array([p for path in paths for p in path], dtype=np.float32)
    if all_pts.size == 0:
        return []
    min_xy = all_pts.min(axis=0)
    max_xy = all_pts.max(axis=0)
    span = np.maximum(max_xy - min_xy, 1.0)
    drawable = CANVAS_MM - 2.0 * margin_mm
    scale = min(drawable / span[0], drawable / span[1])
    offset_x = margin_mm + (drawable - span[0] * scale) * 0.5
    offset_y = margin_mm + (drawable - span[1] * scale) * 0.5
    out: list[list[tuple[float, float]]] = []
    for path in paths:
        converted = []
        for x, y in path:
            mm_x = (x - min_xy[0]) * scale + offset_x
            mm_y_top = (y - min_xy[1]) * scale + offset_y
            converted.append((float(mm_x), float(CANVAS_MM - mm_y_top)))
        out.append(converted)
    return out


def order_strokes(strokes: list[list[tuple[float, float]]]) -> list[list[tuple[float, float]]]:
    remaining = [s for s in strokes if len(s) >= 2]
    ordered: list[list[tuple[float, float]]] = []
    current = (0.0, 0.0)
    while remaining:
        best_index = 0
        best_reverse = False
        best_dist = float("inf")
        for i, stroke in enumerate(remaining):
            d0 = math.hypot(stroke[0][0] - current[0], stroke[0][1] - current[1])
            d1 = math.hypot(stroke[-1][0] - current[0], stroke[-1][1] - current[1])
            if d0 < best_dist:
                best_index, best_reverse, best_dist = i, False, d0
            if d1 < best_dist:
                best_index, best_reverse, best_dist = i, True, d1
        stroke = remaining.pop(best_index)
        if best_reverse:
            stroke = list(reversed(stroke))
        ordered.append(stroke)
        current = stroke[-1]
    return ordered


def limit_total_points(strokes: list[list[tuple[float, float]]], max_points: int) -> list[list[tuple[float, float]]]:
    total = sum(len(s) for s in strokes)
    if total <= max_points or max_points <= 0:
        return strokes
    scale = total / max_points
    out = []
    for stroke in strokes:
        if len(stroke) <= 2:
            out.append(stroke)
            continue
        keep = max(2, int(round(len(stroke) / scale)))
        idx = np.linspace(0, len(stroke) - 1, keep).round().astype(int)
        out.append([stroke[int(i)] for i in idx])
    return out


def limit_stroke_count(strokes: list[list[tuple[float, float]]], max_strokes: int) -> list[list[tuple[float, float]]]:
    if len(strokes) <= max_strokes:
        return strokes
    return sorted(strokes, key=path_length, reverse=True)[:max_strokes]


def render_preview(strokes: list[list[dict[str, float]]]) -> np.ndarray:
    img = np.full((PREVIEW_SIZE, PREVIEW_SIZE, 3), 255, np.uint8)
    pad = 36
    draw_size = PREVIEW_SIZE - pad * 2
    cv2.rectangle(img, (pad, pad), (PREVIEW_SIZE - pad, PREVIEW_SIZE - pad), (0, 0, 0), 3)
    for stroke in strokes:
        if len(stroke) < 2:
            continue
        pts = np.array(
            [
                [
                    int(round(pad + p["x"] / CANVAS_MM * draw_size)),
                    int(round(pad + (1.0 - p["y"] / CANVAS_MM) * draw_size)),
                ]
                for p in stroke
            ],
            dtype=np.int32,
        )
        cv2.polylines(img, [pts], False, (0, 0, 0), 1, cv2.LINE_AA)
    return img


def render_source_preview(source: np.ndarray) -> np.ndarray:
    bgr = to_bgr_on_white(source)
    h, w = bgr.shape[:2]
    scale = min((PREVIEW_SIZE - 72) / max(w, h), 1.0)
    nw = max(1, int(round(w * scale)))
    nh = max(1, int(round(h * scale)))
    resized = cv2.resize(bgr, (nw, nh), interpolation=cv2.INTER_AREA)
    img = np.full((PREVIEW_SIZE, PREVIEW_SIZE, 3), 245, np.uint8)
    x = (PREVIEW_SIZE - nw) // 2
    y = (PREVIEW_SIZE - nh) // 2
    img[y : y + nh, x : x + nw] = resized
    return img


def fit_for_display(img: np.ndarray, size: int = DISPLAY_SIZE) -> np.ndarray:
    h, w = img.shape[:2]
    scale = min(size / max(h, w), 1.0)
    nw = max(1, int(round(w * scale)))
    nh = max(1, int(round(h * scale)))
    resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_AREA)
    out = np.full((size, size, 3), 245, np.uint8)
    x = (size - nw) // 2
    y = (size - nh) // 2
    out[y : y + nh, x : x + nw] = resized
    return out


def build_trajectory(
    source: np.ndarray,
    mode: str,
    threshold: int,
    canny_low: int,
    canny_high: int,
    invert: bool,
    max_side: int,
    min_segment_px: int,
    min_area: int,
    close_iter: int,
    margin_mm: float,
    spacing_mm: float,
    simplify_mm: float,
    max_points: int,
) -> TrajectoryResult:
    if mode == MODE_AUTO:
        mode = MODE_SURPRISE
    bgr = resize_for_processing(to_bgr_on_white(source), max_side)
    if mode == MODE_SURPRISE:
        skeleton, pixel_paths = photo_trajectory_paths(bgr, canny_low, canny_high, min_segment_px, min_area)
    elif mode == MODE_PHOTO_DITHER:
        skeleton, pixel_paths = photo_dither_paths(bgr, canny_low, canny_high, max_points)
    elif mode == MODE_FULL_SKETCH:
        skeleton, pixel_paths = full_image_sketch_paths(bgr, canny_low, canny_high, min_segment_px, min_area)
    else:
        mask = preprocess_mask(bgr, mode, threshold, canny_low, canny_high, invert, close_iter, min_area)
        if mode == MODE_DARK_CONTOUR:
            skeleton = mask
            pixel_paths = contours_to_paths(mask, min_segment_px, external_only=False)
        elif mode in (MODE_OUTLINE, MODE_PHOTO_SUBJECT):
            skeleton = mask
            pixel_paths = contours_to_paths(mask, min_segment_px, external_only=(mode == MODE_OUTLINE))
        elif mode == MODE_CENTERLINE:
            skeleton = zhang_suen_thinning(mask)
            pixel_paths = skeleton_to_paths(skeleton, min_segment_px)
        else:
            skeleton = mask
            pixel_paths = contours_to_paths(mask, min_segment_px, external_only=False)
    mm_paths = fit_to_hardware(pixel_paths, skeleton.shape, margin_mm)

    prepared: list[list[tuple[float, float]]] = []
    min_draw_len = 0.02 if mode == MODE_PHOTO_DITHER else 1.5
    for path in mm_paths:
        path = smooth_path(path)
        path = rdp(path, simplify_mm)
        path = resample_path(path, spacing_mm)
        if path_length(path) >= min_draw_len and len(path) >= 2:
            prepared.append(path)
    if mode == MODE_PHOTO_DITHER:
        prepared = limit_total_points(prepared, max_points)
    else:
        if mode == MODE_SURPRISE:
            stroke_limit = 3200
        elif mode == MODE_FULL_SKETCH:
            stroke_limit = 1800
        elif mode == MODE_PHOTO_SUBJECT:
            stroke_limit = 180
        else:
            stroke_limit = 260
        prepared = limit_stroke_count(prepared, stroke_limit)
        prepared = order_strokes(prepared)
        prepared = limit_total_points(prepared, max_points)
    strokes = [
        [{"x": round(x, 2), "y": round(y, 2)} for x, y in stroke]
        for stroke in prepared
        if len(stroke) >= 2
    ]
    preview = render_preview(strokes)
    total_points = sum(len(s) for s in strokes)
    total_len = sum(path_length([(p["x"], p["y"]) for p in s]) for s in strokes)
    stats = f"轨迹段: {len(strokes)}    点数: {total_points}    画线长度: {total_len:.1f} mm    画布: {CANVAS_MM:.0f}x{CANVAS_MM:.0f} mm"
    return TrajectoryResult(strokes=strokes, preview=preview, mask=skeleton, stats=stats)


def limit_stroke_dict_points(
    strokes: list[list[dict[str, float]]],
    max_points: int,
) -> tuple[list[list[dict[str, float]]], bool]:
    total = sum(len(stroke) for stroke in strokes)
    if total <= max_points or max_points <= 0:
        return strokes, False

    scale = total / max_points
    out: list[list[dict[str, float]]] = []
    for stroke in strokes:
        if len(stroke) <= 2:
            out.append(stroke)
            continue
        keep = max(2, int(round(len(stroke) / scale)))
        if keep >= len(stroke):
            out.append(stroke)
            continue
        idx = np.linspace(0, len(stroke) - 1, keep).round().astype(int)
        out.append([stroke[int(i)] for i in idx])
    return out, True


def strokes_to_path_text(strokes: list[list[dict[str, float]]], max_points: int = WEB_EXPORT_MAX_POINTS) -> tuple[str, bool, int]:
    web_strokes, reduced = limit_stroke_dict_points(strokes, max_points)
    lines = []
    for stroke in web_strokes:
        if len(stroke) >= 2:
            lines.append(";".join(f"{p['x']:.2f},{p['y']:.2f}" for p in stroke))
    body = "\n".join(lines) + "\n"
    return body, reduced, sum(len(s) for s in web_strokes)


def usb_upload_lines(strokes: list[list[dict[str, float]]], max_points: int = WEB_EXPORT_MAX_POINTS) -> tuple[list[str], bool, int]:
    web_strokes, reduced = limit_stroke_dict_points(strokes, max_points)
    lines = ["USBPATH BEGIN"]
    point_count = 0
    for stroke in web_strokes:
        if len(stroke) < 2:
            continue
        first = stroke[0]
        lines.append(f"S,{first['x']:.2f},{first['y']:.2f}")
        point_count += 1
        for point in stroke[1:]:
            lines.append(f"P,{point['x']:.2f},{point['y']:.2f}")
            point_count += 1
    lines.append("USBPATH END")
    return lines, reduced, point_count


def usb_upload_binary(strokes: list[list[dict[str, float]]], max_points: int = WEB_EXPORT_MAX_POINTS) -> tuple[bytes, bool, int]:
    web_strokes, reduced = limit_stroke_dict_points(strokes, max_points)
    packets = bytearray()
    point_count = 0
    for stroke in web_strokes:
        if len(stroke) < 2:
            continue
        for index, point in enumerate(stroke):
            x100 = max(0, min(int(round(point["x"] * 100.0)), int(CANVAS_MM * 100)))
            y100 = max(0, min(int(round(point["y"] * 100.0)), int(CANVAS_MM * 100)))
            draw = 0 if index == 0 else 1
            packets.extend(struct.pack("<HHB", x100, y100, draw))
            point_count += 1
    return bytes(packets), reduced, point_count


def write_path_txt(path: str, strokes: list[list[dict[str, float]]]) -> tuple[bool, int]:
    body, reduced, point_count = strokes_to_path_text(strokes)
    Path(path).write_text(body, encoding="utf-8")
    return reduced, point_count


def list_serial_port_labels() -> list[str]:
    try:
        from serial.tools import list_ports  # type: ignore
    except Exception:
        return [DEFAULT_USB_PORT]

    ports = list(list_ports.comports())

    def score(item: object) -> tuple[int, str]:
        device = str(getattr(item, "device", ""))
        desc = str(getattr(item, "description", ""))
        text = f"{device} {desc}".lower()
        value = 100
        if "bluetooth" in text or "蓝牙" in text:
            value += 100
        if "ch343" in text or "usb-enhanced" in text:
            value -= 40
        if "usb" in text:
            value -= 20
        if device.upper() == DEFAULT_USB_PORT:
            value -= 10
        return value, device

    labels = []
    for item in sorted(ports, key=score):
        device = str(getattr(item, "device", ""))
        desc = str(getattr(item, "description", "")).strip()
        labels.append(f"{device} - {desc}" if desc else device)
    return labels or [DEFAULT_USB_PORT]


def port_from_label(label: str) -> str:
    return label.split(" - ", 1)[0].strip() or DEFAULT_USB_PORT


class TrajectoryStudio(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("轨迹图片生成器")
        self.geometry("1280x980")
        self.minsize(980, 650)
        self.source_path: str | None = None
        self.source_image: np.ndarray | None = None
        self.result: TrajectoryResult | None = None
        self.source_preview_file = Path(tempfile.gettempdir()) / "trajectory_studio_source.png"
        self.result_preview_file = Path(tempfile.gettempdir()) / "trajectory_studio_result.png"
        self.source_photo: tk.PhotoImage | None = None
        self.result_photo: tk.PhotoImage | None = None
        self.processing = False
        self.serial_port = tk.StringVar(value=DEFAULT_USB_PORT)
        self.port_combo: ttk.Combobox | None = None
        self._build_ui()
        self.refresh_serial_ports()

    def _build_ui(self) -> None:
        root = ttk.Frame(self, padding=10)
        root.pack(fill=tk.BOTH, expand=True)
        left = ttk.Frame(root)
        left.pack(side=tk.LEFT, fill=tk.Y)
        right = ttk.Frame(root)
        right.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        ttk.Button(left, text="打开图片", command=self.open_image).pack(fill=tk.X, pady=(0, 8))
        ttk.Button(left, text="处理成轨迹", command=self.process).pack(fill=tk.X, pady=(0, 8))
        ttk.Button(left, text="导出轨迹 TXT", command=self.export_txt).pack(fill=tk.X, pady=(0, 8))
        ttk.Button(left, text="导出 JSON", command=self.export_json).pack(fill=tk.X, pady=(0, 8))
        ttk.Button(left, text="导出预览 PNG", command=self.export_png).pack(fill=tk.X, pady=(0, 8))
        ttk.Button(left, text="WiFi上传到板子", command=self.upload_to_board).pack(fill=tk.X, pady=(0, 8))
        ttk.Button(left, text="打开USB调试网页", command=self.open_usb_debug_web).pack(fill=tk.X, pady=(0, 8))
        usb_row = ttk.Frame(left)
        usb_row.pack(fill=tk.X, pady=(0, 8))
        ttk.Label(usb_row, text="USB端口").pack(side=tk.LEFT)
        self.port_combo = ttk.Combobox(usb_row, textvariable=self.serial_port, values=[DEFAULT_USB_PORT], width=24)
        self.port_combo.pack(side=tk.RIGHT)
        ttk.Button(left, text="刷新USB端口", command=self.refresh_serial_ports).pack(fill=tk.X, pady=(0, 8))
        ttk.Button(left, text="USB上传轨迹", command=self.upload_to_usb).pack(fill=tk.X, pady=(0, 16))

        self.mode = tk.StringVar(value=MODE_SURPRISE)
        self.invert = tk.BooleanVar(value=False)
        self.threshold = tk.IntVar(value=180)
        self.canny_low = tk.IntVar(value=45)
        self.canny_high = tk.IntVar(value=120)
        self.close_iter = tk.IntVar(value=1)
        self.min_area = tk.IntVar(value=12)
        self.min_segment = tk.IntVar(value=8)
        self.spacing = tk.DoubleVar(value=0.55)
        self.simplify = tk.DoubleVar(value=0.10)
        self.margin = tk.DoubleVar(value=8.0)
        self.max_points = tk.IntVar(value=250000)
        self.max_side = tk.IntVar(value=960)

        self.combo(left, "模式", self.mode, [MODE_SURPRISE, MODE_PHOTO_DITHER, MODE_FULL_SKETCH, MODE_PHOTO_SUBJECT, MODE_AUTO, MODE_DARK_LINE, MODE_DARK_CONTOUR, MODE_EDGE, MODE_OUTLINE, MODE_CENTERLINE])
        self.check(left, "反相处理", self.invert)
        self.slider(left, "黑线阈值", self.threshold, 40, 245, 1)
        self.slider(left, "边缘低阈值", self.canny_low, 10, 220, 1)
        self.slider(left, "边缘高阈值", self.canny_high, 20, 300, 1)
        self.slider(left, "连接强度", self.close_iter, 0, 4, 1)
        self.slider(left, "去噪面积", self.min_area, 1, 120, 1)
        self.slider(left, "最短轨迹(px)", self.min_segment, 2, 80, 1)
        self.slider(left, "点间距(mm)", self.spacing, 0.25, 2.0, 0.05)
        self.slider(left, "平滑容差(mm)", self.simplify, 0.0, 1.5, 0.05)
        self.slider(left, "边距(mm)", self.margin, 2.0, 25.0, 0.5)
        self.slider(left, "最多点数", self.max_points, 1000, 300000, 1000)
        self.slider(left, "处理分辨率", self.max_side, 360, 1600, 20)

        self.status = tk.StringVar(value="打开一张图片开始。")
        ttk.Label(right, textvariable=self.status, anchor=tk.W).pack(fill=tk.X, pady=(0, 8))
        ttk.Label(right, text="原图", anchor=tk.W).pack(fill=tk.X)
        self.source_label = ttk.Label(right, anchor=tk.CENTER)
        self.source_label.pack(fill=tk.BOTH, expand=True)
        ttk.Label(right, text="转换结果", anchor=tk.W).pack(fill=tk.X, pady=(8, 0))
        self.result_label = ttk.Label(right, anchor=tk.CENTER)
        self.result_label.pack(fill=tk.BOTH, expand=True)

    def combo(self, parent: ttk.Frame, label: str, var: tk.StringVar, values: list[str]) -> None:
        ttk.Label(parent, text=label).pack(anchor=tk.W)
        ttk.Combobox(parent, textvariable=var, values=values, state="readonly", width=18).pack(fill=tk.X, pady=(0, 8))

    def refresh_serial_ports(self) -> None:
        labels = list_serial_port_labels()
        if self.port_combo is not None:
            self.port_combo.configure(values=labels)
        current_port = port_from_label(self.serial_port.get())
        selected = next((label for label in labels if port_from_label(label).upper() == current_port.upper()), labels[0])
        self.serial_port.set(selected)

    def check(self, parent: ttk.Frame, text: str, var: tk.BooleanVar) -> None:
        ttk.Checkbutton(parent, text=text, variable=var).pack(anchor=tk.W, pady=(0, 8))

    def slider(self, parent: ttk.Frame, label: str, var: tk.Variable, lo: float, hi: float, step: float) -> None:
        row = ttk.Frame(parent)
        row.pack(fill=tk.X, pady=(0, 6))
        ttk.Label(row, text=label).pack(side=tk.LEFT)
        value = ttk.Label(row, width=7, anchor=tk.E)
        value.pack(side=tk.RIGHT)

        def update_value(*_: object) -> None:
            raw = float(var.get())
            if isinstance(var, tk.DoubleVar):
                value.config(text=f"{raw:.2f}")
            else:
                value.config(text=f"{int(raw)}")

        scale = tk.Scale(
            parent,
            from_=lo,
            to=hi,
            resolution=step,
            orient=tk.HORIZONTAL,
            variable=var,
            showvalue=False,
            length=190,
        )
        var.trace_add("write", update_value)
        update_value()
        scale.pack(fill=tk.X, pady=(0, 8))

    def open_image(self) -> None:
        path = filedialog.askopenfilename(
            title="选择图片",
            filetypes=[
                ("常用图片", "*.png;*.jpg;*.jpeg;*.bmp;*.tif;*.tiff;*.webp;*.gif"),
                ("所有文件", "*.*"),
            ],
        )
        if not path:
            return
        self.load_image(path)

    def load_image(self, path: str) -> None:
        img = imread_unicode(path)
        if img is None:
            messagebox.showerror("读取失败", "OpenCV 无法读取这张图片。请换 PNG/JPG/BMP/TIFF/WebP，或先另存为 PNG。")
            return
        self.source_path = path
        self.source_image = img
        self.result = None
        imwrite_unicode(str(self.source_preview_file), fit_for_display(render_source_preview(img)))
        self.source_photo = tk.PhotoImage(file=str(self.source_preview_file))
        self.source_label.configure(image=self.source_photo)
        blank = np.full((DISPLAY_SIZE, DISPLAY_SIZE, 3), 255, np.uint8)
        imwrite_unicode(str(self.result_preview_file), blank)
        self.result_photo = tk.PhotoImage(file=str(self.result_preview_file))
        self.result_label.configure(image=self.result_photo)
        self.status.set(f"已打开: {Path(path).name}    正在自动转换...")
        self.process()

    def process(self) -> None:
        if self.source_image is None:
            messagebox.showinfo("没有图片", "请先打开一张图片。")
            return
        if self.processing:
            return
        self.processing = True
        self.status.set("正在处理，请稍等...")
        args = (
            self.source_image.copy(),
            self.mode.get(),
            self.threshold.get(),
            self.canny_low.get(),
            self.canny_high.get(),
            self.invert.get(),
            self.max_side.get(),
            self.min_segment.get(),
            self.min_area.get(),
            self.close_iter.get(),
            self.margin.get(),
            self.spacing.get(),
            self.simplify.get(),
            self.max_points.get(),
        )

        def worker() -> None:
            try:
                result = build_trajectory(*args)
                self.after(0, lambda: self.finish_process(result, None))
            except Exception as exc:
                err = exc
                self.after(0, lambda: self.finish_process(None, err))

        threading.Thread(target=worker, daemon=True).start()

    def finish_process(self, result: TrajectoryResult | None, error: Exception | None) -> None:
        self.processing = False
        if error is not None:
            messagebox.showerror("处理失败", str(error))
            self.status.set("处理失败。")
            return
        if result is None:
            self.status.set("处理失败。")
            return
        self.result = result
        imwrite_unicode(str(self.result_preview_file), fit_for_display(result.preview))
        self.result_photo = tk.PhotoImage(file=str(self.result_preview_file))
        self.result_label.configure(image=self.result_photo)
        name = Path(self.source_path).name if self.source_path else ""
        self.status.set(f"{name}    {result.stats}")

    def ensure_result(self) -> bool:
        if not self.result or not self.result.strokes:
            messagebox.showinfo("没有轨迹", "请先处理图片，并确认预览里有轨迹。")
            return False
        return True

    def export_txt(self) -> None:
        if not self.ensure_result():
            return
        path = filedialog.asksaveasfilename(defaultextension=".txt", filetypes=[("轨迹 TXT", "*.txt")])
        if path:
            reduced, point_count = write_path_txt(path, self.result.strokes)
            note = "，已压缩为板子安全版" if reduced else ""
            self.status.set(f"已导出轨迹 TXT: {path}    {point_count} 点{note}")

    def upload_to_board(self) -> None:
        if not self.ensure_result():
            return
        body, reduced, point_count = strokes_to_path_text(self.result.strokes)
        self.status.set(f"正在上传到板子 {BOARD_URL} ...")

        def worker() -> None:
            try:
                req = urllib.request.Request(
                    f"{BOARD_URL}/path",
                    data=body.encode("utf-8"),
                    headers={"Content-Type": "text/plain; charset=utf-8"},
                    method="POST",
                )
                with urllib.request.urlopen(req, timeout=12) as res:
                    msg = res.read().decode("utf-8", errors="replace")
                note = "，已压缩为板子安全版" if reduced else ""
                self.after(0, lambda: self.status.set(f"上传完成: {msg}    {point_count} 点{note}，到网页点开始即可绘制"))
            except (OSError, urllib.error.URLError) as exc:
                self.after(0, lambda: messagebox.showerror("上传失败", f"请先让电脑连接 MassageDraw 热点，再重试。\n\n{exc}"))
                self.after(0, lambda: self.status.set("上传失败：电脑没有连到板子热点，或网页服务未启动。"))

        threading.Thread(target=worker, daemon=True).start()

    def run_usb_debug_server_thread(self) -> None:
        try:
            import usb_debug_server  # type: ignore

            usb_debug_server.run(open_browser=True)
            self.status.set("USB调试网页已启动: http://127.0.0.1:8765/")
        except OSError as exc:
            text = str(exc)
            if "10048" in text or "Address already in use" in text or "address already in use" in text:
                webbrowser.open("http://127.0.0.1:8765/")
                self.after(0, lambda: self.status.set("USB debug web: http://127.0.0.1:8765/"))
            else:
                self.after(0, lambda exc=exc: messagebox.showerror("启动失败", str(exc)))
        except Exception as exc:
            messagebox.showerror("启动失败", str(exc))

    def open_usb_debug_web(self) -> None:
        threading.Thread(target=self.run_usb_debug_server_thread, daemon=True).start()
        self.status.set("USB调试网页已启动: http://127.0.0.1:8765/")

    def upload_to_usb(self) -> None:
        if not self.ensure_result():
            return

        port = port_from_label(self.serial_port.get())
        payload, reduced, point_count = usb_upload_binary(self.result.strokes)
        self.status.set(f"正在通过 USB {port} 上传 {point_count} 点...")

        def worker() -> None:
            try:
                import serial  # type: ignore

                with serial.Serial(port, USB_BAUD, timeout=0.8, write_timeout=0, rtscts=False, dsrdtr=False) as ser:
                    ser.dtr = False
                    ser.rts = False
                    ser.reset_input_buffer()
                    time.sleep(2.0)
                    ser.write(f"USBBIN {point_count}\n".encode("ascii"))
                    ready_until = time.monotonic() + 5.0
                    ready_lines: list[str] = []
                    while time.monotonic() < ready_until:
                        raw = ser.readline()
                        if not raw:
                            continue
                        text = raw.decode("utf-8", errors="replace").strip()
                        if text:
                            ready_lines.append(text)
                        if text.startswith("USBBIN READY"):
                            break
                        if text.startswith("USBBIN ERROR"):
                            raise RuntimeError(text)
                    else:
                        raise RuntimeError("板子没有进入 USB 二进制接收模式")

                    chunk = 4096
                    for start in range(0, len(payload), chunk):
                        ser.write(payload[start : start + chunk])
                        time.sleep(0.002)

                    reply_lines: list[str] = []
                    end_at = time.monotonic() + 8.0
                    while time.monotonic() < end_at:
                        raw = ser.readline()
                        if not raw:
                            continue
                        text = raw.decode("utf-8", errors="replace").strip()
                        if text:
                            reply_lines.append(text)
                        if text.startswith("USBBIN OK") or text.startswith("USBBIN ERROR") or text.startswith("USBPATH OK") or text.startswith("USBPATH ERROR"):
                            break

                summary = next((line for line in reversed(reply_lines) if line.startswith("USBBIN") or line.startswith("USBPATH")), "")
                if not summary:
                    summary = "未收到板子确认，但数据已发出"
                note = "，已压缩为板子安全版" if reduced else ""
                self.after(0, lambda: self.status.set(f"USB上传完成: {summary}    {point_count} 点{note}，网页或串口发 webstart 可开始"))
            except Exception as exc:
                self.after(0, lambda: messagebox.showerror("USB上传失败", f"确认板子在 {port}，且没有被串口监视器占用。\n\n{exc}"))
                self.after(0, lambda: self.status.set("USB上传失败：端口不可用或被占用。"))

        threading.Thread(target=worker, daemon=True).start()

    def export_json(self) -> None:
        if not self.ensure_result():
            return
        path = filedialog.asksaveasfilename(defaultextension=".json", filetypes=[("JSON", "*.json")])
        if path:
            Path(path).write_text(json.dumps(self.result.strokes, ensure_ascii=False, separators=(",", ":")), encoding="utf-8")
            self.status.set(f"已导出 JSON: {path}")

    def export_png(self) -> None:
        if not self.ensure_result():
            return
        path = filedialog.asksaveasfilename(defaultextension=".png", filetypes=[("PNG", "*.png")])
        if path:
            imwrite_unicode(path, self.result.preview)
            self.status.set(f"已导出预览 PNG: {path}")


def main() -> None:
    app = TrajectoryStudio()
    if len(sys.argv) > 1:
        image_path = sys.argv[1]
        app.after(250, lambda: app.load_image(image_path))
    app.mainloop()


if __name__ == "__main__":
    main()
