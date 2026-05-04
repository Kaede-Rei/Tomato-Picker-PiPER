from __future__ import annotations

from typing import Optional, Tuple

import cv2
import numpy as np


def get_mask_center(mask: np.ndarray) -> Optional[Tuple[float, float]]:
    M = cv2.moments(mask.astype(np.uint8))
    if M["m00"] == 0:
        return None
    return float(M["m10"] / M["m00"]), float(M["m01"] / M["m00"])


def preprocess_selection_mask(
    mask: np.ndarray,
    close_kernel: int = 5,
    open_kernel: int = 3,
) -> np.ndarray:
    binary_mask = (mask > 0).astype(np.uint8) * 255
    if binary_mask.size == 0:
        return binary_mask

    close_kernel = max(1, int(close_kernel))
    open_kernel = max(1, int(open_kernel))
    if close_kernel % 2 == 0:
        close_kernel += 1
    if open_kernel % 2 == 0:
        open_kernel += 1

    kernel_close = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (close_kernel, close_kernel)
    )
    kernel_open = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (open_kernel, open_kernel)
    )

    refined = cv2.morphologyEx(binary_mask, cv2.MORPH_CLOSE, kernel_close)
    refined = cv2.morphologyEx(refined, cv2.MORPH_OPEN, kernel_open)
    return refined


def build_morphological_skeleton(binary_mask: np.ndarray) -> np.ndarray:
    work = (binary_mask > 0).astype(np.uint8) * 255
    if work.size == 0 or cv2.countNonZero(work) == 0:
        return np.zeros_like(work)

    skel = np.zeros_like(work)
    element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))

    while True:
        eroded = cv2.erode(work, element)
        opened = cv2.dilate(eroded, element)
        temp = cv2.subtract(work, opened)
        skel = cv2.bitwise_or(skel, temp)
        work = eroded
        if cv2.countNonZero(work) == 0:
            break

    return skel


def find_cutting_point_from_convexity_defect(
    binary_mask: np.ndarray,
    stem_center: Tuple[float, float],
) -> Tuple[float, float]:
    contours, _ = cv2.findContours(
        binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    if not contours:
        return stem_center

    largest = max(contours, key=cv2.contourArea)
    epsilon = 0.001 * cv2.arcLength(largest, True)
    largest = cv2.approxPolyDP(largest, epsilon, True)
    hull = cv2.convexHull(largest, returnPoints=False)

    try:
        defects = cv2.convexityDefects(largest, hull)
        if defects is None:
            return stem_center

        candidates = []
        for i in range(defects.shape[0]):
            _, _, f, _ = defects[i, 0]
            candidates.append(tuple(largest[f][0]))

        if not candidates:
            return stem_center

        return min(
            candidates,
            key=lambda p: (p[0] - stem_center[0]) ** 2 + (p[1] - stem_center[1]) ** 2,
        )
    except cv2.error:
        return stem_center


def find_cutting_point(
    stem_mask: np.ndarray,
    stem_center: Tuple[float, float],
    method: str = "skeleton_centroid",
    close_kernel: int = 5,
    open_kernel: int = 3,
) -> Tuple[float, float]:
    ys, xs = np.where(stem_mask > 0)
    if len(xs) == 0:
        return stem_center

    x0, x1 = max(0, int(xs.min()) - 2), int(xs.max()) + 3
    y0, y1 = max(0, int(ys.min()) - 2), int(ys.max()) + 3

    cropped = stem_mask[y0:y1, x0:x1]
    binary = preprocess_selection_mask(cropped, close_kernel, open_kernel)
    local_center = (stem_center[0] - x0, stem_center[1] - y0)

    if method == "centroid":
        return stem_center

    if method == "legacy_defect":
        px, py = find_cutting_point_from_convexity_defect(binary, local_center)
        return float(px + x0), float(py + y0)

    skeleton = build_morphological_skeleton(binary)
    skel_y, skel_x = np.where(skeleton > 0)

    if len(skel_x) == 0:
        px, py = find_cutting_point_from_convexity_defect(binary, local_center)
        return float(px + x0), float(py + y0)

    dx = skel_x.astype(np.float64) - float(local_center[0])
    dy = skel_y.astype(np.float64) - float(local_center[1])
    idx = int(np.argmin(dx * dx + dy * dy))

    return float(skel_x[idx] + x0), float(skel_y[idx] + y0)


def build_polygon_mask(
    width: int,
    height: int,
    polygon_points: list[dict],
) -> np.ndarray:
    mask = np.zeros((height, width), dtype=np.uint8)
    if len(polygon_points) < 3:
        return mask

    pts = np.array(
        [[int(round(p["x"])), int(round(p["y"]))] for p in polygon_points],
        dtype=np.int32,
    )
    cv2.fillPoly(mask, [pts], 255)
    return mask


def select_cutting_pixel_from_polygon(
    width: int,
    height: int,
    polygon_points: list[dict],
    method: str,
    close_kernel: int,
    open_kernel: int,
) -> tuple[tuple[float, float], np.ndarray]:
    mask = build_polygon_mask(width, height, polygon_points)
    center = get_mask_center(mask)

    if center is None:
        raise RuntimeError("ROI mask 为空，无法计算中心点")

    cutting_point = find_cutting_point(
        mask,
        center,
        method=method,
        close_kernel=close_kernel,
        open_kernel=open_kernel,
    )

    return cutting_point, mask
