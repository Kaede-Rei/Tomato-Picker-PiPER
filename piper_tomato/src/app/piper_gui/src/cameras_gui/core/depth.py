from __future__ import annotations

from typing import Tuple

import cv2
import numpy as np

from cameras_gui.core.selection import preprocess_selection_mask


def depth_to_pointcloud(
    u: int,
    v: int,
    depth_m: float,
    intrinsics: np.ndarray,
) -> Tuple[float, float, float]:
    fx, fy = intrinsics[0, 0], intrinsics[1, 1]
    cx, cy = intrinsics[0, 2], intrinsics[1, 2]

    x = (u - cx) * depth_m / fx
    y = (v - cy) * depth_m / fy
    z = depth_m

    return float(x), float(y), float(z)


def trimmed_median(values: np.ndarray, trim_ratio: float) -> float:
    if values.size == 0:
        return 0.0

    values = np.sort(values.astype(np.float64))
    trim_ratio = float(max(0.0, min(0.45, trim_ratio)))
    trim_n = int(values.size * trim_ratio)

    if trim_n > 0 and values.size > 2 * trim_n:
        values = values[trim_n:-trim_n]

    return float(np.median(values))


def get_valid_depth_around(
    depth_mm: np.ndarray,
    u: int,
    v: int,
    mask: np.ndarray,
    kernel_size: int = 7,
    trim_ratio: float = 0.15,
    close_kernel: int = 5,
    open_kernel: int = 3,
    center_band_min_dist_px: float = 1.0,
) -> float:
    h, w = depth_mm.shape[:2]

    if mask.shape[:2] != (h, w):
        mask = cv2.resize(mask, (w, h), interpolation=cv2.INTER_NEAREST)

    binary_mask = preprocess_selection_mask(mask, close_kernel, open_kernel)

    kernel_size = max(3, int(kernel_size))
    if kernel_size % 2 == 0:
        kernel_size += 1

    half = kernel_size // 2
    start_x, end_x = max(0, u - half), min(w, u + half + 1)
    start_y, end_y = max(0, v - half), min(h, v + half + 1)

    distance_map = cv2.distanceTransform(binary_mask, cv2.DIST_L2, 5)
    local_radius = float(distance_map[v, u]) if 0 <= v < h and 0 <= u < w else 0.0
    center_threshold = max(center_band_min_dist_px, local_radius * 0.5)

    center_mask = (distance_map >= center_threshold).astype(np.uint8) * 255

    erode_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    eroded_mask = cv2.erode(binary_mask, erode_kernel, iterations=1)

    for candidate_mask in [center_mask, eroded_mask, binary_mask]:
        depth_roi = depth_mm[start_y:end_y, start_x:end_x]
        mask_roi = candidate_mask[start_y:end_y, start_x:end_x]

        valid = (mask_roi > 0) & (depth_roi > 0)
        valid_depths = depth_roi[valid]

        if valid_depths.size >= 3:
            return trimmed_median(valid_depths, trim_ratio)

    full_valid = depth_mm[(binary_mask > 0) & (depth_mm > 0)]
    if full_valid.size == 0:
        return 0.0

    return trimmed_median(full_valid, trim_ratio)
