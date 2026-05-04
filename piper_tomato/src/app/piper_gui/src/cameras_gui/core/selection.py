from collections import deque
from typing import Optional, Tuple

import numpy as np


def get_mask_center(mask: np.ndarray) -> Optional[Tuple[float, float]]:
    ys, xs = np.where(mask > 0)
    if len(xs) == 0:
        return None
    return float(xs.mean()), float(ys.mean())


def build_polygon_mask(
    width: int,
    height: int,
    polygon_points: list[dict],
) -> np.ndarray:
    mask = np.zeros((height, width), dtype=np.uint8)

    if len(polygon_points) < 3:
        return mask

    poly = np.array(
        [[float(p["x"]), float(p["y"])] for p in polygon_points],
        dtype=np.float64,
    )

    xs = np.arange(width, dtype=np.float64) + 0.5
    ys = np.arange(height, dtype=np.float64) + 0.5
    xx, yy = np.meshgrid(xs, ys)

    inside = np.zeros((height, width), dtype=bool)

    xj, yj = poly[-1]
    for xi, yi in poly:
        cross_y = (yi > yy) != (yj > yy)
        x_intersect = (xj - xi) * (yy - yi) / ((yj - yi) + 1e-12) + xi
        inside ^= cross_y & (xx < x_intersect)
        xj, yj = xi, yi

    mask[inside] = 255
    return mask


def _binary_erosion(mask_bool: np.ndarray, structure: np.ndarray) -> np.ndarray:
    h, w = mask_bool.shape
    kh, kw = structure.shape
    ph, pw = kh // 2, kw // 2

    padded = np.pad(
        mask_bool, ((ph, ph), (pw, pw)), mode="constant", constant_values=False
    )
    out = np.ones_like(mask_bool, dtype=bool)

    ys, xs = np.where(structure)
    for dy, dx in zip(ys, xs):
        out &= padded[dy : dy + h, dx : dx + w]

    return out


def _binary_dilation(mask_bool: np.ndarray, structure: np.ndarray) -> np.ndarray:
    h, w = mask_bool.shape
    kh, kw = structure.shape
    ph, pw = kh // 2, kw // 2

    padded = np.pad(
        mask_bool, ((ph, ph), (pw, pw)), mode="constant", constant_values=False
    )
    out = np.zeros_like(mask_bool, dtype=bool)

    ys, xs = np.where(structure)
    for dy, dx in zip(ys, xs):
        out |= padded[dy : dy + h, dx : dx + w]

    return out


def _make_ellipse_kernel(size: int) -> np.ndarray:
    size = max(1, int(size))
    if size % 2 == 0:
        size += 1

    r = size // 2
    yy, xx = np.ogrid[-r : r + 1, -r : r + 1]

    if r <= 0:
        return np.ones((1, 1), dtype=bool)

    return (xx * xx + yy * yy) <= (r * r)


def preprocess_selection_mask(
    mask: np.ndarray,
    close_kernel: int = 5,
    open_kernel: int = 3,
) -> np.ndarray:
    binary = mask > 0

    if binary.size == 0:
        return mask.astype(np.uint8)

    close_k = _make_ellipse_kernel(close_kernel)
    open_k = _make_ellipse_kernel(open_kernel)

    binary = _binary_dilation(binary, close_k)
    binary = _binary_erosion(binary, close_k)

    binary = _binary_erosion(binary, open_k)
    binary = _binary_dilation(binary, open_k)

    return binary.astype(np.uint8) * 255


def _connected_components(binary: np.ndarray) -> list[np.ndarray]:
    mask = binary > 0
    h, w = mask.shape
    visited = np.zeros_like(mask, dtype=bool)
    components: list[np.ndarray] = []

    neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    ys, xs = np.where(mask)
    for sy, sx in zip(ys, xs):
        if visited[sy, sx]:
            continue

        q = deque()
        q.append((int(sy), int(sx)))
        visited[sy, sx] = True
        pts = []

        while q:
            y, x = q.popleft()
            pts.append((y, x))

            for dy, dx in neighbors:
                ny, nx = y + dy, x + dx
                if 0 <= ny < h and 0 <= nx < w:
                    if mask[ny, nx] and not visited[ny, nx]:
                        visited[ny, nx] = True
                        q.append((ny, nx))

        components.append(np.array(pts, dtype=np.int32))

    return components


def keep_largest_component(binary_mask: np.ndarray) -> np.ndarray:
    comps = _connected_components(binary_mask)
    if not comps:
        return np.zeros_like(binary_mask, dtype=np.uint8)

    largest = max(comps, key=lambda c: len(c))
    out = np.zeros_like(binary_mask, dtype=np.uint8)
    out[largest[:, 0], largest[:, 1]] = 255
    return out


def build_morphological_skeleton(binary_mask: np.ndarray) -> np.ndarray:
    work = binary_mask > 0
    if work.size == 0 or not np.any(work):
        return np.zeros_like(binary_mask, dtype=np.uint8)

    skeleton = np.zeros_like(work, dtype=bool)

    cross = np.array(
        [
            [False, True, False],
            [True, True, True],
            [False, True, False],
        ],
        dtype=bool,
    )

    while np.any(work):
        eroded = _binary_erosion(work, cross)
        opened = _binary_dilation(eroded, cross)
        residue = work & (~opened)
        skeleton |= residue
        work = eroded

    return skeleton.astype(np.uint8) * 255


def _nearest_skeleton_point(
    skeleton: np.ndarray,
    center: Tuple[float, float],
) -> Optional[Tuple[float, float]]:
    skel_y, skel_x = np.where(skeleton > 0)

    if len(skel_x) == 0:
        return None

    dx = skel_x.astype(np.float64) - float(center[0])
    dy = skel_y.astype(np.float64) - float(center[1])
    idx = int(np.argmin(dx * dx + dy * dy))

    return float(skel_x[idx]), float(skel_y[idx])


def _principal_axis_center_point(
    binary_mask: np.ndarray,
    center: Tuple[float, float],
) -> Tuple[float, float]:
    ys, xs = np.where(binary_mask > 0)

    if len(xs) == 0:
        return center

    pts = np.stack([xs.astype(np.float64), ys.astype(np.float64)], axis=1)
    mean = pts.mean(axis=0)
    pts0 = pts - mean

    if pts0.shape[0] < 3:
        return float(mean[0]), float(mean[1])

    cov = pts0.T @ pts0 / max(1, pts0.shape[0] - 1)
    eigvals, eigvecs = np.linalg.eigh(cov)

    main_axis = eigvecs[:, int(np.argmax(eigvals))]
    proj = pts0 @ main_axis

    center_arr = np.array([center[0], center[1]], dtype=np.float64)
    center_proj = (center_arr - mean) @ main_axis
    idx = int(np.argmin(np.abs(proj - center_proj)))

    return float(pts[idx, 0]), float(pts[idx, 1])


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
    binary = keep_largest_component(binary)

    local_center = (stem_center[0] - x0, stem_center[1] - y0)

    if method == "centroid":
        return float(stem_center[0]), float(stem_center[1])

    if method == "pca_center":
        px, py = _principal_axis_center_point(binary, local_center)
        return float(px + x0), float(py + y0)

    skeleton = build_morphological_skeleton(binary)
    pt = _nearest_skeleton_point(skeleton, local_center)

    if pt is None:
        px, py = _principal_axis_center_point(binary, local_center)
        return float(px + x0), float(py + y0)

    px, py = pt
    return float(px + x0), float(py + y0)


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
