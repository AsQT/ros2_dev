"""Depth image helpers: ROI median, unit conversion, invalid filtering, single-pixel raw read."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


def depth_at_pixel(
    depth_image: np.ndarray,
    u: int,
    v: int,
    encoding: str,
) -> Tuple[Optional[int], Optional[float], str]:
    """Read the raw depth value and converted distance at a single pixel.

    Args:
        depth_image: 2D depth array (H, W).
        u: Column index (x / width direction).
        v: Row index (y / height direction).
        encoding: sensor_msgs/Image encoding, e.g. ``16UC1`` or ``32FC1``.

    Returns:
        Tuple of (raw_value, dist_meters, encoding).
        - raw_value: int for 16UC1 (millimetres), float for 32FC1 (metres), or None if OOB.
        - dist_meters: depth in metres, or None if invalid/zero.
        - encoding: echoed back so callers don't need to track it separately.
    """
    if depth_image is None or depth_image.size == 0:
        return None, None, encoding

    h, w = depth_image.shape[:2]
    if u < 0 or u >= w or v < 0 or v >= h:
        return None, None, encoding

    raw = depth_image[v, u]

    if encoding in ("16UC1", "mono16"):
        raw_int = int(raw)
        if raw_int == 0:
            return raw_int, None, encoding
        return raw_int, raw_int * 0.001, encoding

    if encoding in ("32FC1",):
        if not math.isfinite(raw) or raw <= 0.0:
            return int(raw), None, encoding
        return int(raw), float(raw), encoding

    # Fallback: best-effort interpretation
    if np.issubdtype(depth_image.dtype, np.floating):
        if not math.isfinite(raw) or raw <= 0.0:
            return None, None, encoding
        return None, float(raw), encoding
    else:
        raw_int = int(raw)
        if raw_int == 0:
            return raw_int, None, encoding
        return raw_int, raw_int * 0.001, encoding


def median_depth_meters(
    depth_image: np.ndarray,
    center_x: int,
    center_y: int,
    half_size: int,
    encoding: str,
) -> Tuple[float, Optional[int]]:
    """Return median depth in meters for a square ROI around (center_x, center_y).

    Invalid values (0, NaN, Inf) are ignored. If the depth image is uint16,
    values are treated as millimeters and converted to meters (RealSense
    aligned depth convention).

    Args:
        depth_image: 2D depth array (H, W).
        center_x: Column index of ROI center.
        center_y: Row index of ROI center.
        half_size: Half side length of the square ROI in pixels (inclusive).
        encoding: sensor_msgs/Image encoding, e.g. ``16UC1`` or ``32FC1``.

    Returns:
        Tuple of (median_depth_meters, roi_median_raw).
        - median_depth_meters: median depth in metres, or ``-1.0`` if no valid samples exist.
        - roi_median_raw: raw uint16 median in mm (for 16UC1), raw float median in metres
          (for 32FC1), or None if no valid samples.
        The caller can display ``roi_median_raw`` to show the raw depth value alongside
        the converted metres value.
    """
    if depth_image is None or depth_image.size == 0:
        return -1.0, None

    h, w = depth_image.shape[:2]
    if h < 1 or w < 1:
        return -1.0, None

    x0 = max(0, center_x - half_size)
    y0 = max(0, center_y - half_size)
    x1 = min(w, center_x + half_size + 1)
    y1 = min(h, center_y + half_size + 1)
    if x0 >= x1 or y0 >= y1:
        return -1.0, None

    roi = depth_image[y0:y1, x0:x1]

    if encoding in ("16UC1", "mono16"):
        roi_f = roi.astype(np.float64)
        valid = roi_f > 0.0
        if not np.any(valid):
            return -1.0, None
        mm = roi_f[valid]
        meters = mm * 0.001
        raw_median = float(np.median(mm))
        roi_median_raw = int(round(raw_median)) if math.isfinite(raw_median) else None
    elif encoding in ("32FC1",):
        roi_f = roi.astype(np.float64)
        valid = np.isfinite(roi_f) & (roi_f > 0.0)
        if not np.any(valid):
            return -1.0, None
        meters = roi_f[valid]
        raw_median = float(np.median(meters))
        roi_median_raw = raw_median if math.isfinite(raw_median) else None
    else:
        # Best effort: treat as float meters if float-like, else mm uint
        if np.issubdtype(roi.dtype, np.floating):
            roi_f = roi.astype(np.float64)
            valid = np.isfinite(roi_f) & (roi_f > 0.0)
            if not np.any(valid):
                return -1.0, None
            meters = roi_f[valid]
            raw_median = float(np.median(meters))
            roi_median_raw = raw_median if math.isfinite(raw_median) else None
        else:
            roi_f = roi.astype(np.float64)
            valid = roi_f > 0.0
            if not np.any(valid):
                return -1.0, None
            mm = roi_f[valid]
            meters = mm * 0.001
            raw_median = float(np.median(mm))
            roi_median_raw = int(round(raw_median)) if math.isfinite(raw_median) else None

    med = float(np.median(meters))
    if not math.isfinite(med) or med <= 0.0:
        return -1.0, None
    return med, roi_median_raw


def bbox_clip_to_image(
    x_min: int,
    y_min: int,
    x_max: int,
    y_max: int,
    width: int,
    height: int,
) -> Tuple[int, int, int, int]:
    """Clip bbox integer coordinates to image bounds."""
    x_min = int(max(0, min(x_min, width - 1)))
    y_min = int(max(0, min(y_min, height - 1)))
    x_max = int(max(0, min(x_max, width - 1)))
    y_max = int(max(0, min(y_max, height - 1)))
    if x_max < x_min:
        x_min, x_max = x_max, x_min
    if y_max < y_min:
        y_min, y_max = y_max, y_min
    return x_min, y_min, x_max, y_max


def robust_center_depth(
    depth_image: np.ndarray,
    center_x: int,
    center_y: int,
    radius: int,
    encoding: str,
    min_depth_m: float = 0.1,
    max_depth_m: float = 2.0,
    outlier_threshold_m: float = 0.02,
    min_valid_samples: int = 5,
) -> Tuple[Optional[float], int, Optional[float], Optional[float], Optional[float]]:
    """Estimate robust center depth from a small window around the bbox center.

    Algorithm:
      1. Collect up to (2*radius+1)^2 raw samples from the window.
      2. Convert each sample to metres, reject invalid (zero/NaN/Inf/out-of-range).
      3. Take the median of valid samples as the anchor.
      4. Reject outliers more than outlier_threshold_m from the median.
      5. Return the mean of remaining samples (or median if too few survive).

    Args:
        depth_image: 2D depth array (H, W).
        center_x: Column index of window center.
        center_y: Row index of window center.
        radius: Half-side of the square window in pixels (window = 2*radius+1 square).
        encoding: sensor_msgs/Image encoding, e.g. ``16UC1`` or ``32FC1``.
        min_depth_m: Minimum plausible depth in metres.
        max_depth_m: Maximum plausible depth in metres.
        outlier_threshold_m: Samples farther than this from the median are rejected.
        min_valid_samples: Minimum valid samples after outlier rejection; if fewer
            remain, return None.

    Returns:
        Tuple of (final_depth_m, valid_count, raw_center_depth_m, median_depth_m, filtered_mean_m):
        - final_depth_m: robust filtered depth in metres, or None if not enough valid samples.
        - valid_count: number of raw valid samples collected (before outlier rejection).
        - raw_center_depth_m: depth at the exact center pixel (single-point reference), in metres.
        - median_depth_m: median of all valid raw samples (before outlier rejection).
        - filtered_mean_m: mean of samples within outlier_threshold_m of the median.
    """
    if depth_image is None or depth_image.size == 0:
        return None, 0, None, None, None

    h, w = depth_image.shape[:2]
    if h < 1 or w < 1:
        return None, 0, None, None, None

    # ── Step 1: collect raw samples from window ────────────────────────────────
    x0 = max(0, center_x - radius)
    y0 = max(0, center_y - radius)
    x1 = min(w, center_x + radius + 1)
    y1 = min(h, center_y + radius + 1)

    if x0 >= x1 or y0 >= y1:
        return None, 0, None, None, None

    window = depth_image[y0:y1, x0:x1]

    # ── Convert to metres and filter invalid ───────────────────────────────────
    raw_center_m: Optional[float] = None

    def _to_meters(val) -> Optional[float]:
        if encoding in ("16UC1", "mono16"):
            if int(val) == 0:
                return None
            return float(val) * 0.001
        if encoding in ("32FC1",):
            v = float(val)
            if not math.isfinite(v) or v <= 0.0:
                return None
            return v
        # Fallback: treat as float metres if float-like, else mm uint
        if np.issubdtype(window.dtype, np.floating):
            v = float(val)
            if not math.isfinite(v) or v <= 0.0:
                return None
            return v
        else:
            if int(val) == 0:
                return None
            return float(val) * 0.001

    valid_meters: list[float] = []
    center_row = center_y - y0
    center_col = center_x - x0

    for row in range(window.shape[0]):
        for col in range(window.shape[1]):
            m = _to_meters(window[row, col])
            if m is None:
                continue
            if m < min_depth_m or m > max_depth_m:
                continue
            if row == center_row and col == center_col:
                raw_center_m = m
            valid_meters.append(m)

    valid_count = len(valid_meters)

    if valid_count < min_valid_samples:
        return None, valid_count, raw_center_m, None, None

    valid_arr = np.array(valid_meters, dtype=np.float64)

    # ── Step 3: median anchor ─────────────────────────────────────────────────
    median_m = float(np.median(valid_arr))

    # ── Step 4: reject outliers ──────────────────────────────────────────────
    deviations = np.abs(valid_arr - median_m)
    mask = deviations <= outlier_threshold_m
    filtered = valid_arr[mask]

    if len(filtered) < min_valid_samples:
        # Not enough after outlier rejection → fall back to median
        return float(median_m), valid_count, raw_center_m, median_m, float(median_m)

    filtered_mean_m = float(np.mean(filtered))

    return filtered_mean_m, valid_count, raw_center_m, median_m, filtered_mean_m


@dataclass
class BboxRoiDepthResult:
    """Result of :func:`robust_bbox_roi_depth`.

    ``surface_depth_m`` is None when there were fewer than ``min_valid_samples``
    valid pixels in the ROI — callers must treat that as "no usable depth for
    this detection" (skip), not fall back to a stale/guessed value.
    """

    surface_depth_m: Optional[float]
    valid_count: int
    total_count: int
    roi_rect: Tuple[int, int, int, int]  # (x0, y0, x1, y1), exclusive on x1/y1
    center_valid: bool


def robust_bbox_roi_depth(
    depth_image: np.ndarray,
    x_min: int,
    y_min: int,
    x_max: int,
    y_max: int,
    encoding: str,
    roi_scale: float = 0.6,
    roi_min_width_px: int = 15,
    roi_min_height_px: int = 15,
    roi_max_width_px: int = 120,
    roi_max_height_px: int = 120,
    stride: int = 1,
    min_depth_m: float = 0.1,
    max_depth_m: float = 2.0,
    outlier_threshold_m: float = 0.03,
    min_valid_samples: int = 20,
    statistic: str = "percentile",
    percentile: float = 20.0,
) -> BboxRoiDepthResult:
    """Estimate a bbox object's surface depth from a bbox-scaled ROI, not just its center pixel.

    A single hole in the depth image at the exact bbox center (common on
    white/glossy surfaces where the RealSense IR pattern is washed out) must
    not by itself make the whole detection unusable — the surrounding ROI is
    very likely to still have valid depth. Algorithm (matches codex2.md
    section 6):

      1. ROI size = bbox size * roi_scale, clamped to
         [roi_min_*_px, roi_max_*_px], centered on the bbox center, clipped
         to image bounds.
      2. Sample every ``stride``-th pixel in the ROI, convert to metres per
         ``encoding``, reject invalid (zero/NaN/Inf) and out-of-range values.
      3. If too few valid samples remain, report failure (surface_depth_m=None).
      4. Otherwise: median-anchor + reject outliers beyond
         ``outlier_threshold_m``; if outlier rejection leaves too few
         samples, keep the unfiltered valid set instead.
      5. Reduce to a single surface depth via ``statistic``: "percentile"
         (default, low percentile — object surface is closer to the camera
         than the table/background it sits on) or "median".
    """
    if depth_image is None or depth_image.size == 0:
        return BboxRoiDepthResult(None, 0, 0, (0, 0, 0, 0), False)

    h, w = depth_image.shape[:2]
    if h < 1 or w < 1:
        return BboxRoiDepthResult(None, 0, 0, (0, 0, 0, 0), False)

    x_min, x_max = (x_min, x_max) if x_min <= x_max else (x_max, x_min)
    y_min, y_max = (y_min, y_max) if y_min <= y_max else (y_max, y_min)
    bbox_w = max(1, x_max - x_min)
    bbox_h = max(1, y_max - y_min)
    center_u = (x_min + x_max) * 0.5
    center_v = (y_min + y_max) * 0.5

    roi_w = min(max(bbox_w * roi_scale, roi_min_width_px), roi_max_width_px)
    roi_h = min(max(bbox_h * roi_scale, roi_min_height_px), roi_max_height_px)

    x0 = int(round(center_u - roi_w * 0.5))
    y0 = int(round(center_v - roi_h * 0.5))
    x1 = int(round(center_u + roi_w * 0.5))
    y1 = int(round(center_v + roi_h * 0.5))
    x0, y0, x1, y1 = bbox_clip_to_image(x0, y0, x1, y1, w, h)
    x1 = max(x1, x0 + 1)
    y1 = max(y1, y0 + 1)
    roi_rect = (x0, y0, x1, y1)

    stride = max(1, int(stride))

    def _to_meters(val) -> Optional[float]:
        if encoding in ("16UC1", "mono16"):
            raw_int = int(val)
            return float(raw_int) * 0.001 if raw_int != 0 else None
        if encoding in ("32FC1",):
            raw_f = float(val)
            return raw_f if math.isfinite(raw_f) and raw_f > 0.0 else None
        if np.issubdtype(depth_image.dtype, np.floating):
            raw_f = float(val)
            return raw_f if math.isfinite(raw_f) and raw_f > 0.0 else None
        raw_int = int(val)
        return float(raw_int) * 0.001 if raw_int != 0 else None

    window = depth_image[y0:y1:stride, x0:x1:stride]
    total_count = int(window.size)

    valid_meters: list[float] = []
    for row in range(window.shape[0]):
        for col in range(window.shape[1]):
            m = _to_meters(window[row, col])
            if m is None:
                continue
            if m < min_depth_m or m > max_depth_m:
                continue
            valid_meters.append(m)

    valid_count = len(valid_meters)

    center_u_i = int(max(0, min(int(round(center_u)), w - 1)))
    center_v_i = int(max(0, min(int(round(center_v)), h - 1)))
    center_m = _to_meters(depth_image[center_v_i, center_u_i])
    center_valid = (
        center_m is not None and min_depth_m <= center_m <= max_depth_m
    )

    if valid_count < min_valid_samples:
        return BboxRoiDepthResult(None, valid_count, total_count, roi_rect, center_valid)

    valid_arr = np.array(valid_meters, dtype=np.float64)
    anchor_m = float(np.median(valid_arr))
    deviations = np.abs(valid_arr - anchor_m)
    filtered = valid_arr[deviations <= outlier_threshold_m]
    dataset = filtered if len(filtered) >= min_valid_samples else valid_arr

    if statistic == "percentile":
        surface_depth_m = float(np.percentile(dataset, percentile))
    else:
        surface_depth_m = float(np.median(dataset))

    if not math.isfinite(surface_depth_m) or surface_depth_m <= 0.0:
        return BboxRoiDepthResult(None, valid_count, total_count, roi_rect, center_valid)

    return BboxRoiDepthResult(surface_depth_m, valid_count, total_count, roi_rect, center_valid)


@dataclass
class BoxHeightGridResult:
    """Result of :func:`box_height_grid_cluster`.

    ``height_m`` is None when there weren't enough valid grid points, or no
    cluster survived selection — callers must fall back (to
    ``robust_bbox_roi_depth``-based height, then to skipping the object),
    never reuse a stale height.
    """

    valid_points: int
    total_points: int
    heights: list  # 16 raw height_i (m) or None per grid point, in row-major order
    points_uv: list  # matching (u, v) pixel for each grid point
    selected_cluster: list  # height_i values (m) in the winning cluster
    rejected_clusters: list  # other candidate clusters, each a list of height_i (m)
    height_m: Optional[float]


def _patch_median_depth_m(
    depth_image: np.ndarray,
    u: int,
    v: int,
    half_size: int,
    encoding: str,
    min_depth_m: float,
    max_depth_m: float,
    min_valid_px: int,
) -> Optional[float]:
    h, w = depth_image.shape[:2]
    x0 = max(0, u - half_size)
    y0 = max(0, v - half_size)
    x1 = min(w, u + half_size + 1)
    y1 = min(h, v + half_size + 1)
    if x0 >= x1 or y0 >= y1:
        return None
    patch = depth_image[y0:y1, x0:x1]

    def _to_m(val) -> Optional[float]:
        if encoding in ("16UC1", "mono16"):
            raw = int(val)
            return float(raw) * 0.001 if raw != 0 else None
        if encoding in ("32FC1",):
            f = float(val)
            return f if math.isfinite(f) and f > 0.0 else None
        if np.issubdtype(depth_image.dtype, np.floating):
            f = float(val)
            return f if math.isfinite(f) and f > 0.0 else None
        raw = int(val)
        return float(raw) * 0.001 if raw != 0 else None

    valid: list[float] = []
    for row in range(patch.shape[0]):
        for col in range(patch.shape[1]):
            m = _to_m(patch[row, col])
            if m is None:
                continue
            if m < min_depth_m or m > max_depth_m:
                continue
            valid.append(m)

    if len(valid) < min_valid_px:
        return None
    return float(np.median(valid))


def _extract_height_clusters(heights: list, tolerance_m: float) -> list:
    """Partition a list of heights into clusters of mutually-close values.

    Repeatedly finds the largest contiguous run (in sorted order) whose
    max-min <= tolerance_m, removes it, and repeats on the remainder — so
    e.g. a "box top" band and a separate "pad/table" band end up as two
    distinct clusters instead of being averaged together.
    """
    remaining = sorted(heights)
    clusters: list = []
    while remaining:
        n = len(remaining)
        best_start, best_end, best_len = 0, 1, 1
        for i in range(n):
            j = i
            while j + 1 < n and remaining[j + 1] - remaining[i] <= tolerance_m:
                j += 1
            length = j - i + 1
            if length > best_len:
                best_len = length
                best_start, best_end = i, j + 1
        cluster = remaining[best_start:best_end]
        clusters.append(cluster)
        remaining = remaining[:best_start] + remaining[best_end:]
    return clusters


def box_height_grid_cluster(
    depth_image: np.ndarray,
    x_min: int,
    y_min: int,
    x_max: int,
    y_max: int,
    encoding: str,
    camera_table_depth_m: float,
    grid_rows: int = 4,
    grid_cols: int = 4,
    inner_scale: float = 0.70,
    patch_half_size_px: int = 3,
    patch_min_valid_px: int = 5,
    grid_min_valid_points: int = 6,
    min_depth_m: float = 0.1,
    max_depth_m: float = 2.0,
    box_height_min_m: float = 0.005,
    box_height_max_m: float = 0.5,
    cluster_tolerance_m: float = 0.02,
    cluster_min_points: int = 4,
    cluster_statistic: str = "mean",
    prefer_higher_cluster: bool = True,
    low_cluster_reject_enable: bool = True,
    low_cluster_threshold_m: float = 0.07,
) -> BoxHeightGridResult:
    """Estimate a box's height from a 4x4 (default) grid of sample points
    inside the bbox, clustering the resulting height_i values instead of
    averaging all of them.

    Rationale: a single ROI-wide statistic (see robust_bbox_roi_depth) can
    still land on a pad/support-plane height if enough bbox-interior pixels
    belong to it (e.g. a corner of the box floats over the pad edge, or the
    box is translucent/reflective at some patches). Sampling a structured
    grid and explicitly picking the *box-top* cluster — preferring more
    points, then higher height, and hard-rejecting a low (~pad-height)
    cluster when a higher valid one exists — makes the result robust to a
    minority of points landing on the wrong surface.
    """
    if depth_image is None or depth_image.size == 0:
        total = grid_rows * grid_cols
        return BoxHeightGridResult(0, total, [None] * total, [], [], [], None)

    h, w = depth_image.shape[:2]
    x_min, x_max = (x_min, x_max) if x_min <= x_max else (x_max, x_min)
    y_min, y_max = (y_min, y_max) if y_min <= y_max else (y_max, y_min)
    bbox_w = max(1, x_max - x_min)
    bbox_h = max(1, y_max - y_min)
    center_u = (x_min + x_max) * 0.5
    center_v = (y_min + y_max) * 0.5
    inner_w = bbox_w * inner_scale
    inner_h = bbox_h * inner_scale
    inner_x0 = center_u - inner_w * 0.5
    inner_y0 = center_v - inner_h * 0.5

    points_uv: list = []
    heights: list = []
    for row in range(grid_rows):
        fy = (row + 0.5) / grid_rows
        v = int(round(inner_y0 + fy * inner_h))
        v = max(0, min(v, h - 1))
        for col in range(grid_cols):
            fx = (col + 0.5) / grid_cols
            u = int(round(inner_x0 + fx * inner_w))
            u = max(0, min(u, w - 1))
            points_uv.append((u, v))

            depth_i = _patch_median_depth_m(
                depth_image, u, v, patch_half_size_px, encoding,
                min_depth_m, max_depth_m, patch_min_valid_px,
            )
            if depth_i is None:
                heights.append(None)
                continue
            height_i = camera_table_depth_m - depth_i
            if (
                not math.isfinite(height_i)
                or height_i < box_height_min_m
                or height_i > box_height_max_m
            ):
                heights.append(None)
                continue
            heights.append(height_i)

    total_points = grid_rows * grid_cols
    valid_heights = [hh for hh in heights if hh is not None]
    valid_points = len(valid_heights)

    if valid_points < grid_min_valid_points:
        return BoxHeightGridResult(valid_points, total_points, heights, points_uv, [], [], None)

    clusters = _extract_height_clusters(valid_heights, cluster_tolerance_m)
    viable = [c for c in clusters if len(c) >= cluster_min_points]

    if not viable:
        return BoxHeightGridResult(valid_points, total_points, heights, points_uv, [], clusters, None)

    # codex2.md addendum: never silently pick a low pad/support-plane cluster
    # when a higher valid cluster is also available — even if the low
    # cluster has more points.
    selectable = list(viable)
    if low_cluster_reject_enable:
        has_higher = any((sum(c) / len(c)) >= low_cluster_threshold_m for c in viable)
        if has_higher:
            filtered = [c for c in selectable if (sum(c) / len(c)) >= low_cluster_threshold_m]
            if filtered:
                selectable = filtered

    max_count = max(len(c) for c in selectable)
    if prefer_higher_cluster:
        tied = [c for c in selectable if len(c) >= max_count - 1]
        winner = max(tied, key=lambda c: sum(c) / len(c))
    else:
        tied = [c for c in selectable if len(c) == max_count]
        if len(tied) == 1:
            winner = tied[0]
        else:
            def _std(c):
                return float(np.std(np.array(c, dtype=np.float64)))
            min_std = min(_std(c) for c in tied)
            tied2 = [c for c in tied if abs(_std(c) - min_std) < 1e-9]
            winner = max(tied2, key=lambda c: float(np.median(c)))

    rejected = [c for c in clusters if c is not winner]

    if cluster_statistic == "median":
        height_m = float(np.median(winner))
    else:
        height_m = float(np.mean(winner))

    return BoxHeightGridResult(
        valid_points, total_points, heights, points_uv, winner, rejected, height_m
    )
