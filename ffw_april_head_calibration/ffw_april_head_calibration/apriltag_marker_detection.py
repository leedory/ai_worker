# Copyright 2025 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""AprilTag detection: image → grayscale → optional resize → OpenCV aruco."""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np

from ffw_april_head_calibration.subprocess_apriltag import (
    SubprocessAprilTagDetector,
)

# OpenCV worker defaults (tuning is done in apriltag_detect_worker if needed).
APRILTAG_QUAD_DECIMATE = 1.0
APRILTAG_SUBPROCESS_TIMEOUT_SEC = 45.0


def bgr_and_gray(
    cv_image: np.ndarray, encoding: str,
) -> tuple[np.ndarray, np.ndarray]:
    """ROS/cv_bridge image → BGR (draw) and single-channel gray."""
    enc = encoding.lower()
    if enc == 'rgb8':
        bgr = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
        gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
    elif enc == 'rgba8':
        bgr = cv2.cvtColor(cv_image, cv2.COLOR_RGBA2BGR)
        gray = cv2.cvtColor(cv_image, cv2.COLOR_RGBA2GRAY)
    elif enc in ('bgra8', 'bgr8'):
        if enc == 'bgra8':
            bgr = cv2.cvtColor(cv_image, cv2.COLOR_BGRA2BGR)
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGRA2GRAY)
        else:
            bgr = np.ascontiguousarray(cv_image)
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    else:
        bgr = np.ascontiguousarray(cv_image)
        if bgr.ndim == 2:
            gray = bgr
            bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        else:
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    return bgr, gray


def _as_uint8_gray(gray: np.ndarray) -> np.ndarray | None:
    if gray is None or gray.ndim != 2:
        return None
    h, w = int(gray.shape[0]), int(gray.shape[1])
    if h < 8 or w < 8:
        return None
    if np.issubdtype(gray.dtype, np.floating):
        gray = np.clip(gray, 0.0, 255.0)
    return np.array(gray, dtype=np.uint8, order='C', copy=True)


def downscale_for_detection(
    gray: np.ndarray, max_side: int,
) -> tuple[np.ndarray | None, float, float]:
    """
    If max(h,w) > max_side, resize gray for detection.

    Returns (small_gray, sx, sy) to map corners to full resolution
    (x *= sx, y *= sy). max_side <= 0 means no resize.
    """
    g = _as_uint8_gray(gray)
    if g is None:
        return None, 1.0, 1.0
    h, w = int(g.shape[0]), int(g.shape[1])
    if max_side <= 0 or max(h, w) <= max_side:
        return g, 1.0, 1.0
    s = max_side / float(max(h, w))
    nw = max(8, int(round(w * s)))
    nh = max(8, int(round(h * s)))
    small = cv2.resize(g, (nw, nh), interpolation=cv2.INTER_AREA)
    return small, w / float(nw), h / float(nh)


def scale_corners_to_full(corners: list, sx: float, sy: float) -> list:
    if sx == 1.0 and sy == 1.0:
        return corners
    out = []
    for c in corners:
        a = np.asarray(c, dtype=np.float32).reshape(1, 4, 2).copy()
        a[..., 0] *= sx
        a[..., 1] *= sy
        out.append(a)
    return out


def marker_center(marker_corners) -> tuple[float, float]:
    pts = np.asarray(marker_corners, dtype=np.float64).reshape(-1, 2)
    c = pts.mean(axis=0)
    return float(c[0]), float(c[1])


def marker_area(marker_corners) -> float:
    pts = np.asarray(marker_corners, dtype=np.float64).reshape(-1, 2)
    return float(cv2.contourArea(pts.astype(np.float32)))


def draw_markers_overlay(bgr: np.ndarray, corners: list, ids) -> None:
    if not corners:
        return
    ids_flat = ids.flatten().tolist() if ids is not None else None
    for i, c in enumerate(corners):
        pts = np.asarray(c, dtype=np.float32).reshape(-1, 2).astype(np.int32)
        cv2.polylines(bgr, [pts], True, (0, 255, 0), 2)
        cx = float(pts[:, 0].mean())
        cy = float(pts[:, 1].mean())
        icx, icy = int(round(cx)), int(round(cy))
        r = 4
        cv2.circle(bgr, (icx, icy), r + 1, (0, 0, 0), 1)
        cv2.circle(bgr, (icx, icy), r, (0, 255, 255), -1)
        if ids_flat is not None and i < len(ids_flat):
            tid = int(ids_flat[i])
            cv2.putText(
                bgr, str(tid), (icx + r + 4, icy - r - 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 100, 0), 2)


class MarkerDetector:
    """
    Runs AprilTag on grayscale via an isolated subprocess (OpenCV aruco),
    optional downscale, corners mapped back to full resolution.
    """

    def __init__(
        self,
        *,
        logger,
        family: str,
        max_image_side: int,
        worker_script: Path,
    ) -> None:
        self._log = logger
        self._family = str(family)
        self._quad_decimate = float(APRILTAG_QUAD_DECIMATE)
        self._max_side = int(max_image_side)
        self._timeout = float(APRILTAG_SUBPROCESS_TIMEOUT_SEC)
        wp = Path(worker_script)
        if not wp.is_file():
            raise ValueError(f'AprilTag worker script not found: {wp}')
        self._sub = SubprocessAprilTagDetector(wp, logger)

    def close(self) -> None:
        self._sub.close()

    def detect(
        self, gray_full: np.ndarray,
    ) -> tuple[list | None, np.ndarray | None]:
        """
        Corners and ids in full image pixels, or (None, None) on failure.

        Empty detection is ([], None).
        """
        small, sx, sy = downscale_for_detection(gray_full, self._max_side)
        if small is None:
            return None, None

        corners, ids = self._sub.detect(
            small,
            self._family,
            self._quad_decimate,
            timeout_sec=self._timeout,
        )

        if corners is None:
            return None, None
        if len(corners) > 0 and (sx != 1.0 or sy != 1.0):
            corners = scale_corners_to_full(corners, sx, sy)
        return corners, ids
