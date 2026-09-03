# Copyright 2025 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""AprilTag families via OpenCV aruco."""

from __future__ import annotations

import cv2
import numpy as np

_FAMILY_TO_DICT: dict[str, int] = {
    'tag36h11': cv2.aruco.DICT_APRILTAG_36h11,
    'tag36h10': cv2.aruco.DICT_APRILTAG_36h10,
    'tag25h9': cv2.aruco.DICT_APRILTAG_25h9,
    'tag16h5': cv2.aruco.DICT_APRILTAG_16h5,
}


def family_to_dict_id(family: str) -> int:
    """Map family name (e.g. tag36h11) to a cv2.aruco predefined dictionary."""
    key = str(family).strip().lower()
    if key not in _FAMILY_TO_DICT:
        supported = ', '.join(sorted(_FAMILY_TO_DICT))
        raise ValueError(
            f'Unsupported apriltag_family {family!r}; '
            f'OpenCV backend supports: {supported}')
    return _FAMILY_TO_DICT[key]


_APRILTAG_DICT_IDS: frozenset[int] = frozenset(
    int(getattr(cv2.aruco, n)) for n in (
        'DICT_APRILTAG_36h11', 'DICT_APRILTAG_36h10',
        'DICT_APRILTAG_25h9', 'DICT_APRILTAG_16h5',
    ) if hasattr(cv2.aruco, n))


def _apriltag_dict(dict_id: int) -> bool:
    return int(dict_id) in _APRILTAG_DICT_IDS


def _detector_params_for_dict(dict_id: int) -> object:
    params = cv2.aruco.DetectorParameters_create()
    # AprilTag: OpenCV docs often suggest borderBits=2, but on OpenCV 4.6
    # (Jetson/apt) detectMarkers returns no markers for synthetic and printed
    # tag36h11 with borderBits=2; borderBits=1 matches drawMarker and works.
    params.markerBorderBits = 1 if _apriltag_dict(dict_id) else 2
    if _apriltag_dict(dict_id) and hasattr(
            cv2.aruco, 'CORNER_REFINE_APRILTAG'):
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
    return params


_DET_CACHE: dict[int, tuple[object, object]] = {}


def _dictionary_and_params(dict_id: int) -> tuple[object, object]:
    if dict_id not in _DET_CACHE:
        dictionary = cv2.aruco.getPredefinedDictionary(int(dict_id))
        _DET_CACHE[dict_id] = (dictionary, _detector_params_for_dict(dict_id))
    return _DET_CACHE[dict_id]


def _quad_decimate_gray(
    gray_uint8: np.ndarray, quad_decimate: float,
) -> tuple[np.ndarray, float, float]:
    """Approximate quad_decimate>1 by shrinking the image before detect."""
    g = gray_uint8
    h0, w0 = int(g.shape[0]), int(g.shape[1])
    q = float(quad_decimate)
    if q <= 1.001:
        return g, 1.0, 1.0
    inv = 1.0 / q
    nw = max(8, int(round(w0 * inv)))
    nh = max(8, int(round(h0 * inv)))
    small = cv2.resize(g, (nw, nh), interpolation=cv2.INTER_AREA)
    return small, w0 / float(nw), h0 / float(nh)


def detect_markers_opencv(
    gray_uint8: np.ndarray,
    family: str,
    quad_decimate: float,
) -> tuple[list, np.ndarray | None]:
    """
    Detect AprilTag markers using OpenCV aruco.

    Returns corners as a list of (1,4,2) float32 (full-resolution coords),
    and ids shaped (N,1) int32, or ([], None) when empty.
    """
    if gray_uint8 is None or gray_uint8.ndim != 2:
        return [], None
    h, w = int(gray_uint8.shape[0]), int(gray_uint8.shape[1])
    if h < 8 or w < 8:
        return [], None

    g = np.asarray(gray_uint8, dtype=np.uint8, order='C')
    if g.dtype != np.uint8:
        g = np.clip(g, 0, 255).astype(np.uint8)

    dict_id = family_to_dict_id(family)
    work, qsx, qsy = _quad_decimate_gray(g, quad_decimate)
    dictionary, params = _dictionary_and_params(dict_id)

    if hasattr(cv2.aruco, 'ArucoDetector'):
        try:
            detector = cv2.aruco.ArucoDetector(dictionary, params)
            corners_raw, ids, _rej = detector.detectMarkers(work)
        except (TypeError, AttributeError):
            corners_raw, ids, _rej = cv2.aruco.detectMarkers(
                work, dictionary, parameters=params)
    else:
        corners_raw, ids, _rej = cv2.aruco.detectMarkers(
            work, dictionary, parameters=params)

    if not corners_raw or ids is None:
        return [], None

    corners: list = []
    for c in corners_raw:
        a = np.asarray(c, dtype=np.float32).reshape(-1, 2)
        if a.shape[0] != 4:
            continue
        a = a.reshape(1, 4, 2)
        if qsx != 1.0 or qsy != 1.0:
            a = a.copy()
            a[..., 0] *= qsx
            a[..., 1] *= qsy
        corners.append(a)

    if not corners:
        return [], None

    ids_arr = np.asarray(ids, dtype=np.int32).reshape(-1, 1)
    n = min(len(corners), int(ids_arr.shape[0]))
    if n == 0:
        return [], None
    return corners[:n], ids_arr[:n].copy()
