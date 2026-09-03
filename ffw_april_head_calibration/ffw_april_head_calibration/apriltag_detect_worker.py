#!/usr/bin/env python3
#
# Copyright 2025 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
#
# Standalone stdin/stdout worker: numpy + OpenCV aruco (no cv_bridge / rclpy).
# Isolates marker detection from the main ROS process.

from __future__ import annotations

import json
import struct
import sys

import numpy as np

try:
    import cv2  # noqa: F401
except ImportError as e:
    cv2 = None  # type: ignore[misc, assignment]
    _IMPORT_ERR = str(e)
else:
    _IMPORT_ERR = ''

from opencv_apriltag_detect import detect_markers_opencv, family_to_dict_id


def _read_exact(stream, n: int) -> bytes | None:
    parts: list[bytes] = []
    remaining = n
    while remaining > 0:
        chunk = stream.read(remaining)
        if not chunk:
            return None
        parts.append(chunk)
        remaining -= len(chunk)
    return b''.join(parts)


def _detect(family: str, quad_decimate: float, gray: np.ndarray) -> dict:
    gray = np.array(gray, dtype=np.uint8, order='C', copy=True)
    corners, ids = detect_markers_opencv(gray, family, quad_decimate)
    if not corners:
        return {'ok': True, 'ids': [], 'corners': []}
    ids_flat = [int(ids[i, 0]) for i in range(len(corners))]
    corners_out = []
    for c in corners:
        arr = np.asarray(c, dtype=np.float64).reshape(4, 2)
        corners_out.append(arr.tolist())
    return {'ok': True, 'ids': ids_flat, 'corners': corners_out}


def main() -> None:
    line = sys.stdin.buffer.readline()
    if not line:
        return
    try:
        cfg = json.loads(line.decode('utf-8'))
        family = str(cfg.get('family', 'tag36h11'))
        quad_decimate = float(cfg.get('quad_decimate', 1.0))
    except (json.JSONDecodeError, TypeError, ValueError) as e:
        sys.stdout.write(json.dumps({'ok': False, 'msg': str(e)}) + '\n')
        sys.stdout.flush()
        return

    if cv2 is None:
        sys.stdout.write(
            json.dumps(
                {
                    'ok': False,
                    'msg': f'OpenCV import failed: {_IMPORT_ERR}',
                },
            )
            + '\n',
        )
        sys.stdout.flush()
        return

    try:
        family_to_dict_id(family)
    except ValueError as e:
        sys.stdout.write(
            json.dumps({'ok': False, 'msg': str(e)}) + '\n')
        sys.stdout.flush()
        return

    sys.stdout.write(
        json.dumps({'ok': True, 'status': 'ready'}) + '\n')
    sys.stdout.flush()

    while True:
        hdr = _read_exact(sys.stdin.buffer, 8)
        if hdr is None or len(hdr) < 8:
            break
        h, w = struct.unpack('!II', hdr)
        n = int(h) * int(w)
        if n <= 0 or n > 60_000_000:
            sys.stdout.write(
                json.dumps({'ok': False, 'msg': 'bad dimensions'}) + '\n')
            sys.stdout.flush()
            continue
        buf = _read_exact(sys.stdin.buffer, n)
        if buf is None or len(buf) != n:
            break

        gray = np.frombuffer(buf, dtype=np.uint8, count=n).reshape(h, w)
        gray = np.array(gray, dtype=np.uint8, copy=True)
        try:
            out = _detect(family, quad_decimate, gray)
        except Exception as e:
            out = {'ok': False, 'msg': repr(e)}
        sys.stdout.write(json.dumps(out) + '\n')
        sys.stdout.flush()


if __name__ == '__main__':
    main()
