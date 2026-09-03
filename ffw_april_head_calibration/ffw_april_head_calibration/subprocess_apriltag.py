# Copyright 2025 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Run OpenCV marker detection in a subprocess (isolate native crashes)."""

from __future__ import annotations

import json
import select
import struct
import subprocess
import sys
import threading
from pathlib import Path

import numpy as np


class SubprocessAprilTagDetector:
    """One-line-JSON protocol; see apriltag_detect_worker.py."""

    def __init__(self, worker_script: Path, logger):
        self._worker_script = Path(worker_script)
        self._log = logger
        self._proc: subprocess.Popen | None = None
        self._config_key: tuple[str, float] | None = None
        self._io_lock = threading.Lock()

    def close(self) -> None:
        if self._proc is not None:
            try:
                if self._proc.stdin:
                    self._proc.stdin.close()
            except (BrokenPipeError, OSError):
                pass
            self._proc.terminate()
            try:
                self._proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self._proc.kill()
            self._proc = None
        self._config_key = None

    def _spawn(self) -> bool:
        self.close()
        if not self._worker_script.is_file():
            self._log.error(
                f'AprilTag worker script not found: {self._worker_script}')
            return False
        try:
            self._proc = subprocess.Popen(
                [sys.executable, '-u', str(self._worker_script)],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                bufsize=0,
            )
        except OSError as e:
            self._log.error(f'Failed to spawn AprilTag worker: {e}')
            self._proc = None
            return False
        self._config_key = None
        return True

    def _read_line(self, timeout_sec: float) -> bytes | None:
        if self._proc is None or self._proc.stdout is None:
            return None
        r, _, _ = select.select([self._proc.stdout], [], [], timeout_sec)
        if not r:
            return None
        line = self._proc.stdout.readline()
        return line if line else None

    def _configure(
        self, family: str, quad_decimate: float,
    ) -> bool:
        assert self._proc is not None and self._proc.stdin is not None
        try:
            payload = {
                'family': family,
                'quad_decimate': float(quad_decimate),
            }
            line = (json.dumps(payload) + '\n').encode('utf-8')
            self._proc.stdin.write(line)
            self._proc.stdin.flush()
        except (BrokenPipeError, OSError) as e:
            self._log.warn(f'AprilTag worker config write failed: {e}')
            return False
        raw = self._read_line(30.0)
        if not raw:
            self._log.error(
                'AprilTag worker: no ready handshake (crashed at startup, '
                'blocked on import, or stdout not connected). '
                'Ensure python3-opencv is installed; for gray topics use '
                'image_transport_encoding:=auto on the tracker node.'
            )
            self.close()
            return False
        try:
            ack = json.loads(raw.decode('utf-8'))
        except json.JSONDecodeError:
            self._log.error(
                f'AprilTag worker bad handshake: {raw[:160]!r}')
            self.close()
            return False
        if not ack.get('ok') or ack.get('status') != 'ready':
            self._log.error(f'AprilTag worker refused config: {ack}')
            self.close()
            return False
        self._config_key = (family, float(quad_decimate))
        return True

    def _ensure_ready(
        self, family: str, quad_decimate: float,
    ) -> bool:
        if self._proc is None or self._proc.poll() is not None:
            if not self._spawn():
                return False
        key = (family, float(quad_decimate))
        if self._config_key != key:
            return self._configure(family, quad_decimate)
        return True

    def detect(
        self,
        gray: np.ndarray,
        family: str,
        quad_decimate: float,
        *,
        timeout_sec: float = 45.0,
    ) -> tuple[list | None, np.ndarray | None]:
        """
        Return (corners, ids) in OpenCV shape: corners list of (1,4,2) float32.

        On failure returns (None, None).
        """
        if gray is None or gray.ndim != 2:
            return None, None
        h, w = int(gray.shape[0]), int(gray.shape[1])
        if h < 8 or w < 8:
            return None, None
        g = np.array(gray, dtype=np.uint8, order='C', copy=True)

        with self._io_lock:
            for _ in range(2):
                if not self._ensure_ready(family, quad_decimate):
                    return None, None
                assert self._proc is not None and self._proc.stdin is not None
                try:
                    self._proc.stdin.write(struct.pack('!II', h, w))
                    self._proc.stdin.flush()
                    self._proc.stdin.write(g.tobytes())
                    self._proc.stdin.flush()
                except (BrokenPipeError, OSError):
                    self._log.warn('AprilTag worker pipe broken; will respawn')
                    self.close()
                    continue

                raw = self._read_line(timeout_sec)
                if raw is None:
                    if self._proc.poll() is not None:
                        self._log.warn(
                            'AprilTag worker exited '
                            '(possible crash in detector)'
                        )
                    else:
                        self._log.warn('AprilTag worker timed out')
                    self.close()
                    continue

                try:
                    data = json.loads(raw.decode('utf-8'))
                except json.JSONDecodeError:
                    self._log.warn(
                        f'AprilTag worker invalid JSON: {raw[:120]!r}')
                    self.close()
                    continue

                if not data.get('ok', False):
                    msg = data.get('msg', '')
                    if msg:
                        self._log.debug(f'AprilTag worker: {msg}')
                    return None, None

                return self._deserialize(data)

            return None, None

    @staticmethod
    def _deserialize(data: dict) -> tuple[list, np.ndarray | None]:
        ids_list = list(data.get('ids') or [])
        corners_list = list(data.get('corners') or [])
        n = min(len(ids_list), len(corners_list))
        if n == 0:
            return [], None
        ids_list = ids_list[:n]
        corners_list = corners_list[:n]
        corners = []
        for quad in corners_list:
            arr = np.array(quad, dtype=np.float32).reshape(1, 4, 2)
            corners.append(arr)
        ids = np.array(ids_list, dtype=np.int32).reshape(-1, 1)
        return corners, ids
