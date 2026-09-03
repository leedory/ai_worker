#!/usr/bin/env python3
#
# Copyright 2025 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Track an AprilTag in a camera image and command head joints to center it."""

from __future__ import annotations

import math
import pathlib
import time

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray

import ffw_april_head_calibration
from ffw_april_head_calibration import apply_head_homing_offset_from_delta as _apply_hom
from ffw_april_head_calibration.apriltag_marker_detection import (
    MarkerDetector,
    bgr_and_gray,
    draw_markers_overlay,
    marker_area,
    marker_center,
)

# FFW head naming and soft limits (matches ffw_description / typical URDF).
HEAD_JOINT1 = 'head_joint1'
HEAD_JOINT2 = 'head_joint2'
_HEAD_LIMIT_RAD = 1.4
HEAD_JOINT1_MIN = -_HEAD_LIMIT_RAD
HEAD_JOINT1_MAX = _HEAD_LIMIT_RAD
HEAD_JOINT2_MIN = -_HEAD_LIMIT_RAD
HEAD_JOINT2_MAX = _HEAD_LIMIT_RAD

HEAD_JOINT_LOG_INTERVAL_SEC = 1.0

# Centering loop: extra gain when error is small (normalized coords).
NEAR_CENTER_GAIN_BOOST = 0.2
NEAR_CENTER_BOOST_RADIUS = 0.12

# Lost-tag pitch sweep (only when enable_lost_tag_search is true).
LOST_TAG_SEARCH_PITCH_MIN = 0.0
LOST_TAG_SEARCH_PITCH_MAX = 0.65
LOST_TAG_SEARCH_PITCH_STEP = 0.015

DEBUG_IMAGE_TOPIC = 'debug_image'
RVIZ_MARKERS_TOPIC = 'apriltag_rviz_markers'
RVIZ_PIXEL_TO_METER = 0.001
RVIZ_PLANE_Z = 0.5
RVIZ_MARKER_LINE_WIDTH = 0.012
RVIZ_MARKER_LIFETIME_SEC = 0.5

SAVE_DELTA_SRV_NAME = 'save_delta'


def _rad_to_pulse(rad: float, resolution: int) -> int:
    return int(round((rad / (2.0 * math.pi)) * resolution))


def _yaml_map_key(name: str) -> str:
    """Quote map keys if they are not simple unquoted YAML identifiers."""
    s = str(name)
    if s and all(c.isalnum() or c == '_' for c in s) and not s[0].isdigit():
        return s
    escaped = s.replace('\\', '\\\\').replace('"', '\\"')
    return f'"{escaped}"'


def _format_head_pulse_delta_yaml(
    *,
    saved_at_unix_sec: float,
    pulse_resolution: int,
    joint1_name: str,
    joint2_name: str,
    ref1: int,
    ref2: int,
    cur1: int,
    cur2: int,
    d1: int,
    d2: int,
) -> str:
    k1 = _yaml_map_key(joint1_name)
    k2 = _yaml_map_key(joint2_name)
    lines = [
        '# Written by ffw_april_head_calibration save_delta (std_srvs/Trigger).',
        f'saved_at_unix_sec: {saved_at_unix_sec:.6f}',
        f'pulse_resolution: {pulse_resolution}',
        'reference_pulse:',
        f'  {k1}: {ref1}',
        f'  {k2}: {ref2}',
        'current_pulse:',
        f'  {k1}: {cur1}',
        f'  {k2}: {cur2}',
        'delta_pulse:',
        f'  {k1}: {d1}',
        f'  {k2}: {d2}',
    ]
    return '\n'.join(lines) + '\n'


def _param_bool(val: object) -> bool:
    if isinstance(val, str):
        return val.lower() in ('true', '1', 'yes', 'on')
    return bool(val)


def _pixel_to_rviz_point(
    u: float, v: float, w: int, h: int, scale: float, z: float,
) -> Point:
    """Map image pixel (u right, v down) to a coarse camera-plane point for RViz."""
    cx = w * 0.5
    cy = h * 0.5
    p = Point()
    p.x = float((u - cx) * scale)
    p.y = float((v - cy) * scale)
    p.z = float(z)
    return p


class AprilHeadTrackerNode(Node):
    def __init__(self):
        super().__init__('april_head_calibration')

        self.declare_parameter(
            'image_topic',
            '/zed/zed_node/rgb_raw/image_raw_color',
        )
        self.declare_parameter(
            'joint_trajectory_topic',
            '/leader/joystick_controller_left/joint_trajectory',
        )
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('head_joint_pulse_resolution', 2048)
        self.declare_parameter(
            'head_joint1_pulse_reference',
            0,
        )
        self.declare_parameter(
            'head_joint2_pulse_reference',
            0,
        )

        self.declare_parameter(
            'apriltag_family',
            'tag36h11',
        )
        self.declare_parameter('target_marker_id', -1)

        self.declare_parameter('yaw_gain', 0.65)
        self.declare_parameter('pitch_gain', 0.65)
        self.declare_parameter('yaw_sign', -1.0)
        self.declare_parameter('pitch_sign', 1.0)
        self.declare_parameter('max_step_yaw', 0.1)
        self.declare_parameter('max_step_pitch', 0.1)
        self.declare_parameter('pixel_deadband', 1.0)

        self.declare_parameter('image_transport_encoding', 'auto')
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('apriltag_detection_max_side', 640)

        self.declare_parameter('enable_lost_tag_search', False)
        self.declare_parameter(
            'lost_tag_miss_grace_frames',
            15,
        )

        self.declare_parameter('publish_rviz_markers', True)

        self.declare_parameter(
            'save_delta_yaml_path',
            '',
        )
        self.declare_parameter('ros2_control_xacro_path', '')
        self.declare_parameter('save_delta_apply_ros2_control', True)

        self._image_topic = self.get_parameter('image_topic').value
        self._traj_topic = self.get_parameter('joint_trajectory_topic').value
        self._js_topic = self.get_parameter('joint_states_topic').value
        self._j1 = HEAD_JOINT1
        self._j2 = HEAD_JOINT2
        self._head_joint_log_dt = HEAD_JOINT_LOG_INTERVAL_SEC
        self._head_pulse_res = max(
            1, int(self.get_parameter('head_joint_pulse_resolution').value))
        self._pulse_ref_j1 = int(
            round(float(self.get_parameter(
                'head_joint1_pulse_reference').value)))
        self._pulse_ref_j2 = int(
            round(float(self.get_parameter(
                'head_joint2_pulse_reference').value)))
        self._apriltag_family = self.get_parameter('apriltag_family').value
        self._target_id = int(self.get_parameter('target_marker_id').value)

        self._yaw_gain = float(self.get_parameter('yaw_gain').value)
        self._pitch_gain = float(self.get_parameter('pitch_gain').value)
        self._yaw_sign = float(self.get_parameter('yaw_sign').value)
        self._pitch_sign = float(self.get_parameter('pitch_sign').value)
        self._max_sy = float(self.get_parameter('max_step_yaw').value)
        self._max_sp = float(self.get_parameter('max_step_pitch').value)
        self._deadband = float(self.get_parameter('pixel_deadband').value)
        self._near_boost = NEAR_CENTER_GAIN_BOOST
        self._near_boost_r = max(1e-6, NEAR_CENTER_BOOST_RADIUS)

        self._j1_min = HEAD_JOINT1_MIN
        self._j1_max = HEAD_JOINT1_MAX
        self._j2_min = HEAD_JOINT2_MIN
        self._j2_max = HEAD_JOINT2_MAX

        self._fallback_pitch = 0.0
        self._fallback_yaw = 0.0

        self._encoding_param = self.get_parameter(
            'image_transport_encoding').value
        self._debug_image_enabled = _param_bool(
            self.get_parameter('publish_debug_image').value)
        self._debug_topic = DEBUG_IMAGE_TOPIC

        self._apriltag_max_side = int(
            self.get_parameter('apriltag_detection_max_side').value)

        self._lost_search_enabled = _param_bool(
            self.get_parameter('enable_lost_tag_search').value)
        self._lost_search_pmin = LOST_TAG_SEARCH_PITCH_MIN
        self._lost_search_pmax = LOST_TAG_SEARCH_PITCH_MAX
        self._lost_search_step = max(1e-6, LOST_TAG_SEARCH_PITCH_STEP)
        self._miss_grace_frames = max(
            0, int(self.get_parameter('lost_tag_miss_grace_frames').value))

        self._rviz_markers_enabled = _param_bool(
            self.get_parameter('publish_rviz_markers').value)
        self._rviz_topic = RVIZ_MARKERS_TOPIC
        self._rviz_px_scale = RVIZ_PIXEL_TO_METER
        self._rviz_plane_z = RVIZ_PLANE_Z
        self._rviz_line_w = RVIZ_MARKER_LINE_WIDTH
        self._rviz_life = RVIZ_MARKER_LIFETIME_SEC

        _worker = pathlib.Path(ffw_april_head_calibration.__file__).parent / (
            'apriltag_detect_worker.py')
        self._marker_detector = MarkerDetector(
            logger=self.get_logger(),
            family=self._apriltag_family,
            max_image_side=self._apriltag_max_side,
            worker_script=_worker,
        )

        self._bridge = CvBridge()
        self._head_pitch: float | None = None
        self._head_yaw: float | None = None
        self._cmd_pitch: float | None = None
        self._cmd_yaw: float | None = None
        self._warned_openloop: bool = False
        self._head_joint_log_last_t: float = 0.0
        self._lost_search_pitch: float | None = None
        self._lost_search_fixed_yaw: float | None = None
        self._lost_search_dir: float = 1.0
        self._miss_streak: int = 0
        self._last_good_track_pitch: float | None = None
        self._last_good_track_yaw: float | None = None

        self._joint_names = [self._j1, self._j2]
        self._traj_pub = self.create_publisher(JointTrajectory, self._traj_topic, 10)
        self._debug_pub = None
        if self._debug_image_enabled:
            self._debug_pub = self.create_publisher(Image, self._debug_topic, 1)
        self._rviz_pub = None
        if self._rviz_markers_enabled:
            self._rviz_pub = self.create_publisher(
                MarkerArray, self._rviz_topic, 10)
        self.create_subscription(Image, self._image_topic, self._on_image, 1)

        self.create_subscription(
            JointState, self._js_topic, self._on_joint_state, 10)

        self._save_delta_srv = self.create_service(
            Trigger,
            SAVE_DELTA_SRV_NAME,
            self._on_save_delta,
        )

        self.get_logger().info(
            f'AprilTag head calibration: image={self._image_topic} '
            f'trajectory={self._traj_topic} family={self._apriltag_family!r}'
        )
        self.get_logger().info(
            f'  cv_bridge image_transport_encoding={self._encoding_param!r} '
            f'(use auto for mono8/gray topics); AprilTag in subprocess worker'
        )
        if self._debug_image_enabled and self._debug_pub is not None:
            self.get_logger().info(
                'RViz2: Add → Image → set Image Topic to '
                f'{self._debug_pub.topic_name} (bgr8)'
            )
        if self._rviz_markers_enabled and self._rviz_pub is not None:
            self.get_logger().info(
                'RViz2: Add → MarkerArray → topic '
                f'{self._rviz_pub.topic_name} (Fixed Frame = parent of camera '
                f'frame in image header; markers use coarse pixel→meter layout)'
            )
        self.get_logger().info(
            f'save_delta: std_srvs/Trigger service '
            f'{self._save_delta_srv.service_name}'
        )
        if self._lost_search_enabled:
            lo = min(self._lost_search_pmin, self._lost_search_pmax)
            hi = max(self._lost_search_pmin, self._lost_search_pmax)
            self.get_logger().info(
                f'lost_tag_search: {self._j1} sweep {lo:.3f}..{hi:.3f} rad '
                f'(step={self._lost_search_step:.4f} / frame), '
                f'{self._j2} yaw held; disable with enable_lost_tag_search:=false'
            )
        self.get_logger().info(
            f'track_loss: after a good track, hold last command for '
            f'{self._miss_grace_frames} consecutive miss frame(s) before '
            f'pitch sweep (lost_tag_miss_grace_frames; 0 = immediate sweep)'
        )

    def _save_delta_yaml_path_resolved(self) -> pathlib.Path:
        raw = self.get_parameter('save_delta_yaml_path').value
        path_str = str(raw or '').strip()
        if not path_str:
            return pathlib.Path.home() / '.ros' / 'ffw_head_joint_pulse_delta.yaml'
        return pathlib.Path(path_str).expanduser()

    def _on_save_delta(self, _request, response):
        if self._head_pitch is None or self._head_yaw is None:
            response.success = False
            response.message = (
                f'No joint positions for {self._j1}/{self._j2} on '
                f'{self._js_topic!r} yet; cannot compute pulse delta.'
            )
            return response

        res = max(1, int(self.get_parameter('head_joint_pulse_resolution').value))
        ref1 = int(round(float(
            self.get_parameter('head_joint1_pulse_reference').value)))
        ref2 = int(round(float(
            self.get_parameter('head_joint2_pulse_reference').value)))

        cur1 = _rad_to_pulse(self._head_pitch, res)
        cur2 = _rad_to_pulse(self._head_yaw, res)
        d1 = cur1 - ref1
        d2 = cur2 - ref2

        path = self._save_delta_yaml_path_resolved()
        text = _format_head_pulse_delta_yaml(
            saved_at_unix_sec=time.time(),
            pulse_resolution=res,
            joint1_name=self._j1,
            joint2_name=self._j2,
            ref1=ref1,
            ref2=ref2,
            cur1=cur1,
            cur2=cur2,
            d1=d1,
            d2=d2,
        )
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(text, encoding='utf-8')
        except OSError as e:
            response.success = False
            response.message = f'Failed to write {path}: {e}'
            return response

        msg_parts = [f'Wrote head pulse delta YAML to {path}']

        if _param_bool(self.get_parameter('save_delta_apply_ros2_control').value):
            xacro_raw = self.get_parameter('ros2_control_xacro_path').value
            xacro_path = _apply_hom.resolve_ros2_control_xacro_path(
                str(xacro_raw) if xacro_raw is not None else '',
            )
            if xacro_path is None:
                response.success = False
                response.message = (
                    f'{msg_parts[0]}; could not resolve ros2_control xacro '
                    f'(ros2_control_xacro_path={xacro_raw!r}). Homing Offset not patched.'
                )
                self.get_logger().error(response.message)
                return response
            rc = _apply_hom.run(
                path,
                xacro_path,
                dict(_apply_hom.DEFAULT_JOINT_TO_GPIO),
                dry_run=False,
                backup=False,
            )
            if rc in (2, 3):
                response.success = False
                response.message = (
                    f'{msg_parts[0]}; ros2_control xacro patch failed (code {rc}) '
                    f'for {xacro_path}.'
                )
                self.get_logger().error(response.message)
                return response
            if rc == 4:
                msg_parts.append(
                    f'xacro {xacro_path} already matched Homing Offset (no file change).'
                )
            else:
                msg_parts.append(
                    f'Updated Homing Offset in {xacro_path}.'
                )

        response.success = True
        response.message = '; '.join(msg_parts)
        self.get_logger().info(response.message)
        return response

    def destroy_node(self):
        self._marker_detector.close()
        super().destroy_node()

    def _on_joint_state(self, msg):
        try:
            i1 = msg.name.index(self._j1)
            i2 = msg.name.index(self._j2)
        except ValueError:
            return
        if i1 < len(msg.position) and i2 < len(msg.position):
            self._head_pitch = float(msg.position[i1])
            self._head_yaw = float(msg.position[i2])
            self._cmd_pitch = self._head_pitch
            self._cmd_yaw = self._head_yaw

            if self._head_joint_log_dt > 0.0:
                now = time.monotonic()
                if now - self._head_joint_log_last_t >= self._head_joint_log_dt:
                    self._head_joint_log_last_t = now
                    p1 = _rad_to_pulse(self._head_pitch, self._head_pulse_res)
                    p2 = _rad_to_pulse(self._head_yaw, self._head_pulse_res)
                    d1 = p1 - self._pulse_ref_j1
                    d2 = p2 - self._pulse_ref_j2
                    self.get_logger().info(
                        f'{self._j1} pulse={p1} (ref={self._pulse_ref_j1}, '
                        f'delta={d1}), {self._j2} pulse={p2} '
                        f'(ref={self._pulse_ref_j2}, delta={d2}) '
                        f'(resolution={self._head_pulse_res})',
                    )

    def _base_head_pitch_yaw(self) -> tuple[float, float]:
        """Prefer live joint state; else last commanded pose; else fallback."""
        if self._head_pitch is not None and self._head_yaw is not None:
            return self._head_pitch, self._head_yaw
        if self._cmd_pitch is not None and self._cmd_yaw is not None:
            return self._cmd_pitch, self._cmd_yaw
        return self._fallback_pitch, self._fallback_yaw

    def _lost_tag_search_limits(self) -> tuple[float, float] | None:
        lo = min(self._lost_search_pmin, self._lost_search_pmax)
        hi = max(self._lost_search_pmin, self._lost_search_pmax)
        lo = float(np.clip(lo, self._j1_min, self._j1_max))
        hi = float(np.clip(hi, self._j1_min, self._j1_max))
        if hi - lo < 1e-6:
            return None
        return lo, hi

    def _publish_head_trajectory(self, pitch: float, yaw: float) -> None:
        pitch = float(np.clip(pitch, self._j1_min, self._j1_max))
        yaw = float(np.clip(yaw, self._j2_min, self._j2_max))
        self._cmd_pitch = pitch
        self._cmd_yaw = yaw
        traj = JointTrajectory()
        traj.header.stamp.sec = 0
        traj.header.stamp.nanosec = 0
        traj.joint_names = self._joint_names.copy()
        pt = JointTrajectoryPoint()
        pt.positions = [pitch, yaw]
        pt.time_from_start.sec = 0
        pt.time_from_start.nanosec = 50_000_000  # 50 ms
        traj.points = [pt]
        self._traj_pub.publish(traj)

    def _handle_detection_miss_motion(self) -> None:
        """Avoid sweep jitter: hold last track pose for brief detection dropouts."""
        self._miss_streak += 1
        if (
            self._last_good_track_pitch is not None
            and self._last_good_track_yaw is not None
        ):
            if (
                self._miss_streak <= self._miss_grace_frames
                or not self._lost_search_enabled
            ):
                self._publish_head_trajectory(
                    self._last_good_track_pitch,
                    self._last_good_track_yaw,
                )
                return
        if self._lost_search_enabled:
            self._publish_lost_tag_search_trajectory()

    def _publish_lost_tag_search_trajectory(self) -> None:
        if not self._lost_search_enabled:
            return
        lim = self._lost_tag_search_limits()
        if lim is None:
            return
        lo, hi = lim
        bp, by = self._base_head_pitch_yaw()
        if self._lost_search_pitch is None:
            self._lost_search_pitch = float(np.clip(bp, lo, hi))
            self._lost_search_fixed_yaw = float(
                np.clip(by, self._j2_min, self._j2_max))
            dist_up = hi - self._lost_search_pitch
            dist_down = self._lost_search_pitch - lo
            self._lost_search_dir = 1.0 if dist_up >= dist_down else -1.0
        step = self._lost_search_step
        nxt = self._lost_search_pitch + self._lost_search_dir * step
        if nxt >= hi:
            nxt = hi
            self._lost_search_dir = -1.0
        elif nxt <= lo:
            nxt = lo
            self._lost_search_dir = 1.0
        self._lost_search_pitch = float(nxt)
        yaw_hold = (
            self._lost_search_fixed_yaw
            if self._lost_search_fixed_yaw is not None
            else float(np.clip(by, self._j2_min, self._j2_max)))
        self._publish_head_trajectory(self._lost_search_pitch, yaw_hold)

    def _draw_lost_tag_search_overlay(self, viz: np.ndarray) -> None:
        lo = min(self._lost_search_pmin, self._lost_search_pmax)
        hi = max(self._lost_search_pmin, self._lost_search_pmax)
        in_grace = (
            self._last_good_track_pitch is not None
            and self._last_good_track_yaw is not None
            and self._miss_streak < self._miss_grace_frames
        )
        if in_grace:
            nxt = self._miss_streak + 1
            line = (
                f'Tag dropout: hold pose ({nxt}/{self._miss_grace_frames} misses)'
            )
        else:
            line = (
                f'Lost tag: {self._j1} sweep {lo:.2f}..{hi:.2f} rad ({self._j2} held)'
            )
        cv2.putText(
            viz,
            line,
            (20, 75),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 2,
        )

    def _publish_debug_frame(self, msg: Image, bgr: np.ndarray) -> None:
        if self._debug_pub is None:
            return
        out = self._bridge.cv2_to_imgmsg(bgr, encoding='bgr8')
        out.header = msg.header
        self._debug_pub.publish(out)

    def _rviz_lifetime_msg(self) -> Duration:
        lt = Duration()
        s = max(0.05, self._rviz_life)
        lt.sec = int(s)
        lt.nanosec = int(round((s - int(s)) * 1e9))
        return lt

    def _publish_rviz_clear(self, header) -> None:
        if self._rviz_pub is None:
            return
        del_m = Marker()
        del_m.header = header
        del_m.ns = 'apriltag_rviz'
        del_m.id = 0
        del_m.action = Marker.DELETEALL
        arr = MarkerArray()
        arr.markers.append(del_m)
        self._rviz_pub.publish(arr)

    def _publish_rviz_markers(
        self,
        header,
        corners: list,
        ids,
        h: int,
        w: int,
        choice_idx: int | None,
    ) -> None:
        if self._rviz_pub is None or not corners:
            return
        arr = MarkerArray()
        clear = Marker()
        clear.header = header
        clear.ns = 'apriltag_rviz'
        clear.id = 0
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        ids_flat = ids.flatten().tolist() if ids is not None else []
        scale = self._rviz_px_scale
        z0 = self._rviz_plane_z
        life = self._rviz_lifetime_msg()
        z_txt = z0 + 0.02

        green = ColorRGBA(r=0.15, g=0.95, b=0.2, a=0.9)
        yellow = ColorRGBA(r=1.0, g=0.92, b=0.1, a=1.0)
        magenta = ColorRGBA(r=0.9, g=0.1, b=0.9, a=0.85)

        mid = 1
        for i, c in enumerate(corners):
            pts = np.asarray(c, dtype=np.float64).reshape(-1, 2)
            strip = Marker()
            strip.header = header
            strip.ns = 'apriltag_rviz'
            strip.id = mid
            mid += 1
            strip.type = Marker.LINE_STRIP
            strip.action = Marker.ADD
            strip.scale.x = self._rviz_line_w
            strip.pose.orientation.w = 1.0
            strip.lifetime = life
            strip.color = yellow if choice_idx is not None and i == choice_idx else green
            for k in range(5):
                u, v = pts[k % 4]
                strip.points.append(
                    _pixel_to_rviz_point(u, v, w, h, scale, z0))
            arr.markers.append(strip)

            tid = int(ids_flat[i]) if i < len(ids_flat) else i
            cx = float(pts[:, 0].mean())
            cy = float(pts[:, 1].mean())
            txt = Marker()
            txt.header = header
            txt.ns = 'apriltag_rviz'
            txt.id = mid
            mid += 1
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.orientation.w = 1.0
            txt.pose.position = _pixel_to_rviz_point(
                cx, cy, w, h, scale, z_txt)
            txt.scale.z = max(0.03, min(0.2, 0.04 * (h / 720.0)))
            txt.color = ColorRGBA(r=1.0, g=0.5, b=0.1, a=1.0)
            txt.text = f'AprilTag {tid}'
            txt.lifetime = life
            arr.markers.append(txt)

        if choice_idx is not None and 0 <= choice_idx < len(corners):
            cx, cy = marker_center(corners[choice_idx])
            beam = Marker()
            beam.header = header
            beam.ns = 'apriltag_rviz'
            beam.id = mid
            beam.type = Marker.LINE_LIST
            beam.action = Marker.ADD
            beam.scale.x = self._rviz_line_w * 0.85
            beam.pose.orientation.w = 1.0
            beam.lifetime = life
            beam.color = magenta
            beam.points.append(
                _pixel_to_rviz_point(w * 0.5, h * 0.5, w, h, scale, z0))
            beam.points.append(
                _pixel_to_rviz_point(cx, cy, w, h, scale, z0))
            arr.markers.append(beam)

        self._rviz_pub.publish(arr)

    def _encoding_settings(self, msg: Image) -> tuple[str, str]:
        p = str(self._encoding_param or 'auto').strip().lower()
        if p in ('auto', '', 'passthrough'):
            return 'passthrough', (
                getattr(msg, 'encoding', None) or 'bgr8').lower()
        enc = str(self._encoding_param)
        return enc, enc.lower()

    def _on_image(self, msg: Image):
        desired, logical_enc = self._encoding_settings(msg)
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired)
        except CvBridgeError as e:
            self.get_logger().warn(f'cv_bridge: {e}')
            return

        bgr, gray = bgr_and_gray(cv_image, logical_enc)
        viz = bgr.copy()
        h, w = gray.shape[:2]
        cx0, cy0 = int(w // 2), int(h // 2)
        cv2.drawMarker(
            viz, (cx0, cy0), (0, 255, 0), markerType=cv2.MARKER_CROSS,
            markerSize=24, thickness=2)

        corners, ids = self._marker_detector.detect(gray)

        if corners is None:
            self._publish_rviz_clear(msg.header)
            if self._debug_image_enabled:
                cv2.putText(
                    viz, 'AprilTag worker / detect failed', (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                if self._lost_search_enabled:
                    self._draw_lost_tag_search_overlay(viz)
                self._publish_debug_frame(msg, viz)
            self._handle_detection_miss_motion()
            return

        if ids is None or len(corners) == 0:
            self._publish_rviz_clear(msg.header)
            if self._debug_image_enabled:
                cv2.putText(
                    viz, 'No AprilTags', (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 165, 255), 2)
                if self._lost_search_enabled:
                    self._draw_lost_tag_search_overlay(viz)
                self._publish_debug_frame(msg, viz)
            self._handle_detection_miss_motion()
            return

        draw_markers_overlay(viz, corners, ids)
        ids_flat = ids.flatten()

        choice: int | None = None
        if self._target_id >= 0:
            for i, mid in enumerate(ids_flat):
                if int(mid) == self._target_id:
                    choice = i
                    break
            if choice is None:
                if self._debug_image_enabled:
                    cv2.putText(
                        viz, f'Target id {self._target_id} not seen', (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                    if self._lost_search_enabled:
                        self._draw_lost_tag_search_overlay(viz)
                    self._publish_debug_frame(msg, viz)
                self._publish_rviz_markers(
                    msg.header, corners, ids, h, w, None)
                self._handle_detection_miss_motion()
                return
        else:
            choice = int(max(
                range(len(corners)),
                key=lambda i: marker_area(corners[i]),
            ))

        self._lost_search_pitch = None
        self._lost_search_fixed_yaw = None
        self._miss_streak = 0

        cx, cy = marker_center(corners[choice])
        tid = int(ids_flat[choice])
        tcx, tcy = int(round(cx)), int(round(cy))
        cv2.circle(viz, (tcx, tcy), 18, (255, 0, 255), 3)
        cv2.line(viz, (cx0, cy0), (tcx, tcy), (255, 0, 255), 2)
        cv2.putText(
            viz, f'Track id {tid}', (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)

        if self._debug_image_enabled:
            self._publish_debug_frame(msg, viz)

        self._publish_rviz_markers(
            msg.header, corners, ids, h, w, choice)

        if (self._head_pitch is None or self._head_yaw is None) and (
                not self._warned_openloop):
            self._warned_openloop = True
            self.get_logger().warn(
                f'No {self._j1}/{self._j2} in {self._js_topic!r} yet — '
                'centering uses open-loop from joint origin (0) / last command. '
                'Fix joint_states_topic or bring up joint_state_broadcaster for '
                'closed-loop tracking.'
            )

        bp, by = self._base_head_pitch_yaw()

        ex = cx - (w * 0.5)
        ey = cy - (h * 0.5)
        if abs(ex) < self._deadband:
            ex = 0.0
        if abs(ey) < self._deadband:
            ey = 0.0

        nx = ex / max(w * 0.5, 1.0)
        ny = ey / max(h * 0.5, 1.0)

        err_n = float(np.hypot(nx, ny))
        gain_scale = 1.0 + self._near_boost * max(
            0.0, 1.0 - min(err_n / self._near_boost_r, 1.0))

        d_yaw = self._yaw_sign * self._yaw_gain * gain_scale * nx
        d_pitch = self._pitch_sign * self._pitch_gain * gain_scale * ny
        d_yaw = float(np.clip(d_yaw, -self._max_sy, self._max_sy))
        d_pitch = float(np.clip(d_pitch, -self._max_sp, self._max_sp))

        pitch = float(np.clip(bp + d_pitch, self._j1_min, self._j1_max))
        yaw = float(np.clip(by + d_yaw, self._j2_min, self._j2_max))

        self._last_good_track_pitch = pitch
        self._last_good_track_yaw = yaw
        self._publish_head_trajectory(pitch, yaw)


def main(args=None):
    rclpy.init(args=args)
    node = AprilHeadTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
