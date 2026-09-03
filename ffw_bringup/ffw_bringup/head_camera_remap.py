"""Publish the SG2 canonical 640x480 head-camera stream from ZED HD720."""

from __future__ import annotations

import copy

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image


class HeadCameraRemap(Node):
    """Convert ZED's rectified HD720 left image to the SG2 640x480 contract."""

    def __init__(self) -> None:
        super().__init__("head_camera_remap")
        self.declare_parameter("input_image_topic", "/zed/zed_node/left/image_rect_color")
        self.declare_parameter("input_camera_info_topic", "/zed/zed_node/left/camera_info")
        self.declare_parameter("output_image_topic", "/head_camera/color/image_rect/compressed")
        self.declare_parameter("output_camera_info_topic", "/head_camera/color/camera_info")
        self.declare_parameter("output_width", 640)
        self.declare_parameter("output_height", 480)
        self.declare_parameter("output_fx", 489.7808024)
        self.declare_parameter("output_fy", 489.7808024)
        self.declare_parameter("output_cx", 320.0)
        self.declare_parameter("output_cy", 240.0263)
        self.declare_parameter("jpeg_quality", 90)

        self._output_width = int(self.get_parameter("output_width").value)
        self._output_height = int(self.get_parameter("output_height").value)
        self._output_fx = float(self.get_parameter("output_fx").value)
        self._output_fy = float(self.get_parameter("output_fy").value)
        self._output_cx = float(self.get_parameter("output_cx").value)
        self._output_cy = float(self.get_parameter("output_cy").value)
        self._jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self._source_info: CameraInfo | None = None
        self._map_key: tuple[float, float, float, float, int, int] | None = None
        self._map_x: np.ndarray | None = None
        self._map_y: np.ndarray | None = None
        self._bridge = CvBridge()

        self.create_subscription(
            Image, self.get_parameter("input_image_topic").value, self._image_callback, 10
        )
        self.create_subscription(
            CameraInfo,
            self.get_parameter("input_camera_info_topic").value,
            self._camera_info_callback,
            10,
        )
        self._image_publisher = self.create_publisher(
            CompressedImage, self.get_parameter("output_image_topic").value, 10
        )
        self._camera_info_publisher = self.create_publisher(
            CameraInfo, self.get_parameter("output_camera_info_topic").value, 10
        )

    def _camera_info_callback(self, message: CameraInfo) -> None:
        if message.k[0] > 0.0 and message.k[4] > 0.0:
            self._source_info = message
        else:
            self.get_logger().warning("Ignoring CameraInfo with invalid focal length.")

    def _ensure_maps(self, source_info: CameraInfo) -> None:
        key = (
            float(source_info.k[0]), float(source_info.k[4]),
            float(source_info.k[2]), float(source_info.k[5]),
            int(source_info.width), int(source_info.height),
        )
        if key == self._map_key:
            return
        x_coords, y_coords = np.meshgrid(
            np.arange(self._output_width, dtype=np.float32),
            np.arange(self._output_height, dtype=np.float32),
        )
        # Source K may change after ZED self-calibration; output K is pinned.
        self._map_x = key[2] + (x_coords - self._output_cx) * key[0] / self._output_fx
        self._map_y = key[3] + (y_coords - self._output_cy) * key[1] / self._output_fy
        self._map_key = key

    def _canonical_camera_info(self, image_header) -> CameraInfo:
        assert self._source_info is not None
        message = copy.deepcopy(self._source_info)
        message.header = image_header
        message.width, message.height = self._output_width, self._output_height
        message.distortion_model = "plumb_bob"
        message.d = [0.0] * 5
        message.k = [self._output_fx, 0.0, self._output_cx, 0.0, self._output_fy,
                     self._output_cy, 0.0, 0.0, 1.0]
        message.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        message.p = [self._output_fx, 0.0, self._output_cx, 0.0, 0.0,
                     self._output_fy, self._output_cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        message.binning_x = message.binning_y = 0
        message.roi.x_offset = message.roi.y_offset = 0
        message.roi.width = message.roi.height = 0
        message.roi.do_rectify = False
        return message

    def _image_callback(self, message: Image) -> None:
        if self._source_info is None:
            self.get_logger().warning("Waiting for ZED CameraInfo before publishing head images.")
            return
        if (message.width, message.height) != (self._source_info.width, self._source_info.height):
            self.get_logger().warning("ZED image and CameraInfo dimensions differ; dropping frame.")
            return
        self._ensure_maps(self._source_info)
        try:
            source = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            output = cv2.remap(source, self._map_x, self._map_y, cv2.INTER_LINEAR)
            encoded, payload = cv2.imencode(
                ".jpg", output, [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality]
            )
        except Exception as error:
            self.get_logger().error(f"Head-camera remap failed: {error}")
            return
        if not encoded:
            self.get_logger().error("Head-camera JPEG encoding failed.")
            return
        compressed = CompressedImage()
        compressed.header, compressed.format, compressed.data = message.header, "jpeg", payload.tobytes()
        self._image_publisher.publish(compressed)
        self._camera_info_publisher.publish(self._canonical_camera_info(message.header))


def main() -> None:
    rclpy.init()
    node = HeadCameraRemap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
