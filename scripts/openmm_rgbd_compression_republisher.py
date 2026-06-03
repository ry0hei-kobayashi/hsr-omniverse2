#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import struct
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image


class OpenMMRgbdCompressionRepublisher(Node):
    """Publish OpenMM-compatible compressed RGB and compressedDepth topics."""

    def __init__(self) -> None:
        super().__init__('openmm_rgbd_compression_republisher')

        self.declare_parameter('rgb_input_topic', '/head_rgbd_sensor/rgb/image_rect_color')
        self.declare_parameter('rgb_output_topic', '/head_rgbd_sensor/rgb/image_rect_color/compressed')
        self.declare_parameter('depth_input_topic', '/head_rgbd_sensor/depth_registered/image_raw')
        self.declare_parameter('depth_output_topic', '/head_rgbd_sensor/depth_registered/image_raw/compressedDepth')
        self.declare_parameter('jpeg_quality', 95)
        self.declare_parameter('png_level', 3)
        self.declare_parameter('depth_quant_a', 1000.0)
        self.declare_parameter('depth_quant_b', 0.0)

        self.rgb_input_topic = self.get_parameter('rgb_input_topic').get_parameter_value().string_value
        self.rgb_output_topic = self.get_parameter('rgb_output_topic').get_parameter_value().string_value
        self.depth_input_topic = self.get_parameter('depth_input_topic').get_parameter_value().string_value
        self.depth_output_topic = self.get_parameter('depth_output_topic').get_parameter_value().string_value
        self.jpeg_quality = self.get_parameter('jpeg_quality').get_parameter_value().integer_value
        self.png_level = self.get_parameter('png_level').get_parameter_value().integer_value
        self.depth_quant_a = self.get_parameter('depth_quant_a').get_parameter_value().double_value
        self.depth_quant_b = self.get_parameter('depth_quant_b').get_parameter_value().double_value

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        self.rgb_pub = self.create_publisher(CompressedImage, self.rgb_output_topic, sensor_qos)
        self.depth_pub = self.create_publisher(CompressedImage, self.depth_output_topic, sensor_qos)
        self.rgb_sub = self.create_subscription(Image, self.rgb_input_topic, self._on_rgb, sensor_qos)
        self.depth_sub = self.create_subscription(Image, self.depth_input_topic, self._on_depth, sensor_qos)

        self._published_rgb_once = False
        self._published_depth_once = False
        self.get_logger().info(
            'OpenMM RGB-D compression republisher started: '
            f'{self.rgb_input_topic} -> {self.rgb_output_topic}, '
            f'{self.depth_input_topic} -> {self.depth_output_topic}'
        )

    def _on_rgb(self, msg: Image) -> None:
        bgr = self._image_to_bgr(msg)
        if bgr is None:
            return

        encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), int(self.jpeg_quality)]
        ok, encoded = cv2.imencode('.jpg', bgr, encode_params)
        if not ok:
            self.get_logger().warn('Failed to encode RGB image as JPEG')
            return

        out = CompressedImage()
        out.header = msg.header
        out.format = f'{msg.encoding}; jpeg compressed bgr8'
        out.data = encoded.tobytes()
        self.rgb_pub.publish(out)

        if not self._published_rgb_once:
            self.get_logger().info('Published first RGB compressed frame')
            self._published_rgb_once = True

    def _on_depth(self, msg: Image) -> None:
        compressed = self._depth_to_compressed_depth(msg)
        if compressed is None:
            return

        self.depth_pub.publish(compressed)
        if not self._published_depth_once:
            self.get_logger().info('Published first compressedDepth frame')
            self._published_depth_once = True

    def _image_to_bgr(self, msg: Image) -> Optional[np.ndarray]:
        try:
            row = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
            encoding = msg.encoding.lower()

            if encoding in ('rgb8', 'bgr8'):
                img = row[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
                return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if encoding == 'rgb8' else img.copy()

            if encoding in ('rgba8', 'bgra8'):
                img = row[:, :msg.width * 4].reshape(msg.height, msg.width, 4)
                return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR) if encoding == 'rgba8' else cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)

            if encoding in ('mono8', '8uc1'):
                img = row[:, :msg.width]
                return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

            if encoding == '8uc3':
                return row[:, :msg.width * 3].reshape(msg.height, msg.width, 3).copy()

            self.get_logger().warn(f'Unsupported RGB encoding: {msg.encoding}')
            return None
        except Exception as exc:
            self.get_logger().warn(f'Failed to convert RGB image: {exc}')
            return None

    def _depth_to_compressed_depth(self, msg: Image) -> Optional[CompressedImage]:
        encoding = msg.encoding.upper()
        try:
            if encoding == '32FC1':
                depth = self._image_to_array(msg, np.float32, 4)
                valid = np.isfinite(depth) & (depth > 0.0)
                quantized = np.zeros(depth.shape, dtype=np.uint16)
                quantized[valid] = np.clip(
                    np.rint(self.depth_quant_a / depth[valid] + self.depth_quant_b),
                    1,
                    65535,
                ).astype(np.uint16)
                compressed_format = '32FC1; compressedDepth png'
            elif encoding == '16UC1':
                quantized = self._image_to_array(msg, np.uint16, 2)
                compressed_format = '16UC1; compressedDepth png'
            else:
                self.get_logger().warn(f'Unsupported depth encoding: {msg.encoding}')
                return None

            encode_params = [int(cv2.IMWRITE_PNG_COMPRESSION), int(self.png_level)]
            ok, encoded = cv2.imencode('.png', quantized, encode_params)
            if not ok:
                self.get_logger().warn('Failed to encode depth image as PNG')
                return None

            out = CompressedImage()
            out.header = msg.header
            out.format = compressed_format
            out.data = struct.pack('iff', 0, self.depth_quant_a, self.depth_quant_b) + encoded.tobytes()
            return out
        except Exception as exc:
            self.get_logger().warn(f'Failed to convert depth image: {exc}')
            return None

    @staticmethod
    def _image_to_array(msg: Image, dtype: np.dtype, bytes_per_pixel: int) -> np.ndarray:
        row = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        packed = row[:, :msg.width * bytes_per_pixel]
        return packed.copy().view(dtype).reshape(msg.height, msg.width)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OpenMMRgbdCompressionRepublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
