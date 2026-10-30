#!/usr/bin/env python3
"""ROS 2 GelSight Driver Node.

Continuously discovers, connects to, and publishes tactile images from
an attached GelSight sensor (e.g. GelSight Mini). Automatically reconnects
if the sensor is disconnected and re-attached.
"""

import os
import threading
import time
from typing import Optional

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


def find_gelsight_device(name_filter: str = "gelsight") -> Optional[str]:
    """Search attached video devices for a GelSight sensor.

    Checks:
    1. /dev/v4l/by-id for symlinks matching name_filter and ending in index0.
    2. /sys/class/video4linux for entries matching name_filter with index 0.

    Returns the canonical /dev/video* path or None if not found.
    """
    filter_token = name_filter.lower().replace(" ", "_")
    filter_raw = name_filter.lower()

    # 1. Search persistent /dev/v4l/by-id symlinks
    by_id_dir = "/dev/v4l/by-id"
    if os.path.isdir(by_id_dir):
        try:
            for entry in sorted(os.listdir(by_id_dir)):
                entry_lower = entry.lower()
                if (filter_token in entry_lower or filter_raw in entry_lower) and entry.endswith("index0"):
                    full_path = os.path.join(by_id_dir, entry)
                    return os.path.realpath(full_path)
        except OSError:
            pass

    # 2. Search /sys/class/video4linux
    v4l_dir = "/sys/class/video4linux"
    if os.path.isdir(v4l_dir):
        try:
            for dev in sorted(os.listdir(v4l_dir)):
                dev_path = os.path.join(v4l_dir, dev)
                name_file = os.path.join(dev_path, "name")
                index_file = os.path.join(dev_path, "index")
                if os.path.isfile(name_file):
                    try:
                        with open(name_file, "r") as f:
                            dev_name = f.read().strip()
                        dev_index = "0"
                        if os.path.isfile(index_file):
                            with open(index_file, "r") as f:
                                dev_index = f.read().strip()
                        if (filter_raw in dev_name.lower()) and dev_index == "0":
                            return f"/dev/{dev}"
                    except (OSError, IOError):
                        continue
        except OSError:
            pass

    return None


def crop_and_resize(
    img: np.ndarray, target_w: int, target_h: int, apply_crop: bool = True
) -> np.ndarray:
    """Resize and optionally crop the raw GelSight image.

    Applies standard 1/7 border cropping to remove border distortion while
    preserving tactile aspect ratio, matching the GelSight Mini specifications.
    """
    if not apply_crop:
        if target_w > 0 and target_h > 0:
            return cv2.resize(img, (target_w, target_h))
        return img

    h, w = img.shape[:2]
    border_y = int(h * (1.0 / 7.0))
    border_x = int(np.floor(w * (1.0 / 7.0)))
    cropped_h = h - 2 * border_y
    cropped_w = w - 2 * border_x

    extra_border_h = 0
    extra_border_w = 0
    if target_w > 0 and target_h > 0:
        if cropped_h * target_w / target_h > cropped_w + 1e-8:
            extra_border_h = int(cropped_h - cropped_w * target_h / target_w)
        elif cropped_h * target_w / target_h < cropped_w - 1e-8:
            extra_border_w = int(cropped_w - cropped_h * target_w / target_h)

    img_cropped = img[
        border_y + extra_border_h : h - border_y,
        border_x + extra_border_w : w - border_x,
    ]

    if target_w > 0 and target_h > 0:
        return cv2.resize(img_cropped, (target_w, target_h))
    return img_cropped


class GelSightDriverNode(Node):
    """ROS 2 node that automatically detects, connects to, and streams a GelSight tactile sensor."""

    def __init__(self):
        super().__init__("gelsight_driver_node")

        # Declare parameters
        self.declare_parameter("device_name", "GelSight")
        self.declare_parameter("device_path", "")
        self.declare_parameter("device_index", -1)
        self.declare_parameter("frame_id", "gelsight_frame")
        self.declare_parameter("image_topic", "image_raw")
        self.declare_parameter("frame_rate", 30.0)
        self.declare_parameter("image_width", 1280)
        self.declare_parameter("image_height", 720)
        self.declare_parameter("crop", True)
        self.declare_parameter("reconnect_interval", 2.0)

        # Retrieve parameters
        self.device_name = self.get_parameter("device_name").get_parameter_value().string_value
        self.device_path = self.get_parameter("device_path").get_parameter_value().string_value
        self.device_index = self.get_parameter("device_index").get_parameter_value().integer_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.frame_rate = self.get_parameter("frame_rate").get_parameter_value().double_value
        self.image_width = self.get_parameter("image_width").get_parameter_value().integer_value
        self.image_height = self.get_parameter("image_height").get_parameter_value().integer_value
        self.crop = self.get_parameter("crop").get_parameter_value().bool_value
        self.reconnect_interval = (
            self.get_parameter("reconnect_interval").get_parameter_value().double_value
        )

        self.cv_bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, self.image_topic, 10)

        self._running = True
        self._cap: Optional[cv2.VideoCapture] = None
        self._connected_dev: Optional[str] = None

        self.get_logger().info(
            f"GelSight Driver initialized. Target topic: '{self.image_topic}', "
            f"frame_id: '{self.frame_id}', searching for: '{self.device_name}'"
        )

        # Start worker thread for device polling and image streaming
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

    def _resolve_device(self) -> Optional[str]:
        """Resolve device path from parameters or by auto-discovery."""
        if self.device_index >= 0:
            return f"/dev/video{self.device_index}"
        if self.device_path:
            return self.device_path
        return find_gelsight_device(self.device_name)

    def _open_device(self, dev_path: str) -> bool:
        """Attempt to open and configure the camera device."""
        try:
            self.get_logger().info(f"Connecting to GelSight at {dev_path}...")
            cap = cv2.VideoCapture(dev_path, cv2.CAP_V4L2)
            if not cap.isOpened():
                # Fallback to default backend if V4L2 backend failed
                cap = cv2.VideoCapture(dev_path)

            if not cap.isOpened():
                self.get_logger().warn(f"Failed to open video device at {dev_path}")
                cap.release()
                return False

            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

            # Test reading a frame to verify camera is active
            ret, frame = cap.read()
            if not ret or frame is None:
                self.get_logger().warn(
                    f"Connected to {dev_path} but failed to capture initial frame."
                )
                cap.release()
                return False

            self._cap = cap
            self._connected_dev = dev_path
            h, w = frame.shape[:2]
            self.get_logger().info(
                f"Successfully connected to GelSight at {dev_path} (native resolution: {w}x{h})"
            )
            return True
        except Exception as e:
            self.get_logger().error(f"Error opening camera {dev_path}: {e}")
            return False

    def _close_device(self):
        """Cleanly release the camera device."""
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                pass
            self._cap = None
        self._connected_dev = None

    def _run_loop(self):
        """Worker loop that continuously checks device connection and publishes images."""
        last_log_time = 0.0
        frame_interval = 1.0 / self.frame_rate if self.frame_rate > 0 else 0.0

        while self._running and rclpy.ok():
            # 1. If not connected, continuously search for GelSight device
            if self._cap is None or not self._cap.isOpened():
                now = time.time()
                if now - last_log_time >= 5.0:
                    self.get_logger().info(
                        f"Searching for attached GelSight device ('{self.device_name}')..."
                    )
                    last_log_time = now

                dev_path = self._resolve_device()
                if dev_path is not None:
                    if self._open_device(dev_path):
                        continue

                # Wait before polling again
                time.sleep(self.reconnect_interval)
                continue

            # 2. If connected, capture and publish frames
            loop_start = time.time()
            try:
                ret, frame = self._cap.read()
            except Exception as e:
                self.get_logger().warn(f"Exception while reading frame: {e}")
                ret, frame = False, None

            if not ret or frame is None:
                self.get_logger().warn(
                    f"GelSight device ({self._connected_dev}) disconnected or read failed. "
                    "Resetting connection..."
                )
                self._close_device()
                time.sleep(self.reconnect_interval)
                continue

            # Process image (crop & resize)
            try:
                processed = crop_and_resize(
                    frame, self.image_width, self.image_height, apply_crop=self.crop
                )
                msg = self.cv_bridge.cv2_to_imgmsg(processed, encoding="bgr8")
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.frame_id
                self.image_pub.publish(msg)
            except Exception as e:
                self.get_logger().error(f"Error processing/publishing image: {e}")

            # Sleep to regulate frame rate if processing finished early
            if frame_interval > 0:
                elapsed = time.time() - loop_start
                sleep_time = frame_interval - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

    def destroy_node(self):
        """Clean shutdown hook."""
        self._running = False
        self._close_device()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GelSightDriverNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
