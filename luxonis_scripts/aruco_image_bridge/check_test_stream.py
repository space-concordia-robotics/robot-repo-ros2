"""Receive and check the synthetic ROS stream. This is NOT the team tracker.

OpenCV is used only as an independent image-content check. This script does
not publish ArucoDetections or TF and does not validate physical poses.

Run from luxonis_scripts/, while publish_test_images is running:
    python3 -m aruco_image_bridge.check_test_stream [--dictionary 5X5_50]
"""

import argparse
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from .synthetic_frames import DICTIONARIES, FRAME_ID, HEIGHT, WIDTH, detect_ids, synthetic_calibration


def timestamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


class StreamCheck(Node):
    def __init__(self, dictionary_name, marker_id):
        super().__init__("aruco_test_stream_checker")
        self.dictionary_name, self.marker_id = dictionary_name, marker_id
        self.images, self.infos = {}, {}
        self.stamps = []
        self.received_at = []
        self.marker_frames = self.blank_frames = 0
        self.result = None
        self.previous_kind = None
        self.image_sub = self.create_subscription(Image, "/test/camera/front/image_raw", self.on_image, qos_profile_sensor_data)
        self.info_sub = self.create_subscription(CameraInfo, "/test/camera/front/camera_info", self.on_info, 10)
        self.get_logger().info("Waiting for 30 matching Image/CameraInfo pairs (about 6 seconds)...")

    def on_image(self, image):
        stamp = timestamp_ns(image.header)
        self.images[stamp] = image
        self.check_pair(stamp)

    def on_info(self, info):
        stamp = timestamp_ns(info.header)
        self.infos[stamp] = info
        self.check_pair(stamp)

    def check_pair(self, stamp):
        # Bound memory even when one topic is absent or timestamps mismatch.
        for pending in (self.images, self.infos):
            while len(pending) > 20:
                del pending[min(pending)]
        if self.result is not None or stamp not in self.images or stamp not in self.infos:
            return
        image, info = self.images.pop(stamp), self.infos.pop(stamp)
        try:
            self.validate(image, info, stamp)
        except Exception as error:
            self.get_logger().error(f"FAIL: {error}")
            self.result = False

    def validate(self, image, info, stamp):
        def require(condition, message):
            if not condition:
                raise ValueError(message)

        require(image.header.frame_id == info.header.frame_id == FRAME_ID, "Wrong/mismatched optical frame")
        require(stamp > 0, "Zero timestamp")
        require(not self.stamps or stamp > self.stamps[-1], "Duplicate or out-of-order timestamp")
        require((image.width, image.height) == (info.width, info.height) == (WIDTH, HEIGHT), "Geometry mismatch")
        require(image.encoding == "bgr8", "Expected BGR8 encoding")
        require(image.step == WIDTH * 3 and len(image.data) == image.step * HEIGHT, "Wrong buffer size/stride")
        calibration = synthetic_calibration()
        require(info.distortion_model == calibration.distortion_model, "Wrong distortion model")
        for field in ("d", "k", "r", "p"):
            require(
                np.allclose(getattr(info, field), getattr(calibration, field)),
                f"Unexpected synthetic calibration: {field}",
            )

        frame = np.frombuffer(bytes(image.data), dtype=np.uint8).reshape(HEIGHT, WIDTH, 3)
        ids = detect_ids(frame, self.dictionary_name)
        blank = bool(np.all(frame == 255))
        if blank:
            require(ids == [], f"Blank image produced false detection: {ids}")
            self.blank_frames += 1
            kind = "blank image: no marker"
        else:
            require(ids == [self.marker_id], f"Expected marker {self.marker_id}, received {ids}; check dictionary")
            self.marker_frames += 1
            kind = f"marker image: ID {self.marker_id}"
        if kind != self.previous_kind:
            self.get_logger().info(kind)
            self.previous_kind = kind

        self.stamps.append(stamp)
        self.received_at.append(time.monotonic())
        if len(self.stamps) >= 30:
            source_fps = (len(self.stamps) - 1) * 1e9 / (self.stamps[-1] - self.stamps[0])
            receive_fps = (len(self.received_at) - 1) / (self.received_at[-1] - self.received_at[0])
            require(self.marker_frames >= 5 and self.blank_frames >= 3, "Both marker and blank phases must be observed")
            require(3.0 <= source_fps <= 7.0, f"Source rate {source_fps:.2f} FPS is far from the 5 FPS target")
            require(3.0 <= receive_fps <= 7.0, f"Receive rate {receive_fps:.2f} FPS is far from the 5 FPS target")
            self.get_logger().info(
                f"PASS: {len(self.stamps)} matched pairs; {self.marker_frames} marker frames, "
                f"{self.blank_frames} blank frames; source {source_fps:.2f} FPS, received {receive_fps:.2f} FPS. "
                "Synthetic publisher/transport check only; team tracker and hardware remain untested."
            )
            self.result = True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dictionary", choices=DICTIONARIES, default="4X4_100")
    parser.add_argument("--marker-id", type=int, default=7)
    args = parser.parse_args()
    rclpy.init(args=[])
    node = StreamCheck(args.dictionary, args.marker_id)
    deadline = time.monotonic() + 25.0
    try:
        while rclpy.ok() and node.result is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        if node.result is None:
            node.get_logger().error(
                f"FAIL: timeout; {len(node.stamps)} matching pairs. Check the publisher, ROS_DOMAIN_ID, ROS_LOCALHOST_ONLY, and matching timestamps/QoS."
            )
    except KeyboardInterrupt:
        pass
    finally:
        passed = node.result is True
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()
