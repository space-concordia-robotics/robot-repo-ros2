"""
Receive and check the synthetic ROS stream. This is NOT the team tracker.

OpenCV is used only as an independent image-content check. This script does
not publish ArucoDetections or TF and does not validate physical poses.

Run from luxonis_scripts/, while publish_test_images is running:
    python3 -m aruco_image_bridge.check_test_stream [--dictionary 5X5_50] [--encoding mono8]
"""

from __future__ import annotations

import argparse
import time

import numpy as np
import numpy.typing as npt
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header

from .luxonis_adapter import OUTPUT_ENCODINGS
from .ros_bridge import ENCODINGS
from .synthetic_frames import DICTIONARIES, FRAME_ID, HEIGHT, WHITE, WIDTH, detect_ids, synthetic_calibration

TEST_PREFIX = "/test/camera/front"
REQUIRED_PAIRS = 30
MAX_PENDING = 20
MIN_MARKER_FRAMES = 5
MIN_BLANK_FRAMES = 3
MIN_FPS, MAX_FPS = 3.0, 7.0
TIMEOUT_S = 25.0
SPIN_PERIOD_S = 0.2
CAMERA_INFO_QUEUE = 10
NS_PER_S = 1_000_000_000


class CheckFailedError(Exception):
    """A property of the stream is wrong."""


def require(condition: bool, message: str) -> None:
    """Raise CheckFailedError with ``message`` unless ``condition`` holds."""
    if not condition:
        raise CheckFailedError(message)


def timestamp_ns(header: Header) -> int:
    """Header stamp as integer nanoseconds."""
    return header.stamp.sec * NS_PER_S + header.stamp.nanosec


class StreamCheck(Node):
    """Pairs Image and CameraInfo by stamp and checks each pair, then the rates."""

    dictionary_name: str
    marker_id: int
    encoding: str
    images: dict[int, Image]
    infos: dict[int, CameraInfo]
    stamps: list[int]
    received_at: list[float]
    marker_frames: int
    blank_frames: int
    result: bool | None
    previous_kind: str | None

    def __init__(self, dictionary_name: str, marker_id: int, encoding: str) -> None:
        """Subscribe to the synthetic topics."""
        super().__init__("aruco_test_stream_checker")
        self.dictionary_name, self.marker_id, self.encoding = dictionary_name, marker_id, encoding
        self.images, self.infos = {}, {}
        self.stamps, self.received_at = [], []
        self.marker_frames = self.blank_frames = 0
        self.result = None
        self.previous_kind = None
        self.create_subscription(Image, f"{TEST_PREFIX}/image_raw", self.on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, f"{TEST_PREFIX}/camera_info", self.on_info, CAMERA_INFO_QUEUE)
        self.get_logger().info(f"Waiting for {REQUIRED_PAIRS} matching Image/CameraInfo pairs (about 6 seconds)...")

    def on_image(self, image: Image) -> None:
        """Store an image until its CameraInfo arrives."""
        stamp = timestamp_ns(image.header)
        self.images[stamp] = image
        self.check_pair(stamp)

    def on_info(self, info: CameraInfo) -> None:
        """Store a CameraInfo until its image arrives."""
        stamp = timestamp_ns(info.header)
        self.infos[stamp] = info
        self.check_pair(stamp)

    def check_pair(self, stamp: int) -> None:
        """Validate the pair for ``stamp`` once both halves are in."""
        # Bound memory even when one topic is absent or timestamps mismatch.
        for pending in (self.images, self.infos):
            while len(pending) > MAX_PENDING:
                del pending[min(pending)]
        if self.result is not None or stamp not in self.images or stamp not in self.infos:
            return
        image, info = self.images.pop(stamp), self.infos.pop(stamp)
        try:
            self.validate(image, info, stamp)
        except CheckFailedError as error:
            self.get_logger().error(f"FAIL: {error}")
            self.result = False

    def validate(self, image: Image, info: CameraInfo, stamp: int) -> None:
        """Check one pair; after enough pairs, check phases and rates."""
        self._check_headers(image, info, stamp)
        self._check_calibration(info)
        self._check_content(self._pixels(image))
        self.stamps.append(stamp)
        self.received_at.append(time.monotonic())
        if len(self.stamps) >= REQUIRED_PAIRS:
            self._finish()

    def _check_headers(self, image: Image, info: CameraInfo, stamp: int) -> None:
        require(image.header.frame_id == info.header.frame_id == FRAME_ID, "Wrong/mismatched optical frame")
        require(stamp > 0, "Zero timestamp")
        require(not self.stamps or stamp > self.stamps[-1], "Duplicate or out-of-order timestamp")
        require((image.width, image.height) == (info.width, info.height) == (WIDTH, HEIGHT), "Geometry mismatch")

    def _pixels(self, image: Image) -> npt.NDArray[np.uint8]:
        require(image.encoding == self.encoding, f"Expected {self.encoding} encoding, got {image.encoding}")
        channels = ENCODINGS[self.encoding].channels
        require(image.step == WIDTH * channels and len(image.data) == image.step * HEIGHT, "Wrong buffer size/stride")
        return np.frombuffer(bytes(image.data), dtype=np.uint8).reshape(HEIGHT, WIDTH, channels)

    @staticmethod
    def _check_calibration(info: CameraInfo) -> None:
        calibration = synthetic_calibration()
        require(info.distortion_model == calibration.distortion_model, "Wrong distortion model")
        for field in ("d", "k", "r", "p"):
            require(np.allclose(getattr(info, field), getattr(calibration, field)), f"Unexpected synthetic calibration: {field}")

    def _check_content(self, frame: npt.NDArray[np.uint8]) -> None:
        ids = detect_ids(frame, self.dictionary_name)
        if bool(np.all(frame == WHITE)):
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

    def _finish(self) -> None:
        source_fps = (len(self.stamps) - 1) * NS_PER_S / (self.stamps[-1] - self.stamps[0])
        receive_fps = (len(self.received_at) - 1) / (self.received_at[-1] - self.received_at[0])
        require(
            self.marker_frames >= MIN_MARKER_FRAMES and self.blank_frames >= MIN_BLANK_FRAMES,
            "Both marker and blank phases must be observed",
        )
        require(MIN_FPS <= source_fps <= MAX_FPS, f"Source rate {source_fps:.2f} FPS is far from the 5 FPS target")
        require(MIN_FPS <= receive_fps <= MAX_FPS, f"Receive rate {receive_fps:.2f} FPS is far from the 5 FPS target")
        self.get_logger().info(
            f"PASS: {len(self.stamps)} matched pairs; {self.marker_frames} marker frames, "
            f"{self.blank_frames} blank frames; source {source_fps:.2f} FPS, received {receive_fps:.2f} FPS. "
            "Synthetic publisher/transport check only; team tracker and hardware remain untested.",
        )
        self.result = True


def main() -> None:
    """Run the check and exit 0 on PASS, 1 otherwise."""
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dictionary", choices=DICTIONARIES, default="4X4_100")
    parser.add_argument("--marker-id", type=int, default=7)
    parser.add_argument("--encoding", choices=OUTPUT_ENCODINGS, default="bgr8")
    args = parser.parse_args()
    rclpy.init(args=[])
    node = StreamCheck(args.dictionary, args.marker_id, args.encoding)
    deadline = time.monotonic() + TIMEOUT_S
    try:
        while rclpy.ok() and node.result is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=SPIN_PERIOD_S)
        if node.result is None:
            node.get_logger().error(
                f"FAIL: timeout; {len(node.stamps)} matching pairs. Check the publisher, ROS_DOMAIN_ID, "
                "ROS_AUTOMATIC_DISCOVERY_RANGE, the --encoding value and matching timestamps/QoS.",
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
