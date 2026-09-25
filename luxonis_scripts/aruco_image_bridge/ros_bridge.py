"""
ROS 2 output side of the bridge: messages, publishers and the rclpy node.

This module does not use cv_bridge. The camera scripts run in a virtualenv with
pip-installed OpenCV/NumPy while ROS uses the system packages; building the
Image message straight from the NumPy buffer avoids mixing the two.
"""

from __future__ import annotations

import array
from dataclasses import dataclass

import numpy as np
import numpy.typing as npt
import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from .calibration import CameraCalibration

_NS_PER_S = 1_000_000_000
_CAMERA_INFO_QUEUE = 10


@dataclass(frozen=True)
class PixelLayout:
    """NumPy layout that a ROS image encoding requires."""

    dtype: type[np.unsignedinteger]
    channels: int


ENCODINGS: dict[str, PixelLayout] = {
    "mono8": PixelLayout(np.uint8, 1),
    "mono16": PixelLayout(np.uint16, 1),
    "bgr8": PixelLayout(np.uint8, 3),
    "rgb8": PixelLayout(np.uint8, 3),
    "bgra8": PixelLayout(np.uint8, 4),
    "rgba8": PixelLayout(np.uint8, 4),
}

# Encoding assumed for each (dtype, channels) when the caller does not name one;
# 3/4 channels default to OpenCV's BGR order.
_DEFAULT_ENCODINGS: dict[tuple[type[np.unsignedinteger], int], str] = {
    (np.uint8, 1): "mono8",
    (np.uint16, 1): "mono16",
    (np.uint8, 3): "bgr8",
    (np.uint8, 4): "bgra8",
}


def stamp_from_ns(nanoseconds: int) -> Time:
    """Convert integer nanoseconds to a builtin_interfaces/Time."""
    stamp = Time()
    stamp.sec, stamp.nanosec = divmod(int(nanoseconds), _NS_PER_S)
    return stamp


def _channels(frame: npt.NDArray[np.generic]) -> int:
    if frame.ndim == 2:  # noqa: PLR2004
        return 1
    if frame.ndim == 3:  # noqa: PLR2004
        return int(frame.shape[2])
    raise ValueError(f"Expected an HxW or HxWxC image, got shape {frame.shape}")


def infer_encoding(frame: npt.NDArray[np.generic]) -> str:
    """Encoding for a frame from its dtype and channel count (BGR order for colour)."""
    key = (frame.dtype.type, _channels(frame))
    if key not in _DEFAULT_ENCODINGS:
        raise ValueError(f"No default ROS encoding for dtype {frame.dtype} with {key[1]} channel(s); pass one of {list(ENCODINGS)}")
    return _DEFAULT_ENCODINGS[key]


def image_msg(frame: npt.NDArray[np.generic], stamp: Time, frame_id: str, encoding: str | None = None) -> Image:
    """
    Wrap an image array in a sensor_msgs/Image.

    ``encoding`` is one of ``ENCODINGS``; when omitted it is inferred from the
    array (``mono8``, ``mono16``, ``bgr8`` or ``bgra8``).
    """
    if not isinstance(frame, np.ndarray):
        raise TypeError(f"Expected a NumPy image array, got {type(frame).__name__}")
    if not frame_id:
        raise ValueError("An optical frame_id is required")
    encoding = encoding or infer_encoding(frame)
    if encoding not in ENCODINGS:
        raise ValueError(f"Unsupported encoding '{encoding}'; supported: {list(ENCODINGS)}")
    layout = ENCODINGS[encoding]
    if frame.dtype.type is not layout.dtype or _channels(frame) != layout.channels:
        raise ValueError(
            f"Encoding '{encoding}' needs {layout.channels} channel(s) of {np.dtype(layout.dtype)}, got {_channels(frame)} of {frame.dtype}",
        )

    # Contiguous, little-endian pixels, so is_bigendian is always 0.
    pixels = np.ascontiguousarray(frame, dtype=frame.dtype.newbyteorder("<"))
    height, width = pixels.shape[:2]

    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = height
    msg.width = width
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = width * layout.channels * pixels.itemsize
    # array('B') is the fast path for uint8[] fields (the same approach cv_bridge uses).
    msg.data = array.array("B", pixels.tobytes())
    return msg


def camera_info_msg(calibration: CameraCalibration, stamp: Time, frame_id: str) -> CameraInfo:
    """sensor_msgs/CameraInfo for an unrectified image described by ``calibration``."""
    if not frame_id:
        raise ValueError("An optical frame_id is required")
    msg = CameraInfo()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.width = calibration.width
    msg.height = calibration.height
    msg.distortion_model = calibration.distortion_model
    msg.d = [float(value) for value in calibration.d]
    msg.k = [float(value) for value in calibration.k]
    msg.r = [float(value) for value in calibration.r]
    msg.p = [float(value) for value in calibration.p]
    return msg


class CameraPublisher:
    """Publishes one camera's Image + CameraInfo pair with identical headers."""

    image_topic: str
    info_topic: str
    frame_id: str
    published: int
    _bridge: RosBridge
    _image_pub: Publisher
    _info_pub: Publisher

    def __init__(self, bridge: RosBridge, image_topic: str, info_topic: str, frame_id: str) -> None:
        """Create the two publishers on ``bridge``'s node."""
        self._bridge = bridge
        self.image_topic = image_topic
        self.info_topic = info_topic
        self.frame_id = frame_id
        # Best effort for images, matching the tracker's default image_sub_qos.
        self._image_pub = bridge.node.create_publisher(Image, image_topic, qos_profile_sensor_data)
        # Reliable/volatile, matching the tracker's CameraInfo subscription.
        self._info_pub = bridge.node.create_publisher(CameraInfo, info_topic, _CAMERA_INFO_QUEUE)
        self.published = 0

    def publish(
        self,
        frame: npt.NDArray[np.generic],
        calibration: CameraCalibration,
        stamp_ns: int | None = None,
        encoding: str | None = None,
    ) -> None:
        """Publish CameraInfo then Image, both stamped ``stamp_ns`` (default: now)."""
        height, width = frame.shape[:2]
        if (width, height) != (calibration.width, calibration.height):
            raise ValueError(f"Image is {width}x{height} but calibration is for {calibration.width}x{calibration.height}")
        stamp = stamp_from_ns(self._bridge.now_ns() if stamp_ns is None else stamp_ns)
        self._info_pub.publish(camera_info_msg(calibration, stamp, self.frame_id))
        self._image_pub.publish(image_msg(frame, stamp, self.frame_id, encoding))
        self.published += 1


class RosBridge:
    """Owns rclpy and a single node. Publishing needs no executor or spin thread."""

    node: Node
    _owns_context: bool
    _publishers: dict[tuple[str, str, str], CameraPublisher]

    def __init__(self, node_name: str = "luxonis_ros_bridge") -> None:
        """Initialise rclpy if needed and create the node."""
        self._owns_context = not rclpy.ok()
        if self._owns_context:
            # args=[] so rclpy never tries to parse this script's own command line.
            rclpy.init(args=[])
        self.node = rclpy.create_node(node_name)
        self._publishers = {}

    def now_ns(self) -> int:
        """Current ROS time in nanoseconds."""
        return int(self.node.get_clock().now().nanoseconds)

    def info(self, message: str) -> None:
        """Log at INFO level through the node's logger."""
        self.node.get_logger().info(message)

    def warn(self, message: str) -> None:
        """Log at WARN level through the node's logger."""
        self.node.get_logger().warning(message)

    def error(self, message: str) -> None:
        """Log at ERROR level through the node's logger."""
        self.node.get_logger().error(message)

    def camera_publisher(self, image_topic: str, info_topic: str, frame_id: str) -> CameraPublisher:
        """Return the publisher pair for these topics, creating it once."""
        key = (image_topic, info_topic, frame_id)
        if key not in self._publishers:
            self._publishers[key] = CameraPublisher(self, image_topic, info_topic, frame_id)
        return self._publishers[key]

    def shutdown(self) -> None:
        """Destroy the node, and shut rclpy down if this bridge started it."""
        self.node.destroy_node()
        if self._owns_context and rclpy.ok():
            rclpy.shutdown()
