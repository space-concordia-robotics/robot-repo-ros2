"""ROS 2 output side of the bridge: messages, publishers and the rclpy node."""

from __future__ import annotations

from typing import cast, override

import numpy as np
import numpy.typing as npt
import rclpy
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header

from .calibration import CameraCalibration
from .luxonis_adapter import FramePublisher

_NS_PER_S = 1_000_000_000
_CAMERA_INFO_QUEUE = 10


def stamp_from_ns(nanoseconds: int) -> Time:
    """Convert integer nanoseconds to a builtin_interfaces/Time."""
    stamp = Time()
    stamp.sec, stamp.nanosec = divmod(int(nanoseconds), _NS_PER_S)
    return stamp


_CV_BRIDGE = CvBridge()


def image_msg(frame: npt.NDArray[np.generic], header: Header, encoding: str) -> Image:
    """
    Wrap an image array in a sensor_msgs/Image with ``header``.

    cv_bridge checks that ``encoding`` matches the array's dtype and channel count
    (CvBridgeError otherwise) and copies the pixels as they are, without converting
    between channel orders.
    """
    # cv_bridge is untyped, so its result is Unknown to the type checker.
    return cast("Image", _CV_BRIDGE.cv2_to_imgmsg(frame, encoding=encoding, header=header))


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
    msg.d = calibration.d
    msg.k = calibration.k
    msg.r = calibration.r
    msg.p = calibration.p
    return msg


class CameraPublisher(FramePublisher):
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

    @override
    def publish(
        self,
        frame: npt.NDArray[np.generic],
        calibration: CameraCalibration,
        encoding: str,
        stamp_ns: int | None = None,
    ) -> None:
        """Publish CameraInfo then Image, both stamped ``stamp_ns`` (default: now)."""
        height, width = frame.shape[:2]
        if (width, height) != (calibration.width, calibration.height):
            raise ValueError(f"Image is {width}x{height} but calibration is for {calibration.width}x{calibration.height}")
        stamp = stamp_from_ns(self._bridge.now_ns() if stamp_ns is None else stamp_ns)
        self._info_pub.publish(camera_info_msg(calibration, stamp, self.frame_id))
        self._image_pub.publish(image_msg(frame, Header(stamp=stamp, frame_id=self.frame_id), encoding))
        self.published += 1


class RosBridge:
    """Owns rclpy and a single node. Publishing needs no executor or spin thread."""

    node: Node
    logger: RcutilsLogger
    _owns_context: bool
    _publishers: dict[tuple[str, str, str], CameraPublisher]

    def __init__(self, node_name: str = "luxonis_ros_bridge") -> None:
        """Initialise rclpy if needed and create the node."""
        self._owns_context = not rclpy.ok()
        if self._owns_context:
            # args=[] so rclpy never tries to parse this script's own command line.
            rclpy.init(args=[])
        self.node = rclpy.create_node(node_name)
        self.logger = self.node.get_logger()
        self._publishers = {}

    def now_ns(self) -> int:
        """Current ROS time in nanoseconds."""
        return int(self.node.get_clock().now().nanoseconds)

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
