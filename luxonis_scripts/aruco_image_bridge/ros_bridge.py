"""ROS 2 output side of the bridge: messages, publishers and the rclpy node.

This module does not use cv_bridge. The camera scripts run in a virtualenv with
pip-installed OpenCV/NumPy while ROS uses the system packages; building the
Image message straight from the NumPy buffer avoids mixing the two.
"""

import array

import numpy as np
from builtin_interfaces.msg import Time
from sensor_msgs.msg import CameraInfo, Image

IMAGE_ENCODING = "bgr8"


def stamp_from_ns(nanoseconds):
    stamp = Time()
    stamp.sec, stamp.nanosec = divmod(int(nanoseconds), 1_000_000_000)
    return stamp


def image_msg(frame, stamp, frame_id):
    """Wrap an HxWx3 uint8 BGR array in a sensor_msgs/Image."""
    if not isinstance(frame, np.ndarray) or frame.dtype != np.uint8 or frame.ndim != 3 or frame.shape[2] != 3:
        raise ValueError("Expected an HxWx3 uint8 BGR image")
    if not frame_id:
        raise ValueError("An optical frame_id is required")
    frame = np.ascontiguousarray(frame)
    height, width = frame.shape[:2]

    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = height
    msg.width = width
    msg.encoding = IMAGE_ENCODING
    msg.is_bigendian = 0
    msg.step = width * 3
    # array('B') is the fast path for uint8[] fields (the same approach cv_bridge uses).
    msg.data = array.array("B", frame.tobytes())
    return msg


def camera_info_msg(calibration, stamp, frame_id):
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

    def __init__(self, bridge, image_topic, info_topic, frame_id):
        from rclpy.qos import qos_profile_sensor_data

        self._bridge = bridge
        self.image_topic = image_topic
        self.info_topic = info_topic
        self.frame_id = frame_id
        # Best effort for images, matching the tracker's default image_sub_qos.
        self._image_pub = bridge.node.create_publisher(Image, image_topic, qos_profile_sensor_data)
        # Reliable/volatile, matching the tracker's CameraInfo subscription.
        self._info_pub = bridge.node.create_publisher(CameraInfo, info_topic, 10)
        self.published = 0

    def publish(self, frame, calibration, stamp_ns=None):
        height, width = frame.shape[:2]
        if (width, height) != (calibration.width, calibration.height):
            raise ValueError(f"Image is {width}x{height} but calibration is for {calibration.width}x{calibration.height}")
        stamp = stamp_from_ns(self._bridge.now_ns() if stamp_ns is None else stamp_ns)
        self._info_pub.publish(camera_info_msg(calibration, stamp, self.frame_id))
        self._image_pub.publish(image_msg(frame, stamp, self.frame_id))
        self.published += 1


class RosBridge:
    """Owns rclpy and a single node. Publishing needs no executor or spin thread."""

    def __init__(self, node_name="luxonis_ros_bridge"):
        import rclpy

        self._rclpy = rclpy
        self._owns_context = not rclpy.ok()
        if self._owns_context:
            # args=[] so rclpy never tries to parse this script's own command line.
            rclpy.init(args=[])
        self.node = rclpy.create_node(node_name)
        self._publishers = {}

    def now_ns(self):
        return self.node.get_clock().now().nanoseconds

    def info(self, message):
        self.node.get_logger().info(message)

    def warn(self, message):
        self.node.get_logger().warning(message)

    def error(self, message):
        self.node.get_logger().error(message)

    def camera_publisher(self, image_topic, info_topic, frame_id):
        """Return the publisher pair for these topics, creating it once."""
        key = (image_topic, info_topic, frame_id)
        if key not in self._publishers:
            self._publishers[key] = CameraPublisher(self, image_topic, info_topic, frame_id)
        return self._publishers[key]

    def shutdown(self):
        self.node.destroy_node()
        if self._owns_context and self._rclpy.ok():
            self._rclpy.shutdown()
