"""Message-building tests. Need ROS 2 sourced (sensor_msgs, cv_bridge), but no running nodes."""

from __future__ import annotations

import numpy as np
import pytest

pytest.importorskip("cv_bridge", reason="ROS 2 not sourced (source /opt/ros/jazzy/setup.bash)")

from aruco_image_bridge.ros_bridge import camera_info_msg, image_msg, stamp_from_ns
from aruco_image_bridge.synthetic_frames import synthetic_calibration
from cv_bridge import CvBridgeError
from std_msgs.msg import Header

STAMP = stamp_from_ns(1_790_321_475_149_884_462)
FRAME_ID = "cam_optical_frame"
HEADER = Header(stamp=STAMP, frame_id=FRAME_ID)


def test_stamp_conversion():
    assert (STAMP.sec, STAMP.nanosec) == (1_790_321_475, 149_884_462)


@pytest.mark.parametrize(("encoding", "shape"), [("bgr8", (4, 6, 3)), ("rgb8", (4, 6, 3)), ("mono8", (4, 6))])
def test_pixels_and_header_are_published_as_given(encoding: str, shape: tuple[int, ...]):
    frame = np.arange(np.prod(shape), dtype=np.uint8).reshape(shape)
    msg = image_msg(frame, HEADER, encoding)
    assert (msg.encoding, msg.width, msg.height) == (encoding, 6, 4)
    assert msg.header == HEADER
    # the bytes are the array's: an rgb8 frame is labelled rgb8, not converted to bgr
    assert np.array_equal(np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(shape), frame)


def test_encoding_must_match_the_array():
    with pytest.raises(CvBridgeError):
        image_msg(np.zeros((4, 6, 3), np.uint8), HEADER, "mono8")


def test_camera_info_matches_calibration_and_header():
    cal = synthetic_calibration()
    info = camera_info_msg(cal, STAMP, FRAME_ID)
    image = image_msg(np.zeros((720, 1280, 3), dtype=np.uint8), HEADER, "bgr8")
    assert info.header == image.header
    assert (info.width, info.height, info.distortion_model) == (1280, 720, "plumb_bob")
    assert tuple(info.k) == cal.k
    assert tuple(info.p) == cal.p
