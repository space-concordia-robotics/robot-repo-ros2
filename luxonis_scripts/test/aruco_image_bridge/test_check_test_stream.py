"""The stream checker must accept what the synthetic publisher sends (and only that)."""

from __future__ import annotations

import pytest

pytest.importorskip("cv_bridge", reason="ROS 2 not sourced (source /opt/ros/jazzy/setup.bash)")

from aruco_image_bridge.check_test_stream import CheckFailedError, StreamCheck
from aruco_image_bridge.ros_bridge import camera_info_msg, stamp_from_ns
from aruco_image_bridge.synthetic_frames import FRAME_ID, synthetic_calibration


def test_the_checker_accepts_the_camera_info_the_publisher_sends():
    info = camera_info_msg(synthetic_calibration(), stamp_from_ns(1), FRAME_ID)
    StreamCheck.check_calibration(info)


def test_the_checker_rejects_a_different_distortion_model():
    info = camera_info_msg(synthetic_calibration(), stamp_from_ns(1), FRAME_ID)
    info.distortion_model = "rational_polynomial"
    with pytest.raises(CheckFailedError, match="distortion model"):
        StreamCheck.check_calibration(info)
