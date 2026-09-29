"""Message-building tests. Need ROS 2 sourced (sensor_msgs), but no running nodes."""

from __future__ import annotations

from typing import Any

import numpy as np
import numpy.typing as npt
import pytest

pytest.importorskip("sensor_msgs.msg", reason="ROS 2 not sourced (source /opt/ros/jazzy/setup.bash)")

from aruco_image_bridge.ros_bridge import camera_info_msg, image_msg, stamp_from_ns
from aruco_image_bridge.synthetic_frames import synthetic_calibration

STAMP = stamp_from_ns(1_790_321_475_149_884_462)
FRAME_ID = "cam_optical_frame"
WIDTH = 1280


def test_stamp_conversion():
    assert (STAMP.sec, STAMP.nanosec) == (1_790_321_475, 149_884_462)


def test_bgr_image_layout_round_trips():
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)
    frame[10, 20] = (1, 2, 3)
    msg = image_msg(frame, STAMP, FRAME_ID, "bgr8")
    assert (msg.width, msg.height, msg.encoding, msg.step, msg.is_bigendian) == (1280, 720, "bgr8", 3840, 0)
    restored = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(720, 1280, 3)
    assert np.array_equal(restored, frame)
    assert msg.header.frame_id == FRAME_ID


@pytest.mark.parametrize(
    ("encoding", "dtype", "shape", "step"),
    [
        ("mono8", np.uint8, (4, 6), 6),
        ("mono16", np.uint16, (4, 6), 12),
        ("bgr8", np.uint8, (4, 6, 3), 18),
        ("rgb8", np.uint8, (4, 6, 3), 18),
        ("bgra8", np.uint8, (4, 6, 4), 24),
        ("rgba8", np.uint8, (4, 6, 4), 24),
    ],
)
def test_every_supported_encoding(encoding: str, dtype: type[np.unsignedinteger[Any]], shape: tuple[int, ...], step: int):
    frame = np.arange(np.prod(shape), dtype=dtype).reshape(shape)
    msg = image_msg(frame, STAMP, FRAME_ID, encoding)
    assert (msg.encoding, msg.step, msg.is_bigendian) == (encoding, step, 0)
    restored = np.frombuffer(bytes(msg.data), dtype=np.dtype(dtype).newbyteorder("<")).reshape(shape)
    assert np.array_equal(restored, frame)


def test_big_endian_input_is_published_little_endian():
    frame = np.array([[1, 256]], dtype=">u2")
    msg = image_msg(frame, STAMP, FRAME_ID, "mono16")
    assert bytes(msg.data) == b"\x01\x00\x00\x01"


def test_non_contiguous_frames_are_accepted():
    wide = np.zeros((720, 2560, 3), dtype=np.uint8)
    assert image_msg(wide[:, ::2], STAMP, FRAME_ID, "bgr8").width == WIDTH


@pytest.mark.parametrize(
    ("frame", "encoding"),
    [
        (np.zeros((4, 6, 3), np.uint8), "mono8"),  # channel count does not match
        (np.zeros((4, 6), np.uint8), "mono16"),  # dtype does not match
        (np.zeros((4, 6, 3), np.uint8), "yuv422"),  # unsupported encoding
    ],
)
def test_mismatched_frames_are_rejected(frame: npt.NDArray[np.generic], encoding: str):
    with pytest.raises(ValueError, match="ncoding"):
        image_msg(frame, STAMP, FRAME_ID, encoding)


def test_empty_frame_id_is_rejected():
    with pytest.raises(ValueError, match="frame_id"):
        image_msg(np.zeros((4, 6, 3), np.uint8), STAMP, "", "bgr8")


def test_camera_info_matches_calibration_and_header():
    cal = synthetic_calibration()
    info = camera_info_msg(cal, STAMP, FRAME_ID)
    image = image_msg(np.zeros((720, 1280, 3), dtype=np.uint8), STAMP, FRAME_ID, "bgr8")
    assert info.header == image.header
    assert (info.width, info.height, info.distortion_model) == (1280, 720, "plumb_bob")
    assert list(info.k) == cal.k
    assert list(info.p) == cal.p
