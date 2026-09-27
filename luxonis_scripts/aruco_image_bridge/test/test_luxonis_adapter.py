"""ArucoFrameSink / RosOutputConfig tests with real DepthAI frames (no camera, no ROS)."""

from __future__ import annotations

from datetime import timedelta
from unittest.mock import MagicMock

import depthai as dai
import numpy as np
import pytest

pytest.importorskip("rclpy.impl.rcutils_logger", reason="ROS 2 not sourced (source /opt/ros/jazzy/setup.bash)")

from rclpy.impl.rcutils_logger import RcutilsLogger

from aruco_image_bridge.luxonis_adapter import FRAME_TYPES, ArucoFrameSink, RosOutputConfig
from aruco_image_bridge.test.fakes import IDENTITY, Clock, FakePublisher, make_img_frame, make_transformation

ROS_NOW_NS = 1_000_000_000_000
RECEIVE_NS = 42
ATTEMPTS = 3


def make_sink(
    publisher: FakePublisher,
    *,
    host_s: float = 10.05,
    ros_ns: int = ROS_NOW_NS,
    mono: Clock | None = None,
    encoding: str = "bgr8",
) -> tuple[ArucoFrameSink, MagicMock, Clock]:
    mono = mono or Clock(0.0)
    log = MagicMock(spec=RcutilsLogger)  # only RcutilsLogger's methods exist on it; records the calls
    sink = ArucoFrameSink(
        publisher,
        "FRONT",
        5.0,
        clock_now=lambda: timedelta(seconds=host_s),
        ros_now_ns=lambda: ros_ns,
        log=log,
        encoding=encoding,
        monotonic=mono,
    )
    return sink, log, mono


def test_stamp_is_capture_time_in_ros_clock():
    publisher = FakePublisher()
    sink, _, _ = make_sink(publisher, host_s=10.05)
    assert sink.handle(make_img_frame(capture_s=10.0))
    # the frame is 50 ms old on the DepthAI clock -> 50 ms before ROS "now"
    assert publisher.calls[0][2] == ROS_NOW_NS - 50_000_000


def test_implausible_age_falls_back_to_receive_time():
    publisher = FakePublisher()
    sink, _, _ = make_sink(publisher, host_s=5.0, ros_ns=RECEIVE_NS)
    sink.handle(make_img_frame(capture_s=10.0))  # "from the future"
    assert publisher.calls[0][2] == RECEIVE_NS


def test_rate_limit_drops_frames_that_arrive_too_soon():
    sink, _, mono = make_sink(FakePublisher())
    for t in (0.0, 0.05, 0.10, 0.19, 0.20, 0.25, 0.40):
        mono.value = t
        sink.handle(make_img_frame())
    # published at 0.0, 0.19 (>= 0.18 guard) and 0.40
    assert (sink.published, sink.skipped) == (3, 4)


def test_calibration_is_resolved_once_and_logged():
    publisher = FakePublisher()
    sink, log, mono = make_sink(publisher)
    for t in (0.0, 1.0, 2.0):
        mono.value = t
        sink.handle(make_img_frame())
    assert len({id(call[1]) for call in publisher.calls}) == 1
    assert log.info.call_count == 1


def test_publish_errors_are_contained_and_logged_once():
    sink, log, mono = make_sink(FakePublisher(fail_with=RuntimeError("rmw down")))
    for t in (0.0, 1.0, 2.0):
        mono.value = t
        assert not sink.handle(make_img_frame())
    assert sink.failed == ATTEMPTS
    log.error.assert_called_once()
    assert "RTSP streaming continues" in log.error.call_args.args[0]


def test_missing_calibration_is_an_error_not_a_crash():
    publisher = FakePublisher()
    sink, log, _ = make_sink(publisher)
    assert not sink.handle(make_img_frame(transformation=make_transformation(k=IDENTITY)))
    assert publisher.calls == []
    assert any("calibration" in call.args[0].lower() for call in log.error.call_args_list)


@pytest.mark.parametrize(
    ("encoding", "frame_type", "shape"),
    [
        ("bgr8", dai.ImgFrame.Type.BGR888i, (720, 1280, 3)),
        ("rgb8", dai.ImgFrame.Type.RGB888i, (720, 1280, 3)),
        ("mono8", dai.ImgFrame.Type.GRAY8, (720, 1280)),
    ],
)
def test_sink_publishes_the_configured_encoding(encoding: str, frame_type: dai.ImgFrame.Type, shape: tuple[int, ...]):
    publisher = FakePublisher()
    sink, _, _ = make_sink(publisher, encoding=encoding)
    assert sink.handle(make_img_frame(frame_type=frame_type))
    assert publisher.calls[0][0].shape == shape
    assert publisher.calls[0][3] == encoding


def test_rgb_pixels_are_published_as_the_camera_sent_them():
    # getCvFrame() would return this frame as BGR (channels swapped) under an rgb8 header
    pixels = np.zeros((720, 1280, 3), dtype=np.uint8)
    pixels[0, 0] = (1, 2, 3)
    publisher = FakePublisher()
    sink, _, _ = make_sink(publisher, encoding="rgb8")
    assert sink.handle(make_img_frame(frame_type=dai.ImgFrame.Type.RGB888i, pixels=pixels))
    assert publisher.calls[0][0][0, 0].tolist() == [1, 2, 3]


def test_a_frame_of_the_wrong_type_is_an_error_not_garbage():
    publisher = FakePublisher()
    sink, log, _ = make_sink(publisher, encoding="bgr8")
    assert not sink.handle(make_img_frame(frame_type=dai.ImgFrame.Type.GRAY8))
    assert publisher.calls == []
    log.error.assert_called_once()
    assert "GRAY8" in log.error.call_args.args[0]
    assert "BGR888i" in log.error.call_args.args[0]


def test_frame_type_follows_the_encoding():
    for encoding, frame_type in FRAME_TYPES.items():
        assert RosOutputConfig.from_args("FRONT", "1280x720", 5, encoding).frame_type == frame_type


def test_defaults_follow_rover_description_names():
    config = RosOutputConfig.from_args("front,back", "1280x720", 5)
    assert config.cameras == ("FRONT", "BACK")
    assert config.encoding == "bgr8"
    assert config.topics("FRONT") == ("/ffc/front/image_raw", "/ffc/front/camera_info", "ffc_front_camera_optical_frame")
    assert config.topics("BACK")[0] == "/ffc/rear/image_raw"


@pytest.mark.parametrize(
    ("cameras", "size", "fps", "encoding"),
    [
        ("TOP", "1280x720", 5, "bgr8"),
        ("", "1280x720", 5, "bgr8"),
        ("FRONT", "1280", 5, "bgr8"),
        ("FRONT", "0x720", 5, "bgr8"),
        ("FRONT", "1280x720", 0, "bgr8"),
        ("FRONT", "1280x720", 5, "mono16"),
    ],
)
def test_invalid_options_are_rejected(cameras: str, size: str, fps: float, encoding: str):
    with pytest.raises(ValueError, match=r"(?i)ros"):
        RosOutputConfig.from_args(cameras, size, fps, encoding)
