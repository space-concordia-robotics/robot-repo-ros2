"""ArucoFrameSink / RosOutputConfig tests with fakes (no camera, no ROS)."""

from __future__ import annotations

from datetime import timedelta

import numpy as np
import pytest

from aruco_image_bridge.luxonis_adapter import ArucoFrameSink, RosOutputConfig, convert_bgr_frame
from aruco_image_bridge.test.fakes import IDENTITY, Clock, FakeImgFrame, FakeLog, FakePublisher, FakeTransformation

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
) -> tuple[ArucoFrameSink, FakeLog, Clock]:
    mono = mono or Clock(0.0)
    log = FakeLog()
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
    assert sink.handle(FakeImgFrame(capture_s=10.0))
    # the frame is 50 ms old on the DepthAI clock -> 50 ms before ROS "now"
    assert publisher.calls[0][2] == ROS_NOW_NS - 50_000_000


def test_implausible_age_falls_back_to_receive_time():
    publisher = FakePublisher()
    sink, _, _ = make_sink(publisher, host_s=5.0, ros_ns=RECEIVE_NS)
    sink.handle(FakeImgFrame(capture_s=10.0))  # "from the future"
    assert publisher.calls[0][2] == RECEIVE_NS


def test_rate_limit_drops_frames_that_arrive_too_soon():
    sink, _, mono = make_sink(FakePublisher())
    for t in (0.0, 0.05, 0.10, 0.19, 0.20, 0.25, 0.40):
        mono.value = t
        sink.handle(FakeImgFrame())
    # published at 0.0, 0.19 (>= 0.18 guard) and 0.40
    assert (sink.published, sink.skipped) == (3, 4)


def test_calibration_is_resolved_once_and_logged():
    publisher = FakePublisher()
    sink, log, mono = make_sink(publisher)
    for t in (0.0, 1.0, 2.0):
        mono.value = t
        sink.handle(FakeImgFrame())
    assert len({id(call[1]) for call in publisher.calls}) == 1
    assert log.count("info") == 1


def test_publish_errors_are_contained_and_logged_once():
    sink, log, mono = make_sink(FakePublisher(fail_with=RuntimeError("rmw down")))
    for t in (0.0, 1.0, 2.0):
        mono.value = t
        assert not sink.handle(FakeImgFrame())
    assert sink.failed == ATTEMPTS
    errors = [message for level, message in log.lines if level == "error"]
    assert len(errors) == 1
    assert "RTSP streaming continues" in errors[0]


def test_missing_calibration_is_an_error_not_a_crash():
    publisher = FakePublisher()
    sink, log, _ = make_sink(publisher)
    assert not sink.handle(FakeImgFrame(transformation=FakeTransformation(k=IDENTITY)))
    assert publisher.calls == []
    assert any("calibration" in message.lower() for level, message in log.lines if level == "error")


@pytest.mark.parametrize(("encoding", "shape"), [("bgr8", (720, 1280, 3)), ("rgb8", (720, 1280, 3)), ("mono8", (720, 1280))])
def test_sink_publishes_the_configured_encoding(encoding: str, shape: tuple[int, ...]):
    publisher = FakePublisher()
    sink, _, _ = make_sink(publisher, encoding=encoding)
    assert sink.handle(FakeImgFrame())
    assert publisher.calls[0][0] == shape
    assert publisher.calls[0][3] == encoding


def test_convert_bgr_frame():
    bgr = np.zeros((2, 2, 3), dtype=np.uint8)
    bgr[..., 0] = 255  # pure blue
    assert convert_bgr_frame(bgr, "bgr8") is bgr
    assert convert_bgr_frame(bgr, "rgb8")[0, 0].tolist() == [0, 0, 255]
    assert convert_bgr_frame(bgr, "mono8").shape == (2, 2)
    gray = np.full((2, 2), 128, dtype=np.uint8)
    assert convert_bgr_frame(gray, "mono8") is gray
    assert convert_bgr_frame(gray, "bgr8").shape == (2, 2, 3)
    with pytest.raises(ValueError, match="encoding"):
        convert_bgr_frame(bgr, "yuv422")


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
