"""ArucoFrameSink / RosOutputConfig tests with fakes (no camera, no ROS)."""

import datetime
import unittest

import numpy as np

from aruco_image_bridge.luxonis_adapter import ArucoFrameSink, RosOutputConfig
from aruco_image_bridge.test_calibration import FakeTransformation


class FakeImgFrame:
    def __init__(self, capture_s=10.0, size=(1280, 720)):
        self.capture = datetime.timedelta(seconds=capture_s)
        self.size = size

    def getCvFrame(self):
        width, height = self.size
        return np.zeros((height, width, 3), dtype=np.uint8)

    def getTimestamp(self):
        return self.capture

    def getTransformation(self):
        return FakeTransformation(size=self.size)


class FakePublisher:
    def __init__(self, fail_with=None):
        self.calls = []
        self.fail_with = fail_with

    def publish(self, frame, calibration, stamp_ns):
        if self.fail_with:
            raise self.fail_with
        self.calls.append((frame.shape, calibration, stamp_ns))


class FakeLog:
    def __init__(self):
        self.lines = []

    def info(self, message):
        self.lines.append(("info", message))

    def warn(self, message):
        self.lines.append(("warn", message))

    def error(self, message):
        self.lines.append(("error", message))


class Clock:
    def __init__(self, value):
        self.value = value

    def __call__(self):
        return self.value


def make_sink(publisher=None, host_s=10.05, ros_ns=1_000_000_000_000, mono=None, fps=5.0):
    mono = mono or Clock(0.0)
    log = FakeLog()
    sink = ArucoFrameSink(
        publisher or FakePublisher(),
        "FRONT",
        fps,
        clock_now=lambda: datetime.timedelta(seconds=host_s),
        ros_now_ns=lambda: ros_ns,
        log=log,
        monotonic=mono,
    )
    return sink, log, mono


class ArucoFrameSinkTest(unittest.TestCase):
    def test_stamp_is_capture_time_in_ros_clock(self):
        publisher = FakePublisher()
        sink, _, _ = make_sink(publisher, host_s=10.05, ros_ns=1_000_000_000_000)
        self.assertTrue(sink.handle(FakeImgFrame(capture_s=10.0)))
        # frame is 50 ms old on the DepthAI clock -> 50 ms before ROS "now"
        self.assertEqual(publisher.calls[0][2], 1_000_000_000_000 - 50_000_000)

    def test_implausible_age_falls_back_to_receive_time(self):
        publisher = FakePublisher()
        sink, _, _ = make_sink(publisher, host_s=5.0, ros_ns=42)
        sink.handle(FakeImgFrame(capture_s=10.0))  # "from the future"
        self.assertEqual(publisher.calls[0][2], 42)

    def test_rate_limit_drops_frames_that_arrive_too_soon(self):
        publisher = FakePublisher()
        mono = Clock(0.0)
        sink, _, _ = make_sink(publisher, mono=mono, fps=5.0)
        for t in (0.0, 0.05, 0.10, 0.19, 0.20, 0.25, 0.40):
            mono.value = t
            sink.handle(FakeImgFrame())
        # published at 0.0, 0.19 (>= 0.18 guard), 0.40
        self.assertEqual(sink.published, 3)
        self.assertEqual(sink.skipped, 4)

    def test_calibration_is_resolved_once_and_logged(self):
        publisher = FakePublisher()
        mono = Clock(0.0)
        sink, log, _ = make_sink(publisher, mono=mono)
        for t in (0.0, 1.0, 2.0):
            mono.value = t
            sink.handle(FakeImgFrame())
        calibrations = {id(call[1]) for call in publisher.calls}
        self.assertEqual(len(calibrations), 1)
        self.assertEqual(sum(1 for level, _ in log.lines if level == "info"), 1)

    def test_publish_errors_are_contained_and_logged_once(self):
        sink, log, mono = make_sink(FakePublisher(fail_with=RuntimeError("rmw down")))
        for t in (0.0, 1.0, 2.0):
            mono.value = t
            self.assertFalse(sink.handle(FakeImgFrame()))
        self.assertEqual(sink.failed, 3)
        errors = [message for level, message in log.lines if level == "error"]
        self.assertEqual(len(errors), 1)
        self.assertIn("RTSP streaming continues", errors[0])

    def test_missing_calibration_is_an_error_not_a_crash(self):
        class Uncalibrated(FakeImgFrame):
            def getTransformation(self):
                return FakeTransformation(k=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])

        publisher = FakePublisher()
        sink, log, _ = make_sink(publisher)
        self.assertFalse(sink.handle(Uncalibrated()))
        self.assertEqual(publisher.calls, [])
        self.assertTrue(any("calibration" in message.lower() for level, message in log.lines if level == "error"))


class RosOutputConfigTest(unittest.TestCase):
    def test_defaults_follow_rover_description_names(self):
        config = RosOutputConfig.from_args("front,back", "1280x720", 5)
        self.assertEqual(config.cameras, ("FRONT", "BACK"))
        self.assertEqual(config.topics("FRONT"), ("/ffc/front/image_raw", "/ffc/front/camera_info", "ffc_front_camera_optical_frame"))
        self.assertEqual(config.topics("BACK")[0], "/ffc/rear/image_raw")

    def test_invalid_options_are_rejected(self):
        for cameras, size, fps in (("TOP", "1280x720", 5), ("", "1280x720", 5), ("FRONT", "1280", 5), ("FRONT", "0x720", 5), ("FRONT", "1280x720", 0)):
            with self.subTest(cameras=cameras, size=size, fps=fps):
                with self.assertRaises(ValueError):
                    RosOutputConfig.from_args(cameras, size, fps)


if __name__ == "__main__":
    unittest.main(verbosity=2)
