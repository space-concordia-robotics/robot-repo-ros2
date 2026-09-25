"""Glue between main.py's DepthAI pipeline and the ROS bridge.

No ROS or DepthAI imports here: DepthAI frames, clocks and calibration are
passed in, so this logic is unit-tested with fakes (see test_luxonis_adapter.py).
"""

import time
from dataclasses import dataclass

from . import calibration as calib

# Topic prefixes and frame names follow rover-description's simulated FFC cameras
# (urdf/sensors/ffc-module.urdf: topics ffc/<side>/image_raw and ffc/<side>/camera_info,
# links ffc_<side>_camera, where the back camera is called "rear").
# The *_optical_frame names are PLACEHOLDERS until the URDF gains optical frames;
# see "Open questions" in README.md.
CAMERA_DEFAULTS = {
    "FRONT": ("/ffc/front", "ffc_front_camera_optical_frame"),
    "RIGHT": ("/ffc/right", "ffc_right_camera_optical_frame"),
    "LEFT": ("/ffc/left", "ffc_left_camera_optical_frame"),
    "BACK": ("/ffc/rear", "ffc_rear_camera_optical_frame"),
    "RGB": ("/oakd/rgb", "oakd_rgb_camera_optical_frame"),
}

MAX_FPS = 30.0


@dataclass(frozen=True)
class RosOutputConfig:
    cameras: tuple
    width: int = 1280
    height: int = 720
    fps: float = 5.0

    @property
    def size(self):
        return (self.width, self.height)

    @classmethod
    def from_args(cls, cameras_csv, size_text, fps):
        names = tuple(dict.fromkeys(name.strip().upper() for name in cameras_csv.split(",") if name.strip()))
        if not names:
            raise ValueError("--ros-cameras needs at least one camera")
        unknown = [name for name in names if name not in CAMERA_DEFAULTS]
        if unknown:
            raise ValueError(f"Unknown ROS camera(s) {unknown}; choose from {list(CAMERA_DEFAULTS)}")
        try:
            width, height = (int(part) for part in size_text.lower().split("x"))
        except ValueError:
            raise ValueError(f"--ros-size must look like 1280x720, got '{size_text}'") from None
        if width <= 0 or height <= 0:
            raise ValueError(f"--ros-size must be positive, got '{size_text}'")
        if not 0.0 < fps <= MAX_FPS:
            raise ValueError(f"--ros-fps must be in (0, {MAX_FPS}], got {fps}")
        return cls(names, width, height, float(fps))

    def topics(self, camera):
        prefix, frame_id = CAMERA_DEFAULTS[camera]
        return f"{prefix}/image_raw", f"{prefix}/camera_info", frame_id


class ArucoFrameSink:
    """Receives DepthAI ImgFrames for one camera and publishes them to ROS.

    Every failure is caught and logged here (once per distinct message) so a
    ROS problem can never stop the RTSP streams running in the same loop.
    """

    def __init__(self, publisher, camera, fps, clock_now, ros_now_ns, log, read_calibration=None, socket=None, monotonic=time.monotonic):
        self._publisher = publisher
        self.camera = camera
        # The device already limits the rate; this host-side guard only matters if a
        # firmware ignores the per-output fps. 0.9 tolerates normal frame-time jitter.
        self._min_period = 0.9 / fps
        self._clock_now = clock_now
        self._ros_now_ns = ros_now_ns
        self._log = log
        self._read_calibration = read_calibration
        self._socket = socket
        self._monotonic = monotonic
        self._last_publish = None
        self._calibration = None
        self._reported_errors = set()
        self.published = 0
        self.skipped = 0
        self.failed = 0

    def handle(self, img_frame):
        """Publish ``img_frame`` unless it arrives too soon. Returns True if published."""
        now = self._monotonic()
        if self._last_publish is not None and now - self._last_publish < self._min_period:
            self.skipped += 1
            return False
        try:
            bgr = img_frame.getCvFrame()
            height, width = bgr.shape[:2]
            calibration = self._calibration_for(img_frame, width, height)
            self._publisher.publish(bgr, calibration, self._capture_stamp_ns(img_frame))
        except Exception as error:
            self.failed += 1
            self._report(error)
            return False
        self._last_publish = now
        self.published += 1
        return True

    def _calibration_for(self, img_frame, width, height):
        cached = self._calibration
        if cached is None or (cached.width, cached.height) != (width, height):
            cached = calib.resolve(img_frame, width, height, self._read_calibration, self._socket)
            self._log.info(f"{self.camera} calibration: {cached.summary()}")
            for warning in cached.warnings:
                self._log.warn(f"{self.camera} calibration: {warning}")
            self._calibration = cached
        return cached

    def _capture_stamp_ns(self, img_frame):
        """ROS time at which the frame was captured.

        DepthAI stamps frames on the host's steady clock (dai.Clock). The frame's
        age on that clock is subtracted from ROS "now"; implausible ages fall back
        to the receive time instead of producing a wrong stamp.
        """
        ros_now = self._ros_now_ns()
        try:
            age = (self._clock_now() - img_frame.getTimestamp()).total_seconds()
        except Exception:
            return ros_now
        if 0.0 <= age < 1.0:
            return ros_now - int(age * 1e9)
        return ros_now

    def _report(self, error):
        message = f"{self.camera}: {error}"
        if message not in self._reported_errors:
            self._reported_errors.add(message)
            self._log.error(f"ROS output failed ({message}); RTSP streaming continues")


class RosOutput:
    """What main.py holds when started with --ros: the bridge plus its settings."""

    def __init__(self, bridge, config, clock_now):
        self.bridge = bridge
        self.config = config
        self._clock_now = clock_now

    def sink(self, camera, socket, read_calibration):
        image_topic, info_topic, frame_id = self.config.topics(camera)
        publisher = self.bridge.camera_publisher(image_topic, info_topic, frame_id)
        self.bridge.info(
            f"ROS output {camera}: {self.config.width}x{self.config.height} @ {self.config.fps:g} FPS -> {image_topic}, {info_topic} (frame_id {frame_id})"
        )
        return ArucoFrameSink(
            publisher,
            camera,
            self.config.fps,
            clock_now=self._clock_now,
            ros_now_ns=self.bridge.now_ns,
            log=self.bridge,
            read_calibration=read_calibration,
            socket=socket,
        )

    def shutdown(self):
        self.bridge.shutdown()
