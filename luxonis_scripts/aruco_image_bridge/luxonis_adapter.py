"""
Glue between main.py's DepthAI pipeline and the ROS bridge.

No ROS or DepthAI imports here: DepthAI frames, clocks and calibration are
passed in, so this logic is unit-tested with fakes (see test/test_luxonis_adapter.py).
"""

from __future__ import annotations

import time
from collections.abc import Callable
from dataclasses import dataclass
from datetime import timedelta
from typing import TYPE_CHECKING, Protocol

import cv2
import numpy as np
import numpy.typing as npt

from . import calibration as calib

if TYPE_CHECKING:
    from .ros_bridge import RosBridge

# Topic prefixes and frame names follow rover-description's simulated FFC cameras
# (urdf/sensors/ffc-module.urdf: topics ffc/<side>/image_raw and ffc/<side>/camera_info,
# links ffc_<side>_camera, where the back camera is called "rear").
# The *_optical_frame names are PLACEHOLDERS until the URDF gains optical frames;
# see "Open questions" in README.md.
CAMERA_DEFAULTS: dict[str, tuple[str, str]] = {
    "FRONT": ("/ffc/front", "ffc_front_camera_optical_frame"),
    "RIGHT": ("/ffc/right", "ffc_right_camera_optical_frame"),
    "LEFT": ("/ffc/left", "ffc_left_camera_optical_frame"),
    "BACK": ("/ffc/rear", "ffc_rear_camera_optical_frame"),
    "RGB": ("/oakd/rgb", "oakd_rgb_camera_optical_frame"),
}

# Encodings the camera path can publish; all three are accepted by the ArUco tracker.
# mono8 is a third of the size of bgr8 and ArUco detection works on grayscale anyway.
OUTPUT_ENCODINGS = ("bgr8", "rgb8", "mono8")

_CONVERSIONS: dict[tuple[int, str], int] = {
    (3, "rgb8"): cv2.COLOR_BGR2RGB,
    (3, "mono8"): cv2.COLOR_BGR2GRAY,
    (1, "bgr8"): cv2.COLOR_GRAY2BGR,
    (1, "rgb8"): cv2.COLOR_GRAY2RGB,
}

MAX_FPS = 30.0
_RATE_TOLERANCE = 0.9  # tolerates normal frame-time jitter in the host-side rate guard
_MAX_FRAME_AGE_S = 1.0
_NS_PER_S = 1_000_000_000


class DepthAIFrame(calib.TransformedFrame, Protocol):
    """The parts of ``dai.ImgFrame`` the sink uses."""

    def getCvFrame(self) -> npt.NDArray[np.uint8]: ...  # noqa: N802, D102
    def getTimestamp(self) -> timedelta: ...  # noqa: N802, D102


class FramePublisher(Protocol):
    """Where the sink sends frames; ``ros_bridge.CameraPublisher`` in production."""

    def publish(  # noqa: D102
        self,
        frame: npt.NDArray[np.generic],
        calibration: calib.CameraCalibration,
        stamp_ns: int | None = None,
        encoding: str | None = None,
    ) -> None: ...


class Log(Protocol):
    """Logging interface; ``ros_bridge.RosBridge`` in production."""

    def info(self, message: str) -> None: ...  # noqa: D102
    def warn(self, message: str) -> None: ...  # noqa: D102
    def error(self, message: str) -> None: ...  # noqa: D102


def convert_bgr_frame(frame: npt.NDArray[np.uint8], encoding: str) -> npt.NDArray[np.uint8]:
    """Convert a ``getCvFrame()`` result (BGR, or 2-D grayscale for mono sensors) to ``encoding``."""
    if encoding not in OUTPUT_ENCODINGS:
        raise ValueError(f"Unsupported output encoding '{encoding}'; choose from {OUTPUT_ENCODINGS}")
    channels = 1 if frame.ndim == 2 else frame.shape[2]  # noqa: PLR2004
    code = _CONVERSIONS.get((channels, encoding))
    if code is None:
        return frame
    return np.asarray(cv2.cvtColor(frame, code), dtype=np.uint8)


@dataclass(frozen=True)
class RosOutputConfig:
    """Validated ``--ros*`` command-line options."""

    cameras: tuple[str, ...]
    width: int = 1280
    height: int = 720
    fps: float = 5.0
    encoding: str = "bgr8"

    @property
    def size(self) -> tuple[int, int]:
        """(width, height), as DepthAI's requestOutput expects."""
        return (self.width, self.height)

    @classmethod
    def from_args(cls, cameras_csv: str, size_text: str, fps: float, encoding: str = "bgr8") -> RosOutputConfig:
        """Parse and validate the command-line values; raises ValueError with a readable message."""
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
        if encoding not in OUTPUT_ENCODINGS:
            raise ValueError(f"--ros-encoding must be one of {OUTPUT_ENCODINGS}, got '{encoding}'")
        return cls(names, width, height, float(fps), encoding)

    def topics(self, camera: str) -> tuple[str, str, str]:
        """(image topic, camera_info topic, frame_id) for ``camera``."""
        prefix, frame_id = CAMERA_DEFAULTS[camera]
        return f"{prefix}/image_raw", f"{prefix}/camera_info", frame_id


class ArucoFrameSink:
    """
    Receives DepthAI ImgFrames for one camera and publishes them to ROS.

    Every failure is caught and logged here (once per distinct message) so a
    ROS problem can never stop the RTSP streams running in the same loop.
    """

    camera: str
    encoding: str
    published: int
    skipped: int
    failed: int
    _publisher: FramePublisher
    _min_period: float
    _clock_now: Callable[[], timedelta]
    _ros_now_ns: Callable[[], int]
    _log: Log
    _read_calibration: Callable[[], calib.CalibrationHandlerLike] | None
    _socket: object
    _monotonic: Callable[[], float]
    _last_publish: float | None
    _calibration: calib.CameraCalibration | None
    _reported_errors: set[str]

    def __init__(  # noqa: PLR0913 - explicit dependencies keep this testable without hardware
        self,
        publisher: FramePublisher,
        camera: str,
        fps: float,
        clock_now: Callable[[], timedelta],
        ros_now_ns: Callable[[], int],
        log: Log,
        *,
        encoding: str = "bgr8",
        read_calibration: Callable[[], calib.CalibrationHandlerLike] | None = None,
        socket: object = None,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        """Store the dependencies; nothing is published until ``handle()``."""
        self._publisher = publisher
        self.camera = camera
        self.encoding = encoding
        # The device already limits the rate; this host-side guard only matters if a
        # firmware ignores the per-output fps.
        self._min_period = _RATE_TOLERANCE / fps
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

    def handle(self, img_frame: DepthAIFrame) -> bool:
        """Publish ``img_frame`` unless it arrives too soon. Returns True if published."""
        now = self._monotonic()
        if self._last_publish is not None and now - self._last_publish < self._min_period:
            self.skipped += 1
            return False
        try:
            frame = convert_bgr_frame(img_frame.getCvFrame(), self.encoding)
            height, width = frame.shape[:2]
            calibration = self._calibration_for(img_frame, width, height)
            self._publisher.publish(frame, calibration, self._capture_stamp_ns(img_frame), self.encoding)
        except Exception as error:  # noqa: BLE001 - never let ROS output stop the RTSP loop; reported below
            self.failed += 1
            self._report(error)
            return False
        self._last_publish = now
        self.published += 1
        return True

    def _calibration_for(self, img_frame: DepthAIFrame, width: int, height: int) -> calib.CameraCalibration:
        cached = self._calibration
        if cached is None or (cached.width, cached.height) != (width, height):
            cached = calib.resolve(img_frame, width, height, self._read_calibration, self._socket)
            self._log.info(f"{self.camera} calibration: {cached.summary()}")
            for warning in cached.warnings:
                self._log.warn(f"{self.camera} calibration: {warning}")
            self._calibration = cached
        return cached

    def _capture_stamp_ns(self, img_frame: DepthAIFrame) -> int:
        """
        ROS time at which the frame was captured.

        DepthAI stamps frames on the host's steady clock (dai.Clock). The frame's
        age on that clock is subtracted from ROS "now"; implausible ages fall back
        to the receive time instead of producing a wrong stamp.
        """
        ros_now = self._ros_now_ns()
        try:
            age = (self._clock_now() - img_frame.getTimestamp()).total_seconds()
        except Exception:  # noqa: BLE001 - a bad timestamp must not drop the frame
            return ros_now
        if 0.0 <= age < _MAX_FRAME_AGE_S:
            return ros_now - int(age * _NS_PER_S)
        return ros_now

    def _report(self, error: Exception) -> None:
        message = f"{self.camera}: {error}"
        if message not in self._reported_errors:
            self._reported_errors.add(message)
            self._log.error(f"ROS output failed ({message}); RTSP streaming continues")


class RosOutput:
    """What main.py holds when started with --ros: the bridge plus its settings."""

    bridge: RosBridge
    config: RosOutputConfig
    _clock_now: Callable[[], timedelta]

    def __init__(self, bridge: RosBridge, config: RosOutputConfig, clock_now: Callable[[], timedelta]) -> None:
        """``clock_now`` is ``dai.Clock.now`` in production."""
        self.bridge = bridge
        self.config = config
        self._clock_now = clock_now

    def sink(self, camera: str, socket: object, read_calibration: Callable[[], calib.CalibrationHandlerLike]) -> ArucoFrameSink:
        """Create the publisher pair and the sink for one camera."""
        image_topic, info_topic, frame_id = self.config.topics(camera)
        publisher = self.bridge.camera_publisher(image_topic, info_topic, frame_id)
        self.bridge.info(
            f"ROS output {camera}: {self.config.width}x{self.config.height} {self.config.encoding} @ {self.config.fps:g} FPS -> "
            f"{image_topic}, {info_topic} (frame_id {frame_id})",
        )
        return ArucoFrameSink(
            publisher,
            camera,
            self.config.fps,
            clock_now=self._clock_now,
            ros_now_ns=self.bridge.now_ns,
            log=self.bridge,
            encoding=self.config.encoding,
            read_calibration=read_calibration,
            socket=socket,
        )

    def shutdown(self) -> None:
        """Shut the ROS bridge down."""
        self.bridge.shutdown()
