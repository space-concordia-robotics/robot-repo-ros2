"""Real DepthAI objects built without a camera, plus small stand-ins shared by the tests."""

from __future__ import annotations

from collections.abc import Sequence
from datetime import timedelta
from typing import NamedTuple, override

import depthai as dai
import numpy as np
import numpy.typing as npt

from aruco_image_bridge.calibration import CameraCalibration, Matrix3x3
from aruco_image_bridge.luxonis_adapter import FramePublisher

SOCKET = dai.CameraBoardSocket.CAM_A
# Plausible 1280x720 values for a wide IMX378 (not real rover calibration). DepthAI stores
# float32, so the values are exactly representable in float32 and compare with ==.
K: Matrix3x3 = ((700.0, 0.0, 640.0), (0.0, 700.5, 359.5), (0.0, 0.0, 1.0))
K_1080P: Matrix3x3 = ((1050.0, 0.0, 960.0), (0.0, 1050.0, 540.0), (0.0, 0.0, 1.0))
IDENTITY: Matrix3x3 = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
COEFFS_14 = [-0.125, 0.03125, 0.001953125, -0.00390625, 0.0078125, 0.25, -0.015625, 0.0078125, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def make_transformation(
    size: tuple[int, int] = (1280, 720),
    k: Matrix3x3 = K,
    coeffs: Sequence[float] = COEFFS_14,
    model: dai.CameraModel = dai.CameraModel.Perspective,
) -> dai.ImgTransformation:
    """A calibrated transformation; ``dai.ImgTransformation(w, h)`` alone is an uncalibrated one (identity K)."""
    transformation = dai.ImgTransformation(*size)
    transformation.setIntrinsicMatrix([list(row) for row in k])
    transformation.setDistortionCoefficients(list(coeffs))
    transformation.setDistortionModel(model)
    return transformation


def make_calibration_handler(
    k: Matrix3x3 = K_1080P,
    size: tuple[int, int] = (1920, 1080),
    model: dai.CameraModel = dai.CameraModel.Perspective,
) -> dai.CalibrationHandler:
    """A device calibration for ``SOCKET`` at ``size``, like the one stored in a camera's EEPROM."""
    handler = dai.CalibrationHandler()
    width, height = size
    handler.setCameraIntrinsics(SOCKET, [list(row) for row in k], dai.Size2f(width, height))
    handler.setDistortionCoefficients(SOCKET, list(COEFFS_14))
    handler.setCameraType(SOCKET, model)
    return handler


def make_img_frame(
    capture_s: float = 10.0,
    size: tuple[int, int] = (1280, 720),
    transformation: dai.ImgTransformation | None = None,
    frame_type: dai.ImgFrame.Type = dai.ImgFrame.Type.BGR888i,
    pixels: npt.NDArray[np.uint8] | None = None,
) -> dai.ImgFrame:
    """A frame with a transformation and capture timestamp, like a camera output (zeros unless ``pixels``)."""
    width, height = size
    if pixels is None:
        shape = (height, width) if frame_type == dai.ImgFrame.Type.GRAY8 else (height, width, 3)
        pixels = np.zeros(shape, dtype=np.uint8)
    frame = dai.ImgFrame()
    frame.setFrame(pixels)  # raw data, exactly as a camera would send it
    frame.setType(frame_type)
    frame.setWidth(width)
    frame.setHeight(height)
    frame.setTransformation(transformation or make_transformation(size))
    frame.setTimestamp(timedelta(seconds=capture_s))
    return frame


class Published(NamedTuple):
    """One call to ``FakePublisher.publish``."""

    frame: npt.NDArray[np.generic]
    calibration: CameraCalibration
    encoding: str
    stamp_ns: int | None


class FakePublisher(FramePublisher):
    """Records what the sink publishes; can be told to fail."""

    def __init__(self, fail_with: Exception | None = None) -> None:
        self.calls: list[Published] = []
        self.fail_with = fail_with

    @override
    def publish(
        self,
        frame: npt.NDArray[np.generic],
        calibration: CameraCalibration,
        encoding: str,
        stamp_ns: int | None = None,
    ) -> None:
        if self.fail_with is not None:
            raise self.fail_with
        self.calls.append(Published(frame, calibration, encoding, stamp_ns))


class Clock:
    """A settable clock returning ``value``."""

    def __init__(self, value: float) -> None:
        self.value = value

    def __call__(self) -> float:
        return self.value
