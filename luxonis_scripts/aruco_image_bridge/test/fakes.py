"""Fake DepthAI objects and ROS stand-ins shared by the tests (no camera, no ROS)."""

from __future__ import annotations

from collections.abc import Sequence
from datetime import timedelta

import numpy as np
import numpy.typing as npt

from aruco_image_bridge.calibration import CameraCalibration, Matrix3x3

# Plausible 1280x720 values for a wide IMX378 (not real rover calibration).
K: Matrix3x3 = [[700.0, 0.0, 641.2], [0.0, 700.5, 358.9], [0.0, 0.0, 1.0]]
IDENTITY: Matrix3x3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
COEFFS_14 = [-0.1, 0.02, 0.001, -0.002, 0.003, 0.2, -0.01, 0.004, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


class Model:
    """Stands in for ``dai.CameraModel``."""

    def __init__(self, name: str) -> None:
        self.name = name


# Method names below mirror DepthAI's camelCase API.
class FakeTransformation:
    """Stands in for ``dai.ImgTransformation``."""

    def __init__(
        self,
        size: tuple[int, int] = (1280, 720),
        k: Matrix3x3 = K,
        coeffs: Sequence[float] = COEFFS_14,
        model: str = "Perspective",
    ) -> None:
        self.size, self.k, self.coeffs, self.model = size, k, coeffs, Model(model)

    def getSize(self) -> tuple[int, int]:  # noqa: N802
        return self.size

    def getIntrinsicMatrix(self) -> Matrix3x3:  # noqa: N802
        return self.k

    def getDistortionCoefficients(self) -> Sequence[float]:  # noqa: N802
        return self.coeffs

    def getDistortionModel(self) -> Model:  # noqa: N802
        return self.model


class FakeFrame:
    """Stands in for ``dai.ImgFrame`` (calibration only)."""

    def __init__(self, transformation: FakeTransformation) -> None:
        self.transformation = transformation

    def getTransformation(self) -> FakeTransformation:  # noqa: N802
        return self.transformation


class FakeHandler:
    """Stands in for ``dai.CalibrationHandler``; records the requests it receives."""

    def __init__(self, k: Matrix3x3 = K, coeffs: Sequence[float] = COEFFS_14, model: str = "Perspective") -> None:
        self.k, self.coeffs, self.model = k, coeffs, Model(model)
        self.requests: list[tuple[object, int, int]] = []

    def getCameraIntrinsics(self, socket: object, width: int, height: int, /) -> Matrix3x3:  # noqa: N802
        self.requests.append((socket, width, height))
        return self.k

    def getDistortionCoefficients(self, socket: object, /) -> Sequence[float]:  # noqa: N802, ARG002
        return self.coeffs

    def getDistortionModel(self, socket: object, /) -> Model:  # noqa: N802, ARG002
        return self.model


class FakeImgFrame:
    """Stands in for a full ``dai.ImgFrame`` from ``getCvFrame()``."""

    def __init__(self, capture_s: float = 10.0, size: tuple[int, int] = (1280, 720), transformation: FakeTransformation | None = None) -> None:
        self.capture = timedelta(seconds=capture_s)
        self.size = size
        self.transformation = transformation or FakeTransformation(size=size)

    def getCvFrame(self) -> npt.NDArray[np.uint8]:  # noqa: N802
        width, height = self.size
        return np.zeros((height, width, 3), dtype=np.uint8)

    def getTimestamp(self) -> timedelta:  # noqa: N802
        return self.capture

    def getTransformation(self) -> FakeTransformation:  # noqa: N802
        return self.transformation


class FakePublisher:
    """Records what the sink publishes; can be told to fail."""

    def __init__(self, fail_with: Exception | None = None) -> None:
        self.calls: list[tuple[tuple[int, ...], CameraCalibration, int | None, str | None]] = []
        self.fail_with = fail_with

    def publish(
        self,
        frame: npt.NDArray[np.generic],
        calibration: CameraCalibration,
        stamp_ns: int | None = None,
        encoding: str | None = None,
    ) -> None:
        if self.fail_with is not None:
            raise self.fail_with
        self.calls.append((frame.shape, calibration, stamp_ns, encoding))


class FakeLog:
    """Collects log lines as (level, message)."""

    def __init__(self) -> None:
        self.lines: list[tuple[str, str]] = []

    def info(self, message: str) -> None:
        self.lines.append(("info", message))

    def warn(self, message: str) -> None:
        self.lines.append(("warn", message))

    def error(self, message: str) -> None:
        self.lines.append(("error", message))

    def count(self, level: str) -> int:
        return sum(1 for line_level, _ in self.lines if line_level == level)


class Clock:
    """A settable clock returning ``value``."""

    def __init__(self, value: float) -> None:
        self.value = value

    def __call__(self) -> float:
        return self.value
