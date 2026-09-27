"""
Turn DepthAI calibration data into ROS CameraInfo fields.

Two calibration sources are supported, in order of preference:

1. ``ImgFrame.getTransformation()`` (DepthAI v3). Its intrinsic matrix already
   includes the crop and scaling the camera applied to produce this exact frame.
2. ``CalibrationHandler.getCameraIntrinsics(socket, width, height)`` read from
   the device EEPROM, used when the frame carries no usable calibration.

A frame without real calibration still reports a "valid" transformation with
an identity intrinsic matrix, so every result is sanity-checked before use.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field

import depthai as dai

RATIONAL_POLYNOMIAL = "rational_polynomial"
PLUMB_BOB = "plumb_bob"

# DepthAI "Perspective" coefficients use OpenCV's order:
# k1 k2 p1 p2 k3 k4 k5 k6 s1 s2 s3 s4 tauX tauY
_ROS_COEFF_COUNT = 8
_PLUMB_BOB_COEFF_COUNT = 5
_TAIL_TOLERANCE = 1e-6
_ZERO_TOLERANCE = 1e-9
_MIN_FOCAL_PX = 10.0

IDENTITY_3X3 = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)

type _Matrix3x3Row = tuple[float, float, float]
type Matrix3x3 = tuple[_Matrix3x3Row, _Matrix3x3Row, _Matrix3x3Row]


class CalibrationError(ValueError):
    """Calibration is missing, implausible or in an unsupported model."""


@dataclass
class CameraCalibration:
    """Everything a CameraInfo message needs, for one image size."""

    width: int
    height: int
    distortion_model: str
    d: list[float]
    k: list[float]
    r: list[float] = field(default_factory=lambda: list(IDENTITY_3X3))
    p: list[float] = field(default_factory=list)
    source: str = "unknown"
    warnings: list[str] = field(default_factory=list)

    def __post_init__(self) -> None:
        """Derive the projection matrix [K | 0] when none is given."""
        if not self.p:
            fx, _, cx, _, fy, cy, _, _, _ = self.k
            self.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

    def summary(self) -> str:
        """One-line description for logs."""
        fx, _, cx, _, fy, cy, _, _, _ = self.k
        return f"{self.width}x{self.height} fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f} model={self.distortion_model} source={self.source}"


def to_matrix3x3(rows: Sequence[Sequence[float]]) -> Matrix3x3:
    """Convert DepthAI's nested lists to a fixed-size matrix, checking the shape."""
    try:
        (a, b, c), (d, e, f), (g, h, i) = rows
    except ValueError as error:
        raise CalibrationError("Intrinsic matrix must be 3x3") from error
    return ((float(a), float(b), float(c)), (float(d), float(e), float(f)), (float(g), float(h), float(i)))


def _check_intrinsics(k: list[float], width: int, height: int) -> list[str]:
    """Raise on implausible intrinsics; return warnings for odd but usable ones."""
    if not all(math.isfinite(value) for value in k):
        raise CalibrationError("Intrinsic matrix contains NaN or infinity")
    fx, skew, cx, zero1, fy, cy, zero2, zero3, one = k
    if fx < _MIN_FOCAL_PX or fy < _MIN_FOCAL_PX:
        raise CalibrationError(f"Implausible focal length fx={fx}, fy={fy}; the camera is probably uncalibrated")
    if not (0.0 < cx < width and 0.0 < cy < height):
        raise CalibrationError(f"Principal point ({cx}, {cy}) lies outside the {width}x{height} image")
    if any(abs(value) > _ZERO_TOLERANCE for value in (zero1, zero2, zero3)) or abs(one - 1.0) > _ZERO_TOLERANCE:
        raise CalibrationError("Intrinsic matrix does not have the expected [.., .., ..; 0 .. ..; 0 0 1] form")
    if abs(skew) > _ZERO_TOLERANCE:
        return [f"Non-zero skew {skew} is not representable in most ROS tools"]
    return []


def _ros_distortion(coefficients: Sequence[float]) -> tuple[str, list[float], list[str]]:
    """Map DepthAI Perspective coefficients to a ROS distortion model and D vector."""
    coeffs = [float(value) for value in coefficients]
    if not coeffs:
        raise CalibrationError("No distortion coefficients available")
    if not all(math.isfinite(value) for value in coeffs):
        raise CalibrationError("Distortion coefficients contain NaN or infinity")
    if len(coeffs) <= _PLUMB_BOB_COEFF_COUNT:
        return PLUMB_BOB, coeffs + [0.0] * (_PLUMB_BOB_COEFF_COUNT - len(coeffs)), []
    tail = coeffs[_ROS_COEFF_COUNT:]
    warnings = []
    if any(abs(value) > _TAIL_TOLERANCE for value in tail):
        warnings.append(f"Thin-prism/tilt coefficients are non-zero but ROS rational_polynomial only carries 8 values; dropped {tail}")
    return RATIONAL_POLYNOMIAL, (coeffs + [0.0] * _ROS_COEFF_COUNT)[:_ROS_COEFF_COUNT], warnings


def build_calibration(
    intrinsics: Matrix3x3,
    distortion: Sequence[float],
    model: dai.CameraModel,
    width: int,
    height: int,
    source: str,
) -> CameraCalibration:
    """Validate DepthAI-style calibration values and convert them to ROS fields."""
    if width <= 0 or height <= 0:
        raise CalibrationError(f"Invalid image size {width}x{height}")
    k = [value for row in intrinsics for value in row]
    warnings = _check_intrinsics(k, width, height)

    if model != dai.CameraModel.Perspective:
        raise CalibrationError(
            f"Distortion model '{model.name}' is not supported: the ArUco tracker passes CameraInfo.d "
            "straight to OpenCV solvePnP, which only understands the Perspective (pinhole) model",
        )
    distortion_model, d, distortion_warnings = _ros_distortion(distortion)

    return CameraCalibration(
        width=width,
        height=height,
        distortion_model=distortion_model,
        d=d,
        k=k,
        source=source,
        warnings=warnings + distortion_warnings,
    )


def from_frame_transformation(img_frame: dai.ImgFrame, width: int, height: int) -> CameraCalibration:
    """Calibration carried by a DepthAI v3 ImgFrame, matching its crop/scale."""
    transformation = img_frame.getTransformation()
    size = transformation.getSize()
    if size != (width, height):
        raise CalibrationError(f"Frame transformation size {size} does not match image {width}x{height}")
    return build_calibration(
        to_matrix3x3(transformation.getIntrinsicMatrix()),
        transformation.getDistortionCoefficients(),
        transformation.getDistortionModel(),
        width,
        height,
        source="frame transformation",
    )


def from_device(calibration_handler: dai.CalibrationHandler, socket: dai.CameraBoardSocket, width: int, height: int) -> CameraCalibration:
    """
    Calibration from the device EEPROM, rescaled to width x height.

    DepthAI assumes a centred crop that keeps the aspect ratio. If the camera
    uses a sensor mode with a different crop, this can be slightly off, which
    is why the frame transformation is preferred.
    """
    return build_calibration(
        to_matrix3x3(calibration_handler.getCameraIntrinsics(socket, width, height)),
        calibration_handler.getDistortionCoefficients(socket),
        calibration_handler.getDistortionModel(socket),
        width,
        height,
        source="device EEPROM",
    )


def resolve(
    img_frame: dai.ImgFrame,
    width: int,
    height: int,
    read_calibration: Callable[[], dai.CalibrationHandler] | None = None,
    socket: dai.CameraBoardSocket | None = None,
) -> CameraCalibration:
    """Return the best available calibration, or raise with every reason it failed."""
    errors = []
    try:
        return from_frame_transformation(img_frame, width, height)
    except Exception as error:  # noqa: BLE001 - DepthAI raises its own types; every reason is reported below
        errors.append(f"frame transformation: {error}")
    if read_calibration is not None and socket is not None:
        try:
            return from_device(read_calibration(), socket, width, height)
        except Exception as error:  # noqa: BLE001 - as above
            errors.append(f"device EEPROM: {error}")
    raise CalibrationError("No usable calibration. " + "; ".join(errors))
