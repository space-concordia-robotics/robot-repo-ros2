"""Turn DepthAI calibration data into ROS CameraInfo fields.

Nothing here imports ROS or DepthAI; DepthAI objects are used through their
methods only, so the logic can be unit-tested with small fake objects.

Two calibration sources are supported, in order of preference:

1. ``ImgFrame.getTransformation()`` (DepthAI v3). Its intrinsic matrix already
   includes the crop and scaling the camera applied to produce this exact frame.
2. ``CalibrationHandler.getCameraIntrinsics(socket, width, height)`` read from
   the device EEPROM, used when the frame carries no usable calibration.

A frame without real calibration still reports a "valid" transformation with
an identity intrinsic matrix, so every result is sanity-checked before use.
"""

import math
from dataclasses import dataclass, field

RATIONAL_POLYNOMIAL = "rational_polynomial"
PLUMB_BOB = "plumb_bob"

# DepthAI "Perspective" coefficients use OpenCV's order:
# k1 k2 p1 p2 k3 k4 k5 k6 s1 s2 s3 s4 tauX tauY
_ROS_COEFF_COUNT = 8
_TAIL_TOLERANCE = 1e-6
_MIN_FOCAL_PX = 10.0


class CalibrationError(ValueError):
    """Calibration is missing, implausible or in an unsupported model."""


@dataclass
class CameraCalibration:
    """Everything a CameraInfo message needs, for one image size."""

    width: int
    height: int
    distortion_model: str
    d: list
    k: list
    r: list = field(default_factory=lambda: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
    p: list = field(default_factory=list)
    source: str = "unknown"
    warnings: list = field(default_factory=list)

    def __post_init__(self):
        if not self.p:
            fx, _, cx, _, fy, cy, _, _, _ = self.k
            self.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

    def summary(self):
        fx, _, cx, _, fy, cy, _, _, _ = self.k
        return f"{self.width}x{self.height} fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f} model={self.distortion_model} source={self.source}"


def _model_name(model):
    name = getattr(model, "name", None) or str(model)
    return name.split(".")[-1]


def _flatten_3x3(matrix):
    rows = [list(row) for row in matrix]
    if len(rows) != 3 or any(len(row) != 3 for row in rows):
        raise CalibrationError("Intrinsic matrix must be 3x3")
    return [float(value) for row in rows for value in row]


def build_calibration(intrinsics, distortion, model, width, height, source):
    """Validate DepthAI-style calibration values and convert them to ROS fields.

    ``intrinsics`` is a 3x3 nested list, ``distortion`` a flat list, ``model``
    a ``dai.CameraModel`` (or anything whose name ends in the model name).
    """
    if not (isinstance(width, int) and isinstance(height, int) and width > 0 and height > 0):
        raise CalibrationError(f"Invalid image size {width}x{height}")

    k = _flatten_3x3(intrinsics)
    if not all(math.isfinite(value) for value in k):
        raise CalibrationError("Intrinsic matrix contains NaN or infinity")
    fx, skew, cx, zero1, fy, cy, zero2, zero3, one = k
    if fx < _MIN_FOCAL_PX or fy < _MIN_FOCAL_PX:
        raise CalibrationError(f"Implausible focal length fx={fx}, fy={fy}; the camera is probably uncalibrated")
    if not (0.0 < cx < width and 0.0 < cy < height):
        raise CalibrationError(f"Principal point ({cx}, {cy}) lies outside the {width}x{height} image")
    if abs(zero1) > 1e-9 or abs(zero2) > 1e-9 or abs(zero3) > 1e-9 or abs(one - 1.0) > 1e-9:
        raise CalibrationError("Intrinsic matrix does not have the expected [.., .., ..; 0 .. ..; 0 0 1] form")

    warnings = []
    if abs(skew) > 1e-9:
        warnings.append(f"Non-zero skew {skew} is not representable in most ROS tools")

    name = _model_name(model)
    if name != "Perspective":
        raise CalibrationError(
            f"Distortion model '{name}' is not supported: the ArUco tracker passes CameraInfo.d "
            "straight to OpenCV solvePnP, which only understands the Perspective (pinhole) model"
        )

    coeffs = [float(value) for value in distortion]
    if not coeffs:
        raise CalibrationError("No distortion coefficients available")
    if not all(math.isfinite(value) for value in coeffs):
        raise CalibrationError("Distortion coefficients contain NaN or infinity")

    if len(coeffs) <= 5:
        d = coeffs + [0.0] * (5 - len(coeffs))
        model_name = PLUMB_BOB
    else:
        d = (coeffs + [0.0] * _ROS_COEFF_COUNT)[:_ROS_COEFF_COUNT]
        model_name = RATIONAL_POLYNOMIAL
        tail = coeffs[_ROS_COEFF_COUNT:]
        if any(abs(value) > _TAIL_TOLERANCE for value in tail):
            warnings.append(f"Thin-prism/tilt coefficients are non-zero but ROS rational_polynomial only carries 8 values; dropped {tail}")

    return CameraCalibration(
        width=width,
        height=height,
        distortion_model=model_name,
        d=d,
        k=k,
        source=source,
        warnings=warnings,
    )


def from_frame_transformation(img_frame, width, height):
    """Calibration carried by a DepthAI v3 ImgFrame, matching its crop/scale."""
    get_transformation = getattr(img_frame, "getTransformation", None)
    if get_transformation is None:
        raise CalibrationError("This DepthAI frame has no getTransformation()")
    transformation = get_transformation()
    size = tuple(transformation.getSize())
    if size != (width, height):
        raise CalibrationError(f"Frame transformation size {size} does not match image {width}x{height}")
    return build_calibration(
        transformation.getIntrinsicMatrix(),
        transformation.getDistortionCoefficients(),
        transformation.getDistortionModel(),
        width,
        height,
        source="frame transformation",
    )


def from_device(calibration_handler, socket, width, height):
    """Calibration from the device EEPROM, rescaled to width x height.

    DepthAI assumes a centred crop that keeps the aspect ratio. If the camera
    uses a sensor mode with a different crop, this can be slightly off, which
    is why the frame transformation is preferred.
    """
    return build_calibration(
        calibration_handler.getCameraIntrinsics(socket, width, height),
        calibration_handler.getDistortionCoefficients(socket),
        calibration_handler.getDistortionModel(socket),
        width,
        height,
        source="device EEPROM",
    )


def resolve(img_frame, width, height, read_calibration=None, socket=None):
    """Return the best available calibration, or raise with every reason it failed."""
    errors = []
    try:
        return from_frame_transformation(img_frame, width, height)
    except Exception as error:  # DepthAI can raise its own errors too; report them all
        errors.append(f"frame transformation: {error}")
    if read_calibration is not None and socket is not None:
        try:
            return from_device(read_calibration(), socket, width, height)
        except Exception as error:
            errors.append(f"device EEPROM: {error}")
    raise CalibrationError("No usable calibration. " + "; ".join(errors))
