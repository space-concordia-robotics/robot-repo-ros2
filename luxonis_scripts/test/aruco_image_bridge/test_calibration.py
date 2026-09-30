"""Calibration conversion tests with real DepthAI objects (no camera, no ROS)."""

from __future__ import annotations

import depthai as dai
import pytest
from aruco_image_bridge import calibration as calib
from aruco_image_bridge.calibration import Matrix3x3
from fakes import (
    COEFFS_14,
    IDENTITY,
    K,
    make_calibration_handler,
    make_img_frame,
    make_transformation,
)

PERSPECTIVE = dai.CameraModel.Perspective
PLUMB_BOB_COEFFS = 5


def test_perspective_14_coefficients_become_rational_polynomial():
    cal = calib.build_calibration(K, COEFFS_14, PERSPECTIVE, 1280, 720, "test")
    assert cal.distortion_model == calib.DistortionModel.RATIONAL_POLYNOMIAL
    assert cal.d == tuple(COEFFS_14[:8])
    assert cal.k == (700.0, 0.0, 640.0, 0.0, 700.5, 359.5, 0.0, 0.0, 1.0)
    assert cal.p == (700.0, 0.0, 640.0, 0.0, 0.0, 700.5, 359.5, 0.0, 0.0, 0.0, 1.0, 0.0)
    assert cal.r == (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
    assert cal.warnings == []


def test_five_coefficients_become_plumb_bob():
    cal = calib.build_calibration(K, [0.1, 0.0, 0.0, 0.0, 0.0], PERSPECTIVE, 1280, 720, "test")
    assert cal.distortion_model == calib.DistortionModel.PLUMB_BOB
    assert len(cal.d) == PLUMB_BOB_COEFFS


def test_nonzero_tail_coefficients_warn():
    coeffs = [*COEFFS_14[:8], 0.01, 0.0, 0.0, 0.0, 0.0, 0.0]
    cal = calib.build_calibration(K, coeffs, PERSPECTIVE, 1280, 720, "test")
    assert len(cal.warnings) == 1


@pytest.mark.parametrize(
    ("k", "match"),
    [
        (IDENTITY, "focal length"),
        (((700.0, 0.0, 1500.0), (0.0, 700.0, 360.0), (0.0, 0.0, 1.0)), "Principal point"),
        (((float("nan"), 0.0, 640.0), (0.0, 700.0, 360.0), (0.0, 0.0, 1.0)), "NaN"),
    ],
)
def test_implausible_intrinsics_are_rejected(k: Matrix3x3, match: str):
    with pytest.raises(calib.CalibrationError, match=match):
        calib.build_calibration(k, COEFFS_14, PERSPECTIVE, 1280, 720, "test")


def test_fisheye_is_rejected():
    with pytest.raises(calib.CalibrationError, match="Fisheye"):
        calib.build_calibration(K, [0.1, 0.0, 0.0, 0.0], dai.CameraModel.Fisheye, 1280, 720, "test")


def test_missing_distortion_is_rejected():
    with pytest.raises(calib.CalibrationError, match="distortion"):
        calib.build_calibration(K, [], PERSPECTIVE, 1280, 720, "test")


def test_to_matrix3x3_checks_the_shape():
    assert calib.to_matrix3x3([[1, 2, 3], [4, 5, 6], [7, 8, 9]]) == ((1.0, 2.0, 3.0), (4.0, 5.0, 6.0), (7.0, 8.0, 9.0))
    for bad in ([[1, 2, 3], [4, 5, 6]], [[1, 2], [3, 4], [5, 6]], [[1, 2, 3]] * 4):
        with pytest.raises(calib.CalibrationError, match="3x3"):
            calib.to_matrix3x3(bad)


def test_frame_transformation_is_preferred():
    cal = calib.resolve(make_img_frame(), 1280, 720, make_calibration_handler, socket=dai.CameraBoardSocket.CAM_A)
    assert cal.source == "frame transformation"
    assert cal.k[0] == K[0][0]


def test_uncalibrated_frame_falls_back_to_the_device_and_scales_it():
    frame = make_img_frame(transformation=dai.ImgTransformation(1280, 720))  # identity K, like an uncalibrated camera
    cal = calib.resolve(frame, 1280, 720, make_calibration_handler, socket=dai.CameraBoardSocket.CAM_A)
    assert cal.source == "device EEPROM"
    # 1080p calibration scaled to 720p by DepthAI: 1050 * 720/1080 = 700, principal point (640, 360)
    assert cal.k[0] == pytest.approx(700.0)
    assert (cal.k[2], cal.k[5]) == pytest.approx((640.0, 360.0))


def test_transformation_size_mismatch_falls_back_to_the_device():
    frame = make_img_frame(transformation=make_transformation(size=(1920, 1080)))
    cal = calib.resolve(frame, 1280, 720, make_calibration_handler, socket=dai.CameraBoardSocket.CAM_A)
    assert cal.source == "device EEPROM"


def test_no_usable_source_reports_every_reason():
    frame = make_img_frame(transformation=make_transformation(k=IDENTITY))

    def broken_eeprom() -> dai.CalibrationHandler:
        raise RuntimeError("no calibration on device")

    with pytest.raises(calib.CalibrationError) as caught:
        calib.resolve(frame, 1280, 720, broken_eeprom, socket=dai.CameraBoardSocket.CAM_A)
    assert "frame transformation" in str(caught.value)
    assert "no calibration on device" in str(caught.value)
