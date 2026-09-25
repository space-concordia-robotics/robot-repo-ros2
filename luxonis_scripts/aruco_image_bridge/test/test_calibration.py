"""Calibration conversion tests with fake DepthAI objects (no camera, no ROS)."""

from __future__ import annotations

from typing import override

import pytest

from aruco_image_bridge import calibration as calib
from aruco_image_bridge.calibration import Matrix3x3
from aruco_image_bridge.test.fakes import COEFFS_14, IDENTITY, FakeFrame, FakeHandler, FakeTransformation, K, Model

PERSPECTIVE = Model("Perspective")
PLUMB_BOB_COEFFS = 5


def test_perspective_14_coefficients_become_rational_polynomial():
    cal = calib.build_calibration(K, COEFFS_14, PERSPECTIVE, 1280, 720, "test")
    assert cal.distortion_model == calib.RATIONAL_POLYNOMIAL
    assert cal.d == COEFFS_14[:8]
    assert cal.k == [700.0, 0.0, 641.2, 0.0, 700.5, 358.9, 0.0, 0.0, 1.0]
    assert cal.p == [700.0, 0.0, 641.2, 0.0, 0.0, 700.5, 358.9, 0.0, 0.0, 0.0, 1.0, 0.0]
    assert cal.r == [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    assert cal.warnings == []


def test_five_coefficients_become_plumb_bob():
    cal = calib.build_calibration(K, [0.1, 0.0, 0.0, 0.0, 0.0], PERSPECTIVE, 1280, 720, "test")
    assert cal.distortion_model == calib.PLUMB_BOB
    assert len(cal.d) == PLUMB_BOB_COEFFS


def test_nonzero_tail_coefficients_warn():
    coeffs = [*COEFFS_14[:8], 0.01, 0.0, 0.0, 0.0, 0.0, 0.0]
    cal = calib.build_calibration(K, coeffs, PERSPECTIVE, 1280, 720, "test")
    assert len(cal.warnings) == 1


@pytest.mark.parametrize(
    ("k", "match"),
    [
        (IDENTITY, "focal length"),
        ([[700.0, 0.0, 1500.0], [0.0, 700.0, 360.0], [0.0, 0.0, 1.0]], "Principal point"),
        ([[float("nan"), 0.0, 640.0], [0.0, 700.0, 360.0], [0.0, 0.0, 1.0]], "NaN"),
    ],
)
def test_implausible_intrinsics_are_rejected(k: Matrix3x3, match: str):
    with pytest.raises(calib.CalibrationError, match=match):
        calib.build_calibration(k, COEFFS_14, PERSPECTIVE, 1280, 720, "test")


def test_fisheye_is_rejected():
    with pytest.raises(calib.CalibrationError, match="Fisheye"):
        calib.build_calibration(K, [0.1, 0.0, 0.0, 0.0], Model("Fisheye"), 1280, 720, "test")


def test_missing_distortion_is_rejected():
    with pytest.raises(calib.CalibrationError, match="distortion"):
        calib.build_calibration(K, [], PERSPECTIVE, 1280, 720, "test")


def test_dai_style_enum_string_is_understood():
    class DaiLikeEnum:
        @override
        def __str__(self) -> str:
            return "CameraModel.Perspective"

    cal = calib.build_calibration(K, COEFFS_14, DaiLikeEnum(), 1280, 720, "test")
    assert cal.distortion_model == calib.RATIONAL_POLYNOMIAL


def test_frame_transformation_is_preferred():
    handler = FakeHandler()
    cal = calib.resolve(FakeFrame(FakeTransformation()), 1280, 720, lambda: handler, socket="CAM_A")
    assert cal.source == "frame transformation"
    assert handler.requests == []


def test_uncalibrated_frame_falls_back_to_device():
    handler = FakeHandler()
    frame = FakeFrame(FakeTransformation(k=IDENTITY, coeffs=[]))
    cal = calib.resolve(frame, 1280, 720, lambda: handler, socket="CAM_A")
    assert cal.source == "device EEPROM"
    assert handler.requests == [("CAM_A", 1280, 720)]


def test_transformation_size_mismatch_falls_back_to_device():
    frame = FakeFrame(FakeTransformation(size=(1920, 1080)))
    cal = calib.resolve(frame, 1280, 720, FakeHandler, socket="CAM_A")
    assert cal.source == "device EEPROM"


def test_no_usable_source_reports_every_reason():
    frame = FakeFrame(FakeTransformation(k=IDENTITY))

    def broken_eeprom() -> FakeHandler:
        raise RuntimeError("no calibration on device")

    with pytest.raises(calib.CalibrationError) as caught:
        calib.resolve(frame, 1280, 720, broken_eeprom, socket="CAM_A")
    assert "frame transformation" in str(caught.value)
    assert "no calibration on device" in str(caught.value)
