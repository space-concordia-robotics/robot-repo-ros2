"""Calibration conversion tests with fake DepthAI objects (no camera, no ROS)."""

import unittest

from aruco_image_bridge import calibration as calib

# Plausible 1280x720 values for a wide IMX378 (not real rover calibration).
K = [[700.0, 0.0, 641.2], [0.0, 700.5, 358.9], [0.0, 0.0, 1.0]]
COEFFS_14 = [-0.1, 0.02, 0.001, -0.002, 0.003, 0.2, -0.01, 0.004, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


class Model:
    def __init__(self, name):
        self.name = name


class FakeTransformation:
    def __init__(self, size=(1280, 720), k=K, coeffs=COEFFS_14, model="Perspective"):
        self.size, self.k, self.coeffs, self.model = size, k, coeffs, Model(model)

    def getSize(self):
        return self.size

    def getIntrinsicMatrix(self):
        return self.k

    def getDistortionCoefficients(self):
        return self.coeffs

    def getDistortionModel(self):
        return self.model


class FakeFrame:
    def __init__(self, transformation):
        self.transformation = transformation

    def getTransformation(self):
        return self.transformation


class FakeHandler:
    def __init__(self, k=K, coeffs=COEFFS_14, model="Perspective"):
        self.k, self.coeffs, self.model = k, coeffs, Model(model)
        self.requests = []

    def getCameraIntrinsics(self, socket, width, height):
        self.requests.append((socket, width, height))
        return self.k

    def getDistortionCoefficients(self, socket):
        return self.coeffs

    def getDistortionModel(self, socket):
        return self.model


class BuildCalibrationTest(unittest.TestCase):
    def test_perspective_14_coefficients_become_rational_polynomial(self):
        cal = calib.build_calibration(K, COEFFS_14, Model("Perspective"), 1280, 720, "test")
        self.assertEqual(cal.distortion_model, calib.RATIONAL_POLYNOMIAL)
        self.assertEqual(cal.d, COEFFS_14[:8])
        self.assertEqual(cal.k, [700.0, 0.0, 641.2, 0.0, 700.5, 358.9, 0.0, 0.0, 1.0])
        self.assertEqual(cal.p, [700.0, 0.0, 641.2, 0.0, 0.0, 700.5, 358.9, 0.0, 0.0, 0.0, 1.0, 0.0])
        self.assertEqual(cal.r, [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
        self.assertEqual(cal.warnings, [])

    def test_five_coefficients_become_plumb_bob(self):
        cal = calib.build_calibration(K, [0.1, 0.0, 0.0, 0.0, 0.0], Model("Perspective"), 1280, 720, "test")
        self.assertEqual(cal.distortion_model, calib.PLUMB_BOB)
        self.assertEqual(len(cal.d), 5)

    def test_nonzero_tail_coefficients_warn(self):
        coeffs = COEFFS_14[:8] + [0.01, 0.0, 0.0, 0.0, 0.0, 0.0]
        cal = calib.build_calibration(K, coeffs, Model("Perspective"), 1280, 720, "test")
        self.assertEqual(len(cal.warnings), 1)

    def test_identity_matrix_from_uncalibrated_frame_is_rejected(self):
        identity = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        with self.assertRaisesRegex(calib.CalibrationError, "focal length"):
            calib.build_calibration(identity, COEFFS_14, Model("Perspective"), 1280, 720, "test")

    def test_principal_point_outside_image_is_rejected(self):
        k = [[700.0, 0.0, 1500.0], [0.0, 700.0, 360.0], [0.0, 0.0, 1.0]]
        with self.assertRaisesRegex(calib.CalibrationError, "Principal point"):
            calib.build_calibration(k, COEFFS_14, Model("Perspective"), 1280, 720, "test")

    def test_fisheye_is_rejected(self):
        with self.assertRaisesRegex(calib.CalibrationError, "Fisheye"):
            calib.build_calibration(K, [0.1, 0.0, 0.0, 0.0], Model("Fisheye"), 1280, 720, "test")

    def test_missing_distortion_is_rejected(self):
        with self.assertRaisesRegex(calib.CalibrationError, "distortion"):
            calib.build_calibration(K, [], Model("Perspective"), 1280, 720, "test")

    def test_nan_is_rejected(self):
        k = [[float("nan"), 0.0, 640.0], [0.0, 700.0, 360.0], [0.0, 0.0, 1.0]]
        with self.assertRaises(calib.CalibrationError):
            calib.build_calibration(k, COEFFS_14, Model("Perspective"), 1280, 720, "test")

    def test_dai_style_enum_string_is_understood(self):
        class DaiLikeEnum:
            def __str__(self):
                return "CameraModel.Perspective"

        cal = calib.build_calibration(K, COEFFS_14, DaiLikeEnum(), 1280, 720, "test")
        self.assertEqual(cal.distortion_model, calib.RATIONAL_POLYNOMIAL)


class ResolveTest(unittest.TestCase):
    def test_frame_transformation_is_preferred(self):
        handler = FakeHandler()
        cal = calib.resolve(FakeFrame(FakeTransformation()), 1280, 720, lambda: handler, socket="CAM_A")
        self.assertEqual(cal.source, "frame transformation")
        self.assertEqual(handler.requests, [])

    def test_uncalibrated_frame_falls_back_to_device(self):
        identity = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        frame = FakeFrame(FakeTransformation(k=identity, coeffs=[]))
        handler = FakeHandler()
        cal = calib.resolve(frame, 1280, 720, lambda: handler, socket="CAM_A")
        self.assertEqual(cal.source, "device EEPROM")
        self.assertEqual(handler.requests, [("CAM_A", 1280, 720)])

    def test_transformation_size_mismatch_falls_back_to_device(self):
        frame = FakeFrame(FakeTransformation(size=(1920, 1080)))
        cal = calib.resolve(frame, 1280, 720, FakeHandler, socket="CAM_A")
        self.assertEqual(cal.source, "device EEPROM")

    def test_no_usable_source_reports_every_reason(self):
        identity = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        frame = FakeFrame(FakeTransformation(k=identity))

        def broken_eeprom():
            raise RuntimeError("no calibration on device")

        with self.assertRaises(calib.CalibrationError) as caught:
            calib.resolve(frame, 1280, 720, broken_eeprom, socket="CAM_A")
        self.assertIn("frame transformation", str(caught.exception))
        self.assertIn("no calibration on device", str(caught.exception))


if __name__ == "__main__":
    unittest.main(verbosity=2)
