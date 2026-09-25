"""Controlled image fixtures. All calibration in this file is SYNTHETIC.

Used by the tests and by publish_test_images.py. Never used by main.py --ros.
"""

import cv2
import numpy as np

from .calibration import PLUMB_BOB, CameraCalibration

DICTIONARIES = ("4X4_100", "5X5_50", "5X5_100")
WIDTH, HEIGHT = 1280, 720
FRAME_ID = "synthetic_front_camera_optical_frame"


def get_dictionary(name):
    if name not in DICTIONARIES:
        raise ValueError(f"Choose one of {DICTIONARIES}")
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, "DICT_" + name))


def make_frame(dictionary_name, marker_id=7, visible=True):
    dictionary = get_dictionary(dictionary_name)
    if not 0 <= marker_id < len(dictionary.bytesList):
        raise ValueError("Marker ID is outside the selected dictionary")
    frame = np.full((HEIGHT, WIDTH, 3), 255, dtype=np.uint8)
    if visible:
        # OpenCV packaged with Ubuntu 22.04 uses drawMarker; newer versions
        # expose generateImageMarker. Keep both paths supported.
        if hasattr(cv2.aruco, "generateImageMarker"):
            marker = cv2.aruco.generateImageMarker(dictionary, marker_id, 200)
        else:
            marker = cv2.aruco.drawMarker(dictionary, marker_id, 200)
        x, y = (WIDTH - 200) // 2, (HEIGHT - 200) // 2
        frame[y : y + 200, x : x + 200] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
    return frame


def detect_ids(frame, dictionary_name):
    dictionary = get_dictionary(dictionary_name)
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary)
        _, ids, _ = detector.detectMarkers(frame)
    else:
        _, ids, _ = cv2.aruco.detectMarkers(frame, dictionary)
    return [] if ids is None else [int(value) for value in ids.flatten()]


def synthetic_calibration():
    """Calibration of the test fixture only, never of a physical Luxonis camera.

    With the 200 px marker and fx = 800, a tracker configured with
    marker_size = 0.15 m should report z = 800 * 0.15 / 199 = 0.603 m
    (the detected corners sit 199 px apart).
    """
    fx, fy = 800.0, 800.0
    cx, cy = (WIDTH - 1) / 2.0, (HEIGHT - 1) / 2.0
    return CameraCalibration(
        width=WIDTH,
        height=HEIGHT,
        distortion_model=PLUMB_BOB,
        d=[0.0] * 5,
        k=[fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0],
        source="synthetic",
    )
