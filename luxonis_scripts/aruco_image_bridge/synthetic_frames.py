"""
Controlled image fixtures. All calibration in this file is SYNTHETIC.

Used by the tests and by publish_test_images.py. Never used by main.py --ros.
"""

from __future__ import annotations

from collections.abc import Callable

import cv2
import numpy as np
import numpy.typing as npt

from .calibration import PLUMB_BOB, CameraCalibration

DICTIONARIES = ("4X4_50", "4X4_100", "5X5_50", "5X5_100")
ENCODINGS = ("bgr8", "rgb8", "mono8")
_COLOUR_CONVERSIONS = {"rgb8": cv2.COLOR_BGR2RGB, "mono8": cv2.COLOR_BGR2GRAY}
WIDTH, HEIGHT = 1280, 720
FRAME_ID = "synthetic_front_camera_optical_frame"
MARKER_PX = 200
WHITE = 255
SYNTHETIC_FOCAL_PX = 800.0

# Ubuntu 24.04's apt OpenCV (4.6) and pip's OpenCV (>= 4.7) name these differently;
# looking them up by name keeps both working and keeps type checkers quiet on both.
_MARKER_GENERATORS = ("generateImageMarker", "drawMarker")


def get_dictionary(name: str) -> cv2.aruco.Dictionary:
    """Predefined OpenCV dictionary for one of ``DICTIONARIES``."""
    if name not in DICTIONARIES:
        raise ValueError(f"Choose one of {DICTIONARIES}")
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, "DICT_" + name))


def _marker_image(dictionary: cv2.aruco.Dictionary, marker_id: int, size: int) -> npt.NDArray[np.uint8]:
    for name in _MARKER_GENERATORS:
        generate: Callable[..., npt.NDArray[np.uint8]] | None = getattr(cv2.aruco, name, None)
        if generate is not None:
            return generate(dictionary, marker_id, size)
    raise RuntimeError("This OpenCV build has no ArUco marker generator")


def make_frame(dictionary_name: str, marker_id: int = 7, *, visible: bool = True, encoding: str = "bgr8") -> npt.NDArray[np.uint8]:
    """White 1280x720 frame in ``encoding``, with the marker drawn in the centre when ``visible``."""
    if encoding not in ENCODINGS:
        raise ValueError(f"Choose one of {ENCODINGS}")
    dictionary = get_dictionary(dictionary_name)
    if not 0 <= marker_id < len(dictionary.bytesList):
        raise ValueError("Marker ID is outside the selected dictionary")
    frame = np.full((HEIGHT, WIDTH, 3), WHITE, dtype=np.uint8)
    if visible:
        marker = _marker_image(dictionary, marker_id, MARKER_PX)
        x, y = (WIDTH - MARKER_PX) // 2, (HEIGHT - MARKER_PX) // 2
        frame[y : y + MARKER_PX, x : x + MARKER_PX] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
    if encoding in _COLOUR_CONVERSIONS:
        return np.asarray(cv2.cvtColor(frame, _COLOUR_CONVERSIONS[encoding]), dtype=np.uint8)
    return frame


def detect_ids(frame: npt.NDArray[np.uint8], dictionary_name: str) -> list[int]:
    """Independent OpenCV check of which marker IDs a frame contains."""
    dictionary = get_dictionary(dictionary_name)
    detector_class = getattr(cv2.aruco, "ArucoDetector", None)
    if detector_class is not None:
        _, ids, _ = detector_class(dictionary).detectMarkers(frame)
    else:
        legacy_name = "detectMarkers"  # OpenCV < 4.7 only
        _, ids, _ = getattr(cv2.aruco, legacy_name)(frame, dictionary)
    return [] if ids is None else [int(value) for value in ids.flatten()]


def synthetic_calibration() -> CameraCalibration:
    """
    Calibration of the test fixture only, never of a physical Luxonis camera.

    With the 200 px marker and fx = 800, a tracker configured with
    marker_size = 0.15 m should report z = 800 * 0.15 / 199 = 0.603 m
    (the detected corners sit 199 px apart).
    """
    fx = fy = SYNTHETIC_FOCAL_PX
    cx, cy = (WIDTH - 1) / 2.0, (HEIGHT - 1) / 2.0
    return CameraCalibration(
        width=WIDTH,
        height=HEIGHT,
        distortion_model=PLUMB_BOB,
        d=(0.0,) * 5,
        k=(fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0),
        source="synthetic",
    )
