"""Fixture checks, runnable with plain Python and OpenCV (no ROS)."""

from __future__ import annotations

import numpy as np
import pytest
from aruco_image_bridge.synthetic_frames import DICTIONARIES, ENCODINGS, HEIGHT, WIDTH, detect_ids, make_frame


@pytest.mark.parametrize("dictionary", DICTIONARIES)
def test_marker_ids_survive_rendering(dictionary: str):
    frame = make_frame(dictionary, 7, visible=True)
    assert frame.shape == (HEIGHT, WIDTH, 3)
    assert frame.dtype == np.uint8
    assert detect_ids(frame, dictionary) == [7]


@pytest.mark.parametrize("dictionary", DICTIONARIES)
def test_blank_images_have_no_markers(dictionary: str):
    assert detect_ids(make_frame(dictionary, 7, visible=False), dictionary) == []


def test_invalid_marker_is_rejected():
    with pytest.raises(ValueError, match="outside"):
        make_frame("5X5_50", 50, visible=True)


def test_unknown_dictionary_is_rejected():
    with pytest.raises(ValueError, match="Choose one of"):
        make_frame("6X6_250", 7, visible=True)


@pytest.mark.parametrize(("encoding", "shape"), [("bgr8", (HEIGHT, WIDTH, 3)), ("rgb8", (HEIGHT, WIDTH, 3)), ("mono8", (HEIGHT, WIDTH))])
def test_every_encoding_renders_and_is_detected(encoding: str, shape: tuple[int, ...]):
    assert set(ENCODINGS) == {"bgr8", "rgb8", "mono8"}
    frame = make_frame("4X4_50", 7, visible=True, encoding=encoding)
    assert frame.shape == shape
    assert detect_ids(frame, "4X4_50") == [7]


def test_unknown_encoding_is_rejected():
    with pytest.raises(ValueError, match="Choose one of"):
        make_frame("4X4_50", 7, visible=True, encoding="yuv422")
