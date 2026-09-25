"""Non-ROS fixture checks, runnable with system Python and OpenCV."""

import unittest

from aruco_image_bridge.synthetic_frames import DICTIONARIES, HEIGHT, WIDTH, detect_ids, make_frame


class SyntheticFramesTest(unittest.TestCase):
    def test_marker_ids_survive_rendering(self):
        for dictionary in DICTIONARIES:
            for marker_id in (0, 7, 49):
                with self.subTest(dictionary=dictionary, marker_id=marker_id):
                    frame = make_frame(dictionary, marker_id)
                    self.assertEqual(frame.shape, (HEIGHT, WIDTH, 3))
                    self.assertEqual(detect_ids(frame, dictionary), [marker_id])

    def test_blank_images_have_no_markers(self):
        for dictionary in DICTIONARIES:
            with self.subTest(dictionary=dictionary):
                self.assertEqual(detect_ids(make_frame(dictionary, visible=False), dictionary), [])

    def test_invalid_marker_is_rejected(self):
        for marker_id in (-1, 50):
            with self.subTest(marker_id=marker_id):
                with self.assertRaises(ValueError):
                    make_frame("5X5_50", marker_id)

    def test_unknown_dictionary_is_rejected(self):
        with self.assertRaises(ValueError):
            make_frame("NOT_A_DICTIONARY")


if __name__ == "__main__":
    unittest.main(verbosity=2)
