"""Message-building tests. Need ROS 2 sourced (sensor_msgs), but no running nodes."""

import unittest

import numpy as np

try:
    from aruco_image_bridge.ros_bridge import camera_info_msg, image_msg, stamp_from_ns
    from aruco_image_bridge.synthetic_frames import synthetic_calibration

    HAVE_ROS = True
except ImportError:
    HAVE_ROS = False


@unittest.skipUnless(HAVE_ROS, "ROS 2 not sourced (source /opt/ros/jazzy/setup.bash)")
class MessageTest(unittest.TestCase):
    def setUp(self):
        self.stamp = stamp_from_ns(1_790_321_475_149_884_462)
        self.frame = np.zeros((720, 1280, 3), dtype=np.uint8)
        self.frame[10, 20] = (1, 2, 3)

    def test_stamp_conversion(self):
        self.assertEqual((self.stamp.sec, self.stamp.nanosec), (1_790_321_475, 149_884_462))

    def test_image_layout(self):
        msg = image_msg(self.frame, self.stamp, "cam_optical_frame")
        self.assertEqual((msg.width, msg.height, msg.encoding, msg.step), (1280, 720, "bgr8", 3840))
        self.assertEqual(len(msg.data), 1280 * 720 * 3)
        restored = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(720, 1280, 3)
        self.assertTrue(np.array_equal(restored, self.frame))
        self.assertEqual(msg.header.frame_id, "cam_optical_frame")

    def test_non_contiguous_frames_are_accepted(self):
        wide = np.zeros((720, 2560, 3), dtype=np.uint8)
        msg = image_msg(wide[:, ::2], self.stamp, "cam_optical_frame")
        self.assertEqual(msg.width, 1280)

    def test_bad_frames_are_rejected(self):
        for bad in (np.zeros((720, 1280), np.uint8), np.zeros((720, 1280, 3), np.float32), "image"):
            with self.subTest(bad=type(bad)):
                with self.assertRaises(ValueError):
                    image_msg(bad, self.stamp, "cam_optical_frame")
        with self.assertRaises(ValueError):
            image_msg(self.frame, self.stamp, "")

    def test_camera_info_matches_calibration_and_header(self):
        cal = synthetic_calibration()
        info = camera_info_msg(cal, self.stamp, "cam_optical_frame")
        image = image_msg(self.frame, self.stamp, "cam_optical_frame")
        self.assertEqual(info.header, image.header)
        self.assertEqual((info.width, info.height), (1280, 720))
        self.assertEqual(info.distortion_model, "plumb_bob")
        self.assertEqual(list(info.k), cal.k)
        self.assertEqual(list(info.p), cal.p)


if __name__ == "__main__":
    unittest.main(verbosity=2)
