"""
Synthetic ROS Image + CameraInfo publisher. TEST TOOL, not the camera source.

It publishes through the same ros_bridge.CameraPublisher that main.py --ros
uses, so a passing test exercises the production message code.

Run from luxonis_scripts/:
    python3 -m aruco_image_bridge.publish_test_images [--dictionary 5X5_50] [--marker-id 7] [--encoding mono8]

No hardware is opened and no robot motion commands are published.
"""

from __future__ import annotations

import argparse

import rclpy

from .luxonis_adapter import OUTPUT_ENCODINGS
from .ros_bridge import RosBridge
from .synthetic_frames import DICTIONARIES, FRAME_ID, make_frame, synthetic_calibration

TEST_PREFIX = "/test/camera/front"
PERIOD_S = 0.2  # 5 FPS
CYCLE = 30  # frames per cycle: 20 with the marker (4 s), 10 blank (2 s)
VISIBLE = 20


def main() -> None:
    """Publish the marker/blank cycle until Ctrl+C."""
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dictionary", choices=DICTIONARIES, default="4X4_50")
    parser.add_argument("--marker-id", type=int, default=7)
    parser.add_argument("--encoding", choices=OUTPUT_ENCODINGS, default="bgr8")
    args = parser.parse_args()
    visible = make_frame(args.dictionary, args.marker_id, visible=True, encoding=args.encoding)  # validates dictionary/ID
    blank = make_frame(args.dictionary, args.marker_id, visible=False, encoding=args.encoding)
    calibration = synthetic_calibration()

    bridge = RosBridge("aruco_test_image_publisher")
    publisher = bridge.camera_publisher(f"{TEST_PREFIX}/image_raw", f"{TEST_PREFIX}/camera_info", FRAME_ID)
    counter = {"index": 0}

    def tick() -> None:
        frame = visible if counter["index"] % CYCLE < VISIBLE else blank
        publisher.publish(frame, calibration, encoding=args.encoding)
        counter["index"] += 1

    bridge.node.create_timer(PERIOD_S, tick)
    bridge.logger.info(
        f"SYNTHETIC test: {args.dictionary}, marker {args.marker_id}, {args.encoding}, "  # noqa: G004
        f"{calibration.width}x{calibration.height} at 5 FPS. Cycles 4 s marker / 2 s blank. "
        "Calibration is artificial.",
    )
    bridge.logger.info(f"Publishing {publisher.image_topic} and {publisher.info_topic}")  # noqa: G004
    try:
        rclpy.spin(bridge.node)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.shutdown()


if __name__ == "__main__":
    main()
