#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from aruco_opencv_msgs.msg import ArucoDetection
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from scipy.spatial.transform import Rotation as R

class ArucoPoseAnalyzer(Node):
    def __init__(self):
        super().__init__('aruco_pose_analyzer')

        # Parameters
        self.declare_parameter('angle_threshold_deg', 15.0)
        self.declare_parameter('detections_topic', '/aruco_detections')

        self.angle_threshold = self.get_parameter('angle_threshold_deg').value
        self.detections_topic = self.get_parameter('detections_topic').value

        # Subscription
        self.sub = self.create_subscription(
            ArucoDetection,
            self.detections_topic,
            self.detection_callback,
            10
        )

        # Publisher for Rviz markers
        self.marker_pub = self.create_publisher(MarkerArray, '/aruco_markers_vis', 10)

        self.get_logger().info(
            f"Aruco Pose Analyzer running. Threshold {self.angle_threshold:.1f}° | "
            f"subscribing to {self.detections_topic}"
        )

    def detection_callback(self, msg: ArucoDetection):
        marker_array = MarkerArray()

        # Loop through all markers in this detection message
        for marker in msg.markers:
            pose = marker.pose
            orientation = pose.orientation
            position = pose.position

            # Convert quaternion to rotation matrix
            q = [orientation.x, orientation.y, orientation.z, orientation.w]
            rot = R.from_quat(q)
            rot_matrix = rot.as_matrix()

            # Marker normal vector (Z-axis)
            normal_vector = rot_matrix[:, 2]

            # World vertical axis
            vertical_axis = np.array([0, 0, 1])

            # Angle between marker normal and vertical
            cos_angle = np.dot(normal_vector, vertical_axis)
            cos_angle = np.clip(np.abs(cos_angle), 0.0, 1.0)
            angle_deg = np.degrees(np.arccos(cos_angle))
            is_vertical = angle_deg < self.angle_threshold

            # Log detection
            self.get_logger().info(
                f"[ID {marker.marker_id}] Pos: ({position.x:.4f}, {position.y:.4f}, {position.z:.4f}) | "
                f"Angle: {angle_deg:.2f}° | Vertical: {is_vertical}"
            )

            # Create Rviz marker
            rviz_marker = Marker()
            rviz_marker.header.frame_id = 'camera_frame'
            rviz_marker.header.stamp = self.get_clock().now().to_msg()
            rviz_marker.ns = "aruco_markers"
            rviz_marker.id = marker.marker_id
            rviz_marker.type = Marker.ARROW
            rviz_marker.action = Marker.ADD
            rviz_marker.pose = pose
            rviz_marker.scale.x = 0.05
            rviz_marker.scale.y = 0.01
            rviz_marker.scale.z = 0.01
            rviz_marker.color.r = 1.0
            rviz_marker.color.g = 0.0
            rviz_marker.color.b = 0.0
            rviz_marker.color.a = 1.0

            marker_array.markers.append(rviz_marker)

        # Publish markers
        if marker_array.markers:
            self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoPoseAnalyzer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

