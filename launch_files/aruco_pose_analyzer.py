"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np
from scipy.spatial.transform import Rotation as R

class ArucoPoseAnalyzer(Node):
    def __init__(self):
        super().__init__('aruco_pose_analyzer')

        # Subscribe to pose from ArUco marker
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/aruco_marker/pose',  # Should match the topic from aruco_opencv
            self.pose_callback,
            10
        )

        self.get_logger().info("Aruco Pose Analyzer node started.")

    def pose_callback(self, msg):
        orientation = msg.pose.orientation
        position = msg.pose.position

        # Convert quaternion to rotation matrix
        q = [orientation.x, orientation.y, orientation.z, orientation.w]
        rot = R.from_quat(q)
        rot_matrix = rot.as_matrix()

        # Z-axis of the marker frame is considered the normal vector
        normal_vector = rot_matrix[:, 2]

        # Y-up coordinate frame
        vertical_axis = np.array([0, 1, 0])

        # Angle between normal and vertical axis
        cos_angle = np.dot(normal_vector, vertical_axis) / (
            np.linalg.norm(normal_vector) * np.linalg.norm(vertical_axis)
        )
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        angle_deg = np.degrees(np.arccos(cos_angle))

        is_vertical = angle_deg < 15.0  # You can adjust this threshold

        self.get_logger().info(
            f"Position: ({position.x:.2f}, {position.y:.2f}, {position.z:.2f}) | "
            f"Angle to vertical: {angle_deg:.2f}° | Is Vertical: {is_vertical}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = ArucoPoseAnalyzer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
"""
