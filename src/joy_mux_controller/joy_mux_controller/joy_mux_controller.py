import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Vector3

class JoyMuxController(Node):
    def __init__(self):
        super().__init__('joy_mux_controller')
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.rover_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_pub = self.create_publisher(Vector3, '/arm_xyz_cmd', 10)

        self.deadman_button = 4
        self.toggle_button = 12
        self.current_mode = 0
        self.last_toggle = 0

    def joy_callback(self, msg: Joy):
        if msg.buttons[self.toggle_button] == 1 and self.last_toggle == 0:
            self.current_mode = 1 - self.current_mode
            self.get_logger().info(f"Switched to {'Arm' if self.current_mode else 'Rover'} mode")
        self.last_toggle = msg.buttons[self.toggle_button]

        if msg.buttons[self.deadman_button] != 1:
            # Explicitly set all values to zero when the deadman button is not pressed
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            self.rover_pub.publish(twist)

            vec = Vector3()
            vec.x = 0.0
            vec.y = 0.0
            vec.z = 0.0
            self.arm_pub.publish(vec)
            return

        if self.current_mode == 0:
            twist = Twist()
            twist.linear.x = msg.axes[1]
            twist.angular.z = msg.axes[3]
            self.rover_pub.publish(twist)
        else:
            vec = Vector3()
            vec.x = msg.axes[0]
            vec.y = msg.axes[1]
            vec.z = msg.axes[4]
            self.arm_pub.publish(vec)

def main(args=None):
    rclpy.init(args=args)
    node = JoyMuxController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

