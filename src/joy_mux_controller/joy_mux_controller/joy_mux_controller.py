import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import Twist

class JoyMuxController(Node):
    def __init__(self):
        super().__init__('joy_mux_controller')
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.rover_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_pub = self.create_publisher(JointState, '/arm_xyz_cmd', 10)  # Changed to JointState

        self.deadman_button = 4
        self.toggle_button = 10
        self.current_mode = 0
        self.last_toggle = 0

    def joy_callback(self, msg: Joy):
        
        if msg.buttons[self.toggle_button] == 1 and self.last_toggle == 0:
            self.current_mode = 1 - self.current_mode
            self.get_logger().info(f"Switched to {'Arm' if self.current_mode else 'Rover'} mode")
        self.last_toggle = msg.buttons[self.toggle_button]

        if msg.buttons[self.deadman_button] == 1:
            if self.current_mode == 0:
                twist = Twist()
                twist.linear.x = msg.axes[1]
                twist.angular.z = msg.axes[0]
                twist.linear.y = msg.axes[7]
                twist.linear.z = msg.axes[6]
                self.rover_pub.publish(twist)
            else:
                joint_state = JointState()
                joint_state.name = [f'joint{i+1}' for i in range(7)]  # Names for 7 joints
                joint_state.velocity = [
                    float(msg.axes[7]),  # Joint 1
                    float(msg.axes[6]),  # Joint 2
                    float(msg.axes[4]),  # Joint 3
                    float((1 if msg.buttons[3] else 0) - (1 if msg.buttons[1] else 0)),   # Joint 4
                    float(msg.axes[3]),   # Joint 5
                    float(msg.axes[0]),  # Joint 6
                    float((1 if msg.buttons[0] else 0) - (1 if msg.buttons[1] else 0))  # Joint 7: Positive (button 0) and negative (button 1)
                ]
                joint_state.position = []  # Empty position field
                joint_state.effort = []    # Empty effort field
                self.arm_pub.publish(joint_state)
        return

def main(args=None):
    rclpy.init(args=args)
    node = JoyMuxController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

