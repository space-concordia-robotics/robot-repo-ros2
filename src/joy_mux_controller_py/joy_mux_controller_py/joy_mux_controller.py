import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import Twist
from enum import IntEnum


class Buttons(IntEnum):
        
    DEADMAN = 4
    TOGGLE = 10
    X = 0
    CIRCLE = 1
    TRIANGLE = 2
    SQUARE = 3
    LEFT_BUMPER = 4
    RIGHT_BUMPER = 5
    CHANGE_VIEW = 6
    SHARE = 8
    HOME = 10
    LEFT_STICK_CLICK = 11
    RIGHT_STICK_CLICK = 12

class Axes(IntEnum):

    LEFT_STICK_X = 0
    LEFT_STICK_Y = 1
    LEFT_TRIGGER = 2
    RIGHT_TRIGGER = 5
    RIGHT_STICK_X = 3
    RIGHT_STICK_Y = 4
    D_PAD_X = 6
    D_PAD_Y = 7


class JoyMuxController(Node):


    def __init__(self):
        super().__init__('joy_mux_controller')
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.rover_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_pub = self.create_publisher(JointState, '/arm_xyz_cmd', 10)  # Changed to JointState
        self.current_mode = 0
        self.last_toggle = 0
    

    def joy_callback(self, msg: Joy):
        
        if msg.buttons[Buttons.TOGGLE] == 1 and self.last_toggle == 0:
            self.current_mode = 1 - self.current_mode
            self.get_logger().info(f"Switched to {'Arm' if self.current_mode else 'Rover'} mode")
        self.last_toggle = msg.buttons[Buttons.TOGGLE]

        if msg.buttons[Buttons.DEADMAN] == 1:
            if self.current_mode == 0:
                twist = Twist()
                twist.linear.x = msg.axes[Axes.LEFT_STICK_X]
                twist.angular.z = msg.axes[Axes.LEFT_STICK_Y]
                twist.linear.y = msg.axes[Axes.RIGHT_STICK_X]
                twist.linear.z = msg.axes[Axes.RIGHT_STICK_Y]
                self.rover_pub.publish(twist)
            else:
                joint_state = JointState()
                joint_state.name = [f'joint{i+1}' for i in range(7)]  # Names for 7 joints
                joint_state.velocity = [
                    float(msg.axes[Axes.D_PAD_X]),  # Joint 1
                    float(msg.axes[Axes.D_PAD_Y]),  # Joint 2
                    float(msg.axes[Axes.RIGHT_STICK_X]),  # Joint 3
                    float((1 if msg.buttons[Buttons.X] else 0) - (1 if msg.buttons[Buttons.TRIANGLE] else 0)),   # Joint 4
                    float(msg.axes[Axes.LEFT_STICK_Y]),   # Joint 5
                    float(msg.axes[Axes.LEFT_STICK_X]),  # Joint 6
                    float((1 if msg.buttons[Buttons.CIRCLE] else 0) - (1 if msg.buttons[Buttons.SQUARE] else 0))  # Joint 7: Positive (button 0) and negative (button 1)
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
