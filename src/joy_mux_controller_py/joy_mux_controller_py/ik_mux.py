import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import TwistStamped, Twist
from control_msgs.msg import JointJog
from std_msgs.msg import Float32

#Mapping Joystick button binds:
X = 0
SQUARE = 3
TRIANGLE = 2
CIRCLE = 1
RBUMPER = 5

#Mapping Axis binds: 

LEFT_STICK_X = 0
LEFT_STICK_Y = 1
RIGHT_STICK_X = 3
RIGHT_STICK_Y = 4
D_PAD_X = 6
D_PAD_Y = 7
LTRIGGER = 2
RTRIGGER = 5

AXIS_DEFAULTS = 1.0 # Default value for triggers when not pressed

def convertJoytoCmd(joy_msg: Joy):
   
    if joy_msg.buttons[X] or joy_msg.buttons[SQUARE] or joy_msg.buttons[TRIANGLE] or joy_msg.buttons[CIRCLE] or joy_msg.axes[D_PAD_X] or joy_msg.axes[D_PAD_Y]:
        # If any of the buttons or D-Pad axes are pressed, we assume it's a joint jog command
        # and not a twist command.

        joint = JointJog()
        joint.joint_names = [
        'base_pivot_shoulder_gearbox_joint',
        'bicep_tube_gearbox_joint',
        'forearm_tube_wrist_gearbox_joint',
        'gripper_claw_joint'
        ]
        joint.velocities = [
            float(joy_msg.axes[D_PAD_X]), 
            float(joy_msg.axes[D_PAD_Y]),
            float(joy_msg.buttons[X] - joy_msg.buttons[TRIANGLE]),
            float(joy_msg.buttons[SQUARE] - joy_msg.buttons[CIRCLE])
            ]
        return True, None, joint 
    #Setting this flag to False tells the node not to publish a joint jog commands.
    #This is because were returning a 3-tuple, and the publish flag is associated to either True/False
        
    twist_stamped = TwistStamped()
    twist_stamped.twist.linear.z = joy_msg.axes[LEFT_STICK_Y]
    twist_stamped.twist.linear.y = joy_msg.axes[LEFT_STICK_X]
    
    lin_x_right = -0.5*(joy_msg.axes[RTRIGGER] - AXIS_DEFAULTS)
    lin_x_left = 0.5*(joy_msg.axes[LTRIGGER] - AXIS_DEFAULTS)
    twist_stamped.twist.linear.x = lin_x_right + lin_x_left

    return True, twist_stamped , None


class JoyMuxController(Node):
    def __init__(self):
        super().__init__('joy_mux_controller')
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.rover_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_pub = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10) 
        self.jog_pub = self.create_publisher(JointJog, '/servo_node/delta_joint_cmds', 10)

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
                twist.linear.x = msg.axes[0]
                twist.angular.z = msg.axes[1]
                twist.linear.y = msg.axes[7]
                twist.linear.z = msg.axes[6]
                self.rover_pub.publish(twist)
            else:
                publish, twist_stamped, joint = convertJoytoCmd(msg)
                if publish and twist_stamped is not None:
                    twist_stamped.header.stamp = self.get_clock().now().to_msg()
                    twist_stamped.header.frame_id = 'base_structure_link'
                    self.arm_pub.publish(twist_stamped)
                elif publish and joint is not None:
                    joint.header.stamp = self.get_clock().now().to_msg()
                    joint.header.frame_id = 'base_structure_link'
                    self.jog_pub.publish(joint)     
        return

def main(args=None):
    rclpy.init(args=args)
    node = JoyMuxController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()