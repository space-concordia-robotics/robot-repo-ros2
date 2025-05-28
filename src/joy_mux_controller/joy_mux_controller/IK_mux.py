import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import TwistStamped, Twist
from control_msgs.msg import JointJog
from std_msgs.msg import Float32

class JoyMuxController(Node):
    def __init__(self):
        super().__init__('joy_mux_controller')
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.rover_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_pub = self.create_publisher(TwistStamped, '/arm_xyz_cmd', 10) 
        #self.jog_pub = self.create_publisher(JointJog, '/joint_jog_cmd', 10)

        self.deadman_button = 4
        self.toggle_button = 12
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
                stamp = TwistStamped()
                stamp.header.stamp = self.get_clock().now().to_msg()
                stamp.header.frame_id = "base_structure_link"
                stamp.twist.linear.x = msg.axes[0]
                stamp.twist.linear.y = msg.axes[1]
                stamp.twist.linear.z = msg.axes[5]
                self.arm_pub.publish(stamp)

                
                #jog = JointJog()
                #jog.header.frame_id = "base_structure_link"
                #jog.header.stamp = self.get_clock().now()
                #jog.joint_names = ['base_pivot_shoulder_gearbox_joint', 'base_structure_joint', 'bicep_tube_gearbox_joint', 'forearm_tube_wrist_gearbox_joint', 'gripper_claw_joint']
                #jog.velocities = [
                #   msg.axes[0],
                #   msg.axes[1],
                #   msg.axes[2],
                #   msg.axes[3],
                #   msg.axes[4]
                #]
                #self.jog_pub.publish(jog)
        return

def main(args=None):
    rclpy.init(args=args)
    node = JoyMuxController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()