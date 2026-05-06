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


# Largest indices used in this node: buttons[12], axes[7] → need lengths 13 and 8.
_JOY_MIN_BUTTONS = 13
_JOY_MIN_AXES = 8


class JoyMuxController(Node):

    def __init__(self):
        super().__init__('joy_mux_controller')

        max_cmd_publish_hz = self.declare_parameter("max_cmd_publish_hz", 100.0).value
        self._skip_identical = self.declare_parameter("skip_identical", True).value
        self._epsilon = self.declare_parameter("identical_epsilon", 1e-3).value

        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.rover_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_pub = self.create_publisher(JointState, '/arm_xyz_cmd', 10)

        period_s = 1.0 / max(0.1, max_cmd_publish_hz)
        self._publish_timer = self.create_timer(period_s, self._tick)

        self.current_mode = 0
        self.last_toggle = 0
        self._deadman_held = False
        self._prev_deadman = False

        # Cached commands built from latest /joy
        self._cached_twist: Twist | None = None
        self._cached_joint: JointState | None = None

        # Last-published payloads for skip_identical
        self._last_twist_vals: tuple[float, ...] | None = None
        self._last_joint_vals: tuple[float, ...] | None = None

        self.get_logger().info(
            f"joy_mux_controller ready — max_cmd_publish_hz={max_cmd_publish_hz}, "
            f"skip_identical={self._skip_identical}, epsilon={self._epsilon}"
        )

    def _publish_all_stop(self) -> None:
        self.rover_pub.publish(Twist())
        stopped = JointState()
        stopped.name = [f'joint{i+1}' for i in range(7)]
        stopped.velocity = [0.0] * 7
        stopped.position = []
        stopped.effort = []
        self.arm_pub.publish(stopped)
        self._last_twist_vals = None
        self._last_joint_vals = None

    def _floats_equal(self, a: tuple[float, ...], b: tuple[float, ...] | None) -> bool:
        if b is None or len(a) != len(b):
            return False
        return all(abs(x - y) < self._epsilon for x, y in zip(a, b))

    def joy_callback(self, msg: Joy):
        if len(msg.buttons) < _JOY_MIN_BUTTONS or len(msg.axes) < _JOY_MIN_AXES:
            self.get_logger().warning(
                (
                    f'Joy message too short (buttons={len(msg.buttons)}, axes={len(msg.axes)}); '
                    f'need at least {_JOY_MIN_BUTTONS} and {_JOY_MIN_AXES}. Publishing stop.'
                ),
                throttle_duration_sec=2.0,
            )
            self._publish_all_stop()
            return

        if msg.buttons[Buttons.TOGGLE] == 1 and self.last_toggle == 0:
            self.current_mode = 1 - self.current_mode
            self._last_twist_vals = None
            self._last_joint_vals = None
            self.get_logger().info(f"Switched to {'Arm' if self.current_mode else 'Rover'} mode")
        self.last_toggle = msg.buttons[Buttons.TOGGLE]

        self._deadman_held = msg.buttons[Buttons.DEADMAN] == 1

        if self._deadman_held:
            if self.current_mode == 0:
                twist = Twist()
                twist.linear.x = msg.axes[Axes.LEFT_STICK_X]
                twist.angular.z = msg.axes[Axes.LEFT_STICK_Y]
                twist.linear.y = msg.axes[Axes.RIGHT_STICK_X]
                twist.linear.z = msg.axes[Axes.RIGHT_STICK_Y]
                self._cached_twist = twist
            else:
                joint_state = JointState()
                joint_state.name = [f'joint{i+1}' for i in range(7)]
                joint_state.velocity = [
                    float(msg.axes[Axes.D_PAD_X]),
                    float(msg.axes[Axes.D_PAD_Y]),
                    float(msg.axes[Axes.RIGHT_STICK_X]),
                    float((1 if msg.buttons[Buttons.X] else 0) - (1 if msg.buttons[Buttons.TRIANGLE] else 0)),
                    float(msg.axes[Axes.LEFT_STICK_Y]),
                    float(msg.axes[Axes.LEFT_STICK_X]),
                    float((1 if msg.buttons[Buttons.CIRCLE] else 0) - (1 if msg.buttons[Buttons.SQUARE] else 0))
                ]
                joint_state.position = []
                joint_state.effort = []
                self._cached_joint = joint_state
        elif self._prev_deadman:
            self._publish_all_stop()

        self._prev_deadman = self._deadman_held

    def _tick(self):
        if not self._deadman_held:
            return

        if self.current_mode == 0:
            tw = self._cached_twist
            if tw is None:
                return
            vals = (tw.linear.x, tw.linear.y, tw.linear.z, tw.angular.z)
            if self._skip_identical and self._floats_equal(vals, self._last_twist_vals):
                return
            self.rover_pub.publish(tw)
            self._last_twist_vals = vals
        else:
            js = self._cached_joint
            if js is None:
                return
            vals = tuple(js.velocity)
            if self._skip_identical and self._floats_equal(vals, self._last_joint_vals):
                return
            self.arm_pub.publish(js)
            self._last_joint_vals = vals


def main(args=None):
    rclpy.init(args=args)
    node = JoyMuxController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
