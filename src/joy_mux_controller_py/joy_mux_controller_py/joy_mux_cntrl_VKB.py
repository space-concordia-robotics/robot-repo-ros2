import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import Twist
from enum import IntEnum

"""Button configurations for VKB joystick."""
class VKBButtonLayout(IntEnum):

    FIRST_FIRE = 0
    SECOND_FIRE = 1
    A2 = 2
    B2 = 3
    D1 = 4
    A5_UP = 5
    A5_RIGHT = 6
    A5_DOWN = 7
    A5_LEFT = 8
    A5_PRESS = 9
    A4_UP = 10
    A4_RIGHT = 11
    A4_DOWN = 12
    A4_LEFT = 13
    A4_PRESS = 14
    C1_UP = 15
    C1_RIGHT = 16
    C1_DOWN = 17
    C1_LEFT = 18
    C1_PRESS = 19
    TRIGGER_UP = 20
    TRIGGER_DOWN = 21
    EN1_UP = 22
    EN1_DOWN = 23
    EN2_UP = 24
    EN2_DOWN = 25
    F1 = 26
    F2 = 27
    F3 = 28


"""Axes configuration for VKB joystick."""
class VKBAxesLayout(IntEnum):

    STICK_X = 0
    STICK_Y = 1
    STICK_Z = 5
    MIDDLE_SCROLL = 2
    A1_X_LED_ON = 3
    A1_Y_LED_ON = 4
    A1_X_LED_OFF = 8
    A1_Y_LED_OFF = 9

"""Button configuration for Logitech joystick."""
class LGTButtonLayout(IntEnum):
        
    ONE = 0
    TWO = 1
    THREE = 2
    FOUR = 3
    FIVE = 4
    SIX = 5
    SEVEN = 6
    EIGHT = 7
    NEIN = 8
    TEN = 9
    ELEVEN = 10
    TWELVE = 11

"""Axes configuration for Logitech joystick."""
class LGTAxesLayout(IntEnum):

    STICK_X = 0
    STICK_Y = 1
    STICK_Z = 2
    SWITCH = 5
    JOY_X = 3
    JOY_Y = 4
 




# Largest indices used in this node: buttons[12], axes[7] -> need lengths 13 and 8.
_JOY_MIN_BUTTONS: int = 13
_JOY_MIN_AXES: int = 8
_ARM_JOINT_COUNT: int = 7
_MODE_WHEELS: int = 0
_MODE_ARM: int = 1


class JoyMuxController(Node):

    def __init__(self):
        super().__init__('joy_mux_controller')

        max_cmd_publish_hz = self.declare_parameter("max_cmd_publish_hz", 100.0).value
        self._skip_identical = self.declare_parameter("skip_identical", True).value
        self._epsilon = self.declare_parameter("identical_epsilon", 1e-3).value

        self.VKB_subscription = self.create_subscription(Joy, '/joy_VKB', self.joy_callback, 10)
        self.LGT_subscription = self.create_subscription(Joy, '/joy_LGT', self.joy_callback, 10)
        self.rover_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_pub = self.create_publisher(JointState, '/arm_xyz_cmd', 10)

        period_s = 1.0 / max(0.1, max_cmd_publish_hz)
        self._publish_timer = self.create_timer(period_s, self._tick)

        self.current_mode = _MODE_WHEELS
        self.last_toggle_wheels = 0
        self.last_toggle_arm = 0
        self._deadman_held = False
        self._prev_deadman = False

        # Cached commands built from latest /joy
        self._cached_twist: Twist | None = None
        self._cached_joint: JointState | None = None

        # Last-published payloads for skip_identical
        self._last_twist_vals: tuple[float, ...] | None = None
        self._last_joint_vals: tuple[float, ...] | None = None

        # Zero-burst: keep publishing stop commands for this duration after deadman release
        self._stop_burst_until: float = 0.0
        self._stop_burst_duration_s = self.declare_parameter(
            "stop_burst_duration_s", 0.5
        ).value

        self.get_logger().info(
            "joy_mux_controller ready - max_cmd_publish_hz=%s, skip_identical=%s, epsilon=%s",
            max_cmd_publish_hz,
            self._skip_identical,
            self._epsilon,
        )

    def _set_mode(self, mode: int, label: str) -> None:
        if self.current_mode != mode:
            self.current_mode = mode
            self._last_twist_vals = None
            self._last_joint_vals = None
            self.get_logger().info("%s", label)

    def _publish_all_stop(self) -> None:
        self.rover_pub.publish(Twist())
        stopped = JointState()
        stopped.name = [f'joint{i+1}' for i in range(_ARM_JOINT_COUNT)]
        stopped.velocity = [0.0] * _ARM_JOINT_COUNT
        stopped.position = []
        stopped.effort = []
        self.arm_pub.publish(stopped)
        self._last_twist_vals = None
        self._last_joint_vals = None

    def _build_twist(self, msg: Joy) -> Twist:
        twist = Twist()
        twist.linear.x = msg.axes[LGTAxesLayout.STICK_Y]
        twist.angular.z = msg.axes[LGTAxesLayout.STICK_Z]
        return twist

    def _build_joint_state(self, msg: Joy) -> JointState:
        joint_state = JointState()
        joint_state.name = [f'joint{i+1}' for i in range(_ARM_JOINT_COUNT)]
        joint_state.velocity = [
            float(msg.axes[LGTAxesLayout.STICK_Z]),
            float(msg.axes[LGTAxesLayout.STICK_Y]),
            float(msg.axes[VKBAxesLayout.STICK_Y]),
            float(msg.axes[VKBAxesLayout.STICK_X]),
            float(msg.axes[VKBAxesLayout.STICK_Z]),
            float(msg.axes[LGTAxesLayout.STICK_X]),
            float(msg.axes[VKBAxesLayout.A1_Y_LED_ON]),
        ]
        joint_state.position = []
        joint_state.effort = []
        return joint_state

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

        if msg.buttons[VKBButtonLayout.F1] == 1 and self.last_toggle_wheels == 0:
            self._set_mode(_MODE_WHEELS, "Wheel Mode Activated")
        self.last_toggle_wheels = msg.buttons[VKBButtonLayout.F1]

        if msg.buttons[VKBButtonLayout.F2] == 1 and self.last_toggle_arm == 0:
            self._set_mode(_MODE_ARM, "Arm Mode Activated")
        self.last_toggle_arm = msg.buttons[VKBButtonLayout.F2]
        
        self._deadman_held = msg.buttons[LGTButtonLayout.ONE] == 1

        if self._deadman_held:
            if self.current_mode == _MODE_WHEELS:
                self._cached_twist = self._build_twist(msg)
            else:
                self._cached_joint = self._build_joint_state(msg)
        elif self._prev_deadman:
            self._stop_burst_until = (
                self.get_clock().now().nanoseconds * 1e-9
                + self._stop_burst_duration_s
            )
            self._publish_all_stop()

        self._prev_deadman = self._deadman_held

    def _tick(self):
        now_s = self.get_clock().now().nanoseconds * 1e-9
        in_stop_burst = now_s < self._stop_burst_until

        if not self._deadman_held:
            if in_stop_burst:
                self._publish_all_stop()
            return

        if self.current_mode == _MODE_WHEELS:
            tw = self._cached_twist
            if tw is None:
                return
            vals = (tw.linear.x, tw.angular.z)
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