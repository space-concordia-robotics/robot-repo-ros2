import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import Twist
from enum import IntEnum

"""Button configurations for VKB joystick."""
class VKBButtonLayout(IntEnum):

    FIRST_FIRE: int = 0
    SECOND_FIRE: int = 1
    A2: int = 2
    B2: int = 3
    D1: int = 4
    A5_UP: int = 5
    A5_RIGHT: int = 6
    A5_DOWN: int = 7
    A5_LEFT: int = 8
    A5_PRESS: int = 9
    A4_UP: int = 10
    A4_RIGHT: int = 11
    A4_DOWN: int = 12
    A4_LEFT: int = 13
    A4_PRESS: int = 14
    C1_UP: int = 15
    C1_RIGHT: int = 16
    C1_DOWN: int = 17
    C1_LEFT: int = 18
    C1_PRESS: int = 19
    TRIGGER_UP: int = 20
    TRIGGER_DOWN: int = 21
    EN1_UP: int = 22
    EN1_DOWN: int = 23
    EN2_UP: int = 24
    EN2_DOWN: int = 25
    F1: int = 26
    F2: int = 27
    F3: int = 28


"""Axes configuration for VKB joystick."""
class VKBAxesLayout(IntEnum):

    STICK_X: int = 0
    STICK_Y: int = 1
    STICK_Z: int = 5
    MIDDLE_SCROLL: int = 2
    A1_X_LED_ON: int = 3
    A1_Y_LED_ON: int = 4
    A1_X_LED_OFF: int = 8
    A1_Y_LED_OFF: int = 9

"""Button configuration for Logitech joystick."""
class LGTButtonLayout(IntEnum):
        
    ONE: int = 0
    TWO: int = 1
    THREE: int = 2
    FOUR: int = 3
    FIVE: int = 4
    SIX: int = 5
    SEVEN: int = 6
    EIGHT: int = 7
    NEIN: int = 8
    TEN: int = 9
    ELEVEN: int = 10
    TWELVE: int = 11

"""Axes configuration for Logitech joystick."""
class LGTAxesLayout(IntEnum):

    STICK_X: int = 0
    STICK_Y: int = 1
    STICK_Z: int = 2
    SWITCH: int = 5
    JOY_X: int = 3
    JOY_Y: int = 4
 




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