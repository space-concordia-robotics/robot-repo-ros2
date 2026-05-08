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

        # Zero-burst: keep publishing stop commands for this duration after deadman release
        self._stop_burst_until: float = 0.0
        self._stop_burst_duration_s = self.declare_parameter(
            "stop_burst_duration_s", 0.5
        ).value
        # Keep button-driven arm commands active briefly to improve tap reliability.
        self._arm_button_min_hold_s = self.declare_parameter(
            "arm_button_min_hold_s", 0.08
        ).value
        self._m4_latched_cmd = 0.0
        self._m4_hold_until = 0.0

        self.get_logger().info(
            f"joy_mux_controller ready — max_cmd_publish_hz={max_cmd_publish_hz}, "
            f"skip_identical={self._skip_identical}, epsilon={self._epsilon}, "
            f"arm_button_min_hold_s={self._arm_button_min_hold_s}"
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
                # Rotate joystick mapping to correct rover cardinal directions.
                twist.linear.x = msg.axes[Axes.LEFT_STICK_Y]
                twist.angular.z = msg.axes[Axes.LEFT_STICK_X]
                tank_turn = (1 if msg.buttons[Buttons.SQUARE] else 0) - (1 if msg.buttons[Buttons.CIRCLE] else 0)
                if tank_turn != 0:
                    # Tank turn override: spin in place with opposite wheel directions.
                    twist.linear.x = 0.0
                    twist.angular.z = float(tank_turn)
                self._cached_twist = twist
            else:
                joint_state = JointState()
                joint_state.name = [f'joint{i+1}' for i in range(7)]
                now_s = self.get_clock().now().nanoseconds * 1e-9
                m4_raw = float((1 if msg.buttons[Buttons.CIRCLE] else 0) - (1 if msg.buttons[Buttons.SQUARE] else 0))
                if m4_raw != 0.0:
                    self._m4_latched_cmd = m4_raw
                    self._m4_hold_until = now_s + self._arm_button_min_hold_s
                elif now_s < self._m4_hold_until:
                    m4_raw = self._m4_latched_cmd
                else:
                    self._m4_latched_cmd = 0.0
                m4 = m4_raw
                joint_state.velocity = [
                    float(msg.axes[Axes.D_PAD_X]),
                    float(msg.axes[Axes.D_PAD_Y]),
                    float(msg.axes[Axes.RIGHT_STICK_Y]),
                    m4,
                    float(msg.axes[Axes.LEFT_STICK_Y]),
                    float((1 if msg.buttons[Buttons.TRIANGLE] else 0) - (1 if msg.buttons[Buttons.X] else 0)),
                    float((1 if msg.buttons[Buttons.SQUARE] else 0) - (1 if msg.buttons[Buttons.CIRCLE] else 0)),
                ]
                joint_state.position = []
                joint_state.effort = []
                self._cached_joint = joint_state
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

        if self.current_mode == 0:
            tw = self._cached_twist
            if tw is None:
                return
            vals = (tw.linear.x, tw.linear.y, tw.linear.z, tw.angular.z)
            # Wheels benefit from a steady frame stream while deadman is held.
            self.rover_pub.publish(tw)
            self._last_twist_vals = vals
        else:
            js = self._cached_joint
            if js is None:
                return
            vals = tuple(js.velocity)
            # Arm button control is more reliable when we continuously stream held/release values.
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
