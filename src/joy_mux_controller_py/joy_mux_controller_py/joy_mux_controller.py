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
    A3_UP = 5
    A3_RIGHT = 6
    A3_DOWN = 7
    A3_LEFT = 8
    A3_PRESS = 9
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


_JOY_MIN_BUTTONS = 13
_JOY_MIN_AXES = 8
_ARM_JOINT_COUNT = 7


class ArmVelocityScale:
    """Max velocity scale per arm motor (normalized stick/button input in [-1, 1])."""

    M1_STICK_Z = 1.0
    M2_A3_VERTICAL = 0.4
    M3_STICK_Y = 0.7
    M4_STICK_X = 0.7
    M5_A3_HORIZONTAL = 0.8


class ThrottleAxisMap:
    """VKB middle scroll (axis 2) -> A3 speed multiplier for M2/M5."""

    AXIS_MIN = -1.0
    AXIS_MAX = 1.0
    MULT_MIN = 0.2
    MULT_MAX = 1.0


def _throttle_multiplier(axis: float) -> float:
    axis_span = ThrottleAxisMap.AXIS_MAX - ThrottleAxisMap.AXIS_MIN
    t = (axis - ThrottleAxisMap.AXIS_MIN) / axis_span
    mult = ThrottleAxisMap.MULT_MIN + t * (
        ThrottleAxisMap.MULT_MAX - ThrottleAxisMap.MULT_MIN
    )
    return max(ThrottleAxisMap.MULT_MIN, min(ThrottleAxisMap.MULT_MAX, mult))


class JoyMuxController(Node):

    def __init__(self):
        super().__init__('joy_mux_controller')

        max_cmd_publish_hz = self.declare_parameter("max_cmd_publish_hz", 100.0).value

        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.rover_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_pub = self.create_publisher(JointState, '/arm_xyz_cmd', 10)

        period_s = 1.0 / max(0.1, max_cmd_publish_hz)
        self._publish_timer = self.create_timer(period_s, self._tick)

        self.current_mode = 0
        self.last_toggle = 0
        self._last_mode_toggle_at_s = -1e9
        self._mode_toggle_cooldown_s = self.declare_parameter(
            "mode_toggle_cooldown_s", 0.35
        ).value
        self._deadman_held = False
        self._prev_deadman = False

        self._cached_twist: Twist | None = None
        self._cached_joint: JointState | None = None

        self._stop_burst_until: float = 0.0
        self._stop_burst_duration_s = self.declare_parameter(
            "stop_burst_duration_s", 0.5
        ).value

        # Mode-switch stop window: publish zeros on the mode we just left so
        # firmware without a watchdog actually halts.
        self._mode_switch_stop_until: float = 0.0
        self._mode_switch_stop_duration_s = self.declare_parameter(
            "mode_switch_stop_duration_s", 0.5
        ).value

        self._arm_button_min_hold_s = self.declare_parameter(
            "arm_button_min_hold_s", 0.08
        ).value
        self._m4_latched_cmd = 0.0
        self._m4_hold_until = 0.0

        self._rover_boost_trigger_up = self.declare_parameter(
            "rover_boost_trigger_up", 1.5
        ).value
        self._rover_boost_trigger_down = self.declare_parameter(
            "rover_boost_trigger_down", 2.0
        ).value

        self.get_logger().info(
            f"joy_mux_controller ready — max_cmd_publish_hz={max_cmd_publish_hz}, "
            f"stop_burst_duration_s={self._stop_burst_duration_s}, "
            f"mode_switch_stop_duration_s={self._mode_switch_stop_duration_s}, "
            f"mode_toggle_cooldown_s={self._mode_toggle_cooldown_s}, "
            f"arm_button_min_hold_s={self._arm_button_min_hold_s}, "
            f"rover_boost_trigger_up={self._rover_boost_trigger_up}, "
            f"rover_boost_trigger_down={self._rover_boost_trigger_down}"
        )

    def _publish_all_stop(self) -> None:
        self.rover_pub.publish(Twist())
        stopped = JointState()
        stopped.name = [f'joint{i+1}' for i in range(_ARM_JOINT_COUNT)]
        stopped.velocity = [0.0] * _ARM_JOINT_COUNT
        stopped.position = []
        stopped.effort = []
        self.arm_pub.publish(stopped)

    def _rover_boost(self, buttons) -> float:
        if buttons[VKBButtonLayout.TRIGGER_DOWN]:
            return self._rover_boost_trigger_down
        if buttons[VKBButtonLayout.TRIGGER_UP]:
            return self._rover_boost_trigger_up
        return 1.0

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

        now_s = self.get_clock().now().nanoseconds * 1e-9
        home_down = msg.buttons[VKBButtonLayout.A2] == 1
        home_rising = home_down and self.last_toggle == 0
        if (
            home_rising
            and (now_s - self._last_mode_toggle_at_s) >= self._mode_toggle_cooldown_s
        ):
            self.current_mode = 1 - self.current_mode
            self._last_mode_toggle_at_s = now_s
            self._cached_twist = None
            self._cached_joint = None
            self._m4_latched_cmd = 0.0
            self._m4_hold_until = 0.0
            self._mode_switch_stop_until = now_s + self._mode_switch_stop_duration_s
            self._publish_all_stop()
            self.get_logger().info(f"Switched to {'Arm' if self.current_mode else 'Rover'} mode")
        self.last_toggle = 1 if home_down else 0

        self._deadman_held = msg.buttons[VKBButtonLayout.D1] == 1

        if self._deadman_held:
            if self.current_mode == 0:
                twist = Twist()
                twist.linear.x = msg.axes[VKBAxesLayout.STICK_Y]
                twist.angular.z = msg.axes[VKBAxesLayout.STICK_Z]
                tank_turn = (1 if msg.buttons[VKBButtonLayout.A4_LEFT] else 0) - (1 if msg.buttons[VKBButtonLayout.A4_RIGHT] else 0)
                if tank_turn != 0:
                    twist.linear.x = 0.0
                    twist.angular.z = float(tank_turn)
                boost = self._rover_boost(msg.buttons)
                twist.linear.x *= boost
                twist.angular.z *= boost
                self._cached_twist = twist
            else:
                joint_state = JointState()
                joint_state.name = [f'joint{i+1}' for i in range(_ARM_JOINT_COUNT)]
                m4_raw = float(msg.axes[VKBAxesLayout.STICK_X])
                if m4_raw != 0.0:
                    self._m4_latched_cmd = m4_raw
                    self._m4_hold_until = now_s + self._arm_button_min_hold_s
                elif now_s < self._m4_hold_until:
                    m4_raw = self._m4_latched_cmd
                else:
                    self._m4_latched_cmd = 0.0
                a3_throttle = _throttle_multiplier(
                    float(msg.axes[VKBAxesLayout.MIDDLE_SCROLL])
                )
                joint_state.velocity = [
                    float(msg.axes[VKBAxesLayout.STICK_Z]) * ArmVelocityScale.M1_STICK_Z,
                    float(msg.buttons[VKBButtonLayout.A3_UP] - msg.buttons[VKBButtonLayout.A3_DOWN])
                    * ArmVelocityScale.M2_A3_VERTICAL * a3_throttle,
                    -float(msg.axes[VKBAxesLayout.STICK_Y]) * ArmVelocityScale.M3_STICK_Y,
                    m4_raw * ArmVelocityScale.M4_STICK_X,
                    float(msg.buttons[VKBButtonLayout.A3_LEFT] - msg.buttons[VKBButtonLayout.A3_RIGHT])
                    * ArmVelocityScale.M5_A3_HORIZONTAL * a3_throttle,
                    0.0,
                    0.0,
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
        in_mode_switch_stop = now_s < self._mode_switch_stop_until

        if in_mode_switch_stop:
            if self.current_mode == 0:
                stopped = JointState()
                stopped.name = [f'joint{i+1}' for i in range(_ARM_JOINT_COUNT)]
                stopped.velocity = [0.0] * _ARM_JOINT_COUNT
                stopped.position = []
                stopped.effort = []
                self.arm_pub.publish(stopped)
            else:
                self.rover_pub.publish(Twist())

        if not self._deadman_held:
            if in_stop_burst:
                self._publish_all_stop()
            return

        if self.current_mode == 0:
            if self._cached_twist is not None:
                self.rover_pub.publish(self._cached_twist)
        else:
            if self._cached_joint is not None:
                self.arm_pub.publish(self._cached_joint)


def main(args=None):
    rclpy.init(args=args)
    node = JoyMuxController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
