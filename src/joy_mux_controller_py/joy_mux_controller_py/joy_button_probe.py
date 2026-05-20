"""joy_button_probe — calibrate joystick button indices against VKBButtonLayout.

Subscribes to /joy and prints a line for each button rising edge:

    [INFO] buttons[25] PRESSED  ->  VKBButtonLayout at index 25: EN2_DOWN
    [INFO] buttons[26] PRESSED  ->  VKBButtonLayout at index 26: F1

If no enum member is defined for an index, it prints UNKNOWN so you can identify
buttons that are not yet in VKBButtonLayout.

Usage:
    ros2 run joy joy_node
    ros2 run joy_mux_controller_py joy_button_probe

    Press one button at a time and read the printed index and name.
    Use the confirmed indices when overriding can_safety_node params:

    ros2 launch can_safety_node safety.launch.py \\
        wheel_force_stop_button:=<F1_index> \\
        wheel_resume_button:=<F2_index>
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

from .vkb_layout import VKBButtonLayout


def _build_reverse_map() -> dict[int, str]:
    return {member.value: member.name for member in VKBButtonLayout}


class JoyButtonProbe(Node):

    def __init__(self):
        super().__init__('joy_button_probe')
        self._reverse_map: dict[int, str] = _build_reverse_map()
        self._prev_buttons: list[int] = []

        self._sub = self.create_subscription(
            Joy, '/joy', self._on_joy, 10)

        self.get_logger().info(
            'joy_button_probe ready — press any button to see its index and '
            'VKBButtonLayout name. Use Ctrl+C to exit.'
        )

    def _on_joy(self, msg: Joy) -> None:
        current = list(msg.buttons)
        n = len(current)

        if len(self._prev_buttons) != n:
            self._prev_buttons = [0] * n

        for i in range(n):
            if current[i] == 1 and self._prev_buttons[i] == 0:
                name = self._reverse_map.get(i, 'UNKNOWN (not in VKBButtonLayout)')
                self.get_logger().info(
                    f'buttons[{i}] PRESSED  ->  VKBButtonLayout at index {i}: {name}'
                )

        self._prev_buttons = current


def main(args=None):
    rclpy.init(args=args)
    node = JoyButtonProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
