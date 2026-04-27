import select
import sys
import termios
import threading
import tty
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


HELP_TEXT = """
Keyboard FK control
-------------------
Use keys 1..7 to select a joint.
Use j/l to command negative/positive velocity for the selected joint.
Use k to stop all joints.
Use +/- to decrease/increase speed scale.
Use q to quit.
"""


class FKKeyboardController(Node):
    def __init__(self) -> None:
        super().__init__("fk_keyboard_controller")

        self.publisher = self.create_publisher(JointState, "/arm_xyz_cmd", 10)
        self.timer = self.create_timer(0.05, self.publish_joint_command)

        self.selected_joint = 0
        self.speed_scale = 0.25
        self.current_direction = 0.0
        self.running = True

        self.get_logger().info(HELP_TEXT)
        self.get_logger().info("Selected joint: 1")

        self._input_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self._input_thread.start()

    def keyboard_loop(self) -> None:
        input_fd = None
        tty_file = None

        if sys.stdin.isatty():
            input_fd = sys.stdin.fileno()
        else:
            try:
                tty_file = open("/dev/tty", "rb", buffering=0)
                input_fd = tty_file.fileno()
            except OSError:
                self.get_logger().error("No interactive TTY available for keyboard input")
                self.running = False
                rclpy.shutdown()
                return

        old_settings = termios.tcgetattr(input_fd)

        try:
            tty.setcbreak(input_fd)
            while rclpy.ok() and self.running:
                ready, _, _ = select.select([input_fd], [], [], 0.1)
                if not ready:
                    continue

                key = os.read(input_fd, 1).decode(errors="ignore")
                if not key:
                    continue

                if key in "1234567":
                    self.selected_joint = int(key) - 1
                    self.get_logger().info(f"Selected joint: {self.selected_joint + 1}")
                elif key == "j":
                    self.current_direction = -1.0
                elif key == "l":
                    self.current_direction = 1.0
                elif key == "k":
                    self.current_direction = 0.0
                elif key in ["+", "="]:
                    self.speed_scale = min(1.0, self.speed_scale + 0.05)
                    self.get_logger().info(f"Speed scale: {self.speed_scale:.2f}")
                elif key in ["-", "_"]:
                    self.speed_scale = max(0.05, self.speed_scale - 0.05)
                    self.get_logger().info(f"Speed scale: {self.speed_scale:.2f}")
                elif key == "q":
                    self.get_logger().info("Quit requested from keyboard")
                    self.running = False
                    rclpy.shutdown()
                    break
        finally:
            termios.tcsetattr(input_fd, termios.TCSADRAIN, old_settings)
            if tty_file is not None:
                tty_file.close()

    def publish_joint_command(self) -> None:
        msg = JointState()
        msg.name = [f"joint{i + 1}" for i in range(7)]
        msg.velocity = [0.0] * 7
        msg.velocity[self.selected_joint] = self.current_direction * self.speed_scale
        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FKKeyboardController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
