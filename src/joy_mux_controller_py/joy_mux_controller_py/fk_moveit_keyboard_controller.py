import os
import select
import sys
import termios
import threading
import tty

import rclpy
from control_msgs.msg import JointJog
from moveit_msgs.srv import ServoCommandType as ServoCommandTypeSrv
from rclpy.node import Node
from std_srvs.srv import Trigger


HELP_TEXT = """
Keyboard FK control for MoveIt Servo
------------------------------------
Select joint: 1..6 (maps to joint1, joint2, joint3, joint4, joint5, joint7)
Velocity: j (negative), l (positive), k (stop)
Speed scale: + increase, - decrease
Quit: q
"""


class FKMoveItKeyboardController(Node):
    def __init__(self) -> None:
        super().__init__("fk_moveit_keyboard_controller")

        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint7"]
        self.command_frame = "base_structure_link"

        self.selected_joint = 0
        self.speed_scale = 0.30
        self.current_direction = 0.0
        self.running = True
        self.servo_started = False
        self.start_request_in_flight = False
        self.command_type_set = False
        self.command_type_request_in_flight = False

        self.joint_pub = self.create_publisher(JointJog, "/servo_node/delta_joint_cmds", 10)
        self.start_servo_client = self.create_client(Trigger, "/servo_node/start_servo")
        self.command_type_client = self.create_client(
            ServoCommandTypeSrv, "/servo_node/switch_command_type"
        )

        self.publish_timer = self.create_timer(0.05, self.publish_joint_jog)
        self.start_timer = self.create_timer(0.5, self.try_start_servo)
        self.command_type_timer = self.create_timer(0.5, self.try_set_command_type)

        self.get_logger().info(HELP_TEXT)
        self.get_logger().info("Selected joint: joint1")

        self._input_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self._input_thread.start()

    def try_start_servo(self) -> None:
        if self.servo_started or self.start_request_in_flight:
            return

        if not self.start_servo_client.service_is_ready():
            return

        request = Trigger.Request()
        self.start_request_in_flight = True
        future = self.start_servo_client.call_async(request)
        future.add_done_callback(self._on_servo_start_response)

    def _on_servo_start_response(self, future) -> None:
        try:
            response = future.result()
            self.servo_started = bool(response.success)
            if self.servo_started:
                self.get_logger().info("MoveIt Servo started")
            else:
                self.get_logger().warn(f"MoveIt Servo start request failed: {response.message}")
        except Exception as exc:
            self.get_logger().warn(f"MoveIt Servo start call failed: {exc}")
        finally:
            self.start_request_in_flight = False

    def try_set_command_type(self) -> None:
        if self.command_type_set or self.command_type_request_in_flight:
            return

        if not self.command_type_client.service_is_ready():
            self.get_logger().debug("switch_command_type service not ready yet", throttle_duration_sec=2.0)
            return

        request = ServoCommandTypeSrv.Request()
        request.command_type = ServoCommandTypeSrv.Request.JOINT_JOG
        self.command_type_request_in_flight = True
        future = self.command_type_client.call_async(request)
        future.add_done_callback(self._on_command_type_response)

    def _on_command_type_response(self, future) -> None:
        try:
            response = future.result()
            self.command_type_set = bool(response.success)
            if self.command_type_set:
                self.get_logger().info("MoveIt Servo command type set to JOINT_JOG")
            else:
                self.get_logger().warn(
                    f"Failed to set MoveIt Servo command type: {response.message}"
                )
        except Exception as exc:
            self.get_logger().warn(f"MoveIt Servo command type call failed: {exc}")
        finally:
            self.command_type_request_in_flight = False

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

                if key in "123456":
                    self.selected_joint = int(key) - 1
                    name = self.joint_names[self.selected_joint]
                    self.get_logger().info(f"Selected joint: {name}")
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

    def publish_joint_jog(self) -> None:
        msg = JointJog()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.command_frame
        msg.joint_names = list(self.joint_names)
        msg.velocities = [0.0] * len(self.joint_names)
        msg.velocities[self.selected_joint] = self.current_direction * self.speed_scale
        msg.duration = 0.1
        self.joint_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FKMoveItKeyboardController()

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
