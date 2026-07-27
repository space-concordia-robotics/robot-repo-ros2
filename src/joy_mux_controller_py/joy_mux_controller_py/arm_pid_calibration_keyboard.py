import os
import select
import sys
import termios
import threading
import tty

import rclpy
from builtin_interfaces.msg import Duration
from rcl_interfaces.msg import Parameter as ParamMsg
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


HELP_TEXT = """
Arm PID calibration (direct arm_controller, no MoveIt Servo)
------------------------------------------------------------
Joint select: 1..6  -> joint1..joint5, joint7 (same mapping as FK keyboard)
Jog velocity: j (negative), l (positive), k (stop)
Speed scale: + / -
PID (selected joint, via ros2_control gains.*):
  [ / ]     decrease / increase P
  ; / '     decrease / increase I   (apostrophe key)
  . / /     decrease / increase D   (period / slash)
  ,         print P I D i_clamp for selected joint
Quit: q

Publishes JointTrajectory to /{controller}/joint_trajectory
Sets parameters on /{controller} (e.g. gains.joint1.p)
"""


class ArmPidCalibrationKeyboard(Node):
    def __init__(self) -> None:
        super().__init__("arm_pid_calibration_keyboard")

        self.declare_parameter("controller_name", "arm_controller")
        self.declare_parameter("p_step", 2.0)
        self.declare_parameter("i_step", 0.05)
        self.declare_parameter("d_step", 1.0)

        self.controller_name = self.get_parameter("controller_name").get_parameter_value().string_value
        self.p_step = self.get_parameter("p_step").get_parameter_value().double_value
        self.i_step = self.get_parameter("i_step").get_parameter_value().double_value
        self.d_step = self.get_parameter("d_step").get_parameter_value().double_value

        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint7"]
        self._positions: dict[str, float] = {}
        self._joint_state_ready = False

        self.selected_joint = 0
        self.speed_scale = 0.15
        self.current_direction = 0.0
        self.running = True
        self._param_busy = False

        traj_topic = f"/{self.controller_name}/joint_trajectory"
        self.traj_pub = self.create_publisher(JointTrajectory, traj_topic, 10)
        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)

        self.set_params_client = self.create_client(
            SetParameters, f"/{self.controller_name}/set_parameters"
        )
        self.get_params_client = self.create_client(
            GetParameters, f"/{self.controller_name}/get_parameters"
        )

        self.publish_timer = self.create_timer(0.05, self.publish_trajectory)

        self.get_logger().info(HELP_TEXT.format(controller=self.controller_name))
        self.get_logger().info(f"Selected joint: {self.joint_names[self.selected_joint]}")
        self.get_logger().info(f"Trajectory topic: {traj_topic}")

        self._input_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self._input_thread.start()

    def _on_joint_states(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            self._positions[name] = float(pos)
        self._joint_state_ready = True

    def publish_trajectory(self) -> None:
        if not self._joint_state_ready:
            return

        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = list(self.joint_names)

        positions = []
        velocities = []
        sel_name = self.joint_names[self.selected_joint]
        for name in self.joint_names:
            positions.append(self._positions.get(name, 0.0))
            if name == sel_name:
                velocities.append(self.current_direction * self.speed_scale)
            else:
                velocities.append(0.0)

        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = velocities
        dt = 0.05
        sec = int(dt)
        nanosec = int((dt - sec) * 1e9)
        point.time_from_start = Duration(sec=sec, nanosec=nanosec)
        msg.points = [point]
        self.traj_pub.publish(msg)

    def _selected_joint_name(self) -> str:
        return self.joint_names[self.selected_joint]

    def _float_param(self, suffix: str, value: float) -> ParamMsg:
        name = f"gains.{self._selected_joint_name()}.{suffix}"
        pv = ParameterValue()
        pv.type = ParameterType.PARAMETER_DOUBLE
        pv.double_value = float(value)
        return ParamMsg(name=name, value=pv)

    def _set_gain_async(self, suffix: str, delta: float) -> None:
        if self._param_busy:
            self.get_logger().warn("Parameter call in progress, wait...")
            return

        jn = self._selected_joint_name()
        pname = f"gains.{jn}.{suffix}"
        if not self.get_params_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().error(f"Service not available: {self.get_params_client.srv_name}")
            return

        req = GetParameters.Request()
        req.names = [pname]
        self._param_busy = True
        future = self.get_params_client.call_async(req)

        def on_get(fut):
            try:
                resp = fut.result()
                if not resp.values:
                    self.get_logger().error(f"No value for {pname}")
                    self._param_busy = False
                    return
                val = resp.values[0]
                if val.type != ParameterType.PARAMETER_DOUBLE:
                    self.get_logger().error(
                        f"{pname} is not a double parameter (type={val.type})"
                    )
                    self._param_busy = False
                    return
                cur = val.double_value
                new_val = cur + delta
                sreq = SetParameters.Request()
                sreq.parameters = [self._float_param(suffix, new_val)]
                fut2 = self.set_params_client.call_async(sreq)

                def on_set(f2):
                    try:
                        sresp = f2.result()
                        ok = sresp.results and sresp.results[0].successful
                        if ok:
                            self.get_logger().info(
                                f"{jn} {suffix}: {cur:.6g} -> {new_val:.6g}"
                            )
                        else:
                            reason = (
                                sresp.results[0].reason if sresp.results else "unknown"
                            )
                            self.get_logger().error(
                                f"set_parameters failed for {pname}: {reason}"
                            )
                    except Exception as exc:
                        self.get_logger().error(f"set_parameters exception: {exc}")
                    finally:
                        self._param_busy = False

                fut2.add_done_callback(on_set)
            except Exception as exc:
                self.get_logger().error(f"get_parameters failed: {exc}")
                self._param_busy = False

        future.add_done_callback(on_get)

    def _print_gains_async(self) -> None:
        if self._param_busy:
            return
        jn = self._selected_joint_name()
        names = [
            f"gains.{jn}.p",
            f"gains.{jn}.i",
            f"gains.{jn}.d",
            f"gains.{jn}.i_clamp",
        ]
        if not self.get_params_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().error("get_parameters not available")
            return
        self._param_busy = True
        req = GetParameters.Request()
        req.names = names
        future = self.get_params_client.call_async(req)

        def on_done(fut):
            self._param_busy = False
            try:
                resp = fut.result()
                if len(resp.values) != len(names):
                    self.get_logger().error("Unexpected get_parameters response")
                    return
                parts = []
                for n, v in zip(names, resp.values):
                    parts.append(f"{n.split('.')[-1]}={v.double_value:.6g}")
                self.get_logger().info(f"{jn}: " + ", ".join(parts))
            except Exception as exc:
                self.get_logger().error(f"get_parameters: {exc}")

        future.add_done_callback(on_done)

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
                    self.get_logger().info(
                        f"Selected joint: {self.joint_names[self.selected_joint]}"
                    )
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
                    self.speed_scale = max(0.01, self.speed_scale - 0.05)
                    self.get_logger().info(f"Speed scale: {self.speed_scale:.2f}")
                elif key == "[":
                    self._set_gain_async("p", -self.p_step)
                elif key == "]":
                    self._set_gain_async("p", self.p_step)
                elif key == ";":
                    self._set_gain_async("i", -self.i_step)
                elif key == "'":
                    self._set_gain_async("i", self.i_step)
                elif key == ".":
                    self._set_gain_async("d", -self.d_step)
                elif key == "/":
                    self._set_gain_async("d", self.d_step)
                elif key == ",":
                    self._print_gains_async()
                elif key == "q":
                    self.get_logger().info("Quit requested")
                    self.running = False
                    rclpy.shutdown()
                    break
        finally:
            termios.tcsetattr(input_fd, termios.TCSADRAIN, old_settings)
            if tty_file is not None:
                tty_file.close()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArmPidCalibrationKeyboard()

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
