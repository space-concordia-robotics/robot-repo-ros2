#!/usr/bin/env python3
"""Teleop for SIL board LEDs — presets and custom R/G/B/brightness input."""

import sys
import rclpy
from rclpy.node import Node
from sil_board.msg import LedCommand

PRESETS = {
    "white":  (255, 255, 255, 255),
    "red":    (255, 0,   0,   255),
    "green":  (0,   255, 0,   255),
    "blue":   (0,   0,   255, 255),
    "yellow": (255, 255, 0,   255),
    "cyan":   (0,   255, 255, 255),
    "purple": (255, 0,   255, 255),
    "off":    (0,   0,   0,   0),
}

HELP_TEXT = """
SIL Board LED Teleop
--------------------
Presets (type the name and press Enter):
  white, red, green, blue, yellow, cyan, purple, off

Custom (type four space-separated values 0-255):
  R G B BRIGHTNESS
  e.g.  128 0 255 200

Type 'q' or Ctrl-C to quit.
"""


class SilBoardTeleop(Node):
    def __init__(self):
        super().__init__("sil_board_teleop")
        self.pub = self.create_publisher(LedCommand, "/sil_board/rgb", 10)

    def send(self, r: int, g: int, b: int, brightness: int):
        msg = LedCommand()
        msg.r = self._clamp(r)
        msg.g = self._clamp(g)
        msg.b = self._clamp(b)
        msg.brightness = self._clamp(brightness)
        self.pub.publish(msg)
        self.get_logger().info(
            f"Published LED: R={msg.r} G={msg.g} B={msg.b} BR={msg.brightness}"
        )

    @staticmethod
    def _clamp(v: int) -> int:
        return max(0, min(255, int(v)))


def main(args=None):
    rclpy.init(args=args)
    node = SilBoardTeleop()
    print(HELP_TEXT)

    try:
        while rclpy.ok():
            try:
                line = input("> ").strip().lower()
            except EOFError:
                break

            if not line:
                continue
            if line in ("q", "quit", "exit"):
                break

            if line in PRESETS:
                node.send(*PRESETS[line])
                continue

            parts = line.replace(",", " ").split()
            if len(parts) == 4:
                try:
                    vals = [int(p) for p in parts]
                    node.send(*vals)
                    continue
                except ValueError:
                    pass

            print(f"Unknown input '{line}'. Type a preset name or R G B BRIGHTNESS.")
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
