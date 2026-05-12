#!/usr/bin/env python3
"""Joystick-driven LED teleop for SIL board — presets, patterns, rate-limited."""

import math
import sys
import time
from enum import IntEnum
from pathlib import Path

_script_dir = str(Path(__file__).resolve().parent)
if _script_dir not in sys.path:
    sys.path.insert(0, _script_dir)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from sil_board.msg import LedCommand
from sil_board_presets import PRESETS, PRESET_ORDER


# ---------------------------------------------------------------------------
# PS4-style button / axis indices  (mirrors joy_mux_controller.py)
# ---------------------------------------------------------------------------
class Buttons(IntEnum):
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
    RIGHT_STICK_X = 3
    RIGHT_STICK_Y = 4
    RIGHT_TRIGGER = 5
    D_PAD_X = 6
    D_PAD_Y = 7

DEADMAN = Buttons.LEFT_BUMPER

_JOY_MIN_BUTTONS = 13
_JOY_MIN_AXES = 8


# ---------------------------------------------------------------------------
# Pattern definitions
# ---------------------------------------------------------------------------
class Pattern(IntEnum):
    SOLID = 0
    BLINK = 1
    BREATHE = 2
    RGB_CYCLE = 3
    MANUAL = 4

PATTERN_NAMES = {
    Pattern.SOLID: "solid",
    Pattern.BLINK: "blink",
    Pattern.BREATHE: "breathe",
    Pattern.RGB_CYCLE: "rgb_cycle",
    Pattern.MANUAL: "manual (sticks)",
}


def _clamp(v: int) -> int:
    return max(0, min(255, int(v)))


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------
class SilBoardJoyTeleop(Node):
    def __init__(self):
        super().__init__("sil_board_joy_teleop")

        joy_topic = self.declare_parameter("joy_topic", "/joy").value
        led_topic = self.declare_parameter("led_topic", "/sil_board/rgb").value
        max_publish_hz = self.declare_parameter("max_publish_hz", 10.0).value
        self._skip_identical = self.declare_parameter("skip_identical", True).value

        self._pub = self.create_publisher(LedCommand, led_topic, 10)
        self.create_subscription(Joy, joy_topic, self._on_joy, 10)

        period_s = 1.0 / max(0.1, max_publish_hz)
        self._timer = self.create_timer(period_s, self._tick)

        # State
        self._preset_idx = 0
        self._pattern = Pattern.SOLID
        self._deadman_held = False

        # Edge-detection for button presses
        self._prev_buttons: list[int] = []

        # Latest axis values (updated every /joy callback)
        self._axes: list[float] = [0.0] * _JOY_MIN_AXES

        # Current computed colour (what the pattern / preset produces)
        self._current = PRESETS[PRESET_ORDER[self._preset_idx]]

        # Last published tuple — for skip_identical
        self._last_published: tuple[int, int, int, int] | None = None

        self.get_logger().info(
            f"Joy teleop ready — topic {joy_topic}, publish to {led_topic}, "
            f"max {max_publish_hz} Hz, skip_identical={self._skip_identical}"
        )

    # ----- joy callback (runs at /joy rate; only updates state) ------------

    def _on_joy(self, msg: Joy):
        if len(msg.buttons) < _JOY_MIN_BUTTONS or len(msg.axes) < _JOY_MIN_AXES:
            return

        self._deadman_held = msg.buttons[DEADMAN] == 1
        self._axes = list(msg.axes)

        prev = self._prev_buttons
        btns = list(msg.buttons)

        def rising(idx: int) -> bool:
            return btns[idx] == 1 and (len(prev) <= idx or prev[idx] == 0)

        # D-pad right / left → cycle preset colour
        if rising(Buttons.CIRCLE):
            self._preset_idx = (self._preset_idx + 1) % len(PRESET_ORDER)
            name = PRESET_ORDER[self._preset_idx]
            self._current = PRESETS[name]
            self.get_logger().info(f"Preset: {name}")

        if rising(Buttons.SQUARE):
            self._preset_idx = (self._preset_idx - 1) % len(PRESET_ORDER)
            name = PRESET_ORDER[self._preset_idx]
            self._current = PRESETS[name]
            self.get_logger().info(f"Preset: {name}")

        # Triangle → cycle pattern mode
        if rising(Buttons.TRIANGLE):
            self._pattern = Pattern((self._pattern + 1) % len(Pattern))
            self.get_logger().info(f"Pattern: {PATTERN_NAMES[self._pattern]}")

        # X → force "off" preset
        if rising(Buttons.X):
            self._preset_idx = PRESET_ORDER.index("off")
            self._current = PRESETS["off"]
            self.get_logger().info("Preset: off")

        self._prev_buttons = btns

    # ----- timer tick (rate-limited publishing) ----------------------------

    def _tick(self):
        if not self._deadman_held:
            return

        r, g, b, br = self._apply_pattern()
        colour = (_clamp(r), _clamp(g), _clamp(b), _clamp(br))

        if self._skip_identical and colour == self._last_published:
            return

        msg = LedCommand()
        msg.r, msg.g, msg.b, msg.brightness = colour
        self._pub.publish(msg)
        self._last_published = colour

    # ----- pattern math ----------------------------------------------------

    def _apply_pattern(self) -> tuple[int, int, int, int]:
        r, g, b, br = self._current
        t = time.monotonic()

        if self._pattern == Pattern.SOLID:
            return (r, g, b, br)

        if self._pattern == Pattern.BLINK:
            on = (int(t * 2) % 2) == 0
            return (r, g, b, br) if on else (0, 0, 0, 0)

        if self._pattern == Pattern.BREATHE:
            # Sine-wave brightness, keep colour
            phase = (math.sin(t * 2.0 * math.pi * 0.5) + 1.0) / 2.0
            return (r, g, b, int(br * phase))

        if self._pattern == Pattern.RGB_CYCLE:
            # Slow hue sweep at full brightness
            h = (t * 0.15) % 1.0
            cr, cg, cb = _hsv_to_rgb(h)
            return (cr, cg, cb, br)

        if self._pattern == Pattern.MANUAL:
            # Left stick X → Red, Left stick Y → Green, Right stick Y → Blue
            # Right trigger → Brightness  (trigger rests at 1.0, pulled = -1.0)
            ax = self._axes
            mr = int(((ax[Axes.LEFT_STICK_X] + 1.0) / 2.0) * 255)
            mg = int(((ax[Axes.LEFT_STICK_Y] + 1.0) / 2.0) * 255)
            mb = int(((ax[Axes.RIGHT_STICK_Y] + 1.0) / 2.0) * 255)
            mbr = int(((1.0 - ax[Axes.RIGHT_TRIGGER]) / 2.0) * 255)
            return (mr, mg, mb, mbr)

        return (r, g, b, br)


def _hsv_to_rgb(h: float, s: float = 1.0, v: float = 1.0) -> tuple[int, int, int]:
    """Convert HSV (h in 0-1) to 0-255 RGB."""
    i = int(h * 6.0)
    f = h * 6.0 - i
    p = v * (1.0 - s)
    q = v * (1.0 - f * s)
    t_ = v * (1.0 - (1.0 - f) * s)
    i %= 6
    if i == 0:
        r, g, b = v, t_, p
    elif i == 1:
        r, g, b = q, v, p
    elif i == 2:
        r, g, b = p, v, t_
    elif i == 3:
        r, g, b = p, q, v
    elif i == 4:
        r, g, b = t_, p, v
    else:
        r, g, b = v, p, q
    return int(r * 255), int(g * 255), int(b * 255)


def main(args=None):
    rclpy.init(args=args)
    node = SilBoardJoyTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
