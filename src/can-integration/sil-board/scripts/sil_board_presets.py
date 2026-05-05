"""Shared LED preset names and RGBA tuples for SIL board teleop nodes."""

PRESETS: dict[str, tuple[int, int, int, int]] = {
    "white":  (255, 255, 255, 255),
    "red":    (255, 0,   0,   255),
    "green":  (0,   255, 0,   255),
    "blue":   (0,   0,   255, 255),
    "yellow": (255, 255, 0,   255),
    "cyan":   (0,   255, 255, 255),
    "purple": (255, 0,   255, 255),
    "off":    (0,   0,   0,   0),
}

PRESET_ORDER: tuple[str, ...] = (
    "white", "red", "green", "blue", "yellow", "cyan", "purple", "off",
)
