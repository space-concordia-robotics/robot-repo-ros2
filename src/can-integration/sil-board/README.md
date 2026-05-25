# sil-board (package: `ci_sil_board`)

SIL board LED controller — sends RGB + brightness values over CAN using an
extended 29-bit frame ID.

> **Package name vs directory:** the colcon package name is `ci_sil_board`
> (CAN-integration SIL stack; distinct from upstream `hardware/sil-board` → `sil_board`).
> The directory remains `sil-board`.

## Build

From the workspace root:

```bash
colcon build --packages-select ci_sil_board
source install/setup.bash
```

## Nodes and scripts

### `sil_board_node` (C++)

CAN bridge that subscribes to `/ci_sil_board/rgb` (`ci_sil_board/msg/LedCommand`)
and sends each message as a single CAN frame via `can_util::CANController`.
No rate limiting — one topic message equals one `sendBlockingFrame` call.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `can_interface` | `can0` | SocketCAN interface name |
| `sil_can_id` | `0x0A08C050` | Raw 29-bit extended CAN ID |

```bash
ros2 run ci_sil_board sil_board_node --ros-args -p can_interface:=can0
```

### `sil_board_teleop.py` (keyboard)

Interactive CLI for one-shot commands. Type a preset name or four
space-separated values (`R G B BRIGHTNESS`, each 0-255).

Presets: `white`, `red`, `green`, `blue`, `yellow`, `cyan`, `purple`, `off`.

```bash
ros2 run ci_sil_board sil_board_teleop.py
```

### `sil_board_joy_teleop.py` (controller)

Joystick-driven teleop with presets, animated patterns, and rate-limited
publishing so the CAN bus is not flooded.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `joy_topic` | `/joy` | Joy input topic |
| `led_topic` | `/ci_sil_board/rgb` | LedCommand output topic |
| `max_publish_hz` | `10.0` | Max publish rate (Hz) |
| `skip_identical` | `true` | Suppress duplicate frames |

```bash
ros2 run ci_sil_board sil_board_joy_teleop.py --ros-args -p max_publish_hz:=10.0
```

**Controller mapping (PS4 layout, deadman = L1):**

| Input | Action |
|-------|--------|
| L1 (hold) | Deadman — required for any output |
| Circle | Next colour preset |
| Square | Previous colour preset |
| X | Force off |
| Triangle | Cycle pattern: solid / blink / breathe / RGB cycle / manual |

In **manual** pattern mode the analog sticks drive colour directly:

| Stick / Trigger | Channel |
|-----------------|---------|
| Left stick X | Red |
| Left stick Y | Green |
| Right stick Y | Blue |
| Right trigger | Brightness |

## Custom message

`msg/LedCommand.msg`:

```
uint8 r
uint8 g
uint8 b
uint8 brightness
```

## Typical stack

```bash
# Terminal 1 — CAN bridge
ros2 run ci_sil_board sil_board_node --ros-args -p can_interface:=can0

# Terminal 2 — joy driver
ros2 run joy_linux joy_linux_node

# Terminal 3 — joy LED control
ros2 run ci_sil_board sil_board_joy_teleop.py

# Terminal 4 (optional) — verify CAN frames
candump can0
```

## Related: joy_mux_controller publish rate

`joy_mux_controller` (package `joy_mux_controller_py`) republishes cached
commands at a fixed rate via `max_cmd_publish_hz` (default 100 Hz) while the
deadman is held. There is no deduplication — steady streaming is intentional
for CAN and arm motor reliability. Deadman release still triggers an
immediate all-stop followed by a zero-burst window.

Note: `sil_board_joy_teleop.py` retains its own `skip_identical` (default
true) and `max_publish_hz` (default 10 Hz) independently.
