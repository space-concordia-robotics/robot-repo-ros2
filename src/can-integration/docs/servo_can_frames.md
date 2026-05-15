# Servo CAN frames (spin / clamp)

Reference for **extended CAN 2.0B** frames used by `BuildAddress::buildServoFrame` and `SystemFrameBuilder` (`sendSpinServoPosition`, `sendClampServoPosition`, `sendSpinServoSpeed`, `sendClampServoSpeed`). Implementation: `can-utils/buildAddress.hpp`, `can-utils/system_controller.cpp`, `can-utils/prefixes.hpp`.

## Encoding summary

- **ID (29-bit arbitration)**: built from `DeviceType::ENCODER (0x07)`, `Manufacturer::TEAM_USE (0x08)`, `severity::SEV_CNTRL (0x03)`, an **8-bit instruction** (see below), and a **6-bit device id** (`SPIN_SERVO = 0x0C`, `CLAMP_SERVO = 0x0D`). The socket API also sets the extended-frame flag (`CAN_EFF_FLAG`).
- **DLC**: **5** bytes.
- **Payload**:
  - `data[0]`: servo selector on the wire — **`0x05`** = spin, **`0x06`** = clamp (`ServoSelector` in `prefixes.hpp`). This is **not** the same as the CAN device id field.
  - `data[1..4]`: **IEEE-754 float32, little-endian** — radians for position commands, **rad/s** for speed commands.

## CW / CCW sign convention

Use a **small magnitude** so moves stay gentle for testing.

**Convention in this document:** **positive float = CW**, **negative float = CCW**. If your gearbox flips direction, swap signs.

Demo step size: **±0.125** (exactly **1/8** rad or **1/8** rad/s).

| Float | IEEE-754 LE bytes (hex) |
|-------|-------------------------|
| **+0.125** | `00 00 00 3E` |
| **−0.125** | `00 00 00 BE` |
| **0.0** | `00 00 00 00` |

## Instructions (instruction byte in the CAN id)

| Instruction constant | Value | Meaning |
|----------------------|-------|---------|
| `SERVO_MOVE_TO_POSITION` | `0x01` | Closed-loop position: float = target angle **rad**. |
| `SERVO_MOVE_AT_SPEED` | `0x04` | Velocity: float = **rad/s** (not degrees). |

## 29-bit CAN IDs (hex)

Computed as: `(0x07 << 24) | (0x08 << 16) | (((0x03 << 8) \| instruction) << 6) | device_id`.

| Description | Instruction | Device id | CAN ID (29-bit) |
|-------------|-------------|-----------|-----------------|
| **Spin servo — move to position** | `0x01` | `0x0C` | **`0x0708C04C`** |
| **Clamp servo — move to position** | `0x01` | `0x0D` | **`0x0708C04D`** |
| **Spin servo — move at speed** | `0x04` | `0x0C` | **`0x0708C10C`** |
| **Clamp servo — move at speed** | `0x04` | `0x0D` | **`0x0708C10D`** |

## Every combination: small CW / CCW (+0.125 / −0.125)

Each row is one distinct **servo × mode × direction** pairing. Payload is always selector + float32 LE.

| # | Servo | Mode | Direction | Value | CAN ID | Data (hex) |
|---|-------|------|-----------|-------|--------|------------|
| 1 | Spin | Position | CW | +0.125 rad | `0x0708C04C` | `05 00 00 00 3E` |
| 2 | Spin | Position | CCW | −0.125 rad | `0x0708C04C` | `05 00 00 00 BE` |
| 3 | Clamp | Position | CW | +0.125 rad | `0x0708C04D` | `06 00 00 00 3E` |
| 4 | Clamp | Position | CCW | −0.125 rad | `0x0708C04D` | `06 00 00 00 BE` |
| 5 | Spin | Speed | CW | +0.125 rad/s | `0x0708C10C` | `05 00 00 00 3E` |
| 6 | Spin | Speed | CCW | −0.125 rad/s | `0x0708C10C` | `05 00 00 00 BE` |
| 7 | Clamp | Speed | CW | +0.125 rad/s | `0x0708C10D` | `06 00 00 00 3E` |
| 8 | Clamp | Speed | CCW | −0.125 rad/s | `0x0708C10D` | `06 00 00 00 BE` |

### Zero command (stop / hold setpoint at 0)

| # | Servo | Mode | Note | CAN ID | Data (hex) |
|---|-------|------|------|--------|------------|
| 9 | Spin | Speed | 0 rad/s | `0x0708C10C` | `05 00 00 00 00` |
| 10 | Clamp | Speed | 0 rad/s | `0x0708C10D` | `06 00 00 00 00` |
| 11 | Spin | Position | 0 rad target | `0x0708C04C` | `05 00 00 00 00` |
| 12 | Clamp | Position | 0 rad target | `0x0708C04D` | `06 00 00 00 00` |

## Stopping or holding while moving to a position

This stack only exposes **position** and **speed** servo frames (no separate “brake” opcode in software).

- **Cut motion via speed command**: send **`SERVO_MOVE_AT_SPEED`** with float **`0.0`** for that servo (rows 9–10). Whether the servo **holds position** or **coasts** depends on device firmware.
- **Reassert setpoint**: send **`SERVO_MOVE_TO_POSITION`** again with the same target if you need to refresh the position command after a stop.
- **System-level e-stop**: `sendForceStop` / `STOP_COMMAND` in this codebase targets **hub/compat** style devices, not the `ENCODER` servo message ids above; use only if your wiring maps that to the servos.

## `cansend` examples (Linux SocketCAN, `can-utils`)

For **classical** CAN 2.0 (what these servo frames use), `cansend` expects **`#`** once:  
`<8-hex-digit extended id>#<payload hex>`.

**Do not use `##` here.** In `cansend`, **`##` means a CAN FD frame**: the next character is the FD **flags** nibble, not the DLC.

Payload is **5 bytes** = **10** hex digits after `#`: byte0 = selector, bytes1–4 = float32 LE.

```text
# --- Position ±0.125 rad (dots optional) ---
cansend can0 0708C04C#05.00.00.00.3E   # spin CW
cansend can0 0708C04C#05.00.00.00.BE   # spin CCW
cansend can0 0708C04D#06.00.00.00.3E   # clamp CW
cansend can0 0708C04D#06.00.00.00.BE   # clamp CCW

# --- Speed ±0.125 rad/s ---
cansend can0 0708C10C#05.00.00.00.3E   # spin CW
cansend can0 0708C10C#05.00.00.00.BE   # spin CCW
cansend can0 0708C10D#06.00.00.00.3E   # clamp CW
cansend can0 0708C10D#06.00.00.00.BE   # clamp CCW

# --- Zero ---
cansend can0 0708C10C#05.00.00.00.00   # spin speed 0
cansend can0 0708C10D#06.00.00.00.00   # clamp speed 0
cansend can0 0708C04C#05.00.00.00.00   # spin position 0
cansend can0 0708C04D#06.00.00.00.00   # clamp position 0
```

Compact form (same 10 hex nibbles, no dots):

```text
cansend can0 0708C04C#050000003E
cansend can0 0708C04C#05000000BE
cansend can0 0708C04D#060000003E
cansend can0 0708C04D#06000000BE
cansend can0 0708C10C#050000003E
cansend can0 0708C10C#05000000BE
cansend can0 0708C10D#060000003E
cansend can0 0708C10D#06000000BE
cansend can0 0708C10C#0500000000
cansend can0 0708C10D#0600000000
cansend can0 0708C04C#0500000000
cansend can0 0708C04D#0600000000
```

## ROS2 / C++ helpers

Same numeric values as the tables:

- Position: `sendSpinServoPosition(±0.125f)` / `sendClampServoPosition(±0.125f)` — use **`0.0f`** for rows 11–12.
- Speed: `sendSpinServoSpeed(±0.125f)` / `sendClampServoSpeed(±0.125f)` — use **`0.0f`** for rows 9–10.
