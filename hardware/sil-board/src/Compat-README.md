# SCRB motor board firmware — CAN API

This document describes the **application-level CAN protocol** implemented in `Core/Inc/protocol.h` and handled in `Core/Src/main.c`. The MCU talks to the bus through an **MCP25625** controller (SPI). All application frames use **CAN 2.0B extended identifiers** (29-bit IDs) as produced by `mcp_pack_ext_id_29()` in `Core/Src/mcp25625.c`.

**Source of truth:** `protocol.h` for macros and bit layouts; `main.c` for what the firmware actually does today.


---

## Frame format

| Field | Description |
|--------|-------------|
| **ID** | 29-bit extended. Two different encodings: **`MAKE_ID`** (host → board) and **`JETSON_MAKE_ID`** (board → Jetson). |
| **Data** | Up to 8 bytes. Motor commands use **DLC = 4** (one IEEE-754 **single**, little-endian). |
| **Endianness** | Multi-byte payload is **little-endian** (matches ARM `memcpy` into `float`). |

**Host tools (SocketCAN):** extended IDs are written as **8 hex digits** (e.g. `0001808C`). Data after `#` is **raw payload bytes in order** (byte 0 first). For a little-endian float, that order is the same as memory layout on ARM.

```bash
# Force stop, DLC 0
cansend can0 0001808C#

# Motor 1: payload is float 1.0f in little-endian → bytes 00 00 80 3F → hex 0000803f
# (See "Motor velocity scaling" — 1.0 is a small PWM fraction, not "1% duty".)
cansend can0 0001848C#0000803f
```

Configure the interface bitrate to match the MCP25625 bit timing in `MCP_init()` (`CNF1` / `CNF2` / `CNF3` in `Core/Src/mcp25625.c`) and your transceiver hardware. The README does **not** compute kbps from those registers; use the same setting as the board or measure with a scope / analyzer.

---

## IDs: host → board (`MAKE_ID`)

Commands sent **to** the motor board must use this layout:

```c
#define MAKE_ID(severity, instruction) \
    ( ((uint32_t)(severity)     << 15) \
    | ((uint32_t)(instruction)  <<  7) \
    | ((uint32_t)BOARD_DEVICE_ID << 1) )
```

Firmware decodes an incoming `id` as:

| Field | Extraction | Notes |
|--------|------------|--------|
| **device_id** | `(id >> 1) & 0x3F` | Must equal **`BOARD_DEVICE_ID` (6)** for motor/control handling. |
| **instruction** | `(id >> 7) & 0xFF` | Command opcode (see table below). |
| **severity** | `(id >> 15) & 0x03` | Must equal **`ctrl_id_type` (3)** for motor/control handling. |

Constants (`protocol.h`):

- `BOARD_DEVICE_ID` = **6** (`0b000110`)
- `ctrl_id_type` = **3** (`0b11`)

### Precomputed control IDs (hex)

| Symbol | Meaning | 29-bit ID (hex) | `cansend` ID |
|--------|---------|-----------------|--------------|
| `FORCE_STOP_ID` | Emergency stop motors | `0x1808C` | `0001808C` |
| `RESUME_ID` | Resume motors (see behavior below) | `0x1810C` | `0001810C` |
| `MOTOR_1_ID` | Set motor velocity | `0x1848C` | `0001848C` |
| `MOTOR_2_ID` | Set motor velocity | `0x1850C` | `0001850C` |
| `MOTOR_3_ID` | Set motor velocity | `0x1858C` | `0001858C` |
| `MOTOR_4_ID` | Set motor velocity | `0x1860C` | `0001860C` |
| `MOTOR_5_ID` | Set motor velocity | `0x1868C` | `0001868C` |
| `LED_ID` | Defined in `protocol.h` | `0x1870C` | `0001870C` |

`ERROR_ID` (`0x1018C`) uses **`status_id_type`** (2) and **`error_id_bit`** (3). It is defined for **board-oriented** status naming; the current `main.c` does **not** transmit this ID on a generic fault path (force-stop uses `JETSON_MAKE_ID` + `E070` instead). Hosts may still reserve or use `0x1018C` per your system design.

### Instruction values (`protocol.h`)

| Name | Value | Role |
|------|-------|------|
| `force_stop_id_bit` | `0b01` | Force stop |
| `resume_id_bit` | `0b10` | Resume |
| `error_id_bit` | `0b11` | Error / status (naming) |
| `motor_1_id_bit` … `motor_5_id_bit` | `0b1001` … `0b1101` | Per-motor velocity |
| `led_id_bit` | `0b1110` | Reserved in `protocol.h`; **not implemented** in `main.c` today |

---

## Commands (behavior in `main.c`)

### Force stop (`FORCE_STOP_ID`, DLC 0)

1. Stops PWM (`stop_motors()`).
2. Enters a loop: every **100 ms** sends a **heartbeat / error notify** frame to the Jetson (see below) and listens for **`RESUME_ID`** with valid `MAKE_ID` decode (`device_id == 6`, `severity == ctrl`).
3. On resume match, calls `start_motors()` and returns to normal polling.

### Resume (`RESUME_ID`, DLC 0)

- **Handled inside the force-stop loop only** (while the board is sending the E070 heartbeat).
- There is **no** separate top-level handler in the main loop; sending resume while not in that state **does not** start motors by itself in the current firmware.

### Motor velocity (`MOTOR_1_ID` … `MOTOR_5_ID`)

- **DLC:** send **4 bytes** (IEEE-754 **float**, **little-endian**). The firmware copies 4 bytes into a `float` (`memcpy`); it does **not** validate `frame.dlc == 4`. Hosts should **always** send DLC 4.

#### Motor velocity scaling (how the float becomes PWM)

The payload is treated as a **commanded magnitude** in **rad/s** for protocol semantics, but the implementation is **open-loop PWM**: there is no encoder feedback in this path.

For each motor, firmware computes a timer **compare value** (CCR), not a “percent duty” variable:

\[
\text{compare} = \min\!\Bigl(\text{Period},\ \frac{|\texttt{rads}|}{\texttt{MAX\_RADS}} \times \text{Period}\Bigr)
\]

- **`rads`** — float from the CAN payload.  
- **`fabsf(rads)`** — only the **absolute value** drives PWM width.  
- **`MAX_RADS`** — **`1024.0f`** in `protocol.h`. This is the **full-scale command**: \(|\texttt{rads}| = 1024\) maps to **compare = Period** (maximum PWM high time for that timer setup).  
- **`Period`** — `htim1.Init.Period` for motors 1–4 (**799** in the shipped Cube init), `htim3.Init.Period` for motor 5. Same formula, different timer.

**Direction:** sign of `rads` sets the direction GPIO (`M1_DIR` … `M5_DIR`): non-negative vs negative; magnitude still uses `fabsf`.

**Important:** the float is **not** “duty cycle %.” Example: **`1.0f`** (`0000803f` LE) gives compare ≈ \((1/1024) \times 799 \approx 0.78\) → **very low** PWM, not “1%” or “1 rad/s of real shaft speed.” To approximate **half** of full-scale PWM command, send about **`512.0f`**, not `0.5f`. Real rad/s at the mechanism depends on gearbox, supply voltage, and load; this layer only maps float → compare.

**Clamping:** if the product exceeds `Period`, compare is capped at `Period`.

---

## IDs: board → Jetson (`JETSON_MAKE_ID`)

Frames **from** the board to the Jetson (e.g. heartbeat after force-stop) use a **different** ID layout:

```c
#define JETSON_MAKE_ID(severity, instruction) \
    ( ((uint32_t)(device_type_CODE)     << 24) \
    | ((uint32_t)(manufacturer_CODE)     << 16) \
    | ((uint32_t)(severity)     << 14) \
    | ((uint32_t)(instruction)  <<  6) \
    | ((uint32_t)JETSON_ID << 0) )
```

With `device_type_CODE = 0`, `manufacturer_CODE = 0b00001000` (8), `JETSON_ID = 1`, **`JETSON_MAKE_ID(status_id_type, error_id_bit)`** evaluates to **`0x0880C1`**.

After force-stop, the board sends:

- **ID:** `JETSON_MAKE_ID(status_id_type, error_id_bit)` → **`0x0880C1`**
- **DLC:** 4  
- **Data:** ASCII **`E070`** (`0x45 0x30 0x37 0x30`)

**Jetson / PC software** must **not** parse these frames with the `MAKE_ID` field extraction (`>> 1`, `>> 7`, `>> 15`) used for host→board traffic; use the `JETSON_MAKE_ID` bitfields instead.

---

## Rate and robustness (for integrators)

- The MCP25625 has **two** RX buffers. Bursts of many frames faster than the MCU drains SPI can set **RX overflow** in `EFLG`; the firmware clears overflow flags and drives the panic LED (see `main.c`). **TX bus-off** (`TXBO`) also lights the panic LED; there is no automatic bus recovery loop in the snippet reviewed here—check `main.c` for the latest behavior.
- Prefer a **bounded update rate** from ROS / the host, or **batch** commands if the protocol is extended later.
- **`MCP_receive_frame()`** can return **two** back-to-back RX messages across successive calls when both RX buffers were filled in one SPI service (see `pending_frame` logic in `mcp25625.c`).

---

## Quick FAQ (common confusions)

| Topic | Detail |
|--------|--------|
| **Little-endian float hex** | `1.0f` = `0x3F800000` in IEEE-754; on the wire (LE) the byte order is `00 00 80 3F` → `cansend` data `0000803f`. **`3f800000`** after `#` would be wrong endianness for this firmware. |
| **`MAX_RADS` vs physical rad/s** | `1024` is the **command full scale** for PWM mapping, not a calibrated “true max rad/s” unless you tune it that way in mechanics + controls. |
| **Resume** | Only honored **inside** the force-stop heartbeat state; see **Resume** above. |
| **`JETSON_MAKE_ID` vs `MAKE_ID`** | Different bit packing; do not decode Jetson-bound frames with host→board field extraction. |
| **`LED_ID`** | Defined in `protocol.h` but **no handler** in `main.c`; sending it has no effect today. |

---

## Project layout (firmware)

| Path | Role |
|------|------|
| `Core/Inc/protocol.h` | ID macros, severities, instructions |
| `Core/Src/main.c` | Command handling, motors, LEDs |
| `Core/Src/mcp25625.c` | MCP25625 SPI driver, extended ID pack/unpack, RX/TX |
| `Core/Inc/mcp25625.h` | `CanFrame`, register maps, MCP API |

---

## References

- **MCP25625** datasheet — register map, `EFLG`, RX overflow, bit timing.
- **STM32CubeIDE / HAL** — build and flash for the STM32G0 target in this project.
