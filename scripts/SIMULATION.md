# Motor command simulation (desk math)

This file explains how to **predict PWM compare values** from a CAN motor command **without hardware**, using the same rules as `main.c`. For the full protocol, see [README.md](README.md).

**Firmware reference:** `Core/Src/main.c` (motor branches), `Core/Inc/protocol.h` (`MAX_RADS`).

---

## Model (open-loop)

The board does **not** close the loop on shaft position or speed in this path. The CAN float is mapped to a timer **compare register** (PWM high-time within one period).

\[
\text{compare} = \min\!\Bigl(\text{Period},\ \frac{|\texttt{rads}|}{\texttt{MAX\_RADS}} \times \text{Period}\Bigr)
\]

| Symbol | Value in firmware | Notes |
|--------|-------------------|--------|
| `MAX_RADS` | `1024.0f` | Full-scale **command**; \(|\texttt{rads}| = 1024\) → compare = `Period`. |
| `Period` (motors 1–4) | `htim1.Init.Period` | **799** in the current Cube configuration. |
| `Period` (motor 5) | `htim3.Init.Period` | **799** in the current Cube configuration. |

**Direction:** sign of `rads` only affects the DIR GPIO; magnitude uses `fabsf(rads)`.

---

## Worked examples (`Period = 799`)

| Command `rads` | \(\|rads\|/1024\) | compare (before clamp) | Rounded `uint16_t` compare |
|----------------|-------------------|---------------------------|----------------------------|
| `0.0` | 0 | 0 | 0 |
| `1.0` | ≈ 0.000977 | ≈ 0.78 | 0 |
| `512.0` | 0.5 | 399.5 | 399 |
| `1024.0` | 1.0 | 799 | 799 |
| `-1024.0` | 1.0 | 799 | 799 (same magnitude) |
| `2000.0` | > 1 | 1560.5 | **799** (clamped) |

So **`1.0f`** produces **zero** effective compare after integer truncation (0.78 → 0). Use larger commands for visible motion.

---

## Payload bytes (little-endian float)

Host tools need the **four payload bytes** in **little-endian** order.

**Python 3:**

```python
import struct

def motor_payload_le(rads: float) -> str:
    """Hex string for cansend data (no spaces)."""
    return struct.pack("<f", rads).hex()

# Examples:
# motor_payload_le(1.0)   -> '0000803f'
# motor_payload_le(512.0) -> '00000044'  (0x44000000 as LE bytes)
```

**Sanity check:** `1.0f` → IEEE `0x3F800000` → LE bytes `00 00 80 3F` → `0000803f`.

---

## Simulating the CAN **bus** on a PC (vcan)

This only exercises **Linux SocketCAN** tools and any **host** nodes you attach; the **STM32 does not run on vcan**. Use it to test Jetson/ROS nodes or scripts that encode IDs and floats.

```bash
# Create virtual CAN (root typically required)
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

# Watch traffic
candump vcan0 &

# Example: motor 1 half-scale command (512.0f)
python3 -c "import struct; print(struct.pack('<f',512.0).hex())"
# -> 00000044

cansend vcan0 0001848C#00000044
```

Match **extended** 8-digit IDs and **DLC 4** motor payloads as in [README.md](README.md).

---

## When this model is wrong

- You change **`MAX_RADS`**, timer **`Period`**, or the motor scaling code in `main.c`.
- You add **closed-loop** control; then CAN semantics may change.
- **Real** joint speed ≠ this ratio without calibration (gearbox, voltage, friction).

Re-run the math from the current `main.c` after any such change.
