# BAB → Jetson CAN Telemetry Protocol

Reference for parsing CAN frames transmitted by the BAB (Battery Arbiter Board) firmware.
All frames use **CAN 2.0B extended identifiers** (29-bit).

---

## 1. Extended ID Layout

```
Bits [28:24]  DevType      (5 bits)
Bits [23:16]  Manufacturer (8 bits)
Bits [15:14]  Severity     (2 bits)
Bits [13:6]   InstrID      (8 bits)
Bits [5:0]    DeviceID     (6 bits)
```

Construct / deconstruct with:

```c
uint32_t ext_id = (devtype << 24) | (mfr << 16) | (sev << 14) | (instr << 6) | devid;

// Decompose:
uint8_t devtype = (ext_id >> 24) & 0x1F;
uint8_t mfr     = (ext_id >> 16) & 0xFF;
uint8_t sev     = (ext_id >> 14) & 0x03;
uint8_t instr   = (ext_id >>  6) & 0xFF;
uint8_t devid   = (ext_id)       & 0x3F;
```

### Fixed field values for BAB telemetry

| Field       | Value  | Meaning            |
|-------------|--------|--------------------|
| DevType     | `0x01` | BAB                |
| Manufacturer| `0x01` | SCC                |
| DeviceID    | `0x01` | BAB instance       |

### Severity codes

| Code | Name                | Meaning                       |
|------|---------------------|-------------------------------|
| 0x00 | SEV_EMERGENCY_MANUAL| Manual emergency intervention |
| 0x01 | SEV_EMERGENCY_AUTO  | Automatic emergency           |
| 0x02 | SEV_STATUS          | Periodic status telemetry     |
| 0x03 | SEV_CONTROL         | Control command (Jetson→BAB)  |

### Telemetry Instruction IDs (BAB → Jetson, severity = SEV_STATUS = 0x02)

| InstrID | DLC | Message             |
|---------|-----|---------------------|
| `0x00`  | 4   | Battery telemetry   |
| `0x02`  | 6   | Rail telemetry      |
| `0x03`  | 4   | TCU temperature     |
| `0x08`  | 1   | Relay status        |
| `0x0A`  | 1   | TCU fan status      |

### Emergency Instruction ID (BAB → Jetson, severity = SEV_EMERGENCY_AUTO = 0x01)

| InstrID | DLC | Message                    |
|---------|-----|----------------------------|
| `0x02`  | 1   | Automatic PDS rail shutdown |

---

## 2. Battery Telemetry (InstrID 0x00, DLC 4)

Sent once per battery (bat1 then bat2) every main-loop cycle (~1 s).

### Payload bit layout (32 bits, MSB-first)

```
Bit  31       : Battery index (0 = battery 1, 1 = battery 2)
Bits 30:20    : Voltage × 100  (11 bits, range 0–2047 → 0.00–20.47 V)
Bits 19:6     : |Current| × 100 (14 bits, range 0–16383 → 0.00–163.83 A)
Bits 5:0      : Temperature − 20 (6 bits, range 0–63 → 20–83 °C)
```

### Decoding (Python example)

```python
def decode_battery(data: bytes) -> dict:
    payload = int.from_bytes(data[:4], 'big')
    bat_idx  = (payload >> 31) & 0x01        # 0 or 1
    voltage  = ((payload >> 20) & 0x7FF) / 100.0
    current  = ((payload >>  6) & 0x3FFF) / 100.0
    temp_c   = (payload & 0x3F) + 20.0
    return {"battery": bat_idx + 1, "voltage_V": voltage,
            "current_A": current, "temp_C": temp_c}
```

---

## 3. Rail Telemetry (InstrID 0x02, DLC 6)

Sent once per PDS channel (CH1, CH2, CH3) every cycle.

**IMPORTANT:** All three rail frames share the **same CAN arbitration ID**.
Differentiate them by parsing the rail index from the data payload (bits 43:42).
Do NOT key telemetry storage by CAN ID alone — you will overwrite previous rails.

### Payload bit layout (44 bits packed into 6 bytes, upper 4 bits zero)

```
Bits 47:44    : (unused, always 0)
Bits 43:42    : Rail index (0 = CH1 / 5V, 1 = CH2 / Arm, 2 = CH3 / Wheel)
Bit  41       : Switch state (1 = rail ON)
Bits 40:30    : Voltage × 100  (11 bits, range 0–2047 → 0.00–20.47 V)
Bits 29:16    : |Current| × 100 (14 bits, range 0–16383 → 0.00–163.83 A)
Bits 15:0     : Power × 10 (16 bits, range 0–65535 → 0.0–6553.5 W)
```

### Decoding (Python example)

```python
def decode_rail(data: bytes) -> dict:
    payload = int.from_bytes(data[:6], 'big')
    rail_idx  = (payload >> 42) & 0x03
    switch_on = bool((payload >> 41) & 0x01)
    voltage   = ((payload >> 30) & 0x7FF) / 100.0
    current   = ((payload >> 16) & 0x3FFF) / 100.0
    power     = (payload & 0xFFFF) / 10.0
    return {"rail": rail_idx, "switch_on": switch_on,
            "voltage_V": voltage, "current_A": current, "power_W": power}
```

---

## 4. TCU Temperature (InstrID 0x03, DLC 4)

Payload is a raw **IEEE 754 single-precision float** (little-endian as stored by `memcpy` on Cortex-M4).

```python
import struct

def decode_tcu_temp(data: bytes) -> float:
    return struct.unpack('<f', data[:4])[0]  # degrees Celsius
```

---

## 5. Relay Status (InstrID 0x08, DLC 1)

Sent **twice** per cycle (once for each relay), same CAN ID.

| Bit | Meaning                                 |
|-----|-----------------------------------------|
| 0   | Relay index (0 = relay 1, 1 = relay 2)  |
| 1   | State (1 = closed / ON, 0 = open / OFF) |

```python
def decode_relay(data: bytes) -> dict:
    b = data[0]
    return {"relay": (b & 0x01) + 1, "closed": bool(b & 0x02)}
```

---

## 6. TCU Fan Status (InstrID 0x0A, DLC 1)

| Bit | Meaning                      |
|-----|------------------------------|
| 0   | Fan state (1 = ON, 0 = OFF)  |

```python
def decode_tcu_fan(data: bytes) -> dict:
    return {"fan_on": bool(data[0] & 0x01)}
```

---

## 7. Automatic PDS Rail Shutdown (Severity SEV_EMERGENCY_AUTO, InstrID 0x02, DLC 1)

Sent when BAB autonomously shuts down a rail (overcurrent / fault).

| Byte 0 (bits 1:0) | Meaning                                |
|--------------------|----------------------------------------|
| `0x00`             | All rails shut down                    |
| `0x01`             | Arm rail shut down                     |
| `0x02`             | Wheel rail shut down                   |

---

## 8. Jetson → BAB Commands (Severity SEV_CONTROL = 0x03)

To command the BAB, send a CAN frame with `DevType=0x01`, `Sev=0x03`, the appropriate InstrID, and a 2-byte big-endian data word selecting the target.

### Data words

| Word     | Meaning              |
|----------|----------------------|
| `0x000F` | Relay 1 / Arm rail   |
| `0x00F0` | Relay 2 / Wheel rail |

### Command table

| InstrID | Action                      | Data word         |
|---------|-----------------------------|-------------------|
| `0x00`  | Emergency: cut both relays  | (no data needed)  |
| `0x8F`  | Emergency: cut all PDS rails| (no data needed)  |
| `0x01`  | Open (disconnect) a relay   | select relay      |
| `0x02`  | Close (connect) a relay     | select relay      |
| `0x04`  | Turn OFF a PDS rail         | select rail       |
| `0x06`  | Turn ON a PDS rail          | select rail       |
| `0x08`  | Turn OFF TCU fan            | select (ignored)  |
| `0x0A`  | Turn ON TCU fan             | select (ignored)  |

### Example: turn on arm rail

```python
import can

ext_id = (0x01 << 24) | (0x01 << 16) | (0x03 << 14) | (0x06 << 6) | 0x01
msg = can.Message(arbitration_id=ext_id, is_extended_id=True,
                  data=b'\x00\x0F')  # DATA_SELECT_1 = arm rail
bus.send(msg)
```

---

## 9. Timing and Known Caveats

- BAB broadcasts all telemetry once per main-loop iteration, spaced by a 1-second `HAL_Delay`.
- Each burst contains ~9 CAN frames transmitted back-to-back with no inter-frame delay.
- The STM32F4 CAN peripheral has only **3 TX mailboxes**. Frames issued when all mailboxes
  are full are **silently dropped** (no retry, no error flag). Under load the 2nd rail frame
  (`CAN_SendRailTelemetry(1)`) is the most likely victim because it is the 5th or 6th frame
  in the burst sequence.
- All three rail telemetry frames share the **same arbitration ID** — the rail index lives
  in the data field only. If your receive handler stores "last value per CAN ID" you will
  see only whichever rail arrived last.

---

## 10. Known Firmware Bugs (why the 2nd rail goes missing)

### Bug 1 — `CAN_Transmit` silently drops frames (PRIMARY CAUSE)

**Location:** `main.c` — `CAN_Transmit()` function (~line 895)

```c
if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0) {
    can_tx_result = HAL_CAN_AddTxMessage(&hcan1, &can_tx_header, data, &can_tx_mailbox);
}
```

**What's wrong:** If all 3 TX mailboxes are occupied, the frame is simply not sent — no
retry, no queue, no error flag. The main loop fires 9 frames back-to-back every cycle:

```
CAN_SendBatteryTelemetry(1)   ← fills mailbox 1
CAN_SendBatteryTelemetry(2)   ← fills mailbox 2
CAN_SendRailTelemetry(0)      ← fills mailbox 3
CAN_SendRailTelemetry(1)      ← ALL FULL → DROPPED
CAN_SendRailTelemetry(2)      ← maybe dropped too
CAN_SendTCUTemp()
CAN_SendRelayStatus() × 2
CAN_SendTCUStatus()
```

At standard CAN bit rates (500 kbit/s) an extended-ID frame takes ~230 µs. After the
first 3 frames fill the mailboxes, the 4th call arrives within microseconds — long before
a mailbox drains. This consistently drops `CAN_SendRailTelemetry(1)` (the 2nd rail /
CH2 / Arm rail).

**Why it's not the HAL delays:** The `200` in `HAL_UART_Transmit(..., 200)` is a *timeout*
ceiling, not a sleep. The UART finishes in microseconds to low-milliseconds at typical baud
rates. `HAL_Delay(1000)` at the end of the loop spaces entire bursts — it does not
selectively skip one rail within a burst.

**Fix (summary):** See [Section 11 — Possible solutions for CAN TX frame dropping](#11-possible-solutions-for-can-tx-frame-dropping) for concrete approaches.

---

### Bug 2 — Duplicate CAN arbitration ID for all 3 rail frames (JETSON-SIDE GOTCHA)

**Location:** `CAN_SendRailTelemetry()` — all three calls use `INSTR_TX_RAIL_TELEM` = 0x02
with identical `DevType`, `Mfr`, `Sev`, and `DevID` fields, producing the **same 29-bit
extended ID**.

**What's wrong:** If the Jetson CAN receive code (or any logger / bus tool) stores
telemetry keyed by arbitration ID alone, it will overwrite the previous rail's data
every time a new rail frame arrives. Only the last-received rail survives.

**Fix (Jetson side):** Key telemetry storage by `(arbitration_id, rail_index)` where
`rail_index` is parsed from payload bits 43:42.

**Fix (firmware side, optional):** Encode the channel in the CAN ID itself (e.g. add
`channel` to the DeviceID field or use distinct InstrIDs per rail).

---

### Bug 3 — Blocking UART echo inside the USART2 RX interrupt (RELIABILITY RISK)

**Location:** `HAL_UART_RxCpltCallback` — USART2 branch (~line 762)

```c
HAL_GPIO_WritePin(DE1_GPIO_Port, DE1_Pin, GPIO_PIN_SET);
HAL_UART_Transmit(&huart1, rs485_rx_buffer, RS485_RX_BUFFER_SIZE, 200);
while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET) {}
HAL_GPIO_WritePin(DE1_GPIO_Port, DE1_Pin, GPIO_PIN_RESET);
```

**What's wrong (multiple issues):**

1. **Wrong UART and DE pin:** PDS communication uses USART2/DE2, but this echoes on
   `huart1`/DE1 (BMS UART). This is leftover debug code.
2. **Transmits 512 bytes regardless of message length:** Uses `RS485_RX_BUFFER_SIZE` (512)
   instead of `rs485_rx_index + 1` (actual message length). Sends garbage past the null
   terminator.
3. **Blocks inside an ISR:** `HAL_UART_Transmit` is a polling/blocking call. At 115200
   baud, 512 bytes takes ~44 ms. During that time **all other UART interrupts are masked**,
   causing missed BMS bytes on USART1/USART3 and potentially corrupting the next PDS
   response already arriving on USART2.
4. **Guard condition is wrong:** `rs485_rx_index >= 1` allows triggering when `rs485_rx_index`
   is 1, but then accesses `rs485_rx_buffer[rs485_rx_index - 2]` = index −1 (undefined
   behaviour / reads before buffer start).

**Why this could contribute to rail issues:** The ~44 ms stall can cause the PDS response
to be partially lost on the next cycle, leading to `sscanf` in `RS485_PDS()` parsing fewer
than 22 fields and updating **no** channel data at all (stale values forwarded via CAN).

**Fix:** Remove the debug echo entirely. If forwarding is needed, do it outside the ISR
(set a flag + handle in main loop) and use the actual message length.

---

### Bug 4 — `RS485_PDS` sscanf all-or-nothing parse

**Location:** `RS485_PDS()` (~line 816)

```c
if (parsed == 22) { ... }
```

**What's wrong:** If any single field in the multi-line PDS response is malformed (e.g. a
single corrupted byte in CH2's line), `sscanf` returns < 22 and **all three channels plus
TCU** get no update — even channels that were perfectly valid. Stale data persists until the
next successful full parse.

**Impact:** Not the direct cause of "only CH2 missing" (it's all-or-nothing), but combined
with Bug 3 causing partial PDS responses, it means entire telemetry cycles can be silently
stale.

**Fix:** Parse each `PDS:CH#` line independently so a corrupt single channel doesn't
invalidate the others.

---

## 11. Possible solutions for CAN TX frame dropping

**What goes wrong today:** `CAN_Transmit` only calls `HAL_CAN_AddTxMessage` when
`HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0`. The STM32F4 CAN controller has **three** TX
mailboxes. If all three are busy (previous frames still arbitrating or shifting out on the
bus), the next frame is **thrown away** with no retry. Your code still calls the same
`CAN_Send*` functions; some of them never reach the wire.

Below are **three** firmware-side approaches that keep the **same CAN IDs and payloads** as
today (no Jetson protocol change). They differ in *where* you wait and *when* you still
might lose a frame.

---

### Option A — Bounded wait inside `CAN_Transmit` (smallest change)

**Idea:** Treat “no free mailbox” as **temporary**, not “skip this frame.” Before each
`HAL_CAN_AddTxMessage`, **loop until** `GetTxMailboxesFreeLevel() > 0`, or until a **timeout**
(e.g. a few milliseconds) expires.

**Flow:** Caller invokes `CAN_Transmit` → if mailboxes are full, spin (or tight-loop) until
one frees → then submit the frame. Call sites (`CAN_SendBatteryTelemetry`, etc.) stay
unchanged.

**When is a frame still dropped?** Only if the **timeout** elapses while **all** mailboxes
stay busy. That should be rare on a healthy bus (each frame is on the order of hundreds of
microseconds). It *can* happen if the controller is in **bus-off / heavy error** state, or
if the timeout is set too aggressively. In normal operation you expect **zero** drops.

**Cost:** Short **busy-wait** in whatever context calls `CAN_Transmit` (today: main loop).
For a ~9-frame burst once per second, that is usually acceptable.

```c
void CAN_Transmit(uint32_t ext_id, uint8_t *data, uint8_t dlc)
{
    /* ... fill can_tx_header as today ... */

    uint32_t t0 = HAL_GetTick();
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
        if ((HAL_GetTick() - t0) > 5U) {
            /* optional: increment a drop counter, set a debug flag */
            return;
        }
    }
    (void)HAL_CAN_AddTxMessage(&hcan1, &can_tx_header, data, &can_tx_mailbox);
}
```

---

### Option B — Software TX queue + mailbox-complete interrupt

**Idea:** Never depend on “a mailbox is free *right now*” at the call site. Every send
**appends** a small record `(extended_id, up to 8 data bytes, DLC)` to a **ring buffer**.
A **TX mailbox complete** interrupt (`HAL_CAN_TxMailbox0CompleteCallback` and/or 1 and 2)
runs when the hardware has finished with a mailbox; from there you **pop** the next queued
frame and call `HAL_CAN_AddTxMessage` if a mailbox is free.

**Flow:** `CAN_Transmit` only enqueues → returns quickly. Hardware + ISR **drain** the queue
at bus speed. Multiple producers could enqueue if you design for it (not required today).

**When is a frame dropped?** Almost never during normal bursts, **unless the ring buffer
fills** (enqueue faster than the bus + ISR drain). Size the queue for at least one full
telemetry burst (~9 frames) plus a small margin.

**Cost:** More **code and RAM**, and correct handling of re-entrancy (main vs ISR). Benefit:
**no long busy-wait** in the main loop; good if you later add more TX traffic or real-time
constraints.

---

### Option C — Pace sends to mailbox capacity (no queue, no wait inside `CAN_Transmit`)

**Idea:** Keep `CAN_Transmit` as “submit if possible,” but **change the caller** so you never
submit more than **three** frames back-to-back without **waiting for hardware to catch up**.
For example: send three frames, then **block or poll** until `GetTxMailboxesFreeLevel() == 3`
(or until all pending TX completes—depends how you define “safe”), then send the next group.

**Important:** This is **not** the same as a blind `HAL_Delay(N)` between every frame. A
fixed delay can still **overflow** three mailboxes if `N` is too small, or waste time if `N`
is too large. The robust version is **tied to mailbox state** (or TX-complete flags), not
only wall-clock time.

**When is a frame dropped?** Same as today **if** you forget to pace after adding more
`CAN_Send*` calls, or if you pace on the wrong condition. If pacing is correct, you avoid
overflowing three slots without putting the wait inside `CAN_Transmit`.

**Cost:** **Scattered logic** at every burst site (easy to get wrong when the send list
grows). Option A centralizes the wait in one function.

---

### Recommendation

- Use **Option A** for the **minimum** change: one place to fix, same call pattern, drops only
  on genuine timeout / bus fault.
- Move to **Option B** if you want **no** busy-wait in the main loop or expect **more** TX
  frames per cycle.
- Use **Option C** only if you strongly prefer **not** touching `CAN_Transmit` and are willing
  to maintain pacing at each send site; functionally it overlaps with A but is easier to
  break as code evolves.
