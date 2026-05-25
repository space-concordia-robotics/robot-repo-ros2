# BAB → Jetson CAN Telemetry Protocol

Reference for parsing CAN frames transmitted by the BAB (Battery Arbiter Board) firmware.
Source of truth: `Firmware/BAB_MX/Core/Src/main.c`. All frames use **CAN 2.0B extended
identifiers** (29-bit).

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

### Fixed field values (firmware `main.c`)

Defined as `CAN_DEVTYPE_BAB`, `CAN_MFR_SCC`, and `CAN_DEVID_BAB` in
`Firmware/BAB_MX/Core/Src/main.c`. Used for **both** telemetry TX and command RX
(`CAN_HandleRx` rejects frames that do not match all three).

| Field        | Value  | Firmware define   |
|--------------|--------|-------------------|
| DevType      | `0x00` | `CAN_DEVTYPE_BAB` |
| Manufacturer | `0x08` | `CAN_MFR_SCC`     |
| DeviceID     | `0x00` | `CAN_DEVID_BAB`   |

Telemetry extended IDs (`severity = SEV_STATUS = 0x02`):

| Telemetry frame | Extended ID  | Decomposition (DevType / Mfr / Sev / InstrID / DevID) |
|-----------------|--------------|--------------------------------------------------------|
| Battery         | `0x00088000` | `0x00 / 0x08 / 0x02 / 0x00 / 0x00`                     |
| Rail            | `0x00088080` | `0x00 / 0x08 / 0x02 / 0x02 / 0x00`                     |
| TCU temperature | `0x000880C0` | `0x00 / 0x08 / 0x02 / 0x03 / 0x00`                     |
| Relay status    | `0x00088200` | `0x00 / 0x08 / 0x02 / 0x08 / 0x00`                     |
| TCU fan status  | `0x00088280` | `0x00 / 0x08 / 0x02 / 0x0A / 0x00`                     |

Emergency notification (`severity = SEV_EMERGENCY_AUTO = 0x01`):

| Frame                    | Extended ID  | Decomposition (DevType / Mfr / Sev / InstrID / DevID) |
|--------------------------|--------------|--------------------------------------------------------|
| Automatic PDS rail shut  | `0x00084080` | `0x00 / 0x08 / 0x01 / 0x02 / 0x00`                     |

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

Sent once per battery (bat1 then bat2) every telemetry burst (~3 s; see §9).

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

## 8. Jetson → BAB Commands

All commands must use the same ID fields as telemetry (§1):
`DevType=0x00`, `Manufacturer=0x08`, `DeviceID=0x00`.
`CAN_HandleRx` ignores frames that do not match.

### Emergency commands (`SEV_EMERGENCY_MANUAL = 0x00`)

Require **DLC ≥ 4**. Bytes 0–3 are a big-endian 32-bit token; both supported
tokens are `0x00000000` (`DATA_EMERG_RELAY_TOKEN` / `DATA_EMERG_PDS_TOKEN`).

| InstrID | Action                       | Token (4 bytes, BE) |
|---------|------------------------------|---------------------|
| `0x00`  | Cut both relays              | `0x00000000`        |
| `0x8F`  | Command PDS to cut all rails | `0x00000000`        |

### Control commands (`SEV_CONTROL = 0x03`)

Require **DLC ≥ 2**. Bytes 0–1 are a big-endian 16-bit data word.

| Word     | Meaning              |
|----------|----------------------|
| `0x000F` | Relay 1 / Arm rail   |
| `0x00F0` | Relay 2 / Wheel rail |

| InstrID | Action                      | Data word    |
|---------|-----------------------------|--------------|
| `0x01`  | Open (disconnect) a relay   | select relay |
| `0x02`  | Close (connect) a relay     | select relay |
| `0x04`  | Turn OFF a PDS rail         | select rail  |
| `0x06`  | Turn ON a PDS rail          | select rail  |
| `0x08`  | Turn OFF TCU fan            | (ignored)    |
| `0x0A`  | Turn ON TCU fan             | (ignored)    |

PDS rail commands map `DATA_SELECT_1` → rail `1` (arm), `DATA_SELECT_2` → rail `2`
(wheel). There is no CAN command for CH1 (5 V) in the current firmware.

### Example: turn on arm rail

```python
import can

ext_id = (0x00 << 24) | (0x08 << 16) | (0x03 << 14) | (0x06 << 6) | 0x00
msg = can.Message(arbitration_id=ext_id, is_extended_id=True,
                  data=b'\x00\x0F')  # DATA_SELECT_1
bus.send(msg)
```

---

## 9. Timing and Known Caveats

- Telemetry is sent when `HAL_GetTick() - last_can_bab_data_transfer >= 3000` ms
  (3 s between bursts), not every main-loop iteration.
- Each burst contains 9 CAN frames (2 battery + 3 rail + TCU temp + 2 relay + fan)
  issued back-to-back from the main loop.
- `CAN_Transmit` waits up to **10 ms** for a free TX mailbox before giving up (see §10).
  The STM32F4 still has only **3 TX mailboxes**, but frames are no longer dropped
  immediately when all three are busy.
- All three rail telemetry frames share the **same arbitration ID** — the rail index lives
  in the data field only. If your receive handler stores "last value per CAN ID" you will
  see only whichever rail arrived last.

---

## 10. Known Issues

### TX mailbox wait (fixed in current firmware)

**Location:** `Firmware/BAB_MX/Core/Src/main.c` — `CAN_Transmit()`

Earlier builds submitted only when `GetTxMailboxesFreeLevel() > 0`, which dropped the
4th+ frame in a 9-frame burst (often `CAN_SendRailTelemetry(1)`). Current firmware
spins until a mailbox frees or a **10 ms** deadline expires:

```c
uint32_t deadline = HAL_GetTick() + 10U;
while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U) {
    if (HAL_GetTick() >= deadline) return;
}
can_tx_result = HAL_CAN_AddTxMessage(&hcan1, &can_tx_header, data, &can_tx_mailbox);
```

A frame can still be lost if the bus is off or all mailboxes stay busy for 10 ms.

---

### Duplicate CAN arbitration ID for all 3 rail frames (host-side gotcha)

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

### `RS485_PDS` sscanf all-or-nothing parse

**Location:** `RS485_PDS()` in `main.c`

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

## 11. CAN TX frame dropping — options

**Current firmware:** Option A below is already implemented in `CAN_Transmit()` (10 ms
mailbox wait). The notes remain useful if you need a queue or stricter guarantees.

The STM32F4 CAN controller has **three** TX mailboxes. Without waiting, back-to-back sends
in a telemetry burst can overflow all three slots and drop frames.

Below are three approaches that keep the **same CAN IDs and payloads**. They differ in
*where* you wait and *when* you might still lose a frame.

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

    uint32_t deadline = HAL_GetTick() + 10U;
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U) {
        if (HAL_GetTick() >= deadline) return;
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

- **Option A** is in use today; increase the timeout or add drop counting if you still
  see missing frames under bus fault.
- Move to **Option B** for a software TX queue and no busy-wait in the main loop.
- **Option C** only if you prefer pacing at each call site instead of centralizing wait
  logic in `CAN_Transmit`.
