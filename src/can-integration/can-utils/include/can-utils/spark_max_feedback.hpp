// spark_max_feedback.hpp
//
// Decode REV SPARK MAX periodic status frames (STATUS_0, STATUS_2) from the
// CAN bus and provide thread-safe per-motor accessors. Also exposes a helper
// to send the SET_STATUSES_ENABLED frame so STATUS_2 (velocity + position),
// which is disabled by default, can be turned on at runtime.
//
// References:
//   https://github.com/REVrobotics/REV-Specs/blob/main/can-frames/spark-frames-2.0.0-dev.11
//
// SPARK MAX 29-bit CAN ID layout:
//   [28:24] Device Type    = 0x02 (MotorController)
//   [23:16] Manufacturer   = 0x05 (REV)
//   [15:10] API Class      (6 bits)
//   [ 9: 6] API Index      (4 bits)
//   [ 5: 0] Device Number  (6 bits, values 1-62 in normal use)
//
// Frames of interest:
//   STATUS_0 (apiClass=46, apiIndex=0) base 0x0205B800, period 10 ms,
//     payload bytes:
//       bits  0..15  int16  APPLIED_OUTPUT  (scale 1/32442 -> [-1, 1])
//       bits 16..27  uint12 VOLTAGE         (scale 0.0073260073, V)
//       bits 28..39  uint12 CURRENT         (scale 0.0366300366, A)
//       bits 40..47  uint8  MOTOR_TEMPERATURE (degC)
//       (faults / limits in remaining bits, ignored here)
//
//   STATUS_2 (apiClass=46, apiIndex=2) base 0x0205B880, period 20 ms,
//     enabled by default = false. Payload:
//       bytes 0..3  float32 LE PRIMARY_ENCODER_VELOCITY (RPM)
//       bytes 4..7  float32 LE PRIMARY_ENCODER_POSITION (rotations)
//
// Command frame:
//   SET_STATUSES_ENABLED (apiClass=1, apiIndex=0) base 0x02050400, DLC 4,
//     bytes 0..1 uint16 LE MASK            (which bits we're modifying)
//     bytes 2..3 uint16 LE ENABLED_BITFIELD (target value for those bits)
//   Convention: bit N corresponds to STATUS_N. To enable STATUS_2 we set
//   MASK=0x0004 and ENABLED_BITFIELD=0x0004.

#pragma once

#include "can-utils/can_interface.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

namespace spark_max
{

// Bit positions within the SET_STATUSES_ENABLED bitfield.
constexpr uint16_t kStatusBit0 = 1u << 0;
constexpr uint16_t kStatusBit1 = 1u << 1;
constexpr uint16_t kStatusBit2 = 1u << 2;

// Base (device-id 0) arbitration IDs for the periodic status frames and
// command frames we care about. Add the SPARK MAX device ID (1..62) to get
// the actual frame ID.
constexpr uint32_t kStatus0BaseId            = 0x0205B800u;  // applied output, voltage, current, motor temp
constexpr uint32_t kStatus2BaseId            = 0x0205B880u;  // primary encoder velocity + position
constexpr uint32_t kSetStatusesEnabledBaseId = 0x02050400u;  // command: enable/disable periodic frames

// 6-bit mask for the device-id field in a SPARK MAX 29-bit CAN ID.
constexpr uint32_t kDeviceIdMask = 0x3Fu;
// Mask used to extract the API class + API index + frame type from an arbId,
// after stripping the device id. Equal to ~kDeviceIdMask masked into the
// 29-bit space.
constexpr uint32_t kFrameTypeMask = 0x1FFFFFC0u;

struct WheelFeedback
{
  // STATUS_2 fields (default unit RPM / rotations).
  float velocity_rpm{0.0f};
  float position_rot{0.0f};
  // STATUS_0 fields.
  float applied_output{0.0f};       // [-1, 1]
  float bus_voltage_v{0.0f};
  float current_a{0.0f};
  float motor_temperature_c{0.0f};

  // Most recent times any STATUS_2 / STATUS_0 frame was decoded. Use for
  // staleness checks (e.g. anti-slip should not trust feedback older than
  // ~50 ms when STATUS_2 is enabled at 20 ms period).
  std::chrono::steady_clock::time_point status0_stamp{};
  std::chrono::steady_clock::time_point status2_stamp{};
  bool status2_seen{false};
};

class SparkMaxFeedback
{
public:
  // Bind to an existing CANController (caller retains ownership) and watch
  // status frames for the given device ids (1..62). The constructor registers
  // a frame callback that runs on the CANController read thread.
  SparkMaxFeedback(std::shared_ptr<can_util::CANController> can,
                   std::vector<uint8_t> device_ids);

  // Send SET_STATUSES_ENABLED to every watched device, enabling STATUS_2.
  // Returns true if every send succeeded. Idempotent: safe to call again
  // after a SPARK MAX power-cycles.
  bool enableStatus2();

  // Snapshot the latest decoded values for a single motor. Returns false if
  // no frame has been seen yet (or the device id is not watched).
  bool getFeedback(uint8_t device_id, WheelFeedback & out) const;

  // Convenience: did STATUS_2 arrive within the given freshness window?
  bool isStatus2Fresh(uint8_t device_id, std::chrono::milliseconds max_age) const;

private:
  void onFrame(uint32_t id, const std::vector<uint8_t> & data);
  void decodeStatus0(uint8_t device_id, const std::vector<uint8_t> & data);
  void decodeStatus2(uint8_t device_id, const std::vector<uint8_t> & data);

  std::shared_ptr<can_util::CANController> can_;
  std::vector<uint8_t> device_ids_;

  // device_id -> latest decoded snapshot. Built once in the constructor so
  // lookup is O(1) without allocating during the read thread.
  std::unordered_map<uint8_t, WheelFeedback> feedback_;
  mutable std::mutex mutex_;

  std::shared_ptr<can_util::CANFrameCallback> frame_callback_;
};

}  // namespace spark_max
