#include "can-utils/spark_max_feedback.hpp"

#include <linux/can.h>

#include <cstring>
#include <stdexcept>

namespace spark_max
{

namespace
{
// Decode scale factors from the REV SPARK MAX frame spec (STATUS_0).
constexpr float kAppliedOutputScale = 1.0f / 32442.0f;     // int16 -> [-1, 1]
constexpr float kVoltageScale       = 0.0073260073f;       // uint12 -> V
constexpr float kCurrentScale       = 0.0366300366f;       // uint12 -> A
}  // namespace

SparkMaxFeedback::SparkMaxFeedback(std::shared_ptr<can_util::CANController> can,
                                   std::vector<uint8_t> device_ids)
    : can_(std::move(can)), device_ids_(std::move(device_ids))
{
  if (!can_) {
    throw std::invalid_argument("SparkMaxFeedback requires a non-null CANController");
  }
  for (uint8_t id : device_ids_) {
    feedback_[id] = WheelFeedback{};
  }
  frame_callback_ = can_->registerFrameCallback(
    [this](uint32_t id, const std::vector<uint8_t> & data) { onFrame(id, data); });
}

bool SparkMaxFeedback::enableStatus2()
{
  // Build the SET_STATUSES_ENABLED payload: enable STATUS_2 only, leave
  // every other status frame untouched. MASK selects the bits we modify,
  // ENABLED_BITFIELD provides the new value (1 = enable).
  uint8_t data[4];
  const uint16_t mask    = kStatusBit2;
  const uint16_t enabled = kStatusBit2;
  // Little-endian (matches the REV spec isBigEndian=false).
  data[0] = static_cast<uint8_t>(mask & 0xFF);
  data[1] = static_cast<uint8_t>((mask >> 8) & 0xFF);
  data[2] = static_cast<uint8_t>(enabled & 0xFF);
  data[3] = static_cast<uint8_t>((enabled >> 8) & 0xFF);

  bool all_ok = true;
  for (uint8_t device_id : device_ids_) {
    can_frame frame{};
    frame.can_id = (kSetStatusesEnabledBaseId + (device_id & kDeviceIdMask)) | CAN_EFF_FLAG;
    frame.len = 4;
    std::memcpy(frame.data, data, sizeof(data));
    if (!can_->sendBlockingFrame(frame)) {
      all_ok = false;
    }
  }
  return all_ok;
}

bool SparkMaxFeedback::getFeedback(uint8_t device_id, WheelFeedback & out) const
{
  std::lock_guard<std::mutex> lk(mutex_);
  auto it = feedback_.find(device_id);
  if (it == feedback_.end()) {
    return false;
  }
  out = it->second;
  return true;
}

bool SparkMaxFeedback::isStatus2Fresh(uint8_t device_id, std::chrono::milliseconds max_age) const
{
  std::lock_guard<std::mutex> lk(mutex_);
  auto it = feedback_.find(device_id);
  if (it == feedback_.end() || !it->second.status2_seen) {
    return false;
  }
  const auto age = std::chrono::steady_clock::now() - it->second.status2_stamp;
  return age <= max_age;
}

void SparkMaxFeedback::onFrame(uint32_t id, const std::vector<uint8_t> & data)
{
  // The CANController callback delivers the raw 29-bit ID (no EFF flag).
  const uint32_t arb_id     = id & 0x1FFFFFFFu;
  const uint32_t frame_type = arb_id & kFrameTypeMask;
  const uint8_t device_id   = static_cast<uint8_t>(arb_id & kDeviceIdMask);

  // Cheap early-out for unrelated frames (any non-SPARK-MAX traffic on the
  // bus, e.g. arm motor / encoder / BAB frames).
  if ((arb_id >> 16) != 0x0205u) {  // device type + manufacturer prefix
    return;
  }

  // Only handle frames whose device id was registered with the constructor.
  // Avoid touching the mutex if we don't have to.
  bool watched = false;
  for (uint8_t d : device_ids_) {
    if (d == device_id) {
      watched = true;
      break;
    }
  }
  if (!watched) {
    return;
  }

  if (frame_type == kStatus0BaseId) {
    decodeStatus0(device_id, data);
  } else if (frame_type == kStatus2BaseId) {
    decodeStatus2(device_id, data);
  }
}

void SparkMaxFeedback::decodeStatus0(uint8_t device_id, const std::vector<uint8_t> & data)
{
  if (data.size() < 8) {
    return;
  }

  // Pack the 8 payload bytes into a 64-bit word so we can shift bits across
  // byte boundaries. SPARK MAX uses little-endian bit ordering, so byte 0 is
  // the least-significant 8 bits.
  uint64_t raw = 0;
  for (int i = 0; i < 8; ++i) {
    raw |= static_cast<uint64_t>(data[i]) << (8 * i);
  }

  // bits  0..15: int16 APPLIED_OUTPUT
  const int16_t applied_raw = static_cast<int16_t>(raw & 0xFFFFu);
  // bits 16..27: uint12 VOLTAGE
  const uint16_t voltage_raw = static_cast<uint16_t>((raw >> 16) & 0x0FFFu);
  // bits 28..39: uint12 CURRENT
  const uint16_t current_raw = static_cast<uint16_t>((raw >> 28) & 0x0FFFu);
  // bits 40..47: uint8 MOTOR_TEMPERATURE
  const uint8_t temp_raw = static_cast<uint8_t>((raw >> 40) & 0xFFu);

  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lk(mutex_);
  auto & fb = feedback_[device_id];
  fb.applied_output      = static_cast<float>(applied_raw) * kAppliedOutputScale;
  fb.bus_voltage_v       = static_cast<float>(voltage_raw) * kVoltageScale;
  fb.current_a           = static_cast<float>(current_raw) * kCurrentScale;
  fb.motor_temperature_c = static_cast<float>(temp_raw);
  fb.status0_stamp       = now;
}

void SparkMaxFeedback::decodeStatus2(uint8_t device_id, const std::vector<uint8_t> & data)
{
  if (data.size() < 8) {
    return;
  }
  float velocity_rpm = 0.0f;
  float position_rot = 0.0f;
  std::memcpy(&velocity_rpm, data.data(),     sizeof(float));
  std::memcpy(&position_rot, data.data() + 4, sizeof(float));

  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lk(mutex_);
  auto & fb = feedback_[device_id];
  fb.velocity_rpm  = velocity_rpm;
  fb.position_rot  = position_rot;
  fb.status2_stamp = now;
  fb.status2_seen  = true;
}

}  // namespace spark_max
