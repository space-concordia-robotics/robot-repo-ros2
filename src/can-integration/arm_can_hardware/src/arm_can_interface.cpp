#include "arm_can_hardware/arm_can_interface.hpp"

#include "can-utils/buildAddress.hpp"
#include "can-utils/can_connect.hpp"
#include "can-utils/parser.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace arm_can_hardware
{

namespace
{
constexpr double kCommandDeadzone = 0.05;

// Standard arm motor velocity payload range (matches buildAddress::ARM_MOTOR_*).
constexpr float kArmMotorPayloadMax = buildAddress::BuildAddress::ARM_MOTOR_VELOCITY_MAX;
constexpr float kArmMotorPayloadMin = buildAddress::BuildAddress::ARM_MOTOR_VELOCITY_MIN;
}  // namespace

JointKind ArmCanInterface::parseKind(const std::string & s)
{
  if (s == "spin_servo") {
    return JointKind::SPIN_SERVO;
  }
  if (s == "clamp_servo") {
    return JointKind::CLAMP_SERVO;
  }
  return JointKind::ARM_MOTOR;
}

ServoMode ArmCanInterface::parseServoMode(const std::string & /*s*/)
{
  return ServoMode::POSITION;
}

Instructions::Inst ArmCanInterface::parseInstruction(const std::string & s)
{
  // Allow either decimal or 0x-prefixed hex in the URDF param.
  if (s.empty()) {
    return Instructions::Inst::ARM_MOTOR_1;
  }
  return static_cast<Instructions::Inst>(static_cast<uint32_t>(std::stoul(s, nullptr, 0)));
}

std::string ArmCanInterface::getParam(const hardware_interface::ComponentInfo & joint,
                                      const std::string & key,
                                      const std::string & fallback)
{
  auto it = joint.parameters.find(key);
  if (it == joint.parameters.end()) {
    return fallback;
  }
  return it->second;
}

hardware_interface::CallbackReturn ArmCanInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Hardware-level parameters (declared at the <hardware><param> level in the
  // URDF). Both are optional.
  auto hw_iter = info_.hardware_parameters.find("can_interface");
  if (hw_iter != info_.hardware_parameters.end()) {
    can_interface_name_ = hw_iter->second;
  }
  hw_iter = info_.hardware_parameters.find("send_heartbeat_on_activate");
  if (hw_iter != info_.hardware_parameters.end()) {
    send_heartbeat_on_activate_ = (hw_iter->second == "true" || hw_iter->second == "1");
  }

  if (info_.joints.empty()) {
    RCLCPP_FATAL(logger_, "No joints defined in <ros2_control> hardware info");
    return hardware_interface::CallbackReturn::ERROR;
  }

  joints_.clear();
  joints_.reserve(info_.joints.size());

  for (const auto & joint : info_.joints) {
    JointConfig cfg;
    cfg.name = joint.name;
    cfg.kind = parseKind(getParam(joint, "kind", "arm_motor"));

    switch (cfg.kind) {
      case JointKind::ARM_MOTOR: {
        cfg.arm_inst = parseInstruction(getParam(joint, "arm_instruction", "0x12"));
        try {
          cfg.velocity_scale = std::stof(getParam(joint, "velocity_scale", "1.0"));
        } catch (const std::exception &) {
          cfg.velocity_scale = 1.0f;
        }
        try {
          cfg.encoder_device_id = static_cast<uint32_t>(
            std::stoul(getParam(joint, "encoder_device_id", "0"), nullptr, 0));
        } catch (const std::exception &) {
          cfg.encoder_device_id = 0;
        }
        try {
          cfg.direction = std::stof(getParam(joint, "direction", "1.0"));
        } catch (const std::exception &) {
          cfg.direction = 1.0f;
        }

        // Derive default encoder TX IDs from device_id if not explicitly provided.
        // Firmware layout (questions.md §A): TX abs = 0x0108_C_<dev>01, speed = +0x40.
        // Device IDs: BASE=0x07, SHOULDER=0x08, ELBOW=0x09, WRIST=0x0B.
        auto deriveAbsId = [](uint32_t dev_id) -> uint32_t {
          // ID = (0x01 << 24) | (0x08 << 16) | ((dev_id << 2) << 6) | 0x01
          // Precomputed: 0x0108C001 + ((dev_id - 0x07) * 0x100) ... simpler: use the table.
          return (0x0108C000u) | ((dev_id & 0xFFu) << 8) | 0x01u;
        };
        auto deriveSpeedId = [&deriveAbsId](uint32_t dev_id) -> uint32_t {
          return deriveAbsId(dev_id) + 0x40u;
        };

        auto parseOptionalId = [&](const std::string & param_name) -> uint32_t {
          const std::string val = getParam(joint, param_name, "");
          if (val.empty()) {
            return 0;
          }
          try {
            return static_cast<uint32_t>(std::stoul(val, nullptr, 0));
          } catch (const std::exception &) {
            return 0;
          }
        };

        const uint32_t abs_id   = parseOptionalId("encoder_abs_can_id");
        const uint32_t speed_id = parseOptionalId("encoder_speed_can_id");

        if (abs_id != 0) {
          cfg.encoder_abs_can_id   = abs_id;
          cfg.encoder_speed_can_id = (speed_id != 0) ? speed_id : (abs_id + 0x40u);
        } else if (cfg.encoder_device_id != 0) {
          cfg.encoder_abs_can_id   = deriveAbsId(cfg.encoder_device_id);
          cfg.encoder_speed_can_id = deriveSpeedId(cfg.encoder_device_id);
        }
        break;
      }
      case JointKind::SPIN_SERVO:
      case JointKind::CLAMP_SERVO: {
        cfg.servo_mode = parseServoMode(getParam(joint, "servo_mode", "position"));
        const std::string default_max = (cfg.kind == JointKind::SPIN_SERVO) ? "90.0" : "15.0";
        try {
          cfg.servo_max = std::stof(getParam(joint, "servo_max", default_max));
        } catch (const std::exception &) {
          cfg.servo_max = (cfg.kind == JointKind::SPIN_SERVO) ? 90.0f : 15.0f;
        }
        break;
      }
    }

    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(logger_,
                   "Joint '%s' must declare exactly one velocity command interface (found %zu).",
                   joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    joints_.push_back(cfg);
  }

  const size_t n = joints_.size();
  hw_commands_velocity_.assign(n, 0.0);
  hw_states_position_.assign(n, std::numeric_limits<double>::quiet_NaN());
  hw_states_velocity_.assign(n, std::numeric_limits<double>::quiet_NaN());

  // Build O(1) lookup tables from full 29-bit encoder TX IDs -> joint index.
  abs_can_id_to_joint_.clear();
  speed_can_id_to_joint_.clear();
  for (size_t i = 0; i < joints_.size(); ++i) {
    if (joints_[i].kind != JointKind::ARM_MOTOR) {
      continue;
    }
    if (joints_[i].encoder_abs_can_id != 0) {
      abs_can_id_to_joint_[joints_[i].encoder_abs_can_id] = i;
      RCLCPP_DEBUG(logger_, "Joint '%s': encoder abs ID 0x%08X, speed ID 0x%08X",
                   joints_[i].name.c_str(),
                   joints_[i].encoder_abs_can_id,
                   joints_[i].encoder_speed_can_id);
    }
    if (joints_[i].encoder_speed_can_id != 0) {
      speed_can_id_to_joint_[joints_[i].encoder_speed_can_id] = i;
    }
  }

  RCLCPP_INFO(logger_, "Initialised ArmCanInterface with %zu joints on CAN '%s'",
              joints_.size(), can_interface_name_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmCanInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  try {
    can_controller_ = can_util::getSharedCanController(can_interface_name_, logger_);
    if (!can_controller_) {
      can_controller_.reset();
      return hardware_interface::CallbackReturn::ERROR;
    }
    frame_builder_ = std::make_unique<SystemFrameBuilder>(can_controller_);

    frame_callback_ = can_controller_->registerFrameCallback(
      [this](uint32_t id, const std::vector<uint8_t> & data) { onCanFrame(id, data); });

    RCLCPP_INFO(logger_, "Configured ArmCanInterface on CAN '%s'", can_interface_name_.c_str());
    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_FATAL(
      logger_, "Exception during ArmCanInterface configure on '%s': %s",
      can_interface_name_.c_str(), e.what());
    can_util::logCanSetupRecoveryHints(logger_, can_interface_name_);
    frame_callback_.reset();
    frame_builder_.reset();
    can_controller_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  } catch (...) {
    RCLCPP_FATAL(
      logger_, "Unknown exception during ArmCanInterface configure on '%s'",
      can_interface_name_.c_str());
    can_util::logCanSetupRecoveryHints(logger_, can_interface_name_);
    frame_callback_.reset();
    frame_builder_.reset();
    can_controller_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  }
}

hardware_interface::CallbackReturn ArmCanInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Reset commands so the first write() doesn't lurch the arm.
  std::fill(hw_commands_velocity_.begin(), hw_commands_velocity_.end(), 0.0);

  if (send_heartbeat_on_activate_ && frame_builder_) {
    // The wheel motors share the bus heartbeat; sending it here is harmless
    // and matches the legacy can_controller_node behaviour.
    constexpr uint64_t kWheelMotorMask = 0x7E;
    frame_builder_->startMotors(kWheelMotorMask);
  }

  RCLCPP_INFO(logger_, "ArmCanInterface activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmCanInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::fill(hw_commands_velocity_.begin(), hw_commands_velocity_.end(), 0.0);
  // Best effort: send a zero velocity to every arm motor so the hardware
  // doesn't keep moving after the controller is deactivated.
  if (frame_builder_) {
    for (const auto & cfg : joints_) {
      if (cfg.kind == JointKind::ARM_MOTOR) {
        frame_builder_->sendArmMotorVelocity(deviceType::DeviceType::ARM_MOTOR_CONTROLLER,
                                             cfg.arm_inst,
                                             DeviceId::ID::ARM_MOTOR_CONTROLLER, 0.0f);
      }
    }
  }
  RCLCPP_INFO(logger_, "ArmCanInterface deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ArmCanInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Snapshot the latest encoder feedback gathered by onCanFrame() into the
  // ros2_control state buffers.
  std::lock_guard<std::mutex> lk(feedback_mutex_);
  for (size_t i = 0; i < joints_.size(); ++i) {
    if (joints_[i].kind == JointKind::ARM_MOTOR) {
      hw_states_position_[i] = joints_[i].position * joints_[i].direction;
      hw_states_velocity_[i] = joints_[i].velocity * joints_[i].direction;
    } else {
      // Servos do not provide position feedback over CAN today; mirror the
      // commanded value so the controller has something coherent to read.
      hw_states_position_[i] = std::isnan(hw_states_position_[i]) ? 0.0 : hw_states_position_[i];
      hw_states_velocity_[i] = hw_commands_velocity_[i];
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArmCanInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!frame_builder_) {
    return hardware_interface::return_type::ERROR;
  }

  for (size_t i = 0; i < joints_.size(); ++i) {
    const JointConfig & cfg = joints_[i];
    const double raw = hw_commands_velocity_[i];
    if (std::isnan(raw)) {
      continue;
    }

    switch (cfg.kind) {
      case JointKind::ARM_MOTOR: {
        // Treat the velocity command as a normalized [-1, 1] signal scaled to
        // the firmware payload range, matching the legacy can_controller_node
        // behaviour. The scaling is: raw * velocity_scale * ARM_MOTOR_VELOCITY_MAX.
        const double scaled = raw * static_cast<double>(cfg.direction);
        float payload = (std::abs(scaled) < kCommandDeadzone)
                          ? 0.0f
                          : static_cast<float>(scaled * cfg.velocity_scale * kArmMotorPayloadMax);
        payload = std::clamp(payload, kArmMotorPayloadMin, kArmMotorPayloadMax);
        frame_builder_->sendArmMotorVelocity(deviceType::DeviceType::ARM_MOTOR_CONTROLLER,
                                             cfg.arm_inst,
                                             DeviceId::ID::ARM_MOTOR_CONTROLLER, payload);
        break;
      }
      case JointKind::SPIN_SERVO: {
        const int32_t deg = static_cast<int32_t>(
            std::round(std::clamp(static_cast<float>(raw), -1.0f, 1.0f) * cfg.servo_max));
        if (deg != 0) {
          frame_builder_->sendSpinServoPosition(deg);
        }
        break;
      }
      case JointKind::CLAMP_SERVO: {
        const int32_t deg = static_cast<int32_t>(
            std::round(std::clamp(static_cast<float>(raw), -1.0f, 1.0f) * cfg.servo_max));
        if (deg != 0) {
          frame_builder_->sendClampServoPosition(deg);
        }
        break;
      }
    }
  }

  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> ArmCanInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> ifs;
  ifs.reserve(joints_.size() * 2);
  for (size_t i = 0; i < joints_.size(); ++i) {
    ifs.emplace_back(joints_[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]);
    ifs.emplace_back(joints_[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]);
  }
  return ifs;
}

std::vector<hardware_interface::CommandInterface> ArmCanInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ifs;
  ifs.reserve(joints_.size());
  for (size_t i = 0; i < joints_.size(); ++i) {
    ifs.emplace_back(joints_[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocity_[i]);
  }
  return ifs;
}

void ArmCanInterface::onCanFrame(uint32_t id, const std::vector<uint8_t> & data)
{
  // Route by full 29-bit arbitration ID (stripped of EFF flag by CANController).
  // Firmware TX layout per docs/integration/questions.md §A:
  //   Abs  frame IDs: 0x0108C701/C801/C901/CB01  (BASE/SHOULDER/ELBOW/WRIST)
  //   Speed frame IDs: each abs ID + 0x40

  auto abs_it = abs_can_id_to_joint_.find(id);
  if (abs_it != abs_can_id_to_joint_.end()) {
    // Absolute position frame (DLC = 6):
    //   bytes 0–1 : uint16 LE calibrated angle (TLE5012B 15-bit signed counts)
    //   bytes 2–3 : uint16 LE TLE5012B status register
    //   byte  4   : reserved (0x00)
    //   byte  5   : validity flag (0x01 = valid, 0x00 = sensor error)
    if (data.size() < 6) {
      return;
    }
    if (data[5] != 0x01) {
      // Validity flag not set — sensor error. Keep last known position; log once.
      RCLCPP_WARN(logger_,
                  "Encoder abs frame 0x%08X: validity flag 0x%02X (sensor error)",
                  id, data[5]);
      return;
    }
    // Reconstruct signed 15-bit count from LE uint16.
    const uint16_t raw_u16 = static_cast<uint16_t>(data[0]) |
                             (static_cast<uint16_t>(data[1]) << 8);
    // TLE5012B angle is a 15-bit signed value in a 16-bit field (bit 15 unused/sign).
    const int16_t counts = static_cast<int16_t>(raw_u16);
    // Scale: 360° / 32768 counts, but we want radians: 2π / 32768 = π / 16384
    constexpr double kCountsToRad = M_PI / 16384.0;
    const double position_rad = static_cast<double>(counts) * kCountsToRad;
    const double dir = static_cast<double>(joints_[abs_it->second].direction);
    RCLCPP_DEBUG(logger_, "Encoder abs 0x%08X: counts=%d  pos_rad=%.4f", id, counts, position_rad);

    std::lock_guard<std::mutex> lk(feedback_mutex_);
    joints_[abs_it->second].position = position_rad * dir;
    return;
  }

  auto spd_it = speed_can_id_to_joint_.find(id);
  if (spd_it != speed_can_id_to_joint_.end()) {
    // Angular velocity frame (DLC = 6):
    //   bytes 0–3 : float32 angular velocity rad/s, PDP (middle) endian
    //               Firmware stores via memcpy then swaps within 16-bit halves:
    //               wire order [b1, b0, b3, b2] — undo swap before cast.
    //   byte  4   : sign flag (0 = positive, 1 = negative) — informational only
    //   bytes 5–7 : reserved
    if (data.size() < 4) {
      return;
    }
    // Undo PDP-endian swap: swap bytes 0↔1 and 2↔3, then reinterpret as float32 LE.
    uint8_t buf[4] = { data[1], data[0], data[3], data[2] };
    float velocity_rads = 0.0f;
    std::memcpy(&velocity_rads, buf, sizeof(float));
    const double dir = static_cast<double>(joints_[spd_it->second].direction);
    RCLCPP_DEBUG(logger_, "Encoder spd 0x%08X: vel_rads=%.4f", id, velocity_rads);

    std::lock_guard<std::mutex> lk(feedback_mutex_);
    joints_[spd_it->second].velocity = static_cast<double>(velocity_rads) * dir;
  }
}

}  // namespace arm_can_hardware

PLUGINLIB_EXPORT_CLASS(arm_can_hardware::ArmCanInterface, hardware_interface::SystemInterface)
