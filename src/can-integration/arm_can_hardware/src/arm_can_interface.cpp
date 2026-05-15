#include "arm_can_hardware/arm_can_interface.hpp"

#include "can-utils/buildAddress.hpp"
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

ServoMode ArmCanInterface::parseServoMode(const std::string & s)
{
  if (s == "position") {
    return ServoMode::POSITION;
  }
  return ServoMode::SPEED;
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

hardware_interface::CallbackReturn ArmCanInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
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
        break;
      }
      case JointKind::SPIN_SERVO:
      case JointKind::CLAMP_SERVO: {
        cfg.servo_mode = parseServoMode(getParam(joint, "servo_mode", "speed"));
        try {
          cfg.servo_max = std::stof(getParam(joint, "servo_max",
                                             cfg.servo_mode == ServoMode::POSITION ? "1.5707963" : "1.5707963"));
        } catch (const std::exception &) {
          cfg.servo_max = 1.5707963f;
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

  // Build an O(1) lookup from encoder DeviceId -> joint index for read().
  encoder_id_to_joint_.clear();
  for (size_t i = 0; i < joints_.size(); ++i) {
    if (joints_[i].kind == JointKind::ARM_MOTOR && joints_[i].encoder_device_id != 0) {
      encoder_id_to_joint_[joints_[i].encoder_device_id] = i;
    }
  }

  RCLCPP_INFO(logger_, "Initialised ArmCanInterface with %zu joints on CAN '%s'",
              joints_.size(), can_interface_name_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmCanInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  can_controller_ = std::make_shared<can_util::CANController>(can_interface_name_, logger_);
  if (!can_controller_->configureCan()) {
    RCLCPP_FATAL(logger_, "Failed to configure CAN on '%s'", can_interface_name_.c_str());
    can_controller_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  }
  frame_builder_ = std::make_unique<SystemFrameBuilder>(can_controller_);

  frame_callback_ = can_controller_->registerFrameCallback(
    [this](uint32_t id, const std::vector<uint8_t> & data) { onCanFrame(id, data); });

  RCLCPP_INFO(logger_, "Configured ArmCanInterface on CAN '%s'", can_interface_name_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
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
      hw_states_position_[i] = joints_[i].position;
      hw_states_velocity_[i] = joints_[i].velocity;
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
        float payload = (std::abs(raw) < kCommandDeadzone)
                          ? 0.0f
                          : static_cast<float>(raw * cfg.velocity_scale * kArmMotorPayloadMax);
        payload = std::clamp(payload, kArmMotorPayloadMin, kArmMotorPayloadMax);
        frame_builder_->sendArmMotorVelocity(deviceType::DeviceType::ARM_MOTOR_CONTROLLER,
                                             cfg.arm_inst,
                                             DeviceId::ID::ARM_MOTOR_CONTROLLER, payload);
        break;
      }
      case JointKind::SPIN_SERVO: {
        const float clamped = std::clamp(static_cast<float>(raw), -1.0f, 1.0f) * cfg.servo_max;
        if (cfg.servo_mode == ServoMode::POSITION) {
          frame_builder_->sendSpinServoPosition(clamped);
        } else {
          frame_builder_->sendSpinServoSpeed(clamped);
        }
        break;
      }
      case JointKind::CLAMP_SERVO: {
        const float clamped = std::clamp(static_cast<float>(raw), -1.0f, 1.0f) * cfg.servo_max;
        if (cfg.servo_mode == ServoMode::POSITION) {
          frame_builder_->sendClampServoPosition(clamped);
        } else {
          frame_builder_->sendClampServoSpeed(clamped);
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
  // Encoder boards transmit on DeviceType::ENCODER. Filter early to avoid
  // doing work for every frame on the bus.
  const DecodedFrame decoded = CANParser::parse(id, data);
  if (decoded.deviceType != static_cast<uint8_t>(deviceType::DeviceType::ENCODER)) {
    return;
  }

  auto it = encoder_id_to_joint_.find(decoded.deviceId);
  if (it == encoder_id_to_joint_.end()) {
    return;
  }

  // Encoder payload is currently assumed to be:
  //   bytes 0..3 : float32 LE position (rad)
  //   bytes 4..7 : float32 LE velocity (rad/s) -- optional
  // Update once the encoder firmware spec is finalised.
  if (data.size() < 4) {
    return;
  }

  float position = 0.0f;
  std::memcpy(&position, data.data(), sizeof(float));
  float velocity = 0.0f;
  if (data.size() >= 8) {
    std::memcpy(&velocity, data.data() + 4, sizeof(float));
  }

  std::lock_guard<std::mutex> lk(feedback_mutex_);
  joints_[it->second].position = static_cast<double>(position);
  if (data.size() >= 8) {
    joints_[it->second].velocity = static_cast<double>(velocity);
  }
}

}  // namespace arm_can_hardware

PLUGINLIB_EXPORT_CLASS(arm_can_hardware::ArmCanInterface, hardware_interface::SystemInterface)
