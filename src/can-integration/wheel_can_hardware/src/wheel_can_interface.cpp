#include "wheel_can_hardware/wheel_can_interface.hpp"

#include "can-utils/prefixes.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace wheel_can_hardware
{

namespace
{
// rad/s -> RPM conversion factor (60 / (2 * pi)).
constexpr double kRadSToRpm = 9.5492965855137201;
// RPM -> rad/s conversion factor (2 * pi / 60).
constexpr double kRpmToRadS = 0.10471975511965977;
// rotations -> radians.
constexpr double kRotToRad = 6.283185307179586;
}  // namespace

std::string WheelCanInterface::getParam(const hardware_interface::ComponentInfo & joint,
                                        const std::string & key,
                                        const std::string & fallback)
{
  auto it = joint.parameters.find(key);
  if (it == joint.parameters.end()) {
    return fallback;
  }
  return it->second;
}

std::string WheelCanInterface::getParam(const std::unordered_map<std::string, std::string> & params,
                                        const std::string & key,
                                        const std::string & fallback)
{
  auto it = params.find(key);
  if (it == params.end()) {
    return fallback;
  }
  return it->second;
}

hardware_interface::CallbackReturn WheelCanInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto & hp = info_.hardware_parameters;
  can_interface_name_         = getParam(hp, "can_interface", can_interface_name_);
  send_heartbeat_on_activate_ = (getParam(hp, "send_heartbeat_on_activate", "true") != "false");
  try {
    heartbeat_motor_mask_ = static_cast<uint64_t>(
      std::stoull(getParam(hp, "heartbeat_motor_mask", "0x7E"), nullptr, 0));
  } catch (const std::exception &) {
    heartbeat_motor_mask_ = 0x7Eu;
  }

  // Two ways to disable anti-slip: anti_slip_enabled=false (legacy) or
  // anti_slip_mode=off. Both are honoured.
  if (getParam(hp, "anti_slip_enabled", "true") == "false") {
    anti_slip_mode_ = AntiSlipMode::OFF;
  } else {
    anti_slip_mode_ = parseMode(getParam(hp, "anti_slip_mode", "pid"));
  }
  try { slip_threshold_        = std::stof(getParam(hp, "slip_threshold",        "0.30")); } catch (...) {}
  try { slip_kp_               = std::stof(getParam(hp, "slip_kp",               "0.10")); } catch (...) {}
  try { max_slip_correction_   = std::stof(getParam(hp, "max_slip_correction",   "0.50")); } catch (...) {}
  try { feedback_freshness_ms_ = std::stof(getParam(hp, "feedback_freshness_ms", "50.0")); } catch (...) {}
  try { stall_current_a_       = std::stof(getParam(hp, "stall_current_a",       "30.0")); } catch (...) {}
  try { stall_relief_factor_   = std::stof(getParam(hp, "stall_relief_factor",   "0.50")); } catch (...) {}

  if (info_.joints.empty()) {
    RCLCPP_FATAL(logger_, "No wheel joints declared in <ros2_control>");
    return hardware_interface::CallbackReturn::ERROR;
  }

  wheels_.clear();
  wheels_.reserve(info_.joints.size());
  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(logger_, "Wheel joint '%s' must declare one velocity command interface",
                   joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    WheelConfig cfg;
    cfg.name = joint.name;
    try {
      cfg.device_id = static_cast<uint8_t>(std::stoul(getParam(joint, "device_id", "0"), nullptr, 0));
    } catch (const std::exception &) {
      cfg.device_id = 0;
    }
    if (cfg.device_id == 0) {
      RCLCPP_FATAL(logger_, "Wheel joint '%s' is missing a non-zero <param name=\"device_id\">",
                   joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    const auto side = getParam(joint, "side", "left");
    cfg.side = (side == "right") ? WheelSide::RIGHT : WheelSide::LEFT;

    try { cfg.direction  = std::stof(getParam(joint, "direction", "1.0")); } catch (...) {}
    try { cfg.gear_ratio = std::stof(getParam(joint, "gear_ratio", "1.0")); } catch (...) {}
    if (cfg.gear_ratio == 0.0f) {
      cfg.gear_ratio = 1.0f;
    }

    wheels_.push_back(cfg);
  }

  const size_t n = wheels_.size();
  hw_commands_velocity_.assign(n, 0.0);
  hw_states_position_.assign(n, std::numeric_limits<double>::quiet_NaN());
  hw_states_velocity_.assign(n, std::numeric_limits<double>::quiet_NaN());
  last_motor_rpm_cmd_.assign(n, 0.0f);

  const char * mode_str =
    (anti_slip_mode_ == AntiSlipMode::OFF)         ? "off"        :
    (anti_slip_mode_ == AntiSlipMode::CLAMP)       ? "clamp"      :
    (anti_slip_mode_ == AntiSlipMode::PID)         ? "pid"        :
    /*CURRENT_PID*/                                  "current_pid";
  RCLCPP_INFO(logger_,
              "Initialised WheelCanInterface with %zu wheels on CAN '%s' (anti_slip_mode=%s)",
              wheels_.size(), can_interface_name_.c_str(), mode_str);
  return hardware_interface::CallbackReturn::SUCCESS;
}

WheelCanInterface::AntiSlipMode WheelCanInterface::parseMode(const std::string & s)
{
  if (s == "off")          return AntiSlipMode::OFF;
  if (s == "clamp")        return AntiSlipMode::CLAMP;
  if (s == "current_pid")  return AntiSlipMode::CURRENT_PID;
  return AntiSlipMode::PID;
}

hardware_interface::CallbackReturn WheelCanInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  can_controller_ = std::make_shared<can_util::CANController>(can_interface_name_, logger_);
  if (!can_controller_->configureCan()) {
    RCLCPP_FATAL(logger_, "Failed to configure CAN on '%s'", can_interface_name_.c_str());
    can_controller_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  }
  frame_builder_ = std::make_unique<SystemFrameBuilder>(can_controller_);

  std::vector<uint8_t> ids;
  ids.reserve(wheels_.size());
  for (const auto & w : wheels_) {
    ids.push_back(w.device_id);
  }
  feedback_ = std::make_unique<spark_max::SparkMaxFeedback>(can_controller_, std::move(ids));

  // Tell every SPARK MAX to start broadcasting STATUS_2 (velocity + position).
  // It's idempotent so safe to call again on reconfigure / after a power
  // cycle of the motor controllers.
  if (!feedback_->enableStatus2()) {
    RCLCPP_WARN(logger_, "One or more SPARK MAX SET_STATUSES_ENABLED frames failed to send");
  } else {
    RCLCPP_INFO(logger_, "STATUS_2 enable command sent to %zu SPARK MAX motors", wheels_.size());
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WheelCanInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::fill(hw_commands_velocity_.begin(), hw_commands_velocity_.end(), 0.0);
  std::fill(last_motor_rpm_cmd_.begin(), last_motor_rpm_cmd_.end(), 0.0f);

  if (send_heartbeat_on_activate_ && frame_builder_) {
    frame_builder_->startMotors(heartbeat_motor_mask_);
  }
  RCLCPP_INFO(logger_, "WheelCanInterface activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WheelCanInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Send a final zero velocity to every wheel to make sure nothing keeps
  // moving once the controller is detached.
  if (frame_builder_) {
    for (const auto & w : wheels_) {
      const auto did = static_cast<DeviceId::ID>(w.device_id);
      frame_builder_->sendWheelMotorVelocity(did, 0.0f);
    }
  }
  std::fill(hw_commands_velocity_.begin(), hw_commands_velocity_.end(), 0.0);
  std::fill(last_motor_rpm_cmd_.begin(), last_motor_rpm_cmd_.end(), 0.0f);
  RCLCPP_INFO(logger_, "WheelCanInterface deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type WheelCanInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!feedback_) {
    return hardware_interface::return_type::ERROR;
  }

  spark_max::WheelFeedback fb{};
  for (size_t i = 0; i < wheels_.size(); ++i) {
    if (!feedback_->getFeedback(wheels_[i].device_id, fb)) {
      continue;
    }
    // SPARK MAX reports velocity at the motor shaft; divide by the gear
    // reduction to get wheel-side units. Direction inversion is applied so
    // that "positive command -> rover moves forward" stays consistent for
    // both sides.
    const double motor_velocity_rad_s = static_cast<double>(fb.velocity_rpm) * kRpmToRadS;
    const double motor_position_rad   = static_cast<double>(fb.position_rot) * kRotToRad;
    hw_states_velocity_[i] = motor_velocity_rad_s / wheels_[i].gear_ratio * wheels_[i].direction;
    hw_states_position_[i] = motor_position_rad   / wheels_[i].gear_ratio * wheels_[i].direction;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type WheelCanInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!frame_builder_) {
    return hardware_interface::return_type::ERROR;
  }

  spark_max::WheelFeedback fb{};
  const auto fresh_window = std::chrono::milliseconds(static_cast<int>(feedback_freshness_ms_));

  for (size_t i = 0; i < wheels_.size(); ++i) {
    const WheelConfig & w = wheels_[i];

    // 1) Convert the controller command (rad/s at the wheel) into the
    //    SPARK MAX setpoint (RPM at the motor shaft).
    const double wheel_rad_s = hw_commands_velocity_[i];
    const double motor_rpm   = wheel_rad_s * kRadSToRpm * static_cast<double>(w.gear_ratio);
    float target_motor_rpm   = static_cast<float>(motor_rpm) * w.direction;

    // 2) Apply anti-slip correction using the latest STATUS_2 feedback (if
    //    fresh enough; otherwise we trust the open-loop command). STATUS_0
    //    current is read in the same snapshot for current-aware modes.
    float measured_motor_rpm = 0.0f;
    float motor_current_a    = 0.0f;
    bool fresh = false;
    if (feedback_ && feedback_->getFeedback(w.device_id, fb) && fb.status2_seen) {
      measured_motor_rpm = fb.velocity_rpm;
      motor_current_a    = fb.current_a;
      const auto age = std::chrono::steady_clock::now() - fb.status2_stamp;
      fresh = age <= fresh_window;
    }
    const float corrected_motor_rpm = applyAntiSlip(i, target_motor_rpm,
                                                    measured_motor_rpm,
                                                    motor_current_a, fresh);

    // 3) Send.
    last_motor_rpm_cmd_[i] = corrected_motor_rpm;
    const auto did = static_cast<DeviceId::ID>(w.device_id);
    frame_builder_->sendWheelMotorVelocity(did, corrected_motor_rpm);
  }

  // Re-issue the SPARK MAX heartbeat / start-motors frame on every cycle, as
  // the legacy can_controller_node did. Without this the SPARKs latch into a
  // safe-disable state if the heartbeat times out.
  frame_builder_->startMotors(heartbeat_motor_mask_);
  return hardware_interface::return_type::OK;
}

float WheelCanInterface::applyAntiSlip(size_t /*wheel_index*/,
                                       float target_rpm, float measured_rpm,
                                       float motor_current_a, bool feedback_fresh)
{
  if (anti_slip_mode_ == AntiSlipMode::OFF || !feedback_fresh) {
    return target_rpm;
  }

  // Slip ratio is signed: positive means the wheel is going faster than
  // commanded (free-spinning), negative means it's lagging behind (stalled
  // or being dragged). The denominator avoids division by zero at very low
  // commanded speeds.
  const float denom = std::max(std::abs(target_rpm), 1.0f);
  const float slip_ratio = (measured_rpm - target_rpm) / denom;

  if (std::abs(slip_ratio) < slip_threshold_) {
    return target_rpm;  // within tolerance, no action needed
  }

  switch (anti_slip_mode_) {
    case AntiSlipMode::OFF:
      return target_rpm;

    case AntiSlipMode::CLAMP: {
      // Only act on free-spin (positive slip ratio): pull the command down
      // toward the measured speed. Stalled wheels are left alone.
      if (slip_ratio <= 0.0f) {
        return target_rpm;
      }
      const float pull = slip_kp_ * (measured_rpm - target_rpm);
      const float cap = max_slip_correction_ * std::abs(target_rpm);
      const float correction = std::clamp(pull, -cap, cap);
      // pull is positive here (measured > target), so correction increases
      // target toward measured: use the negative direction to back off.
      return target_rpm + std::clamp(-correction, -cap, cap);
    }

    case AntiSlipMode::PID: {
      // Symmetric P controller: nudge the command toward the actual measured
      // velocity. Capped so a single noisy reading can't reverse the wheel.
      const float error = target_rpm - measured_rpm;
      const float cap = max_slip_correction_ * std::abs(target_rpm);
      const float correction = std::clamp(slip_kp_ * error, -cap, cap);
      return target_rpm + correction;
    }

    case AntiSlipMode::CURRENT_PID: {
      // Same PID as above plus a current-based stall guard: when the SPARK
      // MAX is drawing more than stall_current_a_ AND the wheel is dragging
      // (negative slip), reduce demand by stall_relief_factor_ instead of
      // pushing harder. This prevents the inside wheels from cooking the
      // motors during a tank turn against high-friction ground.
      const float error = target_rpm - measured_rpm;
      const float cap = max_slip_correction_ * std::abs(target_rpm);
      float corrected = target_rpm + std::clamp(slip_kp_ * error, -cap, cap);

      const bool stalled = (std::abs(motor_current_a) > stall_current_a_) && (slip_ratio < 0.0f);
      if (stalled) {
        corrected *= stall_relief_factor_;
      }
      return corrected;
    }
  }

  return target_rpm;
}

std::vector<hardware_interface::StateInterface> WheelCanInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> ifs;
  ifs.reserve(wheels_.size() * 2);
  for (size_t i = 0; i < wheels_.size(); ++i) {
    ifs.emplace_back(wheels_[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]);
    ifs.emplace_back(wheels_[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]);
  }
  return ifs;
}

std::vector<hardware_interface::CommandInterface> WheelCanInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ifs;
  ifs.reserve(wheels_.size());
  for (size_t i = 0; i < wheels_.size(); ++i) {
    ifs.emplace_back(wheels_[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocity_[i]);
  }
  return ifs;
}

}  // namespace wheel_can_hardware

PLUGINLIB_EXPORT_CLASS(wheel_can_hardware::WheelCanInterface, hardware_interface::SystemInterface)
