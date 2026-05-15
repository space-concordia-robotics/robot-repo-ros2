// wheel_can_interface.hpp
//
// ros2_control SystemInterface plugin that drives the rover's six REV
// SPARK MAX wheel motors over CAN. Velocity commands flow rad/s -> RPM ->
// SystemFrameBuilder::sendWheelMotorVelocity(). Encoder feedback comes back
// from the SPARK MAX STATUS_2 frames decoded by spark_max::SparkMaxFeedback
// and is exposed in the standard ros2_control rad / rad/s units.
//
// The plugin also implements an opt-in anti-slip differential to help with
// skid-steer (tank) turns. See WheelCanInterface::write() for the details.

#pragma once

#include "can-utils/can_interface.hpp"
#include "can-utils/spark_max_feedback.hpp"
#include "can-utils/system_controller.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace wheel_can_hardware
{

enum class WheelSide
{
  LEFT,
  RIGHT,
};

struct WheelConfig
{
  std::string name;
  // SPARK MAX device id (1..62) on the CAN bus.
  uint8_t device_id{0};
  WheelSide side{WheelSide::LEFT};
  // -1.0f or +1.0f. Multiplied into the commanded RPM to flip direction for
  // motors that are physically mounted reversed (the legacy controller node
  // hard-coded the right-hand side as inverted).
  float direction{1.0f};
  // Motor reductions: motor revolutions per wheel revolution. RPM and
  // rotations from the SPARK MAX are at the motor shaft; we divide by this
  // to express feedback at the wheel.
  float gear_ratio{1.0f};
};

class WheelCanInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(WheelCanInterface)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

private:
  // Anti-slip mode selector (string param "anti_slip_mode" in URDF / YAML).
  enum class AntiSlipMode
  {
    OFF,         // pass commands through unchanged
    CLAMP,       // Level 1: only attenuate wheels spinning faster than commanded
    PID,         // Level 2: proportional correction in both directions
    CURRENT_PID  // Level 3: PID + STATUS_0 current threshold to back off stalls
  };

  // Apply the configured anti-slip strategy to one wheel's commanded RPM
  // using its measured RPM. Returns the corrected motor RPM. If anti-slip
  // is disabled or feedback is stale, returns target_rpm unchanged.
  //
  // motor_current_a is the SPARK MAX STATUS_0 current draw in amperes,
  // used by CURRENT_PID to detect stalled wheels (high current + low
  // velocity) and back off the demand instead of fighting the obstacle.
  float applyAntiSlip(size_t wheel_index, float target_rpm, float measured_rpm,
                      float motor_current_a, bool feedback_fresh);

  static AntiSlipMode parseMode(const std::string & s);

  static std::string getParam(const hardware_interface::ComponentInfo & joint,
                              const std::string & key,
                              const std::string & fallback);
  static std::string getParam(const std::unordered_map<std::string, std::string> & params,
                              const std::string & key,
                              const std::string & fallback);

  // Hardware-level params.
  std::string can_interface_name_{"can0"};
  bool send_heartbeat_on_activate_{true};
  uint64_t heartbeat_motor_mask_{0x7Eu};

  // Anti-slip configuration. Defaults match the plan's Phase 5 values.
  AntiSlipMode anti_slip_mode_{AntiSlipMode::PID};
  float  slip_threshold_{0.30f};        // |actual - target| / max(|target|, 1) over which we react
  float  slip_kp_{0.10f};               // proportional gain on the (target - actual) error
  float  max_slip_correction_{0.50f};   // max fraction of |target| to add as correction
  float  feedback_freshness_ms_{50.0f};
  float  stall_current_a_{30.0f};       // STATUS_0 current above which we treat the wheel as stalled
  float  stall_relief_factor_{0.50f};   // multiplicative reduction applied when a stall is detected

  std::shared_ptr<can_util::CANController>      can_controller_;
  std::unique_ptr<SystemFrameBuilder>           frame_builder_;
  std::unique_ptr<spark_max::SparkMaxFeedback>  feedback_;

  std::vector<WheelConfig> wheels_;

  // ros2_control state buffers (one entry per wheel joint, units rad / rad/s).
  std::vector<double> hw_states_position_;
  std::vector<double> hw_states_velocity_;
  std::vector<double> hw_commands_velocity_;

  // Last computed motor RPM per wheel; used for diagnostics / debug logging.
  std::vector<float> last_motor_rpm_cmd_;

  rclcpp::Logger logger_{rclcpp::get_logger("wheel_can_interface")};
};

}  // namespace wheel_can_hardware
