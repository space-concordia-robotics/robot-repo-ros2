// arm_can_interface.hpp
//
// ros2_control SystemInterface plugin that drives the rover arm motors and
// gripper servos over the team CAN protocol. Replaces the serial-based
// arm_interface from the inverse-kinematics workspace.
//
// Joint configuration is taken from the URDF <ros2_control> block. Per-joint
// parameters declared with <param name="..."> are used to map URDF joints onto
// CAN motor IDs / servo selectors.

#pragma once

#include "can-utils/can_interface.hpp"
#include "can-utils/prefixes.hpp"
#include "can-utils/system_controller.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace arm_can_hardware
{

// What kind of CAN device a URDF joint maps onto. ARM_MOTOR uses
// SystemFrameBuilder::sendArmMotorVelocity(); SPIN_SERVO / CLAMP_SERVO use the
// dedicated servo helpers and respect a position-vs-speed mode parameter.
enum class JointKind
{
  ARM_MOTOR,
  SPIN_SERVO,
  CLAMP_SERVO,
};

enum class ServoMode
{
  POSITION,  // velocity command interpreted as target rad
  SPEED,     // velocity command interpreted as target rad/s
};

// Per-joint configuration parsed once during on_init() from the URDF.
struct JointConfig
{
  std::string name;
  JointKind kind{JointKind::ARM_MOTOR};

  // ARM_MOTOR fields
  Instructions::Inst arm_inst{Instructions::Inst::ARM_MOTOR_1};
  // Multiplicative gain applied to the [-1, 1] command before scaling to the
  // ARM_MOTOR_VELOCITY_MAX range. Useful for joints that need to be slowed
  // (e.g. motor 4 historically ran at 0.5 of the others).
  float velocity_scale{1.0f};
  // Encoder DeviceId used to filter feedback frames; set to 0 to disable.
  uint32_t encoder_device_id{0};

  // SERVO fields
  ServoMode servo_mode{ServoMode::SPEED};
  float servo_max{1.5707963f};

  // Latest feedback (radians and rad/s for arm motors). Protected by the
  // ArmCanInterface::feedback_mutex_.
  double position{std::numeric_limits<double>::quiet_NaN()};
  double velocity{std::numeric_limits<double>::quiet_NaN()};
};

class ArmCanInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ArmCanInterface)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

private:
  // CAN frame callback. Currently decodes encoder feedback frames assumed to
  // carry a float32 LE radians payload in bytes 0-3 (and optional float32 LE
  // rad/s in bytes 4-7). Update once the encoder firmware payload is locked.
  void onCanFrame(uint32_t id, const std::vector<uint8_t> & data);

  static JointKind parseKind(const std::string & s);
  static ServoMode parseServoMode(const std::string & s);
  // Parse an instruction byte from a hex/dec string in the URDF param.
  static Instructions::Inst parseInstruction(const std::string & s);

  // Read a string parameter from a joint's <param> map, returning fallback if
  // the key is missing.
  static std::string getParam(const hardware_interface::ComponentInfo & joint,
                              const std::string & key,
                              const std::string & fallback);

  std::string can_interface_name_{"can0"};
  bool send_heartbeat_on_activate_{true};

  std::shared_ptr<can_util::CANController> can_controller_;
  std::unique_ptr<SystemFrameBuilder> frame_builder_;
  std::shared_ptr<can_util::CANFrameCallback> frame_callback_;

  // Indexed parallel to info_.joints / hw_*_ vectors.
  std::vector<JointConfig> joints_;
  std::unordered_map<uint32_t, size_t> encoder_id_to_joint_;

  // ros2_control state buffers (one entry per URDF joint).
  std::vector<double> hw_states_position_;
  std::vector<double> hw_states_velocity_;
  std::vector<double> hw_commands_velocity_;

  mutable std::mutex feedback_mutex_;

  rclcpp::Logger logger_{rclcpp::get_logger("arm_can_interface")};
};

}  // namespace arm_can_hardware
