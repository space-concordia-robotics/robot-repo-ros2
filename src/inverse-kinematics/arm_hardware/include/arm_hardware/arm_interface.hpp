
#ifndef ARM_INTERFACE_HPP
#define ARM_INTERFACE_HPP

#include "arm_hardware/arm_controller_node.h"
#include "arm_hardware/absenc.h"

#include <string>
#include <unordered_map>
#include <vector>
#include <limits>

#include <termios.h>
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

using hardware_interface::return_type;

namespace arm_interface
{

class ArmInterface : public hardware_interface::SystemInterface //class inheriting from SystemInterface
{
public:
  //Lifecycle node override
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  // SystemInterface node override
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    //read whatever information the hardware sends us and goes to the controller

  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    //controller updates commands thatwas received and sends next command to write method so it could send it to the hardware


  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

private:
   
    int serial_fd_{-1}; //for encoder port
    int motor_serial_fd_{-1}; //for arm motor port

    std::string port_ = "/dev/ttyUSB0";

    std::vector<double> hw_states_position_ = {};
    std::vector<double> hw_states_velocity_ = {};
    std::vector<double> hw_commands_velocity_ = {};

    float old_angle_4 = 0;
    int8_t angle_4_zone = 0;


  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

};

}  // namespace arm_interface

#endif  // ARM_INTERFACE_HPP