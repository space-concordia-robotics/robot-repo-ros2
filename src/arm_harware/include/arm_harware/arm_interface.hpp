
#ifndef ARM_INTERFACE_HPP
#define ARM_INTERFACE_HPP

#include "arm_hardware/arm_controller_node.h"
#include "arm_hardware/absenc.h"

#include "string"
#include "unordered_map"
#include "vector"

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

class ArmInterface : public hardware_interface::SystemInterface
{
public:
  //Lifecycle node override
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  harware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  // SystemInterface node override
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    //read whatever information the hardware sends us and goes to the controller

  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    //controller updates commands thatwas received and sends next command to write method so it could send it to the hardware


  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

private:
   std::shared_ptr<AbsencDriver> absenc_; 
   std::shared_ptr<ArmControllerNode> arm_controller_; 
   std::string port_; 
   int joint1_id_;
   int joint2_id_;
   int joint3_id_;
   int joint4_id_;
   int joint5_id_;
   int joint6_id_; 
   int joint7_id_;
   int joint8_id_;
   int joint9_id_;
};

}  // namespace arm_hardware

#endif  // ARM_INTERFACE_HPP