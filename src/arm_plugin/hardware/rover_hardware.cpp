#include "hardware/arm_hardware.hpp"
#include <string>
#include <vector>

#include <iostream>

namespace arm_hardware
{
 CallbackReturn RobotSystem::on_init(const hardware_interface::HardwareInfo &info)
 {
    if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }
    //Setup communication with robot hardware
    return CallbackReturn::SUCCESS;
 }
 CallbackReturn RobotSystem::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
 {
    // Setup communication with robot hardware
    return CallbackReturn::SUCCESS;
 }
 return_type RobotSystem::read(const rclcpp::Time & time, const rclcpp::Duration &period)
 {
    //read hardware values for the state interfaces, ex: joint encoders and sensor readings. 

    return return_type::OK;
 }
 return_type RobotSystem::write(const rclcpp::Time & time, const rclcpp::Duration & period)
 {
    //send commands interface values to hardware, e.g joint set joint velocity.

    return return_type::OK;
 }

}
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(arm_hardware::RobotSystem, hardware_interface::SystemInterface)




