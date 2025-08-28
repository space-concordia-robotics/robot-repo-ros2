#include "arm_hardware/r6bot_hardware.hpp"
#include <string>
#include <vector>

#include <iostream>

namespace arm_hardware
{
 CallbackReturn on_init(const hardware_interface::HardwareInfo &info)
 {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackRetrun::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }
    //Setup communication with robot hardware
    return CallbackReturn::SUCCESS;
 }

}
CallbackReturn RobotSystem::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    // Setup communication with robot hardware
{
    return CallbackReturn::SUCCESS;
}
CallbackReturn RobotSystem::read(const rclcpp::Time & time, const rclcpp::Duration &period)
{
    //read hardware values for the state interfaces, ex: joint encoders and sensor readings. 

    return return_type::OK;
}
return_type::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    //send commands interface values to hardware, e.g joint set joint velocity.

    return return_type::OK;
}


#include "pluginlib/class_list_macros.hpp"

PLUGIN_EXPORT_CLASS(arm_hardware::RobotSystem, hardware_interface::SystemInterface)






