#include "arm_hardware/arm_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <termios.h>
#include <chrono>
#include <cstring>
#include "rclcpp/logger.hpp"




namespace arm_interface
{


CallbackReturn ArmInterface::on_init(const hardware_interface::HardwareInfo &info)
{
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    info_ = info;

    

    RCLCPP_INFO(rclcpp::get_logger("ArmInterface"), "on_init() successfully completed.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn ArmInterface::on_configure(const rclcpp_lifecycle::State & previous_state)
{

    RCLCPP_INFO(rclcpp::get_logger("ArmInterface"), "Successfully configured! Serial port %s opened: %d", port_.c_str(), SerialPort);
    return CallbackReturn::SUCCESS;
}

CallbackReturn ArmInterface::on_activate(const rclcpp_lifecycle::State & previous_state)
{  
   
}

CallbackReturn ArmInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{

}

// Reading information from the hardware and then goes to the controller
return_type ArmInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
   
    return return_type::OK;
}

// The write sends the controller data back to the hardware data.
return_type ArmInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{

    return return_type::OK; 
}

std::vector<hardware_interface::StateInterface> ArmInterface::export_state_interfaces()
{
   
}

std::vector<hardware_interface::CommandInterface> ArmInterface::export_command_interfaces()
{

} 

} // namespace arm_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(arm_hardware::ArmInterface, hardware_interface::SystemInterface)
