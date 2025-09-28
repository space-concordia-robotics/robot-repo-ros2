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
#include <limits>
#include "rclcpp/logger.hpp"


namespace arm_interface
{


hardware_interface::CallbackReturn ArmInterface::on_init(const hardware_interface::HardwareInfo &info)
{
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    info_ = info;   

    //mapping joint states and commands
    const size_t joint_id = info_.joints.size();
    hw_commands_position_.resize(joint_id, 0.0); //initalizing commands to 0
    hw_states_position_.resize(joint_id, std::numeric_limits<double>::quiet_NaN());
    hw_states_velocity_.resize(joint_id, std::numeric_limits<double>::quiet_NaN());

    //2. Opening encoder port
    std::string port_ = "/dev/ttyUSB0";
    int serial_fd;
    ABSENC_Error_t err = AbsencDriver::OpenPort(port_.c_str(), serial_fd);
    if(err.error != NO_ERROR) {
        RCLCPP_ERROR(rclcpp::get_logger("ArmInterface"), "Failed to open encoder port.");
        return CallbackReturn::ERROR;
    }
    serial_fd_ = serial_fd; //This is the encoder port

    // configure ArmControllerNode with serial port here if needed

    //3. opening and configuring motor port
    // --- Motor port initialization from ArmCOntrollerNode.cpp -- //
    motor_serial_fd_ = open("/dev/ttyTHS1", O_RDWR);
    if(motor_serial_fd_ < 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("ArmInterface"), "Error opening port %s", strerror(errno));
        AbsencDriver::ClosePort(serial_fd_);
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Configure the motor serial port (replication of termios setup)
    struct termios ttycfg;
    memset(&ttycfg, 0, sizeof(ttycfg)); // Initialize to zero
    ttycfg.c_cflag = CS8 | CREAD | CLOCAL; // 8N1, ignore modem signals
    ttycfg.c_lflag = 0;
    ttycfg.c_iflag = 0;
    ttycfg.c_oflag = 0;
    ttycfg.c_cc[VTIME] = 1; // 100ms timeout
    ttycfg.c_cc[VMIN] = 0;  // Return anything read so far
    cfsetispeed(&ttycfg, B57600);
    cfsetospeed(&ttycfg, B57600);
    tcsetattr(motor_serial_fd_, TCSANOW, &ttycfg);

    RCLCPP_INFO(rclcpp::get_logger("ArmInterface"), "Successfully opened motor port /dev/ttyTHS1.");

//4. Validating command interface (checking for positon)
    for(const auto & joint : info_.joints)
    {
        if(joint.command_interfaces.size() != 1 || joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
            RCLCPP_FATAL(rclcpp::get_logger("ArmInterface"), "Joint '%s' has %zu command interfaces, but exactly one position command interface is required", 
                joint.name.c_str(), joint.command_interfaces.size());
            //Best practice to close ports before returning error
            AbsencDriver::ClosePort(serial_fd_);
            close(motor_serial_fd_);
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("ArmInterface"), "on_init() successfully completed.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmInterface::on_configure(const rclcpp_lifecycle::State & previous_state)
{
 (void)previous_state;
 return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmInterface::on_activate(const rclcpp_lifecycle::State & previous_state)
{  
 (void)previous_state;
 return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
 (void)previous_state;
 return hardware_interface::CallbackReturn::SUCCESS;
}

// Reading information from the hardware and then goes to the controller
hardware_interface::return_type ArmInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
   (void)time;
   (void)period;
   
   return return_type::OK;
}

// The write sends the controller data back to the hardware data.
hardware_interface::return_type ArmInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
   (void)time;
   (void)period;

   return return_type::OK; 
}

std::vector<hardware_interface::StateInterface> ArmInterface::export_state_interfaces()
{
   std::vector<hardware_interface::StateInterface> state_interfaces;
   // TODO: Add your state interfaces here
   return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ArmInterface::export_command_interfaces()
{
   std::vector<hardware_interface::CommandInterface> command_interfaces;
   // TODO: Add your command interfaces here
   return command_interfaces;
} 

} // namespace arm_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(arm_interface::ArmInterface, hardware_interface::SystemInterface)
