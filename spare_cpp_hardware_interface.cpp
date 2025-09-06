#include "hardware/arm_hardware.hpp"
#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <termios.h>
#include <chrono>
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace arm_hardware
{
 CallbackReturn RobotSystem::on_init(const hardware_interface::HardwareInfo &info)
 {
    if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    info_ = info;
    
    //resizes vectors to match number of joints in URDF + initializes all values to 0
    hw_states_positions_.resize(info.joints.size(), 0.0);
    hw_commands_.resize(info.joints.size(), 0.0);
    hw_states_velocities_.resize(info.joints.size(), 0.0);

    return CallbackReturn::SUCCESS;
 }

 CallbackReturn RobotSystem::on_configure(const rclcpp_lifecycle::State & previous_state)
 {
    std::string port = "/dev/ttyUSB0";
    SerialPort = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    
    if (SerialPort < 0)
    {
        RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Error %i from open: %s", errno, strerror(errno));
        return CallbackReturn::ERROR;
    }

    if(tcgetattr(SerialPort, &tty) != 0)
    {
        RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Error %i from tcgetattr: %s", errno, strerror(errno));
        close(SerialPort);
        return CallbackReturn::ERROR;
    }

    //termios serial port configuration 
    //control flags
    tty.c_cflag &= ~PARENB; 
    tty.c_cflag &= ~CSTOPB; 
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    //local flags
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHOK;
    tty.c_lflag &= ~ECHONL;
    //input flags
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
    //output flags
    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;
    //control characters
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 0;

    //baud rate
    speed_t speed = B57600;
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tcflush(SerialPort, TCIOFLUSH);

    if (tcsetattr(SerialPort, TCSANOW, &tty) !=0)
    {
        RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Error %i from tcgetattr: %s", errno, strerror(errno));
        close(SerialPort);
        return CallbackReturn::ERROR;
    }

    //initialize hardware buffers
    for (uint i = 0; i < hw_states_positions_.size(); i++)
    {
        hw_states_positions_[i] = 0.0;
        hw_commands_[i] = 0.0;
        hw_states_velocities_[i] = 0.0;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "SERIAL PORT OPENED: %d", SerialPort);
    RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Successfully configured!");
    return CallbackReturn::SUCCESS;
 }
 CallbackReturn RobotSystem::on_activate(const rclcpp_lifecycle::State & previous_state)
 {
    // to do
    RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Successfully activated!");
    return CallbackReturn::SUCCESS;
 }

 CallbackReturn RobotSystem::on_deactivate(const rclcpp_lifecycle::State & previous_state)
 {
    if(SerialPort == -1)
    {
        return CallbackReturn::SUCCESS;
    }
    tcflush(SerialPort, TCIOFLUSH);
    close(SerialPort);
    RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Successfully deactivated!");
    return CallbackReturn::SUCCESS;
 }
 //Reading information from the hardware and then goes to the controller
 return_type RobotSystem::read(const rclcpp::Time & time, const rclcpp::Duration & period)  //request data from arduino
 {
    if(SerialPort == -1)
    {
        return return_type::ERROR;  
    }

    constexpr int FRAME_LENGTH = 27; 
    uint8_t buf[FRAME_LENGTH];
    int n = ::read(SerialPort, buf, FRAME_LENGTH);

    if(n == FRAME_LENGTH && buf[0] == 0x4E && buf[1] == hw_states_positions_.size() * sizeof(float))
    {
        for(size_t i = 0; i < hw_states_positions_.size(); i++)
        {
            float val; 
            std::memcpy(&val, &buf[2 + i * sizeof(float)], sizeof(float));
            hw_states_positions_[i] = static_cast<double>(val);
            //optionally fill velocities if firmware provides them
        }
    } 
    else 
    {
        RCLCPP_DEBUG(rclcpp::get_logger("RobotSystem"), "Bad or partial fram received");
    }
    
    return return_type::OK;
 }
 //The write sends the controller data back to the hardware data.
 return_type RobotSystem::write(const rclcpp::Time & time, const rclcpp::Duration & period)
 {
    if ( SerialPort == -1)
    {
        return return_type::ERROR;
    }
    //Send 6 floating-point values per joint
    constexpr int FLOAT_COUNT = 6; 
    //Total size of the frame we're sending over serial
    constexpr int FRAME_LENGTH = 27; 

    uint8_t buf[FRAME_LENGTH];  
    buf[0] = 0x4E;
    //math to send bites to joints
    buf[1] = FLOAT_COUNT * sizeof(float); 

    //cope floats into buffer starting at index 2
    for(int i = 0; i < FLOAT_COUNT; i++)
    {
        float val = static_cast<float>(hw_commands_[i]);
        std::memcpy(&buf[2 + i * sizeof(float)], &val, sizeof(float));
    }

    buf[FRAME_LENGTH -1] = 0x0A; 
    int written = ::write(SerialPort, buf, FRAME_LENGTH);
    if(written != FRAME_LENGTH)
    {
        RCLCPP_WARN(rclcpp::get_logger("RobotSystem"), "Failed to write full fram to STM32");
        return return_type::ERROR;
    }

    return return_type::OK; 
 }

 std::vector<hardware_interface::StateInterface> RobotSystem::export_state_interfaces()  //hardware_interface?
 {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for(size_t i = 0; i< hw_states_positions_.size(); i++)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_positions_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]));
    }
    return state_interfaces;
 }

 std::vector<hardware_interface::CommandInterface> RobotSystem::export_command_interfaces()
 {
    std::vector<hardware_interface::CommandInterface> command_interfaces; 
    for(size_t i = 0; i<hw_commands_.size(); i++)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
    }
    return command_interfaces;
 }

}
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(arm_hardware::RobotSystem, hardware_interface::SystemInterface) //hardware_interface ?




