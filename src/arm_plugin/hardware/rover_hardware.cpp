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
#include <cstring>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logger.hpp"

// Define a simple serial protocol for clarity
// Header (1 byte): 0x4E
// Length (1 byte): number of payload bytes
// Payload (N bytes): joint positions (floats)
// Footer (1 byte): 0x0A (for writes, not used for reads in this example)

namespace arm_hardware
{

// Enum for the serial read state machine
enum ReadState {
    WAITING_FOR_HEADER,
    READING_LENGTH,
    READING_PAYLOAD
};

CallbackReturn RobotSystem::on_init(const hardware_interface::HardwareInfo &info)
{
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    info_ = info;

    // Read the serial port parameter from the URDF/YAML file
    port_ = info_.hardware_parameters["port"];
    
    // Resize vectors to match number of joints in URDF + initializes all values to 0
    hw_states_positions_.resize(info.joints.size(), 0.0);
    hw_states_velocities_.resize(info.joints.size(), 0.0); // No velocity data is being read, but we need to export the interface
    hw_commands_.resize(info.joints.size(), 0.0);

    RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "on_init() successfully completed.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn RobotSystem::on_configure(const rclcpp_lifecycle::State & previous_state)
{
    SerialPort = open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    
    if (SerialPort < 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("RobotSystem"), "Error %i from open: %s", errno, strerror(errno));
        return CallbackReturn::ERROR;
    }

    if(tcgetattr(SerialPort, &tty) != 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("RobotSystem"), "Error %i from tcgetattr: %s", errno, strerror(errno));
        close(SerialPort);
        return CallbackReturn::ERROR;
    }

    // Termios serial port configuration 
    // control flags
    tty.c_cflag &= ~PARENB; 
    tty.c_cflag &= ~CSTOPB; 
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    // local flags
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHOK;
    tty.c_lflag &= ~ECHONL;
    // input flags
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
    // output flags
    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;
    // control characters
    tty.c_cc[VMIN] = 1; // Read returns after 1 byte
    tty.c_cc[VTIME] = 0; // No inter-byte timeout

    // Baud rate
    speed_t speed = B57600;
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tcflush(SerialPort, TCIOFLUSH);

    if (tcsetattr(SerialPort, TCSANOW, &tty) !=0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("RobotSystem"), "Error %i from tcsetattr: %s", errno, strerror(errno));
        close(SerialPort);
        return CallbackReturn::ERROR;
    }

    // Initialize hardware buffers
    for (uint i = 0; i < hw_states_positions_.size(); i++)
    {
        hw_states_positions_[i] = 0.0;
        hw_commands_[i] = 0.0;
        hw_states_velocities_[i] = 0.0;
    }
    
    // Initialize serial communication state machine
    read_state_ = WAITING_FOR_HEADER;
    read_buffer_pos_ = 0;

    RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Successfully configured! Serial port %s opened: %d", port_.c_str(), SerialPort);
    return CallbackReturn::SUCCESS;
}

CallbackReturn RobotSystem::on_activate(const rclcpp_lifecycle::State & previous_state)
{
    RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Successfully activated!");
    return CallbackReturn::SUCCESS;
}

CallbackReturn RobotSystem::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
    if(SerialPort != -1)
    {
        tcflush(SerialPort, TCIOFLUSH);
        close(SerialPort);
        SerialPort = -1;
    }
    RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Successfully deactivated!");
    return CallbackReturn::SUCCESS;
}

// Reading information from the hardware and then goes to the controller
return_type RobotSystem::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    if(SerialPort == -1)
    {
        return return_type::ERROR;  
    }

    // Read available bytes one by one to avoid issues with partial frames
    uint8_t byte_in;
    ssize_t n_read = 0;
    while((n_read = ::read(SerialPort, &byte_in, 1)) > 0)
    {
        // State machine to parse the serial data frame
        switch (read_state_) {
            case WAITING_FOR_HEADER:
                if (byte_in == 0x4E) {
                    read_state_ = READING_LENGTH;
                }
                break;
            
            case READING_LENGTH:
                read_payload_length_ = byte_in;
                read_buffer_pos_ = 0; // Reset payload buffer position
                if (read_payload_length_ > 0) {
                    read_state_ = READING_PAYLOAD;
                } else {
                    read_state_ = WAITING_FOR_HEADER; // Invalid frame
                }
                break;

            case READING_PAYLOAD:
                read_buffer_[read_buffer_pos_++] = byte_in;
                if (read_buffer_pos_ >= read_payload_length_ && read_payload_length_ > 0) {
                    // We have received the full payload
                    // Validate payload length against the expected size
                    if (read_payload_length_ == hw_states_positions_.size() * sizeof(float)) {
                        for(size_t i = 0; i < hw_states_positions_.size(); i++)
                        {
                            float val;
                            // Use a temporary buffer for safe memcpy
                            uint8_t temp_buf[sizeof(float)];
                            std::memcpy(temp_buf, &read_buffer_[i * sizeof(float)], sizeof(float));
                            std::memcpy(&val, temp_buf, sizeof(float));
                            hw_states_positions_[i] = static_cast<double>(val);
                        }
                        RCLCPP_DEBUG(rclcpp::get_logger("RobotSystem"), "Successfully received and parsed a full frame.");
                    } else {
                        RCLCPP_WARN(rclcpp::get_logger("RobotSystem"), "Received frame with invalid payload length: %d vs expected %d", read_payload_length_, (int)(hw_states_positions_.size() * sizeof(float)));
                    }
                    read_state_ = WAITING_FOR_HEADER;
                }
                break;
        }
    }
    // Check for read errors
    if(n_read < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            RCLCPP_ERROR(rclcpp::get_logger("RobotSystem"), "Serial read error: %s", strerror(errno));
            return return_type::ERROR;
        }
    }
    
    return return_type::OK;
}

// The write sends the controller data back to the hardware data.
return_type RobotSystem::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    if ( SerialPort == -1)
    {
        return return_type::ERROR;
    }
    
    // Total size of the frame we're sending over serial
    constexpr size_t HEADER_SIZE = 2; // Magic byte + length byte
    constexpr size_t FOOTER_SIZE = 1; // Footer byte
    const size_t payload_size = hw_commands_.size() * sizeof(float);
    const size_t frame_length = HEADER_SIZE + payload_size + FOOTER_SIZE; 
    
    std::vector<uint8_t> buf(frame_length);  
    buf[0] = 0x4E;
    buf[1] = static_cast<uint8_t>(payload_size); 

    // Copy floats into buffer starting at index 2
    for(size_t i = 0; i < hw_commands_.size(); i++)
    {
        float val = static_cast<float>(hw_commands_[i]);
        std::memcpy(&buf[2 + i * sizeof(float)], &val, sizeof(float));
    }

    buf[frame_length - 1] = 0x0A; 
    ssize_t written = ::write(SerialPort, buf.data(), frame_length);

    if(written != static_cast<ssize_t>(frame_length))
    {
        RCLCPP_WARN(rclcpp::get_logger("RobotSystem"), "Failed to write full frame to STM32, wrote %ld of %ld bytes.", written, frame_length);
        return return_type::ERROR;
    }

    return return_type::OK; 
}

std::vector<hardware_interface::StateInterface> RobotSystem::export_state_interfaces()
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

} // namespace arm_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(arm_hardware::RobotSystem, hardware_interface::SystemInterface)
