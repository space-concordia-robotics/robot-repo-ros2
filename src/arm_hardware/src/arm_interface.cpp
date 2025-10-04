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
#include <algorithm>
#include "rclcpp/logger.hpp"

#define KP_GAIN 5.0 //tunable constant. 


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
    hw_commands_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN()); //initalizing commands to 0
    hw_states_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_states_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    //2. Opening encoder port
    std::string port_ = "/dev/ttyUSB0";
    int serial_fd;
    ABSENC_Error_t err = AbsencDriver::OpenPort(port_.c_str(), serial_fd);
    if(err.error != NO_ERROR) {
        RCLCPP_WARN(rclcpp::get_logger("ArmInterface"), "Failed to open encoder port. Running in simulation mode.");
        serial_fd_ = -1; // Mark as invalid for simulation mode
    } else {
        serial_fd_ = serial_fd; //This is the encoder port
    }

    // configure ArmControllerNode with serial port here if needed

    //3. opening and configuring motor port
    // --- Motor port initialization from ArmCOntrollerNode.cpp -- //
    motor_serial_fd_ = open("/dev/ttyTHS1", O_RDWR);
    if(motor_serial_fd_ < 0)
    {
        RCLCPP_WARN(rclcpp::get_logger("ArmInterface"), "Error opening motor port: %s. Running in simulation mode.", strerror(errno));
        motor_serial_fd_ = -1; // Mark as invalid for simulation mode
    } else {
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
    }

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

   // If running in simulation mode (no hardware), provide fake data
   if (serial_fd_ == -1) {
       // Provide some fake joint positions for simulation
       for (size_t i = 0; i < hw_states_position_.size(); ++i) {
           hw_states_position_[i] = 0.0; // or keep current position
           hw_states_velocity_[i] = 0.0;
       }
       return return_type::OK;
   }

    ABSENC_Meas_t absenc_meas_1, absenc_meas_2, absenc_meas_3, absenc_meas_4;

    ABSENC_Error_t err1 = AbsencDriver::PollSlave(1, &absenc_meas_1, serial_fd_);
    ABSENC_Error_t err2 = AbsencDriver::PollSlave(2, &absenc_meas_2, serial_fd_);
    ABSENC_Error_t err3 = AbsencDriver::PollSlave(3, &absenc_meas_3, serial_fd_);
    ABSENC_Error_t err4 = AbsencDriver::PollSlave(4, &absenc_meas_4, serial_fd_);

    if (err1.error != NO_ERROR) {
        RCLCPP_ERROR(rclcpp::get_logger("ArmInterface"), "Error on 1: %s cause %d line %d\n", strAbsencErr(err1.error), err1.cause, err1.line);
        return return_type::ERROR;
    }
    if (err2.error != NO_ERROR) {
        RCLCPP_ERROR(rclcpp::get_logger("ArmInterface"), "Error on 2: %s cause %d line %d\n", strAbsencErr(err2.error), err2.cause, err2.line);
        return return_type::ERROR;
    }
    if (err3.error != NO_ERROR) {
        RCLCPP_ERROR(rclcpp::get_logger("ArmInterface"), "Error on 3: %s cause %d line %d\n", strAbsencErr(err3.error), err3.cause, err3.line);
        return return_type::ERROR;
    }
    if (err4.error != NO_ERROR) {
        RCLCPP_ERROR(rclcpp::get_logger("ArmInterface"), "Error on 4: %s cause %d line %d\n", strAbsencErr(err4.error), err4.cause, err4.line);
        return return_type::ERROR;
    }

    if (absenc_meas_1.status != 0 || absenc_meas_2.status != 0 || absenc_meas_3.status != 0 || absenc_meas_4.status != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("ArmInterface"),
            "One of the absenc status returned an error. Here are the error codes: %d %d %d %d\n",
            absenc_meas_1.status, absenc_meas_2.status, absenc_meas_3.status, absenc_meas_4.status);
        return return_type::ERROR;
    }


    // Fix the Home
    float angle_1 = absenc_meas_1.angval + 25; //-355
    float angle_2 = absenc_meas_2.angval - 174; //-175
    float angle_3 = absenc_meas_3.angval * -1; 
    float angle_4 = absenc_meas_4.angval / 4.0f;

    // Normalize angles to range [-180, 180) rn it's 0 to 360
    //////////////////////////////////////////////////
    angle_1 = angle_1 < 180 ? angle_1 : angle_1 - 360;
    angle_2 = angle_2 > -180 ? angle_2 : angle_2 + 360; 
    
    
    //////////////////////////////////////////////////
    // fixing 1 to 4 ratio of angle 4

    // Finding the zone of the angle 4
    if(old_angle_4 - angle_4 > 70 ) {
        this -> angle_4_zone = (this -> angle_4_zone + 1) % 4;
    }

    if(old_angle_4 - angle_4 < -70 ) {
      this -> angle_4_zone = (this -> angle_4_zone - 1) % 4;
    }


    // update the old angle
    this -> old_angle_4 = angle_4;
    
    angle_4 = angle_4 + this -> angle_4_zone * 90 - 30;
    /////////////////////////////////////////////////

    const double deg_to_rad = M_PI/180;

    hw_states_position_[0] = angle_4*deg_to_rad;
    hw_states_velocity_[0] = absenc_meas_4.angspd * deg_to_rad; 

    hw_states_position_[1] = angle_1*deg_to_rad;
    hw_states_velocity_[1] = absenc_meas_1.angspd * deg_to_rad;

    hw_states_position_[2] = angle_2*deg_to_rad;
    hw_states_velocity_[2] = absenc_meas_2.angspd * deg_to_rad;

    hw_states_position_[3] = angle_3*deg_to_rad;
    hw_states_velocity_[3] = absenc_meas_3.angspd * deg_to_rad;

 
    RCLCPP_INFO(rclcpp::get_logger("ArmInterface"), "Read Pos (rad): [%.3f, %.3f, %.3f, %.3f]",
        hw_states_position_[0], hw_states_position_[1], hw_states_position_[2], hw_states_position_[3]);

   return return_type::OK;
}

// The write sends the controller data back to the hardware data.
hardware_interface::return_type ArmInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
   (void)time;
   (void)period;

   // If running in simulation mode (no hardware), just return OK
   if (motor_serial_fd_ == -1) {
       // In simulation, update position to approach commanded position
       for (size_t i = 0; i < hw_commands_position_.size() && i < hw_states_position_.size(); ++i) {
           // Simple simulation: move towards commanded position
           double error = hw_commands_position_[i] - hw_states_position_[i];
           hw_states_position_[i] += error * 0.1; // 10% of error per cycle
       }
       return return_type::OK;
   }

    if (info_.joints.size() < 7) {
        RCLCPP_ERROR(rclcpp::get_logger("ArmInterface"), "Received JointState message with insufficient velocity data.");
        return return_type::ERROR;
    }

    // Create a buffer to send motor commands
    uint8_t out_buf[1 + 1 + sizeof(float) * 6 + 1] = {}; // Correct buffer size
    out_buf[0] = SET_MOTOR_SPEED;
    out_buf[1] = sizeof(float) * 6;



    // Map JointState velocities to motor speeds
    for (size_t i = 0; i < info_.joints.size(); i++) {

        if( i < info_.joints.size()){
        //Calculate p-control velocity command: 

        double positional_error = hw_commands_position_[i] - hw_states_position_[i];

        double velocity_commands_ = std::clamp(positional_error * KP_GAIN, -1.0, 1.0); 

        float speed_to_send = static_cast<float>(velocity_commands_)* MAX_MOTOR_SPEED;
        memcpy(&out_buf[(i * sizeof(float)) + 2], &speed_to_send, sizeof(float));
        }else{
            // If the motor index is beyond the defined joints, send zero speed
            float zero_speed = 0.0f;
            memcpy(&out_buf[(i * sizeof(float)) + 2], &zero_speed, sizeof(float));
            RCLCPP_WARN_ONCE(rclcpp::get_logger("ArmInterface"), 
                "Motor index %zu is outside the defined joint count. Sending zero speed.", i);
        }
    }
    out_buf[14] = 0x0A; // End of message

     // 4. Send the command buffer via the motor serial port
    int status = ::write(motor_serial_fd_, out_buf, sizeof(out_buf));

    if (status == -1) {
        RCLCPP_ERROR(rclcpp::get_logger("ArmInterface"), "Error writing command to device: %s", strerror(errno));
        return return_type::ERROR;
    }

   return return_type::OK; 
}

std::vector<hardware_interface::StateInterface> ArmInterface::export_state_interfaces()
{
   std::vector<hardware_interface::StateInterface> state_interfaces;
   
   for(auto i = 0u; i< info_.joints.size(); i++){
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
    }

   return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ArmInterface::export_command_interfaces()
{
   std::vector<hardware_interface::CommandInterface> command_interfaces;
   for(auto i = 0u; i< info_.joints.size(); i++)
   {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_position_[i]));
   }

   return command_interfaces;
} 

} // namespace arm_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(arm_interface::ArmInterface, hardware_interface::SystemInterface)
