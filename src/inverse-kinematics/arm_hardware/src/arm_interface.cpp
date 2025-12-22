/*
WRITING A HARDWARE INTERFACE:

BACKGROUND:

    A hardware interface is written as a managed life cycle node, this allows for greater control
    over the state of the ROS2 system at hand. it allows ros2_control to ensure that all
    components have been properly intialized correctly before allowing any component
    to begin executing its behaviour. 

    There are 5 different transition states in a lifecycle node: 

    onInit(): This function is used to intialize member variables, declare parameters, and
    set up anything that needs to exist before the lifecycle state machine starts. 
    NOTE: The node is not active at this point. (cant publish or subscribe to date), 
    it is purely meant for initalization and declarations of parameters. 

    onConfigure(): The onConfigure callback is called to allow the node
    to load its configuration and conduct any required setup. Here,
    we initialize publisher/subscribers, create timers, load configs from params, etc.
    The node essentially uses this state to set up any resources it must hold throughout
    its life. 

    onActivate(): This method is expected to do any final preparations to start executing the node. 
    This may include aquiring resources that are only held while the node is active, such as access
    to hardware. Ideally, no prep that requires time some time here should be done. 

    onDeactivate(): Here, we basically clean everything the onActivate() method does.
    In other words, it does the exact opposite the onActivate() node does.

    (and there is also onError(), onCleanup() & onShutdown(), but I will not be using these for this HW interface.)

    ROS2_CONTROL STUFFS

        READ & WRITE METHODS: 

            READ: This function is called periodically by the controller manager's update loop
            -> it reads data from the physical hardware (sensors, encoders, etc)
            -> updates the internal hw_positions_ & hw_velocities_
            -> then sends the update values over to the state interface

            WRITE: 
            -> Takes command values (from controllers via CommandInterfaces)
            -> Sends these commands to the physical hardware (ex: motors drivers)

        EXPORT STATE/COMMAND INTERFACES: 

            EXPORT COMMAND INTERFACES: 
            -> This function tells ROS2 what control inputs your hardware accepts.
            -> returns a vector of CommandInterface objects
            -> Each CommandInterface links a joint name, interface type and a pointer
            to the actual variable in the HW class. 


            EXPORT STATE INTERFACES:
            -> This function tells ros2 what state variables the hardware is providing
            -> returns a vector of StateInterface objects
            -> Each StateInterface links a joint name, interface type and a pointer
            to the actual variable in the HW class. 
            
            */


#include "arm_hardware/arm_interface.hpp"
#include "hardware_interface/system_interface.hpp"
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

//Input: input parameters, initialize hw_states_position_, hw_states_velocity & hw_command_positions
//Outputs: Successful log messages or failure log messages, initialized state and command joints
//Error checks: Check How many joints were found, chech if all joints have valid command and state interfaces and, 
            //check if initialization was successfully completed. 
hardware_interface::CallbackReturn ArmInterface::on_init(const hardware_interface::HardwareInfo &info)
{
    RCLCPP_INFO(rclcpp::get_logger("ArmInterface"), "Starting on_init()...");
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("ArmInterface"), "Error: Failed to complete intialization stage");
        return hardware_interface::CallbackReturn::ERROR;
    } 

    // Debug: Print joint information
    RCLCPP_INFO(rclcpp::get_logger("ArmInterface"), "Found %zu joints:", info_.joints.size());
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        RCLCPP_INFO(rclcpp::get_logger("ArmInterface"), "  Joint %zu: %s", i, info_.joints[i].name.c_str());
    }

    //Initializing state and commands storage
    hw_commands_position_.resize(info_.joints.size(), 0.0);
    hw_states_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_states_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    //Ensure that all joints have valid state and command interfaces
    if(info_.joints.empty())
    {
        RCLCPP_FATAL(rclcpp::get_logger("ArmInterface"), "Found no joint interfaces in da ros2_control URDF");
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("ArmInterface"), " --- Joint Interface Configuration --- ");
    for(const hardware_interface::ComponentInfo& joint: info_.joints)
    {
        RCLCPP_INFO(rclcpp::get_logger("ArmInterface"), "Joint: '%s'", joint.name.c_str());

        // Iterate through each joint and display command interface on terminal screen
        std::string command_interfaces;

        for(const auto & Interface: joint.command_interfaces)
        {
            command_interfaces += " " + Interface.name;
        }
        RCLCPP_INFO(rclcpp::get_logger("ArmInterface"), "Command Interfaces: %s", command_interfaces.c_str());


        //iterating through each joint to now get state interfaces on terminal screen
        std::string state_interfaces;

        for(const auto & Interface : joint.state_interfaces) // Note to self: Interface is an InterfaceInfo object
        {
            state_interfaces += " " + Interface.name;
        }
        RCLCPP_INFO(rclcpp::get_logger("ArmInterface"), "State Interfaces: %s", state_interfaces.c_str());
    }

    RCLCPP_INFO(rclcpp::get_logger("ArmInterface"), "All joints have their respective command and state interfaces!");

    RCLCPP_INFO(rclcpp::get_logger("ArmInterface"), "on_init() successfully completed.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

//Input: Port id's for absenc encoders and motors
//Outputs: Established serial port communications with encoders and motors, checked for exported state and command interfaces
//setting baud rate.
//Error checks: Check if hardware is present and responding correctly, check if port communications have been successfully intialized. 
hardware_interface::CallbackReturn ArmInterface::on_configure(const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    // 1) Ensure info_.joints is non-empty
    if (info_.joints.empty()) {
        RCLCPP_FATAL(rclcpp::get_logger("ArmInterface"), "No joints defined in hardware info!");
        return hardware_interface::CallbackReturn::ERROR;
    }

    //Ensure our internal vectors match the number of joints
    const size_t nj = info_.joints.size();
    hw_commands_position_.resize(nj);
    hw_states_position_.resize(nj);
    hw_states_velocity_.resize(nj);

    // Initialize values (no NaNs left behind)
    for (size_t i = 0; i < nj; i++) {
        hw_commands_position_[i] = 0.0;
        hw_states_position_[i] = 0.0;
        hw_states_velocity_[i] = 0.0;
    }


    if (serial_fd_ == -1) {
        // Attempt to open encoder port 
        int fd;
        ABSENC_Error_t err = AbsencDriver::OpenPort("/dev/ttyUSB0", fd);
        if (err.error != NO_ERROR) {
            RCLCPP_ERROR(rclcpp::get_logger("ArmInterface"),
                         "Failed to open encoder port '/dev/ttyUSB0' in on_configure(): %s.",
                         strAbsencErr(err.error));
            return hardware_interface::CallbackReturn::ERROR;
        } else {
            serial_fd_ = fd;
            RCLCPP_INFO(rclcpp::get_logger("ArmInterface"), "Opened encoder port in on_configure()");
        }
    }

    if (motor_serial_fd_ == -1) {
        motor_serial_fd_ = open("/dev/ttyTHS1", O_RDWR);
        if (motor_serial_fd_ < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("ArmInterface"),
                         "Failed to open motor port '/dev/ttyTHS1' in on_configure(): %s.",
                         strerror(errno));
            return hardware_interface::CallbackReturn::ERROR;
        } else {
            // configure termios same as before
            struct termios ttycfg;
            memset(&ttycfg, 0, sizeof(ttycfg));
            ttycfg.c_cflag = CS8 | CREAD | CLOCAL;
            ttycfg.c_lflag = 0;
            ttycfg.c_iflag = 0;
            ttycfg.c_oflag = 0;
            ttycfg.c_cc[VTIME] = 1;
            ttycfg.c_cc[VMIN] = 0;
            cfsetispeed(&ttycfg, B57600);
            cfsetospeed(&ttycfg, B57600);
            tcsetattr(motor_serial_fd_, TCSANOW, &ttycfg);
            RCLCPP_INFO(rclcpp::get_logger("ArmInterface"), "Opened motor port in on_configure()");
        }
    }

    // 4) Final verification: ensure every joint has exactly one position command interface (as required)
    for (const auto &joint : info_.joints) {
        if (joint.command_interfaces.size() != 1 ||
            joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(rclcpp::get_logger("ArmInterface"),
                         "Joint '%s' must expose exactly one position command interface (found %zu).",
                         joint.name.c_str(), joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("ArmInterface"), "on_configure() completed successfully.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

//Input: Fully initialized joint size vector and obtained joint positions from URDF file. 
//Outputs: SET INITIAL STATE of robot, send commands to hardware, 
//Errors checks: Ensure serial communication is  
hardware_interface::CallbackReturn ArmInterface::on_activate(const rclcpp_lifecycle::State & previous_state)
{ 
     (void)previous_state;

    const size_t nj = info_.joints.size();
    if (hw_states_position_.size() != nj || hw_commands_position_.size() != nj) {
        RCLCPP_FATAL(rclcpp::get_logger("ArmInterface"),
                     "Size mismatch in on_activate(): info_.joints=%zu states=%zu cmds=%zu",
                     nj, hw_states_position_.size(), hw_commands_position_.size());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Copy current state into command buffer to avoid sudden jumps when controller starts
    for (size_t i = 0; i < nj; ++i) {
        if (!std::isnan(hw_states_position_[i])) {
            hw_commands_position_[i] = hw_states_position_[i];
        } else {
            // If state is NaN for some reason, set to zero and warno
            hw_states_position_[i] = 0.0;
            hw_commands_position_[i] = 0.0;
            RCLCPP_WARN(rclcpp::get_logger("ArmInterface"),
                        "hw_states_position_[%zu] was NaN on activate; resetting to 0.", i);
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("ArmInterface"), "Hardware interface activated successfully.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

//Input: Everything that was in onActivate()
//Outputs: Shutting down the hardware interface
//Errors checks: none for now
hardware_interface::CallbackReturn ArmInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    //setting joint state commands back to 0
    for(size_t i = 0; i< hw_commands_position_.size(); i++)
    {
        hw_commands_position_[i] = 0.0;
    }



    RCLCPP_INFO(rclcpp::get_logger("ArmInterface"), "Hardware interface deactivated successfully.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

//Input: Absolute encoder angle readings from user input
//Outputs: Mapping given encorder angles (in radians) to each joint in the URDF and publish angles on terminal  
//Errors checks: Check to see if all encoders are properly working
hardware_interface::return_type ArmInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
   (void)time;
   (void)period;

    ABSENC_Meas_t absenc_meas_1, absenc_meas_2, absenc_meas_3, absenc_meas_4;

    ABSENC_Error_t err1 = AbsencDriver::PollSlave(1, &absenc_meas_1, serial_fd_);
    ABSENC_Error_t err2 = AbsencDriver::PollSlave(2, &absenc_meas_2, serial_fd_);
    ABSENC_Error_t err3 = AbsencDriver::PollSlave(3, &absenc_meas_3, serial_fd_);
    ABSENC_Error_t err4 = AbsencDriver::PollSlave(4, &absenc_meas_4, serial_fd_);

    if (err1.error != NO_ERROR) {
        RCLCPP_ERROR(rclcpp::get_logger("ArmInterface"), "Error on 1: %s cause 0x%04x line 0x%04x\n", strAbsencErr(err1.error), err1.cause, err1.line);
        return return_type::ERROR;
    }
    if (err2.error != NO_ERROR) {
        RCLCPP_ERROR(rclcpp::get_logger("ArmInterface"), "Error on 2: %s cause 0x%04x line 0x%04x\n", strAbsencErr(err2.error), err2.cause, err2.line);
        return return_type::ERROR;
    }
    if (err3.error != NO_ERROR) {
        RCLCPP_ERROR(rclcpp::get_logger("ArmInterface"), "Error on 3: %s cause 0x%04x line 0x%04x\n", strAbsencErr(err3.error), err3.cause, err3.line);
        return return_type::ERROR;
    }
    if (err4.error != NO_ERROR) {
        RCLCPP_ERROR(rclcpp::get_logger("ArmInterface"), "Error on 4: %s cause 0x%04x line 0x%04x\n", strAbsencErr(err4.error), err4.cause, err4.line);
        return return_type::ERROR;
    }

    if (absenc_meas_1.status != 0 || absenc_meas_2.status != 0 || absenc_meas_3.status != 0 || absenc_meas_4.status != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("ArmInterface"),
            "One of the absenc status returned an error. Here are the error codes: 0x%04x 0x%04x 0x%04x 0x%04x\n",
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
    

    // update the old angl
    this -> old_angle_4 = angle_4;
    
    angle_4 = angle_4 + this -> angle_4_zone * 90 - 30;
    /////////////////////////////////////////////////

    const double deg_to_rad = M_PI/180;

    // Map encoders to URDF joint order: joint1, joint2, joint3, joint5
    hw_states_position_[0] = angle_1*deg_to_rad;  // joint1 <- encoder 1
    hw_states_velocity_[0] = absenc_meas_1.angspd * deg_to_rad; 

    hw_states_position_[1] = angle_2*deg_to_rad;  // joint2 <- encoder 2
    hw_states_velocity_[1] = absenc_meas_2.angspd * deg_to_rad;

    hw_states_position_[2] = angle_3*deg_to_rad;  // joint3 <- encoder 3
    hw_states_velocity_[2] = absenc_meas_3.angspd * deg_to_rad;

    hw_states_position_[3] = angle_4*deg_to_rad;  // joint5 <- encoder 4
    hw_states_velocity_[3] = absenc_meas_4.angspd * deg_to_rad;

    if (absenc_meas_1.status == 0 || absenc_meas_2.status == 0 || absenc_meas_3.status == 0 || absenc_meas_4.status == 0)
    {
        RCLCPP_INFO_THROTTLE(rclcpp::get_logger("ArmInterface"), steady_clock_, 5000,
            "Read Pos (rad): [%.3f, %.3f, %.3f, %.3f]",
            hw_states_position_[0], hw_states_position_[1], hw_states_position_[2], hw_states_position_[3]);
    }

   return return_type::OK;
}


//Input: Incoming absenc encoder values 
//Outputs: commands to make the arm motors move upon user input
//Errors  checks: Nan
hardware_interface::return_type ArmInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
   (void)time;
   (void)period;

    if (info_.joints.size() < 4) {
        RCLCPP_ERROR(rclcpp::get_logger("ArmInterface"), "Received JointState message with insufficient data.");
        return return_type::ERROR;
    }

    // Create a buffer to send motor commands
    uint8_t out_buf[1 + 1 + sizeof(float) * 4 + 1] = {}; // 19 bytes total: 1+1+16+1
    out_buf[0] = SET_MOTOR_SPEED;
    out_buf[1] = sizeof(float) * 4;  // 16 bytes of data

    // Map JointState positions to motor speeds (4 motors)
    size_t num_motors = std::min(static_cast<size_t>(4), info_.joints.size());
    
    for (size_t i = 0; i < num_motors; i++) {
        //Calculate p-control velocity command: 
        double positional_error = hw_commands_position_[i] - hw_states_position_[i];
        double velocity_commands_ = std::clamp(positional_error * KP_GAIN, -1.0, 1.0); 
        float speed_to_send = static_cast<float>(velocity_commands_) * MAX_MOTOR_SPEED;
        memcpy(&out_buf[(i * sizeof(float)) + 2], &speed_to_send, sizeof(float));
    }
    
    // Fill remaining motor slots with zero if we have fewer than 4 joints
    for (size_t i = num_motors; i < 4; i++) {
        float zero_speed = 0.0f;
        memcpy(&out_buf[(i * sizeof(float)) + 2], &zero_speed, sizeof(float));
    }
    
    out_buf[18] = 0x0A; // End of message (correct index: 1+1+16 = 18)

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
   
   // Export state interfaces for all joints defined in the URDF
   for(auto i = 0u; i < info_.joints.size(); i++){
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
    }

   RCLCPP_INFO(rclcpp::get_logger("ArmInterface"), "Exported %zu state interfaces", state_interfaces.size());
   return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ArmInterface::export_command_interfaces()
{
   std::vector<hardware_interface::CommandInterface> command_interfaces;
   
   // Export command interfaces for all joints defined in the URDF
   for(auto i = 0u; i < info_.joints.size(); i++)
   {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_position_[i]));
   }

   RCLCPP_INFO(rclcpp::get_logger("ArmInterface"), "Exported %zu command interfaces", command_interfaces.size());
   return command_interfaces;
} 

} // namespace arm_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(arm_interface::ArmInterface, hardware_interface::SystemInterface)

