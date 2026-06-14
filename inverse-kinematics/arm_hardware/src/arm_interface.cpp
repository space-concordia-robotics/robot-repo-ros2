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

#include <cstring>
#include <fcntl.h>
#include <limits>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>

static constexpr auto KP_GAIN = 5.0; // tunable constant.
static constexpr auto MAX_JOINT_VELOCITY = 1.0;

using hardware_interface::return_type;

namespace arm_interface {
    // Input: input parameters, initialize hw_states_position_, hw_states_velocity & hw_command_positions
    // Outputs: Successful log messages or failure log messages, initialized state and command joints
    // Error checks: Check How many joints were found, chech if all joints have valid command and state interfaces and,
    // check if initialization was successfully completed.
    hardware_interface::CallbackReturn ArmInterface::on_init(const hardware_interface::HardwareInfo& info) {
        RCLCPP_INFO(this->get_logger(), "Starting on_init()...");
        // ReSharper disable once CppDeprecatedEntity
        if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Error: Failed to complete intialization stage");
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Debug: Print joint information
        RCLCPP_INFO(this->get_logger(), "Found %zu joints:", info_.joints.size());
        for (size_t i = 0; i < info_.joints.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "  Joint %zu: %s", i, info_.joints.at(i).name.c_str());
        }

        // Initializing state and commands storage
        hw_commands_velocity_.resize(info_.joints.size(), 0.0);
        hw_states_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_states_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

        // Ensure that all joints have valid state and command interfaces
        if (info_.joints.empty()) {
            RCLCPP_FATAL(this->get_logger(), "Found no joint interfaces in da ros2_control URDF");
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(this->get_logger(), " --- Joint Interface Configuration --- ");
        for (const hardware_interface::ComponentInfo& joint : info_.joints) {
            RCLCPP_INFO(this->get_logger(), "Joint: '%s'", joint.name.c_str());

            // Iterate through each joint and display command interface on terminal screen
            std::string command_interfaces;

            for (const auto& Interface : joint.command_interfaces) {
                command_interfaces += " " + Interface.name;
            }
            RCLCPP_INFO(this->get_logger(), "Command Interfaces: %s", command_interfaces.c_str());

            // iterating through each joint to now get state interfaces on terminal screen
            std::string state_interfaces;

            for (const auto& Interface : joint.state_interfaces) // Note to self: Interface is an InterfaceInfo object
            {
                state_interfaces += " " + Interface.name;
            }
            RCLCPP_INFO(this->get_logger(), "State Interfaces: %s", state_interfaces.c_str());
        }

        RCLCPP_INFO(this->get_logger(), "All joints have their respective command and state interfaces!");

        RCLCPP_INFO(this->get_logger(), "on_init() successfully completed.");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // Input: Port id's for absenc encoders and motors
    // Outputs: Established serial port communications with encoders and motors, checked for exported state and command interfaces
    // setting baud rate.
    // Error checks: Check if hardware is present and responding correctly, check if port communications have been successfully intialized.
    hardware_interface::CallbackReturn ArmInterface::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
        // 1) Ensure info_.joints is non-empty
        if (info_.joints.empty()) {
            RCLCPP_FATAL(this->get_logger(), "No joints defined in hardware info!");
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Ensure our internal vectors match the number of joints
        const size_t nj = info_.joints.size();
        hw_commands_velocity_.resize(nj, 0.0);
        hw_states_position_.resize(nj, 0.0);
        hw_states_velocity_.resize(nj, 0.0);

        if (serial_fd_ == -1) {
            // Attempt to open encoder port
            int fd = 0;
            if (const ABSENC_Error_t err = AbsencDriver::OpenPort("/dev/ttyUSB0", fd); err.error != NO_ERROR) {
                RCLCPP_ERROR(this->get_logger(),
                             "Failed to open encoder port '/dev/ttyUSB0' in on_configure(): %s.",
                             strAbsencErr(err.error));
                return hardware_interface::CallbackReturn::ERROR;
            } else {
                serial_fd_ = fd;
                RCLCPP_INFO(this->get_logger(), "Opened encoder port in on_configure()");
            }
        }

        if (motor_serial_fd_ == -1) {
            motor_serial_fd_ = open("/dev/ttyTHS1", O_RDWR); // NOLINT(*-pro-type-vararg)
            if (motor_serial_fd_ < 0) {
                RCLCPP_ERROR(this->get_logger(),
                             "Failed to open motor port '/dev/ttyTHS1' in on_configure(): %s.",
                             strerrordesc_np(errno));
                return hardware_interface::CallbackReturn::ERROR;
            } else {
                termios ttycfg{};
                ttycfg.c_cflag = CS8 | CREAD | CLOCAL; // 8N1, ignore modem signals
                ttycfg.c_lflag = 0;
                ttycfg.c_iflag = 0;
                ttycfg.c_oflag = 0;
                ttycfg.c_line = 0;
                ttycfg.c_cc[VTIME] = 1; // 100ms timeout
                ttycfg.c_cc[VMIN] = 0; // Return anything read so far
                cfsetispeed(&ttycfg, B57600);
                cfsetospeed(&ttycfg, B57600);

                tcsetattr(motor_serial_fd_, TCSANOW, &ttycfg);

                RCLCPP_INFO(this->get_logger(), "Opened motor port in on_configure()");
            }
        }

        // 4) Final verification: ensure every joint has exactly one velocity command interface (as required)
        for (const auto& joint : info_.joints) {
            if (joint.command_interfaces.size() != 1 ||
                joint.command_interfaces.at(0).name != hardware_interface::HW_IF_VELOCITY) {
                RCLCPP_FATAL(this->get_logger(),
                             "Joint '%s' must expose exactly one velocity command interface (found %zu).",
                             joint.name.c_str(), joint.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        RCLCPP_INFO(this->get_logger(), "on_configure() completed successfully.");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // Input: Fully initialized joint size vector and obtained joint positions from URDF file.
    // Outputs: SET INITIAL STATE of robot, send commands to hardware,
    // Errors checks: Ensure serial communication is
    hardware_interface::CallbackReturn ArmInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
        const size_t nj = info_.joints.size();
        if (hw_states_position_.size() != nj || hw_commands_velocity_.size() != nj) {
            RCLCPP_FATAL(this->get_logger(),
                         "Size mismatch in on_activate(): info_.joints=%zu states=%zu cmds=%zu",
                         nj, hw_states_position_.size(), hw_commands_velocity_.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Copy current state into command buffer to avoid sudden jumps when controller starts
        for (size_t i = 0; i < nj; ++i) {
            if (!std::isnan(hw_states_position_.at(i))) {
                hw_commands_velocity_.at(i) = hw_states_position_.at(i);
            } else {
                // If state is NaN for some reason, set to zero and warno
                hw_states_position_.at(i) = 0.0;
                hw_commands_velocity_.at(i) = 0.0;
                RCLCPP_WARN(this->get_logger(),
                            "hw_states_position_[%zu] was NaN on activate; resetting to 0.", i);
            }
        }

        RCLCPP_INFO(this->get_logger(), "Hardware interface activated successfully.");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // Input: Everything that was in on_activate()
    // Outputs: Shutting down the hardware interface
    // Errors checks: none for now
    hardware_interface::CallbackReturn ArmInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
        // setting joint state commands back to 0
        for (double& velocity : hw_commands_velocity_) {
            velocity = 0.0;
        }

        RCLCPP_INFO(this->get_logger(), "Hardware interface deactivated successfully.");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // Input: Absolute encoder angle readings from user input
    // Outputs: Mapping given encorder angles (in radians) to each joint in the URDF and publish angles on terminal
    // Errors checks: Check to see if all encoders are properly working
    return_type ArmInterface::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
        ABSENC_Meas_t absenc_meas_1{};
        ABSENC_Meas_t absenc_meas_2{};
        ABSENC_Meas_t absenc_meas_3{};
        ABSENC_Meas_t absenc_meas_4{};

        auto poll = [&](const int id, ABSENC_Meas_t& meas) -> bool {
            if (const auto [err, cause, line] = AbsencDriver::PollSlave(id, &meas, serial_fd_); err != NO_ERROR) {
                RCLCPP_ERROR(this->get_logger(), "Error on %d: %s cause 0x%04x line 0x%04x\n", id, strAbsencErr(err), cause, line);
                return false;
            }

            return true;
        };

        const auto measurements = std::array{&absenc_meas_1, &absenc_meas_2, &absenc_meas_3, &absenc_meas_4};

        for (int i = 0; i < 4; ++i)
            if (!poll(i + 1, *measurements.at(i)))
                return return_type::ERROR;

        if (absenc_meas_1.status != 0 || absenc_meas_2.status != 0 || absenc_meas_3.status != 0 || absenc_meas_4.status != 0) {
            RCLCPP_ERROR(this->get_logger(),
                         "One of the absenc status returned an error. Here are the error codes: 0x%04x 0x%04x 0x%04x 0x%04x\n",
                         absenc_meas_1.status, absenc_meas_2.status, absenc_meas_3.status, absenc_meas_4.status);
            return return_type::ERROR;
        }

        // Fix the Home
        // TODO 2026-06-13 (Will Free): where are these constants from?
        float angle_1 = absenc_meas_1.angval + 25; //-355
        float angle_2 = absenc_meas_2.angval - 174; //-175
        float angle_3 = absenc_meas_3.angval * -1;
        float angle_4 = absenc_meas_4.angval / 4.0f;

        // Normalize angles to range [-180, 180) rn it's 0 to 360
        //////////////////////////////////////////////////
        angle_1 = angle_1 < 180 ? angle_1 : angle_1 - 360;
        angle_2 = angle_2 > -180 ? angle_2 : angle_2 + 360;

        // update the old angle
        this->old_angle_4 = angle_4;

        angle_4 = angle_4 + this->angle_4_zone * 90 - 30;
        /////////////////////////////////////////////////

        constexpr double deg_to_rad = M_PI / 180;

        // Map encoders to URDF joint order: joint1, joint2, joint3, joint5
        hw_states_position_.at(0) = angle_1 * deg_to_rad; // joint1 <- encoder 1
        hw_states_velocity_.at(0) = absenc_meas_1.angspd * deg_to_rad;

        hw_states_position_.at(1) = angle_2 * deg_to_rad; // joint2 <- encoder 2
        hw_states_velocity_.at(1) = absenc_meas_2.angspd * deg_to_rad;

        hw_states_position_.at(2) = angle_3 * deg_to_rad; // joint3 <- encoder 3
        hw_states_velocity_.at(2) = absenc_meas_3.angspd * deg_to_rad;

        hw_states_position_.at(3) = angle_4 * deg_to_rad; // joint5 <- encoder 4
        hw_states_velocity_.at(3) = absenc_meas_4.angspd * deg_to_rad;

        if (absenc_meas_1.status == 0 || absenc_meas_2.status == 0 || absenc_meas_3.status == 0 || absenc_meas_4.status == 0) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), steady_clock_, 10,
                                 "Read Pos (rad): [%.3f, %.3f, %.3f, %.3f]",
                                 hw_states_position_.at(0), hw_states_position_.at(1), hw_states_position_.at(2), hw_states_position_.at(3));
        }

        return return_type::OK;
    }

    // Input: Incoming absenc encoder values
    // Outputs: commands to make the arm motors move upon user input
    // Errors  checks: Nan
    return_type ArmInterface::write(const rclcpp::Time& time, const rclcpp::Duration& period) {
        (void)time;
        (void)period;

        if (info_.joints.size() < 4) {
            RCLCPP_ERROR(this->get_logger(), "Received JointState message with insufficient data.");
            return return_type::ERROR;
        }

        // Create a buffer to send motor commands
        std::array<uint8_t, 1 + 1 + sizeof(float) * 6 + 1> out_buf = {
            // 19 bytes total: 1+1+16+1
            SET_MOTOR_SPEED,
            sizeof(float) * 6 // 16 bytes of data
        };

        // const auto joint_state = sensor_msgs::msg::JointState::SharedPtr();
        // joint_state->velocity = hw_commands_velocity_;

        // Map command velocities to motor speeds
        for (size_t i = 0; i < hw_commands_velocity_.size() && i < 4; i++) {
            const double joint_velocities = hw_commands_velocity_.at(i); // in rad/s
            float speed = static_cast<float>(joint_velocities) * MAX_MOTOR_SPEED;
            memcpy(&out_buf.at(i * sizeof(float) + 2), &speed, sizeof(float));
        }
        out_buf.at(14) = 0x0A; // End of message

        RCLCPP_INFO_THROTTLE(rclcpp::get_logger("ArmInterfac"), steady_clock_, 10, "Writing joint velocity commands [%.2f, %.2f, %.2f, %.2f]",
                             hw_commands_velocity_.at(0), hw_commands_velocity_.at(1), hw_commands_velocity_.at(2), hw_commands_velocity_.at(3));

        // Send the motor commands via the motor serial port
        const int status = ::write(motor_serial_fd_, out_buf.data(), sizeof(out_buf));
        if (status == -1) {
            RCLCPP_ERROR(this->get_logger(), "SHORT WRITE: %d/%zu (%s)", status, sizeof(out_buf), strerrordesc_np(errno));
            return return_type::ERROR;
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), steady_clock_, 10, "status: %d, sizeof out buf: %zu (errno: %s)", status, sizeof(out_buf),
                             strerrordesc_np(errno));
        // RCLCPP_INFO_THROTTLE(this->get_logger(), steady_clock_, 10, "Got status: %d", status);

        return return_type::OK;
    }

    std::vector<hardware_interface::StateInterface> ArmInterface::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        state_interfaces.reserve(info_.joints.size() * 2);

        // Export state interfaces for all joints defined in the URDF
        for (auto i = 0u; i < info_.joints.size(); i++) {
            // ReSharper disable CppDeprecatedEntity
            state_interfaces.emplace_back(
                info_.joints.at(i).name, hardware_interface::HW_IF_POSITION, &hw_states_position_.at(i));
            state_interfaces.emplace_back(
                info_.joints.at(i).name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_.at(i));
            // ReSharper restore CppDeprecatedEntity
        }

        RCLCPP_INFO(this->get_logger(), "Exported %zu state interfaces", state_interfaces.size());
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> ArmInterface::export_command_interfaces() {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        command_interfaces.reserve(info_.joints.size());
        // Export command interfaces for all joints defined in the URDF
        for (auto i = 0u; i < info_.joints.size(); i++) {
            // ReSharper disable CppDeprecatedEntity
            command_interfaces.emplace_back(info_.joints.at(i).name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocity_.at(i));
            // ReSharper restore CppDeprecatedEntity
        }

        RCLCPP_INFO(this->get_logger(), "Exported %zu command interfaces", command_interfaces.size());
        return command_interfaces;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(arm_interface::ArmInterface, hardware_interface::SystemInterface)
