#pragma once

#include <string>
#include <vector>
#include <hardware_interface/system_interface.hpp>
#include <ros2_fmt_logger/ros2_fmt_logger.hpp>

#include "arm_hardware/absenc.hpp"

static constexpr auto SET_MOTOR_SPEED = 0x4E;
static constexpr auto MAX_MOTOR_SPEED = 1024.f;

namespace arm_interface {
    class ArmInterface : public hardware_interface::SystemInterface {
    public:
        //Lifecycle node override
        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

        // SystemInterface node override
        // TODO 2026-06-13 (Will Free): deprecated
        [[deprecated]] hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

        hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
        //read whatever information the hardware sends us and goes to the controller

        hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;
        //controller updates commands that was received and sends next command to write method so it could send it to the hardware

        // TODO 2026-06-13 (Will Free): deprecated
        [[deprecated]] std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        // TODO 2026-06-13 (Will Free): deprecated
        [[deprecated]] std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    private:
        int serial_fd_ = -1; //for encoder port
        int motor_serial_fd_ = -1; //for arm motor port

        std::string port_ = "/dev/ttyUSB0";

        std::vector<double> hw_states_position_;
        std::vector<double> hw_states_velocity_;
        std::vector<double> hw_commands_velocity_;

        float old_angle_4 = 0;
        int8_t angle_4_zone = 0;


        rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

        std::shared_ptr<ros2_fmt_logger::Logger> logger;
    };
}
