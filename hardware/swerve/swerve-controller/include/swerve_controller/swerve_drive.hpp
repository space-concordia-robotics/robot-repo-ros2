#pragma once

#include <memory>
#include <control_toolbox/rate_limiter.hpp>
#include <controller_interface/controller_interface.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <ros2_fmt_logger/logger.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include "swerve_kinematics.hpp"
#include "swerve_controller/swerve_controller_parameters.hpp"

namespace swerve_controller {
    class SwerveController : public controller_interface::ControllerInterface {
        using TwistStamped = geometry_msgs::msg::TwistStamped;

    public:
        SwerveController();

        ~SwerveController() override;

        controller_interface::CallbackReturn on_init() override;

        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

        controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

        controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

        controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    private:
        std::shared_ptr<ros2_fmt_logger::Logger> logger;

        std::shared_ptr<ParamListener> param_listener;
        Params params;

        // TODO 2026-10-07 (Will Free): can I avoid the pointer here?
        std::unique_ptr<SwerveKinematics> kinematics = nullptr;

        double wheelbase = 1.0;
        double track_width = 1.0;
        double wheel_radius = 0.1;

        bool subscription_active = false;

        rclcpp::Subscription<TwistStamped>::SharedPtr cmd_vel_subscription;

        // TODO 2026-10-07 (Will Free): replace with RealtimeThreadSafeBox
        realtime_tools::RealtimeBuffer<TwistStamped> velocity_message;

        struct SwerveModuleJoints {
            std::string steering_joint;
            std::string wheel_joint;
        };

        std::array<SwerveModuleJoints, 4> joints;

        struct SwerveModuleHandle {
            std::reference_wrapper<const hardware_interface::LoanedStateInterface> steering_feedback;
            std::reference_wrapper<hardware_interface::LoanedCommandInterface> steering_position;
            std::reference_wrapper<const hardware_interface::LoanedStateInterface> wheel_feedback;
            std::reference_wrapper<hardware_interface::LoanedCommandInterface> wheel_velocity;
        };

        // I want to use an std::array<SwerveModuleHandle, 4> here, but can't because it requires all the structs be initialized in the constructor
        std::vector<SwerveModuleHandle> modules;

        std::array<std::tuple<Eigen::Vector2d, double>, 2> previous_two_commands;

        control_toolbox::RateLimiter<double> limiter_linear_x;
        control_toolbox::RateLimiter<double> limiter_linear_y;
        control_toolbox::RateLimiter<double> limiter_angular;
    };
}
