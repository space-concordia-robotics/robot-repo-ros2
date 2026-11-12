/**
 * @file swerve_drive.cpp
 * @brief Implementation of the `SwerveController` ros2_control lifecycle plugin.
 */

#include "swerve_controller/swerve_drive.hpp"

#include <expected>
#include <ranges>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace swerve_controller {
    SwerveController::SwerveController() = default;

    SwerveController::~SwerveController() = default;

    controller_interface::CallbackReturn SwerveController::on_init() {
        // Initialize the fmt-style ROS 2 logger bound to this lifecycle node
        logger = std::make_shared<ros2_fmt_logger::Logger>(get_node()->get_logger());

        try {
            // Create the auto-generated parameter listener and fetch initial parameter values
            param_listener = std::make_shared<ParamListener>(get_node());
            params = param_listener->get_params();
        } catch (const std::exception& e) {
            logger->error("Exception thrown during swerve controller initialization: {}", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration SwerveController::command_interface_configuration() const {
        // Request individual position command interfaces for the 4 steering joints
        // and velocity command interfaces for the 4 drive wheel joints
        return {
            .type  = controller_interface::interface_configuration_type::INDIVIDUAL,
            .names = {
                // steering joints (commanded in radians)
                fmt::format("{}/position", params.modules.front_left.steering_joint),
                fmt::format("{}/position", params.modules.front_right.steering_joint),
                fmt::format("{}/position", params.modules.rear_left.steering_joint),
                fmt::format("{}/position", params.modules.rear_right.steering_joint),

                // wheel joints (commanded in rad/s)
                fmt::format("{}/velocity", params.modules.front_left.wheel_joint),
                fmt::format("{}/velocity", params.modules.front_right.wheel_joint),
                fmt::format("{}/velocity", params.modules.rear_left.wheel_joint),
                fmt::format("{}/velocity", params.modules.rear_right.wheel_joint),
            },
        };
    }

    controller_interface::InterfaceConfiguration SwerveController::state_interface_configuration() const {
        // Request individual position state feedback for the 4 steering joints
        // and velocity state feedback for the 4 drive wheel joints
        return {
            .type  = controller_interface::interface_configuration_type::INDIVIDUAL,
            .names = {
                // steering joints (measured in radians)
                fmt::format("{}/position", params.modules.front_left.steering_joint),
                fmt::format("{}/position", params.modules.front_right.steering_joint),
                fmt::format("{}/position", params.modules.rear_left.steering_joint),
                fmt::format("{}/position", params.modules.rear_right.steering_joint),

                // wheel joints (measured in rad/s)
                fmt::format("{}/velocity", params.modules.front_left.wheel_joint),
                fmt::format("{}/velocity", params.modules.front_right.wheel_joint),
                fmt::format("{}/velocity", params.modules.rear_left.wheel_joint),
                fmt::format("{}/velocity", params.modules.rear_right.wheel_joint),
            },
        };
    }

    controller_interface::CallbackReturn SwerveController::on_configure(const rclcpp_lifecycle::State& previous_state) {
        // TODO 2026-10-07 (Will Free): can /tf be used for this?

        // Load physical chassis dimensions from ROS parameters
        wheelbase = params.wheelbase;
        track_width = params.track_width;
        wheel_radius = params.wheel_radius;

        // Compute the 2D (x, y) coordinates of each module relative to the chassis center (base_link)
        const double x_front = wheelbase / 2.0;
        const double x_rear = -wheelbase / 2.0;

        const double y_left = track_width / 2.0;
        const double y_right = -track_width / 2.0;

        // Order of modules: [0] front_left, [1] front_right, [2] rear_left, [3] rear_right
        const auto wheel_positions = Eigen::Matrix<double, 4, 2>({
            {x_front, y_left},
            {x_front, y_right},
            {x_rear, y_left},
            {x_rear, y_right},
        });

        // Instantiate the inverse kinematics solver with the computed module positions
        kinematics = std::make_unique<SwerveKinematics>(wheel_positions);

        // Subscribe to TwistStamped velocity commands on ~/cmd_vel
        cmd_vel_subscription = get_node()->create_subscription<TwistStamped>(
            "~/cmd_vel", rclcpp::SystemDefaultsQoS(),
            // ReSharper disable once CppPassValueParameterByConstReference
            // NOLINTNEXTLINE(*-unnecessary-value-param)
            [this](const std::shared_ptr<TwistStamped> msg) -> void {
                // Ignore incoming commands until the controller lifecycle reaches the active state
                if (!subscription_active) {
                    logger->warn("Can't accept new commands. Subscriber is inactive.");
                    return;
                }

                // Populate header timestamp with current node time if publisher left it at zero
                if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) {
                    logger->warn_once("Received TwistStamped with zero timestamp, setting it to current time. This message will only be shown once.");
                    msg->header.stamp = get_node()->now();
                }

                // Copy message into the real-time safe buffer for consumption in update()
                velocity_message.writeFromNonRT(*msg);

                // TODO 2026-10-07 (Will Free): handle timeout
                /*const auto current_time_diff = get_node()->now() - msg->header.stamp;

                if (
                    cmd_vel_timeout_ == rclcpp::Duration::from_seconds(0.0) ||
                    current_time_diff < cmd_vel_timeout_) {
                    received_velocity_msg_.set(*msg);
                } else {
                    RCLCPP_WARN(
                        get_node()->get_logger(),
                        "Ignoring the received message (timestamp %.10f) because it is older than "
                        "the current time by %.10f seconds, which exceeds the allowed timeout (%.4f)",
                        rclcpp::Time(msg->header.stamp).seconds(), current_time_diff.seconds(),
                        cmd_vel_timeout_.seconds());
                }*/
            }
        );

        // Configure velocity, acceleration, and jerk rate limiters from parameters
        try {
            limiter_linear_x = control_toolbox::RateLimiter(
                params.linear.x.min_velocity, params.linear.x.max_velocity,
                params.linear.x.max_acceleration_reverse, params.linear.x.max_acceleration,
                params.linear.x.max_deceleration, params.linear.x.max_deceleration_reverse,
                params.linear.x.min_jerk, params.linear.x.max_jerk
            );
            limiter_linear_y = control_toolbox::RateLimiter(
                params.linear.x.min_velocity, params.linear.x.max_velocity,
                params.linear.x.max_acceleration_reverse, params.linear.x.max_acceleration,
                params.linear.x.max_deceleration, params.linear.x.max_deceleration_reverse,
                params.linear.x.min_jerk, params.linear.x.max_jerk
            );
        } catch (const std::invalid_argument& e) {
            logger->error("Failed to configure command speed limiters: {}", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }
        try {} catch (const std::invalid_argument& e) {
            logger->error("Failed to configure linear speed limiter: {}", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        return ControllerInterface::on_configure(previous_state);
    }


    controller_interface::CallbackReturn SwerveController::on_activate(const rclcpp_lifecycle::State& previous_state) {
        // Helper lambda to locate a specific loaned command interface by joint name and interface type
        const auto find_command_handle = [&logger = logger](
            std::string name,
            std::string interface_name,
            std::vector<hardware_interface::LoanedCommandInterface>& command_interfaces
        ) -> std::expected<std::reference_wrapper<hardware_interface::LoanedCommandInterface>, controller_interface::CallbackReturn> {
            const auto command_handle = std::ranges::find_if(
                command_interfaces,
                [&name,&interface_name](const auto& interface) {
                    return interface.get_prefix_name() == name && interface.get_interface_name() == interface_name;
                }
            );

            if (command_handle == command_interfaces.end()) {
                logger->error("Unable to obtain joint command interface handle for {}/{}", name, interface_name);
                return std::unexpected(controller_interface::CallbackReturn::ERROR);
            }

            return std::ref(*command_handle);
        };

        // Helper lambda to locate a specific loaned state interface by joint name and interface type
        const auto find_state_handle = [&logger = logger](
            const std::string& name,
            const std::string& interface_name,
            const std::vector<hardware_interface::LoanedStateInterface>& state_interfaces
        ) -> std::expected<std::reference_wrapper<const hardware_interface::LoanedStateInterface>, controller_interface::CallbackReturn> {
            const auto state_handle = std::ranges::find_if(
                state_interfaces,
                [&name,&interface_name](const auto& interface) {
                    return interface.get_prefix_name() == name && interface.get_interface_name() == interface_name;
                }
            );

            if (state_handle == state_interfaces.end()) {
                logger->error("Unable to obtain joint state interface handle for {}/{}", name, interface_name);
                return std::unexpected(controller_interface::CallbackReturn::ERROR);
            }

            return std::cref(*state_handle);
        };

        // Helper lambda to bundle all 4 hardware interfaces (steering state/command, wheel state/command) for one module
        const auto find_module_handles = [&find_command_handle, &find_state_handle](
            const std::string& steering_joint,
            const std::string& wheel_joint,
            std::vector<hardware_interface::LoanedCommandInterface>& command_interfaces,
            const std::vector<hardware_interface::LoanedStateInterface>& state_interfaces
        ) -> std::expected<SwerveModuleHandle, controller_interface::CallbackReturn> {
            auto steering_state_handle = find_state_handle(steering_joint, hardware_interface::HW_IF_POSITION, state_interfaces);
            if (!steering_state_handle.has_value())
                return std::unexpected(steering_state_handle.error());

            auto steering_command_handle = find_command_handle(steering_joint, hardware_interface::HW_IF_POSITION, command_interfaces);
            if (!steering_command_handle.has_value())
                return std::unexpected(steering_command_handle.error());

            auto wheel_state_handle = find_state_handle(wheel_joint, hardware_interface::HW_IF_VELOCITY, state_interfaces);
            if (!wheel_state_handle.has_value())
                return std::unexpected(wheel_state_handle.error());

            auto wheel_command_handle = find_command_handle(wheel_joint, hardware_interface::HW_IF_VELOCITY, command_interfaces);
            if (!wheel_command_handle.has_value())
                return std::unexpected(wheel_command_handle.error());

            return SwerveModuleHandle{
                .steering_feedback = steering_state_handle.value(),
                .steering_position = steering_command_handle.value(),
                .wheel_feedback    = wheel_state_handle.value(),
                .wheel_velocity    = wheel_command_handle.value(),
            };
        };

        // Acquire hardware handles for all 4 swerve modules (front-left, front-right, rear-left, rear-right)
        const auto front_left_module = find_module_handles(
            params.modules.front_left.steering_joint, params.modules.front_left.wheel_joint,
            command_interfaces_, state_interfaces_
        );
        if (!front_left_module.has_value())
            return front_left_module.error();

        const auto front_right_module = find_module_handles(
            params.modules.front_right.steering_joint, params.modules.front_right.wheel_joint,
            command_interfaces_, state_interfaces_
        );
        if (!front_right_module.has_value())
            return front_right_module.error();

        const auto rear_left_module = find_module_handles(
            params.modules.rear_left.steering_joint, params.modules.rear_left.wheel_joint,
            command_interfaces_, state_interfaces_
        );
        if (!rear_left_module.has_value())
            return rear_left_module.error();

        const auto rear_right_module = find_module_handles(
            params.modules.rear_right.steering_joint, params.modules.rear_right.wheel_joint,
            command_interfaces_, state_interfaces_
        );
        if (!rear_right_module.has_value())
            return rear_right_module.error();

        // Clear the modules vector in case it wasn't done previously and populate in [FL, FR, RL, RR] order
        modules.clear();

        modules.push_back(front_left_module.value());
        modules.push_back(front_right_module.value());
        modules.push_back(rear_left_module.value());
        modules.push_back(rear_right_module.value());

        // Enable the ~/cmd_vel subscriber callback
        subscription_active = true;

        logger->info("Subscriber and publisher are now active");

        return ControllerInterface::on_activate(previous_state);
    }

    controller_interface::CallbackReturn SwerveController::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
        // Stop accepting new velocity commands and release module interface references
        subscription_active = false;
        modules.clear();
        // TODO 2026-10-07 (Will Free): reset buffers
        // TODO 2026-10-07 (Will Free): stop wheels
        return ControllerInterface::on_deactivate(previous_state);
    }

    controller_interface::return_type SwerveController::update(const rclcpp::Time& /*time*/, const rclcpp::Duration& period) {
        /*
         INVARIANT:
         we assume that if this method is called then modules contains 4 (and exactly 4) entries.
         */
        assert(modules.size() == 4);

        // Read the most recent TwistStamped message from the real-time buffer
        const auto cmd_ptr = velocity_message.readFromRT();
        if (cmd_ptr == nullptr)
            return controller_interface::return_type::OK;

        const auto cmd = *cmd_ptr;

        // Extract desired chassis velocities in base_link frame: linear (vx, vy) and angular (omega_z)
        auto angular_command = cmd.twist.angular.z;
        auto linear_command = Eigen::Vector2d({cmd.twist.linear.x, cmd.twist.linear.y});

        auto& [last_linear, last_angular] = previous_two_commands.at(1);
        auto& [second_to_last_linear, second_to_last_angular] = previous_two_commands.at(0);

        // Apply velocity, acceleration, and jerk limits based on the control loop period (dt)
        limiter_linear_x.limit(linear_command.x(), last_linear.x(), second_to_last_linear.x(), period.seconds());
        limiter_linear_y.limit(linear_command.y(), last_linear.y(), second_to_last_linear.y(), period.seconds());
        limiter_angular.limit(angular_command, last_angular, second_to_last_angular, period.seconds());

        // Shift command history window forward by one step
        previous_two_commands.at(0) = previous_two_commands.at(1);
        previous_two_commands.at(1) = {linear_command, angular_command};

        // TODO 2026-10-07 (Will Free): odometry

        // Read current steering joint angles (radians) from hardware state interfaces
        Eigen::Vector4d module_angles = Eigen::Vector4d::Zero();
        for (const auto& [i, module] : std::views::enumerate(modules)) {
            auto position_feedback_optional = module.steering_feedback.get().get_optional();
            if (!position_feedback_optional.has_value()) {
                logger->debug("Unable to retrieve the position feedback data from swerve module {}", i);
                return controller_interface::return_type::OK;
            }

            // * instead of .value() to avoid branch prediction miss
            const auto position_feedback = *position_feedback_optional;

            if (std::isnan(position_feedback)) {
                logger->error("Position feedback is invalid for swerve module {}", i);
                return controller_interface::return_type::ERROR;
            }

            module_angles[i] = position_feedback;
        }

        // Compute target wheel speeds (m/s) and optimized steering angles (rad) via inverse kinematics
        const auto states = kinematics->inverse(linear_command, angular_command, module_angles);

        assert(states.size() == modules.size());

        auto success = true;

        // pretty sure the compiler will optimize away std::views::zip, but not 100% sure
        // though from looking at godbolt a bit, it seems that it does.
        // - Will Free
        // Write target steering angle (rad) and wheel angular velocity (omega = v / r, in rad/s) to hardware interfaces
        for (const auto& [module, state] : std::views::zip(modules, states)) {
            success &= module.steering_position.get().set_value(state.angle);
            success &= module.wheel_velocity.get().set_value(state.speed / wheel_radius);
        }

        if (!success)
            return controller_interface::return_type::ERROR;

        return controller_interface::return_type::OK;
    }
}

// Register SwerveController as a controller_interface plugin with pluginlib
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(swerve_controller::SwerveController, controller_interface::ControllerInterface)
