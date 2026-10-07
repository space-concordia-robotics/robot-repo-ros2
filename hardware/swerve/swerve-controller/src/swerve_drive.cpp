#include "swerve_controller/swerve_drive.hpp"

#include <expected>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace swerve_controller {
    SwerveController::SwerveController() {}

    SwerveController::~SwerveController() = default;

    controller_interface::CallbackReturn SwerveController::on_init() {
        auto_declare<std::string>("fl_steering_joint", "fl_steering_joint");
        auto_declare<std::string>("fr_steering_joint", "fr_steering_joint");
        auto_declare<std::string>("rl_steering_joint", "rl_steering_joint");
        auto_declare<std::string>("rr_steering_joint", "rr_steering_joint");

        auto_declare<std::string>("fl_wheel_joint", "fl_wheel_joint");
        auto_declare<std::string>("fr_wheel_joint", "fr_wheel_joint");
        auto_declare<std::string>("rl_wheel_joint", "rl_wheel_joint");
        auto_declare<std::string>("rr_wheel_joint", "rr_wheel_joint");

        auto_declare<double>("wheelbase", 1.0);
        auto_declare<double>("track_width", 1.0);
        auto_declare<double>("wheel_radius", 0.1);

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration SwerveController::command_interface_configuration() const {
        return {
            .type  = controller_interface::interface_configuration_type::INDIVIDUAL,
            .names = {
                // steering joints
                fl_steering_joint + "/position",
                fr_steering_joint + "/position",
                rl_steering_joint + "/position",
                rr_steering_joint + "/position",
                // wheel joints
                fl_wheel_joint + "/velocity",
                fr_wheel_joint + "/velocity",
                rl_wheel_joint + "/velocity",
                rr_wheel_joint + "/velocity",
            },
        };
    }

    controller_interface::InterfaceConfiguration SwerveController::state_interface_configuration() const {
        return {
            .type  = controller_interface::interface_configuration_type::INDIVIDUAL,
            .names = {
                fl_steering_joint + "/position",
                fr_steering_joint + "/position",
                rl_steering_joint + "/position",
                rr_steering_joint + "/position",
            },
        };
    }

    controller_interface::CallbackReturn SwerveController::on_configure(const rclcpp_lifecycle::State& previous_state) {
        fl_steering_joint = get_node()->get_parameter("fl_steering_joint").as_string();
        fr_steering_joint = get_node()->get_parameter("fr_steering_joint").as_string();
        rl_steering_joint = get_node()->get_parameter("rl_steering_joint").as_string();
        rr_steering_joint = get_node()->get_parameter("rr_steering_joint").as_string();


        fl_wheel_joint = get_node()->get_parameter("fl_wheel_joint").as_string();
        fr_wheel_joint = get_node()->get_parameter("fr_wheel_joint").as_string();
        rl_wheel_joint = get_node()->get_parameter("rl_wheel_joint").as_string();
        rr_wheel_joint = get_node()->get_parameter("rr_wheel_joint").as_string();

        // TODO 2026-10-07 (Will Free): can /tf be used for this?

        wheelbase = get_node()->get_parameter("wheelbase").as_double();
        track_width = get_node()->get_parameter("track_width").as_double();
        wheel_radius = get_node()->get_parameter("wheel_radius").as_double();

        const double x_front = wheelbase / 2.0;
        const double x_rear = -wheelbase / 2.0;

        const double y_left = track_width / 2.0;
        const double y_right = -track_width / 2.0;

        const auto wheel_positions = Eigen::Matrix<double, 4, 2>({
            {x_front, y_left},
            {x_front, y_right},
            {x_rear, y_left},
            {x_rear, y_right},
        });

        kinematics = std::make_unique<SwerveKinematics>(wheel_positions);

        cmd_vel_subscription = get_node()->create_subscription<TwistStamped>(
            "~/cmd_vel", rclcpp::SystemDefaultsQoS(),
            // ReSharper disable once CppPassValueParameterByConstReference
            // NOLINTNEXTLINE(*-unnecessary-value-param)
            [this](const std::shared_ptr<TwistStamped> msg) -> void {
                if (!subscription_active) {
                    logger->warn("Can't accept new commands. Subscriber is inactive.");
                    return;
                }

                if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) {
                    logger->warn_once("Received TwistStamped with zero timestamp, setting it to current time. This message will only be shown once.");
                    msg->header.stamp = get_node()->now();
                }

                // copy message into rt buffer
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

        return ControllerInterface::on_configure(previous_state);
    }


    controller_interface::CallbackReturn SwerveController::on_activate(const rclcpp_lifecycle::State& previous_state) {
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

            auto wheel_command_handle = find_command_handle(wheel_joint, hardware_interface::HW_IF_VELOCITY, command_interfaces);
            if (!wheel_command_handle.has_value())
                return std::unexpected(wheel_command_handle.error());

            return SwerveModuleHandle{
                .steering_feedback = steering_state_handle.value(),
                .steering_position = steering_command_handle.value(),
                .wheel_velocity    = wheel_command_handle.value(),
            };
        };

        const auto front_left_module = find_module_handles(fl_steering_joint, fl_wheel_joint, command_interfaces_, state_interfaces_);
        if (!front_left_module.has_value())
            return front_left_module.error();

        const auto front_right_module = find_module_handles(fr_steering_joint, fr_wheel_joint, command_interfaces_, state_interfaces_);
        if (!front_right_module.has_value())
            return front_right_module.error();

        const auto rear_left_module = find_module_handles(rl_steering_joint, fl_wheel_joint, command_interfaces_, state_interfaces_);
        if (!rear_left_module.has_value())
            return rear_left_module.error();

        const auto rear_right_module = find_module_handles(rr_steering_joint, fr_wheel_joint, command_interfaces_, state_interfaces_);
        if (!rear_right_module.has_value())
            return rear_right_module.error();

        // clear the modules vector in case it wasn't done previously for some reason
        modules.clear();

        modules.push_back(front_left_module.value());
        modules.push_back(front_right_module.value());
        modules.push_back(rear_left_module.value());
        modules.push_back(rear_right_module.value());

        subscription_active = true;

        logger->info("Subscriber and publisher are now active");

        return ControllerInterface::on_activate(previous_state);
    }

    controller_interface::CallbackReturn SwerveController::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
        subscription_active = false;
        modules.clear();
        // TODO 2026-10-07 (Will Free): reset buffers
        // TODO 2026-10-07 (Will Free): stop wheels
        return ControllerInterface::on_deactivate(previous_state);
    }

    controller_interface::return_type SwerveController::update(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
        /*
         INVARIANT:
         we assume that if this method is called then modules contains 4 (and exactly 4) entries.
         */
        assert(modules.size() == 4);

        const auto cmd_ptr = velocity_message.readFromRT();
        if (cmd_ptr == nullptr)
            return controller_interface::return_type::OK;

        const auto cmd = *cmd_ptr;

        const double vx = cmd.twist.linear.x;
        const double vy = cmd.twist.linear.y;
        const double wz = cmd.twist.angular.z;

        // TODO 2026-10-07 (Will Free): odometry

        Eigen::Vector4d angles = Eigen::Vector4d::Zero();
        for (auto i = 0u; i < modules.size(); i++) {
            // NOLINTNEXTLINE(*-pro-bounds-avoid-unchecked-container-access): we're using the size to iterate over it, so this is safe
            auto position_feedback_optional = modules[i].steering_feedback.get().get_optional();
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

            angles[i] = position_feedback;
        }

        const auto states = kinematics->inverse({vx, vy}, wz, angles);

        assert(states.size() == modules.size());

        auto success = true;

        for (auto i = 0u; i < modules.size(); i++) {
            // NOLINTNEXTLINE(*-pro-bounds-avoid-unchecked-container-access): we're using the size to iterate over it, so this is safe
            const auto& module = modules[i];
            // NOLINTNEXTLINE(*-pro-bounds-avoid-unchecked-container-access, *-pro-bounds-constant-array-index): modules and states are always the same size
            const auto& [speed, angle] = states[i];
            success &= module.steering_position.get().set_value(angle);
            success &= module.wheel_velocity.get().set_value(states.at(0).speed / wheel_radius);
        }

        if (!success)
            return controller_interface::return_type::ERROR;

        return controller_interface::return_type::OK;
    }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(swerve_controller::SwerveController, controller_interface::ControllerInterface)
