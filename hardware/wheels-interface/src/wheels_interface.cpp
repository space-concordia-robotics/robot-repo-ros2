#include "wheels_interface/wheels_interface.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <ranges>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <fmt/chrono.h>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

#include "scrb_common_util/string_parsing.hpp"


namespace wheels_interface {
    using hardware_interface::HW_IF_POSITION;
    using hardware_interface::HW_IF_VELOCITY;

    constexpr auto ENCODER_MULTIPLIER = 64;

    namespace {
        /**
         * Converts a value from RPM to m/s.
         *
         * @param value RPM
         * @param radius the relevant radius
         * @return m/s
         */
        inline double rpmToMetersPerSecond(const double value, const double radius) {
            return value * std::numbers::pi * 2 * radius / 60.0;
        }

        /**
         * Converts a value from m/s to RPM.
         *
         * @param value m/s
         * @param radius the relevant radius
         * @return RPM
         */
        inline double metersPerSecondToRPM(const double value, const double radius) {
            return value * 60 / (std::numbers::pi * 2 * radius);
        }

        /**
         * Convert a value from rotations to m.
         *
         * @param value rotations
         * @param radius the relevant radius
         * @return rotations
         */
        inline double rotationsToMeters(const double value, const double radius) {
            return value * std::numbers::pi * 2 * radius;
        }
    }

    CallbackReturn RoverSystemWheelsHardware::on_init(const hardware_interface::HardwareComponentInterfaceParams& params) {
        const auto& info = params.hardware_info;

        if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
            return CallbackReturn::ERROR;

        auto rcl_logger = get_logger();

        diagnostic_updater = std::make_shared<diagnostic_updater::Updater>(get_node());
        // TODO 2026-09-28 (Will Free): should each wheel have a unique hardware id?
        diagnostic_updater->setHardwareID(get_hardware_info().name);

        logger = std::make_shared<ros2_fmt_logger::Logger>(rcl_logger);

        if (!info_.hardware_parameters.contains("can_path")) {
            logger->fatal("Missing parameter can_path.");
            return CallbackReturn::ERROR;
        }

        if (!info_.hardware_parameters.contains("multiplier")) {
            logger->fatal("Missing parameter multiplier.");
            return CallbackReturn::ERROR;
        }

        multiplier = scrb::common_util::parse_double(info_.hardware_parameters["multiplier"]);

        using namespace std::chrono_literals;

        const auto parse_rate_parameter = [&](const std::string& name, std::chrono::milliseconds& field) {
            if (info_.hardware_parameters.contains(name)) {
                field = std::chrono::duration_cast<std::chrono::milliseconds>(
                    scrb::common_util::parse_duration(info_.hardware_parameters[name])
                );
            }
        };


        // status periods
        status_periods = StatusPeriods{
            .period0 = 250ms,
            .period1 = 10ms,
            .period2 = 10ms,
            .period3 = 500ms,
            .period4 = 500ms,
        };

        parse_rate_parameter("period0_period", status_periods.period0);
        parse_rate_parameter("period1_period", status_periods.period1);
        parse_rate_parameter("period2_period", status_periods.period2);
        parse_rate_parameter("period3_period", status_periods.period3);
        parse_rate_parameter("period4_period", status_periods.period4);

        // heartbeat period
        // TODO 2026-03-01 (Will Free): is 20ms a correct value for the heartbeat period here?
        heartbeat_period = 20ms;
        parse_rate_parameter("heartbeat_period", heartbeat_period);

        const auto& can_path = info_.hardware_parameters["can_path"];

        can_controller = can_util::CANController::make_shared(can_path, rcl_logger);

        for (auto i = 0u; i < info.joints.size(); i++) {
            const auto& joint = info.joints.at(i);

            auto hasInterface = [&](const std::vector<InterfaceInfo>& interfaces, const std::string& name) {
                return std::ranges::any_of(
                    interfaces,
                    [&](const auto& iface) {
                        return iface.name == name;
                    }
                );
            };

            if (joint.command_interfaces.size() != 1) {
                logger->fatal("Joint '{}' has {} command interface, 1 expected.", joint.name, joint.state_interfaces.size());
                return CallbackReturn::ERROR;
            }

            if (!hasInterface(joint.command_interfaces, HW_IF_VELOCITY)) {
                logger->fatal("Joint '{}' must have '{}' command interface.", joint.name, HW_IF_VELOCITY);
                return CallbackReturn::ERROR;
            }

            if (joint.state_interfaces.size() != 2) {
                logger->fatal("Joint '{}' has {} state interface, 2 expected.", joint.name, joint.state_interfaces.size());
                return CallbackReturn::ERROR;
            }

            if (!hasInterface(joint.state_interfaces, HW_IF_POSITION)) {
                logger->fatal("Joint '{}' must have '{}' state interface.", joint.name, HW_IF_POSITION);
                return CallbackReturn::ERROR;
            }

            if (!hasInterface(joint.state_interfaces, HW_IF_VELOCITY)) {
                logger->fatal("Joint '{}' must have '{}' state interface.", joint.name, HW_IF_VELOCITY);
                return CallbackReturn::ERROR;
            }

            auto parameters = joint.parameters;
            if (!parameters.contains("can_id")) {
                logger->fatal("Joint '{}' does not have parameter can_id.", joint.name);
                return CallbackReturn::ERROR;
            }

            if (!parameters.contains("radius")) {
                logger->fatal("Joint '{}' does not have parameter radius.", joint.name);
                return CallbackReturn::ERROR;
            }

            // TODO 2026-02-14 (Will Free): properly handle errors here

            const auto canId = scrb::common_util::parse_int32(parameters["can_id"]);

            const auto controller = SparkMax::make_shared(rcl_logger, *can_controller, canId);

            const auto radius = scrb::common_util::parse_double(parameters["radius"]);

            const auto wheel = WheelDescription::make_shared(controller, joint.name, radius);

            wheels.push_back(wheel);

            diagnostic_updater->add(fmt::format("{} Motor {} Status", info.name, i), [&, i](auto& stat) {
                // ReSharper disable once CppDeclarationHidesLocal
                const auto wheel = wheels.at(i);
                if (!wheel)
                    return;

                produce_diagnostics(stat, wheel);
            });
        }

        logger->info(
            "Initialized drivetrain with multiplier: {}, canbus path: {}, and period rates 0, 1, 2, 3, 4: {}, {}, {}, {}, {}",
            multiplier,
            can_path,
            status_periods.period0,
            status_periods.period1,
            status_periods.period2,
            status_periods.period3,
            status_periods.period4
        );

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn RoverSystemWheelsHardware::on_configure(const rclcpp_lifecycle::State& previous_state) {
        // TODO 2026-03-01 (Will Free): handle reading initial position values to always adjust them for the state

        // reset values always when configuring hardware
        for (const auto& name : joint_state_interfaces_ | std::views::keys) {
            set_state(name, 0.0);
        }

        for (const auto& name : joint_command_interfaces_ | std::views::keys) {
            set_command(name, 0.0);
        }

        return SystemInterface::on_configure(previous_state);
    }

    CallbackReturn RoverSystemWheelsHardware::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
        logger->info("Activating...");

        if (const auto status = can_controller->initialize(); !status) {
            logger->fatal("Failed to initialize canbus");
            return CallbackReturn::FAILURE;
        }

        // command and state should be equal when starting
        for (const auto& name : joint_command_interfaces_ | std::views::keys) {
            set_command(name, get_state(name));
        }

        // configure period
        for (const auto& wheel : wheels) {
            // sadly, I don't think there's a good way to deduplicate these in a way that cleans it up,
            // due to there being a conditional return here
            // - Will Free

            if (!wheel->motor->setPeriodicStatus0Period(status_periods.period0.count())) {
                logger->fatal("Failed to set period0 rate to {} for wheel {}", status_periods.period0, wheel->name);
                return CallbackReturn::FAILURE;
            }
            if (!wheel->motor->setPeriodicStatus0Period(status_periods.period1.count())) {
                logger->fatal("Failed to set period1 rate to {} for wheel {}", status_periods.period1, wheel->name);
                return CallbackReturn::FAILURE;
            }
            if (!wheel->motor->setPeriodicStatus0Period(status_periods.period2.count())) {
                logger->fatal("Failed to set period2 rate to {} for wheel {}", status_periods.period2, wheel->name);
                return CallbackReturn::FAILURE;
            }
            if (!wheel->motor->setPeriodicStatus0Period(status_periods.period3.count())) {
                logger->fatal("Failed to set period3 rate to {} for wheel {}", status_periods.period3, wheel->name);
                return CallbackReturn::FAILURE;
            }
            if (!wheel->motor->setPeriodicStatus0Period(status_periods.period4.count())) {
                logger->fatal("Failed to set period4 rate to {} for wheel {}", status_periods.period4, wheel->name);
                return CallbackReturn::FAILURE;
            }
        }

        using namespace std::chrono_literals;

        heartbeat_timer = get_node()->create_wall_timer(heartbeat_period, [this] {
            heartbeat();
        });

        logger->info("Successfully activated");

        return CallbackReturn::SUCCESS;
    }

    // TODO 2026-03-01 (Will Free): rolling has stuff like init_hardware_status_message, look at that
    void RoverSystemWheelsHardware::produce_diagnostics(
        diagnostic_updater::DiagnosticStatusWrapper& stat,
        const WheelDescription::ConstSharedPtr& wheel
    ) const {
        using namespace diagnostic_msgs::msg;

        if (!wheel)
            return;

        const auto& motor = wheel->motor;

        const auto faults = motor->getFaults();
        const auto stickyFaults = motor->getStickyFaults();

        if (stickyFaults != 0)
            stat.summary(DiagnosticStatus::ERROR, fmt::format("Motor has {} sticky fault(s)", stickyFaults));
        else if (faults != 0)
            stat.summary(DiagnosticStatus::WARN, fmt::format("Motor has {} non-sticky fault(s)", faults));
        else
            stat.summary(DiagnosticStatus::OK, "Motor is OK");

        // we are adding 15 different fields here.
        // if the fields we send is ever modified, please adjust this.
        stat.values.reserve(15);

        stat.add("name", wheel->name);
        stat.add("faults", faults);
        stat.add("sticky_faults", stickyFaults);

        const auto velocity = motor->getVelocity();
        const auto temperature = motor->getTemperature();
        const auto voltage = motor->getVoltage();
        const auto current = motor->getCurrent();
        const auto position = motor->getPosition();
        const auto iAccum = motor->getIAccum();
        const auto analogVoltage = motor->getAnalogVoltage();
        const auto analogVelocity = motor->getAnalogVelocity();
        const auto analogPosition = motor->getAnalogPosition();
        const auto altEncoderVelocity = motor->getAltEncoderVelocity();
        const auto altEncoderPosition = motor->getAltEncoderPosition();

        stat.add("velocity", velocity);
        stat.add("temperature", temperature);
        stat.add("voltage", voltage);
        stat.add("current", current);
        stat.add("position", position);
        stat.add("i_accum", iAccum);
        stat.add("analog_voltage", analogVoltage);
        stat.add("analog_velocity", analogVelocity);
        stat.add("analog_position", analogPosition);
        stat.add("alt_encoder_velocity", altEncoderVelocity);
        stat.add("alt_encoder_position", altEncoderPosition);

        const auto command = get_command(wheel->velocity_interface_name) * ENCODER_MULTIPLIER;

        const auto targetVelocity = metersPerSecondToRPM(command, wheel->radius) * multiplier;

        stat.add("velocity_command", targetVelocity);
    }

    void RoverSystemWheelsHardware::heartbeat() const {
        if (wheels.size() < 0)
            return;

        // AFAIK you can send the same heartbeat command for all the motors?
        // might need to double check that...

        // ReSharper disable once CppExpressionWithoutSideEffects
        wheels.at(0)->motor->heartbeat();
    }

    CallbackReturn RoverSystemWheelsHardware::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
        logger->info("Deactivating...");

        auto result = CallbackReturn::SUCCESS;

        for (const auto& wheel : wheels) {
            if (const auto status = wheel->motor->setVelocity(0.0); !status)
                result = CallbackReturn::ERROR;
        }

        if (result != CallbackReturn::SUCCESS) {
            logger->error("Failure to deactivate while stopping wheels");
            return result;
        }

        // wait 2 seconds for all the motors to successfully stop
        constexpr auto DEACTIVATION_DELAY = std::chrono::seconds(2);

        std::this_thread::sleep_for(DEACTIVATION_DELAY);

        try {
            if (heartbeat_timer) {
                // NOLINTNEXTLINE(*-ambiguous-smartptr-reset-call): we are calling reset on the heartbeat, not the pointer
                heartbeat_timer->reset();
                heartbeat_timer->cancel();
                heartbeat_timer = nullptr;
            }
        } catch (const std::runtime_error& e) {
            logger->error("Failure to deactivate while stopping heartbeat: {}", e.what());
            return CallbackReturn::ERROR;
        }

        logger->info("Successfully deactivated");

        return result;
    }

    return_type RoverSystemWheelsHardware::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
        for (const auto& wheel : wheels) {
            const auto motor = wheel->motor;
            const auto velocityRPM = static_cast<double>(motor->getVelocity() / ENCODER_MULTIPLIER);
            const auto velocity = rpmToMetersPerSecond(velocityRPM, wheel->radius) / multiplier;

            const auto rotations = static_cast<double>(motor->getPosition() / ENCODER_MULTIPLIER);
            const auto position = rotationsToMeters(rotations, wheel->radius) / multiplier;

            set_state(wheel->velocity_interface_name, velocity);
            set_state(wheel->position_interface_name, position);
        }

        return return_type::OK;
    }

    return_type RoverSystemWheelsHardware::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
        auto result = return_type::OK;
        for (const auto& wheel : wheels) {
            const auto command = get_command(wheel->velocity_interface_name) * ENCODER_MULTIPLIER;

            const auto targetVelocity = metersPerSecondToRPM(command, wheel->radius) * multiplier;

            try {
                if (const auto status = wheel->motor->setVelocity(static_cast<float>(targetVelocity)); !status)
                    result = return_type::ERROR;
            } catch (const std::exception& e) {
                logger->error("Caught exception while attempting to send command to motor: {}", e.what());
                result = return_type::ERROR;
            }
        }

        return result;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(wheels_interface::RoverSystemWheelsHardware, hardware_interface::SystemInterface)
