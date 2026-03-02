#include "wheels_interface/wheels_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <hardware_interface/lexical_casts.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <boost/algorithm/string.hpp>

namespace wheels_interface {
    constexpr auto HW_IF_VELOCITY = hardware_interface::HW_IF_VELOCITY;
    constexpr auto HW_IF_POSITION = hardware_interface::HW_IF_POSITION;

    constexpr auto ENCODER_MULTIPLIER = 64;

    std::string camelCaseToSnakeCase(const std::string_view& input) {
        std::string result;
        result.reserve(input.size());

        for (size_t i = 0; i < input.size(); ++i) {
            const auto c = input[i];

            if (i > 0 && isupper(c)) {
                if (islower(input[i - 1]))
                    result.push_back('_');

                if (const auto isLast = i + 1 == input.size(); !isLast && islower(input[i + 1]))
                    result.push_back('_');
            } else {
                result.push_back(c);
            }

            result.push_back(tolower(c));
        }

        return result;
    }

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

#if HARDWARE_INTERFACE_IS_JAZZY
    CallbackReturn RoverSystemWheelsHardware::on_init(const hardware_interface::HardwareComponentInterfaceParams& params) {
        const auto& info = params.hardware_info;
        const auto& executor = params.executor;

        if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
            return CallbackReturn::ERROR;

        auto rcl_logger = get_logger();

        // if (auto locked_executor = executor.lock()) {
        //     // ... executor is valid and can be used here ...
        // } else {
        //     return hardware_interface::CallbackReturn::ERROR;
        // }

        const auto node = get_node();
#else // @formatter:off
    CallbackReturn RoverSystemWheelsHardware::on_init(const HardwareInfo& info) {
        if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
            return CallbackReturn::ERROR;

        auto rcl_logger = rclcpp::get_logger("RoverSystemWheelsHardwareBot");

        clock = std::make_shared<rclcpp::Clock>();

        const auto nodeName = camelCaseToSnakeCase(info.name);
        const auto node = std::make_shared<rclcpp::Node>(nodeName);
#endif // @formatter:on

        diagnostic_updater = std::make_shared<diagnostic_updater::Updater>(node);
        diagnostic_updater->setHardwareID(get_hardware_info().name);

        if (!node) {
            logger->fatal("Could not get/initialize node");
            return CallbackReturn::ERROR;
        }

        logger = std::make_shared<ros2_fmt_logger::Logger>(rcl_logger);

        if (!info_.hardware_parameters.contains("can_path")) {
            logger->fatal("Missing parameter can_path.");
            return CallbackReturn::ERROR;
        }

        if (!info_.hardware_parameters.contains("multiplier")) {
            logger->fatal("Missing parameter multiplier.");
            return CallbackReturn::ERROR;
        }

        multiplier = hardware_interface::stod(info_.hardware_parameters["multiplier"]);

        const auto can_path = info_.hardware_parameters["can_path"];

        can_controller = std::make_shared<CANController>(can_path, rcl_logger);

        for (auto i = 0u; i < info.joints.size(); i++) {
            const auto& joint = info.joints[i];
            // DiffBotSystem has exactly two states and one command interface on each joint
            if (joint.command_interfaces.size() != 1) {
                logger->fatal("Joint '{}' has {} command interfaces found. 1 expected.", joint.name, joint.command_interfaces.size());
                return CallbackReturn::ERROR;
            }

            // TODO 2026-02-25 (Will Free): look at gazebo's hardware interface to see how it handles different orderings of the interfaces?

            if (joint.command_interfaces[0].name != HW_IF_VELOCITY) {
                logger->fatal("Joint '{}' has {} command interface. '{}' expected.", joint.name, joint.command_interfaces[0].name.c_str(), HW_IF_VELOCITY);
                return CallbackReturn::ERROR;
            }

            if (joint.state_interfaces.size() != 2) {
                logger->fatal("Joint '{}' has {} state interface. 2 expected.", joint.name, joint.state_interfaces.size());
                return CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[0].name != HW_IF_POSITION) {
                logger->fatal("Joint '{}' has '{}' as first state interface. '{}' expected.", joint.name, joint.state_interfaces[0].name, HW_IF_POSITION);
                return CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[1].name != HW_IF_VELOCITY) {
                logger->fatal("Joint '{}' has '{}' as second state interface. '{}' expected.", joint.name, joint.state_interfaces[1].name, HW_IF_VELOCITY);
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

            const auto canId = stoi(parameters["can_id"]);

            const auto controller = SparkMax::make_shared(rcl_logger, *can_controller, canId);

            const auto radius = hardware_interface::stod(parameters["radius"]);

            const auto wheel = WheelDescription::make_shared(controller, joint.name, radius);

            wheels.push_back(wheel);

            const auto diagnosticCallback = std::bind(&RoverSystemWheelsHardware::produce_diagnostics, std::placeholders::_1, std::placeholders::_2, wheel);
            diagnostic_updater->add(fmt::format("{} Motor {} Status", info.name, i), [&](auto& stat) {
                produce_diagnostics(stat, wheel);
            });
        }

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn RoverSystemWheelsHardware::on_configure(const rclcpp_lifecycle::State& previous_state) {
        // TODO 2026-03-01 (Will Free): handle reading initial position values to always adjust them for the state

        // reset values always when configuring hardware
        for (const auto& [name, descr] : joint_state_interfaces_) {
            set_state(name, 0.0);
        }

        for (const auto& [name, descr] : joint_command_interfaces_) {
            set_command(name, 0.0);
        }

        return SystemInterface::on_configure(previous_state);
    }

#if !HARDWARE_INTERFACE_IS_JAZZY
    std::vector<StateInterface> RoverSystemWheelsHardware::export_state_interfaces() {
        std::vector<StateInterface> state_interfaces = {};

        state_interfaces.reserve(wheels.size() * 2);
        for (const auto& wheel : wheels) {
            state_interfaces.emplace_back(wheel->name, hardware_interface::HW_IF_POSITION, &wheel->position_state);
            state_interfaces.emplace_back(wheel->name, hardware_interface::HW_IF_VELOCITY, &wheel->velocity_state);
        }

        return state_interfaces;
    }

    std::vector<CommandInterface> RoverSystemWheelsHardware::export_command_interfaces() {
        std::vector<CommandInterface> command_interfaces = {};

        command_interfaces.reserve(wheels.size());
        for (const auto& wheel : wheels) {
            command_interfaces.emplace_back(wheels.name, hardware_interface::HW_IF_VELOCITY, &wheel->velocity_command); // NOLINT(*-use-emplace)
        }

        return command_interfaces;
    }

#endif

    CallbackReturn RoverSystemWheelsHardware::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
        logger->info("Activating...");

        if (const auto status = can_controller->initialize(); !status) {
            logger->fatal("Failed to initialize canbus");
            return CallbackReturn::FAILURE;
        }

        // command and state should be equal when starting
        for (const auto& [name, descr] : joint_command_interfaces_) {
            set_command(name, get_state(name));
        }


        using namespace std::chrono_literals;

        // TODO 2026-03-01 (Will Free): is 20ms a correct value for the heartbeat period here?
        constexpr auto HEARTBEAT_PERIOD = 20ms;
#if HARDWARE_INTERFACE_IS_JAZZY
        heartbeat_timer = get_node()->create_wall_timer(HEARTBEAT_PERIOD, std::bind(&RoverSystemWheelsHardware::heartbeat, this));
#else
        heartbeat_running = true;

        heartbeat_thread = std::thread([&] {
            const auto rate = rclcpp::WallRate::make_shared(HEARTBEAT_PERIOD);
            rate->reset();

            while (heartbeat_running&& rclcpp::ok()) {
                heartbeat();
            }
        });
#endif

        logger->info("Successfully activated");

        return CallbackReturn::SUCCESS;
    }

    // TODO 2026-03-01 (Will Free): rolling has stuff like init_hardware_status_message, look at that
    void RoverSystemWheelsHardware::produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat, const WheelDescription::ConstSharedPtr& wheel) {
        using namespace diagnostic_msgs::msg;

        const auto& motor = wheel->motor;

        const auto faults = motor->getFaults();
        const auto stickyFaults = motor->getStickyFaults();

        if (stickyFaults != 0)
            stat.summary(DiagnosticStatus::ERROR, "Motor has sticky fault");
        else if (faults != 0)
            stat.summary(DiagnosticStatus::WARN, "Motor has non-sticky fault");
        else
            stat.summary(DiagnosticStatus::OK, "Motor is OK");

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

#if HARDWARE_INTERFACE_IS_JAZZY
        const auto command = get_command(wheel->velocity_interface_name) * ENCODER_MULTIPLIER;
#else
        const auto command = hw_commands[i] * ENCODER_MULTIPLIER;
#endif

        const auto targetVelocity = metersPerSecondToRPM(command, wheel->radius) * multiplier;

        stat.add("velocity_command", targetVelocity);
    }

    void RoverSystemWheelsHardware::heartbeat() const {
        constexpr auto HEARTBEAT_PERIOD = std::chrono::milliseconds(20);
        const auto rate = rclcpp::WallRate::make_shared(HEARTBEAT_PERIOD);
        rate->reset();

        if (wheels.size() < 0)
            return;

        // AFAIK you can send the same heartbeat command for all the motors?
        // might need to double check that...

        // ReSharper disable once CppExpressionWithoutSideEffects
        wheels[0]->motor->heartbeat();
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

#if HARDWARE_INTERFACE_IS_JAZZY
        try {
            heartbeat_timer->reset();
            heartbeat_timer->cancel();
        } catch (std::runtime_error e) {
            logger->error("Failure to deactivate while stopping heartbeat");
            return CallbackReturn::ERROR;
        }
#else
        heartbeat_running = false;
        if (heartbeat_thread.joinable())
            heartbeat_thread.join();
#endif

        logger->info("Successfully deactivated");

        return result;
    }

    return_type RoverSystemWheelsHardware::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
        for (const auto& wheel : wheels) {
            const auto motor = wheel->motor;
            const auto velocityRPM = static_cast<double>(motor->getVelocity() / ENCODER_MULTIPLIER);
            const auto velocity = rpmToMetersPerSecond(velocityRPM, wheel->radius) / multiplier;

            const auto rotations = static_cast<double>(motor->getPosition() / ENCODER_MULTIPLIER);
            // TODO 2026-03-01 (Will Free): double check if the position is also inverted or not
            const auto position = rotationsToMeters(rotations, wheel->radius) / multiplier;

#if HARDWARE_INTERFACE_IS_JAZZY
            set_state(wheel->velocity_interface_name, velocity);
            set_state(wheel->position_interface_name, position);
#else
            wheel->velocity_state = velocity;
            wheel->position_state = position;
#endif
        }

        return return_type::OK;
    }

    return_type RoverSystemWheelsHardware::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
        auto result = return_type::OK;
        for (const auto& wheel : wheels) {
#if HARDWARE_INTERFACE_IS_JAZZY
            const auto command = get_command(wheel->velocity_interface_name) * ENCODER_MULTIPLIER;
#else
            const auto command = hw_commands[i] * ENCODER_MULTIPLIER;
#endif

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
