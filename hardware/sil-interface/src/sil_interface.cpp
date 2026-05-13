#include "sil_interface/sil_interface.hpp"

#include "sil_interface/hardware_interface_util.hpp"

namespace sil_interface {
    /**
     * Create the CAN data for SIL
     * @param r red channel
     * @param g green channel
     * @param b blue channel
     * @param brightness brightness
     * @param blinking if the LED is blinking
     * @param blink_period blink period (in hundredths of a second)
     * @return the CAN data
     */
    // TODO 2026-05-12 (Will Free): convert blinking to a chrono
    static constexpr std::array<uint8_t, 6> createSILCanData(
        const uint8_t r,
        const uint8_t g,
        const uint8_t b,
        const uint8_t brightness,
        const bool blinking = false,
        const uint8_t blink_period = 0
    ) {
        return {
            r,
            g,
            b,
            brightness,
            static_cast<uint8_t>(blinking),
            blink_period
        };
    }

    SILSystemHardware::SILSystemHardware() {}

    CallbackReturn SILSystemHardware::on_init(const hardware_interface::HardwareComponentInterfaceParams& params) {
        const auto& info = params.hardware_info;

        if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
            return CallbackReturn::ERROR;

        auto rcl_logger = get_logger();

        const auto node = get_node();

        // TODO 2026-03-12 (Will Free): re-enable
        diagnostic_updater = std::make_shared<diagnostic_updater::Updater>(node);
        diagnostic_updater->setHardwareID(get_hardware_info().name);

        if (!node) {
            logger->fatal("Could not get/initialize node");
            return CallbackReturn::ERROR;
        }

        logger = std::make_shared<ros2_fmt_logger::Logger>(rcl_logger);

        if (!info.hardware_parameters.contains("can_path")) {
            logger->fatal("Missing parameter can_path.");
            return CallbackReturn::ERROR;
        }

        const auto can_path = info_.hardware_parameters["can_path"];

        can_controller = can_util::CANController::make_shared(can_path, rcl_logger);

        if (info.joints.size() != 1) {
            logger->fatal("Must have exactly one joint");
            return CallbackReturn::ERROR;
        }

        const auto& joint = info.joints[0];

        if (joint.name != "sil") {
            logger->fatal("Joint '{}' does not have name 'sil'", joint.name, joint.command_interfaces.size());
            return CallbackReturn::ERROR;
        }

        if (joint.command_interfaces.size() != 4) {
            logger->fatal("Joint '{}' has {} command interfaces found. 4 expected.", joint.name, joint.command_interfaces.size());
            return CallbackReturn::ERROR;
        }

        // TODO 2026-02-25 (Will Free): look at gazebo's hardware interface to see how it handles different orderings of the interfaces?

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_RED) {
            logger->fatal(
                "Joint '{}' has {} command interface. '{}' expected.",
                joint.name, joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_RED
            );
            return CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[1].name != hardware_interface::HW_IF_GREEN) {
            logger->fatal(
                "Joint '{}' has {} command interface. '{}' expected.",
                joint.name, joint.command_interfaces[1].name.c_str(), hardware_interface::HW_IF_GREEN
            );
            return CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[2].name != hardware_interface::HW_IF_BLUE) {
            logger->fatal(
                "Joint '{}' has {} command interface. '{}' expected.",
                joint.name, joint.command_interfaces[2].name.c_str(), hardware_interface::HW_IF_BLUE
            );
            return CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[3].name != hardware_interface::HW_IF_BRIGHTNESS) {
            logger->fatal(
                "Joint '{}' has {} command interface. '{}' expected.",
                joint.name, joint.command_interfaces[3].name.c_str(), hardware_interface::HW_IF_BRIGHTNESS
            );
            return CallbackReturn::ERROR;
        }

        auto parameters = joint.parameters;
        if (!parameters.contains("device_id")) {
            logger->fatal("Joint '{}' does not have parameter device_id.", joint.name);
            return CallbackReturn::ERROR;
        }

        // TODO 2026-02-14 (Will Free): properly handle errors here

        // TODO 2026-05-13 (Will Free): move away from using the raw canbus id to instead only specifying the device id and constructing it from that.
        device_id = hardware_interface::stoui32(parameters["device_id"]);

        // const auto diagnosticCallback = std::bind(&RoverSystemWheelsHardware::produce_diagnostics, std::placeholders::_1, std::placeholders::_2, wheel);
        // diagnostic_updater->add(fmt::format("{} Motor {} Status", info.name, i), [&](auto& stat) {
        //     produce_diagnostics(stat, wheel);
        // });

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn SILSystemHardware::on_configure(const rclcpp_lifecycle::State& previous_state) {
        // reset values always when configuring hardware
        for (const auto& [name, descr] : joint_command_interfaces_) {
            set_command<uint8_t>(name, 0);
        }

        return SystemInterface::on_configure(previous_state);
    }

    CallbackReturn SILSystemHardware::on_activate(const rclcpp_lifecycle::State& previous_state) {
        logger->info("Activating...");

        if (const auto status = can_controller->initialize(); !status) {
            logger->fatal("Failed to initialize canbus");
            return CallbackReturn::FAILURE;
        }

        logger->info("Successfully activated");

        return SystemInterface::on_activate(previous_state);
    }

    CallbackReturn SILSystemHardware::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
        return SystemInterface::on_deactivate(previous_state);
    }

    return_type SILSystemHardware::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
        return return_type::OK;
    }

    return_type SILSystemHardware::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
        const auto red = get_command<uint8_t>("sil/red");
        const auto green = get_command<uint8_t>("sil/green");
        const auto blue = get_command<uint8_t>("sil/blue");
        const auto brightness = get_command<uint8_t>("sil/brightness");

        if (const auto data = createSILCanData(red, green, blue, brightness); !can_controller->sendBlockingFrame(device_id, data)) {
            using namespace std::chrono_literals;
            logger->warn_throttle(1s, "Failed to send LED CAN frame");
            return return_type::ERROR;
        }

        return return_type::OK;
    }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(sil_interface::SILSystemHardware, hardware_interface::SystemInterface)
