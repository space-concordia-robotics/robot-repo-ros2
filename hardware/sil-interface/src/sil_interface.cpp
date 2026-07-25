#include "sil_interface/sil_interface.hpp"

#include "scrb_common_util/string_parsing.hpp"

#include "sil_interface/hardware_interface_util.hpp"

namespace sil_interface {
    constexpr auto HW_IF_RED = hardware_interface::HW_IF_RED;
    constexpr auto HW_IF_GREEN = hardware_interface::HW_IF_GREEN;
    constexpr auto HW_IF_BLUE = hardware_interface::HW_IF_BLUE;
    constexpr auto HW_IF_BRIGHTNESS = hardware_interface::HW_IF_BRIGHTNESS;

    template <typename I, typename F>
    I denormalize(F value) noexcept {
        static_assert(std::is_integral_v<I>, "I must be an integral type");
        static_assert(std::is_unsigned_v<I>, "I must be an unsigned integral type"); // TODO 2026-05-13 (Will Free): support signed types
        static_assert(std::is_floating_point_v<F>, "T must be floating point type");

        const auto rounded = static_cast<I>(std::numeric_limits<I>::min() + std::lround(std::clamp(value, 0.0, 1.0) * std::numeric_limits<I>::max()));
        return std::clamp(rounded, std::numeric_limits<I>::min(), std::numeric_limits<I>::max());
    }

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

    SILSystemHardware::SILSystemHardware() = default;

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

        const auto& joint = info.joints.at(0);

        if (joint.name != "sil") {
            logger->fatal("Joint '{}' does not have name 'sil'", joint.name, joint.command_interfaces.size());
            return CallbackReturn::ERROR;
        }

        auto hasInterface = [&](const std::vector<InterfaceInfo>& interfaces, const std::string& name) {
            return std::ranges::any_of(
                interfaces,
                [&](const auto& iface) {
                    return iface.name == name;
                }
            );
        };

        if (joint.command_interfaces.size() != 4) {
            logger->fatal("Joint '{}' has {} command interface, 4 expected.", joint.name, joint.command_interfaces.size());
            return CallbackReturn::ERROR;
        }

        static constexpr auto EXPECTED_COMMAND_INTERFACES = std::array{
            HW_IF_RED,
            HW_IF_GREEN,
            HW_IF_BLUE,
            HW_IF_BRIGHTNESS,
        };

        for (const auto& interface : EXPECTED_COMMAND_INTERFACES) {
            if (!hasInterface(joint.command_interfaces, interface)) {
                logger->fatal("Joint '{}' must have '{}' command interface.", joint.name, interface);
                return CallbackReturn::ERROR;
            }
        }

        auto parameters = joint.parameters;
        if (!parameters.contains("device_id")) {
            logger->fatal("Joint '{}' does not have parameter device_id.", joint.name);
            return CallbackReturn::ERROR;
        }

        // TODO 2026-02-14 (Will Free): properly handle errors here

        // TODO 2026-05-13 (Will Free): move away from using the raw canbus id to instead only specifying the device id and constructing it from that.
        device_id = scrb::common_util::parse_uint32(parameters["device_id"], 16);

        // TODO 2026-05-14 (Will Free): add diagnostics for SIL (does it have a heartbeat it returns to us?)
        // const auto diagnosticCallback = std::bind(&RoverSystemWheelsHardware::produce_diagnostics, std::placeholders::_1, std::placeholders::_2, wheel);
        // diagnostic_updater->add(fmt::format("{} Motor {} Status", info.name, i), [&](auto& stat) {
        //     produce_diagnostics(stat, wheel);
        // });

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn SILSystemHardware::on_configure(const rclcpp_lifecycle::State& previous_state) {
        // reset values always when configuring hardware
        for (const auto& name : joint_command_interfaces_ | std::views::keys) {
            set_command(name, 0.0);
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
        const auto red = denormalize<uint8_t>(get_command("sil/red"));
        const auto green = denormalize<uint8_t>(get_command("sil/green"));
        const auto blue = denormalize<uint8_t>(get_command("sil/blue"));
        const auto brightness = denormalize<uint8_t>(get_command("sil/brightness"));

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
