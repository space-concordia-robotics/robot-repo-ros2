#include "can_safety_node/can_safety_node.hpp"

using namespace can_util;

static constexpr auto STOP_COMMAND = 0x01;
static constexpr auto RESUME_COMMAND = 0x02;

static constexpr auto HUB_ID = 0x0E;
static constexpr auto COMPAT_BOARD_ID = 0x06;

namespace can_safety_node {
    CanSafetyNode::CanSafetyNode(const rclcpp::NodeOptions& options) : Node("can_safety_node", options), logger(this->get_logger()) {
        can_interface_name = this->declare_parameter<std::string>("can_interface", can_interface_name);
        wheel_force_stop_button = this->declare_parameter<int>("wheel_force_stop_button", wheel_force_stop_button);
        wheel_resume_button = this->declare_parameter<int>("wheel_resume_button", wheel_resume_button);

        can_controller = std::make_shared<CANController>(can_interface_name, this->get_logger());
        if (!can_controller->initialize()) {
            logger.fatal("Failed to initialize canbus");
            throw std::runtime_error("CAN failed to initialize");
        }

        // Latched QoS so late subscribers see the current state without a tick.
        wheel_stopped_publisher = this->create_publisher<std_msgs::msg::Bool>("can_safety/wheel_stopped", rclcpp::QoS(1).reliable().transient_local());

        joy_subscription = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", rclcpp::SystemDefaultsQoS(),
            [this](const sensor_msgs::msg::Joy::ConstSharedPtr msg) {
                onJoy(msg);
            });

        // Publish the initial (un-stopped) state so /can_safety/wheel_stopped is
        // never empty for downstream consumers.
        std_msgs::msg::Bool msg;
        msg.data = wheel_stopped;
        wheel_stopped_publisher->publish(msg);

        logger.info(
            "can_safety_node ready -- can_interface='{}', force_stop_button={}, resume_button={}",
            can_interface_name, wheel_force_stop_button, wheel_resume_button
        );
    }

    bool CanSafetyNode::risingEdge(const bool current, bool& prev) {
        const bool fired = current && !prev;
        prev = current;
        return fired;
    }

    void CanSafetyNode::onJoy(const sensor_msgs::msg::Joy::ConstSharedPtr& msg) {
        if (const auto needed = static_cast<size_t>(std::max(wheel_force_stop_button, wheel_resume_button)) + 1; msg->buttons.size() < needed) {
            using namespace std::chrono_literals;
            logger.warn_throttle(5s, "Joy message has {} buttons; need at least {} for safety mappings", msg->buttons.size(), needed);
            return;
        }

        const bool fs_pressed = msg->buttons[wheel_force_stop_button] == 1;
        const bool rs_pressed = msg->buttons[wheel_resume_button] == 1;

        const bool fs_edge = risingEdge(fs_pressed, prev_force_stop_pressed);
        const bool rs_edge = risingEdge(rs_pressed, prev_resume_pressed);

        if (fs_edge && !wheel_stopped) {
            wheel_stopped = true;
            sendShutDownRequest(constants::DeviceType::RELAY_CONTROLLER, HUB_ID);
            std_msgs::msg::Bool out;
            out.data = true;
            wheel_stopped_publisher->publish(out);
            RCLCPP_INFO(this->get_logger(), "Wheel force-stop CAN frame sent");
        } else if (rs_edge && wheel_stopped) {
            wheel_stopped = false;
            sendRestartCommand(constants::DeviceType::RELAY_CONTROLLER, HUB_ID);
            std_msgs::msg::Bool out;
            out.data = false;
            wheel_stopped_publisher->publish(out);
            RCLCPP_INFO(this->get_logger(), "Wheel resume CAN frame sent");
        }
    }

    /**
     *  @brief Helper function used to send shutdown request to motors
     *  @param: uint32_t device type, and device ID
     *  @details: Function looks out to see if deviceID is either compat id or hub id.
     *             it then sends the corresponding parameters to the buildCANID function to build the frame
     *              depending on the id it was given.
     *  @return returning frame via a shared_ptr pointing to sendBlockingFrame()
     *
     */
    bool CanSafetyNode::sendShutDownRequest(const constants::DeviceType device_type, const uint32_t device_id) const {
        if (device_id == COMPAT_BOARD_ID) {
            const uint32_t id = createCANFrameId(
                device_type,
                constants::Manufacturer::TEAM_USE,
                constants::Severity::MANUAL_INTERVENTION,
                STOP_COMMAND,
                device_id
            );
            return can_controller->sendBlockingFrame(id, {});
        } else if (device_id == HUB_ID) {
            const uint32_t id = createCANFrameId(
                device_type,
                constants::Manufacturer::TEAM_USE,
                constants::Severity::MANUAL_INTERVENTION,
                STOP_COMMAND,
                device_id
            );
            return can_controller->sendBlockingFrame(id, {});
        } else {
            throw std::invalid_argument(fmt::format("Unknown device id {}", device_id));
        }
    }

    /**
     *  @brief Helper function used to send Restart request to motors
     *  @param: uint32_t device type, and device ID
     *  @details: Function looks out to see if deviceID is either compat id or hub id.
     *             it then sends the corresponding parameters to the buildCANID function to build the frame
     *              depending on the id it was given.
     *  @return returning frame via a shared_ptr pointing to sendBlockingFrame()
     *
     */
    bool CanSafetyNode::sendRestartCommand(const constants::DeviceType device_type, const uint32_t device_id) const {
        if (device_id == COMPAT_BOARD_ID) {
            const uint32_t id = createCANFrameId(
                device_type,
                constants::Manufacturer::TEAM_USE,
                constants::Severity::MANUAL_INTERVENTION,
                RESUME_COMMAND,
                device_id
            );
            return can_controller->sendBlockingFrame(id, {});
        } else if (device_id == HUB_ID) {
            const uint32_t id = createCANFrameId(
                device_type,
                constants::Manufacturer::TEAM_USE,
                constants::Severity::MANUAL_INTERVENTION,
                RESUME_COMMAND,
                device_id
            );
            return can_controller->sendBlockingFrame(id, {});
        } else {
            throw std::invalid_argument(fmt::format("Unknown device id {}", device_id));
        }
    }
}

int main(const int argc, char const* const argv[]) {
    rclcpp::init(argc, argv);
    int exit_code = 0;
    try {
        rclcpp::spin(std::make_shared<can_safety_node::CanSafetyNode>());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("can_safety_node"), "Node failed to start: %s", e.what());
        exit_code = 1;
    } catch (...) {
        RCLCPP_FATAL(rclcpp::get_logger("can_safety_node"), "Node failed to start: unknown exception");
        exit_code = 1;
    }
    rclcpp::shutdown();
    return exit_code;
}
