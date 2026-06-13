#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "can_util/can_controller.hpp"

template <typename T>
struct RateFormatter : fmt::formatter<double> {
    auto format(const T& rate, fmt::format_context& ctx) const {
        using namespace std::chrono_literals;

        const auto hz = 1.0s / rate.period();
        return fmt::format_to(formatter::format(static_cast<double>(hz), ctx), "Hz");
    }
};

template <>
struct fmt::formatter<rclcpp::Rate> : RateFormatter<rclcpp::Rate> {};

template <>
struct fmt::formatter<rclcpp::WallRate> : RateFormatter<rclcpp::WallRate> {};

namespace {
    // SCRB motor board — host→board IDs from Compat-README.md (29-bit extended, MAKE_ID layout).
    constexpr uint32_t kScrbForceStopId = 0x0001808Cu;
    constexpr uint32_t kScrbResumeId = 0x0001810Cu;
    constexpr std::array kScrbMotorIds = {
        0x0001848Cu, 0x0001850Cu, 0x0001858Cu, 0x0001860Cu, 0x0001868Cu
    };

    /// Protocol full-scale command magnitude (see “Motor velocity scaling” in Compat-README.md).
    constexpr float kMaxRads = 1024.0F;

    // PS4-style layout (matches sil_board/scripts/sil_board_joy_teleop.py).
    constexpr int kAxisLeftStickX = 0;
    constexpr int kAxisLeftStickY = 1;
    constexpr int kAxisLeftTrigger = 2;
    constexpr int kAxisRightStickX = 3;
    constexpr int kAxisRightStickY = 4;
    constexpr int kAxisRightTrigger = 5;

    constexpr int kBtnSquareForceStop = 3;
    constexpr int kBtnTriangleResume = 2;
    constexpr int kBtnLeftBumperDeadman = 4;

    constexpr std::size_t kMinAxes = 6;
    constexpr std::size_t kMinButtons = 8;
}

class CompatBoardNode : public rclcpp::Node {
public:
    explicit CompatBoardNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("compat_board_node", options), logger(this->get_logger()) {
        const auto can_path = this->declare_parameter<std::string>("can_interface", "can0");
        joy_topic = this->declare_parameter<std::string>("joy_topic", "/joy");
        velocity_scale = static_cast<float>(this->declare_parameter<double>("velocity_scale", 1024.0));
        can_send_rate_hz = this->declare_parameter<int>("can_send_rate_hz", 50);
        invert_left_y = this->declare_parameter<bool>("invert_left_y", true);
        invert_right_y = this->declare_parameter<bool>("invert_right_y", true);
        axis_deadzone = static_cast<float>(this->declare_parameter<double>("axis_deadzone", 0.08));

        can_controller = std::make_shared<can_util::CANController>(can_path, this->get_logger());
        if (!can_controller->initialize()) {
            logger.fatal("Failed to initialize canbus");
            throw std::runtime_error("CAN failed to initialize");
        }

        joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
            joy_topic,
            rclcpp::SystemDefaultsQoS(),
            [this](const sensor_msgs::msg::Joy::SharedPtr msg) {
                onJoy(msg);
            }
        );

        const int hz = std::max(1, can_send_rate_hz);
        const auto period = std::chrono::milliseconds(1000 / hz);
        send_timer = this->create_wall_timer(period, [this] {
            onSendTimer();
        });

        logger.info("sil_board_node ready — CAN {}, joy {}, scale {:.1f}, {}", can_path, joy_topic, velocity_scale, hz);
    }

private:
    static float applyDeadzone(const float v, const float dz) {
        const float a = std::fabs(v);
        if (a <= dz) {
            return 0.0F;
        }
        const float s = (a - dz) / (1.0F - dz);
        return std::copysign(s, v);
    }

    void onJoy(const sensor_msgs::msg::Joy::SharedPtr& msg) {
        std::lock_guard lock(joy_mutex);
        latest_joy = std::move(msg);
    }

    bool sendDlc0(const uint32_t raw_id) const {
        return can_controller->sendBlockingFrame(raw_id, {});
    }

    bool sendMotorFloat(const uint32_t raw_id, const float velocity_rads) const {
        const float clamped = std::clamp(velocity_rads, -kMaxRads, kMaxRads);

        std::vector<uint8_t> data(sizeof(clamped), 0);
        std::memcpy(data.data(), &clamped, sizeof(clamped));

        return can_controller->sendBlockingFrame(raw_id, data);
    }

    void onSendTimer() {
        sensor_msgs::msg::Joy::SharedPtr joy_copy;
        {
            std::lock_guard lock(joy_mutex);
            if (!latest_joy) {
                return;
            }
            joy_copy = latest_joy;
        }

        const auto& joy = *joy_copy;
        if (joy.axes.size() < kMinAxes || joy.buttons.size() < kMinButtons) {
            return;
        }

        const bool deadman = joy.buttons[kBtnLeftBumperDeadman] != 0;

        const bool force_stop_now = joy.buttons[kBtnSquareForceStop] != 0;
        const bool resume_now = joy.buttons[kBtnTriangleResume] != 0;

        if (force_stop_now && !prev_force_stop) {
            if (!sendDlc0(kScrbForceStopId)) {
                using namespace std::chrono_literals;

                logger.warn_throttle(1s, "Failed to send SCRB force stop");
            }
        }
        if (resume_now && !prev_resume) {
            if (!sendDlc0(kScrbResumeId)) {
                using namespace std::chrono_literals;

                logger.warn_throttle(1s, "Failed to send SCRB resume");
            }
        }
        prev_force_stop = force_stop_now;
        prev_resume = resume_now;

        auto ax = [&joy](const int i) -> float {
            return i >= 0 && static_cast<std::size_t>(i) < joy.axes.size() ? joy.axes[i] : 0.0F;
        };

        const float lx = applyDeadzone(ax(kAxisLeftStickX), axis_deadzone);
        const float ly = applyDeadzone(ax(kAxisLeftStickY), axis_deadzone) * (invert_left_y ? -1 : 1);
        const float rx = applyDeadzone(ax(kAxisRightStickX), axis_deadzone);
        const float ry = applyDeadzone(ax(kAxisRightStickY), axis_deadzone) * (invert_right_y ? -1 : 1);
        const float lt = applyDeadzone(ax(kAxisLeftTrigger), axis_deadzone);
        const float rt = applyDeadzone(ax(kAxisRightTrigger), axis_deadzone);

        std::array<float, 5> cmd{};
        if (deadman) {
            // One axis per motor: left stick → M1/M2, right stick → M3/M4, triggers → M5.
            cmd[0] = ly * velocity_scale;
            cmd[1] = lx * velocity_scale;
            cmd[2] = ry * velocity_scale;
            cmd[3] = rx * velocity_scale;
            cmd[4] = 0.5F * (lt + rt) * velocity_scale;
        }

        for (std::size_t i = 0; i < kScrbMotorIds.size(); ++i) {
            if (!sendMotorFloat(kScrbMotorIds[i], cmd[i])) {
                using namespace std::chrono_literals;

                logger.warn_throttle(1s, "Failed to send motor {} CAN frame", i + 1);
            }
        }
    }

    ros2_fmt_logger::Logger logger;

    std::string joy_topic;
    float velocity_scale{1024.0F};
    int can_send_rate_hz{50};
    bool invert_left_y{true};
    bool invert_right_y{true};
    float axis_deadzone{0.08F};

    bool prev_force_stop{false};
    bool prev_resume{false};

    std::mutex joy_mutex;
    sensor_msgs::msg::Joy::SharedPtr latest_joy;

    std::shared_ptr<can_util::CANController> can_controller;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
    rclcpp::TimerBase::SharedPtr send_timer;
};

int main(const int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CompatBoardNode>());
    rclcpp::shutdown();
    return 0;
}
